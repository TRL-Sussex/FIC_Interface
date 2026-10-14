# fic_api/robot.py — Standalone Robot API for FIC-controlled robots
#
# Usage:
#   from fic_api import Robot
#   with Robot(target='simulation', num_joints=7) as r:
#       r.home()
#       print(r.get_current_joints())

import math
import time
import threading
import importlib
import json
import os
import socket
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Float32MultiArray, Float32, Float64MultiArray, Int32, String
from geometry_msgs.msg import Pose, Wrench
from sensor_msgs.msg import JointState

from .trajectory import parse_trajectory_data


# ─────────────────────────────────────────────────────────────────────────────
#  Topic configuration per target
# ─────────────────────────────────────────────────────────────────────────────

def _topic_config(target: str, rid: str = 'robot1') -> dict:
    configs = {
        'simulation': {
            'desired_joints':   f'{rid}_desired_joint_angles',
            'target_pose':      f'command_ee_position_change_{rid}',
            'gripper':          f'{rid}_gripper_width',
            'control_mode':     f'{rid}_control_mode',
            'control_mode_type':'int',
            'joint_states':     f'/{rid}_simulation/joint_states',
            'ee_pose':          f'{rid}_ee_current_pose',
        },
        'hardware': {
            'desired_joints':   f'{rid}_desired_joint_angles',
            'target_pose':      f'command_ee_position_change_{rid}',
            'gripper':          f'{rid}_gripper_width',
            'control_mode':     f'{rid}_mode_select',
            'control_mode_type':'string',
            'joint_states':     '/joint_states',
            'ee_pose':          f'{rid}_ee_current_pose',
        },
        'vr': {
            'desired_joints':   f'{rid}_desired_joint_angles',
            'target_pose':      f'command_ee_position_change_{rid}',
            'gripper':          f'{rid}_gripper_width',
            'control_mode':     f'{rid}_control_mode',
            'control_mode_type':'int',
            'joint_states':     '/vr/joint_states',
            'ee_pose':          f'{rid}_ee_current_pose',
        },
    }
    return configs[target]

_MODE_NAMES = {
    1: 'Joint Impedance',
    2: 'Joint Impedance With Planner',
    3: 'Joint Admittance',
    4: 'Joint Admittance Haptic',
    5: 'Cartesian Impedance',
    6: 'Cartesian Planner',
    7: 'Cartesian Admittance',
    8: 'Cartesian Admittance Haptic',
}


# ─────────────────────────────────────────────────────────────────────────────
#  Internal ROS2 node
# ─────────────────────────────────────────────────────────────────────────────

#: Where API clients announce themselves (read by FIC Studio's Multi-Robot page).
CLIENTS_TOPIC = '/fic/fleet/clients'
CLIENT_ANNOUNCE_S = 2.0


def _station_of(rid):
    try:
        return int(str(rid).replace('robot', ''))
    except ValueError:
        return None


class _RobotNode(Node):
    """Lightweight ROS2 node that handles all pub/sub for the Robot API."""

    def __init__(self, target: str, num_joints: int, node_name: str, rid: str = 'robot1',
                 state: str = 'full'):
        super().__init__(node_name)
        self._num_joints = num_joints
        self._target = target
        self._rid = rid
        topics = _topic_config(target, rid)

        # ── State (updated by callbacks) ──
        self._lock = threading.Lock()
        self._traj_parsed = None
        self._joint_positions = [0.0] * num_joints   # radians
        self._joint_velocities = [0.0] * num_joints
        self._ee_pose = [0.0] * 6
        self._force_torque = None   # Wrench dict
        self._torque_cmds = None    # list[float]
        self._generic_cache = {}    # topic -> parsed dict
        self._generic_subs = {}     # topic -> subscription

        # ── Publishers ──
        self.joint_pub = self.create_publisher(
            Float32MultiArray, topics['desired_joints'], 10)
        self.pose_pub = self.create_publisher(
            Pose, topics['target_pose'], 10)
        self.gripper_pub = self.create_publisher(
            Float32, topics['gripper'], 10)
        if topics['control_mode_type'] == 'int':
            self.mode_pub = self.create_publisher(
                Int32, topics['control_mode'], 10)
        else:
            self.mode_pub = self.create_publisher(
                String, topics['control_mode'], 10)

        # ── Subscribers ──
        if state == 'lite':
            # The robot's Studio republishes its state at 50 Hz (fic_studio.fleet.lite):
            # what a script on another machine should read, instead of the 1 kHz topics.
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(
                JointState, f'/{rid}_lite/joint_states',
                self._on_joint_states, qos_profile_sensor_data)
            self.create_subscription(
                PoseStamped, f'/{rid}_lite/ee_pose',
                lambda m: self._on_ee_pose(m.pose), qos_profile_sensor_data)
        else:
            self.create_subscription(
                Float64MultiArray, f'/{rid}_trajectory_data',
                self._on_trajectory, 10)
            self.create_subscription(
                JointState, topics['joint_states'],
                self._on_joint_states, 10)
            self.create_subscription(
                Pose, topics['ee_pose'],
                self._on_ee_pose, 10)

        # ── Say who we are, so every Studio's Multi-Robot page can show that
        #    this script commands this robot ("Commanded by"). ~100 bytes / 2 s.
        self._client_pub = self.create_publisher(String, CLIENTS_TOPIC, 10)
        self._client_card = json.dumps({
            'client_id': '%s:%d' % (socket.gethostname(), os.getpid()),
            'host': socket.gethostname().split('.')[0].lower().replace('-', '_'),
            'script': os.path.basename(sys.argv[0]) if sys.argv and sys.argv[0] else 'python',
            'node': node_name,
            'station': _station_of(rid),
            'target': target,
        })
        self.create_timer(CLIENT_ANNOUNCE_S, self._announce)
        self._announce()

    def _announce(self):
        self._client_pub.publish(String(data=self._client_card))

    # ── Callbacks ──

    def _on_trajectory(self, msg):
        raw = list(msg.data)
        parsed = parse_trajectory_data(raw, self._num_joints)
        with self._lock:
            self._traj_parsed = parsed
            if parsed and parsed.get('q_current'):
                self._joint_positions = list(parsed['q_current'])

    def _on_joint_states(self, msg):
        with self._lock:
            if msg.position and len(msg.position) >= self._num_joints:
                self._joint_positions = list(msg.position[:self._num_joints])
            if msg.velocity and len(msg.velocity) >= self._num_joints:
                self._joint_velocities = list(msg.velocity[:self._num_joints])

    def _on_ee_pose(self, msg):
        with self._lock:
            self._ee_pose = [
                msg.position.x, msg.position.y, msg.position.z,
                msg.orientation.x, msg.orientation.y, msg.orientation.z,
            ]

    # ── Generic topic helpers ──

    def subscribe_generic(self, topic_name: str):
        if topic_name in self._generic_subs:
            return
        # Discover type
        names_and_types = self.get_topic_names_and_types()
        type_str = None
        for name, types in names_and_types:
            if name == topic_name:
                type_str = types[0] if types else None
                break
        if type_str is None:
            self.get_logger().warn(f"Topic '{topic_name}' not on ROS graph")
            return
        # Import message class
        msg_cls = self._import_msg(type_str)
        if msg_cls is None:
            self.get_logger().warn(f"Cannot import '{type_str}'")
            return
        def _cb(msg, _t=topic_name):
            from fic_api.robot import _parse_generic
            with self._lock:
                self._generic_cache[_t] = _parse_generic(msg)
        sub = self.create_subscription(msg_cls, topic_name, _cb, 10)
        self._generic_subs[topic_name] = sub
        self.get_logger().info(f"Subscribed to {topic_name} [{type_str}]")

    def unsubscribe_generic(self, topic_name: str):
        sub = self._generic_subs.pop(topic_name, None)
        if sub:
            self.destroy_subscription(sub)
            with self._lock:
                self._generic_cache.pop(topic_name, None)

    def cleanup_generic(self):
        for sub in self._generic_subs.values():
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
        self._generic_subs.clear()
        with self._lock:
            self._generic_cache.clear()

    @staticmethod
    def _import_msg(type_str: str):
        parts = type_str.split('/')
        if len(parts) >= 3:
            mod_name = f"{parts[0]}.{parts[1]}"
            cls_name = parts[2]
            try:
                mod = importlib.import_module(mod_name)
                return getattr(mod, cls_name)
            except Exception:
                pass
        return None


def _parse_generic(msg) -> dict:
    """Convert any ROS2 message to a plain dict."""
    t = type(msg).__name__
    if t in ('Float64MultiArray', 'Float32MultiArray'):
        return {'type': t, 'data': list(msg.data)}
    if t == 'JointState':
        d = {'type': 'JointState'}
        if msg.name:     d['name']     = list(msg.name)
        if msg.position: d['position'] = list(msg.position)
        if msg.velocity: d['velocity'] = list(msg.velocity)
        if msg.effort:   d['effort']   = list(msg.effort)
        return d
    if t == 'Pose':
        return {
            'type': 'Pose',
            'position': {'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z},
            'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y,
                            'z': msg.orientation.z, 'w': msg.orientation.w},
        }
    if t in ('Wrench', 'WrenchStamped'):
        w = msg if t == 'Wrench' else msg.wrench
        return {
            'type': t,
            'force':  {'x': w.force.x,  'y': w.force.y,  'z': w.force.z},
            'torque': {'x': w.torque.x, 'y': w.torque.y, 'z': w.torque.z},
        }
    if hasattr(msg, 'data'):
        return {'type': t, 'data': msg.data}
    return {'type': t, 'raw': str(msg)}


# ─────────────────────────────────────────────────────────────────────────────
#  Public Robot class
# ─────────────────────────────────────────────────────────────────────────────

class Robot:
    """Standalone Python API for FIC-controlled robots.

    Parameters
    ----------
    target : str
        'simulation' or 'hardware'
    num_joints : int
        Number of joints (default 7 for Franka FR3)
    node_name : str
        ROS2 node name (default 'fic_api_api')
    home_angles_deg : list[float] | None
        Home position in degrees; defaults to Franka home
    robot_id : int | str
        Which robot: its station number (robot 2 -> 2) or 'robot2'
    state : str
        'full' (default): read the robot's 1 kHz state topics — right on the
        robot's own PC. 'lite': read the 50 Hz feed its Studio republishes —
        use this from any other machine (a laptop on the lab network).

    Examples
    --------
    >>> from fic_api import Robot
    >>> r = Robot()
    >>> r.home()
    >>> print(r.get_current_joints())
    >>> r.close()

    Or as a context manager:
    >>> with Robot(target='hardware') as r:
    ...     r.move_joints([0, -30, 0, -90, 0, 60, 0])
    """

    def __init__(
        self,
        target: str = 'simulation',
        num_joints: int = 7,
        node_name: str = 'fic_api_api',
        home_angles_deg: list | None = None,
        robot_id: int | str = 1,
        state: str = 'full',
    ):
        if target not in ('simulation', 'hardware', 'vr'):
            raise ValueError(f"target must be 'simulation', 'hardware', or 'vr', got '{target}'")
        if state not in ('full', 'lite'):
            raise ValueError(f"state must be 'full' or 'lite', got '{state}'")

        self._target = target
        self._num_joints = num_joints
        self._home_deg = home_angles_deg or [0, -45, 0, -90, 0, 45, 0]
        self._closed = False
        rid = f'robot{robot_id}' if isinstance(robot_id, int) else str(robot_id)

        # Init ROS2 if needed
        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        # Create node + executor
        self._node = _RobotNode(target, num_joints, node_name, rid=rid, state=state)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # Spin in background thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        # Wait for first trajectory data
        time.sleep(0.3)

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:
            pass

    # ── Context manager ──

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def close(self):
        """Shutdown the ROS2 node and background thread."""
        if self._closed:
            return
        self._closed = True
        self._node.cleanup_generic()
        self._executor.shutdown()
        self._node.destroy_node()
        if self._owns_rclpy:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ═════════════════════════════════════════════════════════════════════════
    #  TRAJECTORY DATA READERS  (values in radians / metres from FIC)
    # ═════════════════════════════════════════════════════════════════════════

    def get_trajectory(self) -> dict:
        """Full parsed /trajectory_data dict."""
        with self._node._lock:
            return self._node._traj_parsed or {}

    def _traj(self, key):
        with self._node._lock:
            p = self._node._traj_parsed
            return p.get(key) if p else None

    def get_current_joints(self) -> list:
        """[rad] current joint positions: from the FIC controller, or the light feed."""
        q = self._traj('q_current')
        if q:
            return q
        with self._node._lock:           # state='lite': joint_states from the 50 Hz feed
            return list(self._node._joint_positions)

    def get_target_joints(self) -> list:
        """[rad] joint targets."""
        return self._traj('q_target') or [0.0] * self._num_joints

    def get_joint_errors(self) -> list:
        """[rad] position error per joint."""
        return self._traj('error') or [0.0] * self._num_joints

    def get_control_effort(self) -> list:
        """[Nm] torque command per joint (joint modes)."""
        return self._traj('control_effort') or [0.0] * self._num_joints

    def get_planner_targets(self) -> list:
        """[rad] planner output (joint modes)."""
        return self._traj('planner_target') or [0.0] * self._num_joints

    def get_admittance_targets(self) -> list:
        """[rad] admittance target (modes 3-4)."""
        return self._traj('q_target_adm') or [0.0] * self._num_joints

    def get_desired_effort(self) -> list:
        """[Nm] desired/gravity effort (modes 3-4)."""
        return self._traj('desired_effort') or [0.0] * self._num_joints

    def get_measured_effort(self) -> list:
        """[Nm] measured torque (modes 3-4)."""
        return self._traj('measured_effort') or [0.0] * self._num_joints

    def get_ee_position(self) -> list | None:
        """[x,y,z] metres (Cartesian modes) or None."""
        return self._traj('ee_position')

    def get_ee_desired(self) -> list | None:
        """[x,y,z] metres (Cartesian modes) or None."""
        return self._traj('ee_desired')

    def get_ee_error(self) -> list | None:
        """[dx,dy,dz] metres (Cartesian modes) or None."""
        return self._traj('ee_error')

    def get_ee_admittance_target(self) -> list | None:
        """[x,y,z] metres (Cartesian modes) or None."""
        return self._traj('ee_admittance_target')

    def get_rotation_error(self) -> list | None:
        """[rx,ry,rz, angle_rad] (Cartesian modes) or None."""
        axis = self._traj('rotation_error')
        if axis is None:
            return None
        angle = self._traj('rotation_angle') or 0.0
        return axis + [angle]

    # ═════════════════════════════════════════════════════════════════════════
    #  MOVEMENT COMMANDS
    # ═════════════════════════════════════════════════════════════════════════

    def move_joints(self, angles_deg: list, velocity: float = 0.5):
        """Move to joint angles in degrees.

        Parameters
        ----------
        angles_deg : list[float]
            Target angles in degrees, one per joint.
        velocity : float
            Velocity scale 0.0-1.0 (informational, FIC uses its own planner).
        """
        if len(angles_deg) != self._num_joints:
            raise ValueError(
                f"Expected {self._num_joints} angles, got {len(angles_deg)}")
        msg = Float32MultiArray()
        msg.data = [math.radians(a) for a in angles_deg]
        self._node.joint_pub.publish(msg)
        time.sleep(0.05)

    def move_cartesian(self, x: float, y: float, z: float,
                       rx: float = 0, ry: float = 0, rz: float = 0):
        """Move end-effector to Cartesian position.

        Parameters
        ----------
        x, y, z : float  — position in metres
        rx, ry, rz : float — orientation as Euler XYZ in radians
        """
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        try:
            from scipy.spatial.transform import Rotation
            q = Rotation.from_euler('xyz', [rx, ry, rz]).as_quat()
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
        except ImportError:
            pose.orientation.w = 1.0
        self._node.pose_pub.publish(pose)
        time.sleep(0.05)

    def home(self):
        """Move to home position."""
        self.move_joints(self._home_deg, velocity=0.3)

    def stop(self):
        """Emergency stop — hold current position."""
        q = self.get_current_joints()
        if q:
            msg = Float32MultiArray()
            msg.data = [float(v) for v in q]
            self._node.joint_pub.publish(msg)

    # ═════════════════════════════════════════════════════════════════════════
    #  STATUS  (GUI-compatible, degrees)
    # ═════════════════════════════════════════════════════════════════════════

    def get_joint_positions(self) -> list:
        """Current joint positions in **degrees** (convenience wrapper)."""
        with self._node._lock:
            return [math.degrees(r) for r in self._node._joint_positions]

    def get_cartesian_pose(self) -> list:
        """[x, y, z, qx, qy, qz] from /ee_current_pose."""
        with self._node._lock:
            return list(self._node._ee_pose)

    def get_joint_velocities(self) -> list:
        """[rad/s] from JointState.velocity."""
        with self._node._lock:
            return list(self._node._joint_velocities)

    def is_moving(self, threshold: float = 0.01) -> bool:
        """True if any joint velocity exceeds *threshold* rad/s."""
        vels = self.get_joint_velocities()
        return any(abs(v) > threshold for v in vels)

    def wait_until_stopped(self, timeout: float = 10.0, threshold: float = 0.01):
        """Block until the robot stops or *timeout* seconds elapse."""
        t0 = time.time()
        time.sleep(0.3)
        while time.time() - t0 < timeout:
            if not self.is_moving(threshold):
                return True
            time.sleep(0.05)
        return False

    # ═════════════════════════════════════════════════════════════════════════
    #  GRIPPER
    # ═════════════════════════════════════════════════════════════════════════

    def set_gripper(self, width: float):
        """Set gripper width in metres (0.0 – 0.08)."""
        msg = Float32()
        msg.data = max(0.0, min(0.08, float(width)))
        self._node.gripper_pub.publish(msg)

    def open_gripper(self):
        self.set_gripper(0.08)

    def close_gripper(self):
        self.set_gripper(0.0)

    # ═════════════════════════════════════════════════════════════════════════
    #  CONTROL MODE
    # ═════════════════════════════════════════════════════════════════════════

    def set_mode(self, mode: int):
        """Set FIC control mode (1-8)."""
        topics = _topic_config(self._target, self._node._rid)
        if topics['control_mode_type'] == 'int':
            msg = Int32()
            msg.data = int(mode)
        else:
            msg = String()
            msg.data = _MODE_NAMES.get(int(mode), f'Mode{mode}')
        self._node.mode_pub.publish(msg)

    # ═════════════════════════════════════════════════════════════════════════
    #  GENERIC TOPIC SUBSCRIPTION
    # ═════════════════════════════════════════════════════════════════════════

    def list_topics(self) -> list:
        """Return [(topic_name, type_str), ...] from the ROS2 graph."""
        try:
            return [(n, t[0] if t else 'unknown')
                    for n, t in self._node.get_topic_names_and_types()]
        except Exception:
            return []

    def subscribe(self, topic_name: str):
        """Subscribe to any topic (type auto-detected)."""
        self._node.subscribe_generic(topic_name)
        time.sleep(0.15)

    def unsubscribe(self, topic_name: str):
        """Remove a subscription."""
        self._node.unsubscribe_generic(topic_name)

    def get_topic(self, topic_name: str) -> dict | None:
        """Latest message from a subscribed topic as a dict."""
        with self._node._lock:
            return self._node._generic_cache.get(topic_name)

    def get_force_torque(self) -> dict | None:
        """Latest simulation force_torque as a Wrench dict."""
        rid = self._node._rid
        t = f'/{rid}_simulation/force_torque'
        if t not in self._node._generic_subs:
            self.subscribe(t)
        with self._node._lock:
            return self._node._generic_cache.get(t)

    def get_torque_commands(self) -> dict | None:
        """Latest fic_torque_commands."""
        rid = self._node._rid
        t = f'/{rid}_fic_torque_commands'
        if t not in self._node._generic_subs:
            self.subscribe(t)
        with self._node._lock:
            return self._node._generic_cache.get(t)

    # ═════════════════════════════════════════════════════════════════════════
    #  INFO
    # ═════════════════════════════════════════════════════════════════════════

    @property
    def target(self) -> str:
        return self._target

    @property
    def num_joints(self) -> int:
        return self._num_joints

    def __repr__(self):
        st = 'closed' if self._closed else 'active'
        return f"Robot(target='{self._target}', joints={self._num_joints}, {st})"
