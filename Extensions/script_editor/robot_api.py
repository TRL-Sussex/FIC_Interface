"""
Script Control API — RobotAPI facade and sub-namespaces.

Provides a grouped API (robot.state, robot.motion, robot.mode, robot.process,
robot.target, robot.tune, robot.safety, robot.topics) plus a flat alias
namespace for backward-compatible scripts.

Thread-safety model: node.publish_* and JointStateMixin reads are called
DIRECTLY from the worker thread; all GUI / subprocess ops go through
marshal() to the main thread.
"""
import inspect
import math
import threading
import time

from PyQt5.QtCore import pyqtSignal, QObject, Qt


# ── Marshalling helpers ────────────────────────────────────────────────────────

class MainThreadProxy(QObject):
    """Run callables on the GUI thread (must be created on the main thread)."""

    _invoke = pyqtSignal(object, object)   # (callable, _Result)

    def __init__(self):
        """Initialise the proxy and wire the queued connection."""
        super().__init__()
        self._invoke.connect(self._run, Qt.QueuedConnection)

    def _run(self, fn, result):
        """Execute fn() on the GUI thread and store the result."""
        try:
            result.value = fn()
        except Exception as exc:      # noqa: BLE001 — surfaced to the script
            result.error = exc
        finally:
            result.event.set()


class _Result:
    """Value holder used by make_marshal to ferry return values across threads."""

    def __init__(self):
        """Initialise to empty pending state."""
        self.value = None
        self.error = None
        self.event = threading.Event()


def make_marshal(proxy):
    """
    Return a marshal callable bound to proxy.

    The returned ``marshal(fn, block=True, timeout=10.0)`` posts *fn* to the
    GUI thread, optionally waits for the result, and returns it.  If *fn*
    raises, the exception is re-raised in the caller's thread.  Pass
    ``block=False`` to fire-and-forget (returns None immediately).
    """
    def marshal(fn, block=True, timeout=10.0):
        """Post fn to the GUI thread and optionally block until it completes."""
        res = _Result()
        proxy._invoke.emit(fn, res)
        if not block:
            return None
        res.event.wait(timeout)
        if res.error is not None:
            raise res.error
        return res.value
    return marshal


# ── Tolerance helpers ──────────────────────────────────────────────────────────

def _within_tolerance(current_deg, target_deg, tol_deg):
    """Return True if every joint is within tol_deg of the corresponding target."""
    if not current_deg or len(current_deg) != len(target_deg):
        return False
    return all(abs(c - t) <= tol_deg for c, t in zip(current_deg, target_deg))


# ── Sub-API classes ────────────────────────────────────────────────────────────

class StateAPI:
    """
    Read-only telemetry — joints, EE pose, velocities, efforts, wrench.

    All angle values are returned in **degrees**.  Reads are thread-safe
    (direct JointStateMixin calls; no marshal needed).
    """

    def __init__(self, host, node, rid):
        """Initialise with host, ROS node, and robot-id string."""
        self._host = host
        self._node = node
        self._rid = rid

    @property
    def joints(self):
        """Return current joint positions in degrees (list[float])."""
        rad = self._host.joint_positions_for(self._rid)
        return [math.degrees(r) for r in rad] if rad else []

    @property
    def joint_velocity(self):
        """Return current joint velocities in rad/s (list[float])."""
        return list(self._host.joint_velocity_for(self._rid) or [])

    @property
    def joint_effort(self):
        """Return current joint efforts in Nm (list[float])."""
        return list(self._host.joint_effort_for(self._rid) or [])

    @property
    def joint_commanded(self):
        """Return commanded joint positions in degrees (list[float])."""
        rad = self._host.joint_commanded_for(self._rid) or []
        return [math.degrees(r) for r in rad]

    @property
    def ee_pose(self):
        """Return EE pose as [x, y, z, qx, qy, qz, qw] or [] when unknown."""
        pose = self._host.ee_pose_for(self._rid)
        if pose is None:
            return []
        p, o = pose.position, pose.orientation
        return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    @property
    def wrench(self):
        """Return tool wrench as (Fx, Fy, Fz, Tx, Ty, Tz) tuple, or None."""
        return self._host.wrench_for(self._rid)


class MotionAPI:
    """
    Joint-space and Cartesian motion commands.

    move_joints publishes angles in radians after converting from degrees.
    When wait=True the call blocks at ~20 Hz until all joints reach the
    target within tolerance, the timeout expires, or is_stopped() fires.
    Sim/VR uses node.publish_joint_angles; hardware uses
    node.publish_real_joint_angles.
    """

    _POLL_HZ = 20           # polling rate for blocking wait
    _DEFAULT_TOL_DEG = 2.0  # degrees

    def __init__(self, host, node, rid, target, is_stopped):
        """Initialise with host, ROS node, robot-id, target string, and stop predicate."""
        self._host = host
        self._node = node
        self._rid = rid
        self._target = target    # 'simulation' | 'hardware' | 'vr'
        self._is_stopped = is_stopped

    def set_target(self, target):
        """Update the motion target so routing follows a mid-script switch."""
        self._target = target

    def move_joints(self, angles_deg, wait=False, timeout=10.0, tolerance=None, **kwargs):
        """
        Publish joint angles (degrees) and optionally block until reached.

        Parameters
        ----------
        angles_deg : list[float]
            Target angles in degrees, one per joint.
        wait : bool
            Block until the robot reaches the target (or timeout / stop).
        timeout : float
            Maximum seconds to wait (only used when wait=True).
        tolerance : float | None
            Per-joint tolerance in degrees.  Defaults to 2 degrees.

        Notes
        -----
        The old positional ``velocity`` argument has been removed.  Scripts
        calling ``move_joints(angles, velocity=0.5)`` will now receive a
        deprecation warning; the ``velocity`` value is silently ignored.

        Returns
        -------
        bool | None
            True if reached; False on timeout; None when wait=False.

        """
        if 'velocity' in kwargs:
            import warnings
            warnings.warn(
                'move_joints(): the velocity argument is no longer supported '
                'and has been ignored.  Use wait=True/False to control blocking.',
                DeprecationWarning,
                stacklevel=2,
            )
        tol = tolerance if tolerance is not None else self._DEFAULT_TOL_DEG
        pos_rad = [math.radians(a) for a in angles_deg]
        names = [f'joint_{i+1}' for i in range(len(angles_deg))]
        if self._target == 'hardware':
            self._node.publish_real_joint_angles(self._rid, pos_rad)
        else:
            # simulation and vr both use the JointState publisher
            self._node.publish_joint_angles(pos_rad, names, self._rid)
        if not wait:
            return None
        return self._wait_for_joints(angles_deg, timeout, tol)

    def _wait_for_joints(self, target_deg, timeout, tol_deg):
        """Block until joints reach target or timeout/stop fires."""
        interval = 1.0 / self._POLL_HZ
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._is_stopped():
                return False
            current = self._host.joint_positions_for(self._rid)
            current_deg = [math.degrees(r) for r in current] if current else []
            if _within_tolerance(current_deg, target_deg, tol_deg):
                return True
            time.sleep(interval)
        return False

    def move_cartesian(self, x, y, z, rx=0.0, ry=0.0, rz=0.0):
        """Publish a Cartesian pose command (metres; rx/ry/rz in radians, XYZ Euler)."""
        from geometry_msgs.msg import Pose
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
        self._node.publish_position(pose, self._rid)

    def home(self):
        """Move to the robot home position (read from the robot config, in degrees)."""
        cfg = self._host.current_robot_config()
        home_deg = cfg.get('q_home_deg', [0.0, -45.0, 0.0, -90.0, 0.0, 45.0, 0.0])
        self.move_joints(home_deg, wait=False)

    def set_gripper(self, width_m):
        """Publish gripper width clamped to [0, 0.08] metres."""
        w = float(max(0.0, min(0.08, width_m)))
        self._node.publish_gripper_width(self._rid, w)

    def open_gripper(self):
        """Open the gripper fully (0.08 m)."""
        self.set_gripper(0.08)

    def close_gripper(self):
        """Close the gripper fully (0.0 m)."""
        self.set_gripper(0.0)


class ModeAPI:
    """
    Control-mode publishing for a single robot.

    publish_mode, postural, elbow, hapfic, admittance_planner are all
    thread-safe direct publishes.  publish_mode also triggers a GUI sync via
    marshal so the studio mode-selector reflects the change.
    """

    def __init__(self, host, node, rid, marshal):
        """Initialise with host, ROS node, robot-id, and marshal callable."""
        self._host = host
        self._node = node
        self._rid = rid
        self._marshal = marshal

    def publish_mode(self, mode_int):
        """
        Publish an integer control mode (1-8) and sync the GUI mode selector.

        The publish is direct (thread-safe); the GUI sync goes through marshal.
        """
        self._node.publish_mode(int(mode_int), self._rid)
        rid = self._rid
        mode_name = (
            self._node._MODE_NAMES[int(mode_int) - 1]
            if 1 <= int(mode_int) <= len(self._node._MODE_NAMES)
            else str(mode_int)
        )
        mw = self._host._mw
        self._marshal(
            lambda: mw.update_mode_ui_only(mode_name, rid)
            if hasattr(mw, 'update_mode_ui_only') else None,
            block=False,
        )

    def set(self, mode_int):  # noqa: A003
        """Apply the given integer mode, delegating to publish_mode."""
        self.publish_mode(mode_int)

    def postural(self, enabled):
        """Enable or disable the null-space postural controller."""
        self._node.publish_postural_enable(bool(enabled), self._rid)

    def elbow(self, enabled):
        """Enable or disable the elbow-up constraint."""
        self._node.publish_elbow_enable(bool(enabled), self._rid)

    def hapfic(self, enabled):
        """Enable or disable HapFIC hybrid Cartesian/joint mode."""
        self._node.publish_hapfic_enable(bool(enabled), self._rid)

    def admittance_planner(self, enabled):
        """Enable or disable the admittance planner inside admittance modes."""
        self._node.publish_admittance_planner(bool(enabled), self._rid)

    def desired_wrench(self, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)):
        """Set the Cartesian admittance's desired wrench (N, N m); zero is plain compliance."""
        values = [float(v) for v in tuple(force) + tuple(torque)]
        if len(values) != 6:
            raise ValueError('force and torque take three values each')
        self._node.publish_desired_wrench(values, self._rid)

    def active_profile(self, profile_name):
        """Publish a named parameter profile (transient-local topic)."""
        from std_msgs.msg import String
        self._node.active_profile_publisher_.publish(String(data=str(profile_name)))


class ProcessAPI:
    """
    Start/stop subprocesses: simulation, controller, driver, VR, bags, plotter.

    All methods go through marshal() because they interact with Qt widgets and
    spawn/kill subprocesses, which must happen on the main thread.
    """

    def __init__(self, host, marshal):
        """Initialise with host and marshal callable."""
        self._host = host
        self._marshal = marshal

    def _mw(self):
        """Return the main window."""
        return self._host._mw

    def start_sim(self):
        """Start the MuJoCo simulation."""
        mw = self._mw()
        self._marshal(lambda: mw._start_simulation())

    def stop_sim(self):
        """Stop the MuJoCo simulation."""
        mw = self._mw()
        self._marshal(lambda: mw._stop_simulation())

    def start_fic(self):
        """Start the FIC simulation controller."""
        mw = self._mw()
        self._marshal(lambda: mw._start_fic_simulation())

    def stop_fic(self):
        """Stop the FIC simulation controller."""
        mw = self._mw()
        self._marshal(lambda: mw._stop_fic_simulation())

    def start_controller(self):
        """Start the hardware FIC controller."""
        mw = self._mw()
        self._marshal(lambda: mw._start_controller())

    def stop_controller(self):
        """Stop the hardware FIC controller."""
        mw = self._mw()
        self._marshal(lambda: mw._stop_controller())

    def start_driver(self):
        """Start the hardware driver."""
        mw = self._mw()
        self._marshal(lambda: mw._start_driver())

    def stop_driver(self):
        """Stop the hardware driver."""
        mw = self._mw()
        self._marshal(lambda: mw._stop_driver())

    def start_vr(self):
        """Start the VR relay."""
        mw = self._mw()
        self._marshal(lambda: mw._start_vr())

    def stop_vr(self):
        """Stop the VR relay."""
        mw = self._mw()
        self._marshal(lambda: mw._stop_vr())

    def start_bags(self):
        """Start the ROS bag recorder."""
        mw = self._mw()
        self._marshal(lambda: mw._start_bags())

    def stop_bags(self):
        """Stop the ROS bag recorder."""
        mw = self._mw()
        self._marshal(lambda: mw._stop_bags())

    def start_plotter(self):
        """Start the signal plotter."""
        mw = self._mw()
        self._marshal(lambda: mw._start_plotter())

    def stop_plotter(self):
        """Stop the signal plotter."""
        mw = self._mw()
        self._marshal(lambda: mw._stop_plotter())


class TargetAPI:
    """
    Read and change the sim/hardware/VR target for this robot.

    set_target is marshalled to the GUI thread because it touches Qt widgets.
    The current property is a direct read (no marshal needed).
    """

    def __init__(self, host, rid, marshal):
        """Initialise with host, robot-id, and marshal callable."""
        self._host = host
        self._rid = rid
        self._marshal = marshal

    @property
    def current(self):
        """Return the current target string: 'simulation', 'hardware', or 'vr'."""
        return self._host.robot_target(self._rid)

    def set_target(self, target):
        """Set the sim/hardware/vr target (marshalled to the GUI thread)."""
        rid = self._rid
        host = self._host
        self._marshal(
            lambda: host._mw._set_control_target(target)
            if hasattr(host._mw, '_set_control_target')
            else host.set_robot_target(rid, target)
        )


class TuneAPI:
    """
    Publish parameter updates and save tuning to XML.

    param_update calls node.publish_param_update directly (thread-safe).
    save is marshalled to the main thread because XMLManager writes files and
    may emit UI signals.
    """

    def __init__(self, host, node, rid, marshal):
        """Initialise with host, ROS node, robot-id, and marshal callable."""
        self._host = host
        self._node = node
        self._rid = rid
        self._marshal = marshal

    def param_update(self, param_type, value):
        """Publish a parameter-update string (thread-safe direct publish)."""
        self._node.publish_param_update(self._rid, str(param_type), str(value))

    def save(self, robot_name=None, joint_id=1, mode=1, params=None):
        """
        Save current tuning parameters to XML via XMLManager (marshalled).

        Parameters
        ----------
        robot_name : str | None
            Robot type name.  Defaults to the host current robot name.
        joint_id : int
            Joint index (1-based).
        mode : int
            Control mode number.
        params : dict | None
            Parameter dict passed to XMLManager.save_joint_params.

        """
        rname = robot_name or self._host.current_robot_name()
        host = self._host

        def _do_save():
            xml = host.xml_manager()
            if params is not None and xml is not None and hasattr(xml, 'save_joint_params'):
                from fic_studio.xml_manager import JointParams
                jp = JointParams(**params) if isinstance(params, dict) else params
                xml.save_joint_params(rname, joint_id, mode, jp)

        self._marshal(_do_save)


class SafetyAPI:
    """
    Safety commands: emergency stop and broadcast-stop.

    estop publishes via the direct node path (mode 1) then fires the GUI
    emergency stop handler via marshal for process cleanup.
    """

    def __init__(self, host, node, rid, marshal, is_stopped):
        """Initialise with host, ROS node, robot-id, marshal, and stop predicate."""
        self._host = host
        self._node = node
        self._rid = rid
        self._marshal = marshal
        self._is_stopped = is_stopped

    def estop(self):
        """Publish mode 1 to park the robot and trigger the GUI emergency stop."""
        # Direct publish (thread-safe)
        self._node.publish_mode(1, self._rid)
        host = self._host
        self._marshal(lambda: host.broadcast_stop(), block=False)

    def broadcast_stop(self):
        """Send stop to every active robot (marshalled for safety)."""
        host = self._host
        self._marshal(lambda: host.broadcast_stop(), block=False)

    def pause(self):
        """Publish mode 1 to park the robot (hardware safe state)."""
        self._node.publish_mode(1, self._rid)


def _serialise_msg(msg):
    """Convert a ROS message to a plain dict (legacy get_topic() format)."""
    mt = type(msg).__name__
    if mt in ('Float64MultiArray', 'Float32MultiArray'):
        return {'type': mt, 'data': list(msg.data)}
    if mt == 'JointState':
        d = {'type': mt}
        for f in ('name', 'position', 'velocity', 'effort'):
            v = getattr(msg, f, None)
            if v:
                d[f] = list(v)
        return d
    if mt == 'Pose':
        p, o = msg.position, msg.orientation
        return {'type': mt,
                'position': {'x': p.x, 'y': p.y, 'z': p.z},
                'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w}}
    if mt in ('Wrench', 'WrenchStamped'):
        w = msg.wrench if mt == 'WrenchStamped' else msg
        return {'type': mt,
                'force': {'x': w.force.x, 'y': w.force.y, 'z': w.force.z},
                'torque': {'x': w.torque.x, 'y': w.torque.y, 'z': w.torque.z}}
    if hasattr(msg, 'data'):
        return {'type': mt, 'data': msg.data}
    try:
        return {'type': mt,
                **{s.lstrip('_'): getattr(msg, s) for s in msg.__slots__}}
    except Exception:
        return {'type': mt, 'raw': str(msg)}


class TopicsAPI:
    """
    Generic ROS topic access: subscribe, unsubscribe, read.

    Subscription management happens on the ROS executor thread.  Reads use a
    lock so they are safe to call from the script worker thread.
    """

    def __init__(self, node):
        """Initialise with the ROS node."""
        self._node = node
        self._subs = {}
        self._cache = {}
        self._lock = threading.Lock()

    def _import_msg_class(self, type_str):
        """Import and return a ROS message class from a type string."""
        _REGISTRY = {
            'std_msgs/msg/Float64MultiArray': ('std_msgs.msg', 'Float64MultiArray'),
            'std_msgs/msg/Float32MultiArray': ('std_msgs.msg', 'Float32MultiArray'),
            'std_msgs/msg/Int32': ('std_msgs.msg', 'Int32'),
            'std_msgs/msg/Float32': ('std_msgs.msg', 'Float32'),
            'std_msgs/msg/Float64': ('std_msgs.msg', 'Float64'),
            'std_msgs/msg/String': ('std_msgs.msg', 'String'),
            'std_msgs/msg/Bool': ('std_msgs.msg', 'Bool'),
            'sensor_msgs/msg/JointState': ('sensor_msgs.msg', 'JointState'),
            'geometry_msgs/msg/Pose': ('geometry_msgs.msg', 'Pose'),
            'geometry_msgs/msg/Wrench': ('geometry_msgs.msg', 'Wrench'),
            'geometry_msgs/msg/WrenchStamped': ('geometry_msgs.msg', 'WrenchStamped'),
        }
        entry = _REGISTRY.get(type_str)
        if entry:
            import importlib
            return getattr(importlib.import_module(entry[0]), entry[1])
        parts = type_str.replace('/', '.')
        last_dot = parts.rfind('.')
        if last_dot > 0:
            try:
                import importlib
                return getattr(importlib.import_module(parts[:last_dot]),
                               parts[last_dot + 1:])
            except Exception:
                pass
        return None

    def list_topics(self):
        """Return a list of (topic_name, type_string) tuples from the ROS graph."""
        try:
            return [(n, t[0] if t else 'unknown')
                    for n, t in self._node.get_topic_names_and_types()]
        except Exception:
            return []

    def subscribe(self, topic_name):
        """Subscribe to a topic by name, auto-discovering the message type."""
        if topic_name in self._subs:
            return
        type_str = next(
            (types[0] for name, types
             in self._node.get_topic_names_and_types()
             if name == topic_name and types),
            None)
        if type_str is None:
            return
        msg_cls = self._import_msg_class(type_str)
        if msg_cls is None:
            return

        def _cb(msg, _t=topic_name):
            with self._lock:
                self._cache[_t] = _serialise_msg(msg)

        sub = self._node.create_subscription(msg_cls, topic_name, _cb, 10)
        self._subs[topic_name] = sub

    def unsubscribe(self, topic_name):
        """Destroy the subscription for topic_name and clear its cached value."""
        sub = self._subs.pop(topic_name, None)
        if sub is not None:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        with self._lock:
            self._cache.pop(topic_name, None)

    def get(self, topic_name):
        """Return the latest message for topic_name, or None."""
        with self._lock:
            return self._cache.get(topic_name)

    def cleanup(self):
        """Destroy all subscriptions (call at script teardown)."""
        for sub in list(self._subs.values()):
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._subs.clear()
        with self._lock:
            self._cache.clear()


# ── Top-level facade ───────────────────────────────────────────────────────────

class RobotAPI:
    """
    Grouped robot control API.

    Parameters
    ----------
    host : ExtensionHost
        The extension host (joint_positions_for, ee_pose_for, etc.).
    node : FicStudioNode
        The ROS node (owns all publisher helpers).
    rid : str
        Robot-id string, e.g. 'robot1'.
    target : str
        Initial target: 'simulation', 'hardware', or 'vr'.
    marshal : callable
        marshal(fn, block=True, timeout=10.0) — runs fn on the GUI thread.
    is_stopped : callable
        Zero-argument predicate; returns True when the script should abort.

    """

    def __init__(self, host, node, rid, target, marshal, is_stopped):
        """Initialise all sub-APIs from the given parameters."""
        self._host = host
        self._node = node
        self._rid = rid
        self._target = target
        self._marshal = marshal
        self._is_stopped = is_stopped

        self.state = StateAPI(host, node, rid)
        self.motion = MotionAPI(host, node, rid, target, is_stopped)
        self.mode = ModeAPI(host, node, rid, marshal)
        self.process = ProcessAPI(host, marshal)
        self.target = TargetAPI(host, rid, marshal)
        self.tune = TuneAPI(host, node, rid, marshal)
        self.safety = SafetyAPI(host, node, rid, marshal, is_stopped)
        self.topics = TopicsAPI(node)

    @property
    def rid(self):
        """Return the robot-id string (e.g. 'robot1')."""
        return self._rid

    @property
    def target_mode(self):
        """Return the current target string: 'simulation', 'hardware', or 'vr'."""
        return self._target

    def wait(self, seconds):
        """Sleep for seconds, honouring the cooperative stop event at ~20 Hz."""
        interval = 0.05
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._is_stopped():
                return
            time.sleep(interval)

    def is_stopped(self):
        """Return True if a cooperative stop has been requested."""
        return self._is_stopped()


# ── Namespace builder ──────────────────────────────────────────────────────────

def build_namespace(api):
    """
    Build the exec() namespace for a script given a RobotAPI instance.

    Returns a dict containing:

    - ``robot``: the RobotAPI instance (grouped namespaces)
    - flat aliases for all public motion / mode / state / topics methods
    - ``math``, ``time``, ``wait``, ``sleep``, ``is_stopped``

    Flat aliases provide backward compatibility with scripts that call
    move_joints(…) directly rather than robot.motion.move_joints(…).
    """
    ns = {
        'robot': api,
        'math': math,
        'time': time,
        'wait': api.wait,
        'sleep': time.sleep,
        'is_stopped': api.is_stopped,

        # ── motion aliases ──
        'move_joints': api.motion.move_joints,
        'move_cartesian': api.motion.move_cartesian,
        'home': api.motion.home,
        'set_gripper': api.motion.set_gripper,
        'open_gripper': api.motion.open_gripper,
        'close_gripper': api.motion.close_gripper,

        # ── state aliases ──
        'get_joint_positions': lambda: api.state.joints,
        'get_cartesian_pose': lambda: api.state.ee_pose,
        'get_ee_position': lambda: api.state.ee_pose[:3] if api.state.ee_pose else None,
        'get_force_torque': lambda: api.state.wrench,
        'get_joint_velocity': lambda: api.state.joint_velocity,
        'get_joint_effort': lambda: api.state.joint_effort,

        # ── mode aliases ──
        'set_mode': api.mode.publish_mode,

        # ── topics aliases ──
        'list_topics': api.topics.list_topics,
        'subscribe': api.topics.subscribe,
        'unsubscribe': api.topics.unsubscribe,
        'get_topic': api.topics.get,

        # ── utility ──
        'get_target_mode': lambda: api.target_mode,
        'is_moving': lambda: False,
        'wait_until_stopped': lambda timeout=10.0: api.wait(timeout),
        'stop': lambda: api.safety.estop(),
    }
    return ns


# ── API reference builder ──────────────────────────────────────────────────────

def api_reference(api):
    """
    Return a list of dicts describing every public method of the RobotAPI.

    Each dict has keys:

    - ``name``: dotted name (e.g. 'robot.motion.move_joints')
    - ``signature``: inspect.signature string
    - ``doc``: first line of the docstring (or empty string)

    Used by the searchable API reference panel in the editor UI.
    """
    entries = []

    def _entry(name, obj, is_property=False):
        if is_property:
            sig = ''                      # read as robot.state.joints, not called
        else:
            try:
                sig = str(inspect.signature(obj))
            except (ValueError, TypeError):
                sig = '(...)'
        doc = (inspect.getdoc(obj) or '').strip().splitlines()
        first_line = doc[0] if doc else ''
        entries.append({'name': name, 'signature': sig, 'doc': first_line})

    # Sub-APIs and their public methods
    sub_apis = [
        ('robot.state', api.state),
        ('robot.motion', api.motion),
        ('robot.mode', api.mode),
        ('robot.process', api.process),
        ('robot.target', api.target),
        ('robot.tune', api.tune),
        ('robot.safety', api.safety),
        ('robot.topics', api.topics),
    ]
    for prefix, sub in sub_apis:
        # Properties
        for attr in dir(type(sub)):
            if attr.startswith('_'):
                continue
            prop = getattr(type(sub), attr, None)
            if isinstance(prop, property):
                _entry(f'{prefix}.{attr}', prop.fget, is_property=True)
        # Regular methods
        for attr in dir(sub):
            if attr.startswith('_'):
                continue
            obj = getattr(sub, attr, None)
            if callable(obj) and not isinstance(
                    getattr(type(sub), attr, None), property):
                _entry(f'{prefix}.{attr}', obj)

    # Top-level robot helpers
    for attr in ('wait', 'is_stopped'):
        obj = getattr(api, attr, None)
        if callable(obj):
            _entry(f'robot.{attr}', obj)

    # Flat aliases
    ns = build_namespace(api)
    skip = {'robot', 'math', 'time'}
    for name, obj in sorted(ns.items()):
        if name in skip or name.startswith('_'):
            continue
        if callable(obj):
            _entry(name, obj)

    return entries
