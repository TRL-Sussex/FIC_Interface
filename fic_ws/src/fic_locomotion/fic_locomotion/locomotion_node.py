"""
ROS 2 wrapper around LocomotionController.

Subscribes to simulator feedback and operator commands and publishes twelve
joint position targets on the FIC controller's existing joint-target topic. The
FIC node needs no changes at all: in joint-impedance mode it already tracks
whatever appears on `robot{N}_target_joint_angles`, which is exactly the
contract its planner modes already use.

Note the topic deliberately avoided: `robot{N}_desired_joint_angles` truncates
to seven joints in the C++ callback and would move only part of the robot.

Where *measured* state comes from is selected by the `source_prefix`
parameter, following the convention `fic_control.cpp` set for the C++
node. The default, `'simulation'`, resolves to exactly the topics this node
used to hardcode. `'hardware'` resolves the joint vector to the
`{rid}_q_current` `Float64MultiArray` that `fic_control` really publishes --
and resolves the base state to nothing, because nothing in this repository
estimates a floating base on hardware. In that configuration the node refuses
to publish any target at all. See `docs/spot-locomotion-hardware.md`; that
refusal is the correct behaviour and must not be "fixed" by feeding the gait a
synthesised trunk pose.
"""

import math
import time
from typing import List, NamedTuple, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32, String

from fic_locomotion import leg_kinematics as lk
from fic_locomotion.controller import (
    ControllerConfig, LocomotionController, RobotState, VelocityCommand)
from fic_locomotion.posture import PostureGains
from fic_locomotion.trot_scheduler import GaitParams

# Joint-impedance mode: FIC tracks joint targets with fractal impedance.
JOINT_IMPEDANCE_MODE = 1

DEFAULT_RATE_HZ = 200.0

# Where measured state comes from. `source_prefix` follows the convention
# `fic_control.cpp` established for the C++ node -- one parameter naming
# the state source -- so the gait node is not structurally simulation-only.
#
#   'simulation' (default)  /{rid}_simulation/joint_states + .../odom
#   'vr' (or any other tag) /{rid}_{tag}/joint_states      + .../odom
#   'hardware'              {rid}_q_current, and NO base state (see below)
DEFAULT_SOURCE_PREFIX = 'simulation'
HARDWARE_SOURCE = 'hardware'

# How the joint vector arrives.
#   NAMED    sensor_msgs/JointState, matched by name (the simulator).
#   ORDERED  std_msgs/Float64MultiArray, positional, in robots_config.yaml
#            joint order (what `fic_control` publishes on {rid}_q_current;
#            it publishes no JointState at all).
NAMED_JOINTS = 'named'
ORDERED_JOINTS = 'ordered'


class StateTopics(NamedTuple):
    """Which topics carry measured state, and in what form."""

    joints: str
    form: str
    base: str


def resolve_state_topics(rid: str, source_prefix: str,
                         base_state_topic: str = '') -> StateTopics:
    """
    Resolve the feedback topics for one state source.

    `base` is the empty string when the selected source has no base-state
    publisher. That is not an oversight to be papered over: the gait consumes
    trunk height, roll, pitch and trunk-frame velocity, and on the hardware
    path nothing in this repository produces any of them. Synthesising them
    would let a kinematic trot run open-loop on a 50 kg machine, so the node
    refuses to command instead. `base_state_topic` is the seam for a real
    state estimator once one exists.
    """
    prefix = (source_prefix or DEFAULT_SOURCE_PREFIX).strip().strip('/')
    override = (base_state_topic or '').strip()
    if prefix == HARDWARE_SOURCE:
        # No namespace: `fic_control` publishes `{rid}_q_current` relative to
        # its own (default) node namespace, exactly as this node publishes
        # `{rid}_target_joint_angles`.
        return StateTopics('%s_q_current' % rid, ORDERED_JOINTS, override)
    namespace = '/%s_%s/' % (rid, prefix)
    return StateTopics(namespace + 'joint_states', NAMED_JOINTS,
                       override or namespace + 'odom')


# Inclusive (min, max) ranges for every numeric knob that can arrive from
# outside the node -- a launch parameter or a `set <key> <value>` message.
# These are not style: `cycle_time = 0` divides by zero in
# `trot_scheduler.advance` and `duty_factor = 1` in `trot_scheduler.leg_phase`,
# both inside the timer callback, and an exception there propagates out of
# `rclpy.spin` and terminates a node holding a quadruped up on twelve torque
# actuators. Bad values are refused with a warning instead.
SETTING_LIMITS = {
    'cycle_time': (0.05, 5.0),
    'duty_factor': (0.5, 0.95),
    'step_height': (0.0, 0.5),
    'stand_height': (0.1, 1.0),
    'rate_hz': (1.0, 1000.0),
}

# The subset an operator may retune live over `robot{N}_gait_command`.
LIVE_SETTINGS = ('step_height', 'cycle_time', 'stand_height')


def setting_in_range(key: str, value: float) -> bool:
    """Return whether `value` is a usable setting for `key`."""
    bounds = SETTING_LIMITS.get(key)
    if bounds is None:
        return False
    low, high = bounds
    return math.isfinite(value) and low <= value <= high


def _sensor_qos() -> QoSProfile:
    """Match the simulation backend's best-effort sensor stream."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1)


class LocomotionNode(Node):
    """Drive a legged robot by publishing joint targets for FIC to track."""

    def __init__(self):
        super().__init__('locomotion_node')
        self.declare_parameter('station_id', 1)
        self.declare_parameter('robot_name', 'Spot')
        self.declare_parameter('source_prefix', DEFAULT_SOURCE_PREFIX)
        self.declare_parameter('base_state_topic', '')
        self.declare_parameter('rate_hz', DEFAULT_RATE_HZ)
        self.declare_parameter('feedback_timeout', 0.1)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_control_mode', True)
        self.declare_parameter('stand_height', 0.46)
        self.declare_parameter('cycle_time', 0.30)
        self.declare_parameter('duty_factor', 0.65)
        self.declare_parameter('step_height', 0.12)
        self.declare_parameter('max_vx', 0.6)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_yaw_rate', 1.0)

        sid = int(self.get_parameter('station_id').value)
        self._rid = 'robot%d' % sid
        self._timeout = float(self.get_parameter('feedback_timeout').value)
        self._command_timeout = float(self.get_parameter('command_timeout').value)
        self._controller = LocomotionController(self._build_config())

        self._joints: Optional[List[float]] = None
        self._height = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._velocity = (0.0, 0.0)
        self._yaw_rate = 0.0
        self._last_joint_stamp = 0.0
        self._last_odom_stamp = 0.0
        self._last_cmd_stamp = 0.0
        self._command = VelocityCommand()
        # Deliberately not seeded with a pose. Seeding with SIT_Q meant the
        # freeze path could publish a tucked pose at a robot that is standing
        # -- a ~1.16 rad step that bypasses the controller's rate limiter --
        # before any feedback had been read. `_on_joint_states` fills this in
        # from the first measured joint vector, so every target the node ever
        # publishes is derived from measured state.
        self._targets: Optional[List[float]] = None

        sensor_qos = _sensor_qos()
        self._source_prefix = str(
            self.get_parameter('source_prefix').value or DEFAULT_SOURCE_PREFIX)
        topics = resolve_state_topics(
            self._rid, self._source_prefix,
            str(self.get_parameter('base_state_topic').value or ''))
        self._joint_topic = topics.joints
        self._odom_topic = topics.base
        if topics.form == ORDERED_JOINTS:
            self.create_subscription(
                Float64MultiArray, topics.joints, self._on_joint_array,
                sensor_qos)
        else:
            self.create_subscription(
                JointState, topics.joints, self._on_joint_states, sensor_qos)
        if self._odom_topic:
            self.create_subscription(
                Odometry, self._odom_topic, self._on_odom, sensor_qos)
        else:
            # Deliberately no subscription and no substitute. `_tick` reports
            # `no_base_source` every cycle and publishes nothing.
            self.get_logger().error(
                "source_prefix='%s' has no base-state source. Trunk height,"
                ' roll, pitch and trunk-frame velocity are unknown, so this'
                ' node will not command the robot. Set base_state_topic to a'
                ' nav_msgs/Odometry topic from a real base-state estimator.'
                % self._source_prefix)
        self.create_subscription(
            Twist, '%s_cmd_vel' % self._rid, self._on_cmd_vel, 10)
        self.create_subscription(
            String, '%s_gait_command' % self._rid, self._on_gait_command, 10)

        self._pub_targets = self.create_publisher(
            JointState, '%s_target_joint_angles' % self._rid, 10)
        self._pub_mode = self.create_publisher(
            Int32, '%s_control_mode' % self._rid, 10)

        rate = self._validated('rate_hz', DEFAULT_RATE_HZ)
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._tick)

        self._mode_timer = None
        if bool(self.get_parameter('publish_control_mode').value):
            # rclpy timers are periodic, so make this fire once: keep the
            # handle and have _announce_mode cancel it on its first call.
            # That puts the FIC controller into joint impedance once at
            # startup so a headless run works without the GUI having
            # selected a mode, without repeating and overriding it later.
            self._mode_timer = self.create_timer(1.0, self._announce_mode)

        self.get_logger().info(
            'Locomotion node up for %s on %s at %.0f Hz; state from %s'
            ' (joints: %s, base: %s)'
            % (self.get_parameter('robot_name').value, self._rid, rate,
               self._source_prefix, self._joint_topic,
               self._odom_topic or '<none>'))

    def _validated(self, key: str, fallback: float) -> float:
        """Read parameter `key`, falling back with a warning if it is unusable."""
        try:
            value = float(self.get_parameter(key).value)
        except (TypeError, ValueError):
            value = float('nan')
        if setting_in_range(key, value):
            return value
        low, high = SETTING_LIMITS[key]
        self.get_logger().warn(
            'Parameter %s=%r is outside [%g, %g]; using %g instead'
            % (key, value, low, high, fallback))
        return fallback

    def _build_config(self) -> ControllerConfig:
        get = self.get_parameter
        defaults = GaitParams()
        return ControllerConfig(
            gait=GaitParams(
                cycle_time=self._validated('cycle_time', defaults.cycle_time),
                duty_factor=self._validated('duty_factor', defaults.duty_factor),
                step_height=self._validated('step_height', defaults.step_height)),
            gains=PostureGains(),
            stand_height=self._validated('stand_height', 0.46),
            max_vx=float(get('max_vx').value),
            max_vy=float(get('max_vy').value),
            max_yaw_rate=float(get('max_yaw_rate').value))

    # -- callbacks -------------------------------------------------------
    def _on_joint_states(self, msg: JointState) -> None:
        joints, missing = self._extract_joints(msg.name, msg.position)
        if joints is None:
            # Another robot's joint stream, the wrong model, or a station
            # mis-wiring. Silently dropping this left a node that published
            # nothing, logged nothing and never timed out, so say which joints
            # are missing -- throttled, because it arrives at the sensor rate.
            self.get_logger().warn(
                'joint_states is missing %d expected joint(s): %s'
                % (len(missing), ', '.join(missing[:4])),
                throttle_duration_sec=5.0)
            return
        self._accept_joints(joints)

    def _on_joint_array(self, msg: Float64MultiArray) -> None:
        """
        Accept an ordered joint vector, as `fic_control` publishes it.

        `{rid}_q_current` is a bare `Float64MultiArray` in `robots_config.yaml`
        joint order, with no names to match on -- so the length is the only
        thing that can be checked, and it is checked hard: a short or long
        vector accepted positionally would map a hip command onto a knee. The
        order itself is an assumption this node cannot verify from the wire;
        `robots_config.yaml`'s `joint_names` must equal
        `leg_kinematics.JOINT_NAMES` exactly.
        """
        values = list(msg.data)
        if len(values) != len(lk.JOINT_NAMES):
            self.get_logger().error(
                '%s carried %d values; %d are required and there are no names'
                ' to align them by, so it cannot be used'
                % (self._joint_topic, len(values), len(lk.JOINT_NAMES)),
                throttle_duration_sec=5.0)
            return
        self._accept_joints([float(value) for value in values])

    def _accept_joints(self, joints: List[float]) -> None:
        """Record a validated measured joint vector and stamp its arrival."""
        self._joints = joints
        if self._targets is None:
            # First measured pose: anchor the freeze target on it so a hold can
            # only ever republish where the robot actually is.
            self._targets = list(joints)
        self._last_joint_stamp = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(
            orientation.w, orientation.x, orientation.y, orientation.z)
        self._roll, self._pitch = roll, pitch
        self._height = msg.pose.pose.position.z
        # MuJoCo reports free-joint linear velocity in the world frame; the
        # gait reasons in the trunk frame, so rotate it by the heading.
        self._velocity = self._world_to_trunk(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, yaw)
        self._yaw_rate = msg.twist.twist.angular.z
        self._last_odom_stamp = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._command = VelocityCommand(
            msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_stamp = time.monotonic()

    def _on_gait_command(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if text in ('sit', 'stand', 'walk'):
            self._controller.request(text)
            self.get_logger().info('Gait command: %s' % text)
        elif text == 'stop':
            self._command = VelocityCommand()
        elif text.startswith('set '):
            self._apply_setting(text)
        else:
            self.get_logger().warn('Unknown gait command: %r' % msg.data)

    def _apply_setting(self, text: str) -> None:
        """Handle `set <key> <value>` for live gait tuning from the GUI."""
        parts = text.split()
        if len(parts) != 3:
            self.get_logger().warn('Malformed setting: %r' % text)
            return
        _, key, raw = parts
        try:
            value = float(raw)
        except ValueError:
            self.get_logger().warn('Non-numeric setting value: %r' % raw)
            return
        if key not in LIVE_SETTINGS:
            self.get_logger().warn('Unknown setting: %r' % key)
            return
        if not setting_in_range(key, value):
            low, high = SETTING_LIMITS[key]
            self.get_logger().warn(
                'Refusing %s = %r: outside [%g, %g]' % (key, value, low, high))
            return
        gait = self._controller.config.gait
        if key == 'step_height':
            self._controller.config.gait = GaitParams(
                gait.cycle_time, gait.duty_factor, value)
        elif key == 'cycle_time':
            self._controller.config.gait = GaitParams(
                value, gait.duty_factor, gait.step_height)
        else:
            self._controller.config.stand_height = value
        self.get_logger().info('Set %s = %.4f' % (key, value))

    # -- control loop ----------------------------------------------------
    def _tick(self) -> None:
        now = time.monotonic()
        status = self._feedback_status(
            joints_ready=self._targets is not None,
            joint_age=now - self._last_joint_stamp,
            odom_seen=self._last_odom_stamp > 0.0,
            odom_age=now - self._last_odom_stamp,
            timeout=self._timeout,
            base_source=bool(self._odom_topic))

        if status == 'no_base_source':
            # Permanent by construction, not a transient: no topic carries
            # trunk height, attitude or velocity for this source. A kinematic
            # trot with no base feedback is open-loop on a machine that falls
            # over, so the node stays silent for as long as it is configured
            # this way.
            self.get_logger().error(
                "No base-state source for source_prefix='%s'; refusing to"
                ' command the robot. Set base_state_topic to a'
                ' nav_msgs/Odometry topic from a real base-state estimator.'
                % self._source_prefix,
                throttle_duration_sec=5.0)
            return
        if status == 'wait':
            return  # nothing measured yet; stay silent rather than guess
        if status == 'no_odom':
            # Loud, because it is a wiring fault rather than a transient: joints
            # are arriving but odometry never has, so trunk height and attitude
            # are unknown and there is no measured-derived target to hold.
            self.get_logger().error(
                'Joint feedback is arriving but nothing has ever been received'
                ' on %s; refusing to command the robot' % self._odom_topic,
                throttle_duration_sec=2.0)
            return

        self._expire_stale_command(now)

        if status == 'stale':
            # Freeze: republish the last target so FIC holds the pose
            # compliantly instead of gaiting blind on stale state.
            self.get_logger().warn(
                'State feedback stale; holding last target',
                throttle_duration_sec=2.0)
            self._publish(self._targets)
            return

        state = RobotState(
            joint_positions=self._joints,
            height=self._height,
            roll=self._roll,
            pitch=self._pitch,
            velocity=self._velocity,
            yaw_rate=self._yaw_rate,
            valid=all(math.isfinite(v) for v in
                      (self._height, self._roll, self._pitch)))
        self._targets = self._controller.update(self._dt, state, self._command)
        self._publish(self._targets)

    def _expire_stale_command(self, now: float) -> None:
        """Zero a velocity command whose publisher has stopped sending."""
        if self._last_cmd_stamp <= 0.0:
            return  # nobody has ever commanded a velocity; nothing to expire
        if now - self._last_cmd_stamp <= self._command_timeout:
            return
        if self._command.speed == 0.0:
            return
        # `feedback_timeout` covers the robot's state, not the operator's. With
        # no watchdog here, teleop dying mid-trot leaves the last velocity
        # latched and the robot walks away on its own.
        self.get_logger().warn(
            'No cmd_vel for %.1f s; zeroing the velocity command'
            % (now - self._last_cmd_stamp),
            throttle_duration_sec=2.0)
        self._command = VelocityCommand()

    def _publish(self, targets: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(lk.JOINT_NAMES)
        msg.position = [float(value) for value in targets]
        self._pub_targets.publish(msg)

    def _announce_mode(self) -> None:
        """Publish joint-impedance mode once, then cancel this timer."""
        self._pub_mode.publish(Int32(data=JOINT_IMPEDANCE_MODE))
        self._mode_timer.cancel()

    # -- pure helpers ----------------------------------------------------
    @staticmethod
    def _extract_joints(
            names: Sequence[str],
            positions: Sequence[float]
    ) -> Tuple[Optional[List[float]], List[str]]:
        """
        Pull the twelve tracked joints out of a JointState, in canonical order.

        Returns `(joints, missing)`. `joints` is None when any expected joint
        is absent, and `missing` names them so the caller can say what is
        wrong instead of dropping the message silently.
        """
        by_name = dict(zip(names, positions))
        missing = [name for name in lk.JOINT_NAMES if name not in by_name]
        if missing:
            return None, missing
        return [float(by_name[name]) for name in lk.JOINT_NAMES], []

    @staticmethod
    def _feedback_status(joints_ready: bool, joint_age: float,
                         odom_seen: bool, odom_age: float,
                         timeout: float, *, base_source: bool = True) -> str:
        """
        Classify what the two feedback streams permit this tick.

        `'no_base_source'` -- the selected state source publishes no base
        state at all, so there is nothing to wait for and never will be.
        `'wait'` -- no joint feedback yet, so there is no measured pose and
        nothing may be published. `'no_odom'` -- joints are arriving but
        odometry never has; height and attitude are unknown, so again nothing
        may be published, and unlike `'stale'` this is a persistent fault
        worth shouting about. `'stale'` -- both streams have been seen but one
        has gone quiet, so hold the last measured-derived target. `'ok'` --
        both fresh, run the gait.

        `base_source` defaults to True so every pre-existing caller keeps its
        original meaning.
        """
        if not base_source:
            return 'no_base_source'
        if not joints_ready:
            return 'wait'
        if not odom_seen:
            return 'no_odom'
        if joint_age > timeout or odom_age > timeout:
            return 'stale'
        return 'ok'

    @staticmethod
    def _quat_to_rpy(w: float, x: float, y: float, z: float):
        """Decode a quaternion into roll, pitch, yaw (radians)."""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Clamp before asin: a near-vertical odometry frame would otherwise
        # push the argument outside [-1, 1] and raise mid-loop.
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _world_to_trunk(vx: float, vy: float, yaw: float):
        """Rotate a world-frame planar velocity into the trunk frame."""
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        return (vx * cos_y - vy * sin_y, vx * sin_y + vy * cos_y)


def main(args=None):
    """Run the locomotion node until shutdown (entry point for ros2 run)."""
    rclpy.init(args=args)
    node = LocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
