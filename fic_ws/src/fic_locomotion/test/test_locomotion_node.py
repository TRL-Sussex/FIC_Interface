"""
The locomotion node: feedback gating, command validation and pure helpers.

The helpers (orientation decoding, velocity frame conversion) are the two
places a sign or frame error would silently destabilise the gait, so they are
tested against hand-computed values rather than round-trips.

The rest of the file constructs a real `LocomotionNode` against fake
publishers. Nothing here had ever built one, which is exactly why a startup
step command and a divide-by-zero shutdown both shipped.
"""
import contextlib
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402

from fic_locomotion import leg_kinematics as lk  # noqa: E402
from fic_locomotion.locomotion_node import (  # noqa: E402
    DEFAULT_SOURCE_PREFIX, HARDWARE_SOURCE, JOINT_IMPEDANCE_MODE,
    LIVE_SETTINGS, LocomotionNode, ORDERED_JOINTS, NAMED_JOINTS,
    SETTING_LIMITS, resolve_state_topics, setting_in_range)


def test_identity_quaternion_is_level():
    assert LocomotionNode._quat_to_rpy(1.0, 0.0, 0.0, 0.0) == \
        pytest.approx((0.0, 0.0, 0.0))


def test_roll_pitch_yaw_are_decoded_independently():
    half = math.sin(0.05)
    roll, pitch, yaw = LocomotionNode._quat_to_rpy(math.cos(0.05), half, 0.0, 0.0)
    assert roll == pytest.approx(0.1, abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    roll, pitch, yaw = LocomotionNode._quat_to_rpy(math.cos(0.05), 0.0, half, 0.0)
    assert pitch == pytest.approx(0.1, abs=1e-6)
    roll, pitch, yaw = LocomotionNode._quat_to_rpy(math.cos(0.05), 0.0, 0.0, half)
    assert yaw == pytest.approx(0.1, abs=1e-6)


def test_extreme_pitch_does_not_blow_up():
    # asin() must be guarded or a near-gimbal odometry frame raises. An
    # *exactly* normalised quaternion cannot prove that: it makes sinp exactly
    # 1.0, where bare asin succeeds, so deleting the clamp left this test
    # green. Odometry arrives over the wire as float32-rounded values whose
    # norm is not exactly one; 0.70711 is over-normalised (norm^2 = 1.00002),
    # so 2*w*y = 1.0000200 > 1 and an unguarded asin raises ValueError. That
    # is the failure this guard exists for, so that is what is fed in.
    w = y = 0.70711
    assert 2.0 * w * y > 1.0, 'fixture no longer exercises the clamp'
    roll, pitch, yaw = LocomotionNode._quat_to_rpy(w, 0.0, y, 0.0)
    assert all(math.isfinite(v) for v in (roll, pitch, yaw))
    assert pitch == pytest.approx(math.pi / 2, abs=1e-3)


class _FakeTimer:
    """Minimal stand-in for the rclpy timer handle used by one test."""

    def __init__(self):
        self.cancel_calls = 0

    def cancel(self):
        """Record that the timer was cancelled."""
        self.cancel_calls += 1


class _FakePublisher:
    """Minimal stand-in for an rclpy publisher used by one test."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        """Record the message that would have been published."""
        self.published.append(msg)


class _ModeAnnounceStub:
    """Bare object carrying only what `_announce_mode` touches on self."""

    def __init__(self):
        self._pub_mode = _FakePublisher()
        self._mode_timer = _FakeTimer()


class _FakeLogger:
    """Collect log lines so a test can assert the node said something."""

    def __init__(self):
        self.lines = []

    def _record(self, text, **_):
        self.lines.append(text)

    warn = info = error = _record


class _JointArrayStub:
    """Bare object carrying only what `_on_joint_array` touches on self."""

    def __init__(self):
        self._joints = None
        self._targets = None
        self._last_joint_stamp = 0.0
        self._joint_topic = 'robot1_q_current'
        self._logger = _FakeLogger()

    def get_logger(self):
        """Return the collecting logger."""
        return self._logger

    def _accept_joints(self, joints):
        """Delegate to the real implementation, not a fake."""
        LocomotionNode._accept_joints(self, joints)


def _float_array(values):
    msg = Float64MultiArray()
    msg.data = [float(v) for v in values]
    return msg


def test_control_mode_is_announced_only_once():
    stub = _ModeAnnounceStub()
    LocomotionNode._announce_mode(stub)
    assert len(stub._pub_mode.published) == 1
    assert stub._pub_mode.published[0].data == JOINT_IMPEDANCE_MODE
    assert stub._mode_timer.cancel_calls == 1


def test_world_velocity_rotates_into_the_trunk_frame():
    # Facing +y, world +x velocity is the robot's own -y (rightwards).
    vx, vy = LocomotionNode._world_to_trunk(1.0, 0.0, math.pi / 2)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(-1.0, abs=1e-9)


def test_zero_yaw_leaves_velocity_untouched():
    assert LocomotionNode._world_to_trunk(0.3, -0.2, 0.0) == \
        pytest.approx((0.3, -0.2))


# ── feedback gating, as a pure classification ───────────────────────────────
# `_feedback_status` is split out of `_tick` precisely so the three "do not
# publish" reasons can be told apart in a test: they are not interchangeable.

def test_missing_joints_waits_silently():
    assert LocomotionNode._feedback_status(False, 0.0, False, 0.0, 0.1) == 'wait'


def test_joints_without_odometry_is_its_own_loud_case():
    # NOT 'stale': stale republishes a measured-derived target, which is safe.
    # Having never seen odometry means height/attitude are unknown, so there is
    # no derived target to hold and the node must publish nothing and say so.
    status = LocomotionNode._feedback_status(True, 0.0, False, 0.0, 0.1)
    assert status == 'no_odom'
    assert status not in ('ok', 'stale')


def test_either_stream_going_quiet_is_stale():
    assert LocomotionNode._feedback_status(True, 0.5, True, 0.0, 0.1) == 'stale'
    assert LocomotionNode._feedback_status(True, 0.0, True, 0.5, 0.1) == 'stale'


def test_two_fresh_streams_are_ok():
    assert LocomotionNode._feedback_status(True, 0.0, True, 0.0, 0.1) == 'ok'


def test_missing_joint_names_are_reported_not_swallowed():
    # A joint-name mismatch used to be caught and dropped with no log at all:
    # the node published nothing, logged nothing and never timed out.
    joints, missing = LocomotionNode._extract_joints(
        ['fl_hx', 'nonsense'], [0.1, 0.2])
    assert joints is None
    assert 'fl_hy' in missing


def test_a_complete_joint_message_extracts_in_joint_name_order():
    positions = [float(i) for i in range(len(lk.JOINT_NAMES))]
    joints, missing = LocomotionNode._extract_joints(
        list(reversed(lk.JOINT_NAMES)), list(reversed(positions)))
    assert missing == []
    assert joints == positions


# ── state-source resolution ─────────────────────────────────────────────────
# The node used to hardcode '/{rid}_simulation/', which made it structurally
# simulation-only. `resolve_state_topics` is the seam; these tests pin both
# that the default is byte-identical to the hardcoded string it replaced and
# that a non-default source resolves where its data actually lives.

def test_the_default_source_is_todays_simulation_topics():
    topics = resolve_state_topics('robot1', DEFAULT_SOURCE_PREFIX)
    assert topics.joints == '/robot1_simulation/joint_states'
    assert topics.base == '/robot1_simulation/odom'
    assert topics.form == NAMED_JOINTS


def test_an_empty_source_prefix_falls_back_to_the_default():
    assert resolve_state_topics('robot2', '') == \
        resolve_state_topics('robot2', DEFAULT_SOURCE_PREFIX)


def test_a_non_default_prefix_moves_both_feedback_topics():
    topics = resolve_state_topics('robot3', 'vr')
    assert topics.joints == '/robot3_vr/joint_states'
    assert topics.base == '/robot3_vr/odom'
    assert topics.form == NAMED_JOINTS


def test_the_station_id_scopes_the_source_namespace():
    assert resolve_state_topics('robot4', DEFAULT_SOURCE_PREFIX).joints == \
        '/robot4_simulation/joint_states'


def test_the_hardware_source_reads_the_topic_fic_hardware_actually_publishes():
    # fic_control publishes `{rid}_q_current` (Float64MultiArray, in the
    # robots_config.yaml joint order) and no sensor_msgs/JointState at all.
    topics = resolve_state_topics('robot1', HARDWARE_SOURCE)
    assert topics.joints == 'robot1_q_current'
    assert topics.form == ORDERED_JOINTS


def test_the_hardware_source_has_no_base_state_topic():
    # There is no base-state publisher anywhere in this codebase on the
    # hardware path. Inventing one here would let the gait run on a real
    # legged robot with fabricated trunk height and attitude.
    assert resolve_state_topics('robot1', HARDWARE_SOURCE).base == ''


def test_an_explicit_base_state_topic_overrides_the_derived_one():
    # The seam for a future state estimator: point the node at whatever
    # publishes nav_msgs/Odometry for the trunk.
    topics = resolve_state_topics('robot1', HARDWARE_SOURCE, '/spot/state_estimate')
    assert topics.base == '/spot/state_estimate'
    topics = resolve_state_topics('robot1', DEFAULT_SOURCE_PREFIX, '/elsewhere/odom')
    assert topics.base == '/elsewhere/odom'
    assert topics.joints == '/robot1_simulation/joint_states'


def test_a_source_with_no_base_state_refuses_to_command():
    # Distinct from 'no_odom': that one means "a topic exists and nothing has
    # arrived". This one means "no base-state source is configured at all",
    # which can never resolve itself and must be said differently.
    status = LocomotionNode._feedback_status(
        True, 0.0, False, 0.0, 0.1, base_source=False)
    assert status == 'no_base_source'
    assert status not in ('ok', 'stale')


def test_a_base_state_source_is_assumed_present_by_default():
    # Keeps every pre-existing caller of _feedback_status meaning what it did.
    assert LocomotionNode._feedback_status(True, 0.0, True, 0.0, 0.1) == 'ok'


def test_an_ordered_joint_vector_of_the_wrong_length_is_refused():
    # A Float64MultiArray carries no joint names, so length is the only thing
    # that can be checked -- and a short vector silently mis-indexed would map
    # a hip command onto a knee.
    stub = _JointArrayStub()
    LocomotionNode._on_joint_array(stub, _float_array([0.0] * 11))
    assert stub._joints is None
    assert stub._targets is None


def test_an_ordered_joint_vector_of_the_right_length_is_accepted():
    stub = _JointArrayStub()
    values = [float(i) / 100.0 for i in range(len(lk.JOINT_NAMES))]
    LocomotionNode._on_joint_array(stub, _float_array(values))
    assert stub._joints == values
    assert stub._targets == values     # seeded from the measured pose


# ── setting validation ──────────────────────────────────────────────────────

def test_zero_cycle_time_is_out_of_range():
    # trot_scheduler.advance divides by cycle_time inside the timer callback,
    # so a zero here used to raise ZeroDivisionError out of rclpy.spin and kill
    # the node while the robot stood on twelve torque actuators.
    assert not setting_in_range('cycle_time', 0.0)
    assert setting_in_range('cycle_time', 0.30)


def test_negative_geometry_is_out_of_range():
    assert not setting_in_range('step_height', -0.05)
    assert not setting_in_range('stand_height', -0.46)


def test_non_finite_settings_are_out_of_range():
    assert not setting_in_range('cycle_time', float('nan'))
    assert not setting_in_range('step_height', float('inf'))


def test_unknown_settings_are_out_of_range():
    assert not setting_in_range('nonsense', 1.0)


def test_every_live_setting_has_a_declared_range():
    assert set(LIVE_SETTINGS) <= set(SETTING_LIMITS)


# ── the node itself ─────────────────────────────────────────────────────────

@contextlib.contextmanager
def _node(**overrides):
    """Construct a real LocomotionNode with fake publishers, then tear it down."""
    args = None
    if overrides:
        args = ['--ros-args']
        for key, value in overrides.items():
            args += ['-p', '%s:=%s' % (key, value)]
    rclpy.init(args=args)
    node = None
    try:
        # Constructed inside the try so a constructor that raises still shuts
        # the context down; rclpy refuses a second init() otherwise and every
        # later test in the file fails for the wrong reason.
        node = LocomotionNode()
        node._pub_targets = _FakePublisher()
        node._pub_mode = _FakePublisher()
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def _joint_msg(positions):
    msg = JointState()
    msg.name = list(lk.JOINT_NAMES)
    msg.position = [float(value) for value in positions]
    return msg


def _odom_msg(height=0.46):
    msg = Odometry()
    msg.pose.pose.position.z = height
    msg.pose.pose.orientation.w = 1.0
    return msg


def _feed(node, joints, height=0.46):
    node._on_joint_states(_joint_msg(joints))
    node._on_odom(_odom_msg(height))


def test_nothing_is_published_before_the_first_odometry():
    # The node used to seed _targets with SIT_Q and publish it whenever the
    # staleness check tripped -- which it does on every tick before the first
    # odom message, because _last_odom_stamp starts at 0.0. Against a standing
    # robot that is a ~1.16 rad step straight past the rate limiter, and if
    # odometry never arrives it is commanded forever.
    with _node() as node:
        node._on_joint_states(_joint_msg(lk.STAND_Q * 4))
        for _ in range(50):
            node._tick()
        assert node._pub_targets.published == []


def test_the_first_published_target_comes_from_the_measured_pose():
    with _node() as node:
        standing = list(lk.STAND_Q * 4)
        _feed(node, standing)
        node._tick()
        published = list(node._pub_targets.published[-1].position)
        assert max(abs(a - b) for a, b in zip(published, standing)) < 0.05


def test_the_freeze_path_never_publishes_a_pose_it_did_not_measure():
    # The first ever tick takes the freeze branch here: both streams have been
    # seen, but the joint stream has already gone quiet. Whatever `_targets`
    # was seeded with is what gets published, which is why it is seeded from
    # the measured pose rather than from SIT_Q.
    with _node() as node:
        standing = list(lk.STAND_Q * 4)
        _feed(node, standing)
        node._last_joint_stamp -= 10.0
        node._tick()
        held = list(node._pub_targets.published[-1].position)
        assert held == pytest.approx(standing, abs=1e-9)


def test_a_held_target_is_the_measured_one_not_the_sit_pose():
    with _node() as node:
        standing = list(lk.STAND_Q * 4)
        _feed(node, standing)
        node._tick()
        node._last_odom_stamp -= 10.0  # odom drops out after being seen
        node._tick()
        held = list(node._pub_targets.published[-1].position)
        assert max(abs(a - b) for a, b in zip(held, standing)) < 0.05
        assert held != pytest.approx(list(lk.SIT_Q * 4), abs=1e-2)


def test_the_hardware_source_never_commands_the_robot():
    # The whole point of the seam: joints genuinely arrive on hardware, base
    # state genuinely does not, and a kinematic trot without base feedback is
    # open-loop on a 50 kg machine. Joint feedback alone must not be enough.
    with _node(source_prefix='hardware') as node:
        assert node._joint_topic == 'robot1_q_current'
        assert node._odom_topic == ''
        for _ in range(50):
            node._on_joint_array(_float_array(lk.STAND_Q * 4))
            node._tick()
        assert node._joints is not None       # joints did arrive
        assert node._pub_targets.published == []


def test_the_default_node_still_wires_the_simulation_topics():
    with _node() as node:
        assert node._joint_topic == '/robot1_simulation/joint_states'
        assert node._odom_topic == '/robot1_simulation/odom'


def test_a_zero_rate_is_rejected_rather_than_dividing_by_zero():
    with _node(rate_hz=0.0) as node:
        assert node._dt > 0.0
        assert math.isfinite(node._dt)


def test_a_zero_cycle_time_setting_is_rejected():
    with _node() as node:
        before = node._controller.config.gait.cycle_time
        node._apply_setting('set cycle_time 0')
        assert node._controller.config.gait.cycle_time == before


def _tick_for(node, count, command=None):
    for _ in range(count):
        node._last_joint_stamp = node._last_odom_stamp = time.monotonic()
        if command is not None:
            node._on_cmd_vel(command)
        node._tick()


def test_a_rejected_cycle_time_cannot_kill_the_control_loop():
    with _node() as node:
        _feed(node, lk.STAND_Q * 4)
        node._on_gait_command(_string('stand'))
        node._on_gait_command(_string('set cycle_time 0'))
        # Long enough to finish the 2 s stand-up blend, then walk: TROT is the
        # only state that runs the gait clock and so divides by cycle_time.
        _tick_for(node, 500)
        node._on_gait_command(_string('walk'))
        _tick_for(node, 200, _twist(0.3))  # must not raise ZeroDivisionError
        assert node._controller.state.value == 'trot'
        assert node._pub_targets.published


def test_a_zero_cycle_time_launch_parameter_is_rejected():
    with _node(cycle_time=0.0) as node:
        assert node._controller.config.gait.cycle_time > 0.0


def test_a_degenerate_duty_factor_is_rejected():
    # leg_phase divides by (1 - duty_factor).
    with _node(duty_factor=1.0) as node:
        assert node._controller.config.gait.duty_factor < 1.0


def test_a_dead_cmd_vel_publisher_stops_the_robot():
    # Without a command watchdog the last velocity stays latched forever, so a
    # teleop process dying mid-trot leaves the robot walking away.
    with _node() as node:
        _feed(node, lk.STAND_Q * 4)
        node._on_cmd_vel(_twist(0.5))
        node._tick()
        assert node._command.vx == pytest.approx(0.5)
        node._last_cmd_stamp -= 10.0
        node._last_joint_stamp = node._last_odom_stamp = time.monotonic()
        node._tick()
        assert node._command.vx == 0.0
        assert node._command.speed == 0.0


def test_a_live_cmd_vel_publisher_keeps_its_command():
    with _node() as node:
        _feed(node, lk.STAND_Q * 4)
        node._on_cmd_vel(_twist(0.5))
        for _ in range(20):
            node._last_joint_stamp = node._last_odom_stamp = time.monotonic()
            node._on_cmd_vel(_twist(0.5))
            node._tick()
        assert node._command.vx == pytest.approx(0.5)


def test_unknown_and_malformed_gait_commands_are_survivable():
    with _node() as node:
        for text in ('', 'fly', 'set', 'set cycle_time', 'set cycle_time abc',
                     'set nonsense 1.0', 'set step_height -1'):
            node._on_gait_command(_string(text))
        assert node._controller.config.gait.cycle_time > 0.0
        assert node._controller.config.gait.step_height >= 0.0


def _twist(vx=0.0, vy=0.0, yaw_rate=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, yaw_rate
    return msg


def _string(text):
    msg = String()
    msg.data = text
    return msg
