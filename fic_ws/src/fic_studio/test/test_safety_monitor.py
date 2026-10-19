"""
Safety regression: a zone violation ENFORCES a graceful stop (not just a warn).

WS3a of the safety remediation. The 10 Hz monitor used to only log + (teleop
only) stop. It now commands a debounced graceful controller stop on any
violation, for every mode, as a runtime backstop to the target clamping.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.safety_mixin import SafetyMixin  # noqa: E402


class _SafetyStub:
    def __init__(self):
        self.safety_enforce = True
        self._safety_stop_until = 0.0
        self.broadcast_calls = 0
        self.logs = []

    def broadcast_stop(self):
        self.broadcast_calls += 1

    def log_message(self, *a):
        self.logs.append(a)


def test_violation_enforces_graceful_stop_once():
    s = _SafetyStub()
    SafetyMixin._enforce_safety_stop(s, 'Z max 0.9 (limit 0.5)')
    assert s.broadcast_calls == 1            # first violation -> graceful stop
    SafetyMixin._enforce_safety_stop(s, 'Z max 0.9 (limit 0.5)')
    assert s.broadcast_calls == 1            # debounced within the cooldown


def test_enforce_disabled_does_not_stop():
    s = _SafetyStub()
    s.safety_enforce = False
    SafetyMixin._enforce_safety_stop(s, 'Z max')
    assert s.broadcast_calls == 0


# ── "Stop on violation": only clearly past a limit, once per episode, that robot only ──

class _Pos:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Pose:
    def __init__(self, x, y, z):
        self.position = _Pos(x, y, z)


class _Node:
    def __init__(self):
        self.real_joint_publishers = {'robot1': _LockPub(), 'robot2': _LockPub()}


class _LockPub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


class _Host(_SafetyStub):
    _safety_robots = SafetyMixin._safety_robots
    _safety_past = SafetyMixin._safety_past
    _enforce_safety_limits = SafetyMixin._enforce_safety_limits
    _enforce_safety_stop = SafetyMixin._enforce_safety_stop

    def __init__(self):
        super().__init__()
        self.node = _Node()
        self.stopped = []
        self.safety_limits = {
            'cartesian': {'x': {'min': -1.0, 'max': 0.6, 'enabled': True},
                          'y': {'min': -1.0, 'max': 1.0, 'enabled': True},
                          'z': {'min': 0.1, 'max': 1.5, 'enabled': True}},
            'joint_safety_deg': {'lower': [-90.0] * 2, 'upper': [90.0] * 2, 'enabled': True}}
        self.current_ee_pose_by_robot = {'robot1': _Pose(0.3, 0, 0.5), 'robot2': _Pose(0.3, 0, 0.5)}
        self.current_joint_positions_by_robot = {'robot1': [0.1, 0.2], 'robot2': [0.1, 0.2]}

    def stop_for(self, rid):
        self.stopped.append(rid)


def test_pressed_against_a_limit_is_not_stopped():
    s = _Host()
    s.current_ee_pose_by_robot['robot1'] = _Pose(0.62, 0, 0.5)     # 20 mm past X max: the wall holds it
    s._enforce_safety_limits()
    assert s.stopped == []


def test_clearly_past_stops_that_robot_once_and_locks_its_pose():
    s = _Host()
    s.current_ee_pose_by_robot['robot2'] = _Pose(0.3, 0, 0.06)     # 40 mm below Z min
    s._enforce_safety_limits()
    s._enforce_safety_limits()
    assert s.stopped == ['robot2']
    assert s.node.real_joint_publishers['robot2'].sent and not s.node.real_joint_publishers['robot1'].sent
    assert 'Z min' in s.logs[0][1]


def test_rearms_only_once_back_near_the_limit():
    s = _Host()
    s.current_ee_pose_by_robot['robot1'] = _Pose(0.65, 0, 0.5)     # 50 mm past
    s._enforce_safety_limits()
    s._safety_stop_until_by_robot = {}                             # past the cooldown
    s.current_ee_pose_by_robot['robot1'] = _Pose(0.62, 0, 0.5)     # 20 mm: not yet re-armed
    s._enforce_safety_limits()
    s.current_ee_pose_by_robot['robot1'] = _Pose(0.65, 0, 0.5)
    s._enforce_safety_limits()
    assert s.stopped == ['robot1']
    s.current_ee_pose_by_robot['robot1'] = _Pose(0.61, 0, 0.5)     # 10 mm: re-armed
    s._enforce_safety_limits()
    s.current_ee_pose_by_robot['robot1'] = _Pose(0.65, 0, 0.5)
    s._enforce_safety_limits()
    assert s.stopped == ['robot1', 'robot1']


def test_joint_past_its_limit_by_more_than_the_margin_stops():
    import math
    s = _Host()
    s.current_joint_positions_by_robot['robot1'] = [math.radians(91.0), 0.0]
    s._enforce_safety_limits()
    assert s.stopped == []
    s.current_joint_positions_by_robot['robot1'] = [math.radians(93.0), 0.0]
    s._enforce_safety_limits()
    assert s.stopped == ['robot1']


def test_enforce_off_never_stops():
    s = _Host()
    s.safety_enforce = False
    s.current_ee_pose_by_robot['robot1'] = _Pose(0.9, 0, 0.5)
    s._enforce_safety_limits()
    assert s.stopped == []
