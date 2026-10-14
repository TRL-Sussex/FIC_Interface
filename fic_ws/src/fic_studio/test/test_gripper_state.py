"""Tests for the measured gripper state: JointStateMixin + ExtensionHost.

fic_hardware publishes {rid}_gripper_state = [width_m, motor_current] once
the driver writes the gripper slot in shared memory. gripper_state_for()
must hand an extension (width_m, current, age_s) and must return None --
not a zero width, which would read as "fully closed" -- when no message has
ever arrived (simulation, no gripper, an older driver).
"""
import math
import sys
import types

import fic_studio.mixins.joint_state_mixin as jsm
from fic_studio.mixins.joint_state_mixin import JointStateMixin
from rclpy.qos import ReliabilityPolicy


class _Clock:
    """Stands in for the mixin's `time` module so ages are deterministic."""

    def __init__(self, now=100.0):
        self.now = now

    def monotonic(self):
        return self.now


def _host(active_robots=()):
    """Return a JointStateMixin host wired to a recording stub node.

    Same shape as test_contact_telemetry._host(); ``node.subs`` records
    every create_subscription call as (msg_type, topic, callback, qos).
    """
    node = types.SimpleNamespace()
    node.active_robots = list(active_robots)
    node.station_robot_id = 'robot1'
    node.subs = []

    def create_subscription(msg_type, topic, callback, qos):
        node.subs.append((msg_type, topic, callback, qos))
        return object()

    node.create_subscription = create_subscription
    node.destroy_subscription = lambda sub: None

    host = JointStateMixin()
    host.node = node
    host._init_joint_state()
    return host


def _msg(*data):
    return types.SimpleNamespace(data=list(data))


class TestGripperStateFor:

    def test_none_before_first_message(self):
        """No message is 'no gripper feedback', never a zero width."""
        assert _host().gripper_state_for('robot1') is None

    def test_callback_stores_the_state_and_the_age_grows(self, monkeypatch):
        clock = _Clock()
        monkeypatch.setattr(jsm, 'time', clock)
        host = _host()
        host._on_gripper_state('robot1', _msg(0.031, 412.0))
        assert host.gripper_state_for('robot1') == (0.031, 412.0, 0.0)
        clock.now += 0.25
        width, current, age = host.gripper_state_for('robot1')
        assert (width, current) == (0.031, 412.0)
        assert math.isclose(age, 0.25)

    def test_a_new_message_resets_the_age(self, monkeypatch):
        clock = _Clock()
        monkeypatch.setattr(jsm, 'time', clock)
        host = _host()
        host._on_gripper_state('robot1', _msg(0.085, 0.0))
        clock.now += 1.0
        host._on_gripper_state('robot1', _msg(0.020, 380.0))
        assert host.gripper_state_for('robot1') == (0.020, 380.0, 0.0)

    def test_unknown_robot_is_none(self):
        host = _host()
        host._on_gripper_state('robot1', _msg(0.04, 100.0))
        assert host.gripper_state_for('robot2') is None

    def test_rid_is_normalised_like_the_neighbouring_accessors(self):
        host = _host()
        host._on_gripper_state('robot1', _msg(0.04, 100.0))
        for rid in (None, 1, 'robot1', 'robot_1'):
            assert host.gripper_state_for(rid)[:2] == (0.04, 100.0), rid

    def test_a_closed_gripper_is_a_real_reading(self):
        """After the driver's first sample a zero width is genuine."""
        host = _host()
        host._on_gripper_state('robot1', _msg(0.0, 0.0))
        assert host.gripper_state_for('robot1')[:2] == (0.0, 0.0)

    def test_bad_messages_do_not_clobber_the_last_good_reading(self):
        host = _host()
        host._on_gripper_state('robot1', _msg(0.05, 200.0))
        host._on_gripper_state('robot1', _msg())
        host._on_gripper_state('robot1', _msg(0.01))
        host._on_gripper_state('robot1', _msg(float('nan'), 1.0))
        host._on_gripper_state('robot1', _msg(0.01, float('inf')))
        assert host.gripper_state_for('robot1')[:2] == (0.05, 200.0)

    def test_removing_the_robot_drops_its_gripper_state(self):
        host = _host()
        host._js_subs_by_robot['robot1'] = {'gripper_state': object()}
        host._on_gripper_state('robot1', _msg(0.05, 200.0))
        host._remove_robot_subscriptions(1)
        assert host.gripper_state_for('robot1') is None


def test_each_robot_subscribes_to_its_gripper_state(monkeypatch):
    """{rid}_gripper_state, Float64MultiArray, best-effort QoS.

    fic_hardware publishes with SensorDataQoS (best effort); a reliable
    subscription would silently never match it.
    """
    # _add_robot_subscriptions defers `from communication.msg import
    # ContactEstimate`; stub it so this runs without a sourced workspace.
    comm = types.ModuleType('communication')
    comm_msg = types.ModuleType('communication.msg')
    comm_msg.ContactEstimate = object
    comm.msg = comm_msg
    monkeypatch.setitem(sys.modules, 'communication', comm)
    monkeypatch.setitem(sys.modules, 'communication.msg', comm_msg)

    host = _host(active_robots=[2])
    found = [s for s in host.node.subs if s[1] == 'robot2_gripper_state']
    assert len(found) == 1
    msg_type, _topic, callback, qos = found[0]
    assert msg_type is jsm.Float64MultiArray
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert 'gripper_state' in host._js_subs_by_robot['robot2']

    callback(_msg(0.012, 450.0))
    assert host.gripper_state_for('robot2')[:2] == (0.012, 450.0)


# ── ExtensionHost ──────────────────────────────────────────────────────

def test_the_host_delegates_with_a_normalised_rid(qapp):
    from PyQt5.QtCore import QObject
    from fic_studio.extensions.host import ExtensionHost

    class _Window(QObject):

        def __init__(self):
            super().__init__()
            self.asked = []

        def gripper_state_for(self, rid):
            self.asked.append(rid)
            return (0.03, 300.0, 0.02)

    window = _Window()
    host = ExtensionHost(window, 'gaze')
    assert host.gripper_state_for('robot_2') == (0.03, 300.0, 0.02)
    assert host.gripper_state_for(3) == (0.03, 300.0, 0.02)
    assert window.asked == ['robot2', 'robot3']


def test_the_host_passes_through_none(qapp):
    """No gripper feedback stays None through the host."""
    from PyQt5.QtCore import QObject
    from fic_studio.extensions.host import ExtensionHost

    class _Window(QObject):

        def gripper_state_for(self, rid):
            return None

    assert ExtensionHost(_Window(), 'gaze').gripper_state_for('robot1') is None


def test_a_window_without_the_accessor_gives_none(qapp):
    from PyQt5.QtCore import QObject
    from fic_studio.extensions.host import ExtensionHost

    assert ExtensionHost(QObject(), 'gaze').gripper_state_for('robot1') is None
