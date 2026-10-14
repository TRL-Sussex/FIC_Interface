"""Gamepad locomotion mapping, exercised with stubs (no real /dev/input/js*).

gamepad_mixin reads Linux joystick device nodes directly, so it can't be
driven end-to-end on this (macOS) box. These tests stub out the pieces
that would normally come from a live device or a ROS publisher and check
the pure mapping/branching logic instead.
"""
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from fic_studio.mixins.gamepad_mixin import GamepadMixin, _PadState  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Host(GamepadMixin):
    def __init__(self, legged, sticks=None):
        self._legged = legged
        self._sticks = sticks or {}
        self.node = types.SimpleNamespace(
            pubs={'robot1': {'cmd_vel': _Pub()}})

    def _loco_is_legged(self, robot_name):
        return robot_name == 'Spot' if self._legged else False

    def _gp_robot_type_for(self, rid):
        return 'Spot' if self._legged else 'Franka_FR3'

    def _gp_stick(self, axis_name, pad):
        return self._sticks.get(axis_name, 0.0)


def _pad():
    return _PadState(path='/dev/input/js0', target_robot_n=1)


def test_legged_pad_is_detected():
    host = _Host(legged=True)
    assert host._gp_is_legged_pad(_pad()) is True


def test_arm_pad_is_not_legged():
    host = _Host(legged=False)
    assert host._gp_is_legged_pad(_pad()) is False


def test_hasattr_guard_when_locomotion_mixin_absent():
    class _BareHost(GamepadMixin):
        def __init__(self):
            self.node = types.SimpleNamespace(pubs={'robot1': {'cmd_vel': _Pub()}})

    host = _BareHost()
    assert host._gp_is_legged_pad(_pad()) is False


def test_locomotion_tick_publishes_a_twist_from_the_sticks():
    host = _Host(legged=True, sticks={'LY': 0.5, 'LX': -0.2, 'RX': 0.1})
    pad = _pad()
    host._gp_publish_locomotion_tick(pad)
    sent = host.node.pubs['robot1']['cmd_vel'].sent
    assert len(sent) == 1
    msg = sent[0]
    assert msg.linear.x == 0.5 * 0.6
    assert msg.linear.y == -0.2 * -1.0 * 0.4
    assert msg.angular.z == 0.1 * -1.0 * 1.0


def test_locomotion_tick_is_safe_when_the_topic_is_missing():
    host = _Host(legged=True)
    host.node.pubs['robot1'] = {}
    host._gp_publish_locomotion_tick(_pad())  # must not raise
