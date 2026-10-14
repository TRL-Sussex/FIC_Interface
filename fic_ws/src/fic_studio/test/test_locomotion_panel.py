"""Locomotion panel: legged detection, command publishing, velocity clamping."""
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from fic_studio.mixins.locomotion_mixin import LocomotionMixin  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Host(LocomotionMixin):
    def __init__(self, category='legged'):
        self.robot_configs = {
            'Spot': {'robot_category': 'legged', 'num_joints': 12},
            'Franka_FR3': {'num_joints': 7},
        }
        self.robot_dropdown = types.SimpleNamespace(
            currentText=lambda: 'Spot' if category == 'legged' else 'Franka_FR3')
        self.node = types.SimpleNamespace(
            station_robot_id='robot1',
            pubs={'robot1': {'cmd_vel': _Pub(), 'gait_command': _Pub()}})
        self.logs = []

    def log_message(self, *args):
        self.logs.append(args)


def test_detects_a_legged_robot():
    host = _Host()
    assert host._loco_is_legged('Spot') is True
    assert host._loco_is_legged('Franka_FR3') is False
    assert host._loco_is_legged('NoSuchRobot') is False


def test_gait_command_is_published_as_a_string():
    host = _Host()
    host._loco_send('stand')
    sent = host.node.pubs['robot1']['gait_command'].sent
    assert [msg.data for msg in sent] == ['stand']


def test_velocity_publishes_a_twist_from_the_slider_values():
    host = _Host()
    host._loco_velocity = (0.3, -0.1, 0.4)
    host._loco_publish_velocity()
    sent = host.node.pubs['robot1']['cmd_vel'].sent
    assert len(sent) == 1
    assert sent[0].linear.x == 0.3
    assert sent[0].linear.y == -0.1
    assert sent[0].angular.z == 0.4


def test_stop_zeroes_the_command():
    host = _Host()
    host._loco_velocity = (0.5, 0.0, 0.0)
    host._loco_stop()
    assert host._loco_velocity == (0.0, 0.0, 0.0)
    sent = host.node.pubs['robot1']['cmd_vel'].sent
    assert sent[-1].linear.x == 0.0


def test_publishing_is_safe_when_the_topic_is_missing():
    host = _Host()
    host.node.pubs['robot1'] = {}
    host._loco_velocity = (0.3, 0.0, 0.0)
    host._loco_publish_velocity()  # must not raise
    host._loco_send('walk')


def test_setting_command_uses_the_documented_grammar():
    host = _Host()
    host._loco_set('step_height', 0.1)
    sent = host.node.pubs['robot1']['gait_command'].sent
    assert sent[-1].data == 'set step_height 0.1'
