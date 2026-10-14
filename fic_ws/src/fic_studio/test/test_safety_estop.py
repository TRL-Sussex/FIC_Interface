"""
Safety regression: E-STOP commands a graceful controller ramp before any kill.

WS1.1 of the safety remediation. The fix factors _emergency_stop so the
graceful mode-0 ramp (broadcast_stop -> 'Stop Controller') and the last-pose
lock are issued FIRST (the robot decays to a safe hold), and the hard process
teardown happens only after a short ramp window.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.main_window_pro import ProfessionalRobotGUI  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _Pub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(list(msg.data))


class _Node:
    station_robot_id = 'robot1'

    def __init__(self):
        self.real_joint_publishers = {'robot1': _Pub()}


class _EstopStub:
    def __init__(self):
        self.node = _Node()
        self.current_joint_positions_by_robot = {'robot1': [0.1, 0.2, 0.3]}
        self.current_joint_positions = [0.1, 0.2, 0.3]
        self.broadcast_calls = 0
        self.logs = []

    def broadcast_stop(self):
        self.broadcast_calls += 1

    def log_message(self, *a):
        self.logs.append(a)


def test_safe_stop_commands_graceful_ramp_and_locks_pose():
    s = _EstopStub()
    ProfessionalRobotGUI._estop_command_safe_stop(s)
    # graceful mode-0 ramp issued to the controller before any kill
    assert s.broadcast_calls == 1
    # last-known pose published to lock the robot in place (float32 precision)
    published = s.node.real_joint_publishers['robot1'].published
    assert len(published) == 1
    assert all(abs(a - b) < 1e-5 for a, b in zip(published[0], [0.1, 0.2, 0.3]))


def test_emergency_stop_orders_safe_stop_before_kill():
    calls = []
    s = _EstopStub()
    s._ESTOP_RAMP_MS = 0
    s._estop_extensions = lambda: calls.append('ext')
    s._estop_command_safe_stop = lambda: calls.append('safe')
    s._estop_ramp_wait = lambda: calls.append('wait')
    s._estop_kill_processes = lambda: calls.append('kill')
    ProfessionalRobotGUI._emergency_stop(s)
    # the graceful safe-stop must be commanded before the hard process kill
    assert calls.index('safe') < calls.index('wait') < calls.index('kill')
