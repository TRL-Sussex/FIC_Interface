"""The locomotion node starts only for legged robots, and is torn down."""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from fic_studio.main_window_pro import ProfessionalRobotGUI  # noqa: E402
from fic_studio.mixins.dashboard_mixin import DashboardMixin  # noqa: E402
from fic_studio.mixins.process_mixin import _PROC_ATTRS  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _Host:
    def __init__(self, legged=True):
        self.legged = legged
        self.simulation_running = True
        self.fic_running = False
        self.locomotion_running = False
        self.started = []

    def _loco_current_is_legged(self):
        return self.legged

    def _start_fic_simulation(self):
        self.started.append('fic')
        self.fic_running = True

    def _start_locomotion(self):
        self.started.append('locomotion')
        self.locomotion_running = True

    def _update_status(self):
        pass

    def log_message(self, *args):
        pass


def test_locomotion_process_is_registered_for_teardown():
    names = [entry[0] for entry in _PROC_ATTRS]
    assert 'locomotion_process' in names


def test_legged_robot_starts_fic_and_locomotion():
    host = _Host(legged=True)
    DashboardMixin._dash_start_fic_after_sim(host)
    assert host.started == ['fic', 'locomotion']


def test_arm_robot_starts_only_fic():
    host = _Host(legged=False)
    DashboardMixin._dash_start_fic_after_sim(host)
    assert host.started == ['fic']


# ── Finding 1(a): the interactive stop button must also stop locomotion ──────

class _ToggleSimHost:
    """Stub for _dash_toggle_sim's stop branch (sim + FIC + locomotion all up)."""

    def __init__(self):
        self.simulation_running = True
        self.fic_running = True
        self.locomotion_running = True
        self.stopped = []

    def _stop_fic_simulation(self):
        self.stopped.append('fic')
        self.fic_running = False

    def _stop_simulation(self):
        self.stopped.append('sim')
        self.simulation_running = False

    def _stop_locomotion(self):
        self.stopped.append('locomotion')
        self.locomotion_running = False

    def _update_status(self):
        pass

    def log_message(self, *args):
        pass


def test_dash_toggle_sim_stop_branch_also_stops_locomotion():
    host = _ToggleSimHost()
    DashboardMixin._dash_toggle_sim(host)
    assert 'locomotion' in host.stopped
    assert host.locomotion_running is False


def test_dash_toggle_sim_stop_branch_is_a_noop_when_locomotion_not_running():
    host = _ToggleSimHost()
    host.locomotion_running = False
    DashboardMixin._dash_toggle_sim(host)
    assert 'locomotion' not in host.stopped


# ── Finding 1(b): switching to a non-legged robot must stop locomotion ───────

class _Label:
    """Minimal stand-in for robot_status_label — only setText is read."""

    def setText(self, *_):
        pass


class _RobotSwitchHost:
    """
    Stub carrying just the attributes _on_robot_changed reads/writes.

    Deliberately omits joint_sliders_layout, tuning_robot_label,
    robot_configs, real_joint_layout, load_robot_safety_limits, and
    _rc_apply_mode_availability so the corresponding hasattr()-guarded
    branches in _on_robot_changed are skipped.
    """

    def __init__(self, current_robot_is_legged, locomotion_running=True):
        self.robots = {'Spot': 12, 'Franka_FR3': 7}
        self.robot_status_label = _Label()
        self.locomotion_running = locomotion_running
        self._current_robot_is_legged = current_robot_is_legged
        self.stopped = []

    def _loco_is_legged(self, robot_name):
        return self._current_robot_is_legged

    def _stop_locomotion(self):
        self.stopped.append('locomotion')
        self.locomotion_running = False

    def _update_tuning_joint_count(self):
        pass

    def log_message(self, *args):
        pass

    def _sync_robot_menu_check(self):
        pass

    def _loco_refresh_visibility(self):
        pass


def test_switching_to_non_legged_robot_stops_locomotion():
    host = _RobotSwitchHost(current_robot_is_legged=False, locomotion_running=True)
    ProfessionalRobotGUI._on_robot_changed(host, 'Franka_FR3')
    assert host.stopped == ['locomotion']
    assert host.locomotion_running is False


def test_switching_to_legged_robot_leaves_locomotion_running():
    host = _RobotSwitchHost(current_robot_is_legged=True, locomotion_running=True)
    ProfessionalRobotGUI._on_robot_changed(host, 'Spot')
    assert host.stopped == []
    assert host.locomotion_running is True


def test_switching_robots_is_a_noop_when_locomotion_not_running():
    host = _RobotSwitchHost(current_robot_is_legged=False, locomotion_running=False)
    ProfessionalRobotGUI._on_robot_changed(host, 'Franka_FR3')
    assert host.stopped == []
