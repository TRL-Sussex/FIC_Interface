"""Logic tests for the redesigned toolbar launch cluster."""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QPushButton  # noqa: E402

from fic_studio.main_window_pro import ProfessionalRobotGUI  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _Stub:
    """Minimal stand-in carrying just the attributes the launch logic reads."""

    control_target = 'simulation'
    simulation_running = False
    fic_running = False
    driver_running = False
    controller_running = False
    vr_relay_running = False

    _tb_session_running = ProfessionalRobotGUI._tb_session_running
    _tb_toggle_start = ProfessionalRobotGUI._tb_toggle_start

    def __init__(self):
        self.dispatched = []

    def _dash_toggle_sim(self):
        self.dispatched.append('sim')

    def _dash_toggle_hw(self):
        self.dispatched.append('hw')

    def _toggle_vr(self):
        self.dispatched.append('vr')

    def _update_status(self):
        pass


def test_session_running_per_target():
    """_tb_session_running reflects the right flags for each target."""
    s = _Stub()
    s.control_target = 'hardware'
    assert s._tb_session_running() is False
    s.driver_running = True
    assert s._tb_session_running() is True
    s.control_target = 'simulation'
    assert s._tb_session_running() is False
    s.fic_running = True
    assert s._tb_session_running() is True


def test_start_dispatches_by_target():
    """_tb_toggle_start routes to the combined launcher for the active target."""
    s = _Stub()
    s.control_target = 'simulation'
    s._tb_toggle_start()
    s.control_target = 'hardware'
    s._tb_toggle_start()
    s.control_target = 'vr'
    s._tb_toggle_start()
    assert s.dispatched == ['sim', 'hw', 'vr']


class _ToolbarRefreshStub:
    """Stand-in carrying just the widgets _refresh_toolbar_launch touches."""

    control_target = 'simulation'
    simulation_running = False
    fic_running = False
    driver_running = False
    controller_running = False
    vr_relay_running = False

    _tb_session_running = ProfessionalRobotGUI._tb_session_running
    _tb_target_styles = ProfessionalRobotGUI._tb_target_styles
    _tb_startstop_styles = ProfessionalRobotGUI._tb_startstop_styles
    _refresh_toolbar_launch = ProfessionalRobotGUI._refresh_toolbar_launch

    def __init__(self):
        self._tb_target_btns = {}
        self._tb_start_btn = QPushButton()
        self.robot_dropdown = QPushButton()
        self.tb_viewer_btn = QPushButton()


def test_viewer_button_is_visible_on_every_target():
    """The toolbar Viewer button stays visible whatever the control target is.

    It was briefly gated on ``control_target == 'hardware'``, but
    ``control_target`` defaults to 'simulation' (robot_control_mixin), so a
    fresh launch hid the button entirely and the feature was
    indistinguishable from one that had never been installed -- reported
    from a real robot session.

    Hiding it also had no safety value: the viewer works in simulation too,
    because fic_simulation drives the same joint-admittance path and
    publishes ``{rid}_admittance_offset``.
    """
    s = _ToolbarRefreshStub()
    for target in ('simulation', 'hardware', 'vr'):
        s.control_target = target
        s._refresh_toolbar_launch()
        assert s.tb_viewer_btn.isVisible() is True, target


def test_refresh_toolbar_launch_tolerates_a_missing_viewer_button():
    """A host without tb_viewer_btn yet must not crash on refresh."""
    s = _ToolbarRefreshStub()
    del s.tb_viewer_btn
    s.control_target = 'hardware'
    s._refresh_toolbar_launch()
