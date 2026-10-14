"""Headless tests for the Dashboard's grouped numeric monitoring page.

Unlike the previous version of this file, the host here calls the REAL
``_build_dashboard_page`` rather than hand-mirroring its widgets. The old
stub drifted behind the mixin and failed wholesale the moment the page
grew a collaborator; building the real page cannot drift.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio import theme  # noqa: E402
from fic_studio.mixins.dashboard_mixin import DashboardMixin  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402
import pytest  # noqa: E402

_app = QApplication.instance() or QApplication([])

ROBOT = 'KinovaGen3_7DOF'


class _FakeDropdown:
    """Minimal QComboBox stub exposing currentText."""

    def __init__(self, text):
        """Store the fixed robot name."""
        self._text = text

    def currentText(self):
        """Return the fixed robot name."""
        return self._text


class _FakeStack:
    """QStackedWidget stub; index 0 is the dashboard page."""

    def __init__(self, index=0):
        """Store the index currentIndex should report."""
        self.index = index

    def currentIndex(self):
        """Return the configured current index."""
        return self.index


class _Pose:
    """geometry_msgs/Pose stub with position and orientation."""

    class _V:
        def __init__(self, **kw):
            """Set each component passed by keyword."""
            for k, v in kw.items():
                setattr(self, k, float(v))

    def __init__(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Build a pose from position and quaternion components."""
        self.position = self._V(x=x, y=y, z=z)
        self.orientation = self._V(x=qx, y=qy, z=qz, w=qw)


def _wrench(fx=0.0, fy=0.0, fz=0.0, tx=0.0, ty=0.0, tz=0.0):
    """Build a wrench in JointStateMixin.wrench_for()'s real shape.

    That is a plain (Fx, Fy, Fz, Tx, Ty, Tz) tuple, NOT a
    geometry_msgs/Wrench. This stub used to return a Wrench-like object,
    which let the dashboard read ``w.force`` in tests while the real GUI
    raised AttributeError on the first live sample.
    """
    return (float(fx), float(fy), float(fz), float(tx), float(ty), float(tz))


class _Host(DashboardMixin):
    """Minimal host that builds and drives the real dashboard page."""

    def __init__(self):
        """Wire the attributes the page builder and refreshers read."""
        if not theme.current_name():
            theme.apply_theme(self, 'warmth_dark')

        self.main_stack = _FakeStack(0)
        self.robot_dropdown = _FakeDropdown(ROBOT)
        self.robots = {ROBOT: 7}
        self.robot_configs = {
            ROBOT: {
                'num_joints': 7,
                'joint_limits_deg': {'lower': -180, 'upper': 180},
                'joint_torque_limits': {'min': -39.0, 'max': 39.0},
            },
        }

        self.simulation_running = True
        self.fic_running = False
        self.driver_running = False
        self.controller_running = False

        self._pos = {}
        self._cmd = {}
        self._vel = {}
        self._eff = {}
        self._pose = None
        self._wrench = None

        self.dashboard_page = self._build_dashboard_page()

    # ── Telemetry accessors (JointStateMixin's contract) ──────────────
    def joint_positions_for(self, rid):
        """Return synthetic joint positions."""
        return list(self._pos.get('r', []))

    def joint_commanded_for(self, rid):
        """Return synthetic commanded joint positions."""
        return list(self._cmd.get('r', []))

    def joint_velocity_for(self, rid):
        """Return synthetic joint velocities."""
        return list(self._vel.get('r', []))

    def joint_effort_for(self, rid):
        """Return synthetic joint efforts."""
        return list(self._eff.get('r', []))

    def ee_pose_for(self, rid):
        """Return the synthetic EE pose."""
        return self._pose

    def wrench_for(self, rid):
        """Return the synthetic wrench."""
        return self._wrench

    def sim_joint_by_name(self, rid):
        """No named sim joints in the stub."""
        return {}

    def is_page_visible(self, name):
        """Dashboard is page 0 in this stub's fake stack."""
        return name == 'dashboard' and self.main_stack.index == 0

    def robot_target(self, rid=None):
        """Report the simulation target."""
        return 'simulation'

    def joint_update_count(self, rid=None):
        """Report a fixed update count."""
        return 100

    def joint_update_time(self, rid=None):
        """Report no update timestamp."""
        return None

    # ── Process controls the Base card wires buttons to ───────────────
    def _start_simulation(self):
        """No-op process control."""

    def _stop_simulation(self):
        """No-op process control."""

    def _start_fic_simulation(self):
        """No-op process control."""

    def _stop_fic_simulation(self):
        """No-op process control."""

    def _start_driver(self):
        """No-op process control."""

    def _stop_driver(self):
        """No-op process control."""

    def log_message(self, level, msg, source=''):
        """No-op log."""


@pytest.fixture()
def host():
    """Return a freshly built dashboard host."""
    return _Host()


# ── Structure ────────────────────────────────────────────────────────────

def test_page_builds(host):
    """The dashboard page builds without raising."""
    assert host.dashboard_page is not None


def test_overview_and_detailed_tabs_exist(host):
    """The page offers exactly the Overview and Detailed tabs."""
    labels = [host._dash_tabs.tabText(i) for i in range(host._dash_tabs.count())]
    assert labels == ['Overview', 'Detailed']


def test_joint_table_has_one_row_per_joint(host):
    """A 7-DOF robot tables seven joint rows."""
    assert host._dash_joint_table.row_count() == 7


def test_joint_table_columns_match_kinova_overview(host):
    """Overview columns carry Kinova's position/velocity/torque plus tracking."""
    assert host._dash_joint_table.header_labels() == [
        'commanded', 'position', 'error', 'velocity', 'torque',
    ]


def test_ee_table_rows(host):
    """The end-effector block pairs commanded against actual, as Kinova does."""
    assert host._dash_ee_table.row_count() == 4


# ── Refresh ──────────────────────────────────────────────────────────────

def test_absent_telemetry_shows_placeholder(host):
    """With no telemetry every joint cell reads '--', never a fabricated 0."""
    host._refresh_dashboard_tiles()
    assert host._dash_joint_table.cell_text('J1', 'position') == '--'


def test_joint_position_rendered_in_degrees(host):
    """Joint positions arrive in radians and table in degrees."""
    import math
    host._pos['r'] = [math.radians(12.0)] + [0.0] * 6
    host._refresh_dashboard_tiles()
    assert host._dash_joint_table.cell_text('J1', 'position') == '12.00'


def test_joint_tracking_error_is_commanded_minus_position(host):
    """The error column is commanded - position, in degrees."""
    import math
    host._cmd['r'] = [math.radians(12.5)] + [0.0] * 6
    host._pos['r'] = [math.radians(12.0)] + [0.0] * 6
    host._refresh_dashboard_tiles()
    assert host._dash_joint_table.cell_text('J1', 'error') == '0.50'


def test_ee_pose_rendered(host):
    """The actual EE position populates the pose row."""
    host._pose = _Pose(0.412, -0.008, 0.503)
    host._refresh_dashboard_tiles()
    assert host._dash_ee_table.cell_text('pose', 'x') == '0.412'


def test_ee_orientation_rendered_as_euler(host):
    """Orientation tables as the x-y-z extrinsic triple Kinova reports."""
    import math
    s = math.sin(math.radians(45.0))
    c = math.cos(math.radians(45.0))
    host._pose = _Pose(0.0, 0.0, 0.0, qx=0.0, qy=0.0, qz=s, qw=c)
    host._refresh_dashboard_tiles()
    assert host._dash_ee_table.cell_text('pose', 'oz') == '90.000'


def test_wrench_force_rendered(host):
    """External tool wrench force populates its row."""
    host._wrench = _wrench(fx=2.1, fy=0.0, fz=8.4)
    host._refresh_dashboard_tiles()
    assert host._dash_wrench_table.cell_text('force', 'z') == '8.40'


def test_detailed_tab_renders_wrench(host):
    """The Detailed tab reads the same wrench tuple, magnitude included."""
    host._dash_tabs.setCurrentIndex(1)
    host._wrench = _wrench(fx=3.0, fy=4.0, fz=0.0, tz=0.5)
    host._refresh_dashboard_tiles()
    assert host._dash_wrench_detail_table.cell_text('force', 'magnitude') == '5.000'
    assert host._dash_wrench_detail_table.cell_text('torque', 'z') == '0.500'


def test_contact_indicator_reflects_force_magnitude(host):
    """A large force magnitude flips the contact indicator on."""
    host._wrench = _wrench(fx=0.0, fy=0.0, fz=20.0)
    host._refresh_dashboard_tiles()
    assert 'contact' in host._dash_contact_lbl.text()


def test_base_reports_running_processes(host):
    """The Base group reports which processes are up."""
    host._refresh_dashboard_tiles()
    assert host._dash_session_rows['SIM'].text() == 'running'
    assert host._dash_session_rows['FIC'].text() == 'stopped'


def test_safety_warns_near_joint_limit(host):
    """A joint close to its configured limit trips the safety row."""
    import math
    host._pos['r'] = [math.radians(179.0)] + [0.0] * 6
    host._refresh_dashboard_tiles()
    assert host._dash_safety_rows['joints'].ok is False


def test_safety_ok_within_range(host):
    """A joint well inside its limits leaves the safety row clear."""
    import math
    host._pos['r'] = [math.radians(10.0)] + [0.0] * 6
    host._refresh_dashboard_tiles()
    assert host._dash_safety_rows['joints'].ok is True


# ── Refresh gating ───────────────────────────────────────────────────────

def test_refresh_skipped_when_dashboard_not_visible(host):
    """Off the dashboard page the refresh returns before touching widgets."""
    import math
    host.main_stack.index = 5
    host._pos['r'] = [math.radians(12.0)] + [0.0] * 6
    host._refresh_dashboard_tiles()
    assert host._dash_joint_table.cell_text('J1', 'position') == '--'


def test_hidden_tab_is_not_refreshed(host):
    """Only the visible tab refreshes; the Detailed table stays untouched."""
    import math
    host._eff['r'] = [5.0] + [0.0] * 6
    host._pos['r'] = [math.radians(12.0)] + [0.0] * 6
    host._refresh_dashboard_tiles()
    assert host._dash_joint_detail_table.cell_text('J1', 'position') == '--'


def test_detailed_tab_refreshes_when_selected(host):
    """Selecting Detailed makes its table the one that updates."""
    import math
    host._dash_tabs.setCurrentIndex(1)
    host._pos['r'] = [math.radians(12.0)] + [0.0] * 6
    host._refresh_dashboard_tiles()
    assert host._dash_joint_detail_table.cell_text('J1', 'position') == '12.00'
