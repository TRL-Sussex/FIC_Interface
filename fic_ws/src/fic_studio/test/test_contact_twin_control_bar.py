"""Tests for the Contact panel, its ground-truth wiring, and the readout.

The viewer launch/stop toggle itself now lives in the top toolbar (see
test_toolbar_launch.py); this file covers what is still in the control
bar: the Contact panel (Calibrate, ground truth), the toggle *method*
_cb_on_contact_twin (unchanged, just called from a different button
now), and the live link readout.

The readout must show four visually distinct states: '--' (no message
ever arrived), 'UNCAL' (link_index -1: gated off, no statement
possible), 'no contact' (link_index 0: a genuine measurement) and a
link name with confidence (link_index 1..n). -1 and 0 must never
collapse into the same text.
"""
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QComboBox  # noqa: E402

from fic_studio.mixins.control_bar_mixin import ControlBarMixin  # noqa: E402
from fic_studio.mixins.robot_control_mixin import RobotControlMixin  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _FakeParamClient:
    """Records SetParameters calls instead of hitting a real ROS service."""

    def __init__(self, srv_type, name, available=True):
        self.srv_type = srv_type
        self.name = name
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec=1.0):
        """Report whether the (fake) service is up."""
        return self.available

    def call_async(self, request):
        """Record the request instead of sending it anywhere."""
        self.requests.append(request)


def _node():
    """Return a node stub that records published modes and param clients."""
    n = types.SimpleNamespace()
    n.station_id = 1
    n.station_robot_id = 'robot1'
    n._active_robots = [1]
    n.published_modes = []
    n.publish_mode = lambda m: n.published_modes.append(m)
    n.add_robot = lambda i: 'robot%d' % i
    n.param_clients = []

    def _create_client(srv_type, name):
        client = _FakeParamClient(srv_type, name)
        n.param_clients.append(client)
        return client
    n.create_client = _create_client
    return n


class _Host(ControlBarMixin, RobotControlMixin):
    """Control-bar host that also carries the contact telemetry API."""

    def __init__(self):
        self.node = _node()
        self.robot_configs = {
            'Franka_FR3': {
                'num_joints': 7,
                'robot_category': 'manipulator',
                'has_gripper': True,
                'joint_limits_deg': {'lower': [-170.0] * 7, 'upper': [170.0] * 7},
                'default_ee_target': {'position': [0.38, 0.0, 0.432]},
            }
        }
        self.robots = {'Franka_FR3': 7}
        self.robot_dropdown = QComboBox()
        self.robot_dropdown.addItem('Franka_FR3')
        self.control_target = 'simulation'
        self._robot_targets = {}
        self._rc_active_rid = 'robot1'
        self._rc_joint_fields = []
        self._rc_pose_fields = {}
        self._rc_selected_kind = 'joint'
        self._rc_selected_mode_idx = 0
        self._estimates = {}
        self.contact_twin_running = False
        self.twin_calls = []
        self.control_bar = self._build_control_bar()

    def log_message(self, level, msg, source=''):
        """No-op log."""

    def joint_positions_for(self, rid):
        """No live telemetry."""
        return []

    def ee_pose_for(self, rid):
        """No live pose."""
        return None

    def get_extension(self, ext_id):
        """No extensions loaded in this stub."""
        return None

    def robot_target(self, rid=None):
        """Report the simulation target."""
        return 'simulation'

    def contact_estimate_for(self, rid):
        """Return the stub's canned estimate for rid, or None."""
        return self._estimates.get(rid)

    def _start_contact_twin(self):
        """Record a start call instead of spawning a real process."""
        self.twin_calls.append('start')
        self.contact_twin_running = True

    def _stop_contact_twin(self):
        """Record a stop call instead of killing a real process."""
        self.twin_calls.append('stop')
        self.contact_twin_running = False


class TestContactTwinButtonAndPanel:
    """The bar carries a Contact panel; the launch toggle moved out.

    The launch/stop toggle itself now lives in the top toolbar
    (main_window_pro's tb_viewer_btn) -- see test_toolbar_launch.py.
    _cb_on_contact_twin (this file) is what that button calls; only its
    own control-bar action button moved out.
    """

    def test_bar_has_no_twin_action_but_keeps_the_contact_panel(self):
        """The launch toggle is gone from here; Calibrate/ground-truth stay."""
        h = _Host()
        assert h.control_bar.button('twin') is None
        assert h.control_bar.button('contact') is not None

    def test_contact_twin_toggle_starts_the_viewer_when_stopped(self):
        """The toggle method itself is unchanged -- only its caller moved."""
        h = _Host()
        h._cb_on_contact_twin()
        assert h.twin_calls == ['start']
        assert h.contact_twin_running is True

    def test_click_stops_the_twin_when_running(self):
        """Calling it again while running stops the viewer."""
        h = _Host()
        h.contact_twin_running = True
        h._cb_on_contact_twin()
        assert h.twin_calls == ['stop']
        assert h.contact_twin_running is False

    def test_contact_panel_lists_one_entry_per_link_plus_none(self):
        """The ground-truth combo offers 'none' plus every arm link."""
        h = _Host()
        combo = h._cb_gt_combo
        items = [combo.itemText(i) for i in range(combo.count())]
        assert items[0] == 'none'
        assert len(items) == 8


class TestGroundTruthWiring:
    """Selecting a link pushes link_truth to the contact node's own
    parameter service -- never by reaching into the node object."""

    def test_selecting_a_link_sets_link_truth_on_the_right_node(self):
        """The combo targets this station's contact node by name."""
        h = _Host()
        h._cb_gt_combo.setCurrentText('ForeArm_Link')

        assert len(h.node.param_clients) == 1
        client = h.node.param_clients[0]
        assert client.name == 'contact_localization_1/set_parameters'
        assert len(client.requests) == 1
        param = client.requests[0].parameters[0]
        assert param.name == 'link_truth'
        assert param.value.string_value == 'ForeArm_Link'

    def test_none_is_sent_explicitly_not_silently_skipped(self):
        """'none' (no ground truth declared) is its own real message.

        It must never be confused with the operator declaring "no
        contact" (link 0) -- see ContactNode.set_link_truth.
        """
        h = _Host()
        h._cb_gt_combo.setCurrentText('ForeArm_Link')
        h._cb_gt_combo.setCurrentText('none')

        last = h.node.param_clients[-1].requests[-1]
        assert last.parameters[0].value.string_value == 'none'

    def test_selecting_before_the_node_is_up_logs_a_warning(self):
        """An offline contact node must not raise -- just warn and return."""
        h = _Host()
        real_create = h.node.create_client

        def _offline(srv_type, name):
            client = real_create(srv_type, name)
            client.available = False
            return client
        h.node.create_client = _offline

        h._cb_gt_combo.setCurrentText('ForeArm_Link')
        assert h.node.param_clients[0].requests == []


class TestContactReadoutStates:
    """The four states must never collapse into one another."""

    def test_no_message_shows_double_dash(self):
        """No {rid}_contact_estimate message yet -> '--', not 'UNCAL'."""
        h = _Host()
        h._cb_refresh_status()
        assert h._cb_contact_lbl.text() == '--'

    def test_link_index_minus_one_shows_uncal(self):
        """Gated off / uncalibrated -> 'UNCAL', not '--' or 'no contact'."""
        h = _Host()
        h._estimates['robot1'] = {
            'link_index': -1, 'link_name': '', 'confidence': 0.0,
            'calibrated': False,
        }
        h._cb_refresh_status()
        assert h._cb_contact_lbl.text() == 'UNCAL'

    def test_link_index_zero_shows_no_contact(self):
        """A genuine zero reading -> 'no contact', never '--' or 'UNCAL'."""
        h = _Host()
        h._estimates['robot1'] = {
            'link_index': 0, 'link_name': '', 'confidence': 0.95,
            'calibrated': True,
        }
        h._cb_refresh_status()
        assert h._cb_contact_lbl.text() == 'no contact'

    def test_a_contacted_link_shows_its_name_and_confidence_percent(self):
        """1..n reports the trimmed link name and rounded confidence."""
        h = _Host()
        h._estimates['robot1'] = {
            'link_index': 3, 'link_name': 'ForeArm_Link', 'confidence': 0.873,
            'calibrated': True,
        }
        h._cb_refresh_status()
        assert h._cb_contact_lbl.text() == 'ForeArm 87%'

    def test_all_four_states_are_textually_distinct(self):
        """-1 and 0 (and 'no data') must never share the same text."""
        h = _Host()
        h._cb_refresh_status()
        no_data = h._cb_contact_lbl.text()

        h._estimates['robot1'] = {
            'link_index': -1, 'link_name': '', 'confidence': 0.0,
            'calibrated': False,
        }
        h._cb_refresh_status()
        uncal = h._cb_contact_lbl.text()

        h._estimates['robot1'] = {
            'link_index': 0, 'link_name': '', 'confidence': 1.0,
            'calibrated': True,
        }
        h._cb_refresh_status()
        no_contact = h._cb_contact_lbl.text()

        h._estimates['robot1'] = {
            'link_index': 1, 'link_name': 'Shoulder_Link', 'confidence': 0.5,
            'calibrated': True,
        }
        h._cb_refresh_status()
        contacted = h._cb_contact_lbl.text()

        assert len({no_data, uncal, no_contact, contacted}) == 4
