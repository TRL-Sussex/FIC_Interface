"""The global control bar that replaced the Robot Control page.

Kinova's robot control panel sits on every screen and slides panels up
from the bottom. Here the eight FIC modes collapse into two expanding
menus (Joint, Cart), each owning its four modes, its mode-appropriate
modifiers and its config selection.
"""
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QComboBox  # noqa: E402

from fic_studio.mixins.control_bar_mixin import ControlBarMixin  # noqa: E402
from fic_studio.mixins.robot_control_mixin import RobotControlMixin  # noqa: E402

_app = QApplication.instance() or QApplication([])


def _node():
    """Return a node stub that records published modes."""
    n = types.SimpleNamespace()
    n.station_id = 1
    n.station_robot_id = 'robot1'
    n._active_robots = [1]
    n.published_modes = []
    n.publish_mode = lambda m: n.published_modes.append(m)
    n.published_hpmp = []
    n.publish_hpmp_enable = lambda on, robot_id=None: n.published_hpmp.append((on, len(n.published_modes)))
    n.published_wrenches = []
    n.publish_desired_wrench = lambda w, rid=None: n.published_wrenches.append((list(w), rid))
    n._resolve_rid = lambda rid=None: rid or 'robot1'
    n.add_robot = lambda i: 'robot%d' % i
    return n


def _configs(category='manipulator', has_gripper=True):
    return {
        'Franka_FR3': {
            'num_joints': 7,
            'robot_category': category,
            'has_gripper': has_gripper,
            'joint_limits_deg': {'lower': [-170.0] * 7, 'upper': [170.0] * 7},
            'default_ee_target': {'position': [0.38, 0.0, 0.432]},
        }
    }


class _FakeSequences:
    """Stand-in for the sequences extension's public API."""

    def __init__(self):
        self.played = []
        self.stopped = 0
        self.captured = []
        self.is_playing = False

    def list_sequences(self):
        return ['pick_demo', 'home_loop']

    def waypoint_count(self, name):
        return 4

    def play(self, name, loop=False):
        self.played.append((name, loop))

    def stop(self):
        self.stopped += 1

    def capture(self, kind, into=None):
        self.captured.append((kind, into))


class _Host(ControlBarMixin, RobotControlMixin):
    """Minimal host that builds the real control bar."""

    def __init__(self, category='manipulator', has_gripper=True, sequences=None):
        self.node = _node()
        self.robot_configs = _configs(category, has_gripper)
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
        self._sequences = sequences
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
        """Return the fake sequences extension, if configured."""
        return self._sequences if ext_id == 'sequences' else None

    def robot_target(self, rid=None):
        """Report the simulation target."""
        return 'simulation'


class TestBarComposition:
    """The bar carries the agreed slots."""

    def test_bar_has_every_slot(self):
        """Joint, Cart, Gripper, Home, Reset, Snapshot and Quick are present."""
        h = _Host()
        for key in ('joint', 'cart', 'gripper', 'home',
                    'snapshot', 'quick'):
            assert h.control_bar.button(key) is not None, key

    def test_home_opens_no_panel(self):
        """Home is a direct action, not a panel button."""
        assert _Host().control_bar.button('home').isCheckable() is False

    def test_no_reset_button(self):
        """Reset was dropped from the bar."""
        assert _Host().control_bar.button('reset') is None


class TestModeMenus:
    """Opening a menu must not command the robot."""

    def test_opening_joint_panel_publishes_nothing(self):
        """Revealing the panel is not a mode change."""
        h = _Host()
        h.control_bar.toggle('joint')
        assert h.node.published_modes == []

    def test_clicking_a_mode_row_publishes_that_mode(self):
        """Joint row 3 publishes mode 3 (Joint Admittance)."""
        h = _Host()
        h._cb_mode_buttons['joint'][2].click()
        assert h.node.published_modes == [3]

    def test_cart_rows_publish_modes_five_to_eight(self):
        """Cart row 1 publishes mode 5 (Cartesian Impedance), the H-PMP toggle just before it."""
        h = _Host()
        h._cb_mode_buttons['cart'][0].click()
        assert h.node.published_modes == [5]
        assert h.node.published_hpmp == [(True, 0)]

    def test_each_menu_offers_four_modes(self):
        """Eight modes split into two menus of four."""
        h = _Host()
        assert len(h._cb_mode_buttons['joint']) == 4
        assert len(h._cb_mode_buttons['cart']) == 4

    def test_joint_menu_owns_the_admittance_planner(self):
        """The joint-only modifier lives in the joint panel."""
        h = _Host()
        assert h._rc_admittance_planner_toggle is not None

    def test_cart_menu_owns_the_nullspace_modifiers(self):
        """Postural, Elbow and H-PMP live in the cart panel."""
        h = _Host()
        for attr in ('_rc_postural_toggle', '_rc_elbow_toggle',
                     '_rc_hpmp_toggle'):
            assert getattr(h, attr) is not None, attr

    def test_cart_menu_owns_the_desired_wrench(self):
        """The Wrench button opens an editor whose Apply and Zero publish."""
        h = _Host()
        h._rc_desired_wrench_btn.click()
        dlg = h._desired_wrench_dialog
        assert dlg.values() == [0.0] * 6
        dlg.show_values([2.0, 0.0, -3.5, 0.0, 0.25, 0.0])
        dlg.host._publish_desired_wrench(dlg.values(), dlg.rid)
        assert h.node.published_wrenches[-1] == ([2.0, 0.0, -3.5, 0.0, 0.25, 0.0], 'robot1')
        assert h.desired_wrench_for('robot1') == [2.0, 0.0, -3.5, 0.0, 0.25, 0.0]
        dlg.host._publish_desired_wrench([0.0] * 6, dlg.rid)
        assert h.desired_wrench_for('robot1') == [0.0] * 6
        dlg.close()

    def test_desired_wrench_refuses_bad_values(self):
        """Six finite numbers only; nothing is published otherwise."""
        h = _Host()
        for bad in ([1.0, 2.0], [float('inf')] + [0.0] * 5):
            try:
                h._publish_desired_wrench(bad)
            except ValueError:
                pass
            else:
                raise AssertionError('accepted %r' % (bad,))
        assert h.node.published_wrenches == []

    def test_each_menu_has_a_config_selector(self):
        """Both menus let you pick the config profile."""
        h = _Host()
        assert h._cb_profile_combos['joint'] is not None
        assert h._cb_profile_combos['cart'] is not None


class TestAvailability:
    """Buttons disable themselves rather than failing on click."""

    def test_cart_disabled_for_legged_robots(self):
        """kinematics_solver: none means Cartesian modes cannot run."""
        h = _Host(category='legged')
        assert h.control_bar.button('cart').isEnabled() is False

    def test_cart_enabled_for_manipulators(self):
        """A normal arm keeps its Cartesian menu."""
        assert _Host().control_bar.button('cart').isEnabled() is True

    def test_gripper_hidden_without_one(self):
        """A robot with has_gripper: false shows no gripper button.

        isHidden, not isVisible: offscreen Qt reports every widget of an
        unshown top level as not visible, so isVisible would pass here
        whatever the code did.
        """
        h = _Host(has_gripper=False)
        assert h.control_bar.button('gripper').isHidden() is True

    def test_gripper_shown_when_present(self):
        """A robot with a gripper keeps the button."""
        assert _Host().control_bar.button('gripper').isHidden() is False


class TestQuickActions:
    """Quick Actions and Snapshot are backed by the sequences extension."""

    def test_disabled_without_the_extension(self):
        """No extension means the buttons explain themselves and do nothing."""
        h = _Host(sequences=None)
        assert h.control_bar.button('quick').isEnabled() is False
        assert 'equences' in h.control_bar.button('quick').toolTip()

    def test_snapshot_disabled_without_the_extension(self):
        """Snapshot records into a sequence, so it needs the same extension."""
        h = _Host(sequences=None)
        assert h.control_bar.button('snapshot').isEnabled() is False

    def test_enabled_with_the_extension(self):
        """With sequences loaded both buttons are live."""
        h = _Host(sequences=_FakeSequences())
        assert h.control_bar.button('quick').isEnabled() is True
        assert h.control_bar.button('snapshot').isEnabled() is True

    def test_sequence_dropdown_lists_saved_sequences(self):
        """The Quick Actions panel offers the saved sequences."""
        h = _Host(sequences=_FakeSequences())
        items = [h._cb_sequence_combo.itemText(i)
                 for i in range(h._cb_sequence_combo.count())]
        assert items == ['pick_demo', 'home_loop']

    def test_play_delegates_to_the_extension(self):
        """Play does not reimplement playback; it calls the extension."""
        seq = _FakeSequences()
        h = _Host(sequences=seq)
        h._cb_play_sequence()
        assert seq.played == [('pick_demo', False)]

    def test_stop_delegates_to_the_extension(self):
        """Stop likewise delegates."""
        seq = _FakeSequences()
        h = _Host(sequences=seq)
        h._cb_stop_sequence()
        assert seq.stopped == 1

    def test_snapshot_captures_into_the_selected_sequence(self):
        """Snapshot appends a waypoint to the chosen sequence."""
        seq = _FakeSequences()
        h = _Host(sequences=seq)
        h._cb_capture_snapshot()
        assert seq.captured == [('cartesian', 'pick_demo')]


class TestStatusReadout:
    """The always-visible readout mirrors Kinova's mode indicator."""

    def test_reports_mode_and_target(self):
        """The readout names the active mode and where it is going."""
        h = _Host()
        h._cb_refresh_status()
        text = h._cb_status_lbl.text()
        assert 'Joint Impedance' in text
        assert 'SIM' in text          # simulation / hardware / vr -> SIM / HW / VR


class _LateSequences(_FakeSequences):
    """The real extension loads its file after the bar is built; it also has the new API."""

    def __init__(self):
        super().__init__()
        self.names = []
        self.runs, self.saved, self.steps, self.speeds = [], [], [], []
        self._speed = 1.0

    def list_sequences(self):
        return list(self.names)

    def new_sequence(self, name):
        self.names.append(name)
        return name

    def step(self, name):
        self.steps.append(name)
        return 0

    def speed(self):
        return self._speed

    def set_speed(self, v):
        self.speeds.append(v)
        self._speed = v

    def target_is_hardware(self):
        return False

    def run_trajectory(self, *args, hardware_ok=False):
        self.runs.append(args)
        return 17.0

    def save_trajectory(self, name, kind, size, plane):
        self.saved.append((name, kind, size, plane))
        self.names.append(name)
        return name


class TestQuickActionsRework:
    def test_the_list_fills_when_the_drawer_opens(self):
        """The bar is built before the sequences are loaded: opening Quick Actions refreshes it."""
        seq = _LateSequences()
        h = _Host(sequences=seq)
        assert h._cb_sequence_combo.count() == 0
        seq.names[:] = ['a', 'b']
        h._cb_on_panel_changed('quick')
        assert [h._cb_sequence_combo.itemText(i) for i in range(2)] == ['a', 'b']

    def test_new_step_speed_and_trajectory_go_to_the_extension(self, monkeypatch):
        from fic_studio.mixins import control_bar_mixin as cbm
        seq = _LateSequences()
        h = _Host(sequences=seq)
        monkeypatch.setattr(cbm.QInputDialog, 'getText', staticmethod(lambda *a, **k: ('demo', True)))
        h._cb_new_sequence()
        assert h._cb_selected_sequence() == 'demo'
        h._cb_step_sequence()
        assert seq.steps == ['demo']
        h._cb_speed.setValue(2.0)
        assert seq.speeds == [2.0]
        h._cb_traj_kind.setCurrentIndex(1)              # circle
        h._cb_run_trajectory()
        assert seq.runs == [('circle', 0.05, 8.0, 2, 'xy')]
        monkeypatch.setattr(cbm.QInputDialog, 'getText', staticmethod(lambda *a, **k: ('circle_50mm', True)))
        h._cb_save_trajectory()
        assert seq.saved == [('circle_50mm', 'circle', 0.05, 'xy')] and h._cb_selected_sequence() == 'circle_50mm'

    def test_extensions_add_quick_actions(self):
        h = _Host(sequences=_LateSequences())
        ran = []
        h._quick_actions = {'mujoco_playground.pick': {'label': 'Pick', 'run': lambda c: ran.append(c) or 'Picking ' + c,
                                                       'choices': lambda: ['red_cube', 'blue_cube'], 'hint': '', 'button': 'Pick'}}
        h._cb_rebuild_quick_actions()
        combo = h._cb_ext_choice['mujoco_playground.pick']
        assert [combo.itemText(i) for i in range(combo.count())] == ['red_cube', 'blue_cube']
        combo.setCurrentText('blue_cube')
        h._cb_run_quick_action('mujoco_playground.pick')
        assert ran == ['blue_cube'] and h._cb_sequence_status.text() == 'Picking blue_cube'
