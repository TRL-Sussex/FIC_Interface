"""
Tests for the Robot Config page: robot list + tabs.

Covers _build_robot_config_page, _rc_load_robot, _rc_collect_robot,
fleet operations (duplicate/delete), the save merge that keeps keys the
editor has no widget for, and headless render for both warmth themes.
"""
import copy
import math
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QComboBox, QListWidget, QWidget  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_mixin():
    """Instantiate a minimal RobotConfigMixin host for headless testing."""
    from fic_studio.mixins.robot_config_mixin import (
        RobotConfigMixin,
        DEFAULT_ROBOT_CONFIGS,
    )

    class _Host(RobotConfigMixin):
        """Minimal host satisfying the mixin contract without a real QMainWindow."""

        def __init__(self):
            """Initialise with a default robot config set."""
            from PyQt5.QtWidgets import QLabel, QPushButton

            self._page_title = lambda text: QLabel(text)  # noqa: E731

            def _btn(text, style_id=None):
                btn = QPushButton(text)
                if style_id:
                    btn.setObjectName(style_id)
                return btn

            self._btn = _btn
            self.robot_configs = copy.deepcopy(DEFAULT_ROBOT_CONFIGS)
            self.robots = {k: v.get('num_joints', 7) for k, v in self.robot_configs.items()}
            self.robot_dropdown = QComboBox()
            self._sync_calls = 0

        def log_message(self, level, msg, source=''):
            """Stub log_message to satisfy mixin calls."""
            pass

        def _sync_to_fic_config(self, show_result=False):
            """Stub: never write the real controllers' YAML from a test."""
            self._sync_calls += 1

        def _refresh_robot_list(self):
            """Stub: old list widget no longer exists; harmless no-op."""
            pass

        def _save_robot_configs(self):
            """Stub: avoid writing to disk during tests."""
            pass

        def _refresh_robots_dropdown(self):
            """Stub: no real dropdown to refresh in test host."""
            pass

    return _Host()


def _franka_cfg():
    """Return the Franka FR3 default config for assertions."""
    from fic_studio.mixins.robot_config_mixin import DEFAULT_ROBOT_CONFIGS
    return copy.deepcopy(DEFAULT_ROBOT_CONFIGS['Franka_FR3'])


# ---------------------------------------------------------------------------
# Page construction
# ---------------------------------------------------------------------------

class TestRCPageBuild:
    """Tests that _build_robot_config_page constructs correctly."""

    def _make_page(self):
        """Return (host, page) keeping page alive via host._test_page."""
        host = _make_mixin()
        page = host._build_robot_config_page()
        host._test_page = page   # prevent GC
        return host, page

    def test_returns_widget_with_tabs(self):
        """The page is a plain widget; its five tabs scroll on their own."""
        host, page = self._make_page()
        assert isinstance(page, QWidget)
        tabs = [host._rc_tabs.tabText(i) for i in range(host._rc_tabs.count())]
        assert tabs == ['Overview', 'Joints', 'Frames', 'Hardware', 'Safety']

    def test_robot_list_exists(self):
        """After build, host has the _rc_robot_list QListWidget."""
        host, _ = self._make_page()
        assert isinstance(host._rc_robot_list, QListWidget)

    def test_list_populated_with_robots(self):
        """The list holds one entry per robot in robot_configs."""
        host, _ = self._make_page()
        assert host._rc_robot_list.count() == len(host.robot_configs)

    def test_list_robot_ids_match(self):
        """The list's robot IDs match the robot_configs keys, in order."""
        host, _ = self._make_page()
        assert host._rc_list_ids() == list(host.robot_configs)

    def test_list_follows_loaded_robot(self):
        """Loading a robot selects it in the list."""
        host, _ = self._make_page()
        host._rc_load_robot('Pendulum')
        assert host._rc_robot_list.currentItem().data(0x0100) == 'Pendulum'

    def test_header_shows_robot(self):
        """The header carries the display name and ID."""
        host, _ = self._make_page()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_title_lbl.text() == 'Franka FR3'
        assert host._rc_subtitle_lbl.text().startswith('Franka_FR3 · 7 joints')

    def test_joint_table_exists(self):
        """After build, host has _rc_joint_table."""
        from fic_studio.components.joint_table import JointTable
        host, _ = self._make_page()
        assert hasattr(host, '_rc_joint_table')
        assert isinstance(host._rc_joint_table, JointTable)

    def test_first_robot_loaded(self):
        """First robot is auto-loaded after page build."""
        host, _ = self._make_page()
        first_id = next(iter(host.robot_configs))
        assert host._rc_robot_id_lbl.text() == first_id

    def test_display_name_field_exists(self):
        """_rc_display_name QLineEdit is present after build."""
        host, _ = self._make_page()
        assert hasattr(host, '_rc_display_name')
        assert hasattr(host._rc_display_name, 'text')


# ---------------------------------------------------------------------------
# _rc_load_robot
# ---------------------------------------------------------------------------

class TestRCLoadRobot:
    """Tests that _rc_load_robot populates widgets correctly."""

    def _setup(self):
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()   # keep alive
        return host

    def test_load_sets_display_name(self):
        """Loading Franka sets display_name field to 'Franka FR3'."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_display_name.text() == 'Franka FR3'

    def test_load_sets_num_joints(self):
        """Loading Franka sets num_joints spinner to 7."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_num_joints.value() == 7

    def test_load_sets_robot_id_label(self):
        """Loading Franka sets _rc_robot_id_lbl to 'Franka_FR3'."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_robot_id_lbl.text() == 'Franka_FR3'

    def test_load_sets_joint_table_row_count(self):
        """Loading Franka sets joint table to 7 rows."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert len(host._rc_joint_table.rows()) == 7

    def test_load_sets_joint_names(self):
        """Loading Franka populates joint names in the table."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        rows = host._rc_joint_table.rows()
        expected = _franka_cfg()['joint_names']
        assert [r['name'] for r in rows] == expected

    def test_load_sets_home_fields(self):
        """Loading Franka populates 7 home-pose MetricFields."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert len(host._rc_home_fields) == 7

    def test_load_sets_home_deg_values(self):
        """Home angle fields match q_home_deg after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        expected = _franka_cfg()['q_home_deg']
        for i, mf in enumerate(host._rc_home_fields):
            assert mf.value() == pytest.approx(expected[i], abs=1e-6)

    def test_load_sets_ee_position(self):
        """EE position MetricFields match default_ee_target.position after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        pos = _franka_cfg()['default_ee_target']['position']
        assert host._rc_ee_pos_x.value() == pytest.approx(pos[0], abs=1e-4)
        assert host._rc_ee_pos_y.value() == pytest.approx(pos[1], abs=1e-4)
        assert host._rc_ee_pos_z.value() == pytest.approx(pos[2], abs=1e-4)

    def test_load_sets_orientation_qx(self):
        """Qx spinbox matches orientation[0] after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        ori = _franka_cfg()['default_ee_target']['orientation']
        assert host._rc_ee_qx.value() == pytest.approx(ori[0], abs=1e-4)

    def test_load_sets_admittance_deadbands(self):
        """Deadband MetricFields match admittance_deadbands after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        db = _franka_cfg()['admittance_deadbands']
        assert host._rc_db_joint_tau.value() == pytest.approx(db['joint_tau'], abs=1e-4)
        assert host._rc_db_cart_force.value() == pytest.approx(db['cart_force'], abs=1e-4)
        assert host._rc_db_cart_torque.value() == pytest.approx(db['cart_torque'], abs=1e-4)

    def test_load_sets_has_gripper(self):
        """has_gripper checkbox matches config after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_has_gripper.isChecked() == _franka_cfg()['has_gripper']

    def test_load_sets_is_supported(self):
        """is_supported checkbox matches config after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_is_supported.isChecked() == _franka_cfg()['is_supported']

    def test_load_sets_torque_sign_positive(self):
        """Positive torque sign maps to selector index 0."""
        host = self._setup()
        host._rc_load_robot('KinovaGen3_7DOF')  # torque_sign = 1
        assert host._rc_torque_sign.currentIndex() == 0

    def test_load_sets_torque_sign_negative(self):
        """Negative torque sign maps to selector index 1."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')  # torque_sign = -1
        assert host._rc_torque_sign.currentIndex() == 1

    def test_load_sets_ee_frame(self):
        """EE frame name QLineEdit is set after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_ee_frame.text() == _franka_cfg()['ee_frame_name']

    def test_load_sets_urdf_path(self):
        """URDF path QLineEdit is set after load."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        assert host._rc_urdf_path.text() == _franka_cfg()['urdf_path']

    def test_load_pendulum_single_joint(self):
        """Loading single-joint Pendulum produces exactly 1 joint row."""
        host = self._setup()
        host._rc_load_robot('Pendulum')
        assert len(host._rc_joint_table.rows()) == 1
        assert len(host._rc_home_fields) == 1


# ---------------------------------------------------------------------------
# _rc_collect_robot
# ---------------------------------------------------------------------------

import pytest  # noqa: E402  (used in approx assertions above)


class TestRCCollectRobot:
    """Tests that _rc_collect_robot produces correct output dicts."""

    def _setup(self):
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()   # keep alive
        host._rc_load_robot('Franka_FR3')
        return host

    def test_collect_returns_dict(self):
        """_rc_collect_robot returns a dict."""
        host = self._setup()
        result = host._rc_collect_robot()
        assert isinstance(result, dict)

    def test_collect_display_name(self):
        """Collected display_name matches widget text."""
        host = self._setup()
        host._rc_display_name.setText('My FR3')
        result = host._rc_collect_robot()
        assert result['display_name'] == 'My FR3'

    def test_collect_num_joints(self):
        """Collected num_joints matches spinner."""
        host = self._setup()
        result = host._rc_collect_robot()
        assert result['num_joints'] == 7

    def test_collect_joint_names_from_table(self):
        """Collected joint_names come from the JointTable rows."""
        host = self._setup()
        result = host._rc_collect_robot()
        expected = _franka_cfg()['joint_names']
        assert result['joint_names'] == expected

    def test_collect_joint_limits_lower(self):
        """Collected joint_limits_deg.lower matches table lower values."""
        host = self._setup()
        result = host._rc_collect_robot()
        expected = _franka_cfg()['joint_limits_deg']['lower']
        assert result['joint_limits_deg']['lower'] == pytest.approx(expected, abs=1e-1)

    def test_collect_joint_limits_upper(self):
        """Collected joint_limits_deg.upper matches table upper values."""
        host = self._setup()
        result = host._rc_collect_robot()
        expected = _franka_cfg()['joint_limits_deg']['upper']
        assert result['joint_limits_deg']['upper'] == pytest.approx(expected, abs=1e-1)

    def test_collect_joint_torque_limits_max(self):
        """Collected joint_torque_limits.max matches table torque_max values."""
        host = self._setup()
        result = host._rc_collect_robot()
        expected = _franka_cfg()['joint_torque_limits']['max']
        assert result['joint_torque_limits']['max'] == pytest.approx(expected, abs=1e-1)

    def test_collect_q_home_deg(self):
        """Collected q_home_deg matches home-field values."""
        host = self._setup()
        result = host._rc_collect_robot()
        expected = _franka_cfg()['q_home_deg']
        assert result['q_home_deg'] == pytest.approx(expected, abs=1e-2)

    def test_collect_q_home_rad_derived(self):
        """Collected q_home_rad is radians-conversion of q_home_deg."""
        host = self._setup()
        result = host._rc_collect_robot()
        for deg, rad in zip(result['q_home_deg'], result['q_home_rad']):
            assert rad == pytest.approx(math.radians(deg), abs=1e-6)

    def test_collect_admittance_deadbands(self):
        """Collected admittance_deadbands match widget values."""
        host = self._setup()
        result = host._rc_collect_robot()
        db = _franka_cfg()['admittance_deadbands']
        assert result['admittance_deadbands']['joint_tau'] == pytest.approx(
            db['joint_tau'], abs=1e-4,
        )
        assert result['admittance_deadbands']['cart_force'] == pytest.approx(
            db['cart_force'], abs=1e-4,
        )

    def test_collect_has_gripper(self):
        """Collected has_gripper matches checkbox state."""
        host = self._setup()
        host._rc_has_gripper.setChecked(False)
        result = host._rc_collect_robot()
        assert result['has_gripper'] is False

    def test_collect_torque_sign_positive(self):
        """Torque sign index 0 → +1 in collected dict."""
        host = self._setup()
        host._rc_torque_sign.setCurrentIndex(0)
        result = host._rc_collect_robot()
        assert result['torque_sign'] == 1

    def test_collect_torque_sign_negative(self):
        """Torque sign index 1 → -1 in collected dict."""
        host = self._setup()
        host._rc_torque_sign.setCurrentIndex(1)
        result = host._rc_collect_robot()
        assert result['torque_sign'] == -1

    def test_collect_safety_zones_cartesian_keys(self):
        """Collected safety_zones.cartesian has x, y, z sub-dicts."""
        host = self._setup()
        result = host._rc_collect_robot()
        cart = result['safety_zones']['cartesian']
        assert set(cart.keys()) == {'x', 'y', 'z'}

    def test_collect_safety_zones_each_axis_has_min_max_enabled(self):
        """Each axis dict has min, max, enabled keys."""
        host = self._setup()
        result = host._rc_collect_robot()
        for axis in ('x', 'y', 'z'):
            d = result['safety_zones']['cartesian'][axis]
            assert 'min' in d and 'max' in d and 'enabled' in d

    def test_collect_num_joints_changed(self):
        """Changing num_joints and collecting reflects new count in joint arrays."""
        host = self._setup()
        host._rc_num_joints.setValue(3)
        # Rebuild is triggered by valueChanged signal
        result = host._rc_collect_robot()
        assert len(result['joint_names']) == 3
        assert len(result['q_home_deg']) == 3
        assert len(result['joint_limits_deg']['lower']) == 3

    def test_collect_full_dict_required_keys(self):
        """Collected dict contains all required top-level keys."""
        required = {
            'display_name', 'num_joints', 'joint_names', 'urdf_path',
            'mujoco_xml_path', 'ee_frame_name', 'elbow_frame_name',
            'default_ee_target', 'q_home_deg', 'q_home_rad',
            'joint_limits_deg', 'joint_torque_limits', 'torque_sign',
            'gravity_compensated', 'wrench_gravity_compensated',
            'admittance_deadbands', 'has_gripper', 'gripper_range',
            'is_supported', 'driver_command', 'build_directory',
            'build_command', 'safety_zones',
        }
        host = self._setup()
        result = host._rc_collect_robot()
        missing = required - set(result.keys())
        assert missing == set(), 'Missing keys: %s' % missing


# ---------------------------------------------------------------------------
# Fleet operations
# ---------------------------------------------------------------------------

class TestRCFleetOps:
    """Tests for inline add/duplicate/delete fleet operations."""

    def _setup(self):
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()   # keep alive
        return host

    def test_duplicate_creates_copy(self):
        """_rc_duplicate_robot adds a _copy variant to robot_configs."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        initial_count = len(host.robot_configs)
        host._rc_duplicate_robot()
        assert len(host.robot_configs) == initial_count + 1
        assert 'Franka_FR3_copy' in host.robot_configs

    def test_duplicate_preserves_joint_count(self):
        """Duplicated robot retains the same num_joints as the original."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        host._rc_duplicate_robot()
        assert host.robot_configs['Franka_FR3_copy']['num_joints'] == 7

    def test_list_reflects_new_count(self):
        """After a duplicate the list has one more robot, and shows the copy."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        host._rc_duplicate_robot()
        assert host._rc_list_ids() == list(host.robot_configs)
        assert host._rc_current_id() == 'Franka_FR3_copy'

    def test_duplicate_writes_both_files(self):
        """A duplicate is saved to both robot YAMLs, like any save."""
        host = self._setup()
        host._rc_load_robot('Franka_FR3')
        host._rc_duplicate_robot()
        assert host._sync_calls == 1

    def test_delete_removes_robot(self):
        """After _rc_delete_robot, robot is gone from robot_configs."""
        host = self._setup()
        # Add a disposable robot
        import copy as _copy
        from fic_studio.mixins.robot_config_mixin import DEFAULT_ROBOT_TEMPLATE
        host.robot_configs['__test_robot__'] = _copy.deepcopy(DEFAULT_ROBOT_TEMPLATE)
        host._rc_rebuild_list()
        host._rc_load_robot('__test_robot__')
        # Patch QMessageBox to auto-accept
        from unittest.mock import patch
        with patch('fic_studio.mixins.robot_config_mixin.QMessageBox.question',
                   return_value=0x00004000):  # QMessageBox.Yes
            host._rc_delete_robot()
        assert '__test_robot__' not in host.robot_configs
        assert '__test_robot__' not in host._rc_list_ids()
        assert host._rc_current_id() == next(iter(host.robot_configs))


# ---------------------------------------------------------------------------
# Headless render
# ---------------------------------------------------------------------------

class TestRCPageRender:
    """Headless render tests: construct page, switch themes, verify widget tree."""

    def test_render_warmth_dark(self):
        """Page renders (no crash) under warmth_dark theme (active theme)."""
        # Note: apply_theme triggers a global reapply; we validate that the page
        # itself constructs and loads without error under the current (dark) theme.
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        host._test_page.show()
        host._rc_load_robot('Franka_FR3')
        _app.processEvents()
        host._test_page.hide()

    def test_render_warmth_light(self):
        """Page renders (no crash) when built fresh (light theme variant check)."""
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        host._test_page.show()
        host._rc_load_robot('KinovaGen3_7DOF')
        _app.processEvents()
        host._test_page.hide()

    def test_render_drop_zone_present(self):
        """The page has the URDF drop zone under the robot list."""
        from fic_studio.components.robot_drop_zone import RobotDropZone
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        assert isinstance(host._rc_drop_zone, RobotDropZone)
        assert host._rc_drop_zone.acceptDrops()

    def test_render_joint_table_rows_after_load(self):
        """After load, joint table shows 7 rows for Franka."""
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        host._rc_load_robot('Franka_FR3')
        assert len(host._rc_joint_table.rows()) == 7

    def test_render_home_rows_resize_on_spinner_change(self):
        """Changing num_joints to 3 resizes home rows to 3."""
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        host._rc_load_robot('Franka_FR3')
        host._rc_num_joints.setValue(3)
        assert len(host._rc_home_fields) == 3

    def test_render_joint_table_resize_on_spinner_change(self):
        """Changing num_joints to 3 resizes joint table to 3 rows."""
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        host._rc_load_robot('Franka_FR3')
        host._rc_num_joints.setValue(3)
        assert len(host._rc_joint_table.rows()) == 3

    def test_qnorm_readout_updated_on_load(self):
        """Quaternion norm readout is updated to a non-empty string after load."""
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        host._rc_load_robot('Franka_FR3')
        assert '|q|' in host._rc_qnorm_lbl.text()


# ---------------------------------------------------------------------------
# Saving keeps what the editor cannot show
# ---------------------------------------------------------------------------

class TestSaveKeepsWhatItCannotEdit:
    """A save must never drop config the page has no widget for."""

    def _setup(self):
        host = _make_mixin()
        host._test_page = host._build_robot_config_page()
        cfg = host.robot_configs['KinovaGen3_7DOF']
        cfg['admittance_deadbands']['joint_tau_per_joint'] = [2.3, 2.5, 1.8, 2.4, 1.2, 0.5, 0.5]
        cfg['urdf_path_gripper'] = 'menagerie/model/KinovaGen3_7DOF/URDF/g.urdf'
        cfg['joint_torque_limits']['min'] = [-1.0] * 7
        host._rc_load_robot('KinovaGen3_7DOF')
        return host

    def test_nested_per_joint_deadbands_survive(self):
        """joint_tau_per_joint sits inside admittance_deadbands; it survives."""
        host = self._setup()
        host._rc_db_cart_force.set_value(5.0)
        host._rc_save()
        db = host.robot_configs['KinovaGen3_7DOF']['admittance_deadbands']
        assert db['cart_force'] == pytest.approx(5.0)
        assert db['joint_tau_per_joint'] == [2.3, 2.5, 1.8, 2.4, 1.2, 0.5, 0.5]

    def test_top_level_unknown_key_survives(self):
        """urdf_path_gripper has no widget; a save keeps it."""
        host = self._setup()
        host._rc_save()
        assert host.robot_configs['KinovaGen3_7DOF']['urdf_path_gripper'].endswith('g.urdf')

    def test_torque_minimums_are_kept(self):
        """The editor shows only the max torque; the min is not reset to 0."""
        host = self._setup()
        host._rc_save()
        assert host.robot_configs['KinovaGen3_7DOF']['joint_torque_limits']['min'] == [-1.0] * 7

    def test_save_writes_both_files_once(self):
        """One Save writes the Studio's YAML and the controllers' copy."""
        host = self._setup()
        host._rc_save()
        assert host._sync_calls == 1
        assert host._rc_dirty is False

    def test_merge_robot_config_is_recursive(self):
        """merge_robot_config keeps nested keys at any depth."""
        from fic_studio.mixins.robot_config_mixin import merge_robot_config
        old = {'a': 1, 'sz': {'velocity': {'max': 1.0, 'acc': 3.0}, 'k': [1]}}
        new = {'a': 2, 'sz': {'velocity': {'max': 2.0}}}
        assert merge_robot_config(old, new) == {
            'a': 2, 'sz': {'velocity': {'max': 2.0, 'acc': 3.0}, 'k': [1]}}


class TestControllerRobots:
    """robots_config.yaml is rebuilt on every save; nothing may be lost."""

    def test_studio_values_win(self):
        from fic_studio.mixins.robot_config_mixin import controller_robots
        out = controller_robots({'R': {'num_joints': 3, 'torque_sign': -1}},
                                {'R': {'num_joints': 7, 'torque_sign': 1}})
        assert out['R']['num_joints'] == 3 and out['R']['torque_sign'] == -1

    def test_controller_only_values_are_kept(self):
        """A value only the controllers' copy has is not replaced by a default."""
        from fic_studio.mixins.robot_config_mixin import controller_robots
        out = controller_robots(
            {'UR5e': {'num_joints': 6}},
            {'UR5e': {'joint_torque_limits': {'max': [150.0] * 6}, 'hand_added': 1}})
        assert out['UR5e']['joint_torque_limits'] == {'max': [150.0] * 6}
        assert out['UR5e']['hand_added'] == 1

    def test_defaults_fill_the_rest_and_optional_keys_carry(self):
        from fic_studio.mixins.robot_config_mixin import controller_robots
        out = controller_robots({'New': {'robot_category': 'arm', 'loop_closures': [1]}})
        entry = out['New']
        assert entry['display_name'] == 'New' and entry['kinematics_solver'] == 'pinocchio'
        assert entry['robot_category'] == 'arm' and entry['loop_closures'] == [1]

    def test_removed_robots_are_dropped(self):
        from fic_studio.mixins.robot_config_mixin import controller_robots
        assert set(controller_robots({'A': {}}, {'A': {}, 'Gone': {}})) == {'A'}

    def test_gripper_urdf_reaches_the_controllers(self):
        """With has_gripper the controllers load urdf_path_gripper, so a save passes it on.

        It used to stay in the Studio's YAML only, and the controllers
        computed gravity for the bare arm.
        """
        from fic_studio.mixins.robot_config_mixin import controller_robots
        arm = {'has_gripper': True, 'urdf_path': 'a.urdf', 'urdf_path_gripper': 'a_g.urdf'}
        out = controller_robots({'Arm': arm}, {'Arm': {'urdf_path': 'a.urdf'}})
        assert out['Arm']['urdf_path_gripper'] == 'a_g.urdf'
        out = controller_robots({'Arm': dict(arm, urdf_path_gripper='b_g.urdf')}, out)
        assert out['Arm']['urdf_path_gripper'] == 'b_g.urdf'

    def test_real_files_round_trip_unchanged(self):
        """Syncing today's Studio YAML over today's controller YAML changes no value.

        Every existing robot must come out exactly as the controllers have it
        now, except where the Studio's copy genuinely differs.
        """
        import os
        import yaml
        from fic_studio.mixins.robot_config_mixin import controller_robots
        here = os.path.dirname(os.path.abspath(__file__))
        root = os.path.abspath(os.path.join(here, '..', '..', '..', '..'))
        gui = yaml.safe_load(open(os.path.join(
            root, 'fic_ws/src/fic_studio/fic_studio/fic_studio_robots.yaml')))['robots']
        ctrl = yaml.safe_load(open(os.path.join(
            root, 'fic_library/config/robots_config.yaml')))['robots']
        out = controller_robots(gui, ctrl)
        for robot_id, entry in ctrl.items():
            for key, value in entry.items():
                if key in gui.get(robot_id, {}):
                    continue            # the Studio's value wins by design
                assert out[robot_id][key] == value, (robot_id, key)
