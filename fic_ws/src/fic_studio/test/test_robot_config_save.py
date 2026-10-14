"""
Tests for the Robot Config page's save state, fleet ops and path checks.

Verifies:
- The save indicator turns amber when the controllers' YAML is older than
  the Studio's, or when there are unsaved edits.
- It is quiet when everything is saved (or the controllers' file is absent).
- _rc_check_sync_state returns False when deployed YAML does not exist.
- _rc_check_sync_state returns True when GUI YAML has a newer mtime.
- Dirty flag is set when a field is edited and cleared after load/save.
- Picking another robot in the list prompts when the dirty flag is set.
- Path-exists indicator applies error style for missing paths.
- Path-exists indicator applies neutral style for empty or existing paths.
- _rc_collect_robot joint arrays match the JointTable after num_joints change.
- _rc_save writes both YAMLs (calls _sync_to_fic_config) and refreshes
  the indicator.
"""
import copy
import types
import os
import tempfile
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QComboBox  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_mixin(save_stub=True):
    """
    Instantiate a minimal RobotConfigMixin host for headless testing.

    Parameters
    ----------
    save_stub : bool
        When True, ``_save_robot_configs`` is stubbed to avoid disk writes.

    """
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
            self.robots = {
                k: v.get('num_joints', 7) for k, v in self.robot_configs.items()
            }
            self.robot_dropdown = QComboBox()
            self._sync_calls = 0  # counts _sync_to_fic_config calls

            # ROS node stub. _rc_publish_hot_config iterates node.pubs; an
            # empty publisher table is the honest "no ROS running" state and
            # makes that loop a no-op rather than an AttributeError.
            self.node = types.SimpleNamespace(pubs={})

        def log_message(self, level, msg, source=''):
            """Stub log_message to satisfy mixin calls."""

        def _refresh_robot_list(self):
            """Stub: old list widget no longer exists; harmless no-op."""

        def _save_robot_configs(self):
            """Stub: avoid writing to disk during tests."""

        def _refresh_robots_dropdown(self):
            """Stub: no real dropdown to refresh in test host."""

        def _sync_to_fic_config(self):
            """Stub: record calls without writing files."""
            self._sync_calls += 1

    host = _Host()
    return host


def _build(host):
    """Build the robot config page and keep it alive on the host."""
    host._test_page = host._build_robot_config_page()
    return host


# ---------------------------------------------------------------------------
# Sync-state indicator
# ---------------------------------------------------------------------------

class TestSyncIndicator:
    """Tests for the mtime-based sync-state indicator."""

    def test_indicator_widget_exists_after_build(self):
        """_rc_sync_indicator QLabel exists after page build."""
        from PyQt5.QtWidgets import QLabel
        host = _build(_make_mixin())
        assert hasattr(host, '_rc_sync_indicator')
        assert isinstance(host._rc_sync_indicator, QLabel)

    def test_check_sync_returns_false_when_no_deploy_file(self):
        """_rc_check_sync_state returns False when deployed YAML is absent."""
        host = _build(_make_mixin())
        # Point ROBOT_CONFIG_FILE to a temp file; ensure deployed path doesn't exist
        with tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as f:
            host.ROBOT_CONFIG_FILE = f.name
        try:
            # Patch _rc_fic_config_yaml_path to a non-existent path
            host._rc_fic_config_yaml_path = staticmethod(
                lambda: '/tmp/__nonexistent_robots_config__.yaml'
            )
            result = host._rc_check_sync_state()
            assert result is False
        finally:
            os.unlink(host.ROBOT_CONFIG_FILE)

    def test_check_sync_returns_false_when_in_sync(self):
        """_rc_check_sync_state returns False when deployed YAML is at least as new."""
        host = _build(_make_mixin())
        with (
            tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as gui_f,
            tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as dep_f,
        ):
            gui_path = gui_f.name
            dep_path = dep_f.name
        try:
            # Make deployed file newer than GUI file
            t_old = time.time() - 10
            os.utime(gui_path, (t_old, t_old))
            # dep_path has current mtime (newer)
            host.ROBOT_CONFIG_FILE = gui_path
            host._rc_fic_config_yaml_path = staticmethod(lambda: dep_path)
            result = host._rc_check_sync_state()
            assert result is False
        finally:
            os.unlink(gui_path)
            os.unlink(dep_path)

    def test_check_sync_returns_true_when_gui_newer(self):
        """_rc_check_sync_state returns True when GUI YAML is strictly newer."""
        host = _build(_make_mixin())
        with (
            tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as gui_f,
            tempfile.NamedTemporaryFile(suffix='.yaml', delete=False) as dep_f,
        ):
            gui_path = gui_f.name
            dep_path = dep_f.name
        try:
            # Make deployed file older than GUI file
            t_old = time.time() - 10
            os.utime(dep_path, (t_old, t_old))
            host.ROBOT_CONFIG_FILE = gui_path
            host._rc_fic_config_yaml_path = staticmethod(lambda: dep_path)
            result = host._rc_check_sync_state()
            assert result is True
        finally:
            os.unlink(gui_path)
            os.unlink(dep_path)

    def test_indicator_text_amber_when_out_of_sync(self):
        """The indicator says the controllers' copy is older, and to Save."""
        host = _build(_make_mixin())
        host._rc_update_sync_indicator(True)
        text = host._rc_sync_indicator.text().upper()
        assert 'OLDER' in text and 'SAVE' in text

    def test_indicator_text_neutral_when_in_sync(self):
        """The indicator says Saved when in sync."""
        host = _build(_make_mixin())
        host._rc_update_sync_indicator(False)
        assert 'SAVED' in host._rc_sync_indicator.text().upper()

    def test_indicator_shows_unsaved_edits(self):
        """An edit turns the indicator amber until it is saved."""
        from fic_studio import theme
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_display_name.setText('Edited')
        assert 'UNSAVED' in host._rc_sync_indicator.text().upper()
        assert theme.current()['accent_orange'] in host._rc_sync_indicator.styleSheet()
        host._rc_save()
        assert 'UNSAVED' not in host._rc_sync_indicator.text().upper()

    def test_indicator_style_amber_uses_accent_orange(self):
        """Out-of-sync indicator style contains accent_orange token value."""
        from fic_studio import theme
        host = _build(_make_mixin())
        host._rc_update_sync_indicator(True)
        style = host._rc_sync_indicator.styleSheet()
        tok = theme.current()['accent_orange']
        assert tok in style, (
            'Expected accent_orange token (%s) in style: %s' % (tok, style)
        )

    def test_indicator_style_neutral_uses_text_disabled(self):
        """In-sync indicator style contains text_disabled token value."""
        from fic_studio import theme
        host = _build(_make_mixin())
        host._rc_update_sync_indicator(False)
        style = host._rc_sync_indicator.styleSheet()
        tok = theme.current()['text_disabled']
        assert tok in style, (
            'Expected text_disabled token (%s) in style: %s' % (tok, style)
        )


# ---------------------------------------------------------------------------
# Dirty flag
# ---------------------------------------------------------------------------

class TestDirtyFlag:
    """Tests for the unsaved-edits dirty flag."""

    def test_dirty_false_after_build(self):
        """Dirty flag is False immediately after page build."""
        host = _build(_make_mixin())
        assert host._rc_dirty is False

    def test_dirty_false_after_load(self):
        """Dirty flag is False after _rc_load_robot."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        assert host._rc_dirty is False

    def test_dirty_set_on_display_name_edit(self):
        """Editing display_name field sets the dirty flag."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_display_name.setText('Edited Name')
        assert host._rc_dirty is True

    def test_dirty_set_on_num_joints_change(self):
        """Changing num_joints spinner sets the dirty flag."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_num_joints.setValue(5)
        assert host._rc_dirty is True

    def test_dirty_set_on_has_gripper_toggle(self):
        """Toggling has_gripper checkbox sets the dirty flag."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        current = host._rc_has_gripper.isChecked()
        host._rc_has_gripper.setChecked(not current)
        assert host._rc_dirty is True

    def test_dirty_set_on_urdf_path_edit(self):
        """Editing URDF path sets the dirty flag."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_urdf_path.setText('/some/new/path.urdf')
        assert host._rc_dirty is True

    def test_dirty_cleared_by_rc_clear_dirty(self):
        """_rc_clear_dirty resets the dirty flag."""
        host = _build(_make_mixin())
        host._rc_dirty = True
        host._rc_clear_dirty()
        assert host._rc_dirty is False

    def test_dirty_cleared_after_rc_load_robot(self):
        """_rc_load_robot clears dirty even if previously set."""
        host = _build(_make_mixin())
        host._rc_dirty = True
        host._rc_load_robot('Franka_FR3')
        assert host._rc_dirty is False

    def test_selector_change_prompts_when_dirty(self):
        """Switching robots when dirty triggers a QMessageBox prompt."""
        from unittest.mock import patch
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_dirty = True
        # Simulate user pressing Cancel → robot should NOT switch
        with patch(
            'fic_studio.mixins.robot_config_mixin.QMessageBox.question',
            return_value=0x00400000,  # QMessageBox.Cancel
        ):
            host._rc_robot_list.setCurrentRow(1)  # try to switch
        # Robot ID should remain Franka_FR3, and so should the list
        assert host._rc_robot_id_lbl.text() == 'Franka_FR3'
        assert host._rc_robot_list.currentRow() == 0

    def test_selector_change_loads_when_dirty_discarded(self):
        """Switching robots with Discard reply loads the new robot."""
        from unittest.mock import patch
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_dirty = True
        # Select the second robot in the list. This must happen INSIDE the
        # patch: the list's handler pops the real unsaved-changes
        # QMessageBox while dirty, and a modal dialog under offscreen Qt
        # aborts the interpreter.
        with patch(
            'fic_studio.mixins.robot_config_mixin.QMessageBox.question',
            return_value=0x00800000,  # QMessageBox.Discard
        ):
            host._rc_robot_list.setCurrentRow(1)
        assert host._rc_robot_id_lbl.text() == 'KinovaGen3_7DOF'

    def test_selector_change_saves_when_asked(self):
        """Choosing Save in the prompt saves the edit, then switches."""
        from unittest.mock import patch
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_display_name.setText('Renamed FR3')
        with patch(
            'fic_studio.mixins.robot_config_mixin.QMessageBox.question',
            return_value=0x00000800,  # QMessageBox.Save
        ):
            host._rc_robot_list.setCurrentRow(1)
        assert host.robot_configs['Franka_FR3']['display_name'] == 'Renamed FR3'
        assert host._rc_robot_id_lbl.text() == 'KinovaGen3_7DOF'


# ---------------------------------------------------------------------------
# Path-exists brick-border indicator
# ---------------------------------------------------------------------------

class TestPathExistsIndicator:
    """Tests for the path-exists border indicator on URDF / MuJoCo fields."""

    def test_missing_path_applies_error_border(self):
        """A non-existent path sets an accent_red border on the line edit."""
        from fic_studio import theme
        host = _build(_make_mixin())
        host._rc_update_path_indicator(host._rc_urdf_path)
        host._rc_urdf_path.setText('/absolutely/nonexistent/robot.urdf')
        _app.processEvents()
        style = host._rc_urdf_path.styleSheet()
        tok = theme.current()['accent_red']
        assert tok in style, (
            'Expected accent_red (%s) for missing path; got: %s' % (tok, style)
        )

    def test_empty_path_applies_neutral_border(self):
        """An empty path field gets a border_subtle (neutral) border."""
        from fic_studio import theme
        host = _build(_make_mixin())
        host._rc_urdf_path.setText('')
        _app.processEvents()
        style = host._rc_urdf_path.styleSheet()
        tok = theme.current()['border_subtle']
        assert tok in style, (
            'Expected border_subtle (%s) for empty path; got: %s' % (tok, style)
        )

    def test_existing_path_applies_neutral_border(self):
        """An existing path gets a border_subtle (neutral) border."""
        from fic_studio import theme
        host = _build(_make_mixin())
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as f:
            existing = f.name
        try:
            host._rc_urdf_path.setText(existing)
            _app.processEvents()
            style = host._rc_urdf_path.styleSheet()
            tok = theme.current()['border_subtle']
            assert tok in style, (
                'Expected border_subtle (%s) for existing path; got: %s'
                % (tok, style)
            )
        finally:
            os.unlink(existing)

    def test_mujoco_path_indicator_works(self):
        """Path indicator also works for the MuJoCo XML path field."""
        from fic_studio import theme
        host = _build(_make_mixin())
        host._rc_mujoco_path.setText('/no/such/file.xml')
        _app.processEvents()
        style = host._rc_mujoco_path.styleSheet()
        tok = theme.current()['accent_red']
        assert tok in style


# ---------------------------------------------------------------------------
# One Save writes both YAMLs
# ---------------------------------------------------------------------------

class TestSaveWritesBoth:
    """_rc_save replaces Save + Deploy: it writes both robot YAMLs."""

    def test_save_calls_sync(self):
        """_rc_save invokes _sync_to_fic_config exactly once."""
        host = _build(_make_mixin())
        host._rc_save()
        assert host._sync_calls == 1

    def test_save_refreshes_indicator(self):
        """_rc_save calls _rc_check_sync_state (indicator updated)."""
        from unittest.mock import patch
        host = _build(_make_mixin())
        checked = []
        original = host._rc_check_sync_state

        def _spy():
            checked.append(True)
            return original()

        with patch.object(host, '_rc_check_sync_state', side_effect=_spy):
            host._rc_save()
        assert len(checked) >= 1, '_rc_check_sync_state was not called by _rc_save'

    def test_hot_config_goes_only_to_stations_running_this_robot(self):
        """Saving the Franka must not flip the running Kinova's switches."""
        published = []

        class _Pub:
            def __init__(self, rid):
                self.rid = rid

            def publish(self, msg):
                published.append(self.rid)

        host = _build(_make_mixin())
        host.node = types.SimpleNamespace(station_id=1, pubs={
            'robot1': {'gravity_comp': _Pub('robot1')}})
        host.robot_dropdown.addItems(['KinovaGen3_7DOF', 'Franka_FR3'])
        host.robot_dropdown.setCurrentText('KinovaGen3_7DOF')
        host._rc_load_robot('Franka_FR3')
        host._rc_save()
        assert published == []
        host._rc_load_robot('KinovaGen3_7DOF')
        host._rc_save()
        assert published == ['robot1']


# ---------------------------------------------------------------------------
# Joint-array / table consistency after num_joints change
# ---------------------------------------------------------------------------

class TestJointArrayConsistency:
    """Verify _rc_collect_robot joint arrays match the table after resize."""

    def test_collect_after_grow_has_correct_length(self):
        """Growing num_joints to 10 yields 10-element arrays in collected dict."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')  # starts at 7
        host._rc_num_joints.setValue(10)
        result = host._rc_collect_robot()
        assert len(result['joint_names']) == 10
        assert len(result['q_home_deg']) == 10
        assert len(result['joint_limits_deg']['lower']) == 10
        assert len(result['joint_torque_limits']['max']) == 10

    def test_collect_after_shrink_has_correct_length(self):
        """Shrinking num_joints to 3 yields 3-element arrays in collected dict."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')  # starts at 7
        host._rc_num_joints.setValue(3)
        result = host._rc_collect_robot()
        assert len(result['joint_names']) == 3
        assert len(result['q_home_deg']) == 3
        assert len(result['joint_limits_deg']['lower']) == 3

    def test_table_rows_match_collect_joint_names(self):
        """Table rows() names list is identical to collected joint_names."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        table_names = [r['name'] for r in host._rc_joint_table.rows()]
        result = host._rc_collect_robot()
        assert result['joint_names'] == table_names

    def test_home_fields_count_matches_num_joints(self):
        """Number of home MetricFields equals num_joints after resize."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        host._rc_num_joints.setValue(4)
        assert len(host._rc_home_fields) == 4


# ---------------------------------------------------------------------------
# Dirty-flag wiring completeness (previously-missing widgets)
# ---------------------------------------------------------------------------

class TestDirtyFlagWiring:
    """
    Verify every editable widget that was previously unconnected now sets dirty.

    Covers: EE position MetricFields, EE quaternion spinboxes, admittance
    deadband MetricFields, gripper MetricFields, frame-name QLineEdits,
    Cartesian safety spinboxes and checkboxes, joint-safety text/checkbox,
    and velocity spinboxes/checkbox.
    """

    def _host(self):
        """Return a freshly-built, clean-state host."""
        host = _build(_make_mixin())
        host._rc_load_robot('Franka_FR3')
        return host

    # EE position

    def test_dirty_on_ee_pos_x_committed(self):
        """Committing EE X position MetricField sets dirty."""
        host = self._host()
        host._rc_ee_pos_x.committed.emit(0.123)
        assert host._rc_dirty is True

    def test_dirty_on_ee_pos_y_committed(self):
        """Committing EE Y position MetricField sets dirty."""
        host = self._host()
        host._rc_ee_pos_y.committed.emit(0.456)
        assert host._rc_dirty is True

    def test_dirty_on_ee_pos_z_committed(self):
        """Committing EE Z position MetricField sets dirty."""
        host = self._host()
        host._rc_ee_pos_z.committed.emit(0.789)
        assert host._rc_dirty is True

    # EE quaternion

    def test_dirty_on_ee_qx_changed(self):
        """Changing EE Qx spinbox sets dirty."""
        host = self._host()
        host._rc_ee_qx.setValue(0.1)
        assert host._rc_dirty is True

    def test_dirty_on_ee_qy_changed(self):
        """Changing EE Qy spinbox sets dirty."""
        host = self._host()
        host._rc_ee_qy.setValue(0.2)
        assert host._rc_dirty is True

    def test_dirty_on_ee_qz_changed(self):
        """Changing EE Qz spinbox sets dirty."""
        host = self._host()
        host._rc_ee_qz.setValue(0.3)
        assert host._rc_dirty is True

    def test_dirty_on_ee_qw_changed(self):
        """Changing EE Qw spinbox sets dirty."""
        host = self._host()
        host._rc_ee_qw.setValue(0.99)
        assert host._rc_dirty is True

    # Admittance deadbands

    def test_dirty_on_db_joint_tau_committed(self):
        """Committing joint-tau deadband MetricField sets dirty."""
        host = self._host()
        host._rc_db_joint_tau.committed.emit(2.5)
        assert host._rc_dirty is True

    def test_dirty_on_db_cart_force_committed(self):
        """Committing cart-force deadband MetricField sets dirty."""
        host = self._host()
        host._rc_db_cart_force.committed.emit(5.0)
        assert host._rc_dirty is True

    def test_dirty_on_db_cart_torque_committed(self):
        """Committing cart-torque deadband MetricField sets dirty."""
        host = self._host()
        host._rc_db_cart_torque.committed.emit(0.5)
        assert host._rc_dirty is True

    # Gripper

    def test_dirty_on_gripper_closed_committed(self):
        """Committing gripper-closed MetricField sets dirty."""
        host = self._host()
        host._rc_gripper_closed.committed.emit(0.01)
        assert host._rc_dirty is True

    def test_dirty_on_gripper_open_committed(self):
        """Committing gripper-open MetricField sets dirty."""
        host = self._host()
        host._rc_gripper_open.committed.emit(0.09)
        assert host._rc_dirty is True

    # Frame names

    def test_dirty_on_ee_frame_text_changed(self):
        """Editing EE frame name QLineEdit sets dirty."""
        host = self._host()
        host._rc_ee_frame.setText('custom_ee_link')
        assert host._rc_dirty is True

    def test_dirty_on_elbow_frame_text_changed(self):
        """Editing elbow frame name QLineEdit sets dirty."""
        host = self._host()
        host._rc_elbow_frame.setText('link3_custom')
        assert host._rc_dirty is True

    # Safety zones — Cartesian spinboxes and checkboxes

    def test_dirty_on_sz_cart_x_min_changed(self):
        """Changing Cartesian X min safety spinbox sets dirty."""
        host = self._host()
        sb_min, _sb_max, _cb = host._rc_sz_cart['x']
        sb_min.setValue(-0.5)
        assert host._rc_dirty is True

    def test_dirty_on_sz_cart_y_max_changed(self):
        """Changing Cartesian Y max safety spinbox sets dirty."""
        host = self._host()
        _sb_min, sb_max, _cb = host._rc_sz_cart['y']
        sb_max.setValue(0.8)
        assert host._rc_dirty is True

    def test_dirty_on_sz_cart_z_enabled_toggled(self):
        """Toggling Cartesian Z enabled checkbox sets dirty."""
        host = self._host()
        _sb_min, _sb_max, cb_en = host._rc_sz_cart['z']
        cb_en.setChecked(not cb_en.isChecked())
        assert host._rc_dirty is True

    # Safety zones — joint safety text and checkbox

    def test_dirty_on_joint_safety_text_changed(self):
        """Editing joint safety text box sets dirty."""
        host = self._host()
        host._rc_joint_safety_edit.setPlainText('-170, 170\n-100, 100')
        assert host._rc_dirty is True

    def test_dirty_on_joint_safety_enabled_toggled(self):
        """Toggling joint safety enabled checkbox sets dirty."""
        host = self._host()
        current = host._rc_joint_safety_enabled.isChecked()
        host._rc_joint_safety_enabled.setChecked(not current)
        assert host._rc_dirty is True

    # Safety zones — velocity spinboxes and checkbox

    def test_dirty_on_max_joint_vel_changed(self):
        """Changing max joint velocity spinbox sets dirty."""
        host = self._host()
        host._rc_max_joint_vel.setValue(2.0)
        assert host._rc_dirty is True

    def test_dirty_on_max_cart_vel_changed(self):
        """Changing max Cartesian velocity spinbox sets dirty."""
        host = self._host()
        host._rc_max_cart_vel.setValue(1.0)
        assert host._rc_dirty is True

    def test_dirty_on_vel_enabled_toggled(self):
        """Toggling velocity enabled checkbox sets dirty."""
        host = self._host()
        current = host._rc_vel_enabled.isChecked()
        host._rc_vel_enabled.setChecked(not current)
        assert host._rc_dirty is True

    # Home angle MetricFields

    def test_dirty_on_home_angle_committed(self):
        """Committing a home-angle MetricField in any joint row sets dirty."""
        host = self._host()
        assert len(host._rc_home_fields) > 0, 'No home fields present after load'
        host._rc_clear_dirty()
        host._rc_home_fields[0].committed.emit(45.0)
        assert host._rc_dirty is True

    def test_dirty_on_home_angle_last_row_committed(self):
        """Committing the last home-angle MetricField (after grow) sets dirty."""
        host = self._host()
        host._rc_on_num_joints_changed(10)
        host._rc_clear_dirty()
        host._rc_home_fields[-1].committed.emit(-30.0)
        assert host._rc_dirty is True

    def test_dirty_on_home_angle_after_shrink_committed(self):
        """Committing a home-angle MetricField after shrinking still sets dirty."""
        host = self._host()
        host._rc_on_num_joints_changed(3)
        host._rc_clear_dirty()
        host._rc_home_fields[1].committed.emit(90.0)
        assert host._rc_dirty is True


