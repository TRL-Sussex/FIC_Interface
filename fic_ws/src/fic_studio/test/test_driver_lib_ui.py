"""Tests for the driver library UI: the Hardware tab's drop zone, dialog and flow.

Headless (offscreen Qt). The robot_library is a temporary folder and the
robot YAMLs are never written: the page's save functions are stubbed.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.robot_import import driver_lib  # noqa: E402, I100
from PyQt5.QtCore import QMimeData, QPoint, Qt, QUrl  # noqa: E402
from PyQt5.QtGui import QDropEvent  # noqa: E402
from PyQt5.QtWidgets import QApplication, QDialog, QMessageBox  # noqa: E402
import pytest  # noqa: E402
from test_driver_lib import _fake_program, _write, make_cmake_lib  # noqa: E402

_app = QApplication.instance() or QApplication([])


def _drop(zone, paths):
    mime = QMimeData()
    mime.setUrls([QUrl.fromLocalFile(p) for p in paths])
    got = []
    zone.dropped.connect(got.append)
    zone.dropEvent(QDropEvent(QPoint(5, 5), Qt.CopyAction, mime, Qt.LeftButton,
                              Qt.NoModifier))
    return got


class TestLibraryDropZone:

    def test_takes_one_folder_or_zip(self, tmp_path):
        from fic_studio.components.robot_drop_zone import LibraryDropZone
        zone = LibraryDropZone()
        folder = str(tmp_path / 'sdk')
        os.makedirs(folder)
        zpath = _write(str(tmp_path / 'sdk.zip'), 'x')
        assert _drop(zone, [folder]) == [folder]
        assert _drop(LibraryDropZone(), [zpath]) == [zpath]

    def test_ignores_files_and_several_items(self, tmp_path):
        from fic_studio.components.robot_drop_zone import LibraryDropZone
        a, b = str(tmp_path / 'a'), str(tmp_path / 'b')
        os.makedirs(a)
        os.makedirs(b)
        urdf = _write(str(tmp_path / 'r.urdf'), '<robot/>')
        assert _drop(LibraryDropZone(), [urdf]) == []
        assert _drop(LibraryDropZone(), [a, b]) == []


# ── The review dialog ─────────────────────────────────────────────────────

def _dialog(plan, lib_root, configs=None, current=None):
    from fic_studio.components.driver_library_dialog import DriverLibraryDialog
    return DriverLibraryDialog(plan, lib_root, 'R1', current=current, configs=configs or {})


class TestDialog:

    def test_prefilled_from_the_library(self, tmp_path):
        plan = driver_lib.analyse_library(make_cmake_lib(str(tmp_path / 'src')))
        dlg = _dialog(plan, str(tmp_path / 'lib'))
        assert dlg.name_edit.text() == 'acme_driver'
        assert dlg.build_dir_edit.text() == '{robot_lib}/acme_driver/build'
        assert dlg.build_cmd_edit.text().startswith('cmake ..')
        assert dlg.program_combo.currentText() == 'build/acme_bridge'
        assert dlg.command_lbl.text() == 'cd {robot_lib}/acme_driver/build && ./acme_bridge'
        assert 'looked for again' in dlg.program_note.text()
        assert dlg.compile_cb.isChecked() and dlg.ok_btn.isEnabled()
        assert dlg.replace_cb.isHidden()

    def test_renaming_keeps_edits_and_arguments_join_the_command(self, tmp_path):
        plan = driver_lib.analyse_library(make_cmake_lib(str(tmp_path / 'src')))
        dlg = _dialog(plan, str(tmp_path / 'lib'))
        dlg.build_dir_edit.setText('{robot_lib}/acme_driver/build-release')
        dlg.name_edit.setText('acme')
        assert dlg.build_dir_edit.text() == '{robot_lib}/acme/build-release'
        dlg.args_edit.setText('192.168.1.10')
        dlg._accept()
        s = dlg.settings
        assert s['name'] == 'acme' and s['compile_now'] and not s['replace']
        assert s['driver_command'] == 'cd {robot_lib}/acme/build && ./acme_bridge 192.168.1.10'
        assert s['build_directory'] == '{robot_lib}/acme/build-release'

    def test_bad_name_or_clash_blocks_until_replace(self, tmp_path):
        lib_root = str(tmp_path / 'lib')
        os.makedirs(os.path.join(lib_root, 'acme_driver'))
        plan = driver_lib.analyse_library(make_cmake_lib(str(tmp_path / 'src')))
        configs = {'Other': {'build_directory': '{robot_lib}/acme_driver/build'}}
        dlg = _dialog(plan, lib_root, configs=configs)
        assert not dlg.replace_cb.isHidden() and not dlg.ok_btn.isEnabled()
        assert 'used by Other' in dlg.replace_cb.text()
        dlg.replace_cb.setChecked(True)
        assert dlg.ok_btn.isEnabled()
        dlg.name_edit.setText('no/slashes')
        assert not dlg.ok_btn.isEnabled() and not dlg.name_note.isHidden()
        dlg.name_edit.setText('fresh')
        assert dlg.ok_btn.isEnabled() and dlg.replace_cb.isHidden()

    def test_no_build(self, tmp_path):
        plan = driver_lib.analyse_library(make_cmake_lib(str(tmp_path / 'src')))
        dlg = _dialog(plan, str(tmp_path / 'lib'),
                      current={'driver_command': 'cd old && ./old', 'build_command': ''})
        dlg.build_combo.setCurrentText('None: it is already built')
        assert dlg.build_cmd_edit.text() == '' and not dlg.compile_cb.isEnabled()
        dlg._accept()
        assert dlg.settings['build_directory'] == '' and not dlg.settings['compile_now']
        labels = [w.text() for w in dlg.findChildren(type(dlg.program_note))]
        assert any("R1's driver settings now: cd old && ./old" in t for t in labels)


# ── The whole flow on the Robot Configuration page ────────────────────────

def _host():
    """Build the page on a QWidget (dialogs need a parent); saves are counted, not written."""
    import copy
    from fic_studio.mixins.robot_config_mixin import DEFAULT_ROBOT_CONFIGS, RobotConfigMixin
    from PyQt5.QtWidgets import QComboBox, QLabel, QPushButton, QWidget

    class _Host(QWidget, RobotConfigMixin):

        def __init__(self):
            super().__init__()
            self._page_title = lambda text: QLabel(text)  # noqa: E731
            self._btn = lambda text, style_id=None: QPushButton(text)  # noqa: E731
            self.robot_configs = copy.deepcopy(DEFAULT_ROBOT_CONFIGS)
            self.robot_dropdown = QComboBox()
            self.saves = self.syncs = 0

        def log_message(self, *a, **k):
            pass

        def _save_robot_configs(self):
            self.saves += 1

        def _sync_to_fic_config(self, show_result=False):
            self.syncs += 1

        def _refresh_robots_dropdown(self):
            pass

    host = _Host()
    host._test_page = host._build_robot_config_page()
    return host


@pytest.fixture
def page(tmp_path, monkeypatch):
    """Give the page a temporary robot_library, answer its dialog, record message boxes."""
    from fic_studio.components.driver_library_dialog import DriverLibraryDialog
    host = _host()
    lib_root = str(tmp_path / 'robot_library')
    host._rc_library_root = lambda: lib_root
    host.boxes = []
    for kind in ('information', 'warning', 'critical'):
        monkeypatch.setattr(QMessageBox, kind,
                            lambda *a, k=kind, **kw: host.boxes.append((k, a[1], a[2])))

    def accept(dlg):
        host.dialog = dlg
        getattr(host, 'tweak_dialog', lambda d: None)(dlg)
        dlg._accept()
        return QDialog.Accepted
    monkeypatch.setattr(DriverLibraryDialog, 'exec_', accept)
    host._rc_load_robot('Pendulum')
    return host, lib_root


class TestPageFlow:

    def _builds(self, host, lib_root, code=0, program='build/examples/acme_bridge'):
        """Stand in for the build: make the program CMake would, return ``code``."""
        def compile_(robot_id):
            host.compiled = robot_id
            cfg = host.robot_configs[robot_id]
            assert os.path.isdir(driver_lib.expand(cfg['build_directory'], lib_root))
            if code == 0 and program:
                _fake_program(os.path.join(lib_root, 'acme_driver', program))
            return code
        host._compile_robot_api = compile_

    def test_drop_copy_build_and_point_at_the_program(self, tmp_path, page):
        host, lib_root = page
        self._builds(host, lib_root)
        host._rc_start_library(make_cmake_lib(str(tmp_path / 'src')))
        assert host.compiled == 'Pendulum'
        lib = os.path.join(lib_root, 'acme_driver')
        assert os.path.isfile(os.path.join(lib, 'examples', 'acme_bridge.cpp'))
        cfg = host.robot_configs['Pendulum']
        # The guess was build/acme_bridge; the build put it in build/examples.
        assert cfg['driver_command'] == ('cd {robot_lib}/acme_driver/build/examples'
                                         ' && ./acme_bridge')
        assert cfg['build_directory'] == '{robot_lib}/acme_driver/build'
        assert host._rc_driver_cmd.text() == cfg['driver_command']
        assert host._rc_dirty is False and host.saves == 2
        kind, title, text = host.boxes[-1]
        assert kind == 'information' and title == 'Driver ready'
        # A build is not a hardware test: enabling hardware control stays the user's call.
        assert not cfg['is_supported'] and not host._rc_is_supported.isChecked()
        assert 'will not control Pendulum until you tick' in text

    def test_other_unsaved_edits_stay_unsaved(self, tmp_path, page):
        host, lib_root = page
        self._builds(host, lib_root)
        host._rc_display_name.setText('Edited, not saved')
        host._rc_start_library(make_cmake_lib(str(tmp_path / 'src')))
        assert host.robot_configs['Pendulum']['display_name'] != 'Edited, not saved'
        assert host._rc_dirty is True
        assert host.robot_configs['Pendulum']['driver_command'].endswith('./acme_bridge')

    def test_failed_build_keeps_the_guess_and_says_so(self, tmp_path, page):
        host, lib_root = page
        self._builds(host, lib_root, code=2)
        host._rc_start_library(make_cmake_lib(str(tmp_path / 'src')))
        cfg = host.robot_configs['Pendulum']
        assert cfg['driver_command'] == 'cd {robot_lib}/acme_driver/build && ./acme_bridge'
        assert not cfg.get('is_supported')
        assert host.boxes[-1][0] == 'warning' and 'did not finish' in host.boxes[-1][1]

    def test_not_building_now(self, tmp_path, page):
        host, lib_root = page
        self._builds(host, lib_root)
        host.compiled = None
        host.tweak_dialog = lambda d: d.compile_cb.setChecked(False)
        host._rc_start_library(make_cmake_lib(str(tmp_path / 'src')))
        assert host.compiled is None
        assert 'Library set up' in host.boxes[-1][1]
        assert os.path.isdir(os.path.join(lib_root, 'acme_driver'))

    def test_library_in_robot_library_is_used_in_place(self, tmp_path, page):
        host, lib_root = page
        lib = make_cmake_lib(lib_root)
        self._builds(host, lib_root)
        host._rc_start_library(lib)
        assert os.listdir(lib_root) == ['acme_driver']
        assert host.robot_configs['Pendulum']['driver_command'].endswith('./acme_bridge')

    def test_cancelled_dialog_changes_nothing(self, tmp_path, page, monkeypatch):
        from fic_studio.components.driver_library_dialog import DriverLibraryDialog
        host, lib_root = page
        monkeypatch.setattr(DriverLibraryDialog, 'exec_', lambda dlg: QDialog.Rejected)
        before = dict(host.robot_configs['Pendulum'])
        host._rc_start_library(make_cmake_lib(str(tmp_path / 'src')))
        assert host.robot_configs['Pendulum'] == before and host.saves == 0
        assert not os.path.exists(lib_root)

    def test_unreadable_drop_is_explained(self, tmp_path, page):
        host, _ = page
        host._rc_start_library(_write(str(tmp_path / 'bad.zip'), 'nope'))
        assert host.boxes == [('warning', 'Cannot use this library',
                               'bad.zip is not a valid zip file.')]

    def test_hardware_tab_has_the_zone(self, page):
        from fic_studio.components.robot_drop_zone import LibraryDropZone
        host, _ = page
        assert isinstance(host._rc_lib_zone, LibraryDropZone)
