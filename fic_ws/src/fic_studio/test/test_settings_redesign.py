"""
Tests for the Settings page: Save/Discard, fonts, theme on save, debug.

Verifies:
- Save writes all expected YAML keys (unchanged keys preserved).
- Discard reverts widget values to the snapshot taken on page open.
- Theme is NOT applied on widget change; applied only on Save.
- _apply_settings calls QApplication.setFont with the correct font.
- No settings_station_id attribute is built anywhere.
- advanced.verbose_logging / save_rosbag are persisted and applied.
"""

import copy
import logging
import os
from unittest.mock import MagicMock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt  # noqa: E402
from PyQt5.QtGui import QFont  # noqa: E402
from PyQt5.QtWidgets import (  # noqa: E402
    QApplication,
    QLabel,
    QMainWindow,
    QPushButton,
)

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_window(theme_name='warmth_dark', stub_apply=False):
    """
    Return a minimal window satisfying SettingsMixin dependencies.

    Parameters
    ----------
    theme_name : str
        Initial theme to apply.
    stub_apply : bool
        When True the real _apply_settings is replaced by a no-op stub to
        isolate tests that do not want side-effects (log_viewer, timers, …).

    """
    from fic_studio.theme import apply_theme
    from fic_studio.theme.tokens import normalize_theme_name
    from fic_studio.mixins.settings_mixin import SettingsMixin

    class _StubWindow(QMainWindow, SettingsMixin):
        """Minimal stub satisfying SettingsMixin."""

        def _show_page(self, name):
            """Navigate by name, as the real window does."""
            self._shown_page = name
            return True

        def _page_title(self, text):
            """Return a plain QLabel as a page title."""
            lbl = QLabel(text)
            lbl.setObjectName('PageTitle')
            return lbl

        def _btn(self, text, style_id=None):
            """Return a plain QPushButton."""
            btn = QPushButton(text)
            btn.setCursor(Qt.PointingHandCursor)
            if style_id:
                btn.setObjectName(style_id)
            return btn

        def _launch_docs_website(self):
            """Stub docs launcher."""

        def _generate_cyclone_config(self):
            """Stub DDS config generator."""

        def show_dashboard(self):
            """Stub navigation."""

        def show_robot_control_panel(self):
            """Stub navigation."""

        def show_tuning_panel(self):
            """Stub navigation."""

        def show_sequence_panel(self):
            """Stub navigation."""

        def log_message(self, *args):
            """Stub logger."""

        def _emergency_stop(self):
            """Stub e-stop."""

    win = _StubWindow()
    apply_theme(win, theme_name)

    if stub_apply:
        win._apply_settings = lambda: None  # avoid timer/log-viewer deps

    win._init_settings()
    # _init_settings loads the developer's own settings file, so without this
    # the tests read whatever theme happens to be saved on the machine. It was
    # invisible while every theme was warm — the palette half of the answer
    # could only ever be "warmth" — and broke as soon as a cool theme could be
    # picked. Honour the argument the caller passed.
    win.settings['appearance']['theme'] = normalize_theme_name(theme_name)
    return win


def _build(win):
    """Build the settings page on *win* and return the scroll area."""
    page = win._build_settings_page()
    win.setCentralWidget(page)
    return page


# ---------------------------------------------------------------------------
# Station ID — must NOT be created
# ---------------------------------------------------------------------------

class TestNoStationId:
    """settings_station_id must never be created by the settings page."""

    def test_no_settings_station_id_after_build(self):
        """_build_settings_page must not create settings_station_id."""
        win = _make_window(stub_apply=True)
        _build(win)
        assert not hasattr(win, 'settings_station_id'), (
            'settings_station_id should not exist; toolbar is authoritative')

    def test_save_does_not_reference_station_id_widget(self):
        """_save_settings_from_ui must not raise AttributeError for settings_station_id."""
        win = _make_window(stub_apply=True)
        _build(win)
        # Patch file write and apply so only the widget-reading part is tested.
        with patch.object(win, '_save_settings'), \
             patch.object(win, '_apply_settings'):
            with patch('PyQt5.QtWidgets.QMessageBox.information'):
                win._save_settings_from_ui()  # must not raise


# ---------------------------------------------------------------------------
# Save writes expected YAML keys
# ---------------------------------------------------------------------------

class TestSaveWritesYamlKeys:
    """_save_settings_from_ui must write all required YAML keys."""

    def test_all_top_level_sections_written(self):
        """After save, self.settings must contain all expected top-level keys."""
        win = _make_window(stub_apply=True)
        _build(win)

        captured = {}

        def _fake_save():
            """Capture settings dict at save time."""
            captured.update(copy.deepcopy(win.settings))

        with patch.object(win, '_save_settings', side_effect=_fake_save), \
             patch.object(win, '_apply_settings'), \
             patch('PyQt5.QtWidgets.QMessageBox.information'):
            win._save_settings_from_ui()

        required_sections = [
            'general', 'appearance', 'network',
            'simulation', 'tolerances', 'shortcuts', 'advanced',
        ]
        missing = [s for s in required_sections if s not in captured]
        assert not missing, f'Missing top-level YAML sections: {missing}'

    def test_general_station_id_preserved(self):
        """general.station_id must remain in the YAML (entry_pro.py reads it)."""
        win = _make_window(stub_apply=True)
        _build(win)
        # Set a known value directly (mimics what toolbar writes).
        win.settings['general']['station_id'] = 2

        captured = {}

        def _fake_save():
            """Capture settings at save time."""
            captured.update(copy.deepcopy(win.settings))

        with patch.object(win, '_save_settings', side_effect=_fake_save), \
             patch.object(win, '_apply_settings'), \
             patch('PyQt5.QtWidgets.QMessageBox.information'):
            win._save_settings_from_ui()

        assert 'station_id' in captured.get('general', {}), (
            'general.station_id must be preserved in YAML for entry_pro.py')
        assert captured['general']['station_id'] == 2

    def test_ros_domain_id_written(self):
        """network.ros_domain_id must be written by _save_settings_from_ui."""
        win = _make_window(stub_apply=True)
        _build(win)
        win.settings_ros_domain.setValue(42)

        captured = {}

        def _fake_save():
            """Capture settings at save time."""
            captured.update(copy.deepcopy(win.settings))

        with patch.object(win, '_save_settings', side_effect=_fake_save), \
             patch.object(win, '_apply_settings'), \
             patch('PyQt5.QtWidgets.QMessageBox.information'):
            win._save_settings_from_ui()

        assert captured.get('network', {}).get('ros_domain_id') == 42

    def test_advanced_verbose_logging_written(self):
        """advanced.verbose_logging must be written by _save_settings_from_ui."""
        win = _make_window(stub_apply=True)
        _build(win)
        win.verbose_logging.setChecked(True)

        captured = {}

        def _fake_save():
            """Capture settings at save time."""
            captured.update(copy.deepcopy(win.settings))

        with patch.object(win, '_save_settings', side_effect=_fake_save), \
             patch.object(win, '_apply_settings'), \
             patch('PyQt5.QtWidgets.QMessageBox.information'):
            win._save_settings_from_ui()

        assert captured.get('advanced', {}).get('verbose_logging') is True

    def test_advanced_save_rosbag_written(self):
        """advanced.save_rosbag must be written by _save_settings_from_ui."""
        win = _make_window(stub_apply=True)
        _build(win)
        win.save_rosbag.setChecked(True)

        captured = {}

        def _fake_save():
            """Capture settings at save time."""
            captured.update(copy.deepcopy(win.settings))

        with patch.object(win, '_save_settings', side_effect=_fake_save), \
             patch.object(win, '_apply_settings'), \
             patch('PyQt5.QtWidgets.QMessageBox.information'):
            win._save_settings_from_ui()

        assert captured.get('advanced', {}).get('save_rosbag') is True


# ---------------------------------------------------------------------------
# Discard reverts to snapshot
# ---------------------------------------------------------------------------

class TestDiscardReverts:
    """_discard_settings must revert all widgets to the snapshot state."""

    def test_discard_reverts_font_size(self):
        """Changing font_size then discarding must restore the original value."""
        win = _make_window(stub_apply=True)
        _build(win)
        original_size = win.settings['appearance']['font_size']

        # Simulate opening the Settings page (takes snapshot).
        win._settings_snapshot = copy.deepcopy(win.settings)

        # User edits the font size widget.
        win.settings_font_size.setValue(original_size + 4)

        # User clicks Discard.
        win._discard_settings()

        assert win.settings_font_size.value() == original_size, (
            'Font size widget must revert to snapshot value after Discard')

    def test_discard_reverts_ros_domain_id(self):
        """Changing ros_domain_id then discarding must restore original value."""
        win = _make_window(stub_apply=True)
        _build(win)
        original_domain = win.settings['network']['ros_domain_id']

        win._settings_snapshot = copy.deepcopy(win.settings)
        win.settings_ros_domain.setValue(original_domain + 5)
        win._discard_settings()

        assert win.settings_ros_domain.value() == original_domain, (
            'ROS domain ID widget must revert to snapshot after Discard')

    def test_discard_reverts_verbose_logging(self):
        """Toggling verbose_logging then discarding must revert the checkbox."""
        win = _make_window(stub_apply=True)
        _build(win)
        original_verbose = win.settings.get('advanced', {}).get('verbose_logging', False)

        win._settings_snapshot = copy.deepcopy(win.settings)
        win.verbose_logging.setChecked(not original_verbose)
        win._discard_settings()

        assert win.verbose_logging.isChecked() == original_verbose, (
            'verbose_logging checkbox must revert to snapshot value after Discard')

    def test_discard_with_no_snapshot_is_noop(self):
        """_discard_settings with no snapshot must not raise."""
        win = _make_window(stub_apply=True)
        _build(win)
        # Ensure no snapshot attribute exists.
        if hasattr(win, '_settings_snapshot'):
            del win._settings_snapshot
        win._discard_settings()  # must not raise

    def test_discard_reverts_theme_buttons(self):
        """Discard must revert segmented theme buttons to snapshot theme."""
        win = _make_window('warmth_dark', stub_apply=True)
        _build(win)
        # Snapshot with dark theme.
        win._settings_snapshot = copy.deepcopy(win.settings)
        win._settings_snapshot['appearance']['theme'] = 'warmth_dark'

        # Simulate user clicking the light button.
        win._theme_light_btn.setChecked(True)
        win._theme_dark_btn.setChecked(False)

        win._discard_settings()

        assert win._theme_dark_btn.isChecked(), (
            'Dark button must be re-checked after Discard to warmth_dark snapshot')
        assert not win._theme_light_btn.isChecked()


# ---------------------------------------------------------------------------
# Theme applied on Save only — not on widget change
# ---------------------------------------------------------------------------

class TestThemeOnSaveOnly:
    """The live theme must not change when the user clicks a theme button."""

    def test_clicking_light_button_does_not_apply_theme(self):
        """Clicking 'Warm Light' must only change the checked state, not apply the theme."""
        win = _make_window('warmth_dark', stub_apply=True)
        _build(win)

        theme_calls = []
        original_apply = win.apply_theme

        def _spy(name):
            """Record apply_theme calls."""
            theme_calls.append(name)
            original_apply(name)

        win.apply_theme = _spy

        # Simulate clicking the light button.
        win._theme_light_btn.click()

        # The spy must NOT have been called by the click alone.
        assert not theme_calls, (
            f'apply_theme called on button click (should be Save-only): {theme_calls}')

    def test_save_applies_theme(self):
        """Save must call apply_theme via _apply_settings."""
        win = _make_window('warmth_dark')
        _build(win)

        # Select the light theme in the UI.
        win._theme_light_btn.setChecked(True)
        win._theme_dark_btn.setChecked(False)

        theme_calls = []

        def _spy_apply(name):
            """Record apply_theme calls."""
            theme_calls.append(name)

        with patch.object(win, 'apply_theme', side_effect=_spy_apply), \
             patch.object(win, '_save_settings'), \
             patch('PyQt5.QtWidgets.QMessageBox.information'):
            win._save_settings_from_ui()

        # apply_theme must have been called with warmth_light.
        assert any('warmth_light' in c for c in theme_calls), (
            f'apply_theme(warmth_light) not called on Save: {theme_calls}')


# ---------------------------------------------------------------------------
# _apply_settings calls setFont
# ---------------------------------------------------------------------------

class TestApplySettingsFont:
    """_apply_settings must call QApplication.setFont with the configured font."""

    def test_apply_settings_calls_set_font(self):
        """_apply_settings must call QApplication.instance().setFont(...)."""
        from fic_studio.mixins.settings_mixin import SettingsMixin

        win = _make_window()
        _build(win)

        win.settings['appearance']['font_family'] = 'Arial'
        win.settings['appearance']['font_size'] = 14

        set_font_calls = []

        mock_app = MagicMock()
        mock_app.setFont = lambda f: set_font_calls.append(f)

        with patch('fic_studio.mixins.settings_mixin.QApplication') as mock_qapp:
            mock_qapp.instance.return_value = mock_app
            # Call the real _apply_settings directly from the mixin class.
            SettingsMixin._apply_settings(win)

        assert set_font_calls, '_apply_settings did not call setFont'
        called_font = set_font_calls[0]
        assert isinstance(called_font, QFont)
        assert called_font.family() == 'Arial'
        assert called_font.pointSize() == 14

    def test_apply_settings_verbose_logging_sets_debug_level(self):
        """_apply_settings with verbose_logging=True must set root logger to DEBUG."""
        from fic_studio.mixins.settings_mixin import SettingsMixin

        win = _make_window()
        _build(win)

        win.settings['advanced'] = {'verbose_logging': True, 'save_rosbag': False}

        with patch('fic_studio.mixins.settings_mixin.QApplication') as mock_qapp:
            mock_qapp.instance.return_value = MagicMock()
            SettingsMixin._apply_settings(win)

        assert logging.getLogger().level == logging.DEBUG

    def test_apply_settings_verbose_off_sets_info_level(self):
        """_apply_settings with verbose_logging=False must set root logger to INFO."""
        from fic_studio.mixins.settings_mixin import SettingsMixin

        win = _make_window()
        _build(win)

        win.settings['advanced'] = {'verbose_logging': False, 'save_rosbag': False}

        with patch('fic_studio.mixins.settings_mixin.QApplication') as mock_qapp:
            mock_qapp.instance.return_value = MagicMock()
            SettingsMixin._apply_settings(win)

        assert logging.getLogger().level == logging.INFO


# ---------------------------------------------------------------------------
# Snapshot taken in show_settings_panel
# ---------------------------------------------------------------------------

class TestSnapshot:
    """show_settings_panel must snapshot settings before the page is displayed."""

    def test_snapshot_is_deep_copy(self):
        """_settings_snapshot must be independent of self.settings."""
        win = _make_window(stub_apply=True)
        _build(win)

        # Patch navigation methods that show_settings_panel calls.
        win._set_active_sidebar_btn = lambda btn: None
        win.main_stack = MagicMock()
        win._side_buttons = {9: MagicMock()}

        win.settings['appearance']['font_size'] = 12
        win.show_settings_panel()

        # Mutate settings AFTER snapshot.
        win.settings['appearance']['font_size'] = 99

        # Snapshot must still reflect the pre-mutation value.
        assert win._settings_snapshot['appearance']['font_size'] == 12, (
            '_settings_snapshot is not a deep copy — it reflects live mutations')
