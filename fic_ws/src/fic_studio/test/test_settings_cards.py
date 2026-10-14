"""
Tests for the Settings page: card layout, the _settings_card helper, widget attributes.

Verifies that _build_settings_page returns a QScrollArea with the expected
card-based structure and that all required widget attributes are created.
"""

import os
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt  # noqa: E402
from PyQt5.QtWidgets import (  # noqa: E402
    QApplication,
    QFrame,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
)

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_window(theme_name='warmth_dark'):
    """Return a minimal window that satisfies SettingsMixin's dependencies."""
    from fic_studio.theme import apply_theme
    from fic_studio.theme.tokens import normalize_theme_name
    from fic_studio.mixins.settings_mixin import SettingsMixin

    class _StubWindow(QMainWindow, SettingsMixin):
        """Minimal stub satisfying SettingsMixin."""

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

        def show_settings_panel(self):
            """Stub navigation."""

        def _emergency_stop(self):
            """Stub e-stop."""

        def log_message(self, *args):
            """Stub logger."""

        def _apply_settings(self):
            """Stub apply — avoids log_viewer/robot_dropdown dependencies."""

    win = _StubWindow()
    apply_theme(win, theme_name)
    win._init_settings()
    # _init_settings loads ~/.robot_selector_gui_settings.yaml, so without this
    # the tests read whatever theme the developer happens to have saved. That
    # was invisible while every theme was a warm one — the palette half of the
    # answer could only ever be "warmth" — and started failing the moment a
    # cool theme could be selected. Honour the argument the caller passed.
    win.settings['appearance']['theme'] = normalize_theme_name(theme_name)
    return win


# ---------------------------------------------------------------------------
# _settings_card
# ---------------------------------------------------------------------------

class TestSettingsCard:
    """Tests for the _settings_card helper."""

    def test_returns_frame_and_layout(self):
        """_settings_card must return (QFrame, QVBoxLayout)."""
        win = _make_window()
        frame, body = win._settings_card('Test Section')
        assert isinstance(frame, QFrame)
        assert isinstance(body, QVBoxLayout)

    def test_title_uppercase(self):
        """Card title (collapsible header button) text must be uppercase."""
        from PyQt5.QtWidgets import QPushButton
        win = _make_window()
        frame, _ = win._settings_card('appearance')
        # The title is now a clickable collapse header (QPushButton) whose
        # text is the uppercased section name with a disclosure-triangle prefix
        # (e.g. '▾  APPEARANCE'). Accept either a label or a header button.
        widgets = frame.findChildren(QLabel) + frame.findChildren(QPushButton)
        texts = [w.text() for w in widgets if w.text()]
        assert any('APPEARANCE' in t for t in texts), (
            f'Expected APPEARANCE in card title texts {texts}')

    def test_no_restart_pill_by_default(self):
        """Without restart=True there should be no restart pill label."""
        win = _make_window()
        frame, _ = win._settings_card('Startup')
        labels = frame.findChildren(QLabel)
        texts = [lb.text() for lb in labels]
        assert not any('Restart' in t for t in texts), (
            f'Unexpected restart pill: {texts}')

    def test_restart_pill_present(self):
        """With restart=True the card must contain a restart-pill label."""
        win = _make_window()
        frame, _ = win._settings_card('Network', restart=True)
        labels = frame.findChildren(QLabel)
        texts = [lb.text() for lb in labels]
        assert any('Restart' in t for t in texts), (
            f'Restart pill not found in {texts}')


# ---------------------------------------------------------------------------
# _build_settings_page  — structure
# ---------------------------------------------------------------------------

class TestBuildSettingsPage:
    """Tests for _build_settings_page card layout."""

    def test_returns_scroll_area(self):
        """_build_settings_page must return a QScrollArea."""
        win = _make_window()
        page = win._build_settings_page()
        assert isinstance(page, QScrollArea)

    def test_no_station_id_widget(self):
        """settings_station_id must NOT be created (toolbar is authoritative)."""
        win = _make_window()
        win._build_settings_page()
        assert not hasattr(win, 'settings_station_id')

    def test_no_settings_theme_combo(self):
        """settings_theme (old QComboBox) must NOT be created."""
        win = _make_window()
        win._build_settings_page()
        assert not hasattr(win, 'settings_theme')

    def test_segmented_theme_buttons_exist(self):
        """Segmented theme control must expose _theme_dark_btn / _theme_light_btn."""
        win = _make_window()
        win._build_settings_page()
        assert hasattr(win, '_theme_dark_btn')
        assert hasattr(win, '_theme_light_btn')

    def test_required_widget_attributes(self):
        """All expected setting widget attributes must be created."""
        win = _make_window()
        win._build_settings_page()
        required = [
            'settings_default_robot',
            'settings_auto_save',
            'settings_auto_save_interval',
            'settings_show_log',
            'settings_log_height',
            'settings_font_family',
            'settings_font_size',
            'settings_ros_domain',
            'settings_partner_ip',
            'settings_dds_mode',
            'settings_sim_backend',
            'settings_joint_tol',
            'settings_pos_tol',
            'settings_timeout',
            'shortcut_inputs',
            'verbose_logging',
            'save_rosbag',
        ]
        missing = [attr for attr in required if not hasattr(win, attr)]
        assert not missing, f'Missing widget attributes: {missing}'

    def test_shortcut_inputs_keys(self):
        """shortcut_inputs must contain all expected shortcut keys."""
        win = _make_window()
        win._build_settings_page()
        expected_keys = {
            'estop', 'play_sequence', 'stop_sequence', 'save_settings',
            'toggle_log', 'panel_dashboard', 'panel_control',
            'panel_tuning', 'panel_sequence', 'panel_settings',
        }
        assert expected_keys.issubset(set(win.shortcut_inputs.keys()))

    def test_dark_theme_button_checked_by_default(self):
        """With warmth_dark settings the dark button must be checked."""
        win = _make_window('warmth_dark')
        win.settings['appearance']['theme'] = 'warmth_dark'
        page = win._build_settings_page()
        win.setCentralWidget(page)  # keep alive
        assert win._theme_dark_btn.isChecked()
        assert not win._theme_light_btn.isChecked()


# ---------------------------------------------------------------------------
# _get_theme_selection
# ---------------------------------------------------------------------------

class TestGetThemeSelection:
    """Tests for the _get_theme_selection helper."""

    def test_returns_dark_when_dark_checked(self):
        """Returns warmth_dark when the dark button is checked."""
        win = _make_window('warmth_dark')
        page = win._build_settings_page()
        win.setCentralWidget(page)  # keep widgets alive
        win._theme_dark_btn.setChecked(True)
        win._theme_light_btn.setChecked(False)
        assert win._get_theme_selection() == 'warmth_dark'

    def test_returns_light_when_light_checked(self):
        """Returns warmth_light when the light button is checked."""
        win = _make_window('warmth_dark')
        page = win._build_settings_page()
        win.setCentralWidget(page)  # keep widgets alive
        win._theme_dark_btn.setChecked(False)
        win._theme_light_btn.setChecked(True)
        assert win._get_theme_selection() == 'warmth_light'


# ---------------------------------------------------------------------------
# YAML key preservation
# ---------------------------------------------------------------------------

class TestYamlKeyPreservation:
    """Verify _save_settings_from_ui writes the expected YAML keys."""

    def test_required_yaml_keys_written(self, tmp_path):
        """All required YAML keys must be present after save."""
        import yaml

        win = _make_window()
        tmp_file = str(tmp_path / 'settings.yaml')
        win.SETTINGS_FILE = tmp_file
        page = win._build_settings_page()
        win.setCentralWidget(page)  # keep widgets alive

        with patch.object(QMessageBox, 'information', return_value=None):
            win._save_settings_from_ui()

        with open(tmp_file) as f:
            saved = yaml.safe_load(f)

        assert 'station_id' in saved['general'], (
            'general.station_id must stay in YAML (read by entry_pro.py)')
        assert 'ros_domain_id' in saved['network'], (
            'network.ros_domain_id must stay in YAML')
        assert saved['appearance']['theme'] in ('warmth_dark', 'warmth_light')
        assert 'shortcuts' in saved
        assert 'tolerances' in saved

    def test_station_id_not_overwritten_by_widget(self, tmp_path):
        """station_id in YAML must not be reset to 0 — it has no widget."""
        import yaml

        win = _make_window()
        win.settings['general']['station_id'] = 3
        tmp_file = str(tmp_path / 'settings2.yaml')
        win.SETTINGS_FILE = tmp_file
        page = win._build_settings_page()
        win.setCentralWidget(page)  # keep widgets alive

        with patch.object(QMessageBox, 'information', return_value=None):
            win._save_settings_from_ui()

        with open(tmp_file) as f:
            saved = yaml.safe_load(f)

        assert saved['general']['station_id'] == 3, (
            'station_id must be preserved unchanged (no widget writes it)')


# ---------------------------------------------------------------------------
# Review-finding fixes
# ---------------------------------------------------------------------------

class TestReviewFixes:
    """Tests for two fixes to the card layout."""

    def test_discard_button_label(self):
        """The title row's second button is DISCARD (it reverts to the snapshot)."""
        win = _make_window()
        page = win._build_settings_page()
        win.setCentralWidget(page)

        buttons = page.findChildren(QPushButton)
        labels = [b.text() for b in buttons]
        assert 'DISCARD' in labels, (
            f'Expected DISCARD button in title row; found {labels}')

    def test_apply_settings_calls_set_font(self):
        """_apply_settings must call QApplication.instance().setFont()."""
        from unittest.mock import patch as _patch
        from PyQt5.QtGui import QFont as _QFont

        captured = {}

        class _TrackingApp:
            """Spy that records calls to setFont."""

            def setFont(self, font):
                """Record font argument."""
                captured['font'] = font

        # Build a real window but use the real _apply_settings (not the stub).
        from fic_studio.mixins.settings_mixin import SettingsMixin

        class _FullWindow(QMainWindow, SettingsMixin):
            """Window that uses the real _apply_settings."""

            def _page_title(self, text):
                """Stub page title."""
                return QLabel(text)

            def _btn(self, text, style_id=None):
                """Stub button factory."""
                btn = QPushButton(text)
                if style_id:
                    btn.setObjectName(style_id)
                return btn

            def _launch_docs_website(self):
                """Stub docs launcher."""

            def _generate_cyclone_config(self):
                """Stub DDS config generator."""

            def show_dashboard(self):
                """Stub nav."""

            def show_robot_control_panel(self):
                """Stub nav."""

            def show_tuning_panel(self):
                """Stub nav."""

            def show_sequence_panel(self):
                """Stub nav."""

            def show_settings_panel(self):
                """Stub nav."""

            def _emergency_stop(self):
                """Stub e-stop."""

            def log_message(self, *args):
                """Stub logger."""

            def apply_theme(self, name):
                """Stub — avoids full theme engine during unit test."""

            def _setup_keyboard_shortcuts(self):
                """Stub shortcut setup."""

        from fic_studio.theme import apply_theme
        full_win = _FullWindow()
        apply_theme(full_win, 'warmth_dark')
        full_win._init_settings()

        spy = _TrackingApp()
        with _patch('fic_studio.mixins.settings_mixin.QApplication') as mock_qa:
            mock_qa.instance.return_value = spy
            full_win._apply_settings()

        assert 'font' in captured, '_apply_settings did not call QApplication.instance().setFont()'
        assert isinstance(captured['font'], _QFont)
        expected_family = full_win.settings['appearance']['font_family']
        assert captured['font'].family() == expected_family, (
            f"Font family mismatch: got {captured['font'].family()!r}, "
            f'expected {expected_family!r}'
        )
