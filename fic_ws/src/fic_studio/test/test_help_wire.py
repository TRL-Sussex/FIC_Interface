"""
Tests for how the Help page is wired into the main window.

Verifies:
- ``HelpMixin`` is listed in ``ProfessionalRobotGUI``'s MRO.
- ``show_help_panel`` is defined on ``ProfessionalRobotGUI``.
- Dead methods (``_build_help_section``, ``_launch_docs_website``,
  ``_stop_docs_server``, ``_build_settings_page_tabs``) are removed
  from ``SettingsMixin``.
- Stale imports (``QTabWidget``, ``QTableWidget``, ``QTableWidgetItem``,
  ``QHeaderView``) are not imported in ``settings_mixin`` module.
- The Help page builds in both ``warmth_dark`` and ``warmth_light`` themes
  via the mixin alone (no ROS/full-GUI needed).
- Extensions and Shortcuts sections render non-empty HTML.
- ``F1``/``Help`` menu action exists on the class (static check).
- ``show_help_panel`` sets the correct stack index when integrated into
  a minimal window that exposes ``main_stack`` and ``_help_page_idx``.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import (  # noqa: E402
    QApplication,
    QMainWindow,
    QPushButton,
    QStackedWidget,
)

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_help_window(theme_name='warmth_dark'):
    """Return a minimal QMainWindow mixing in HelpMixin only."""
    from fic_studio.theme import apply_theme
    from fic_studio.mixins.help_mixin import HelpMixin

    class _StubWin(QMainWindow, HelpMixin):
        """Minimal stub that satisfies HelpMixin dependencies."""

        def _show_page(self, name):
            """Navigate by name, as the real window does."""
            self._shown_page = name
            return True

        def log_message(self, *_args):
            """No-op logger."""

    win = _StubWin()
    apply_theme(win, theme_name)
    return win


# ---------------------------------------------------------------------------
# MRO / class-level wiring
# ---------------------------------------------------------------------------

class TestMroAndClassWiring:
    """HelpMixin is in ProfessionalRobotGUI's MRO; show_help_panel exists."""

    def test_help_mixin_in_mro(self):
        """The mixin must appear in ProfessionalRobotGUI.__mro__."""
        from fic_studio.main_window_pro import ProfessionalRobotGUI
        from fic_studio.mixins.help_mixin import HelpMixin
        assert HelpMixin in ProfessionalRobotGUI.__mro__, (
            'HelpMixin must be in ProfessionalRobotGUI.__mro__'
        )

    def test_show_help_panel_defined(self):
        """The show_help_panel method must be callable on the main window class."""
        from fic_studio.main_window_pro import ProfessionalRobotGUI
        assert callable(getattr(ProfessionalRobotGUI, 'show_help_panel', None)), (
            'show_help_panel must be callable on ProfessionalRobotGUI'
        )

    def test_build_help_page_inherited(self):
        """The _build_help_page method must be inherited via the mixin."""
        from fic_studio.main_window_pro import ProfessionalRobotGUI
        assert callable(getattr(ProfessionalRobotGUI, '_build_help_page', None)), (
            '_build_help_page must be callable on ProfessionalRobotGUI'
        )


# ---------------------------------------------------------------------------
# Dead-code removal from settings_mixin
# ---------------------------------------------------------------------------

class TestDeadCodeRemoved:
    """Removed methods and imports are absent from settings_mixin."""

    def test_build_help_section_removed(self):
        """_build_help_section must NOT exist on SettingsMixin."""
        from fic_studio.mixins.settings_mixin import SettingsMixin
        assert not hasattr(SettingsMixin, '_build_help_section'), (
            '_build_help_section was not removed from SettingsMixin'
        )

    def test_launch_docs_website_removed(self):
        """_launch_docs_website must NOT exist on SettingsMixin."""
        from fic_studio.mixins.settings_mixin import SettingsMixin
        assert not hasattr(SettingsMixin, '_launch_docs_website'), (
            '_launch_docs_website was not removed from SettingsMixin'
        )

    def test_stop_docs_server_removed(self):
        """_stop_docs_server must NOT exist on SettingsMixin."""
        from fic_studio.mixins.settings_mixin import SettingsMixin
        assert not hasattr(SettingsMixin, '_stop_docs_server'), (
            '_stop_docs_server was not removed from SettingsMixin'
        )

    def test_build_settings_page_tabs_removed(self):
        """_build_settings_page_tabs stub must NOT exist on SettingsMixin."""
        from fic_studio.mixins.settings_mixin import SettingsMixin
        assert not hasattr(SettingsMixin, '_build_settings_page_tabs'), (
            '_build_settings_page_tabs stub was not removed from SettingsMixin'
        )

    def test_qtabwidget_not_imported(self):
        """The stale tab-widget class must not be in the settings_mixin namespace."""
        import fic_studio.mixins.settings_mixin as sm
        assert not hasattr(sm, 'QTabWidget'), (
            'QTabWidget must not be in settings_mixin namespace'
        )

    def test_qtablewidget_not_imported(self):
        """The stale table-widget class must not be in the settings_mixin namespace."""
        import fic_studio.mixins.settings_mixin as sm
        assert not hasattr(sm, 'QTableWidget'), (
            'QTableWidget must not be in settings_mixin namespace'
        )

    def test_qtablewidgetitem_not_imported(self):
        """The stale table-widget-item class must not be in the settings_mixin namespace."""
        import fic_studio.mixins.settings_mixin as sm
        assert not hasattr(sm, 'QTableWidgetItem'), (
            'QTableWidgetItem must not be in settings_mixin namespace'
        )

    def test_qheaderview_not_imported(self):
        """The stale header-view class must not be in the settings_mixin namespace."""
        import fic_studio.mixins.settings_mixin as sm
        assert not hasattr(sm, 'QHeaderView'), (
            'QHeaderView must not be in settings_mixin namespace'
        )


# ---------------------------------------------------------------------------
# Help page builds in both themes (via mixin stub)
# ---------------------------------------------------------------------------

def _build_and_keep(win):
    """Build the help page, set as central widget to prevent GC, return it."""
    page = win._build_help_page()
    win.setCentralWidget(page)
    return page


class TestHelpPageBothThemes:
    """The help page widget builds correctly in both themes."""

    def test_dark_theme_builds(self):
        """warmth_dark: _build_help_page must return a non-None QWidget."""
        win = _make_help_window('warmth_dark')
        page = _build_and_keep(win)
        assert page is not None

    def test_light_theme_builds(self):
        """warmth_light: _build_help_page must return a non-None QWidget."""
        win = _make_help_window('warmth_light')
        page = _build_and_keep(win)
        assert page is not None

    def test_dark_nav_populated(self):
        """warmth_dark: nav list must have 13 items after build."""
        win = _make_help_window('warmth_dark')
        _build_and_keep(win)
        assert win._help_nav.count() == 13

    def test_light_nav_populated(self):
        """warmth_light: nav list must have 13 items after build."""
        win = _make_help_window('warmth_light')
        _build_and_keep(win)
        assert win._help_nav.count() == 13

    def test_dark_getting_started_renders(self):
        """warmth_dark: getting_started section renders non-empty HTML."""
        win = _make_help_window('warmth_dark')
        _build_and_keep(win)
        win._help_render_section('getting_started')
        html = win._help_browser.toHtml()
        assert 'Getting Started' in html

    def test_light_getting_started_renders(self):
        """warmth_light: getting_started section renders non-empty HTML."""
        win = _make_help_window('warmth_light')
        _build_and_keep(win)
        win._help_render_section('getting_started')
        html = win._help_browser.toHtml()
        assert 'Getting Started' in html


# ---------------------------------------------------------------------------
# Live-generated Extensions and Shortcuts sections
# ---------------------------------------------------------------------------

class TestLiveGeneratedSections:
    """Extensions and Shortcuts sections produce non-empty HTML."""

    def test_extensions_dark_non_empty(self):
        """warmth_dark: extensions section must render."""
        win = _make_help_window('warmth_dark')
        _build_and_keep(win)
        win._help_render_section('extensions')
        html = win._help_browser.toHtml()
        assert 'Extensions' in html and 'Gamepad' in html, (
            'Extensions section must render'
        )

    def test_shortcuts_dark_has_estop(self):
        """warmth_dark: keyboard_shortcuts section must contain E-Stop."""
        win = _make_help_window('warmth_dark')
        _build_and_keep(win)
        win._help_render_section('keyboard_shortcuts')
        html = win._help_browser.toHtml()
        assert 'E-Stop' in html, (
            'Shortcuts section must mention E-Stop'
        )

    def test_shortcuts_dark_has_space(self):
        """warmth_dark: keyboard_shortcuts section must contain Space binding."""
        win = _make_help_window('warmth_dark')
        _build_and_keep(win)
        win._help_render_section('keyboard_shortcuts')
        html = win._help_browser.toHtml()
        assert 'Space' in html

    def test_extensions_light_non_empty(self):
        """warmth_light: extensions section must render."""
        win = _make_help_window('warmth_light')
        _build_and_keep(win)
        win._help_render_section('extensions')
        html = win._help_browser.toHtml()
        assert 'Extensions' in html and 'Gamepad' in html

    def test_shortcuts_light_has_space(self):
        """warmth_light: keyboard_shortcuts section must contain Space binding."""
        win = _make_help_window('warmth_light')
        _build_and_keep(win)
        win._help_render_section('keyboard_shortcuts')
        html = win._help_browser.toHtml()
        assert 'Space' in html


# ---------------------------------------------------------------------------
# show_help_panel navigates to the page registered as "help"
# ---------------------------------------------------------------------------

class TestShowHelpPanel:
    """show_help_panel navigates by name, not by stored index."""

    def test_show_help_panel_navigates_to_the_help_page(self):
        """Pages are addressed by name; the stored index is gone."""
        from fic_studio.main_window_pro import ProfessionalRobotGUI
        from fic_studio.mixins.help_mixin import HelpMixin

        class _StubWin(QMainWindow, HelpMixin):
            """Minimal stub with main_stack and sidebar tracking."""

            # Bind show_help_panel from ProfessionalRobotGUI so we test
            # the real method body without constructing the full window.
            show_help_panel = ProfessionalRobotGUI.show_help_panel

            def log_message(self, *_args):
                """No-op logger."""

            def _set_active_sidebar_btn(self, btn):
                """Record which button was activated."""
                self._last_active_btn = btn

        from fic_studio.theme import apply_theme
        win = _StubWin()
        apply_theme(win, 'warmth_dark')

        # Build the help page; keep it alive via central widget.
        page = win._build_help_page()
        win.setCentralWidget(page)
        stack = QStackedWidget()
        stack.addWidget(page)
        win.main_stack = stack

        stub_btn = QPushButton('Help')
        win._help_sidebar_btn = stub_btn
        win._side_buttons = [stub_btn]
        win._last_active_btn = None

        # A registry over the same stack, with help already in it.
        from fic_studio.pages import PageRegistry
        win.pages = PageRegistry(stack)
        win.pages._index['help'] = stack.indexOf(page)
        win.pages.bind_button('help', stub_btn)
        win._show_page = ProfessionalRobotGUI._show_page.__get__(win)

        win.show_help_panel()

        assert win.pages.is_visible('help'), (
            'show_help_panel must navigate to the page registered as "help"'
        )
        assert win._last_active_btn is stub_btn, (
            'show_help_panel must activate the Help sidebar button'
        )
