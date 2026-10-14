"""
Tests for HelpMixin: help page, sections, search, live HTML.

Verifies:
- ``_build_help_page()`` returns a ``QWidget``.
- ``HELP_SECTIONS`` contains all 13 required keys.
- Section nav search filters items (matching count shrinks).
- ``_help_extensions_html()`` contains a known extension NAME from the
  manifests and loaded instances.
- ``_help_shortcuts_html()`` contains 'E-Stop' and 'Space'.
- ``_help_themed_html()`` injects a colour from ``theme.current()``.
- Lint exit 0 (exercised by the lintcheck gate, not this file).
- Render spot-check: both themes produce non-empty HTML for a couple of
  sections.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import (  # noqa: E402
    QApplication,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QTextBrowser,
    QWidget,
)

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_mixin_window(theme_name='warmth_dark'):
    """Return a minimal QMainWindow that mixes in HelpMixin."""
    from fic_studio.theme import apply_theme
    from fic_studio.mixins.help_mixin import HelpMixin

    class _StubWindow(QMainWindow, HelpMixin):
        """Minimal stub that satisfies HelpMixin dependencies."""

        def log_message(self, *_args):
            """Stub logger."""

    win = _StubWindow()
    apply_theme(win, theme_name)
    return win


def _build_and_keep(win):
    """
    Build the help page, set it as central widget, and return it.

    Setting the page as the central widget keeps the C++ Qt objects alive
    for the duration of the test (prevents premature GC-triggered deletion).
    """
    page = win._build_help_page()
    win.setCentralWidget(page)
    return page


# ---------------------------------------------------------------------------
# HELP_SECTIONS structure
# ---------------------------------------------------------------------------

class TestHelpSections:
    """Tests for the HELP_SECTIONS ordered list."""

    def test_has_thirteen_sections(self):
        """HELP_SECTIONS must contain exactly 13 entries."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        assert len(HELP_SECTIONS) == 13

    def test_all_required_keys_present(self):
        """All 13 required section keys must be present."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        keys = {k for k, _, __ in HELP_SECTIONS}
        required = {
            'getting_started',
            'window_layout',
            'pages',
            'control_modes',
            'extensions',
            'gamepad',
            'keyboard_shortcuts',
            'scripting',
            'processes',
            'safety',
            'troubleshooting',
            'glossary',
            'about',
        }
        missing = required - keys
        assert not missing, f'Missing section keys: {missing}'

    def test_order_starts_with_getting_started(self):
        """First section must be getting_started."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        assert HELP_SECTIONS[0][0] == 'getting_started'

    def test_order_ends_with_about(self):
        """Last section must be about."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        assert HELP_SECTIONS[-1][0] == 'about'

    def test_extensions_content_is_none(self):
        """Extensions section content slot must be None (callable resolved by mixin)."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _title, content in HELP_SECTIONS:
            if key == 'extensions':
                assert content is None
                return
        raise AssertionError('extensions section not found')

    def test_keyboard_shortcuts_content_is_none(self):
        """keyboard_shortcuts section content slot must be None."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        for key, _title, content in HELP_SECTIONS:
            if key == 'keyboard_shortcuts':
                assert content is None
                return
        raise AssertionError('keyboard_shortcuts section not found')

    def test_static_sections_have_string_content(self):
        """Non-callable sections (excluding live-generated ones) must have str content."""
        from fic_studio.mixins.help_mixin import HELP_SECTIONS
        # Sections whose content is None (resolved by mixin) or a callable
        # (generated at render time) are excluded from this check.
        callable_keys = {'extensions', 'keyboard_shortcuts', 'scripting'}
        for key, _title, content in HELP_SECTIONS:
            if key not in callable_keys:
                assert isinstance(content, str), (
                    f'Section {key!r} must have str content, got {type(content)}'
                )
                assert len(content) > 10, (
                    f'Section {key!r} content appears empty'
                )


# ---------------------------------------------------------------------------
# _build_help_page widget structure
# ---------------------------------------------------------------------------

class TestBuildHelpPage:
    """Tests for the _build_help_page() widget structure."""

    def test_returns_qwidget(self):
        """_build_help_page must return a QWidget."""
        win = _make_mixin_window()
        page = _build_and_keep(win)
        assert isinstance(page, QWidget)

    def test_contains_qlistwidget(self):
        """Page must contain a QListWidget for section navigation."""
        win = _make_mixin_window()
        page = _build_and_keep(win)
        nav_lists = page.findChildren(QListWidget)
        assert nav_lists, 'No QListWidget found in help page'

    def test_contains_qlineedit(self):
        """Page must contain a QLineEdit for search."""
        win = _make_mixin_window()
        page = _build_and_keep(win)
        search_edits = page.findChildren(QLineEdit)
        assert search_edits, 'No QLineEdit found in help page'

    def test_contains_qtextbrowser(self):
        """Page must contain a QTextBrowser for content display."""
        win = _make_mixin_window()
        page = _build_and_keep(win)
        browsers = page.findChildren(QTextBrowser)
        assert browsers, 'No QTextBrowser found in help page'

    def test_nav_has_thirteen_items(self):
        """Section nav QListWidget must have exactly 13 items."""
        win = _make_mixin_window()
        _build_and_keep(win)
        assert win._help_nav.count() == 13

    def test_first_section_selected_by_default(self):
        """After build, the first nav item must be selected."""
        win = _make_mixin_window()
        _build_and_keep(win)
        assert win._help_nav.currentRow() == 0

    def test_browser_has_content_after_build(self):
        """After build, the browser must display non-empty HTML."""
        win = _make_mixin_window()
        _build_and_keep(win)
        html = win._help_browser.toHtml()
        assert len(html) > 100, 'Browser appears to have no content'

    def test_help_search_attribute_created(self):
        """_build_help_page must set self._help_search."""
        win = _make_mixin_window()
        _build_and_keep(win)
        assert hasattr(win, '_help_search')
        assert isinstance(win._help_search, QLineEdit)

    def test_help_nav_attribute_created(self):
        """_build_help_page must set self._help_nav."""
        win = _make_mixin_window()
        _build_and_keep(win)
        assert hasattr(win, '_help_nav')
        assert isinstance(win._help_nav, QListWidget)

    def test_help_browser_attribute_created(self):
        """_build_help_page must set self._help_browser."""
        win = _make_mixin_window()
        _build_and_keep(win)
        assert hasattr(win, '_help_browser')
        assert isinstance(win._help_browser, QTextBrowser)


# ---------------------------------------------------------------------------
# Search filtering
# ---------------------------------------------------------------------------

class TestHelpSearch:
    """Tests for the search/filter behaviour."""

    def test_search_hides_non_matching_items(self):
        """Search text that matches only one section must hide the rest."""
        win = _make_mixin_window()
        _build_and_keep(win)
        total = win._help_nav.count()

        # 'Glossary' appears only in the Glossary title.
        win._help_search.setText('Glossary')

        visible = sum(
            0 if win._help_nav.item(i).isHidden() else 1
            for i in range(total)
        )
        assert visible < total, (
            f'Expected fewer than {total} visible items after search, '
            f'got {visible}'
        )
        assert visible >= 1, 'Search hid ALL items including the matching one'

    def test_empty_search_shows_all_items(self):
        """Clearing the search must restore all 13 nav items."""
        win = _make_mixin_window()
        _build_and_keep(win)
        win._help_search.setText('zzz_unlikely_match')
        win._help_search.setText('')

        hidden = sum(
            1 if win._help_nav.item(i).isHidden() else 0
            for i in range(win._help_nav.count())
        )
        assert hidden == 0, f'{hidden} items still hidden after clearing search'

    def test_search_case_insensitive(self):
        """Search must be case-insensitive."""
        win = _make_mixin_window()
        _build_and_keep(win)
        win._help_search.setText('SAFETY')

        # 'Safety' section must be visible.
        for i in range(win._help_nav.count()):
            item = win._help_nav.item(i)
            if item.data(0) == 'Safety':
                assert not item.isHidden(), 'Safety item must be visible'
                return
        raise AssertionError('Safety item not found in nav list')

    def test_search_filters_down_from_thirteen(self):
        """A restrictive search must produce fewer visible items than 13."""
        win = _make_mixin_window()
        _build_and_keep(win)
        win._help_search.setText('about')

        visible = sum(
            0 if win._help_nav.item(i).isHidden() else 1
            for i in range(win._help_nav.count())
        )
        assert visible < 13, (
            f'Expected <13 visible items for "about", got {visible}'
        )


# ---------------------------------------------------------------------------
# _help_extensions_html
# ---------------------------------------------------------------------------

class TestHelpExtensionsHtml:
    """Tests for _help_extensions_html()."""

    def test_no_extensions_names_none(self):
        """With nothing discovered or loaded, no extension is named."""
        win = _make_mixin_window()
        html = win._help_extensions_html()
        assert 'No extensions are currently loaded' in html
        assert 'Sequences' not in html

    def test_contains_gamepad_builtin_note(self):
        """Extensions HTML must mention Gamepad as a built-in."""
        win = _make_mixin_window()
        html = win._help_extensions_html()
        assert 'Gamepad' in html

    def test_uses_live_instances_when_available(self):
        """When _extension_instances is set, must use live data over catalog."""
        win = _make_mixin_window()

        class _FakeExt:
            """Minimal fake extension instance."""

            NAME = 'TestExtLive'
            DESCRIPTION = 'A live test extension.'
            SIDEBAR_SECTION = 'Extensions'

        win._extension_instances = {'test_ext': _FakeExt()}
        html = win._help_extensions_html()
        assert 'TestExtLive' in html, (
            'Expected live instance NAME in extensions HTML'
        )

    def test_uses_discovered_manifests(self):
        """Discovered manifests are listed, even when none is loaded."""
        win = _make_mixin_window()

        class _FakeManifest:
            """Minimal fake manifest."""

            name = 'Script Editor'
            description = 'Write and run Python.'
            sidebar_section = 'Programming'

        win._extensions_manifests = [_FakeManifest()]
        html = win._help_extensions_html()
        assert 'Script Editor' in html
        assert 'Programming' in html

    def test_returns_string(self):
        """_help_extensions_html must return a str."""
        win = _make_mixin_window()
        result = win._help_extensions_html()
        assert isinstance(result, str)


# ---------------------------------------------------------------------------
# _help_shortcuts_html
# ---------------------------------------------------------------------------

class TestHelpShortcutsHtml:
    """Tests for _help_shortcuts_html()."""

    def test_contains_estop_description(self):
        """Shortcuts HTML must contain 'E-Stop'."""
        win = _make_mixin_window()
        html = win._help_shortcuts_html()
        assert 'E-Stop' in html, (
            'Expected "E-Stop" in shortcuts HTML'
        )

    def test_contains_space_binding(self):
        """Default E-Stop binding 'Space' must appear in shortcuts HTML."""
        win = _make_mixin_window()
        html = win._help_shortcuts_html()
        assert 'Space' in html, (
            'Expected "Space" key in shortcuts HTML'
        )

    def test_contains_f1(self):
        """F1 (Open documentation) must appear in shortcuts HTML."""
        win = _make_mixin_window()
        html = win._help_shortcuts_html()
        assert 'F1' in html

    def test_contains_ctrl_comma(self):
        """Ctrl+, (Settings) must appear in shortcuts HTML."""
        win = _make_mixin_window()
        html = win._help_shortcuts_html()
        assert 'Ctrl+,' in html

    def test_returns_string(self):
        """_help_shortcuts_html must return a str."""
        win = _make_mixin_window()
        result = win._help_shortcuts_html()
        assert isinstance(result, str)

    def test_reflects_live_settings(self):
        """When settings has a custom E-Stop binding, it must appear."""
        win = _make_mixin_window()
        win.settings = {'shortcuts': {'estop': 'Ctrl+E'}}
        html = win._help_shortcuts_html()
        assert 'Ctrl+E' in html, (
            'Expected custom estop binding "Ctrl+E" in shortcuts HTML'
        )


# ---------------------------------------------------------------------------
# _help_themed_html
# ---------------------------------------------------------------------------

class TestHelpThemedHtml:
    """Tests for _help_themed_html()."""

    def test_injects_theme_colour(self):
        """Output must contain a colour value from the current theme tokens."""
        from fic_studio import theme
        win = _make_mixin_window()
        result = win._help_themed_html('<p>Test</p>')
        tokens = theme.current()
        # Check that at least the bg_base colour appears in the style block.
        assert tokens['bg_base'] in result, (
            f'Expected bg_base colour {tokens["bg_base"]!r} in themed HTML'
        )

    def test_wraps_body_in_html_structure(self):
        """Output must be a complete HTML document with <html> and <body>."""
        win = _make_mixin_window()
        result = win._help_themed_html('<p>Hello</p>')
        assert '<html>' in result
        assert '<body>' in result
        assert '<p>Hello</p>' in result

    def test_contains_style_block(self):
        """Output must contain a <style> block."""
        win = _make_mixin_window()
        result = win._help_themed_html('<p>x</p>')
        assert '<style>' in result

    def test_dark_theme_uses_dark_bg(self):
        """warmth_dark theme must inject the dark bg_base colour."""
        from fic_studio.theme.tokens import WARMTH_DARK
        win = _make_mixin_window('warmth_dark')
        result = win._help_themed_html('<p>x</p>')
        assert WARMTH_DARK['bg_base'] in result

    def test_light_theme_uses_light_bg(self):
        """warmth_light theme must inject the light bg_base colour."""
        from fic_studio.theme.tokens import WARMTH_LIGHT
        win = _make_mixin_window('warmth_light')
        result = win._help_themed_html('<p>x</p>')
        assert WARMTH_LIGHT['bg_base'] in result


# ---------------------------------------------------------------------------
# Render spot-checks (both themes)
# ---------------------------------------------------------------------------

class TestRenderBothThemes:
    """Spot-check that a couple of sections render non-empty HTML in both themes."""

    def _render_section(self, win, key: str) -> str:
        """Render a section and return the browser HTML."""
        win._help_render_section(key)
        return win._help_browser.toHtml()

    def test_dark_getting_started_non_empty(self):
        """warmth_dark: getting_started section must produce non-empty HTML."""
        win = _make_mixin_window('warmth_dark')
        _build_and_keep(win)
        html = self._render_section(win, 'getting_started')
        assert 'Getting Started' in html

    def test_dark_control_modes_non_empty(self):
        """warmth_dark: control_modes section must mention 'Joint Impedance'."""
        win = _make_mixin_window('warmth_dark')
        _build_and_keep(win)
        html = self._render_section(win, 'control_modes')
        assert 'Joint Impedance' in html

    def test_light_getting_started_non_empty(self):
        """warmth_light: getting_started section must produce non-empty HTML."""
        win = _make_mixin_window('warmth_light')
        _build_and_keep(win)
        html = self._render_section(win, 'getting_started')
        assert 'Getting Started' in html

    def test_light_extensions_non_empty(self):
        """warmth_light: the extensions section renders its heading."""
        win = _make_mixin_window('warmth_light')
        _build_and_keep(win)
        html = self._render_section(win, 'extensions')
        assert 'Extensions' in html and 'Gamepad' in html

    def test_dark_shortcuts_render(self):
        """warmth_dark: keyboard_shortcuts section must mention 'Space'."""
        win = _make_mixin_window('warmth_dark')
        _build_and_keep(win)
        html = self._render_section(win, 'keyboard_shortcuts')
        assert 'Space' in html

    def test_light_safety_render(self):
        """warmth_light: safety section must mention 'E-STOP'."""
        win = _make_mixin_window('warmth_light')
        _build_and_keep(win)
        html = self._render_section(win, 'safety')
        assert 'E-STOP' in html

    def test_nav_click_changes_browser_content(self):
        """Clicking a different nav item must update browser content."""
        win = _make_mixin_window()
        _build_and_keep(win)
        # Select getting_started.
        win._help_nav.setCurrentRow(0)
        html_first = win._help_browser.toHtml()
        # Select about (last).
        win._help_nav.setCurrentRow(win._help_nav.count() - 1)
        html_last = win._help_browser.toHtml()
        assert html_first != html_last, (
            'Browser content must change when nav selection changes'
        )
