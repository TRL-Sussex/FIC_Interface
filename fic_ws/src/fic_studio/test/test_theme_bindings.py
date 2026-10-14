"""
Regression tests for the theme binding engine (theme.bind / _reapply_bindings).

The engine keeps a global list of (weakref(widget), factory) bindings and
re-applies them on every theme switch. A widget's Python wrapper can outlive
its underlying C++ object (a closed dialog, a rebuilt page), so a re-apply must
not crash on such a stale entry — it must skip and drop it.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio import theme  # noqa: E402
from PyQt5 import sip  # noqa: E402
from PyQt5.QtWidgets import (  # noqa: E402
    QApplication, QLabel, QPushButton, QWidget,
)

_app = QApplication.instance() or QApplication([])


def test_reapply_skips_deleted_widget_without_crashing():
    """
    A theme switch survives a bound widget whose C++ object was deleted.

    Regression: _reapply_bindings used to only check ``widget is not None``
    (the weakref), so a deleted-but-wrapped widget raised RuntimeError and
    crashed the whole theme switch.
    """
    btn = QPushButton()
    theme.bind(btn, lambda t: 'color: %s;' % t['text_primary'])

    # Destroy the underlying C++ object; the Python wrapper lingers in _bindings.
    sip.delete(btn)
    assert sip.isdeleted(btn)

    # Must not raise even though a bound widget's C++ side is gone.
    theme._reapply_bindings()

    # No surviving binding points to a deleted widget — the dead one was pruned.
    for ref, _ in theme._bindings:
        widget = ref()
        assert widget is None or not sip.isdeleted(widget)


def test_live_binding_is_reapplied_on_theme_switch():
    """A live bound widget is still restyled to the active theme's tokens."""
    win = QWidget()
    lbl = QLabel()
    theme.bind(lbl, lambda t: 'color: %s;' % t['accent_blue'])

    theme.apply_theme(win, 'warmth_light')
    assert theme.current()['accent_blue'] in lbl.styleSheet()

    theme.apply_theme(win, 'warmth_dark')
    assert theme.current()['accent_blue'] in lbl.styleSheet()


def test_deleted_widget_does_not_break_later_live_binding():
    """A dead binding must not stop later live bindings from being applied."""
    dead = QPushButton()
    theme.bind(dead, lambda t: 'color: %s;' % t['accent_red'])
    sip.delete(dead)

    win = QWidget()
    live = QLabel()
    theme.bind(live, lambda t: 'color: %s;' % t['accent_green'])

    # The switch processes the dead binding (skipping it) and the live one.
    theme.apply_theme(win, 'warmth_light')
    assert theme.current()['accent_green'] in live.styleSheet()

    # Leave the engine on the shipped default for any later tests.
    theme.apply_theme(win, 'warmth_dark')


# ── Widget-level combo stylesheets ────────────────────────────────────────────
# A widget's own stylesheet propagates to its children, and a combo's drop-down
# list is one of them. Styling only the closed box therefore restyles the open
# list too — that is how the toolbar's robot picker ended up bold, clipped and
# on the wrong surface while every other dropdown matched the menus.

import re as _re  # noqa: E402
import pathlib as _pathlib  # noqa: E402

from fic_studio import theme as _theme_mod  # noqa: E402
from fic_studio.theme.tokens import THEMES as _THEMES  # noqa: E402


def _studio_sources():
    """Yield every .py file of the Studio package."""
    root = _pathlib.Path(_theme_mod.__file__).resolve().parent.parent
    for path in sorted(root.rglob('*.py')):
        if '__pycache__' in path.parts:
            continue
        yield path


def test_combo_popup_qss_puts_the_list_on_the_menu_surface():
    """The helper must hand back the same surface QMenu uses."""
    for name, tokens in _THEMES.items():
        qss = _theme_mod.combo_popup_qss(tokens)
        assert tokens['bg_raised'] in qss, name
        assert tokens['border_medium'] in qss, name
        assert 'font-weight: 400' in qss, name


def test_no_widget_stylesheet_styles_a_combo_without_styling_its_popup():
    """
    Whoever writes ``QComboBox { ... }`` onto a widget owns its popup too.

    The application stylesheet cannot rescue it — the widget's own sheet wins —
    so every such site has to append ``theme.combo_popup_qss``.
    """
    offenders = []
    for path in _studio_sources():
        if path.name in ('styles.py', 'stylesheet.py', '__init__.py'):
            # styles.py and stylesheet.py build the *application* stylesheet,
            # which does not propagate into a widget's popup the way a widget's
            # own sheet does; __init__.py is where the helper itself lives.
            continue
        text = path.read_text(encoding='utf-8')
        for match in _re.finditer(r"'QComboBox \{", text):
            window = text[match.start():match.start() + 900]
            if 'combo_popup_qss' not in window:
                line = text[:match.start()].count('\n') + 1
                offenders.append('%s:%d' % (path.name, line))
    assert not offenders, (
        'these style a combo box but leave its drop-down list to inherit: %s'
        % ', '.join(offenders))


def test_the_toolbar_robot_picker_does_not_set_a_font_weight():
    """
    Bold cannot be undone from the list side.

    Neither ``QComboBox QAbstractItemView`` nor ``::item`` overrides a font
    inherited from the parent widget, so a weight set on the box reaches the
    popup and nothing in the stylesheet can take it back off.
    """
    root = _pathlib.Path(_theme_mod.__file__).resolve().parent.parent
    text = (root / 'main_window_pro.py').read_text(encoding='utf-8')
    start = text.index('theme.bind(self._tb_robot_id')
    block = text[start:text.index('self._tb_robot_id.view()', start)]
    assert 'font-weight' not in block, (
        'the toolbar robot picker sets a font weight again; it will reappear '
        'in the drop-down list')
