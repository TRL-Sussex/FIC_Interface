"""Runtime theme engine: token access, application, and change signal."""
import weakref

from fic_studio.theme.stylesheet import build_stylesheet
from fic_studio.theme.tokens import DEFAULT_THEME, normalize_theme_name, THEMES
from PyQt5.QtCore import pyqtSignal, QObject


class _ThemeSignals(QObject):
    """Holds the module-level theme_changed signal."""

    theme_changed = pyqtSignal(str)


signals = _ThemeSignals()
_current_name = DEFAULT_THEME


def current_name():
    """Return the active normalized theme name."""
    return _current_name


def current():
    """Return the active theme's token dict (for painted widgets)."""
    return THEMES[_current_name]


def apply_theme(window, name, scale_overlay=''):
    """Style ``window`` with theme ``name`` and broadcast the change."""
    global _current_name
    _current_name = normalize_theme_name(name)
    window.setStyleSheet(build_stylesheet(_current_name, scale_overlay))
    signals.theme_changed.emit(_current_name)
    _reapply_bindings()
    return _current_name


_bindings = []


def combo_popup_qss(tokens):
    """
    Return the popup rules a widget-level ``QComboBox`` stylesheet must carry.

    Qt propagates a widget's own stylesheet to its children, and a combo's drop
    -down list is one of them.  So a rule written to style just the closed box
    — ``QComboBox { font-weight: 600; border: ...; }`` — also lands on the open
    list, which is how the toolbar's robot picker ended up with bold, clipped
    text inside a pale frame while every other dropdown looked like a menu.
    The application stylesheet cannot fix it: the widget's own sheet wins.

    Appending this to such a stylesheet puts the list back on the same surface,
    border and row spacing as ``QMenu``, and resets the weight the closed box
    asked for.  Nothing needs it unless it sets a widget-level sheet.
    """
    return (
        'QComboBox QAbstractItemView {'
        ' background-color: %(bg_raised)s;'
        ' border: 1px solid %(border_medium)s;'
        ' color: %(text_primary)s;'
        ' selection-background-color: %(bg_selected)s;'
        ' selection-color: %(text_on_accent)s;'
        ' outline: none; padding: 4px 0; font-weight: 400; }'
        'QComboBox QAbstractItemView::item {'
        ' padding: 8px 24px; min-height: 24px;'
        ' background-color: transparent; border: none; }'
    ) % tokens


def bind(widget, factory):
    """Apply factory(tokens) to widget now and re-apply on every theme switch."""
    widget.setStyleSheet(factory(current()))
    _bindings.append((weakref.ref(widget), factory))
    return widget


def _reapply_bindings(*_):
    """Re-apply every live binding's style for the active theme.

    A widget's Python wrapper can outlive its underlying C++ object (a closed
    dialog, a rebuilt page), so ``ref()`` may return a wrapper whose C++ side
    is already deleted. Calling ``setStyleSheet`` on it raises ``RuntimeError``
    — guard against that and drop the dead binding so a stale entry can never
    crash a theme switch or leak unbounded.
    """
    tokens = current()
    live = []
    for ref, factory in _bindings:
        widget = ref()
        if widget is None:
            continue
        try:
            widget.setStyleSheet(factory(tokens))
        except RuntimeError:
            continue  # underlying C++ object deleted — drop the binding
        live.append((ref, factory))
    _bindings[:] = live


# ── Widget-level button styles ───────────────────────────────────────────────
# A QToolBar re-applies its own styling over objectName rules, so toolbar
# buttons carry their style inline.  These helpers keep that inline QSS made
# of tokens and quiet by default.

_BTN_BASE = ('border-radius: 5px; font-size: 11px; font-weight: 500;'
             ' padding: 0px 12px;')


def button_qss(t, kind='secondary', colour=None):
    """
    Return a widget-level ``QPushButton`` QSS for a compact button.

    ``kind``: ``secondary`` (neutral outline), ``accent`` (outline and label
    in ``colour``, default the accent), ``quiet`` (bare text) or ``safety``
    (solid, saturated: E-stop only).
    """
    from fic_studio.theme.design import derive, wash
    d = derive(t)
    if kind == 'safety':
        return (
            'QPushButton { background-color: %s; border: 1px solid %s;'
            ' color: %s; font-weight: 700; %s }'
            'QPushButton:hover { background-color: %s; }'
            % (d['safety'], d['safety'], d['safety_text'],
               _BTN_BASE.replace('font-weight: 500;', ''), d['safety_hover']))
    if kind == 'accent':
        colour = colour or d['accent']
        return (
            'QPushButton { background-color: transparent; border: 1px solid %s;'
            ' color: %s; %s }'
            'QPushButton:hover { background-color: %s; }'
            'QPushButton:disabled { color: %s; border-color: %s; }'
            % (colour, colour, _BTN_BASE, wash(colour), d['ink_faint'],
               d['rule']))
    if kind == 'quiet':
        return (
            'QPushButton { background-color: transparent;'
            ' border: 1px solid transparent; color: %s; %s }'
            'QPushButton:hover { background-color: %s; color: %s; }'
            % (d['ink_muted'], _BTN_BASE, d['hover'], d['ink_strong']))
    return (
        'QPushButton { background-color: transparent; border: 1px solid %s;'
        ' color: %s; %s }'
        'QPushButton:hover { background-color: %s; color: %s; }'
        'QPushButton:disabled { color: %s; border-color: %s; }'
        % (d['rule_strong'], d['ink'], _BTN_BASE, d['hover'], d['ink_strong'],
           d['ink_faint'], d['rule']))
