"""Tests for the runtime theme engine."""
from fic_studio import theme
from PyQt5.QtWidgets import QWidget


def test_apply_sets_stylesheet_and_current(qapp):
    """apply_theme styles the window and updates current()."""
    w = QWidget()
    theme.apply_theme(w, 'warmth_light')
    assert theme.current_name() == 'warmth_light'
    assert theme.current()['bg_base'] == '#F4EEE4'
    assert w.styleSheet()  # non-empty
    assert '#f4eee4' in w.styleSheet().lower()


def test_legacy_name_normalizes(qapp):
    """A legacy 'dark' name resolves to warmth_dark."""
    w = QWidget()
    assert theme.apply_theme(w, 'dark') == 'warmth_dark'
    assert theme.current_name() == 'warmth_dark'


def test_theme_changed_signal_emits(qapp):
    """apply_theme emits theme_changed with the normalized name."""
    seen = []
    theme.signals.theme_changed.connect(seen.append)
    theme.apply_theme(QWidget(), 'light')
    assert seen and seen[-1] == 'warmth_light'
