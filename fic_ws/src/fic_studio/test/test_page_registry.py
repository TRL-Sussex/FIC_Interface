"""Pages are addressed by name, not by position in the stack.

Indices were hardcoded in three independent places: the order of
main_stack.addWidget, a positional _side_buttons list with None holes at
indices 3-8 and 12, and four periodic refreshers that throttle themselves by
comparing currentIndex() to a literal. Nothing tied them together, so removing
a page -- there are seven empty placeholder slots -- silently sent navigation
to the wrong page and left the throttles pointing at whatever moved into their
index.

Naming only the navigation half would have been worse than leaving it alone:
"which page is showing" would then have had two representations that drift.
"""
import os
import re
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import (  # noqa: E402
    QApplication, QStackedWidget, QWidget,
)

from fic_studio.pages import PageRegistry  # noqa: E402

# Module-level reference: a QApplication that is only a local goes out of
# scope and Qt takes the interpreter down with it.
_app = QApplication.instance() or QApplication([])

SRC = Path(__file__).resolve().parents[1] / 'fic_studio'


def _sources():
    """Every module that could couple itself to a stack position."""
    return [p for p in SRC.rglob('*.py') if '__pycache__' not in str(p)]


def test_no_literal_setcurrentindex_remains():
    """Navigation goes through the registry, not through a number."""
    offenders = []
    for path in _sources():
        for n, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            if re.search(r'\bmain_stack\.setCurrentIndex\(\s*\d+\s*\)', line):
                offenders.append('%s:%d' % (path.name, n))
    assert not offenders, 'literal stack navigation: %s' % offenders


def test_no_literal_currentindex_comparison_remains():
    """Refresh throttles ask whether their page is showing, by name."""
    offenders = []
    for path in _sources():
        for n, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
            # Only page stacks. Combo boxes and tab widgets legitimately
            # compare their own currentIndex to a number.
            if re.search(r'\b(main_stack|stack)\.currentIndex\(\)\s*[!=]=\s*\d+', line):
                offenders.append('%s:%d' % (path.name, n))
    assert not offenders, 'positional page throttles: %s' % offenders


def test_registry_round_trips_every_registered_page():
    """Registering then showing a page must land on that exact widget."""
    stack = QStackedWidget()
    stack.show()                     # "showing" means on screen (test_hidden_window_idle.py)
    registry = PageRegistry(stack)

    made = {}
    for name in ('dashboard', 'tuning', 'settings'):
        widget = QWidget()
        made[name] = widget
        registry.register(name, widget)

    for name, widget in made.items():
        assert registry.show(name) is True
        assert stack.currentWidget() is widget, name
        assert registry.is_visible(name) is True
        for other in made:
            if other != name:
                assert registry.is_visible(other) is False


def test_unknown_page_is_reported_not_guessed():
    """A typo must fail loudly rather than land on page zero."""
    stack = QStackedWidget()
    registry = PageRegistry(stack)
    registry.register('dashboard', QWidget())

    assert registry.show('dashbord') is False
    assert registry.is_visible('dashbord') is False
    assert stack.currentIndex() == 0


def test_registry_survives_pages_being_added_in_any_order():
    """Names, not positions -- so insertion order carries no meaning."""
    stack = QStackedWidget()
    registry = PageRegistry(stack)

    late = QWidget()
    registry.register('first', QWidget())
    registry.register('second', QWidget())
    registry.register('late', late)

    assert registry.show('late') is True
    assert stack.currentWidget() is late
