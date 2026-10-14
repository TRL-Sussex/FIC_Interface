"""Invariants for the structural stylesheet.

styles.py is a single QSS that theme/stylesheet.py recolours per theme, so
every non-colour property is shared by all six themes at once. These assert
properties the sheet must hold, not a snapshot of what it happens to contain.
"""
from pathlib import Path

import pytest

from fic_studio.theme.stylesheet import build_stylesheet
from fic_studio.theme.tokens import THEMES

STYLES = Path(__file__).resolve().parents[1] / 'fic_studio' / 'styles.py'


@pytest.fixture(scope='module')
def source():
    """The stylesheet module's text."""
    return STYLES.read_text(encoding='utf-8')


def test_no_drop_shadows(source):
    """Flat by construction — a shadow here would be the only one in the app."""
    assert 'box-shadow' not in source


def test_font_weight_is_always_numeric(source):
    """`bold` and `600` in one sheet is a single scale with two spellings."""
    assert 'font-weight: bold' not in source, (
        "use 700 rather than 'bold' so the weight scale reads as one system")


def test_no_unreferenced_theme_literal(source):
    """Only DARK_THEME is structural; a second literal is dead weight.

    LIGHT_THEME was left unreferenced when the token engine landed, as an
    explicit later-cleanup item. Half the file was a copy nothing read, which
    silently doubled every metric and kept a dropdown-arrow bug alive that the
    changelog recorded as fixed.
    """
    assert 'LIGHT_THEME' not in source
    assert 'COLORS_LIGHT' not in source


def test_engine_builds_every_theme(source):
    """The recolour pass must produce a distinct, non-trivial sheet per theme."""
    built = {name: build_stylesheet(name) for name in THEMES}
    for name, qss in built.items():
        assert len(qss) > 10_000, '%s produced only %d chars' % (name, len(qss))
    assert len(set(built.values())) == len(THEMES), 'two themes render identically'


def test_no_dropdown_arrow_triangle(source):
    """Qt does not honour the CSS triangle technique on subcontrols.

    ::down-arrow drawn with transparent side borders renders as a solid
    square. Neither ::drop-down nor ::down-arrow may be styled — they go
    together, and removing only one makes Qt draw no indicator at all.
    """
    import re
    # A rule, not a mention: the comment recording *why* these are unstyled is
    # the knowledge that stops someone re-adding them, and must survive.
    for subcontrol in ('down-arrow', 'drop-down'):
        rule = re.search(r'^[^\n#]*::%s[^\n]*\{' % subcontrol, source, re.M)
        assert rule is None, 'a ::%s rule is back: %s' % (subcontrol, rule.group(0))
