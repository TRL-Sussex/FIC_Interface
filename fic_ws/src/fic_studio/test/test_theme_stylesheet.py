"""Tests for the hex-substitution stylesheet engine."""
from fic_studio.styles import COLORS_DARK
from fic_studio.theme.stylesheet import (
    build_stylesheet,
    scale_overlay_qss,
    unmapped_hexes,
)
from fic_studio.theme.tokens import THEMES


def test_no_canonical_dark_hex_survives_in_either_theme():
    """Every COLORS_DARK hex is recolored away in both themes."""
    for theme in ('warmth_dark', 'warmth_light'):
        out = build_stylesheet(theme).lower()
        for hex_value in COLORS_DARK.values():
            assert hex_value.lower() not in out, (theme, hex_value)


def test_warm_values_are_present():
    """Recolored output contains the active theme's base colors."""
    assert '#1b1714' in build_stylesheet('warmth_dark').lower()
    assert '#f4eee4' in build_stylesheet('warmth_light').lower()


def test_legacy_name_is_accepted():
    """A legacy 'dark' name routes through normalization."""
    assert build_stylesheet('dark') == build_stylesheet('warmth_dark')


def test_scale_overlay_is_appended():
    """The scale overlay is concatenated after the base QSS."""
    out = build_stylesheet('warmth_dark', scale_overlay_qss(150))
    assert out.endswith(scale_overlay_qss(150))
    assert 'font-size: 18px' in out  # 12 * 1.5


def test_every_stray_hex_resolves_via_fallback():
    """unmapped_hexes reports strays but build_stylesheet leaves none raw."""
    strays = unmapped_hexes()
    out = build_stylesheet('warmth_light').lower()
    for hex_value in strays:
        assert hex_value.lower() not in out, hex_value


# ── Dropdowns match the menus ─────────────────────────────────────────────────

def _rule(qss, selector):
    """Return the body of the first rule whose selector starts a line."""
    import re
    match = re.search(r'(?m)^%s \{([^}]*)\}' % re.escape(selector), qss)
    assert match, 'no rule for %s' % selector
    return match.group(1)


def _declaration(body, prop):
    """Return one declaration's value from a rule body."""
    import re
    match = re.search(r'%s:\s*([^;]+);' % re.escape(prop), body)
    return match.group(1).strip() if match else None


def test_an_open_dropdown_matches_the_context_menu():
    """
    An open combo is a menu by another name and must not read as a different
    kind of surface.  It used to sit on bg_overlay while QMenu sat on
    bg_raised, so the two popped up in noticeably different greys.
    """
    for name in THEMES:
        qss = build_stylesheet(name)
        menu = _declaration(_rule(qss, 'QMenu'), 'background-color')
        # QToolBar-scoped included on purpose: it is a *more specific*
        # selector, so it wins outright for the toolbar's robot pickers. The
        # first pass at this fix updated only the plain rule and those two
        # dropdowns carried on opening in the old colour.
        for selector in ('QComboBox QAbstractItemView', 'QComboBox QListView',
                         'QToolBar QComboBox QAbstractItemView'):
            popup = _declaration(_rule(qss, selector), 'background-color')
            assert popup == menu, (
                '%s: %s is %s but QMenu is %s' % (name, selector, popup, menu))


def test_the_dropdown_indicator_is_left_to_the_style():
    """
    Qt's stylesheet engine does not support the CSS triangle technique.

    ``border-left/right: transparent`` plus a solid ``border-top`` is how a
    triangle is drawn on the web; Qt sizes the subcontrol normally and paints
    the box, which is why every combo showed a small solid square. Styling
    ``::drop-down`` without ``::down-arrow`` is worse — Qt then draws no
    indicator at all — so neither may carry a rule.
    """
    for name in THEMES:
        qss = build_stylesheet(name)
        for subcontrol in ('QComboBox::down-arrow', 'QComboBox::drop-down'):
            assert '%s {' % subcontrol not in qss, (
                '%s: %s is styled again; the two must be left to the active '
                'style or the arrow disappears' % (name, subcontrol))
