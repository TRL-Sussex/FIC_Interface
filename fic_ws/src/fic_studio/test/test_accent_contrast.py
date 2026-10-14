"""Accent buttons must be readable in every theme.

The existing contrast test guards body text on surfaces. Nothing guarded text
drawn on a saturated fill, so 14 of 36 accent-button x theme pairs shipped
below WCAG AA -- including BtnEStop, the most safety-critical control.

Root cause is the recolour engine: it resolves each hex by nearest RGB, so
BtnPrimary's #1a5fb4 lands on the bg_selected *surface* token and the button
paints its fill and its label the same colour.

KNOWN_FAILURES is a burn-down list, not an exemption. It may shrink, never
grow. The remainder need text_on_accent split into per-luminance roles, which
is step 3 of the design-system spec -- one token cannot be both dark-on-amber
and light-on-navy.
"""
import re

import pytest

from fic_studio.theme.stylesheet import build_stylesheet
from fic_studio.theme.tokens import THEMES

ACCENT_ROLES = ['BtnPrimary', 'BtnTeal', 'BtnGreen', 'BtnWarn', 'BtnRed',
                'BtnStop', 'BtnEStop']

#: (theme, role) pairs still below AA. Closed by the token-role split: one
#: text_on_accent cannot be both dark-on-amber and light-on-navy.
#: The shipped default (warmth_dark) is clean.
KNOWN_FAILURES = {
    # Outline buttons now draw their label in the state colour on the canvas;
    # these legacy (non-default) palettes have a state colour too pale for
    # that. The shipped default, warmth_dark, is clean.
    ('warmth_light', 'BtnWarn'), ('warmth_light', 'BtnGreen'),
    ('warmth_light', 'BtnTeal'), ('cool_dark', 'BtnEStop'),
    ('cool_light', 'BtnWarn'),
    ('cool_light', 'BtnGreen'),
}

#: Below this a control is not "hard to read", it is invisible -- fill and
#: label the same colour. Nothing may ever ship here again.
INVISIBLE = 2.0


def _luminance(value):
    """WCAG relative luminance of a #rrggbb string."""
    raw = value.lstrip('#')
    channels = [int(raw[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    linear = [(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
              for c in channels]
    return 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2]


def _contrast(first, second):
    """WCAG contrast ratio between two #rrggbb strings."""
    a, b = _luminance(first), _luminance(second)
    return (max(a, b) + 0.05) / (min(a, b) + 0.05)


def _pair(qss, role, canvas):
    """Return the (background, colour) a role renders with, or None.

    Outline buttons have a transparent fill, so their label sits on the
    canvas; solid ones (BtnEStop) carry their own.
    """
    match = re.search(r'QPushButton#%s\s*\{([^}]*)\}' % role, qss)
    if not match:
        return None
    body = match.group(1)
    bg = re.search(r'background(?:-color)?:\s*(#[0-9a-fA-F]{6})', body)
    fg = re.search(r'(?<!-)\bcolor:\s*(#[0-9a-fA-F]{6})', body)
    if not fg:
        return None
    return (bg.group(1) if bg else canvas, fg.group(1))


def _measure():
    """Return {(theme, role): ratio} for every accent button in every theme."""
    out = {}
    for theme in THEMES:
        qss = build_stylesheet(theme)
        for role in ACCENT_ROLES:
            pair = _pair(qss, role, THEMES[theme]['bg_base'])
            if pair:
                out[(theme, role)] = _contrast(*pair)
    return out


@pytest.fixture(scope='module')
def measured():
    """Contrast of every accent button in every theme."""
    return _measure()


def test_nothing_is_effectively_invisible(measured):
    """No control may paint its fill and its label the same colour.

    BtnPrimary did exactly that in four themes -- 1.23:1 in editorial_light --
    because nearest-RGB resolution sent a saturated fill to the bg_selected
    *surface* token its hex collides with.
    """
    invisible = {k: round(v, 2) for k, v in measured.items() if v < INVISIBLE}
    assert not invisible, 'controls rendering as one flat colour: %s' % invisible


def test_the_shipped_default_theme_is_compliant(measured):
    """Whatever the burn-down list still holds, the default must be clean."""
    from fic_studio.theme.tokens import DEFAULT_THEME
    bad = {k: round(v, 2) for k, v in measured.items()
           if k[0] == DEFAULT_THEME and v < 4.5}
    assert not bad, '%s ships failing accent buttons: %s' % (DEFAULT_THEME, bad)


def test_no_new_contrast_failures(measured):
    """The burn-down list may shrink, never grow."""
    failing = {k for k, v in measured.items() if v < 4.5}
    new = failing - KNOWN_FAILURES
    assert not new, 'new accent contrast failures: %s' % sorted(new)


def test_known_failures_are_still_real(measured):
    """A fixed pair must be removed from the list, not left to rot."""
    fixed = {k for k in KNOWN_FAILURES if measured.get(k, 0) >= 4.5}
    assert not fixed, (
        'these now pass and must come off KNOWN_FAILURES: %s' % sorted(fixed))


# ── Text drawn on a neutral surface, not on an accent fill ─────────────────

def _rule(qss, selector):
    """Return the declaration body of one rule, or ''."""
    match = re.search(re.escape(selector) + r'\s*\{([^}]*)\}', qss)
    return match.group(1) if match else ''


def _colour(body, prop='color'):
    """First #rrggbb for a property in a declaration body."""
    pattern = (r'background(?:-color)?:\s*(#[0-9a-fA-F]{6})' if prop == 'background'
               else r'(?<!-)\bcolor:\s*(#[0-9a-fA-F]{6})')
    found = re.search(pattern, body)
    return found.group(1) if found else None


def test_active_navigation_is_visible_in_every_theme():
    """The selected sidebar entry must be readable on the sidebar.

    It was drawn with the token for text on a *saturated accent fill*, but it
    sits on a neutral surface under an 8% wash. In editorial_dark that token
    is near-black on a near-black sidebar: 1.00:1 -- the selected page was
    invisible in the shipped default theme.
    """
    failures = {}
    for theme in THEMES:
        qss = build_stylesheet(theme)
        surface = _colour(_rule(qss, '#Sidebar'), 'background')
        label = _colour(_rule(qss, '#SideButton[active="true"]'))
        if not (surface and label):
            continue
        ratio = _contrast(surface, label)
        if ratio < 4.5:
            failures[theme] = round(ratio, 2)
    assert not failures, 'active nav unreadable: %s' % failures


def test_active_navigation_outranks_the_inactive_state():
    """Selection must read as *more* prominent, not merely different."""
    for theme in THEMES:
        qss = build_stylesheet(theme)
        surface = _colour(_rule(qss, '#Sidebar'), 'background')
        active = _colour(_rule(qss, '#SideButton[active="true"]'))
        resting = _colour(_rule(qss, '#SideButton'))
        if not (surface and active and resting):
            continue
        assert _contrast(surface, active) >= _contrast(surface, resting), (
            '%s: the selected entry is less prominent than an unselected one'
            % theme)
