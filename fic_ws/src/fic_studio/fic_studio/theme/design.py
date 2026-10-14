"""
FIC Studio design tokens: the scales and the derived colours.

``theme/tokens.py`` holds the seventeen palette slots every theme fills.  This
module adds what a *design system* needs on top of them and what a palette
cannot say by itself: the type scale, spacing, radii, and the derived colours
(hover and selection washes, state tints).  ``derive(palette)`` returns one
flat dict that ``styles.py`` fills its template from and that painted widgets
can read.
"""

# ── Scales (px) ──────────────────────────────────────────────────────────────
SPACE = {'1': 4, '2': 8, '3': 12, '4': 16, '5': 24, '6': 32}
RADIUS = {'sm': 3, 'md': 5, 'lg': 8, 'pill': 10}
FONT_SIZE = {'caption': 10, 'small': 11, 'body': 12, 'lead': 13,
             'h3': 15, 'h2': 18, 'h1': 24}
WEIGHT = {'regular': 400, 'medium': 500, 'strong': 600}

FONT_SANS = ("'Inter', 'SF Pro Text', 'Helvetica Neue', 'Segoe UI', 'Ubuntu', "
             "'Roboto', sans-serif")
FONT_SERIF = ("'Instrument Serif', 'Newsreader', 'Lyon Text', Georgia, "
              "'DejaVu Serif', serif")
FONT_MONO = ("'JetBrains Mono', 'SF Mono', 'Menlo', 'Consolas', "
             "'DejaVu Sans Mono', monospace")


def _rgb(value):
    """Return the (r, g, b) ints of a ``#rrggbb`` string."""
    raw = value.lstrip('#')
    return int(raw[0:2], 16), int(raw[2:4], 16), int(raw[4:6], 16)


def mix(first, second, amount):
    """Return ``first`` blended ``amount`` (0..1) of the way to ``second``."""
    a, b = _rgb(first), _rgb(second)
    return '#%02x%02x%02x' % tuple(
        round(x + (y - x) * amount) for x, y in zip(a, b))


def rgba(value, alpha):
    """Return a QSS ``rgba()`` colour for ``value`` at ``alpha`` (0..1)."""
    r, g, b = _rgb(value)
    return 'rgba(%d, %d, %d, %.2f)' % (r, g, b, alpha)


def _luminance(value):
    """Return the 0..1 relative luminance of ``value``."""
    chans = [c / 255.0 for c in _rgb(value)]
    lin = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
           for c in chans]
    return 0.2126 * lin[0] + 0.7152 * lin[1] + 0.0722 * lin[2]


def is_dark(palette):
    """Return True when ``palette``'s canvas is a dark one."""
    return _luminance(palette['bg_base']) < 0.2


def _legible(colour, canvas, pole, floor=4.5):
    """``colour`` nudged toward ``pole`` until it reads on ``canvas`` (WCAG ``floor``)."""
    def ratio(a, b):
        hi, lo = sorted((_luminance(a), _luminance(b)), reverse=True)
        return (hi + 0.05) / (lo + 0.05)
    for step in range(0, 21):
        out = mix(colour, pole, step * 0.025)
        if ratio(out, canvas) >= floor:
            return out
    return out


def derive(palette):
    """Return the full token dict for ``palette`` (a tokens.py theme dict)."""
    dark = is_dark(palette)
    ink = palette['text_primary']
    pole = '#ffffff' if dark else '#000000'
    accent = palette['accent_blue']
    out = dict(palette)
    out.update({
        'dark': dark,
        # Layers: canvas < panel (bg_raised) < sunk well (bg_overlay).
        'canvas': palette['bg_base'],
        'panel': palette['bg_raised'],
        'well': palette['bg_overlay'],
        # Text levels.
        'ink': ink,
        'ink_strong': mix(ink, pole, 0.35),
        'ink_muted': palette['text_secondary'],
        'ink_faint': palette['text_disabled'],
        # Interaction: a hover is a barely-there step, a press one more.
        'hover': palette['bg_hover'],
        'press': palette['bg_selected'],
        'selected': palette['bg_selected'],
        'rule': palette['border_subtle'],
        'rule_strong': palette['border_medium'],
        # The one accent, and its washes.
        'accent': accent,
        'accent_wash': rgba(accent, 0.10),
        'accent_wash_strong': rgba(accent, 0.18),
        # State colours: the palette's own muted accents, plus a wash each.
        'ok': palette['accent_green'],
        'warn': palette['accent_orange'],
        # The state red is used for labels on the canvas, so it is lifted to
        # AA there; the solid safety fill below keeps the palette's own red.
        'err': _legible(palette['accent_red'], palette['bg_base'], pole),
        'ok_wash': rgba(palette['accent_green'], 0.12),
        'warn_wash': rgba(palette['accent_orange'], 0.14),
        'err_wash': rgba(palette['accent_red'], 0.12),
        # Safety: solid, saturated, deliberately not subtle.
        'safety': palette['accent_red'],
        'safety_hover': mix(palette['accent_red'], pole, 0.18),
        'safety_text': palette['text_on_accent'],
        'font_sans': FONT_SANS,
        'font_serif': FONT_SERIF,
        'font_mono': FONT_MONO,
    })
    return out


def wash(colour, alpha=0.10):
    """Return a translucent tint of ``colour`` for hover states."""
    return rgba(colour, alpha)
