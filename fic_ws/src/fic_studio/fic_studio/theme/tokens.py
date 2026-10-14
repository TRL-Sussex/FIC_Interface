"""
Design tokens for every theme.

The token vocabulary reuses the keys of ``styles.COLORS_DARK`` so the
stylesheet engine can map each legacy hex to a token by name.  Every theme
fills the same slots, which is what lets one structural stylesheet be
recoloured into all of them.

A theme name is ``<palette>_<brightness>``: ``warmth`` or ``cool``, dark or
light.  The four are built from two independent choices, and the settings
page offers them that way rather than as four separate buttons.

Two rules hold across all four palettes, and they are not stylistic:

- **``accent_orange`` stays orange and ``accent_red`` stays red.**  They mean
  *warning* and *danger* — the emergency stop is drawn with them.  A cool
  palette cools everything else; making a stop button blue to match would be
  a safety regression, not a theme.
- **Text meets WCAG AA (4.5:1) against the surface it sits on.**  A control
  room is not a design studio: the Studio is read at a distance, on projectors
  and under lab lighting.  ``test_theme_tokens`` computes the ratios rather
  than trusting the eye.
"""

#: Shipped default. The Editorial palette was removed; saved 'editorial_*'
#: names (and the legacy 'dark'/'light') migrate to the warmth pair of the same
#: brightness, the closest remaining look (flat, warm, low colour).
DEFAULT_THEME = 'warmth_dark'

#: The two halves a theme name is built from, in the order the UI shows them.
PALETTES = ('warmth', 'cool')
BRIGHTNESSES = ('dark', 'light')

#: What each half is called on screen.
PALETTE_LABELS = {'warmth': 'Warm', 'cool': 'Cool'}
BRIGHTNESS_LABELS = {'dark': 'Dark', 'light': 'Light'}

WARMTH_DARK = {
    'bg_base':        '#1B1714',
    'bg_raised':      '#241E19',
    'bg_overlay':     '#2D261F',
    'bg_hover':       '#322A22',
    'bg_selected':    '#4A2E1E',
    'border_subtle':  '#3A312A',
    'border_medium':  '#4A3F35',
    'border_strong':  '#5C4F42',
    'text_primary':   '#EDE6D9',
    'text_secondary': '#B3A897',
    'text_disabled':  '#7C7264',
    'text_on_accent': '#FBF6EE',
    'accent_blue':    '#C56A42',
    'accent_teal':    '#B5764F',
    'accent_green':   '#8A9A52',
    'accent_orange':  '#D29A3C',
    'accent_red':     '#C24A3A',
}

WARMTH_LIGHT = {
    'bg_base':        '#F4EEE4',
    'bg_raised':      '#FBF8F1',
    'bg_overlay':     '#FFFEFB',
    'bg_hover':       '#EFE7D8',
    'bg_selected':    '#E7D2C2',
    'border_subtle':  '#DED4C2',
    'border_medium':  '#CFC2B0',
    'border_strong':  '#B8A892',
    'text_primary':   '#2E2A24',
    'text_secondary': '#6B6456',
    'text_disabled':  '#A89E8C',
    'text_on_accent': '#FBF8F1',
    'accent_blue':    '#B5552F',
    'accent_teal':    '#A05A38',
    'accent_green':   '#6E7A3E',
    'accent_orange':  '#C2872A',
    'accent_red':     '#A23A2C',
}

COOL_DARK = {
    'bg_base':        '#14181D',
    'bg_raised':      '#1B2027',
    'bg_overlay':     '#222932',
    'bg_hover':       '#2A323D',
    'bg_selected':    '#1E3A5C',
    'border_subtle':  '#2B333E',
    'border_medium':  '#3B4653',
    'border_strong':  '#4F5C6D',
    'text_primary':   '#E3E9F0',
    'text_secondary': '#A2AEBD',
    'text_disabled':  '#78838F',
    'text_on_accent': '#F6F9FC',
    'accent_blue':    '#4C9BE8',
    'accent_teal':    '#35B3A4',
    'accent_green':   '#63B071',
    'accent_orange':  '#D89A3E',
    'accent_red':     '#DE5B52',
}

COOL_LIGHT = {
    'bg_base':        '#EDF1F6',
    'bg_raised':      '#F9FBFD',
    'bg_overlay':     '#FFFFFF',
    'bg_hover':       '#E2E9F1',
    'bg_selected':    '#D2E2F5',
    'border_subtle':  '#D8DFE8',
    'border_medium':  '#C3CDDA',
    'border_strong':  '#A4B1C1',
    'text_primary':   '#1E262F',
    'text_secondary': '#556475',
    'text_disabled':  '#7C8A99',
    'text_on_accent': '#FFFFFF',
    'accent_blue':    '#1F6BC7',
    'accent_teal':    '#0B7A70',
    'accent_green':   '#2F7D4C',
    'accent_orange':  '#9A6410',
    'accent_red':     '#B23A2E',
}

THEMES = {
    'warmth_dark': WARMTH_DARK,
    'warmth_light': WARMTH_LIGHT,
    'cool_dark': COOL_DARK,
    'cool_light': COOL_LIGHT,
}

_LEGACY = {'dark': 'warmth_dark', 'light': 'warmth_light',
           'editorial_dark': 'warmth_dark', 'editorial_light': 'warmth_light'}


def normalize_theme_name(name):
    """Return a valid THEMES key for ``name`` (maps legacy/unknown)."""
    if name in THEMES:
        return name
    return _LEGACY.get(name, DEFAULT_THEME)


def split_theme_name(name):
    """Return ``(palette, brightness)`` for a theme name, normalising first."""
    normalized = normalize_theme_name(name)
    palette, _, brightness = normalized.rpartition('_')
    return palette, brightness


def compose_theme_name(palette, brightness):
    """
    Return the theme name for a palette and a brightness.

    The settings page offers the two choices separately, so it needs to put
    them back together; anything unrecognised falls back to the default rather
    than producing a key ``THEMES`` does not have.
    """
    candidate = '%s_%s' % (palette, brightness)
    return candidate if candidate in THEMES else DEFAULT_THEME


def theme_label(name):
    """Return the on-screen name for a theme, e.g. ``'Cool Dark'``."""
    palette, brightness = split_theme_name(name)
    return '%s %s' % (PALETTE_LABELS.get(palette, palette.title()),
                      BRIGHTNESS_LABELS.get(brightness, brightness.title()))
