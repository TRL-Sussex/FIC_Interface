"""Tests for the theme token maps."""
import re

from fic_studio.styles import COLORS_DARK
from fic_studio.theme.tokens import (
    BRIGHTNESSES,
    compose_theme_name,
    DEFAULT_THEME,
    normalize_theme_name,
    PALETTES,
    split_theme_name,
    theme_label,
    THEMES,
)

_HEX = re.compile(r'^#[0-9A-Fa-f]{6}$')


def test_themes_are_key_symmetric_with_colors_dark():
    """Every theme defines exactly the COLORS_DARK key set.

    One structural stylesheet is recoloured into all of them, so a theme
    missing a slot would leave that hex at its dark-theme value rather than
    failing — a single stubbornly dark widget on a light page.
    """
    expected = set(COLORS_DARK.keys())
    for name, tokens in THEMES.items():
        assert set(tokens.keys()) == expected, name


def test_all_values_are_six_digit_hex():
    """Every token value is a #RRGGBB string."""
    for theme in THEMES.values():
        for value in theme.values():
            assert _HEX.match(value), value


def test_default_theme_is_warmth_dark():
    """The shipped default is the warm dark theme.

    Changed in v4.2.0. Legacy 'dark'/'light' strings still migrate to the
    warmth pair — an existing install keeps the look it was saved with.
    """
    assert DEFAULT_THEME == 'warmth_dark'
    assert DEFAULT_THEME in THEMES


def test_normalize_handles_legacy_and_unknown():
    """Legacy names map forward; unknowns fall back to default."""
    assert normalize_theme_name('dark') == 'warmth_dark'
    assert normalize_theme_name('light') == 'warmth_light'
    assert normalize_theme_name('warmth_light') == 'warmth_light'
    assert normalize_theme_name('nonsense') == DEFAULT_THEME


# ── The four themes ────────────────────────────────────────────────────────────

def test_every_palette_and_brightness_combination_exists():
    """The settings page offers palette x brightness, so all four must resolve."""
    for palette in PALETTES:
        for brightness in BRIGHTNESSES:
            name = '%s_%s' % (palette, brightness)
            assert name in THEMES, name
    assert len(THEMES) == len(PALETTES) * len(BRIGHTNESSES)


def test_theme_names_split_and_compose_round_trip():
    """A name survives being taken apart and put back together."""
    for name in THEMES:
        palette, brightness = split_theme_name(name)
        assert compose_theme_name(palette, brightness) == name


def test_compose_falls_back_rather_than_inventing_a_key():
    """A half-recognised choice must not produce a key THEMES lacks."""
    assert compose_theme_name('bogus', 'light') == DEFAULT_THEME
    assert compose_theme_name('cool', 'bogus') == DEFAULT_THEME
    assert compose_theme_name('cool', 'light') == 'cool_light'


def test_theme_labels_read_as_english():
    """The names shown in Settings."""
    assert theme_label('warmth_dark') == 'Warm Dark'
    assert theme_label('warmth_light') == 'Warm Light'
    assert theme_label('cool_dark') == 'Cool Dark'
    assert theme_label('cool_light') == 'Cool Light'


# ── Legibility ─────────────────────────────────────────────────────────────────

def _relative_luminance(hex_value):
    """Return the WCAG relative luminance of a #RRGGBB string."""
    raw = hex_value.lstrip('#')
    channels = [int(raw[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    linear = [(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4)
              for c in channels]
    return 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2]


def _contrast(first, second):
    """Return the WCAG contrast ratio between two #RRGGBB strings."""
    a, b = _relative_luminance(first), _relative_luminance(second)
    lighter, darker = max(a, b), min(a, b)
    return (lighter + 0.05) / (darker + 0.05)


def test_body_text_meets_wcag_aa_on_every_surface():
    """
    The Studio is read at a distance, on projectors, under lab lighting.

    Both text tokens are checked against all three surfaces they are drawn on,
    in every theme, so a palette cannot be added by eye and ship illegible.
    """
    for name, tokens in THEMES.items():
        for surface in ('bg_base', 'bg_raised', 'bg_overlay'):
            for text in ('text_primary', 'text_secondary'):
                ratio = _contrast(tokens[text], tokens[surface])
                assert ratio >= 4.5, (
                    '%s: %s on %s is only %.2f:1, below WCAG AA 4.5:1'
                    % (name, text, surface, ratio))


def test_warning_and_danger_stay_warm_in_every_palette():
    """
    A cool palette must not cool the stop button.

    ``accent_orange`` and ``accent_red`` carry meaning, not style — the
    emergency stop is drawn with them. Hue is checked rather than a fixed
    value so a palette may tune them, but never turn them blue or green.
    """
    for name, tokens in THEMES.items():
        for key, low, high in (('accent_orange', 20, 55), ('accent_red', -12, 20)):
            raw = tokens[key].lstrip('#')
            red, green, blue = (int(raw[i:i + 2], 16) for i in (0, 2, 4))
            assert red > blue, '%s: %s is not a warm hue' % (name, key)
            hue = _hue_degrees(red, green, blue)
            assert low <= hue <= high, (
                '%s: %s sits at %d deg, outside the %d-%d band that still '
                'reads as its warning' % (name, key, hue, low, high))


def _hue_degrees(red, green, blue):
    """Return the hue of an RGB triple in degrees, signed near red."""
    r, g, b = red / 255.0, green / 255.0, blue / 255.0
    high, low = max(r, g, b), min(r, g, b)
    span = high - low
    if span == 0:
        return 0
    if high == r:
        hue = 60 * (((g - b) / span) % 6)
    elif high == g:
        hue = 60 * (((b - r) / span) + 2)
    else:
        hue = 60 * (((r - g) / span) + 4)
    return hue - 360 if hue > 300 else hue


def test_editorial_is_gone_and_old_names_migrate_to_warmth():
    from fic_studio.theme.tokens import PALETTES
    assert sorted(THEMES) == ['cool_dark', 'cool_light', 'warmth_dark', 'warmth_light']
    assert PALETTES == ('warmth', 'cool')
    assert normalize_theme_name('editorial_dark') == 'warmth_dark'
    assert normalize_theme_name('editorial_light') == 'warmth_light'
