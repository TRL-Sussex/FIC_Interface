"""Tests for theme defaults and legacy settings migration."""
from fic_studio.mixins.settings_mixin import DEFAULT_SETTINGS
from fic_studio.theme.tokens import normalize_theme_name


def test_default_settings_theme_matches_the_engine_default():
    """The settings default and the token default must not drift apart."""
    from fic_studio.theme.tokens import DEFAULT_THEME
    assert DEFAULT_SETTINGS['appearance']['theme'] == DEFAULT_THEME
    assert DEFAULT_SETTINGS['appearance']['theme'] == 'warmth_dark'


def test_legacy_theme_strings_migrate():
    """Old persisted values map forward to warmth_* names."""
    assert normalize_theme_name('dark') == 'warmth_dark'
    assert normalize_theme_name('light') == 'warmth_light'


# ── The four-theme picker ─────────────────────────────────────────────────────
# Theme is chosen as two independent halves — palette and brightness — so the
# row stays two short pairs instead of growing a button per theme.

from test_settings_cards import _make_window  # noqa: E402
from fic_studio.styles import QSS_TEMPLATE as DARK_THEME  # noqa: E402
from fic_studio.theme.tokens import PALETTES, THEMES  # noqa: E402


def _picker(theme_name):
    """
    Return a window whose settings page has been built for ``theme_name``.

    The built page is kept on the window: it is the only reference, and
    without it Python drops the wrapper, Qt deletes the widgets underneath,
    and every button on the page raises "wrapped C/C++ object has been
    deleted" the moment a test touches it.
    """
    win = _make_window(theme_name)
    win.settings['appearance']['theme'] = theme_name
    win._built_settings_page = win._build_settings_page()
    return win


def test_the_picker_offers_both_halves():
    """Palette and brightness are separate choices, not four buttons."""
    win = _picker('warmth_dark')
    for attr in ('_theme_warm_btn', '_theme_cool_btn',
                 '_theme_dark_btn', '_theme_light_btn'):
        assert hasattr(win, attr), attr


def test_every_theme_can_be_selected_and_read_back():
    """Each of the four is reachable from the two pairs of buttons."""
    win = _picker('warmth_dark')
    for palette_btn, brightness_btn, expected in (
            ('_theme_warm_btn', '_theme_dark_btn', 'warmth_dark'),
            ('_theme_warm_btn', '_theme_light_btn', 'warmth_light'),
            ('_theme_cool_btn', '_theme_dark_btn', 'cool_dark'),
            ('_theme_cool_btn', '_theme_light_btn', 'cool_light')):
        getattr(win, palette_btn).click()
        getattr(win, brightness_btn).click()
        assert win._get_theme_selection() == expected, expected


def test_opening_settings_shows_the_theme_already_in_use():
    """A page built while Cool Light is active must open on Cool Light."""
    win = _picker('cool_light')
    assert win._theme_cool_btn.isChecked()
    assert win._theme_light_btn.isChecked()
    assert not win._theme_warm_btn.isChecked()
    assert not win._theme_dark_btn.isChecked()
    assert win._get_theme_selection() == 'cool_light'


def test_setting_the_selection_ticks_exactly_one_of_each_group():
    """Whatever is restored, neither group may end up blank or multi-on.

    The palette half stopped being a pair when the editorial palette landed,
    so this asserts exactly one of N rather than warm-xor-cool.
    """
    win = _picker('warmth_dark')
    for name in THEMES:
        win._set_theme_selection(name)
        ticked = [k for k, b in win._theme_palette_btns.items() if b.isChecked()]
        assert len(ticked) == 1, '%s ticked %r' % (name, ticked)
        assert win._theme_dark_btn.isChecked() != win._theme_light_btn.isChecked()
        assert win._get_theme_selection() == name, name


def test_every_palette_has_a_button():
    """A palette that cannot be picked may as well not exist."""
    win = _picker('warmth_dark')
    assert set(win._theme_palette_btns) == set(PALETTES)


def test_the_selected_theme_actually_looks_selected():
    """
    The old control used object names the stylesheet never defined.

    ``SegmentLeft``/``SegmentRight`` appear nowhere in styles.py, so a checked
    button rendered exactly like an unchecked one and the current theme was
    invisible. Whatever object name the picker uses must have a ``:checked``
    rule to point at.
    """
    win = _picker('warmth_dark')
    for attr in ('_theme_warm_btn', '_theme_cool_btn',
                 '_theme_dark_btn', '_theme_light_btn'):
        name = getattr(win, attr).objectName()
        assert name, '%s has no object name to style' % attr
        assert 'QPushButton#%s:checked' % name in DARK_THEME, (
            '%s uses object name %r, which the stylesheet gives no checked '
            'state' % (attr, name))
