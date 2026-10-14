"""
Build the application stylesheet for a theme.

The structural QSS lives in ``fic_studio.styles.QSS_TEMPLATE`` as a template
over design tokens; this module fills it from ``theme.design.derive`` for the
chosen theme.  There is no per-hex recolouring any more: every colour in the
output is a token, so a new theme is just a new palette dict.
"""
from fic_studio.styles import QSS_TEMPLATE
from fic_studio.theme.design import derive
from fic_studio.theme.tokens import normalize_theme_name, THEMES


def unmapped_hexes():
    """Return hex literals in the template that bypass tokens (none by design)."""
    import re
    return sorted(set(re.findall(r'#[0-9a-fA-F]{6}', QSS_TEMPLATE)))


def build_stylesheet(theme_name, scale_overlay=''):
    """Return the QSS for ``theme_name`` (+ optional scale overlay)."""
    tokens = derive(THEMES[normalize_theme_name(theme_name)])
    return QSS_TEMPLATE % tokens + scale_overlay


def scale_overlay_qss(scale):
    """Return the font/padding overlay QSS for a UI scale percentage."""
    factor = scale / 100.0
    base = int(12 * factor)
    return (
        '\n'
        '* {{ font-size: {base}px; }}\n'
        '#SideButton {{ padding: {p12}px {p16}px; font-size: {f13}px; }}\n'
        '#SideSection {{ font-size: {f10}px; '
        'padding: {p16}px {p16}px {p8}px {p16}px; }}\n'
        '#PageTitle {{ font-size: {f20}px; }}\n'
        '#SectionLabel {{ font-size: {f12}px; }}\n'
        'QPushButton {{ font-size: {f12}px; min-height: {p28}px; '
        'padding: {p4}px {p14}px; }}\n'
        'QComboBox {{ font-size: {f12}px; padding: {p6}px {p10}px; }}\n'
        'QLineEdit, QSpinBox, QDoubleSpinBox {{ font-size: {f12}px; '
        'padding: {p6}px {p10}px; }}\n'
        'QGroupBox {{ font-size: {f11}px; padding: {p16}px; '
        'padding-top: {p26}px; margin-top: {p14}px; }}\n'
        'QTabBar::tab {{ padding: {p8}px {p16}px; font-size: {f12}px; }}\n'
    ).format(
        base=base,
        p4=int(4 * factor), p14=int(14 * factor), p6=int(6 * factor), p8=int(8 * factor), p10=int(10 * factor),
        p12=int(12 * factor), p16=int(16 * factor), p20=int(20 * factor),
        p24=int(24 * factor), p28=int(28 * factor), p26=int(26 * factor),
        f10=int(10 * factor), f11=int(11 * factor), f12=int(12 * factor),
        f13=int(13 * factor), f20=int(20 * factor),
    )
