"""The installer windows' look: the web interface's dark theme
(fic_ws/src/fic_studio/fic_studio/web_remote/ui/fic.css) in plain values, for
installer_gui.py (tkinter) and build_gui.py (PyQt5). Change the two together."""

COLORS = {
    'canvas':       '#14161a',   # window background
    'surface':      '#181b20',   # cards
    'sunk':         '#1d2127',   # console, input wells
    'ink':          '#c8cacc',   # body text
    'ink_strong':   '#f2f2f0',   # titles
    'ink_muted':    '#8d9096',   # secondary text
    'ink_faint':    '#6c7076',   # disabled text
    'rule':         '#262a30',   # hairlines, progress track
    'rule_strong':  '#3a3f47',   # borders that must show
    'hover':        '#22262c',   # a quiet button under the pointer
    'accent':       '#C56A42',   # the one interactive colour (terracotta)
    'accent_hover': '#D17B54',
    'on_accent':    '#ffffff',
    'ok':           '#6fc08a',
    'warn':         '#e0b04e',
    'bad':          '#f08a86',
}

# First installed family wins, as in the web interface's font stack.
SANS = ('Inter', 'Ubuntu', 'Cantarell', 'Noto Sans', 'DejaVu Sans', 'Liberation Sans')
MONO = ('Ubuntu Mono', 'DejaVu Sans Mono', 'Noto Sans Mono', 'Liberation Mono')


def pick_family(installed, wanted, fallback):
    """Return the first family of ``wanted`` that is in ``installed``."""
    have = {f.lower() for f in installed}
    for family in wanted:
        if family.lower() in have:
            return family
    return fallback
