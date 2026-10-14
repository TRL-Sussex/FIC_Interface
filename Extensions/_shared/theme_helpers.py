"""Shared Warmth helpers for extension UIs.

These centralise the few styling patterns that were duplicated (with
hard-coded hex) across many extensions, so they all track the active
theme via the app's token-substituted stylesheet.
"""

from fic_studio import theme


def set_toggle_button_state(btn, active):
    """Switch a start/stop toggle button between themed object names.

    ``active=True`` -> the running/stop look (``BtnStop``); ``active=False``
    -> the idle/start look (``BtnTeal``).  Both object names are styled by
    the app stylesheet and recolour with the active theme, so no inline hex
    is needed.  The widget is repolished so the new object name takes effect
    immediately.
    """
    if btn is None:
        return
    btn.setObjectName('BtnStop' if active else 'BtnTeal')
    style = btn.style()
    if style is not None:
        style.unpolish(btn)
        style.polish(btn)


def status_color(state):
    """Return a theme token colour for a semantic status string.

    Maps common states (ok/connected/running, warn/waiting, error/stopped,
    idle/muted) onto Warmth tokens.  Use in ``theme.bind`` callbacks
    or refresh handlers instead of hard-coded hex.
    """
    t = theme.current()
    state = (state or '').lower()
    if state in ('ok', 'connected', 'running', 'ready', 'success', 'active'):
        return t['accent_green']
    if state in ('warn', 'warning', 'waiting', 'connecting', 'teach'):
        return t['accent_orange']
    if state in ('error', 'stopped', 'failed', 'disconnected', 'danger'):
        return t['accent_red']
    if state in ('info', 'playing'):
        return t['accent_blue']
    return t['text_disabled']
