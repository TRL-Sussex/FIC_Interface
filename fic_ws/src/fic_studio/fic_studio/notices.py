"""Notices: what the Studio has to tell its operator (a process failed to start, a recording is
empty, a driver is not configured), shown wherever the operator is.

The classic window raised these as Qt message boxes. With the web UI, or a remote browser, a box
on the desktop window reaches nobody; so every notice goes into a short list that the web
snapshot carries (``notices``): the web UI pops each new one up, and keeps them all in its
Notices history until someone presses Clear all (``clear``). Only the classic window still gets
the box. Each notice is also written to the Log page.

    from fic_studio import notices
    notices.alert(self, 'error', 'Simulation failed to start', text)
"""
import itertools
import time
from collections import deque

#: The history holds this many; past it the oldest go (they are still on the Log page).
KEEP = 200
_ids = itertools.count(1)

LEVELS = {'error': 'ERROR', 'warn': 'WARN', 'info': 'INFO'}


def _store(w):
    q = getattr(w, '_notices', None)
    if q is None:
        q = deque(maxlen=KEEP)
        w._notices = q
    return q


def web_ui(w):
    """True when the operator works in the web UI (the Studio window is a browser view)."""
    return getattr(w, '_web_ui_window', None) is not None


def alert(w, level, title, text='', source='Studio', passing=False, window_box=True):
    """Tell the operator. ``level`` is 'error', 'warn' or 'info'. Never raises.

    ``passing``: a notification that drops down and goes by itself after a few seconds (a robot touching
    a safety zone), kept in the history like any other; the classic window shows no box for it.
    ``window_box=False``: no box in the classic window either (one would block the caller, as E-STOP's)."""
    level = level if level in LEVELS else 'info'
    text = str(text or '')
    if len(text) > 4000:
        text = text[:1000] + '\n…\n' + text[-2900:]
    entry = {'id': next(_ids), 't': time.time(), 'level': level, 'title': str(title), 'text': text}
    if passing:
        entry['passing'] = True
    _store(w).append(entry)
    try:
        w.log_message(LEVELS[level], '%s%s' % (title, ': ' + text if text else ''), source)
    except Exception:
        pass
    if not web_ui(w) and not passing and window_box:
        try:
            from PyQt5.QtWidgets import QMessageBox
            box = {'error': QMessageBox.critical, 'warn': QMessageBox.warning}.get(
                level, QMessageBox.information)
            box(w, title, text or title)
        except Exception:
            pass
    return entry


def clear(w, who=''):
    """Clear all: empty the history, for every viewer."""
    n = len(_store(w))
    _store(w).clear()
    if n:
        try:
            w.log_message('INFO', 'Notices cleared (%d) by %s' % (n, who or 'Studio window'), 'Studio')
        except Exception:
            pass
    return n


#: The snapshot carries this many in full (for the pop-ups); the history panel asks for the rest.
RECENT = 5


def history(w):
    """Every notice kept, oldest first."""
    return list(getattr(w, '_notices', None) or [])


def state(w, ctx=None):
    """The snapshot section: counts, a revision that changes with the history, the latest few."""
    q = history(w)
    return {'rev': '%d:%d' % (len(q), q[-1]['id'] if q else 0), 'count': len(q),
            'errors': sum(1 for n in q if n['level'] == 'error'),
            'warnings': sum(1 for n in q if n['level'] == 'warn'),
            'recent': q[-RECENT:]}
