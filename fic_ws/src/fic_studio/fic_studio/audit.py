"""Append-only record of the safety-relevant things done to the robots: E-STOP and its reset,
sessions and processes started or stopped, targets changed, teleoperation, control taken.

The Log page keeps 500 lines in memory and can be cleared; this file cannot be cleared from the
UI and survives a restart. One JSON object per line, rotated at 1 MB (five old files kept), in
``~/.fic_studio/audit.log`` (``$FIC_STUDIO_AUDIT`` moves it). Writing never raises: a full disk
must not stop an E-STOP.

    {"t": "2026-10-18T05:12:03.412+01:00", "event": "estop", "who": "adnan @ 192.168.0.40",
     "detail": "E-STOP from web remote", "target": "hardware", "station": 1}
"""
import json
import logging
import logging.handlers
import os
import threading
import time

MAX_BYTES = 1_000_000
BACKUPS = 5

_lock = threading.Lock()
_logger = None


def path():
    from fic_studio import paths
    return os.environ.get('FIC_STUDIO_AUDIT') or paths.state_path('audit.log')


def _get():
    global _logger
    if _logger is not None:
        return _logger
    log = logging.getLogger('fic_studio.audit')
    log.propagate = False
    log.setLevel(logging.INFO)
    p = path()
    try:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        handler = logging.handlers.RotatingFileHandler(p, maxBytes=MAX_BYTES, backupCount=BACKUPS,
                                                       encoding='utf-8')
        handler.setFormatter(logging.Formatter('%(message)s'))
        log.addHandler(handler)
    except OSError:
        log.addHandler(logging.NullHandler())
    _logger = log
    return log


def _stamp(now=None):
    now = time.time() if now is None else now
    lt = time.localtime(now)
    off = time.strftime('%z', lt)
    return time.strftime('%Y-%m-%dT%H:%M:%S', lt) + '.%03d' % int(now % 1 * 1000) + off[:3] + ':' + off[3:]


def record(event, who='', detail='', **extra):
    """Add one line. ``who`` is the person or source (account @ address, 'Studio window',
    'Gamepad', a station); ``extra`` must be JSON-able."""
    entry = {'t': _stamp(), 'event': str(event), 'who': str(who or ''), 'detail': str(detail or '')}
    entry.update(extra)
    try:
        line = json.dumps(entry, default=str)
        with _lock:
            _get().info(line)
    except Exception:
        pass


def reset_for_tests():
    """Drop the open file, so a test can point ``$FIC_STUDIO_AUDIT`` somewhere else."""
    global _logger
    with _lock:
        if _logger is not None:
            for h in list(_logger.handlers):
                _logger.removeHandler(h)
                h.close()
        _logger = None


def tail(n=200):
    """The last ``n`` entries, newest last (for the Log page)."""
    out = []
    try:
        with open(path(), encoding='utf-8') as f:
            lines = f.readlines()[-n:]
    except OSError:
        return out
    for ln in lines:
        try:
            out.append(json.loads(ln))
        except ValueError:
            continue
    return out
