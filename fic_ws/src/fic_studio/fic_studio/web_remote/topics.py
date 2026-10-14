"""The Topic Monitor page, for the web UI.

The Studio's monitor already subscribes to topics, counts and times what
arrives and turns a message into named fields (``_start_monitoring``,
``_topic_hz``, ``_introspect_msg``). This module hands those to the browser:
the topic list with publisher and subscriber counts, a monitor switch per
topic, and the selected topic's fields and raw stream.

Command: ``topics`` with ``op`` list | monitor | select | clear.
"""
import time
from collections import deque

from fic_studio.web_remote.common import Refused, _num

#: Raw-stream lines kept for the selected topic.
RAW_KEEP = 300


def _namespace(name):
    from fic_studio.mixins.topic_monitor_mixin import _topic_namespace
    return _topic_namespace(name)


def _buffer(w):
    """Collect the raw lines of the selected topic, from the Studio's own message signal."""
    buf = getattr(w, '_web_topic_raw', None)
    if buf is not None:
        return buf
    buf = w._web_topic_raw = {'topic': None, 'lines': deque(maxlen=RAW_KEEP), 'seq': 0}
    signals = getattr(w, 'topic_signals', None)
    if signals is not None:
        def on_msg(topic, text, stamp):
            if topic == buf['topic']:
                buf['seq'] += 1
                buf['lines'].append({'seq': buf['seq'], 'time': time.strftime('%H:%M:%S', time.localtime(stamp)) +
                                     ('%.3f' % (stamp % 1))[1:], 'text': text})
        signals.message_received.connect(on_msg)
    return buf


def _fields(msg):
    from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin, _introspect_msg
    out = []
    for name, value in _introspect_msg(msg).items():
        numeric = isinstance(value, (int, float)) and not isinstance(value, bool)
        out.append({'name': str(name), 'value': TopicMonitorMixin._format_field_value(value),
                    'number': _num(value) if numeric else None})
    return out


def _list(w):
    from fic_studio.mixins.topic_monitor_mixin import _topic_hz
    node = w.node
    topics = sorted(((n, (t[0] if t else 'unknown')) for n, t in node.get_topic_names_and_types()), key=lambda x: x[0])
    if hasattr(w, '_tree_topic_types'):
        for name, kind in topics:
            w._tree_topic_types[name] = kind
    monitored = getattr(w, 'monitored_topics', {}) or {}
    rows = []
    for name, kind in topics:
        stamps = (getattr(w, 'topic_last_msg', {}) or {}).get(name)
        hz = _topic_hz(stamps) if stamps else 0.0
        try:
            pubs, subs = node.count_publishers(name), node.count_subscribers(name)
        except Exception:
            pubs = subs = None
        rows.append({'name': name, 'type': kind, 'ns': _namespace(name), 'pubs': pubs, 'subs': subs,
                     'monitored': name in monitored, 'hz': hz if name in monitored else None,
                     'count': (getattr(w, 'topic_msg_count', {}) or {}).get(name, 0),
                     'kind': 'service' if '/srv/' in kind else 'action' if '/action/' in kind else 'topic'})
    return rows


def topics(w, args, client):
    op = args.get('op') or 'list'
    if op == 'list':
        rows = _list(w)
        return {'ok': True, 'topics': rows,
                'monitoring': len(getattr(w, 'monitored_topics', {}) or {}),
                'total': sum((getattr(w, 'topic_msg_count', {}) or {}).values())}
    name = str(args.get('name') or '')
    known = {r['name']: r['type'] for r in _list(w)} if op in ('monitor', 'select') else {}
    if op in ('monitor', 'select') and name not in known:
        raise Refused('No such topic', 'bad_args')
    if op == 'monitor':
        on = bool(args.get('value'))
        if on != (name in w.monitored_topics):
            w._toggle_monitor(name, known[name])
        return {'ok': True}
    if op == 'select':
        buf = _buffer(w)
        buf['topic'] = name
        buf['lines'].clear()
        w._selected_topic = name
        if name not in w.monitored_topics:                     # inspecting a topic subscribes to it
            w._start_monitoring(name, known[name])
            w._save_monitored_topics()
        return {'ok': True}
    if op == 'inspect':
        buf = _buffer(w)
        name = buf['topic']
        if not name:
            return {'ok': True, 'topic': None}
        since = int(_num(args.get('since'), 0))
        msg = (getattr(w, 'topic_cached_msg', {}) or {}).get(name)
        from fic_studio.mixins.topic_monitor_mixin import _topic_hz
        stamps = (getattr(w, 'topic_last_msg', {}) or {}).get(name)
        return {'ok': True, 'topic': name, 'type': (getattr(w, '_tree_topic_types', {}) or {}).get(name, ''),
                'fields': _fields(msg) if msg is not None else [],
                'hz': _topic_hz(stamps) if stamps else 0.0,
                'count': (getattr(w, 'topic_msg_count', {}) or {}).get(name, 0),
                'lines': [ln for ln in buf['lines'] if ln['seq'] > since], 'seq': buf['seq']}
    if op == 'clear':
        buf = _buffer(w)
        buf['lines'].clear()
        return {'ok': True}
    raise Refused('No such topic command', 'bad_args')


HANDLERS = {'topics': topics}
