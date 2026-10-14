"""The Help page, for the web UI.

The sections are the classic Help page's own (``HELP_SECTIONS`` in
``help_mixin``); live ones (Extensions, Keyboard Shortcuts) are built by the
Studio at the moment they are asked for. The browser shows each in a sandboxed
frame, styled with the page's own colours.

Commands: ``help`` with ``op`` list | section | search | docs.
"""
import re

from fic_studio.web_remote.common import Refused

_TAG = re.compile(r'<[^>]+>')


def _sections():
    from fic_studio.mixins.help_mixin import HELP_SECTIONS
    return HELP_SECTIONS


def _body(w, key, content):
    from fic_studio.mixins.help_mixin import HelpMixin
    if callable(content):
        return content()
    if content is None:
        fn = {'extensions': '_help_extensions_html', 'keyboard_shortcuts': '_help_shortcuts_html'}.get(key)
        if fn is None:
            return '<p>No content available.</p>'
        method = getattr(w, fn, None) or getattr(HelpMixin, fn).__get__(w)
        return method()
    return content


def help_command(w, args, client):
    op = args.get('op') or 'list'
    sections = _sections()
    if op == 'list':
        return {'ok': True, 'sections': [{'key': k, 'title': t} for k, t, _c in sections]}
    if op == 'section':
        for key, title, content in sections:
            if key == args.get('key'):
                return {'ok': True, 'key': key, 'title': title, 'html': _body(w, key, content)}
        raise Refused('No such help section', 'bad_args')
    if op == 'search':
        needle = str(args.get('text') or '').strip().lower()[:80]
        hits = []
        for key, title, content in sections:
            if not needle:
                hits.append(key)
                continue
            body = '' if callable(content) or content is None else _TAG.sub(' ', content).lower()
            if needle in title.lower() or needle in body or callable(content) or content is None:
                hits.append(key)
        return {'ok': True, 'keys': hits}
    if op == 'docs':
        opened = bool(w.serve_docs(str(args.get('page') or 'home'))) if hasattr(w, 'serve_docs') else False
        return {'ok': True, 'opened': opened}
    raise Refused('No such help command', 'bad_args')


HANDLERS = {'help': help_command}
