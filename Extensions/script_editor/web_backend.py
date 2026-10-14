"""
Plain helpers for the Script Editor's web page (web_state / web_action in
extension.py): script names, the output both pages show, the failing line of a
run, and the script files. No Qt, so they are tested on their own.
"""
import ast
import collections
import os
import re
import time

#: Script names the web page may use: no folders, no dots.
NAME_RE = re.compile(r'[A-Za-z0-9_-]+')
#: The file name ScriptRunner compiles a script under, as tracebacks show it.
SCRIPT_FILENAME = '<script>'
_LINE_RE = re.compile(r'File "<script>", line (\d+)')
_FILE_PREFIX_RE = re.compile(r'^\S+\.py\s*[—–-]\s*')
_STAMP_RE = re.compile(r'^\[\d\d:\d\d:\d\d\]')


def check_name(name):
    """Return ``name`` without surrounding spaces; ValueError unless it is a plain script name."""
    name = str(name or '').strip()
    if not NAME_RE.fullmatch(name):
        raise ValueError('Use letters, digits, _ or - in the name')
    return name


def failing_line(traceback_text):
    """The script line a traceback ends in (its last ``File "<script>", line N``), or None."""
    found = _LINE_RE.findall(traceback_text or '')
    return int(found[-1]) if found else None


class OutputStore:
    """
    The run output both pages show.

    Text is written as a stream: complete lines are kept (the newest
    ``maxlen``), a trailing line without its newline stays open and is shown
    last. Each line remembers when it began, for the web page's timestamps.
    ``seq`` changes whenever the output does, so a page redraws only then.
    """

    def __init__(self, maxlen=2000, clock=time.time):
        """Start empty."""
        self._lines = collections.deque(maxlen=maxlen)     # (time it began, text)
        self._open = None                                   # the same for the open line
        self._clock = clock
        self.seq = 0

    def write(self, text):
        """Add stream text."""
        if not text:
            return
        now = self._clock()
        segs = str(text).split('\n')
        began = now
        if self._open is not None:
            began, head = self._open
            segs[0] = head + segs[0]
        *done, rest = segs
        for k, line in enumerate(done):
            self._lines.append((began if k == 0 else now, line))
        self._open = (began if not done else now, rest) if rest else None
        self.seq += 1

    def clear(self):
        """Forget all output."""
        self._lines.clear()
        self._open = None
        self.seq += 1

    def tail(self, n=500, stamped=False):
        """The last ``n`` lines, the open one included; ``stamped`` puts their time in front."""
        items = list(self._lines) + ([self._open] if self._open is not None else [])
        items = items[-n:]
        if not stamped:
            return [line for _, line in items]
        return [line if not line or _STAMP_RE.match(line)
                else time.strftime('[%H:%M:%S] ', time.localtime(began)) + line
                for began, line in items]


def list_scripts(folder):
    """Names (without .py) of the scripts in ``folder`` the web page can use, sorted."""
    try:
        files = os.listdir(folder)
    except OSError:
        return []
    return sorted(f[:-3] for f in files
                  if f.endswith('.py') and NAME_RE.fullmatch(f[:-3])
                  and os.path.isfile(os.path.join(folder, f)))


def describe(source):
    """First line of a script's module docstring, without a leading ``name.py —``; '' if none."""
    try:
        doc = (ast.get_docstring(ast.parse(source)) or '').strip()
    except SyntaxError:
        return ''
    return _FILE_PREFIX_RE.sub('', doc.splitlines()[0]) if doc else ''


def read_script(folder, name):
    """The text of ``folder/name.py``; ValueError('No such script') when it is not there."""
    path = os.path.join(folder, check_name(name) + '.py')
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            return fh.read()
    except FileNotFoundError:
        raise ValueError('No such script') from None
