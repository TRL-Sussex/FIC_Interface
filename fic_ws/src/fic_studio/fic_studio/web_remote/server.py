"""HTTP server for the FIC Studio web remote. Standard library only.

    GET  /               the web app (a fixed set of static files)
    GET  /api/state      one snapshot, for the first paint
    GET  /api/events     Server-Sent Events: 'state' ~10 Hz, 'control' on change
    POST /api/control    take / release / heartbeat the control lease
    POST /api/command    one command; needs the lease, except 'estop'

The server knows nothing about Qt or ROS. It is handed two callables, both
safe to call from any thread: ``dispatch(action, args, client)`` runs a
command and returns a JSON-able dict, and ``snapshot()`` returns the current
state. The Studio pushes fresh state with ``publish()``.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import itertools
import json
import math
from pathlib import Path
import socket
import threading
import time
from urllib.parse import parse_qs, urlparse

from fic_studio.web_remote.common import _who
from fic_studio.web_remote.lease import ControlLease

#: Commands anyone who can see the page may send, without control: E-STOP, and reading the
#: notice history.
NO_LEASE = ('estop', 'notices_list', 'log_read', 'audit_read')
#: Reads that need no control, but do need the page they belong to.
READ_PAGES = {'log_read': 'log', 'audit_read': 'log'}

STATIC_DIR = Path(__file__).resolve().parent / 'static'
#: The redesigned web UI (served under /ui/). Plain files, no build step.
UI_DIR = Path(__file__).resolve().parent / 'ui'
UI_TYPES = {'.html': 'text/html; charset=utf-8', '.css': 'text/css; charset=utf-8',
            '.js': 'text/javascript; charset=utf-8', '.svg': 'image/svg+xml',
            '.json': 'application/json', '.woff2': 'font/woff2', '.png': 'image/png',
            '.mjs': 'text/javascript; charset=utf-8'}
#: What an extension's web/ folder may serve: scripts, styles and assets, never HTML.
EXT_TYPES = {k: v for k, v in UI_TYPES.items() if k != '.html'}
#: Cookie that proves a request came from the desktop window that was handed
#: the one-run token (see ``local_token``).
LOCAL_COOKIE = 'fic_local'
#: Pages of the web UI, in sidebar order: (id, label, group, ready).
UI_PAGES = (
    # The classic sidebar's pages, in its order. ``nav`` says where the side list puts
    # each: the main run, pinned at the bottom (Settings, Logs, Help), or only
    # reachable from a menu (the two web-only extras).
    ('dashboard', 'Dashboard', '', True, 'main'),
    ('multi_robot', 'Multi-Robot', '', True, 'main'),
    ('locomotion', 'Locomotion', '', True, 'main'),
    ('tuning', 'Tuning Panel', 'Tuning', True, 'main'),
    ('robot_config', 'Robot Config', 'System', True, 'system'),
    ('topic_monitor', 'Topic Monitor', 'System', True, 'system'),
    ('safety', 'Safety', 'System', True, 'system'),
    ('session', 'Session', 'System', True, 'system'),
    ('users', 'Users', '', True, 'bottom'),
    ('settings', 'Settings', '', True, 'bottom'),
    ('log', 'Logs', '', True, 'bottom'),
    ('help', 'Help', '', True, 'bottom'),
    ('extensions', 'Extensions', '', True, 'hidden'),
    ('view3d', '3D view', '', True, 'hidden'),
)

#: URL path -> (file in STATIC_DIR, content type). Nothing else is served.
#: The web UI itself (ui/) is served at / and /ui/, to the desktop window and to
#: remote clients alike; the old phone page was retired. These are shared assets.
STATIC_ROUTES = {
    '/manifest.webmanifest': ('manifest.webmanifest', 'application/manifest+json'),
    '/icon.svg': ('icon.svg', 'image/svg+xml'),
    '/three.module.js': ('three.module.js', 'text/javascript; charset=utf-8'),
    '/view3d.js': ('view3d.js', 'text/javascript; charset=utf-8'),
}

#: Cookie the session token travels in (see web_remote.auth).
AUTH_COOKIE = 'fic_session'
#: Pages the web remote has, for a Studio with no accounts configured.
ALL_PAGES = ('control', 'view3d', 'dashboard', 'session', 'log', 'extensions')

MAX_BODY_BYTES = 16 * 1024
#: The desktop session's file uploads (robots_upload) carry a base64 chunk of up to 1 MB.
MAX_LOCAL_BODY_BYTES = 1400 * 1024
MAX_STREAMS = 32
KEEPALIVE_S = 10.0
#: With the configured port taken (a second Studio on this machine), try the
#: next ones up rather than failing.
PORT_ATTEMPTS = 10


def _finite(value):
    """Replace NaN/inf (which JSON cannot carry) with None, recursively."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(v) for v in value]
    return value


def encode_json(obj):
    """Compact JSON bytes; non-finite floats become null."""
    try:
        return json.dumps(obj, separators=(',', ':'), allow_nan=False).encode()
    except ValueError:
        return json.dumps(_finite(obj), separators=(',', ':')).encode()


class WebRemoteServer:
    """The HTTP + SSE server; one per Studio."""

    def __init__(self, dispatch, snapshot, lease=None, on_event=None,
                 host='0.0.0.0', scene=None, panels=None, auth=None, tls_files=None,
                 allow_signup=True, local_token=None, extensions=None):
        #: With a token the server is the desktop's own: every request must
        #: carry the cookie that /ui/?t=<token> sets, and that same trusted
        #: session may take control from a remote holder (``force``).
        self.local_token = local_token or None
        self._dispatch = dispatch
        self._snapshot = snapshot
        #: scene(None) -> the 3D scene as a dict; scene(i) -> mesh i as bytes.
        #: None, or raising, simply means the page offers no 3D view.
        self._scene = scene
        #: panels() -> the Studio's extensions; panels(id) -> one as data.
        #: None means this Studio offers no extensions to a browser.
        self._panels = panels
        #: (file(ext_id, rel) -> Path | None, state(ext_id) -> dict | None) for
        #: extensions with their own web page; None when the Studio has none.
        self._extensions = extensions
        #: Auth: who may open this page and what they may see. Without it the
        #: page is open to anyone who can reach the port, as it always was.
        self.auth = auth
        #: (certificate, key) to serve HTTPS with, so passwords are not in the clear.
        self._tls_files = tls_files
        #: May a visitor make their own account? They get the simulation only
        #: until an admin says otherwise (see web_remote.accounts).
        self.allow_signup = bool(allow_signup)
        self.lease = lease if lease is not None else ControlLease()
        self._on_event = on_event
        self._host = host
        self._httpd = None
        self._cond = threading.Condition()
        self._stopping = False
        self._state_json = b''
        self._state_seq = 0
        self._event_seq = 0
        self._events_log = []                 # (seq, name, json) of the last few named events
        self._streams = {}
        self._streams_lock = threading.Lock()
        self._stream_ids = itertools.count(1)

    # ── Lifecycle ────────────────────────────────────────────────────────
    def start(self, port, attempts=PORT_ATTEMPTS):
        """Bind and serve on a daemon thread; return the port actually bound."""
        port = int(port)
        candidates = [0] if port == 0 else range(port, port + max(1, attempts))
        error = None
        for candidate in candidates:
            try:
                httpd = _HTTPServer((self._host, candidate), _Handler)
            except OSError as exc:
                error = exc
                continue
            if self._tls_files:
                from fic_studio.web_remote import tls
                httpd.tls_context = tls.context(*self._tls_files)
            httpd.remote = self
            with self._cond:
                self._stopping = False
            self._httpd = httpd
            threading.Thread(target=httpd.serve_forever, name='fic-web-remote',
                             daemon=True).start()
            return httpd.server_address[1]
        raise error or OSError('no port available')

    def stop(self):
        httpd, self._httpd = self._httpd, None
        if httpd is None:
            return
        with self._cond:
            self._stopping = True
            self._cond.notify_all()
        httpd.shutdown()
        httpd.server_close()

    def _on_accounts_changed(self):
        """Tell the Studio an admin changed the users, so it saves and shares them."""
        callback = getattr(self, 'on_accounts_changed', None)
        if callback:
            try:
                callback()
            except Exception:
                pass

    @property
    def scheme(self):
        """'https' when serving with a certificate, else 'http'."""
        return 'https' if self._tls_files else 'http'

    @property
    def running(self):
        return self._httpd is not None

    @property
    def port(self):
        return self._httpd.server_address[1] if self._httpd else None

    # ── State fan-out ────────────────────────────────────────────────────
    def publish(self, state):
        """Hand every open event stream a new snapshot."""
        data = encode_json(state)
        with self._cond:
            self._state_json = data
            self._state_seq += 1
            self._cond.notify_all()

    def emit(self, name, payload):
        """Send one named event (for example 'settings') to every open stream now."""
        data = encode_json(payload)
        with self._cond:
            self._event_seq += 1
            self._events_log = (self._events_log + [(self._event_seq, name, data)])[-32:]
            self._cond.notify_all()

    def stream_count(self):
        with self._streams_lock:
            return len(self._streams)

    def clients(self):
        """Return one entry per connected browser: client id, name, address, since."""
        with self._streams_lock:
            seen = {}
            for info in self._streams.values():
                seen.setdefault(info['client'] or info['addr'], dict(info))
            return list(seen.values())

    def control_view(self, client):
        """Return what ``client`` should show about the control lease."""
        holder = self.lease.holder()
        view = {'you': False, 'holder': None, 'timeout_s': self.lease.timeout_s}
        if holder is not None:
            view['you'] = bool(client) and holder['client'] == client
            view['holder'] = {'name': holder['name'], 'addr': holder['addr']}
        return view

    # ── Internals used by the handler ────────────────────────────────────
    def _event(self, kind, info):
        if self._on_event is not None:
            try:
                self._on_event(kind, info)
            except Exception:
                pass

    def _open_stream(self, client, name, addr):
        with self._streams_lock:
            if len(self._streams) >= MAX_STREAMS:
                return None
            sid = next(self._stream_ids)
            self._streams[sid] = {'client': client, 'name': name, 'addr': addr,
                                  'since': time.time()}
        self._event('stream_opened', {'client': client, 'name': name, 'addr': addr})
        return sid

    def _close_stream(self, sid):
        with self._streams_lock:
            info = self._streams.pop(sid, None)
        if info is not None:
            self._event('stream_closed', info)


class _HTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    # Rebinding a port left in TIME_WAIT by the last run. On Linux this does
    # not let two live listeners share a port, so a second Studio still
    # moves on to the next one.
    allow_reuse_address = True
    #: Set by ``WebRemoteServer.start`` when serving HTTPS.
    tls_context = None
    #: A new connection has this long to say something before its thread gives
    #: up on it. A phone that roamed away mid-handshake must not hold one.
    HANDSHAKE_TIMEOUT_S = 10

    def handle_error(self, request, client_address):
        """Say nothing about a browser that walked off mid-answer.

        A phone leaving wifi, a tab closed while the page loads, a TLS
        connection dropped: the handler raises as it writes, and the default
        prints a traceback into the Studio's log. Anything else still does.
        """
        import ssl
        import sys
        kind = sys.exc_info()[0]
        if kind is not None and issubclass(kind, (ConnectionError, BrokenPipeError,
                                                  TimeoutError, ssl.SSLError)):
            return
        super().handle_error(request, client_address)

    def get_request(self):
        """Accept the connection as it is: no TLS handshake on this thread.

        The handshake belongs on the connection's own thread. Done here, in
        the one thread that accepts, a browser that opened ``http://`` — or a
        port scanner — leaves the whole web remote answering nobody.
        """
        return self.socket.accept()

    def process_request_thread(self, request, client_address):
        if self.tls_context is not None:
            request = self._start_tls(request)
            if request is None:
                return
        super().process_request_thread(request, client_address)

    def _start_tls(self, sock):
        """Return the TLS connection, or None when this client gets no further."""
        try:
            sock.settimeout(self.HANDSHAKE_TIMEOUT_S)
            if sock.recv(1, socket.MSG_PEEK) != b'\x16':
                # Not a TLS hello: an http:// address for a page that is now
                # served over https. Say so rather than dropping them.
                self._redirect_to_https(sock)
                self.shutdown_request(sock)
                return None
            connection = self.tls_context.wrap_socket(sock, server_side=True)
            connection.settimeout(None)
            return connection
        except OSError:                      # handshake failed, or nothing came
            self.shutdown_request(sock)
            return None

    def _redirect_to_https(self, sock):
        """Answer a plain-HTTP request on the TLS port with the https address."""
        try:
            head = sock.recv(2048).decode('latin-1', 'replace')
            words = head.split('\r\n', 1)[0].split(' ')
            path = words[1] if len(words) > 1 and words[1].startswith('/') else '/'
            host = ''
            for header in head.split('\r\n')[1:]:
                if header.lower().startswith('host:'):
                    host = header.split(':', 1)[1].strip()
                    break
            if not host:
                address = sock.getsockname()
                host = '%s:%d' % (address[0], address[1])
            body = b'The FIC web remote is served over HTTPS.\n'
            sock.sendall(b'HTTP/1.1 301 Moved Permanently\r\n'
                         b'Location: https://%s%s\r\n'
                         b'Content-Type: text/plain\r\n'
                         b'Content-Length: %d\r\n'
                         b'Connection: close\r\n\r\n%s'
                         % (host.encode('latin-1', 'replace'),
                            path.encode('latin-1', 'replace'), len(body), body))
        except OSError:
            pass


class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'FICStudioWeb/1'
    # A phone that vanishes mid-stream must not pin a thread forever.
    timeout = 30

    def log_message(self, fmt, *args):
        """Silence the per-request access log on stderr."""

    @property
    def remote(self):
        return self.server.remote

    # ── Who is asking ────────────────────────────────────────────────────
    def _token(self):
        from http.cookies import SimpleCookie
        raw = self.headers.get('Cookie')
        if not raw:
            return ''
        try:
            jar = SimpleCookie(raw)
        except Exception:
            return ''
        morsel = jar.get(AUTH_COOKIE)
        return morsel.value if morsel else ''

    def _user(self):
        """Return the signed-in user, or None. Without accounts, everyone passes."""
        auth = self.remote.auth
        return auth.user_for(self._token()) if auth else None

    def _needs_login(self):
        """Say whether accounts are on and this request carries no valid session."""
        return bool(self.remote.auth) and self._user() is None

    def _may(self, page):
        """Say whether the browser asking may open ``page``."""
        auth = self.remote.auth
        return page in auth.accounts.allowed_pages(self._user()) if auth else True

    def _deny(self):
        self._json(401, {'ok': False, 'error': 'login_required'})

    # ── Routing ──────────────────────────────────────────────────────────
    def _local_ok(self):
        """True without a local token, or when the request carries it."""
        token = self.remote.local_token
        if token is None:
            return True
        from http.cookies import SimpleCookie
        try:
            morsel = SimpleCookie(self.headers.get('Cookie') or '').get(LOCAL_COOKIE)
        except Exception:
            return False
        import hmac
        return bool(morsel) and hmac.compare_digest(morsel.value, token)

    def _ui(self, url, rel=None):
        """Serve the web UI's files; /ui/?t=<token> starts a local session."""
        token = self.remote.local_token
        if token is not None:
            given = (parse_qs(url.query).get('t') or [''])[0]
            import hmac
            if given and hmac.compare_digest(given, token):
                self.send_response(302)
                self.send_header('Location', '/ui/')
                self.send_header('Set-Cookie', '%s=%s; Path=/; HttpOnly; SameSite=Strict'
                                 % (LOCAL_COOKIE, token))
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            if not self._local_ok():
                self._json(403, {'ok': False, 'error': 'local_only'})
                return
        rel = rel or url.path[len('/ui/'):] or 'index.html'
        target = (UI_DIR / rel).resolve()
        kind = UI_TYPES.get(target.suffix)
        if kind is None or UI_DIR.resolve() not in target.parents:
            self._json(404, {'ok': False, 'error': 'not_found'})
            return
        self._static_file(target, kind)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == '/ui' or url.path.startswith('/ui/'):
            self._ui(url)
            return
        if url.path in ('/', '/index.html'):
            self._ui(url, 'index.html')          # remote clients get the same UI
            return
        if url.path.startswith('/api/') and not self._local_ok():
            self._json(403, {'ok': False, 'error': 'local_only'})
            return
        if url.path == '/api/ui':
            from fic_studio.web_remote.actions import LOCAL_ONLY
            self._json(200, {'ok': True, 'local': self.remote.local_token is not None,
                             'local_only': sorted(LOCAL_ONLY),
                             'pages': [{'id': i, 'label': l, 'group': g, 'ready': r, 'nav': n}
                                       for i, l, g, r, n in UI_PAGES]})
            return
        if url.path in STATIC_ROUTES:
            self._static(*STATIC_ROUTES[url.path])
            return
        if url.path == '/api/session':
            auth = self.remote.auth
            self._json(200, {'ok': True,
                             'signup': bool(auth) and self.remote.allow_signup,
                             'auth': auth.describe(self._user()) if auth
                             else {'signed_in': True, 'pages': list(ALL_PAGES),
                                   'admin': False, 'control': True, 'accounts': False}})
            return
        if self._needs_login():
            self._deny()
            return
        if url.path == '/api/users':
            self._users_get()
            return
        if url.path.startswith('/ext/'):
            self._ext_file(url.path[len('/ext/'):])
            return
        query = parse_qs(url.query)
        client = (query.get('client') or [''])[0][:64]
        if url.path == '/api/state':
            try:
                state = self.remote._snapshot()
            except Exception as exc:
                self._json(503, {'ok': False, 'error': 'studio_busy', 'message': str(exc)})
                return
            self._json(200, {'ok': True, 'state': state,
                             'control': self.remote.control_view(client)})
        elif url.path in ('/api/panels', '/api/panel'):
            if not self._may('extensions'):
                self._json(403, {'ok': False, 'error': 'not_allowed'})
            else:
                self._panels_json((query.get('id') or [''])[0][:64]
                                  if url.path == '/api/panel' else None)
        elif url.path == '/api/ext_state':
            if not self._may('extensions'):
                self._json(403, {'ok': False, 'error': 'not_allowed'})
            else:
                self._ext_state_json((query.get('id') or [''])[0][:64])
        elif url.path in ('/api/scene', '/api/mesh'):
            # The robot's shape is only for people allowed the 3D page.
            if not self._may('view3d'):
                self._json(403, {'ok': False, 'error': 'not_allowed'})
            elif url.path == '/api/scene':
                self._scene_json()
            else:
                self._mesh((query.get('i') or ['-1'])[0])
        elif url.path == '/api/events':
            self._events(client, (query.get('name') or [''])[0][:48])
        else:
            self._json(404, {'ok': False, 'error': 'not_found'})

    def do_POST(self):
        url = urlparse(self.path)
        if url.path not in ('/api/command', '/api/control', '/api/login',
                            '/api/logout', '/api/users', '/api/register',
                            '/api/my/extensions'):
            self._json(404, {'ok': False, 'error': 'not_found'})
            return
        if not self._local_ok():
            self._json(403, {'ok': False, 'error': 'local_only'})
            return
        # JSON-only: a page on another site can only send this content type
        # after a CORS preflight, which this server never answers, so a
        # website open on the phone cannot fire commands at the robot.
        ctype = self.headers.get('Content-Type', '').split(';')[0].strip().lower()
        if ctype != 'application/json':
            self._json(415, {'ok': False, 'error': 'json_required'})
            return
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            length = -1
        limit = MAX_LOCAL_BODY_BYTES if (url.path == '/api/command' and self.remote.local_token is not None) else MAX_BODY_BYTES
        if not 0 < length <= limit:
            self._json(400, {'ok': False, 'error': 'bad_length'})
            return
        try:
            body = json.loads(self.rfile.read(length))
        except ValueError:
            body = None
        if not isinstance(body, dict):
            self._json(400, {'ok': False, 'error': 'bad_json'})
            return
        if url.path == '/api/login':
            self._login(body)
            return
        if url.path == '/api/register':
            self._register(body)
            return
        if url.path == '/api/logout':
            if self.remote.auth:
                self.remote.auth.logout(self._token())
            self._json(200, {'ok': True}, cookie='')
            return
        if self._needs_login():
            self._deny()
            return
        if url.path == '/api/users':
            self._users_post(body)
            return
        if url.path == '/api/my/extensions':
            self._my_extensions_post(body)
            return
        client = str(body.get('client') or '')[:64]
        if not client:
            self._json(400, {'ok': False, 'error': 'client_required'})
            return
        # Commanding the robot needs the control page; E-STOP never does.
        auth, user = self.remote.auth, self._user()
        if auth and not auth.accounts.may_control(user):
            if url.path != '/api/command' or str(body.get('action') or '') not in NO_LEASE:
                self._json(403, {'ok': False, 'error': 'not_allowed'})
                return
        page = READ_PAGES.get(str(body.get('action') or '')) if url.path == '/api/command' else None
        if page and not self._may(page):
            self._json(403, {'ok': False, 'error': 'not_allowed'})
            return
        who = {'client': client, 'name': str(body.get('name') or '')[:48],
               'addr': self.client_address[0]}
        if user is not None:
            who['account'] = user['name']    # who it is, not what the browser calls itself
        if self.remote.local_token is not None:
            who['local'] = True              # the trusted desktop session (it carried the local token), not a phone
        if url.path == '/api/control':
            self._control(body, who)
        else:
            self._command(body, who)

    # ── Endpoints ────────────────────────────────────────────────────────
    def _control(self, body, who):
        lease, client = self.remote.lease, who['client']
        op = body.get('op')
        if op == 'take':
            # The desktop is the machine; an admin may take over from whoever holds it.
            auth = self.remote.auth
            forced = bool(body.get('force')) and (
                self.remote.local_token is not None
                or (auth is not None and auth.accounts.is_admin(self._user())))
            before = lease.holder()
            if forced:
                lease.revoke()
            granted, holder = lease.take(client, who.get('account') or who['name'], who['addr'])
            if granted and (before is None or before['client'] != client):
                self.remote._event('control_taken', who)
                from fic_studio import audit
                audit.record('control_taken', _who(who),
                             ('taken over from %s' if forced else 'from %s') % _who(before)
                             if before else '')
            reply = {'ok': granted, 'control': self.remote.control_view(client)}
            if not granted:
                reply['error'] = 'held'
                reply['message'] = '%s is in control' % (holder['name'] or holder['addr'])
            self._json(200 if granted else 409, reply)
        elif op == 'release':
            if lease.release(client):
                self.remote._event('control_released', who)
            self._json(200, {'ok': True, 'control': self.remote.control_view(client)})
        elif op == 'heartbeat':
            alive = lease.heartbeat(client)
            self._json(200, {'ok': alive, 'control': self.remote.control_view(client)})
        else:
            self._json(400, {'ok': False, 'error': 'bad_op'})

    def _command(self, body, who):
        action = str(body.get('action') or '')
        args = body.get('args') or {}
        if not isinstance(args, dict):
            self._json(400, {'ok': False, 'error': 'bad_args'})
            return
        # A command from the holder is as good a sign of life as a heartbeat,
        # so it renews the lease; anyone else is refused.
        if action not in NO_LEASE and not self.remote.lease.heartbeat(who['client']):
            self._json(403, {'ok': False, 'error': 'not_in_control',
                             'message': 'Take control first'})
            return
        # With accounts on, the Studio is told what this account may not do;
        # without them, nothing is added and the Studio sees what it always did.
        auth = self.remote.auth
        if auth is not None:
            user = self._user()
            who = dict(who, limits={'sim_only': auth.accounts.sim_only(user),
                                    'admin': auth.accounts.is_admin(user)})
        try:
            result = self.remote._dispatch(action, args, who)
        except TimeoutError as exc:
            self._json(504, {'ok': False, 'error': 'studio_busy', 'message': str(exc)})
            return
        except Exception as exc:
            self._json(500, {'ok': False, 'error': 'failed',
                             'message': '%s: %s' % (type(exc).__name__, exc)})
            return
        self._json(200, result if isinstance(result, dict) else {'ok': True})

    def _events(self, client, name):
        remote = self.remote
        sid = remote._open_stream(client, name, self.client_address[0])
        if sid is None:
            self._json(503, {'ok': False, 'error': 'too_many_streams'})
            return
        self.close_connection = True
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()
            self.wfile.write(b'retry: 1500\n\n')
            self.wfile.flush()
            last_seq, last_ctrl, last_write = -1, None, time.monotonic()
            with remote._cond:
                last_ev = remote._event_seq
            while True:
                with remote._cond:
                    # Wake on new state, or twice a second anyway so a lapsed
                    # lease shows up even while nothing else changes.
                    remote._cond.wait_for(
                        lambda: remote._stopping or remote._state_seq != last_seq or remote._event_seq != last_ev,
                        timeout=0.5)
                    if remote._stopping:
                        break
                    seq, data = remote._state_seq, remote._state_json
                    named = [e for e in remote._events_log if e[0] > last_ev]
                    last_ev = remote._event_seq
                out = [b'event: ' + n.encode() + b'\ndata: ' + d + b'\n\n' for _q, n, d in named]
                if seq != last_seq and data:
                    out.append(b'event: state\ndata: ' + data + b'\n\n')
                    last_seq = seq
                ctrl = encode_json(remote.control_view(client))
                if ctrl != last_ctrl:
                    out.append(b'event: control\ndata: ' + ctrl + b'\n\n')
                    last_ctrl = ctrl
                now = time.monotonic()
                if not out and now - last_write >= KEEPALIVE_S:
                    out.append(b': keepalive\n\n')
                if out:
                    self.wfile.write(b''.join(out))
                    self.wfile.flush()
                    last_write = now
        except OSError:
            pass                      # the browser went away
        finally:
            remote._close_stream(sid)

    # ── Accounts ─────────────────────────────────────────────────────────
    def _login(self, body):
        auth = self.remote.auth
        if auth is None:
            self._json(200, {'ok': True, 'auth': {'signed_in': True, 'accounts': False}})
            return
        name = str(body.get('user') or '')[:64]
        wait = auth.locked_for(name)
        if wait > 0:
            self._json(429, {'ok': False, 'error': 'too_many_tries',
                             'retry_after': int(wait) + 1})
            return
        token = auth.login(name, str(body.get('password') or ''))
        if not token:
            self._json(401, {'ok': False, 'error': 'bad_login'})
            return
        self._json(200, {'ok': True, 'auth': auth.describe(auth.user_for(token))},
                   cookie=token)

    def _register(self, body):
        """Make an account for a visitor: their own password, simulation only."""
        auth = self.remote.auth
        if auth is None or not self.remote.allow_signup:
            self._json(403, {'ok': False, 'error': 'no_signup',
                             'message': 'Ask an admin for an account'})
            return
        try:
            token = auth.sign_up(str(body.get('user') or ''),
                                 str(body.get('password') or ''))
        except ValueError as exc:
            self._json(400, {'ok': False, 'error': 'refused', 'message': str(exc)})
            return
        self.remote._on_accounts_changed()
        self._json(200, {'ok': True, 'auth': auth.describe(auth.user_for(token))},
                   cookie=token)

    def _users_get(self):
        auth = self.remote.auth
        if auth is None or not auth.accounts.is_admin(self._user()):
            self._json(403, {'ok': False, 'error': 'admin_only'})
            return
        self._json(200, {'ok': True, 'users': auth.accounts.public(),
                         'pages': list(ALL_PAGES)})

    def _my_extensions_post(self, body):
        """Set which extensions the signed-in person keeps in their page list.

        Their own view, so it needs no admin and no control lease: hiding a
        page cannot move a robot. It also cannot reveal anything — the list is
        only ever subtracted from what the Studio already loaded and what their
        account already allows. Needs the ``extensions`` page, because someone
        who cannot see extensions has no view to arrange.
        """
        auth = self.remote.auth
        if auth is None:
            # No accounts: there is no "my" to store this against.
            self._json(200, {'ok': True, 'hidden': []})
            return
        if not self._may('extensions'):
            self._json(403, {'ok': False, 'error': 'not_allowed'})
            return
        hidden = body.get('hidden')
        if not isinstance(hidden, list) or any(not isinstance(i, str) for i in hidden):
            self._json(400, {'ok': False, 'error': 'bad_args',
                             'message': 'hidden must be a list of extension ids'})
            return
        user = self._user()
        try:
            auth.accounts.set_hidden_extensions(user.get('name'), hidden[:200])
        except ValueError as exc:
            self._json(400, {'ok': False, 'error': 'refused', 'message': str(exc)})
            return
        self.remote._on_accounts_changed()
        self._json(200, {'ok': True,
                         'hidden': auth.accounts.hidden_extensions(self._user())})

    def _users_post(self, body):
        """Create, change or remove a user. Admins only."""
        auth = self.remote.auth
        if auth is None or not auth.accounts.is_admin(self._user()):
            self._json(403, {'ok': False, 'error': 'admin_only'})
            return
        name = str(body.get('user') or '')[:64].strip()
        if not name:
            self._json(400, {'ok': False, 'error': 'name_required'})
            return
        actor = '%s @ %s' % (self._user()['name'], self.client_address[0])   # before: they may remove themselves
        try:
            if body.get('remove'):
                if not auth.accounts.remove(name):
                    self._json(404, {'ok': False, 'error': 'no_such_user'})
                    return
            elif body.get('preset'):
                auth.accounts.apply_preset(
                    name, str(body['preset']),
                    password=(str(body['password']) if body.get('password') else None))
            else:
                pages = body.get('pages')
                sim_only = body.get('sim_only')
                auth.accounts.set_user(
                    name,
                    password=(str(body['password']) if body.get('password') else None),
                    role=(str(body['role']) if body.get('role') else None),
                    pages=(list(pages) if isinstance(pages, list) else None),
                    sim_only=(bool(sim_only) if sim_only is not None else None))
        except ValueError as exc:
            self._json(400, {'ok': False, 'error': 'refused', 'message': str(exc)})
            return
        from fic_studio import audit
        changed = {k: body[k] for k in ('preset', 'role', 'pages', 'sim_only', 'remove') if k in body}
        if body.get('password'):
            changed['password'] = 'set'
        audit.record('user_changed', actor, name, **changed)
        self.remote._on_accounts_changed()
        self._json(200, {'ok': True, 'users': auth.accounts.public()})

    # ── Extensions ───────────────────────────────────────────────────────
    def _panels_json(self, ext_id):
        """List the Studio's extensions, or describe one of them."""
        provider = self.remote._panels
        if provider is None:
            self._json(200, {'ok': True, 'panels': []})
            return
        try:
            data = provider() if ext_id is None else provider(ext_id)
        except TimeoutError:
            self._json(503, {'ok': False, 'error': 'studio_busy',
                             'message': 'The Studio is busy'})
            return
        except Exception as exc:
            self._json(500, {'ok': False, 'error': 'panel_failed', 'message': str(exc)})
            return
        if ext_id is None:
            # Mark, do not remove: the Extensions page has to list the ones this
            # person has put away so they can bring them back. Only the sidebar
            # leaves them out. With no accounts, nothing is hidden.
            hidden = []
            auth = self.remote.auth
            if auth is not None:
                hidden = auth.accounts.hidden_extensions(self._user())
            for entry in data or ():
                entry['hidden'] = entry.get('id') in hidden
            self._json(200, {'ok': True, 'panels': data, 'hidden': hidden})
        elif data is None:
            self._json(404, {'ok': False, 'error': 'no_such_extension'})
        else:
            self._json(200, {'ok': True, 'panel': data})

    def _ext_file(self, rest):
        """A file from an extension's web/ folder: /ext/<id>/<file>."""
        ext_id, _, rel = rest.partition('/')
        provider = self.remote._extensions
        target = None
        if provider is not None and ext_id and rel and self._may('extensions') and '\x00' not in rel:
            try:
                target = provider[0](ext_id, rel)
            except Exception:
                target = None
        kind = EXT_TYPES.get(Path(str(target)).suffix) if target is not None else None
        if target is None or kind is None:
            self._json(404, {'ok': False, 'error': 'not_found'})
            return
        self._static_file(target, kind)

    def _ext_state_json(self, ext_id):
        """An extension's web_state(), for its open web page."""
        provider = self.remote._extensions
        if provider is None or not ext_id:
            self._json(404, {'ok': False, 'error': 'no_such_extension'})
            return
        try:
            state = provider[1](ext_id)
        except TimeoutError:
            self._json(503, {'ok': False, 'error': 'studio_busy', 'message': 'The Studio is busy'})
            return
        except Exception as exc:
            self._json(500, {'ok': False, 'error': 'state_failed', 'message': str(exc)})
            return
        if state is None:
            self._json(404, {'ok': False, 'error': 'no_such_extension'})
        else:
            self._json(200, {'ok': True, 'state': state})

    # ── The 3D scene ─────────────────────────────────────────────────────
    def _scene_json(self):
        provider = self.remote._scene
        if provider is None:
            self._json(501, {'ok': False, 'error': 'no_3d_view'})
            return
        try:
            scene = provider(None)
        except Exception as exc:
            self._json(503, {'ok': False, 'error': 'scene_failed', 'message': str(exc)})
            return
        self._json(200, {'ok': True, 'scene': scene})

    def _mesh(self, index):
        provider = self.remote._scene
        try:
            data = provider(int(index))
        except (TypeError, ValueError, IndexError):
            self._json(404, {'ok': False, 'error': 'no_such_mesh'})
            return
        except Exception as exc:
            self._json(503, {'ok': False, 'error': 'mesh_failed', 'message': str(exc)})
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'max-age=86400')     # meshes never change
        self.end_headers()
        self.wfile.write(data)

    # ── Responses ────────────────────────────────────────────────────────
    def _static(self, fname, ctype):
        self._static_file(STATIC_DIR / fname, ctype)

    def _static_file(self, path, ctype):
        try:
            data = Path(path).read_bytes()
        except OSError:
            self._json(404, {'ok': False, 'error': 'not_found'})
            return
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(data)

    def _json(self, code, obj, cookie=None):
        data = encode_json(obj)
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        if cookie is not None:
            # HttpOnly: no page script can read it. Secure over HTTPS. SameSite
            # strict: another site cannot make the browser use this session.
            bits = ['%s=%s' % (AUTH_COOKIE, cookie), 'Path=/', 'HttpOnly', 'SameSite=Strict']
            if self.remote.scheme == 'https':
                bits.append('Secure')
            if not cookie:
                bits.append('Max-Age=0')
            self.send_header('Set-Cookie', '; '.join(bits))
        self.end_headers()
        self.wfile.write(data)
