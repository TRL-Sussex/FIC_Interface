"""The web remote's HTTP server, end to end over a real local socket."""
import http.client
import json
import socket
import time

from fic_studio.web_remote.lease import ControlLease
from fic_studio.web_remote.server import encode_json, WebRemoteServer
import pytest


class _Clock:
    """A clock the test moves by hand."""

    def __init__(self):
        self.now = 50.0

    def __call__(self):
        return self.now


@pytest.fixture
def remote():
    calls = []
    events = []

    def dispatch(action, args, who):
        calls.append((action, args, who))
        return {'ok': True, 'echo': action}

    clock = _Clock()
    server = WebRemoteServer(dispatch=dispatch, snapshot=lambda: {'hello': 1},
                             lease=ControlLease(timeout_s=3.0, clock=clock),
                             on_event=lambda kind, info: events.append(kind),
                             host='127.0.0.1')
    port = server.start(0)
    server.calls, server.events, server.clock, server.test_port = calls, events, clock, port
    yield server
    server.stop()


def _request(port, method, path, body=None, ctype='application/json'):
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    headers = {}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        headers['Content-Type'] = ctype
    conn.request(method, path, body=data, headers=headers)
    res = conn.getresponse()
    raw = res.read()
    conn.close()
    try:
        payload = json.loads(raw)
    except ValueError:
        payload = raw
    return res.status, payload, res


def _command(port, client, action, **args):
    return _request(port, 'POST', '/api/command',
                    {'client': client, 'name': client.upper(), 'action': action, 'args': args})


def _control(port, client, op):
    body = {'client': client, 'name': client.upper(), 'op': op}
    return _request(port, 'POST', '/api/control', body)


def test_serves_the_page_and_its_assets(remote):
    status, body, res = _request(remote.test_port, 'GET', '/')
    assert status == 200
    assert b'/ui/app.js' in body and res.getheader('Content-Type').startswith('text/html')
    for path in ('/manifest.webmanifest', '/icon.svg', '/view3d.js', '/three.module.js', '/ui/fic.css', '/ui/remote.js'):
        assert _request(remote.test_port, 'GET', path)[0] == 200
    for path in ('/app.js', '/app.css'):          # the old phone page is retired
        assert _request(remote.test_port, 'GET', path)[0] == 404


def test_serves_nothing_outside_its_route_table(remote):
    assert _request(remote.test_port, 'GET', '/../server.py')[0] == 404
    assert _request(remote.test_port, 'GET', '/static/app.js')[0] == 404


def test_state_endpoint_returns_snapshot_and_control_view(remote):
    status, body, _ = _request(remote.test_port, 'GET', '/api/state?client=a')
    assert status == 200
    assert body['state'] == {'hello': 1}
    assert body['control'] == {'you': False, 'holder': None, 'timeout_s': 3.0}


def test_commands_must_be_json(remote):
    status, body, _ = _request(remote.test_port, 'POST', '/api/command',
                               {'client': 'a', 'action': 'home'}, ctype='text/plain')
    assert status == 415
    assert remote.calls == []


def test_commands_need_control(remote):
    status, body, _ = _command(remote.test_port, 'a', 'home')
    assert status == 403 and body['error'] == 'not_in_control'
    assert remote.calls == []


def test_holder_can_command_and_others_cannot(remote):
    assert _control(remote.test_port, 'a', 'take')[0] == 200
    status, body, _ = _control(remote.test_port, 'b', 'take')
    assert status == 409 and body['error'] == 'held' and 'A' in body['message']

    status, body, _ = _command(remote.test_port, 'a', 'set_mode', index=2)
    assert status == 200 and body == {'ok': True, 'echo': 'set_mode'}
    action, args, who = remote.calls[-1]
    assert (action, args) == ('set_mode', {'index': 2})
    assert who == {'client': 'a', 'name': 'A', 'addr': '127.0.0.1'}

    assert _command(remote.test_port, 'b', 'set_mode', index=2)[0] == 403
    assert 'control_taken' in remote.events


def test_estop_works_without_control(remote):
    _control(remote.test_port, 'a', 'take')
    status, body, _ = _command(remote.test_port, 'b', 'estop')
    assert status == 200
    assert remote.calls[-1][0] == 'estop'


def test_control_passes_on_when_the_holder_goes_quiet(remote):
    _control(remote.test_port, 'a', 'take')
    remote.clock.now += 3.5
    status, body, _ = _control(remote.test_port, 'b', 'take')
    assert status == 200 and body['control']['you'] is True
    assert _command(remote.test_port, 'a', 'home')[0] == 403


def test_commands_from_the_holder_renew_control(remote):
    _control(remote.test_port, 'a', 'take')
    for _ in range(3):
        remote.clock.now += 2.5
        assert _command(remote.test_port, 'a', 'home')[0] == 200
    assert remote.lease.holds('a')


def test_heartbeat_and_release(remote):
    _control(remote.test_port, 'a', 'take')
    remote.clock.now += 2.0
    status, body, _ = _control(remote.test_port, 'a', 'heartbeat')
    assert body['ok'] and body['control']['you']
    _control(remote.test_port, 'a', 'release')
    assert remote.lease.holder() is None
    assert 'control_released' in remote.events


def test_dispatch_timeout_is_reported_as_busy():
    def slow(action, args, who):
        raise TimeoutError('the Studio did not respond in time')
    server = WebRemoteServer(dispatch=slow, snapshot=dict, host='127.0.0.1')
    port = server.start(0)
    try:
        status, body, _ = _command(port, 'a', 'estop')
        assert status == 504 and body['error'] == 'studio_busy'
    finally:
        server.stop()


def _read_events(sock_file, want, deadline):
    """Read SSE blocks until every event name in ``want`` has been seen."""
    seen = {}
    block = {}
    while want - set(seen) and time.monotonic() < deadline:
        line = sock_file.readline().decode().rstrip('\n')
        if line.startswith('event: '):
            block['event'] = line[7:]
        elif line.startswith('data: '):
            block['data'] = json.loads(line[6:])
        elif line == '' and block:
            seen[block.get('event')] = block.get('data')
            block = {}
    return seen


def test_event_stream_delivers_state_and_control(remote):
    sock = socket.create_connection(('127.0.0.1', remote.test_port), timeout=5)
    sock.sendall(b'GET /api/events?client=a&name=A HTTP/1.1\r\nHost: x\r\n\r\n')
    stream = sock.makefile('rb')
    assert b'200' in stream.readline()
    while stream.readline() not in (b'\r\n', b''):
        pass                                     # headers
    deadline = time.monotonic() + 5
    while remote.stream_count() == 0 and time.monotonic() < deadline:
        time.sleep(0.02)
    assert remote.stream_count() == 1
    remote.publish({'joints': [1.5], 'bad': float('nan')})
    seen = _read_events(stream, {'state', 'control'}, deadline)
    assert seen['state'] == {'joints': [1.5], 'bad': None}
    assert seen['control']['you'] is False
    assert remote.clients()[0]['name'] == 'A'
    sock.close()


def test_a_taken_port_moves_to_the_next_one():
    blocker = socket.socket()
    blocker.bind(('127.0.0.1', 0))
    blocker.listen(1)
    taken = blocker.getsockname()[1]
    server = WebRemoteServer(dispatch=lambda *a: {'ok': True}, snapshot=dict, host='127.0.0.1')
    try:
        bound = server.start(taken, attempts=5)
        assert bound != taken and taken < bound <= taken + 4
    finally:
        server.stop()
        blocker.close()


def test_non_finite_numbers_become_null():
    assert json.loads(encode_json({'a': [float('inf'), 1.0], 'b': float('nan')})) == \
        {'a': [None, 1.0], 'b': None}
