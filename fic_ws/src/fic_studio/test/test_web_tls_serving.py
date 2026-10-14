# Copyright 2025 Adnan Takriti
# SPDX-License-Identifier: MIT
"""Serving over HTTPS: one bad client must not stop the web remote."""
import http.client
import socket
import ssl

from fic_studio.web_remote import tls
from fic_studio.web_remote.lease import ControlLease
from fic_studio.web_remote.server import WebRemoteServer
import pytest


@pytest.fixture
def secure(tmp_path):
    cert, key = tls.ensure_cert(str(tmp_path), ['127.0.0.1'])
    server = WebRemoteServer(dispatch=lambda a, g, w: {'ok': True},
                             snapshot=lambda: {'hello': 1},
                             lease=ControlLease(timeout_s=3.0),
                             host='127.0.0.1', tls_files=(cert, key))
    server.test_port = server.start(0)
    server.cert = cert
    yield server
    server.stop()


def _get(server, path='/api/state'):
    context = ssl.create_default_context(cafile=server.cert)
    context.check_hostname = False
    conn = http.client.HTTPSConnection('127.0.0.1', server.test_port,
                                       context=context, timeout=10)
    conn.request('GET', path)
    status = conn.getresponse().status
    conn.close()
    return status


def test_https_works(secure):
    assert _get(secure) == 200
    assert secure.scheme == 'https'


def test_a_client_that_says_nothing_does_not_stop_the_server(secure):
    """The bug that wedged robot1: a connection with no TLS hello."""
    quiet = [socket.create_connection(('127.0.0.1', secure.test_port), timeout=5)
             for _ in range(5)]
    try:
        assert _get(secure) == 200          # everyone else is still served
    finally:
        for sock in quiet:
            sock.close()


def test_an_http_address_is_redirected_to_https(secure):
    """An old http:// bookmark gets the new address, not a dead page."""
    conn = http.client.HTTPConnection('127.0.0.1', secure.test_port, timeout=10)
    conn.request('GET', '/api/state?client=c', headers={'Host': 'robot1:8088'})
    res = conn.getresponse()
    assert res.status == 301
    assert res.getheader('Location') == 'https://robot1:8088/api/state?client=c'
    conn.close()
    assert _get(secure) == 200              # and the server is fine afterwards


def test_rubbish_on_the_port_is_dropped(secure):
    sock = socket.create_connection(('127.0.0.1', secure.test_port), timeout=5)
    sock.sendall(b'\x00\x01\x02not tls at all\n')
    sock.close()
    assert _get(secure) == 200


def test_a_browser_that_walks_off_is_not_a_traceback(secure, capfd):
    """A phone leaving wifi mid-page must not spill into the Studio's log."""
    context = ssl.create_default_context(cafile=secure.cert)
    context.check_hostname = False
    raw = socket.create_connection(('127.0.0.1', secure.test_port), timeout=5)
    conn = context.wrap_socket(raw)
    conn.sendall(b'GET / HTTP/1.1\r\nHost: x\r\n\r\n')
    conn.close()                              # gone before the answer is written
    assert _get(secure) == 200                # and the server carries on
    assert 'Traceback' not in capfd.readouterr().err
