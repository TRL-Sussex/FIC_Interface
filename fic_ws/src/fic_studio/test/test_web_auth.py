"""Signing in to the web remote, and the certificate it serves with."""
import os
import ssl
import threading
import http.client

from fic_studio.web_remote.accounts import Accounts
from fic_studio.web_remote.auth import Auth, LIFETIME_S, MAX_FAILURES
from fic_studio.web_remote import tls
import pytest


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def auth(tmp_path):
    store = Accounts(str(tmp_path / 'accounts.json'))
    store.set_user('alice', password='pw', role='admin')
    store.set_user('bob', password='pw2', pages=['view3d'])
    return Auth(store, clock=_Clock())


class TestSessions:
    def test_a_good_password_gives_a_session_and_a_wrong_one_does_not(self, auth):
        token = auth.login('alice', 'pw')
        assert token and auth.user_for(token)['name'] == 'alice'
        assert auth.login('alice', 'nope') is None
        assert auth.login('nobody', 'pw') is None

    def test_an_unknown_token_is_nobody(self, auth):
        assert auth.user_for('made-up') is None
        assert auth.user_for(None) is None

    def test_signing_out_ends_that_session_only(self, auth):
        one, two = auth.login('alice', 'pw'), auth.login('bob', 'pw2')
        auth.logout(one)
        assert auth.user_for(one) is None and auth.user_for(two)['name'] == 'bob'

    def test_a_session_expires_when_left_alone(self, auth):
        token = auth.login('alice', 'pw')
        auth._clock.t += LIFETIME_S + 1
        assert auth.user_for(token) is None

    def test_using_a_session_keeps_it_alive(self, auth):
        token = auth.login('alice', 'pw')
        for _ in range(4):
            auth._clock.t += LIFETIME_S * 0.6
            assert auth.user_for(token) is not None

    def test_removing_an_account_ends_its_sessions(self, auth):
        token = auth.login('bob', 'pw2')
        auth.accounts.remove('bob')
        assert auth.user_for(token) is None

    def test_an_admin_can_sign_everyone_out(self, auth):
        token = auth.login('alice', 'pw')
        auth.sign_out_everyone()
        assert auth.user_for(token) is None and auth.sessions() == 0


class TestGuessing:
    def test_repeated_wrong_passwords_make_that_name_wait(self, auth):
        for _ in range(MAX_FAILURES):
            assert auth.login('alice', 'wrong') is None
        assert auth.locked_for('alice') > 0
        assert auth.login('alice', 'pw') is None        # even the right one waits
        auth._clock.t += 61
        assert auth.locked_for('alice') == 0
        assert auth.login('alice', 'pw')

    def test_one_name_waiting_does_not_stop_another(self, auth):
        for _ in range(MAX_FAILURES):
            auth.login('alice', 'wrong')
        assert auth.login('bob', 'pw2')

    def test_a_good_password_clears_the_count(self, auth):
        for _ in range(MAX_FAILURES - 1):
            auth.login('alice', 'wrong')
        assert auth.login('alice', 'pw')
        for _ in range(MAX_FAILURES - 1):
            auth.login('alice', 'wrong')
        assert auth.locked_for('alice') == 0


class TestDescribe:
    def test_what_the_page_is_told(self, auth):
        alice = auth.describe(auth.user_for(auth.login('alice', 'pw')))
        bob = auth.describe(auth.user_for(auth.login('bob', 'pw2')))
        assert alice['admin'] and alice['control'] and alice['signed_in']
        assert not bob['admin'] and not bob['control']
        assert bob['pages'] == ['view3d']
        assert auth.describe(None) == {'signed_in': False, 'pages': [],
                                       'admin': False, 'control': False}


class TestCertificate:
    def test_a_certificate_is_made_once_and_kept(self, tmp_path):
        cert, key = tls.ensure_cert(str(tmp_path), ['192.168.150.11'])
        assert os.path.exists(cert) and os.path.exists(key)
        assert oct(os.stat(key).st_mode & 0o777) == '0o600'
        again = tls.ensure_cert(str(tmp_path), ['192.168.150.11'])
        assert again == (cert, key)
        assert os.path.getmtime(cert) == os.path.getmtime(again[0])

    def test_it_names_this_machine_and_its_addresses(self, tmp_path):
        x509 = pytest.importorskip('cryptography.x509')
        cert, _ = tls.ensure_cert(str(tmp_path), ['192.168.150.11'])
        with open(cert, 'rb') as handle:
            loaded = x509.load_pem_x509_certificate(handle.read())
        names = loaded.extensions.get_extension_for_class(
            x509.SubjectAlternativeName).value
        assert '192.168.150.11' in [str(v) for v in names.get_values_for_type(x509.IPAddress)]
        assert 'localhost' in names.get_values_for_type(x509.DNSName)

    def test_a_browser_really_can_speak_tls_to_it(self, tmp_path):
        """Serve one request over the generated certificate."""
        import http.server
        cert, key = tls.ensure_cert(str(tmp_path), [])

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'hello')

            def log_message(self, *args):
                pass

        server = http.server.HTTPServer(('127.0.0.1', 0), Handler)
        server.socket = tls.context(cert, key).wrap_socket(server.socket, server_side=True)
        threading.Thread(target=server.handle_request, daemon=True).start()
        ctx = ssl.create_default_context(cafile=cert)
        ctx.check_hostname = False
        conn = http.client.HTTPSConnection('127.0.0.1', server.server_port, context=ctx, timeout=10)
        conn.request('GET', '/')
        assert conn.getresponse().read() == b'hello'
        server.server_close()
