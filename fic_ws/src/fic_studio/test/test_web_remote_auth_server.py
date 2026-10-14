# Copyright 2025 Adnan Takriti
# SPDX-License-Identifier: MIT
"""Signing in to the web remote, over a real socket."""
import http.client
import json

from fic_studio.web_remote.accounts import Accounts
from fic_studio.web_remote.auth import Auth
from fic_studio.web_remote.lease import ControlLease
from fic_studio.web_remote.server import WebRemoteServer
import pytest


class Client:
    """A browser: keeps the session cookie the server hands out."""

    def __init__(self, port):
        self.port = port
        self.cookie = ''

    def request(self, method, path, body=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        headers = {}
        data = None
        if body is not None:
            data = json.dumps(body).encode()
            headers['Content-Type'] = 'application/json'
        if self.cookie:
            headers['Cookie'] = self.cookie
        conn.request(method, path, body=data, headers=headers)
        res = conn.getresponse()
        raw = res.read()
        set_cookie = res.getheader('Set-Cookie')
        conn.close()
        if set_cookie:
            self.cookie = set_cookie.split(';')[0]
        try:
            payload = json.loads(raw)
        except ValueError:
            payload = raw
        return res.status, payload, set_cookie

    def get(self, path):
        return self.request('GET', path)

    def post(self, path, body):
        return self.request('POST', path, body)

    def sign_in(self, user, password):
        return self.post('/api/login', {'user': user, 'password': password})


@pytest.fixture
def remote(tmp_path):
    accounts = Accounts(str(tmp_path / 'accounts.json'))
    accounts.set_user('admin', password='admin-pass', role='admin')
    accounts.set_user('sam', password='sam-pass', pages=['view3d', 'dashboard'])
    accounts.set_user('kit', password='kit-pass', pages=['control', 'dashboard'])
    saved = []
    server = WebRemoteServer(dispatch=lambda action, args, who: {'ok': True, 'echo': action},
                             snapshot=lambda: {'hello': 1},
                             lease=ControlLease(timeout_s=3.0),
                             host='127.0.0.1', auth=Auth(accounts))
    server.on_accounts_changed = lambda: saved.append(True)
    server.accounts, server.saved = accounts, saved
    server.test_port = server.start(0)
    yield server
    server.stop()


def test_without_a_session_nothing_but_the_page_and_login(remote):
    browser = Client(remote.test_port)
    assert browser.get('/api/state')[0] == 401
    assert browser.post('/api/command', {'client': 'c', 'action': 'home'})[0] == 401
    # the page itself must still load, or there is nowhere to sign in
    assert browser.get('/')[0] == 200
    status, payload, _ = browser.get('/api/session')
    assert status == 200 and payload['auth'] == {'signed_in': False, 'pages': [],
                                                 'admin': False, 'control': False}


def test_signing_in_sets_a_locked_down_cookie(remote):
    browser = Client(remote.test_port)
    status, payload, set_cookie = browser.sign_in('sam', 'sam-pass')
    assert status == 200 and payload['auth']['name'] == 'sam'
    assert payload['auth']['pages'] == ['view3d', 'dashboard']
    assert payload['auth']['control'] is False
    assert 'HttpOnly' in set_cookie and 'SameSite=Strict' in set_cookie
    assert browser.get('/api/state')[0] == 200


def test_a_wrong_password_is_refused(remote):
    browser = Client(remote.test_port)
    status, payload, _ = browser.sign_in('sam', 'nope')
    assert status == 401 and payload['error'] == 'bad_login'
    assert browser.get('/api/state')[0] == 401


def test_only_the_control_page_may_command_but_anyone_may_estop(remote):
    browser = Client(remote.test_port)
    browser.sign_in('sam', 'sam-pass')                    # no control page
    status, payload, _ = browser.post('/api/command', {'client': 'c', 'action': 'home'})
    assert status == 403 and payload['error'] == 'not_allowed'
    assert browser.post('/api/command', {'client': 'c', 'action': 'estop'})[0] == 200
    browser.post('/api/logout', {})
    browser.sign_in('kit', 'kit-pass')                    # has the control page
    assert browser.post('/api/control', {'client': 'c', 'op': 'take'})[0] == 200
    assert browser.post('/api/command', {'client': 'c', 'action': 'home'})[0] == 200


def test_signing_out_ends_the_session(remote):
    browser = Client(remote.test_port)
    browser.sign_in('kit', 'kit-pass')
    assert browser.post('/api/logout', {})[0] == 200
    assert browser.get('/api/state')[0] == 401


def test_users_are_the_admin_s_business(remote):
    sam = Client(remote.test_port)
    sam.sign_in('sam', 'sam-pass')
    assert sam.get('/api/users')[0] == 403
    assert sam.post('/api/users', {'client': 'c', 'user': 'sam', 'role': 'admin'})[0] == 403

    admin = Client(remote.test_port)
    admin.sign_in('admin', 'admin-pass')
    status, payload, _ = admin.get('/api/users')
    assert status == 200
    assert [u['name'] for u in payload['users']] == ['admin', 'kit', 'sam']
    assert 'hash' not in payload['users'][0] and 'salt' not in payload['users'][0]
    assert payload['pages'] == ['control', 'view3d', 'dashboard', 'session', 'log',
                                'extensions']


def test_an_admin_changes_what_someone_may_see(remote):
    admin = Client(remote.test_port)
    admin.sign_in('admin', 'admin-pass')
    status, payload, _ = admin.post('/api/users',
                                    {'client': 'c', 'user': 'sam', 'pages': ['control']})
    assert status == 200
    assert remote.saved == [True]                     # the Studio was told to save
    sam = Client(remote.test_port)
    sam.sign_in('sam', 'sam-pass')
    assert sam.post('/api/command', {'client': 'c', 'action': 'estop'})[0] == 200
    assert [u for u in payload['users'] if u['name'] == 'sam'][0]['pages'] == ['control']


def test_a_new_user_needs_a_password_and_the_last_admin_stays(remote):
    admin = Client(remote.test_port)
    admin.sign_in('admin', 'admin-pass')
    status, payload, _ = admin.post('/api/users', {'client': 'c', 'user': 'new'})
    assert status == 400 and payload['error'] == 'refused'
    status, payload, _ = admin.post('/api/users',
                                    {'client': 'c', 'user': 'admin', 'remove': True})
    assert status == 400 and 'only admin' in payload['message']
    assert admin.post('/api/users', {'client': 'c', 'user': 'kit', 'remove': True})[0] == 200


def test_a_removed_user_s_session_stops_working(remote):
    kit = Client(remote.test_port)
    kit.sign_in('kit', 'kit-pass')
    assert kit.get('/api/state')[0] == 200
    admin = Client(remote.test_port)
    admin.sign_in('admin', 'admin-pass')
    admin.post('/api/users', {'client': 'c', 'user': 'kit', 'remove': True})
    assert kit.get('/api/state')[0] == 401


def test_a_made_up_cookie_is_not_a_session(remote):
    browser = Client(remote.test_port)
    browser.cookie = 'fic_session=' + 'a' * 43
    assert browser.get('/api/state')[0] == 401


def test_the_robot_s_shape_needs_the_3d_page(remote):
    kit = Client(remote.test_port)
    kit.sign_in('kit', 'kit-pass')                    # control + dashboard only
    assert kit.get('/api/scene')[0] == 403
    assert kit.get('/api/mesh?i=0')[0] == 403


def test_a_visitor_signs_themselves_up_and_is_signed_in(remote):
    browser = Client(remote.test_port)
    status, payload, set_cookie = browser.post('/api/register',
                                               {'user': 'yasir', 'password': 'a-good-password'})
    assert status == 200 and 'HttpOnly' in set_cookie
    assert payload['auth']['name'] == 'yasir'
    assert payload['auth']['sim_only'] is True
    assert payload['auth']['pages'] == ['control', 'view3d', 'dashboard', 'session']
    assert browser.get('/api/state')[0] == 200        # signed in already
    assert remote.saved == [True]                     # the Studio saves and shares it


def test_signing_up_cannot_take_an_existing_name(remote):
    browser = Client(remote.test_port)
    status, payload, _ = browser.post('/api/register',
                                      {'user': 'admin', 'password': 'a-good-password'})
    assert status == 400 and 'taken' in payload['message']


def test_signing_up_can_be_turned_off(remote):
    remote.allow_signup = False
    browser = Client(remote.test_port)
    status, payload, _ = browser.post('/api/register',
                                      {'user': 'kim', 'password': 'a-good-password'})
    assert status == 403 and payload['error'] == 'no_signup'
    assert browser.get('/api/session')[1]['signup'] is False


def test_the_page_is_told_whether_it_may_offer_an_account(remote):
    assert Client(remote.test_port).get('/api/session')[1]['signup'] is True


def test_a_new_account_reaches_the_studio_as_simulation_only(remote):
    """The Studio decides what that means; the server passes on who is asking."""
    seen = []
    remote._dispatch = lambda action, args, who: seen.append(who) or {'ok': True}
    browser = Client(remote.test_port)
    browser.post('/api/register', {'user': 'yasir', 'password': 'a-good-password'})
    browser.post('/api/control', {'client': 'c', 'op': 'take'})
    browser.post('/api/command', {'client': 'c', 'action': 'home'})
    assert seen and seen[-1]['limits'] == {'sim_only': True}

    admin = Client(remote.test_port)
    admin.sign_in('admin', 'admin-pass')
    admin.post('/api/users', {'client': 'c', 'user': 'yasir', 'sim_only': False})
    browser.post('/api/command', {'client': 'c', 'action': 'home'})
    assert seen[-1]['limits'] == {'sim_only': False}
