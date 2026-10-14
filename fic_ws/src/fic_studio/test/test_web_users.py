"""Managing users over HTTP, and each person's own view of the extension list.

Two things meet here, so they are tested against a real server rather than the
store alone: an admin changing someone's account (``/api/users``), and anyone
with the Extensions page putting individual extensions away (
``/api/my/extensions``). The second must never be a way to gain the first.
"""
import http.client
import json

import pytest

from fic_studio.web_remote.accounts import Accounts
from fic_studio.web_remote.auth import Auth
from fic_studio.web_remote.server import WebRemoteServer

PANELS = [
    {'id': 'gaze', 'label': 'Gaze', 'section': 'Programming', 'description': 'eyes'},
    {'id': 'sequences', 'label': 'Sequences', 'section': 'Programming', 'description': 'play',
     'web': {'page': '/ext/sequences/page.js', 'css': None, 'poll_ms': 300}},
    {'id': 'teach_mode', 'label': 'Teach Mode', 'section': 'Programming', 'description': 'hand'},
]


@pytest.fixture
def lab(tmp_path):
    """A server with an admin, a student, and three extensions loaded."""
    store = Accounts(str(tmp_path / 'accounts.json'))
    store.set_user('admin', password='adminpass1', role='admin')
    store.sign_up('student', 'studentpass1')
    store.set_user('student', pages=['view3d', 'dashboard', 'extensions'])

    def panels(ext_id=None):
        if ext_id is None:
            return [dict(p) for p in PANELS]
        return next((dict(p) for p in PANELS if p['id'] == ext_id), None)

    server = WebRemoteServer(dispatch=lambda a, b, c: {'ok': True}, snapshot=lambda: {},
                             host='127.0.0.1', panels=panels, auth=Auth(store))
    port = server.start(0)
    yield _Client(port), store
    server.stop()


class _Client:
    """The few HTTP calls these tests make, with a cookie per person."""

    def __init__(self, port):
        self.port = port

    def call(self, path, body=None, cookie=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=5)
        headers = {}
        data = None
        if body is not None:
            data = json.dumps(body)
            headers['Content-Type'] = 'application/json'
        if cookie:
            headers['Cookie'] = cookie
        conn.request('POST' if data is not None else 'GET', path, data, headers)
        r = conn.getresponse()
        out = (r.status, json.loads(r.read() or b'{}'), r.getheader('Set-Cookie') or '')
        conn.close()
        return out

    def sign_in(self, user, password):
        status, body, raw = self.call('/api/login', {'user': user, 'password': password})
        assert status == 200 and body.get('ok'), (status, body)
        return raw.split(';')[0]


class TestManagingUsers:
    def test_an_admin_reads_the_list_and_a_user_does_not(self, lab):
        c, _ = lab
        admin, student = c.sign_in('admin', 'adminpass1'), c.sign_in('student', 'studentpass1')
        status, body, _ = c.call('/api/users', cookie=admin)
        assert status == 200
        assert sorted(u['name'] for u in body['users']) == ['admin', 'student']
        assert c.call('/api/users', cookie=student)[0] == 403

    def test_a_user_cannot_change_accounts(self, lab):
        c, store = lab
        student = c.sign_in('student', 'studentpass1')
        status, body, _ = c.call('/api/users', {'user': 'student', 'role': 'admin'}, cookie=student)
        assert (status, body.get('error')) == (403, 'admin_only')
        assert store.get('student')['role'] == 'user'

    def test_an_admin_adds_changes_and_removes(self, lab):
        c, _ = lab
        admin = c.sign_in('admin', 'adminpass1')
        status, body, _ = c.call('/api/users', {'user': 'newbie', 'password': 'x' * 9}, cookie=admin)
        assert status == 200 and 'newbie' in [u['name'] for u in body['users']]
        _, body, _ = c.call('/api/users', {'user': 'newbie', 'sim_only': False}, cookie=admin)
        assert [u['sim_only'] for u in body['users'] if u['name'] == 'newbie'] == [False]
        _, body, _ = c.call('/api/users', {'user': 'newbie', 'remove': True}, cookie=admin)
        assert 'newbie' not in [u['name'] for u in body['users']]

    def test_the_last_admin_cannot_be_removed(self, lab):
        c, _ = lab
        admin = c.sign_in('admin', 'adminpass1')
        status, body, _ = c.call('/api/users', {'user': 'admin', 'remove': True}, cookie=admin)
        assert status == 400 and body.get('error') == 'refused'


class TestMyExtensions:
    def test_the_list_arrives_with_nothing_hidden(self, lab):
        c, _ = lab
        student = c.sign_in('student', 'studentpass1')
        status, body, _ = c.call('/api/panels', cookie=student)
        assert status == 200
        assert [p['id'] for p in body['panels']] == ['gaze', 'sequences', 'teach_mode']
        assert [p['hidden'] for p in body['panels']] == [False, False, False]

    def test_hiding_marks_it_without_dropping_it(self, lab):
        """The page still lists it, because that is where you get it back."""
        c, _ = lab
        student = c.sign_in('student', 'studentpass1')
        status, body, _ = c.call('/api/my/extensions', {'hidden': ['gaze']}, cookie=student)
        assert (status, body['hidden']) == (200, ['gaze'])
        _, body, _ = c.call('/api/panels', cookie=student)
        assert len(body['panels']) == 3
        assert [(p['id'], p['hidden']) for p in body['panels']] == [
            ('gaze', True), ('sequences', False), ('teach_mode', False)]

    def test_it_is_one_person_s_view_not_everyone_s(self, lab):
        c, _ = lab
        student, admin = c.sign_in('student', 'studentpass1'), c.sign_in('admin', 'adminpass1')
        c.call('/api/my/extensions', {'hidden': ['gaze']}, cookie=student)
        _, body, _ = c.call('/api/panels', cookie=admin)
        assert [p['hidden'] for p in body['panels']] == [False, False, False]

    def test_an_empty_list_brings_them_back(self, lab):
        c, _ = lab
        student = c.sign_in('student', 'studentpass1')
        c.call('/api/my/extensions', {'hidden': ['gaze', 'teach_mode']}, cookie=student)
        status, body, _ = c.call('/api/my/extensions', {'hidden': []}, cookie=student)
        assert (status, body['hidden']) == (200, [])

    def test_it_grants_nothing(self, lab):
        """Hiding is subtraction: it must not widen pages, role or hardware access."""
        c, store = lab
        student = c.sign_in('student', 'studentpass1')
        before = dict(store.get('student'))
        c.call('/api/my/extensions', {'hidden': ['control', 'admin']}, cookie=student)
        after = store.get('student')
        assert after['pages'] == before['pages']
        assert after['role'] == before['role']
        assert Accounts.sim_only(after) is True

    def test_rubbish_is_refused(self, lab):
        c, _ = lab
        student = c.sign_in('student', 'studentpass1')
        assert c.call('/api/my/extensions', {'hidden': 'gaze'}, cookie=student)[0] == 400
        assert c.call('/api/my/extensions', {'hidden': [1, 2]}, cookie=student)[0] == 400

    def test_signed_out_is_refused(self, lab):
        c, _ = lab
        assert c.call('/api/my/extensions', {'hidden': []})[0] == 401

    def test_without_the_extensions_page_there_is_no_view_to_arrange(self, lab):
        c, store = lab
        student = c.sign_in('student', 'studentpass1')
        store.set_user('student', pages=['view3d'])
        status, body, _ = c.call('/api/my/extensions', {'hidden': ['gaze']}, cookie=student)
        assert (status, body.get('error')) == (403, 'not_allowed')
