"""Roles on top of accounts (observer, simulation operator, operator, admin), admin-only
commands, an admin taking control over, and the read-only audit view."""
import json

from fic_studio import audit
from fic_studio.web_remote import actions
from fic_studio.web_remote.accounts import PRESETS, Accounts
from fic_studio.web_remote.auth import Auth
from fic_studio.web_remote.lease import ControlLease
from fic_studio.web_remote.server import NO_LEASE, WebRemoteServer
import pytest
from test_web_remote_actions import WHO, _Studio
from test_web_remote_auth_server import Client


@pytest.fixture(autouse=True)
def audit_file(tmp_path, monkeypatch):
    monkeypatch.setenv('FIC_STUDIO_AUDIT', str(tmp_path / 'audit.log'))
    audit.reset_for_tests()
    yield tmp_path / 'audit.log'
    audit.reset_for_tests()


@pytest.fixture
def remote(tmp_path):
    accounts = Accounts(str(tmp_path / 'accounts.json'))
    accounts.set_user('admin', password='admin-pass', role='admin')
    accounts.apply_preset('olly', 'observer', password='olly-pass')
    accounts.apply_preset('kit', 'operator', password='kit-pass')
    seen = []
    server = WebRemoteServer(dispatch=lambda action, args, who: seen.append((action, who)) or {'ok': True},
                             snapshot=lambda: {}, lease=ControlLease(timeout_s=3.0),
                             host='127.0.0.1', auth=Auth(accounts))
    server.on_accounts_changed = lambda: None
    server.accounts, server.seen = accounts, seen
    server.test_port = server.start(0)
    yield server
    server.stop()


def test_presets_fill_in_role_pages_and_simulation_only(tmp_path):
    acc = Accounts(str(tmp_path / 'a.json'))
    acc.set_user('root', password='pw', role='admin')
    for preset in PRESETS:
        acc.apply_preset('p_' + preset, preset, password='pw')
        assert acc.kind(acc.get('p_' + preset)) == preset
    assert not acc.may_control(acc.get('p_observer'))
    assert acc.sim_only(acc.get('p_sim_operator')) and not acc.sim_only(acc.get('p_operator'))
    # Pages ticked by hand still read back as a kind.
    acc.set_user('hand', password='pw', pages=['control', 'dashboard'], sim_only=True)
    assert acc.kind(acc.get('hand')) == 'sim_operator'
    assert {u['name']: u['kind'] for u in acc.public()}['hand'] == 'sim_operator'
    with pytest.raises(ValueError):
        acc.apply_preset('p_admin', 'nobody')


def test_the_only_admin_cannot_be_made_an_observer(tmp_path):
    acc = Accounts(str(tmp_path / 'a.json'))
    acc.set_user('root', password='pw', role='admin')
    with pytest.raises(ValueError):
        acc.apply_preset('root', 'observer')
    acc.apply_preset('second', 'admin', password='pw')
    acc.apply_preset('root', 'observer')
    assert acc.kind(acc.get('root')) == 'observer'


def test_safety_limits_tuning_and_clearing_the_log_are_for_admins():
    w = _Studio()
    for action in actions.ADMIN_ONLY:
        reply = actions.handle_command(w, action, {}, dict(WHO, limits={'sim_only': False, 'admin': False}))
        assert reply['error'] == 'admin_only', action
    # An admin, or a Studio without accounts (no limits), gets past the role check.
    for client in (dict(WHO, limits={'sim_only': False, 'admin': True}), WHO):
        assert actions.handle_command(w, 'log_clear', {}, client).get('error') != 'admin_only'


def test_an_admin_sets_a_preset_and_it_is_audited(remote, audit_file):
    admin = Client(remote.test_port)
    admin.sign_in('admin', 'admin-pass')
    status, payload, _ = admin.post('/api/users', {'client': 'c', 'user': 'new', 'password': 'pw-12345',
                                                   'preset': 'sim_operator'})
    assert status == 200
    assert {u['name']: u['kind'] for u in payload['users']}['new'] == 'sim_operator'
    rows = [json.loads(ln) for ln in audit_file.read_text().splitlines()]
    assert rows[-1]['event'] == 'user_changed' and rows[-1]['detail'] == 'new'
    assert rows[-1]['preset'] == 'sim_operator' and rows[-1]['password'] == 'set'
    assert 'pw-12345' not in audit_file.read_text()


def test_an_admin_may_remove_themselves_and_it_is_audited(remote, audit_file):
    remote.accounts.apply_preset('second', 'admin', password='second-pass')
    me = Client(remote.test_port)
    me.sign_in('second', 'second-pass')
    status, _, _ = me.post('/api/users', {'client': 'c', 'user': 'second', 'remove': True})
    assert status == 200
    last = [json.loads(ln) for ln in audit_file.read_text().splitlines()][-1]
    assert last['who'] == 'second @ 127.0.0.1' and last['remove'] is True


def test_an_observer_reads_the_log_and_audit_but_commands_nothing(remote):
    olly = Client(remote.test_port)
    olly.sign_in('olly', 'olly-pass')
    assert {'log_read', 'audit_read'} <= set(NO_LEASE)
    for action in ('log_read', 'audit_read', 'estop'):
        assert olly.post('/api/command', {'client': 'o', 'action': action})[0] == 200, action
    assert olly.post('/api/command', {'client': 'o', 'action': 'home'})[0] == 403
    # Without the log page, the reads are refused too.
    remote.accounts.set_user('olly', pages=['dashboard'])
    assert olly.post('/api/command', {'client': 'o', 'action': 'audit_read'})[0] == 403


def test_an_admin_may_take_over_and_nobody_else_may(remote, audit_file):
    kit = Client(remote.test_port)
    kit.sign_in('kit', 'kit-pass')
    assert kit.post('/api/control', {'client': 'k', 'op': 'take'})[0] == 200
    other = Client(remote.test_port)
    remote.accounts.apply_preset('pat', 'operator', password='pat-pass')
    other.sign_in('pat', 'pat-pass')
    assert other.post('/api/control', {'client': 'p', 'op': 'take', 'force': True})[0] == 409
    admin = Client(remote.test_port)
    admin.sign_in('admin', 'admin-pass')
    status, payload, _ = admin.post('/api/control', {'client': 'a', 'op': 'take', 'force': True})
    assert status == 200 and payload['control']['you']
    last = [json.loads(ln) for ln in audit_file.read_text().splitlines()][-1]
    assert last['event'] == 'control_taken' and last['who'] == 'admin @ 127.0.0.1'
    assert last['detail'] == 'taken over from kit @ 127.0.0.1'
    assert not kit.post('/api/control', {'client': 'k', 'op': 'heartbeat'})[1]['ok']


def test_audit_read_returns_the_file_newest_last():
    audit.record('estop', 'kit @ 10.0.0.2', 'E-STOP from web remote', target='hardware')
    audit.record('estop_reset', 'admin @ 10.0.0.3', '')
    reply = actions.handle_command(_Studio(), 'audit_read', {'limit': 1}, WHO)
    assert reply['ok'] and [e['event'] for e in reply['entries']] == ['estop_reset']
    assert reply['path'] == audit.path()
    full = actions.handle_command(_Studio(), 'audit_read', {}, WHO)['entries']
    assert full[0]['target'] == 'hardware'
