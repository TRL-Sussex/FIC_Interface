"""The latched E-STOP, the notice history, the audit log and the hardware confirmations of the
web UI. No ROS, no Qt window."""
import json
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest  # noqa: E402

from fic_studio import audit, notices  # noqa: E402
from fic_studio.mixins.safety_mixin import SafetyMixin  # noqa: E402
from fic_studio.web_remote import actions  # noqa: E402
from fic_studio.web_remote.actions import build_snapshot, handle_command  # noqa: E402
from test_web_remote_actions import ROBOT, WHO, _Studio  # noqa: E402


class _Latching(_Studio):
    """The actions' stand-in Studio, with the real latch and an E-STOP that sets it."""
    _estop_latch_set = SafetyMixin._estop_latch_set
    _estop_latch_reset = SafetyMixin._estop_latch_reset
    estop_latch_state = SafetyMixin.estop_latch_state

    def _emergency_stop(self):
        super()._emergency_stop()
        self._estop_latch_set(getattr(self, '_estop_source', '') or 'Studio window')


@pytest.fixture(autouse=True)
def audit_file(tmp_path, monkeypatch):
    monkeypatch.setenv('FIC_STUDIO_AUDIT', str(tmp_path / 'audit.log'))
    audit.reset_for_tests()
    yield tmp_path / 'audit.log'
    audit.reset_for_tests()


def run(studio, action, **args):
    args.setdefault('robot', ROBOT)
    return handle_command(studio, action, args, WHO)


def entries(path):
    return [json.loads(ln) for ln in path.read_text().splitlines()] if path.exists() else []


def test_estop_latches_until_reset_and_says_who(audit_file):
    w = _Latching()
    assert run(w, 'estop')['ok']
    w._web_estop_time -= actions.ESTOP_HOLDOFF_S + 1          # long after: still latched
    latch = build_snapshot(w)['estop']
    assert latch['latched'] and latch['by'] == 'Phone @ 10.0.0.9'
    for action, args in (('send_joints', {'degrees': [0] * 7}), ('session', {'op': 'start', 'confirm': True}),
                         ('home', {}), ('process', {'id': 'simulation', 'op': 'start', 'confirm': True})):
        assert run(w, action, **args)['error'] == 'estop_latched', action
    assert run(w, 'session', op='stop', confirm=True)['ok']   # stopping is never refused
    assert run(w, 'estop_reset')['error'] == 'confirm'
    assert run(w, 'estop_reset', confirm=True)['ok']
    assert not build_snapshot(w)['estop']['latched']
    assert run(w, 'send_joints', degrees=[0] * 7)['ok']
    assert [e['event'] for e in entries(audit_file)] == ['estop', 'estop_reset']
    assert entries(audit_file)[1]['who'] == 'Phone @ 10.0.0.9'


def test_the_account_name_is_who_did_it(audit_file):
    w = _Latching()
    handle_command(w, 'estop', {}, dict(WHO, account='adnan'))
    assert w.estop_latch_state()['by'] == 'adnan @ 10.0.0.9'
    assert entries(audit_file)[0]['who'] == 'adnan @ 10.0.0.9'


def test_home_on_hardware_is_confirmed_and_live_publish_goes_off(audit_file):
    w = _Latching()
    assert run(w, 'home')['ok']                               # simulation: it just goes

    class _Box:
        on = True

        def isChecked(self):
            return self.on

        def setChecked(self, v):
            self.on = v
    w._rc_live_publish = _Box()
    assert run(w, 'set_target', target='hardware')['ok']
    assert w._rc_live_publish.on is False
    assert run(w, 'home')['error'] == 'confirm'
    assert run(w, 'home', confirm=True)['ok']
    assert [e['event'] for e in entries(audit_file)] == ['target']


def test_notices_are_kept_until_clear_all():
    w = _Latching()
    w._web_ui_window = object()                               # the web UI: no Qt box
    for k in range(7):
        notices.alert(w, 'error' if k % 2 else 'warn', 'Simulation failed to start', 'line %d' % k)
    snap = build_snapshot(w)['notices']
    assert snap['count'] == 7 and snap['errors'] == 3 and snap['warnings'] == 4
    assert len(snap['recent']) == notices.RECENT and snap['recent'][-1]['text'] == 'line 6'
    full = run(w, 'notices_list')['items']
    assert [n['text'] for n in full] == ['line %d' % k for k in range(7)]
    assert any('Simulation failed to start' in m for _, m, _ in w.logs)
    assert run(w, 'notices_clear')['cleared'] == 7
    assert build_snapshot(w)['notices']['count'] == 0 and run(w, 'notices_list')['items'] == []


def test_a_long_log_tail_is_shortened():
    w = _Latching()
    w._web_ui_window = object()
    n = notices.alert(w, 'error', 'Driver failed', 'x' * 10000)
    assert len(n['text']) < 4100


def test_server_lets_anyone_read_notices_without_control():
    from fic_studio.web_remote import server
    assert 'notices_list' in server.NO_LEASE and 'estop' in server.NO_LEASE
    assert 'notices_clear' not in server.NO_LEASE
