"""Kortex supervisor: its safety gate, the Studio's process manager and summary,
and the web command — none of it needs the arm.

The supervisor against an emulated Gen3 end to end:
robot_library/fic_mcu/scripts/test_kortex_supervisor.sh
"""
import importlib.util
import os
import sys
import time
import types

import pytest

from fic_studio import mcu_probe
from fic_studio.mixins.inner_loop_mixin import InnerLoopMixin

_SUP = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'robot_library', 'fic_mcu', 'scripts',
                    'kortex_supervisor.py')


def _load_supervisor():
    spec = importlib.util.spec_from_file_location('kortex_supervisor', _SUP)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)            # rclpy / kortex_api are imported only in main()
    return mod


ks = _load_supervisor()
RUNNING = 'mcu fic_mcu_ventunoq v1 at 192.168.1.20:7350 | running | state 1000 Hz | faults 0x0'
WAITING = 'mcu available, waiting for Start: fic_mcu_ventunoq v1 at 192.168.1.20:7350'


@pytest.mark.parametrize('text, age, busy', [
    (RUNNING, 0.5, True),
    (RUNNING.replace('running', 'hold'), 0.5, True),
    (RUNNING.replace('running', 'ramp'), 0.5, True),
    (RUNNING, 10.0, False),                 # fic_control gone: stale line
    (WAITING, 0.5, False),
    ('standard: vendor driver via shared memory', 0.5, False),
    ('', 0.0, False),
])
def test_mcu_busy(text, age, busy):
    assert ks.mcu_busy(text, age) is busy


def test_moves_refused_while_anyone_drives_the_arm():
    assert 'MCU has the arm' in ks.why_not_move('ARMSTATE_SERVOING_READY', RUNNING, 0.1)
    assert 'being driven' in ks.why_not_move('ARMSTATE_SERVOING_LOW_LEVEL', WAITING, 0.1)
    assert ks.why_not_move('ARMSTATE_IN_FAULT', WAITING, 0.1) == ''
    assert ks.why_not_move('ARMSTATE_SERVOING_READY', '', 99) == ''


def test_angle_error_wraps_at_360():
    assert ks.angle_err_deg([359.5, 10.0], [0.5, 10.0]) == pytest.approx(1.0)
    assert ks.angle_err_deg([], [1.0]) == 999.0


def test_process_manager_needs_the_api(tmp_path, monkeypatch):
    monkeypatch.setattr(mcu_probe, 'KORTEX_PY', str(tmp_path / 'missing'))
    ok, why = mcu_probe.KortexSupervisorProc(script=_SUP).ensure(1, '127.0.0.1')
    assert not ok and 'kortex_py' in why


def test_process_manager_starts_and_stops(tmp_path, monkeypatch):
    api = tmp_path / 'kortex_py' / 'kortex_api'
    api.mkdir(parents=True)
    monkeypatch.setattr(mcu_probe, 'KORTEX_PY', str(tmp_path / 'kortex_py'))
    script = tmp_path / 'sleeper.py'
    script.write_text('import time\ntime.sleep(30)\n')
    p = mcu_probe.KortexSupervisorProc(script=str(script))
    try:
        assert p.ensure(3, '10.0.0.9') == (True, 'started')
        assert p.ensure(3, '10.0.0.9') == (True, 'running') and p.key == (3, '10.0.0.9')
        assert p.ensure(3, '10.0.0.8') == (True, 'started')      # another arm: restarted
    finally:
        p.stop()
    assert not p.running()


class _Host(InnerLoopMixin):
    def __init__(self):
        self._kx_msg = None
        self._kx_note = ''
        self._kx_pub = None
        self.sent = []

    def log_message(self, *a):
        pass


def test_summary_without_supervisor_explains_why(monkeypatch):
    monkeypatch.setattr(mcu_probe, '_KORTEX', types.SimpleNamespace(running=lambda: False))
    h = _Host()
    h._kx_note = 'arm 192.168.1.10 not on this network (MCU-only: Bridge layout)'
    s = h.kortex_summary()
    assert s['available'] is False and 'Bridge layout' in s['note']
    sent, why = h.kortex_command('home')
    assert not sent and 'Bridge layout' in why


def test_summary_and_command_with_supervisor(monkeypatch):
    monkeypatch.setattr(mcu_probe, '_KORTEX', types.SimpleNamespace(running=lambda: True))
    h = _Host()
    h._kx_msg = ({'reachable': True, 'arm_state': 'ARMSTATE_IN_FAULT', 'servoing': 'SINGLE_LEVEL_SERVOING',
                  'base_faults': [1, 0], 'actuator_faults': [[2, 4, 0]], 'camera': True, 'busy': '',
                  'last': 'Kortex e-stop applied'}, time.monotonic())
    s = h.kortex_summary()
    assert s['available'] and s['reachable'] and s['fault'] and s['state'] == 'IN_FAULT'
    assert s['faults'] == 'base 0x1/0x0, joint 2 0x4/0x0' and s['camera'] is True
    h._kx_pub = types.SimpleNamespace(publish=lambda m: h.sent.append(m.data))
    assert h.kortex_command('clear_faults') == (True, '') and h.sent == ['clear_faults']
    assert h.kortex_command('reboot')[0] is False


def test_stale_supervisor_status_is_not_trusted(monkeypatch):
    monkeypatch.setattr(mcu_probe, '_KORTEX', types.SimpleNamespace(running=lambda: True))
    h = _Host()
    h._kx_msg = ({'reachable': True, 'arm_state': 'ARMSTATE_SERVOING_READY'}, time.monotonic() - 60)
    assert h.kortex_summary()['available'] is False


def test_web_command_needs_confirmation_for_home():
    from fic_studio.web_remote import actions
    w = types.SimpleNamespace()
    with pytest.raises(actions.Refused) as e:
        actions._kortex(w, {'op': 'home'}, None)
    assert e.value.args[1] == 'confirm' if len(e.value.args) > 1 else 'confirmation' in str(e.value)
    with pytest.raises(actions.Refused):
        actions._kortex(w, {'op': 'reboot'}, None)
