"""Tests for how the GUI tears down the robot processes it launched.

Regression cover for a hardware incident: the first uncaught exception in a
Qt slot (a dashboard AttributeError) tripped sys.excepthook, which killed the
FIC controller while the Kinova driver was still in torque mode, leaving the
arm on the controller's last torque until the driver was reached.
"""
import atexit
import signal
import sys
import types

from fic_studio.mixins import process_mixin
from fic_studio.mixins.process_mixin import _PROC_ATTRS, ProcessMixin
import pytest


class _Signal:
    """Stand-in for a Qt signal: records connected slots."""

    def __init__(self):
        self.slots = []

    def connect(self, slot):
        self.slots.append(slot)

    def emit(self):
        for slot in self.slots:
            slot()


class _Window:
    """Records every _kill_all_processes() call."""

    def __init__(self):
        self.kills = []

    def _kill_all_processes(self, reason=''):
        self.kills.append(reason)


class _FakeProc:
    """Popen stand-in that is alive until waited on."""

    def __init__(self, pid):
        self.pid = pid
        self._rc = None

    def poll(self):
        return self._rc

    def wait(self, timeout=None):
        self._rc = 0
        return 0


@pytest.fixture
def installed(monkeypatch):
    """install_crash_cleanup() without leaking hooks into the test process."""
    monkeypatch.setattr(atexit, 'register', lambda fn: None)
    monkeypatch.setattr(sys, 'excepthook', sys.excepthook)
    saved = {s: signal.getsignal(s)
             for s in (signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT)}
    window = _Window()
    app = types.SimpleNamespace(aboutToQuit=_Signal())
    process_mixin.install_crash_cleanup(window, app)
    yield window, app
    for s, handler in saved.items():
        signal.signal(s, handler)


def test_driver_is_torn_down_before_its_controller():
    """The driver must leave torque mode while the controller still runs."""
    names = [entry[0] for entry in _PROC_ATTRS]
    assert names.index('driver_process') < names.index('controller_process')


def test_kill_all_signals_driver_before_controller(monkeypatch):
    """_kill_all_processes follows _PROC_ATTRS order, driver first."""
    host = types.SimpleNamespace(
        _PROC_ATTRS=_PROC_ATTRS,
        controller_process=_FakeProc(100), controller_running=True,
        driver_process=_FakeProc(200), driver_running=True,
    )
    signalled = []
    monkeypatch.setattr(process_mixin.os, 'getpgid', lambda pid: pid)
    monkeypatch.setattr(process_mixin.os, 'killpg',
                        lambda pgid, sig: signalled.append(pgid))
    ProcessMixin._kill_all_processes(host, reason='test')
    assert signalled == [200, 100]
    assert host.driver_running is False
    assert host.controller_running is False


def test_slot_exception_does_not_kill_robot_processes(installed, capsys):
    """A non-fatal GUI exception is reported, never turned into a teardown."""
    window, _app = installed
    try:
        raise AttributeError("'tuple' object has no attribute 'force'")
    except AttributeError:
        sys.excepthook(*sys.exc_info())
    assert window.kills == []
    assert "'tuple' object has no attribute 'force'" in capsys.readouterr().err


def test_real_shutdown_still_kills_after_a_slot_exception(installed):
    """A reported exception must not consume the one-shot cleanup latch."""
    window, app = installed
    try:
        raise RuntimeError('non-fatal')
    except RuntimeError:
        sys.excepthook(*sys.exc_info())
    app.aboutToQuit.emit()
    assert window.kills == ['Qt aboutToQuit']
