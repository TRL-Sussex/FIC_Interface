"""Running web-remote work on the Qt thread, and the mixin's E-stop rule."""
import os
import threading
import time
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.web_remote_mixin import WebRemoteMixin  # noqa: E402
from fic_studio.web_remote.invoker import MainThreadInvoker  # noqa: E402
from fic_studio.web_remote.lease import ControlLease  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

_app = QApplication.instance() or QApplication([])


def _pump(until, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not until() and time.monotonic() < deadline:
        _app.processEvents()
        time.sleep(0.005)


def test_call_from_a_worker_runs_on_the_main_thread():
    invoker = MainThreadInvoker()
    result = {}

    def worker():
        result['value'] = invoker.call(
            lambda: threading.current_thread() is threading.main_thread())

    t = threading.Thread(target=worker)
    t.start()
    _pump(lambda: 'value' in result)
    t.join(1)
    assert result['value'] is True


def test_errors_come_back_to_the_caller():
    invoker = MainThreadInvoker()
    result = {}

    def worker():
        try:
            invoker.call(lambda: 1 / 0)
        except ZeroDivisionError:
            result['raised'] = True

    t = threading.Thread(target=worker)
    t.start()
    _pump(lambda: 'raised' in result)
    t.join(1)
    assert result.get('raised')


def test_a_job_that_waited_too_long_is_dropped_not_run_late():
    invoker = MainThreadInvoker()
    ran = []
    outcome = {}

    def worker():
        try:
            invoker.call(lambda: ran.append(1), timeout=0.05)
        except TimeoutError:
            outcome['timeout'] = True

    t = threading.Thread(target=worker)
    t.start()
    t.join(1)                       # the main thread is busy: nothing pumps
    time.sleep(0.1)
    _pump(lambda: False, timeout=0.2)
    assert outcome.get('timeout')
    assert ran == []


def test_estop_keeps_control_with_its_holder():
    host = types.SimpleNamespace()
    host._web_lease = ControlLease()
    host._web_lease.take('phone')
    seen = {}

    class _Invoker:
        """Runs the job inline, recording how it was asked to."""

        def call(self, fn, timeout=3.0, expire=True):
            seen['expire'] = expire
            seen['holder_when_run'] = host._web_lease.holder()
            return fn()

    host._web_invoker = _Invoker()
    host._emergency_stop = lambda: seen.setdefault('stopped', True)
    host.log_message = lambda *a, **k: None
    host.robot_dropdown = types.SimpleNamespace(currentText=lambda: 'X')
    reply = WebRemoteMixin._web_dispatch(host, 'estop', {}, {'client': 'other'})
    assert reply == {'ok': True}
    assert seen['stopped'] and seen['holder_when_run']['client'] == 'phone'
    assert host._web_lease.holds('phone')         # no Take control needed after Reset
    assert seen['expire'] is False                # an E-stop is never dropped
