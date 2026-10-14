"""Run web-remote work on the Studio's Qt thread.

HTTP requests arrive on server threads, but every Studio widget, publisher
and process handle belongs to the Qt main thread. Each request hands its work
to this object, which runs it there and passes the result back.
"""
import threading
import time

from PyQt5.QtCore import pyqtSignal, QObject, Qt


class MainThreadInvoker(QObject):
    """Queue callables onto the thread this object was created on."""

    _submit = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._submit.connect(self._run, Qt.QueuedConnection)

    def _run(self, job):
        # A command that waited past its deadline is dropped rather than run
        # late: a joint target that arrives seconds after the phone sent it is
        # not what the operator is looking at any more.
        if job['deadline'] is not None and time.monotonic() > job['deadline']:
            job['error'] = TimeoutError('expired before the Studio could run it')
        else:
            try:
                job['result'] = job['fn']()
            except Exception as exc:          # handed back to the caller
                job['error'] = exc
        job['done'].set()

    def call(self, fn, timeout=3.0, expire=True):
        """Run ``fn`` on the Qt thread and return its result.

        Raises TimeoutError if the Studio does not get to it within
        ``timeout`` seconds. With ``expire`` (the default) a job still queued
        at that point is also dropped; pass ``expire=False`` for work that
        must happen however late it is (the E-stop).
        """
        if threading.current_thread() is threading.main_thread():
            return fn()
        job = {
            'fn': fn, 'done': threading.Event(), 'result': None, 'error': None,
            'deadline': time.monotonic() + timeout if expire else None,
        }
        self._submit.emit(job)
        if not job['done'].wait(timeout):
            raise TimeoutError('the Studio did not respond in time')
        if job['error'] is not None:
            raise job['error']
        return job['result']

    def post(self, fn):
        """Queue ``fn`` without waiting for it (logging, notifications)."""
        self._submit.emit({'fn': fn, 'done': threading.Event(), 'result': None,
                           'error': None, 'deadline': None})
