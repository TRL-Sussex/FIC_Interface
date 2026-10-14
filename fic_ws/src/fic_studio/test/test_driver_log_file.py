"""The vendor driver's output is saved to a file as well as the panel.

The panel goes with the Studio, and with it the one line saying why the
driver quit ([SESSION] communication lost, [FAULT] ...). On 14 Sep 2026 a
Gaze run lost its arm that way and only the joint readings falling to zero
said so.
"""
import os
import subprocess
import types

from fic_studio.mixins import process_mixin
from fic_studio.mixins.process_mixin import ProcessMixin


class _Timer:
    """A QTimer stand-in: the pump's drain is not what is tested here."""

    def __init__(self, parent=None):
        """Take the parent, as QTimer does."""
        self.timeout = types.SimpleNamespace(connect=lambda cb: None)

    def setInterval(self, ms):
        """Accept the interval."""

    def start(self):
        """Accept the start."""

    def stop(self):
        """Accept the stop."""


class _Host(ProcessMixin):
    """Just what the log pump reads."""

    def __init__(self, proc):
        """Carry the driver process."""
        self.driver_process = proc
        self.robot_dropdown = types.SimpleNamespace(
            currentText=lambda: 'KinovaGen3_7DOF')
        self.logs = []

    def log_message(self, level, msg, source=''):
        """Record what would go to the panel."""
        self.logs.append((level, msg, source))


def test_the_driver_output_is_saved_to_the_ros_log_dir(
        tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    import PyQt5.QtCore
    monkeypatch.setattr(PyQt5.QtCore, 'QTimer', _Timer)
    proc = subprocess.Popen(
        ['printf', 'Sessions created\n\n\n[SESSION] 51 consecutive Refresh '
         'errors\n[SESSION] 51 consecutive Refresh errors\n[SESSION] 51 '
         'consecutive Refresh errors\nbye\n'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        bufsize=1)
    host = _Host(proc)
    host._start_driver_log_pump()
    host._driver_log_thread.join(timeout=5.0)
    proc.wait(timeout=5.0)
    files = [f for f in os.listdir(tmp_path) if f.startswith(
        'driver_KinovaGen3_7DOF_')]
    assert len(files) == 1
    text = open(tmp_path / files[0]).read()
    lines = [line.split(' ', 1)[1] if line[:2].isdigit() else line
             for line in text.splitlines()]
    assert lines == ['Sessions created',
                     '[SESSION] 51 consecutive Refresh errors',
                     '  (repeated x2)', 'bye', '(driver output ended)']
    assert any('saved to' in msg for _lvl, msg, _src in host.logs)
    # The panel still gets every line.
    queued = []
    while not host._driver_log_queue.empty():
        queued.append(host._driver_log_queue.get())
    assert queued[0] == 'Sessions created' and len(queued) == 5


def test_no_log_file_is_no_reason_not_to_run(tmp_path, monkeypatch):
    blocked = tmp_path / 'not-a-dir'
    blocked.write_text('')
    monkeypatch.setenv('ROS_LOG_DIR', str(blocked))
    assert process_mixin._open_driver_log('Kinova') is None
