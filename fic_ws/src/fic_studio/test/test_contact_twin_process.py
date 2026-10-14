"""Tests for launching and stopping the MuJoCo contact twin viewer."""
import subprocess
import sys
import types

from fic_studio.mixins.process_mixin import ProcessMixin, _PROC_ATTRS


class _Host:
    """Stand-in carrying just what _start/_stop_contact_twin read."""

    def __init__(self, xml='menagerie/model/KinovaGen3_7DOF/xml/scene.xml'):
        self.node = types.SimpleNamespace(station_id=1)
        self.robot_dropdown = types.SimpleNamespace(currentText=lambda: 'Kinova')
        self.robot_configs = {'Kinova': {'mujoco_xml_path': xml}} if xml else {}
        self.logs = []

    def log_message(self, level, msg, source=''):
        """Record what would have gone to the log widget."""
        self.logs.append((level, msg, source))


class _FakePopen:
    """Stand-in for subprocess.Popen that records terminate/wait/kill."""

    def __init__(self):
        self.terminated = False
        self.killed = False

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        """Return immediately, as if the process exited cleanly."""
        return 0

    def kill(self):
        self.killed = True


def test_contact_twin_process_is_registered_for_teardown():
    """_kill_all_processes must reclaim the twin on GUI exit/crash."""
    names = [entry[0] for entry in _PROC_ATTRS]
    running_flags = [entry[1] for entry in _PROC_ATTRS]
    assert 'contact_twin_process' in names
    assert 'contact_twin_running' in running_flags


def test_start_is_a_noop_when_already_running(monkeypatch):
    """A second start while running must not spawn a second process."""
    h = _Host()
    h.contact_twin_running = True
    calls = []
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **kw: calls.append(1))
    ProcessMixin._start_contact_twin(h)
    assert calls == []


def test_start_warns_and_skips_when_no_mujoco_xml_path(monkeypatch):
    """A robot with no MJCF has nothing to render — warn, don't launch."""
    h = _Host(xml=None)
    calls = []
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **kw: calls.append(1))
    ProcessMixin._start_contact_twin(h)
    assert calls == []
    assert h.logs and h.logs[0][0] == 'WARN'
    assert getattr(h, 'contact_twin_running', False) is False


def test_start_uses_mjpython_on_darwin(monkeypatch):
    """macOS needs mjpython so the passive viewer owns the main thread."""
    h = _Host()
    fake = _FakePopen()
    captured = {}

    def _fake_popen(cmd, **kwargs):
        captured['cmd'] = cmd
        captured['kwargs'] = kwargs
        return fake

    monkeypatch.setattr(subprocess, 'Popen', _fake_popen)
    monkeypatch.setattr(sys, 'platform', 'darwin')
    ProcessMixin._start_contact_twin(h)

    shell_cmd = captured['cmd'][2]
    assert 'mjpython -m fic_contact.twin' in shell_cmd
    assert '--robot-id robot1' in shell_cmd
    assert 'KinovaGen3_7DOF' in shell_cmd
    assert captured['kwargs']['preexec_fn'] is not None
    assert h.contact_twin_running is True
    assert h.contact_twin_process is fake
    h._twin_log_fh.close()


def test_start_uses_python3_on_linux(monkeypatch):
    """Linux runs the viewer under plain python3, no main-thread quirk."""
    h = _Host()
    captured = {}

    def _fake_popen(cmd, **kwargs):
        captured['cmd'] = cmd
        return _FakePopen()

    monkeypatch.setattr(subprocess, 'Popen', _fake_popen)
    monkeypatch.setattr(sys, 'platform', 'linux')
    ProcessMixin._start_contact_twin(h)

    assert 'python3 -m fic_contact.twin' in captured['cmd'][2]
    assert h.contact_twin_running is True
    h._twin_log_fh.close()


def test_stop_terminates_the_process_and_clears_state(monkeypatch, tmp_path):
    """Stop tears down the process, the pkill net, and the log handle."""
    h = _Host()
    fake = _FakePopen()
    h.contact_twin_process = fake
    h.contact_twin_running = True
    h._twin_log_fh = open(str(tmp_path / 'twin.log'), 'w')
    pkill_calls = []
    monkeypatch.setattr(subprocess, 'run', lambda *a, **kw: pkill_calls.append(a))

    ProcessMixin._stop_contact_twin(h)

    assert fake.terminated is True
    assert h.contact_twin_process is None
    assert h.contact_twin_running is False
    assert h._twin_log_fh is None
    assert pkill_calls and 'fic_contact.twin' in pkill_calls[0][0]


def test_stop_is_safe_when_nothing_is_running(monkeypatch):
    """Stop must not raise when there is no process or log handle."""
    h = _Host()
    monkeypatch.setattr(subprocess, 'run', lambda *a, **kw: None)
    ProcessMixin._stop_contact_twin(h)
    assert h.contact_twin_running is False
    assert h.contact_twin_process is None


def test_start_passes_the_active_robot_joint_names(monkeypatch):
    """The twin must be told which joints to drive, per robot.

    ``fic_contact.twin`` defaults ``--joints`` to the Kinova
    ``Actuator1..7`` names and raises ValueError on any joint absent from
    the MJCF, so launching a Franka/UR5e/xArm twin without passing that
    robot's own joint_names kills the viewer before it opens.
    """
    h = _Host(xml='menagerie/model/Franka_FR3/xml/scene.xml')
    h.robot_configs['Kinova']['joint_names'] = [
        'fr3_joint%d' % i for i in range(1, 8)]
    captured = {}

    def _fake_popen(cmd, **kw):
        captured['cmd'] = cmd
        return _FakePopen()

    monkeypatch.setattr(subprocess, 'Popen', _fake_popen)
    monkeypatch.setattr(sys, 'platform', 'linux')
    ProcessMixin._start_contact_twin(h)

    assert '--joints fr3_joint1,fr3_joint2,fr3_joint3,fr3_joint4,' \
           'fr3_joint5,fr3_joint6,fr3_joint7' in captured['cmd'][2]
    h._twin_log_fh.close()
