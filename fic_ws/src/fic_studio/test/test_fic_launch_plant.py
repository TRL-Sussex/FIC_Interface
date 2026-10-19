"""The FIC controller's launch names the plant the running simulator needs: MuJoCo steps the inner loop in
lockstep (the default), Newton publishes topics the node relays (plant:=topics)."""
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from fic_studio.mixins import process_mixin  # noqa: E402

_app = QApplication.instance() or QApplication([])


def _launch_cmd(monkeypatch, sim_pkg):
    cmds = []
    monkeypatch.setattr(process_mixin.subprocess, 'Popen', lambda args, **kw: cmds.append(args[-1]))
    monkeypatch.setattr(process_mixin, '_apply_btn_style', lambda *a, **k: None)
    host = types.SimpleNamespace(
        robot_dropdown=types.SimpleNamespace(currentText=lambda: 'Pendulum'), robots={'Pendulum': 1},
        node=types.SimpleNamespace(station_id=1, station_robot_id='robot1'),
        robot_target=lambda rid: 'simulation', _vr_drives_station=lambda sid: False, _sim_env=lambda: None,
        log_message=lambda *a: None, fic_toggle_btn=None, _sim_backend_pkg=sim_pkg)
    process_mixin.ProcessMixin._start_fic_simulation(host)
    assert len(cmds) == 1
    return cmds[0]


def test_newton_gets_the_topic_plant(monkeypatch):
    assert 'plant:=topics' in _launch_cmd(monkeypatch, 'mujoco_newton')


def test_mujoco_keeps_lockstep(monkeypatch):
    assert 'plant:=' not in _launch_cmd(monkeypatch, 'mujoco_native')


def _controller_cmd(monkeypatch, mcu_info):
    cmds = []
    monkeypatch.setattr(process_mixin.subprocess, 'Popen', lambda args, **kw: cmds.append(args[-1]))
    monkeypatch.setattr(process_mixin, '_apply_btn_style', lambda *a, **k: None)
    monkeypatch.setattr(process_mixin.QTimer, 'singleShot', lambda *a, **k: None)
    host = types.SimpleNamespace(
        robot_dropdown=types.SimpleNamespace(currentText=lambda: 'KinovaGen3_7DOF'),
        node=types.SimpleNamespace(station_id=1), _mcu_info=mcu_info, _mcu_driver=bool(mcu_info),
        log_message=lambda *a: None, controller_toggle_btn=None,
        _mode_republish_when_ready=lambda: None)
    process_mixin.ProcessMixin._start_controller(host)
    assert len(cmds) == 1
    return cmds[0]


def test_controller_on_an_mcu_starts_no_pc_inner_loop(monkeypatch):
    """The MCU is the robot's inner loop: the PC's fic_inner_loop would only idle beside it (and log)."""
    assert 'inner_loop:=false' in _controller_cmd(
        monkeypatch, {'use': True, 'address': '127.0.0.1', 'via': 'bridge'})


def test_controller_without_an_mcu_starts_the_pc_inner_loop(monkeypatch):
    assert 'inner_loop:=' not in _controller_cmd(monkeypatch, {'use': False, 'address': ''})


def test_a_studio_with_no_display_starts_the_simulator_headless():
    """The simulator's viewer needs a display: on a machine with none (the MCU board, a robot PC's headless
    Studio) it died at start (GLFW: DISPLAY missing), so the Studio asks for no viewer there."""
    assert process_mixin.sim_headless({'QT_QPA_PLATFORM': 'offscreen', 'DISPLAY': ':0'})
    assert process_mixin.sim_headless({})                                   # no display at all
    assert not process_mixin.sim_headless({'DISPLAY': ':0'})
    assert not process_mixin.sim_headless({'WAYLAND_DISPLAY': 'wayland-0'})
