"""The Sequences web page, Studio side: web_state(), web_action() and the
files the web interface loads. Real Qt (offscreen) and a fake host."""
import json
import math
import os
import pathlib
import re
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'fic_ws' / 'src' / 'fic_studio')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
QtWidgets = pytest.importorskip('PyQt5.QtWidgets')
if not hasattr(QtWidgets, 'QApplication') or not isinstance(QtWidgets.QApplication, type):
    pytest.skip('PyQt5 is stubbed in this session (Script Editor tests); run this file on its own',
                allow_module_level=True)

from Extensions._shared.fake_host import FakeHost  # noqa: E402
from Extensions.sequences import extension as seq_mod  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture
def ext(tmp_path, monkeypatch):
    monkeypatch.setattr(seq_mod, 'SEQUENCES_YAML_PATH', str(tmp_path / 'robot_sequences.yaml'))
    e = seq_mod.Extension(FakeHost())
    e.build_page()
    return e


JOINT = {'type': 'joint', 'values': [0, 15, 0, -90, 0, 30, 0], 'hold': 0.5}


def test_state_starts_with_the_default_sequence(ext):
    s = ext.web_state()
    assert s['current'] == 'Default Sequence' and s['waypoints'] == []
    assert s['target'] == 'Simulation' and s['hardware'] is False
    assert s['joints'] == 7 and s['speed'] == 1.0
    assert s['joint_tol_deg'] == 3.0 and s['pos_tol_mm'] == 15.0 and s['timeout_s'] == 30.0


def test_typed_waypoints_are_added_updated_and_checked(ext):
    assert ext.web_action('add', {'waypoint': JOINT}) == {'index': 0}
    wp = ext.web_state()['waypoints'][0]
    assert wp['type'] == 'joint' and wp['values'] == [0, 15, 0, -90, 0, 30, 0] and wp['hold'] == 0.5
    cart = {'type': 'cartesian', 'position': [0.4, 0.1, 0.3], 'axis_angle': [1, 0, 0, 180], 'hold': 1}
    ext.web_action('update', {'index': 0, 'waypoint': cart})
    wp = ext.web_state()['waypoints'][0]
    assert wp['type'] == 'cartesian' and wp['position'] == [0.4, 0.1, 0.3] and 'values' not in wp
    ext.web_action('add', {'waypoint': {'type': 'gripper', 'width': 0.04, 'hold': 0}})
    assert ext.web_state()['waypoints'][1]['width'] == 0.04
    for bad in ({'type': 'joint', 'values': [0] * 6}, {'type': 'joint', 'values': [999] + [0] * 6},
                {'type': 'cartesian', 'position': [0, 0, 9], 'axis_angle': [1, 0, 0, 0]},
                {'type': 'cartesian', 'position': [0, 0, 0], 'axis_angle': [0, 0, 0, 90]},
                {'type': 'gripper', 'width': 0.5}, {'type': 'joint', 'values': [0] * 7, 'hold': -1},
                {'type': 'wave'}):
        with pytest.raises(ValueError):
            ext.web_action('add', {'waypoint': bad})
    assert len(ext.web_state()['waypoints']) == 2


def test_settings_set_the_tolerances(ext):
    ext.web_action('settings', {'key': 'joint_tol_deg', 'value': 5})
    ext.web_action('settings', {'key': 'pos_tol_mm', 'value': 8})
    ext.web_action('settings', {'key': 'timeout_s', 'value': 60})
    ext.web_action('settings', {'key': 'speed', 'value': 2})
    assert math.isclose(ext.joint_tolerance_rad, math.radians(5))
    assert math.isclose(ext.position_tolerance_m, 0.008) and ext.timeout_seconds == 60
    s = ext.web_state()
    assert (s['speed'], s['joint_tol_deg'], s['pos_tol_mm']) == (2.0, 5.0, 8.0)
    with pytest.raises(ValueError):
        ext.web_action('settings', {'key': 'speed', 'value': 9})
    with pytest.raises(ValueError):
        ext.web_action('settings', {'key': 'nope', 'value': 1})


def test_step_and_go_send_to_the_simulator(ext):
    ext.web_action('add', {'waypoint': JOINT})
    ext.web_action('add', {'waypoint': dict(JOINT, values=[10] * 7)})
    ext.seq_waypoint_list.setCurrentRow(-1)
    assert ext.web_action('step', {}) == {'index': 0}
    assert ext.web_action('step', {}) == {'index': 1}
    assert ext.web_action('step', {}) == {'index': 0}          # wraps round
    ext.web_action('goto', {'index': 1})
    sent = [c for c in ext.host.node.calls if c[0] == 'sim_joints']
    assert len(sent) == 4 and math.isclose(sent[-1][2][0], math.radians(10))


def test_the_real_robot_needs_its_confirmation(ext):
    ext.web_action('add', {'waypoint': JOINT})
    ext.web_action('target', {'value': 'Robot 1'})
    assert ext.web_state()['hardware'] is True
    for action, args in (('play', {}), ('step', {}), ('goto', {'index': 0})):
        with pytest.raises(ValueError, match='real robot'):
            ext.web_action(action, args)
    assert ext.host.node.real_joint_publishers['robot1'].sent == []
    ext.web_action('goto', {'index': 0, 'hardware_ok': True})
    assert len(ext.host.node.real_joint_publishers['robot1'].sent) == 1
    ext.web_action('play', {'hardware_ok': True, 'loop': True})
    assert ext.web_state()['playing'] is True and ext.web_state()['loop'] is True
    with pytest.raises(ValueError, match='Stop'):
        ext.web_action('add', {'waypoint': JOINT})
    ext.web_action('stop', {})
    assert ext.web_state()['playing'] is False


def test_sequences_new_rename_delete_save_and_load(ext):
    ext.web_action('new', {'name': 'pick'})
    ext.web_action('add', {'waypoint': JOINT})
    ext.web_action('rename', {'name': 'pick2'})
    assert ext.web_state()['current'] == 'pick2'
    ext.web_action('save', {})
    data = json.loads(json.dumps(__import__('yaml').safe_load(open(seq_mod.SEQUENCES_YAML_PATH))))
    assert [q['name'] for q in data['sequences']] == ['Default Sequence', 'pick2']
    ext.web_action('delete_sequence', {})
    assert 'pick2' not in ext.sequences
    ext.web_action('load', {})
    assert ext.sequences['pick2'][0]['values'] == JOINT['values']
    with pytest.raises(ValueError):
        ext.web_action('new', {'name': 'pick2'})


def test_add_sequence_saves_without_dialogs(ext):
    name = ext.add_sequence('taught', [{'type': 'joint', 'values': [1] * 7, 'hold_time': 0.5}])
    assert name == 'taught' and ext.web_state()['current'] == 'taught'
    assert 'taught' in open(seq_mod.SEQUENCES_YAML_PATH).read()
    with pytest.raises(ValueError):
        ext.add_sequence('  ', [])


def test_every_call_in_the_page_is_an_action(ext):
    page = (HERE / 'web' / 'page.js').read_text()
    names = set(re.findall(r"api\.call\('([a-z_]+)'", page))
    assert names, 'no actions found in the page'
    for name in names:
        try:
            ext.web_action(name, {})
        except ValueError as e:
            assert 'No such action' not in str(e), name


def test_manifest_points_the_web_interface_at_the_page():
    web = json.loads((HERE / 'manifest.json').read_text())['web']
    assert (HERE / 'web' / web['page']).is_file() and (HERE / 'web' / web['css']).is_file()


def _run_out(ext, monkeypatch):
    """Play the trajectory to its end with a fake clock."""
    clock = [1000.0]
    monkeypatch.setattr(seq_mod.time, 'monotonic', lambda: clock[0])
    while ext._traj is not None:
        clock[0] += 1.0 / 60.0
        ext._traj_tick()


def test_a_trajectory_streams_from_the_arm_and_ends_where_it_started(ext, monkeypatch):
    from Extensions._shared.fake_host import pose
    ext.host.current_ee_pose = pose(0.45, 0.0, 0.43)
    modes = []
    ext.host.control_mode = lambda: 'Joint Impedance'
    ext.host.set_mode = modes.append
    clock = [1000.0]
    monkeypatch.setattr(seq_mod.time, 'monotonic', lambda: clock[0])
    r = ext.web_action('trajectory', {'kind': 'circle', 'size': 0.05, 'period': 4, 'cycles': 1, 'plane': 'xy'})
    assert r['duration'] == pytest.approx(5.0) and modes == ['Cartesian Planner']
    assert ext.web_state()['trajectory']['running']
    _run_out(ext, monkeypatch)
    sent = [c[2] for c in ext.host.node.calls if c[0] == 'sim_pose']
    assert len(sent) > 250
    first, last = sent[0].position, sent[-1].position
    assert (first.x, first.y, first.z) == pytest.approx((0.45, 0.0, 0.43))
    assert (last.x, last.y, last.z) == pytest.approx((0.45, 0.0, 0.43), abs=1e-9)
    assert min(p.position.x for p in sent) == pytest.approx(0.35, abs=1e-3)     # the circle's far side
    assert not ext.web_state()['trajectory']['running']


def test_a_trajectory_on_the_real_robot_needs_confirming_is_slower_and_never_switches_mode(ext):
    from Extensions._shared.fake_host import pose
    ext.host.current_ee_pose = pose()
    ext.seq_target.addItem('Robot 1')
    ext.seq_target.setCurrentText('Robot 1')
    ext.host.control_mode = lambda: 'Joint Impedance'
    with pytest.raises(ValueError, match='Confirm'):
        ext.web_action('trajectory', {'kind': 'line', 'size': 0.05, 'period': 8, 'cycles': 1, 'plane': 'xy'})
    with pytest.raises(ValueError, match='Too fast'):
        ext.web_action('trajectory', {'kind': 'circle', 'size': 0.1, 'period': 4, 'cycles': 1, 'plane': 'xy', 'hardware_ok': True})
    with pytest.raises(ValueError, match='Cartesian mode'):
        ext.web_action('trajectory', {'kind': 'line', 'size': 0.05, 'period': 8, 'cycles': 1, 'plane': 'xy', 'hardware_ok': True})
    assert ext._traj is None


def test_a_trajectory_saved_as_a_sequence(ext):
    from Extensions._shared.fake_host import pose
    ext.host.current_ee_pose = pose(0.45, 0.0, 0.43)
    r = ext.web_action('save_trajectory', {'name': 'eight', 'kind': 'eight', 'size': 0.08, 'plane': 'xy', 'per_cycle': 12})
    s = ext.web_state()
    assert r['name'] == 'eight' and s['current'] == 'eight' and len(s['waypoints']) == 12
    assert all(w['type'] == 'cartesian' for w in s['waypoints'])


def test_gripper_waypoints_drive_the_simulated_gripper(ext):
    ext._execute_waypoint({'type': 'gripper', 'width': 0.04, 'hold_time': 0.5})
    assert ('gripper', 'robot1', 0.04) in ext.host.node.calls


def test_new_step_and_speed_for_the_quick_actions(ext):
    ext.new_sequence('qa')
    with pytest.raises(ValueError):
        ext.new_sequence('qa')
    ext.web_action('add', {'waypoint': JOINT})
    ext.web_action('add', {'waypoint': JOINT})
    assert ext.step('qa') == 0 and ext.step('qa') == 1 and ext.step('qa') == 0
    ext.set_speed(2.0)
    assert ext.speed() == 2.0
    with pytest.raises(ValueError):
        ext.set_speed(5)


def test_in_simulation_a_waypoint_switches_to_a_mode_that_follows_it(ext, monkeypatch):
    modes, fired = [], []
    ext.host.control_mode = lambda: 'Cartesian Planner'
    ext.host.set_mode = modes.append
    monkeypatch.setattr(seq_mod.QTimer, 'singleShot', staticmethod(lambda ms, fn: fired.append((ms, fn))))
    ext._execute_waypoint({'type': 'joint', 'values': [0, 15, 0, -90, 0, 30, 0], 'hold_time': 0.5})
    assert modes == ['Joint Impedance With Planner'] and fired[0][0] == ext.MODE_SWITCH_MS
    assert not any(c[0] == 'sim_joints' for c in ext.host.node.calls)      # sent after the switch
    fired[0][1]()
    assert any(c[0] == 'sim_joints' for c in ext.host.node.calls)
    ext.host.control_mode = lambda: 'Joint Impedance With Planner'
    ext._execute_waypoint({'type': 'joint', 'values': [0, 15, 0, -90, 0, 30, 0], 'hold_time': 0.5})
    assert modes == ['Joint Impedance With Planner']                        # already following: no switch


def test_a_real_robot_is_never_switched_by_a_waypoint(ext):
    modes = []
    ext.host.control_mode = lambda: 'Cartesian Planner'
    ext.host.set_mode = modes.append
    ext.seq_target.addItem('Robot 1')
    ext.seq_target.setCurrentText('Robot 1')
    assert ext._mode_for('joint') is False and modes == []


def test_a_waypoint_can_carry_its_own_tolerance(ext):
    from Extensions._shared.fake_host import pose
    ext.host.current_ee_pose = pose(0.5, 0.0, 0.43)
    wp = {'type': 'cartesian', 'position': [0.5, 0.0, 0.40], 'axis_angle': [1, 0, 0, 180]}
    assert not ext._check_cartesian_position_reached(dict(wp))                 # 30 mm > 15 mm
    assert ext._check_cartesian_position_reached(dict(wp, tolerance_mm=40))
