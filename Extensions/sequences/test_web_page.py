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
