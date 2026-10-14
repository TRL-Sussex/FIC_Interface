"""The Teach Mode web page, Studio side: web_state(), web_action(), saving to
Sequences and the files the web interface loads. Real Qt (offscreen) and a
fake host."""
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

from Extensions._shared.fake_host import FakeHost, pose  # noqa: E402
from Extensions.sequences import extension as seq_mod  # noqa: E402
from Extensions.teach_mode import extension as teach_mod  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
JOINTS = [0.0, 0.3, 0.0, -1.5, 0.0, 1.2, 0.0]


def _ext(target='simulation'):
    e = teach_mod.Extension(FakeHost(target))
    e.build_page()
    e.host.current_joint_positions = list(JOINTS)
    e.host.current_ee_pose = pose()
    return e


def _modes(e):
    return [m.data for m in e.host.node.pubs['robot1']['mode_select'].sent]


def test_state_shows_the_live_position_in_degrees_and_mm():
    e = _ext()
    s = e.web_state()
    assert s['teaching'] is False and s['mode'] == 'Joint' and s['target'] == 'simulation'
    assert s['robot'] == 'Robot 1 (KinovaGen3_7DOF)'
    assert s['joints_deg'][3] == round(math.degrees(-1.5), 2)
    assert s['cart'][:3] == [400.0, 0.0, 500.0]


def test_the_qt_readout_is_in_degrees():
    e = _ext()
    e._update_position_display(JOINTS, [0.4, 0, 0.5, 0, 0, 0])
    assert e.teach_joint_labels[3][1].text().strip() == f"{math.degrees(-1.5):.2f}°"


def test_start_record_stop():
    e = _ext()
    with pytest.raises(ValueError, match='Start teaching'):
        e.web_action('record', {})
    e.web_action('mode', {'value': 'Cartesian'})
    e.web_action('start', {})
    assert _modes(e) == ['Cartesian Admittance'] and e.web_state()['teaching'] is True
    with pytest.raises(ValueError):
        e.web_action('mode', {'value': 'Joint'})
    e.web_action('record', {})
    e.web_action('gripper', {'width': 0.02})
    e.web_action('record_gripper', {})
    wps = e.web_state()['waypoints']
    assert [w['mode'] for w in wps] == ['Cartesian', 'Gripper']
    assert wps[1]['text'].startswith('Gripper 20 mm')
    assert e.waypoints_table.rowCount() == 2
    e.web_action('stop', {})
    assert _modes(e)[-1] == 'Cartesian Planner' and e.web_state()['teaching'] is False


def test_recording_needs_robot_feedback():
    e = _ext()
    e.host.current_joint_positions = []
    e.host.current_ee_pose = None
    e.web_action('start', {})
    with pytest.raises(ValueError, match='feedback'):
        e.web_action('record', {})


def test_go_uses_the_simulator_or_the_real_robot():
    e = _ext()
    e.web_action('start', {})
    e.web_action('record', {})
    e.web_action('go', {'index': 0})
    assert e.host.node.calls[-1][0] == 'sim_joints' and e.host.node.calls[-1][2] == JOINTS
    e.host.target = 'hardware'
    with pytest.raises(ValueError, match='real robot'):
        e.web_action('go', {'index': 0})
    e.web_action('go', {'index': 0, 'hardware_ok': True})
    assert e.host.node.calls[-1][0] == 'hw_joints'


def test_starting_on_hardware_and_the_gripper_need_confirmation():
    e = _ext('hardware')
    with pytest.raises(ValueError, match='real robot'):
        e.web_action('start', {})
    assert _modes(e) == []
    e.web_action('start', {'hardware_ok': True})
    assert _modes(e) == ['Joint Admittance']
    with pytest.raises(ValueError, match='real robot'):
        e.web_action('gripper', {'width': 0.03, 'send': True})
    e.web_action('gripper', {'width': 0.03, 'send': True, 'hardware_ok': True})
    assert e.host.node.calls[-1] == ('gripper', 'robot1', 0.03)
    with pytest.raises(ValueError):
        e.web_action('gripper', {'width': 0.5})


def test_delete_keeps_each_rows_buttons_on_its_waypoint():
    e = _ext()
    e.web_action('start', {})
    for x in (0.1, 0.2, 0.3):
        e.host.current_ee_pose = pose(x=x)
        e.web_action('record', {})
    e.web_action('delete', {'index': 0})
    assert [w['index'] for w in e.recorded_waypoints] == [1, 2]
    assert e.waypoints_table.rowCount() == 2 and e.waypoints_table.item(0, 0).text() == '1'
    e.web_action('undo', {})
    assert len(e.recorded_waypoints) == 1
    e.web_action('clear', {})
    assert e.recorded_waypoints == [] and e.waypoints_table.rowCount() == 0


def test_auto_record_settings():
    e = _ext()
    e.web_action('auto', {'interval': 2.5, 'threshold': 12, 'enabled': True})
    assert e.auto_record_interval == 2.5 and e.movement_threshold_mm == 12 and e.auto_record_enabled
    assert e.auto_record_spin.value() == 2.5
    with pytest.raises(ValueError):
        e.web_action('auto', {'interval': 99})


def test_save_as_sequence_goes_into_sequences(tmp_path, monkeypatch):
    monkeypatch.setattr(seq_mod, 'SEQUENCES_YAML_PATH', str(tmp_path / 'robot_sequences.yaml'))
    e = _ext()
    with pytest.raises(ValueError, match='Sequences'):
        e.web_action('start', {}) or e.web_action('record', {}) or e.web_action('save_sequence', {'name': 'x'})
    seq = seq_mod.Extension(e.host)
    seq.build_page()
    e.host.extensions['sequences'] = seq
    assert e.web_state()['sequences'] is True
    e.web_action('mode', {'value': 'Joint'}) if not e.teach_mode_active else None
    r = e.web_action('save_sequence', {'name': 'taught1'})
    assert r == {'name': 'taught1', 'count': 1}
    steps = seq.sequences['taught1']
    assert steps[0]['type'] == 'joint' and math.isclose(steps[0]['values'][3], math.degrees(-1.5))
    assert 'taught1' in open(seq_mod.SEQUENCES_YAML_PATH).read()


def test_every_call_in_the_page_is_an_action():
    e = _ext()
    page = (HERE / 'web' / 'page.js').read_text()
    names = set(re.findall(r"api\.call\('([a-z_]+)'", page))
    assert names
    for name in names:
        try:
            e.web_action(name, {})
        except ValueError as err:
            assert 'No such action' not in str(err), name


def test_manifest_points_the_web_interface_at_the_page():
    web = json.loads((HERE / 'manifest.json').read_text())['web']
    assert (HERE / 'web' / web['page']).is_file() and (HERE / 'web' / web['css']).is_file()
