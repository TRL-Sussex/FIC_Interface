"""The MuJoCo Playground web page, Studio side: web_state(), web_action(), and Start / Stop going
through the Studio's own simulator and controller. A fake host records what the Studio is asked."""
import os
import pathlib
import sys

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'fic_ws' / 'src' / 'fic_studio')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
QtWidgets = pytest.importorskip('PyQt5.QtWidgets')
if not hasattr(QtWidgets, 'QApplication') or not isinstance(QtWidgets.QApplication, type):
    pytest.skip('PyQt5 is stubbed in this session; run this file on its own', allow_module_level=True)

from Extensions.mujoco_playground import extension as pg  # noqa: E402

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
CONFIGS = yaml.safe_load(open(ROOT / 'fic_library' / 'config' / 'robots_config.yaml'))
CONFIGS = CONFIGS.get('robots', CONFIGS)


class Host:
    def __init__(self):
        self.calls, self.settings, self.logs = [], {}, []
        self.robot_configs = CONFIGS
        self.robot = 'KinovaGen3_7DOF'
        self.simulation_running = self.fic_running = self.controller_running = False
        self.targets = {'robot1': 'simulation'}
        self.scenes = {}
        self.node = type('N', (), {'station_id': 1, 'create_subscription': lambda *a: object(),
                                   'destroy_subscription': lambda *a: None})()

    def get_setting(self, k, d=None):
        return self.settings.get(k, d)

    def set_setting(self, k, v):
        self.settings[k] = v

    def log(self, level, msg, src=''):
        self.logs.append((level, msg))

    def current_robot_name(self):
        return self.robot

    def select_robot(self, name):
        self.calls.append(('select', name))
        self.robot = name

    def robot_target(self, rid=None):
        return self.targets.get(rid or 'robot1', 'simulation')

    def set_robot_target(self, rid, t):
        self.calls.append(('target', rid, t))
        self.targets[rid] = t

    def set_simulation_scene(self, path, name=None):
        self.calls.append(('scene', name, bool(path)))
        if path:
            self.scenes[name] = path
        else:
            self.scenes.pop(name, None)

    def simulation_scene(self, name=None):
        return self.scenes.get(name or self.robot)

    def start_simulation(self):
        self.calls.append(('sim', 'start'))
        self.simulation_running = True

    def stop_simulation(self):
        self.calls.append(('sim', 'stop'))
        self.simulation_running = False

    def start_fic_simulation(self):
        self.calls.append(('fic', 'start'))
        self.fic_running = True

    def stop_fic_simulation(self):
        self.calls.append(('fic', 'stop'))
        self.fic_running = False

    quick = {}
    seq = None
    mode = 'Joint Impedance'

    def add_quick_action(self, key, label, run, choices=None, hint='', button=''):
        Host.quick[key] = (run, choices)

    def remove_quick_action(self, key):
        Host.quick.pop(key, None)

    def control_mode(self):
        return self.mode

    def set_mode(self, mode):
        self.calls.append(('mode', mode))
        self.mode = mode

    @property
    def main_window(self):
        return type('MW', (), {'get_extension': lambda _s, name: self.seq if name == 'sequences' else None})()


class Seq:
    def __init__(self, hardware=False):
        self.hw, self.added, self.played = hardware, {}, []

    def target_is_hardware(self):
        return self.hw

    def add_sequence(self, name, wps):
        self.added[name] = wps
        return name

    def play(self, name, loop=False):
        self.played.append(name)


@pytest.fixture
def ext(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    fired = []
    monkeypatch.setattr('PyQt5.QtCore.QTimer.singleShot', lambda ms, fn: fired.append(fn))
    e = pg.Extension(Host())
    e._fired = fired
    return e


def test_state_lists_the_robots_and_the_default_scene(ext):
    s = ext.web_state()
    names = {r['name']: r for r in s['robots']}
    assert names['KinovaGen3_7DOF']['ok'] and not names['IIWA_14']['ok']
    assert s['robot'] == 'KinovaGen3_7DOF' and s['gripper'] and s['table']
    assert [o['name'] for o in s['objects']] == ['red_cube', 'blue_cube'] and not s['running']


def test_objects_are_added_edited_and_removed(ext):
    ext.web_action('add_object', {'object': {'name': 'Ball 1', 'shape': 'sphere', 'size': [0.03], 'x': 0.4, 'y': 0.1, 'color': 'yellow'}})
    assert ext.web_state()['objects'][-1]['name'] == 'ball_1'
    with pytest.raises(ValueError):
        ext.web_action('add_object', {'object': {'name': 'ball_1', 'shape': 'box'}})
    ext.web_action('update_object', {'index': 2, 'object': {'name': 'ball_1', 'shape': 'sphere', 'size': [0.04], 'x': 0.4, 'y': 0.1, 'color': 'green'}})
    assert ext.web_state()['objects'][2]['color'] == 'green'
    ext.web_action('remove_object', {'index': 2})
    ext.web_action('preset', {'key': 'shapes'})
    assert [o['shape'] for o in ext.web_state()['objects']] == ['box', 'sphere', 'cylinder']
    assert ext.host.settings['scene']['preset'] == 'shapes'


def test_start_runs_the_studio_simulator_with_the_scene_then_the_controller(ext):
    ext.web_action('start', {})
    h = ext.host
    assert h.calls[:4] == [('fic', 'stop'), ('sim', 'stop'), ('scene', 'KinovaGen3_7DOF', True), ('sim', 'start')]
    scene = h.scenes['KinovaGen3_7DOF']
    assert scene.endswith('kinovagen3_7dof_playground.xml') and os.path.isfile(scene)
    assert ext.web_state()['starting'] and ext.web_state()['running']
    with pytest.raises(ValueError):
        ext.web_action('preset', {'key': 'stack'})          # no scene changes while it runs
    ext._fired.pop()()                                        # the delayed controller start
    assert h.calls[-1] == ('fic', 'start') and not ext.web_state()['starting']
    ext.web_action('stop', {})
    assert h.calls[-3:] == [('fic', 'stop'), ('sim', 'stop'), ('scene', 'KinovaGen3_7DOF', False)]
    assert not ext.web_state()['running'] and h.simulation_scene('KinovaGen3_7DOF') is None


def test_start_selects_the_robot_and_never_touches_a_running_real_robot(ext):
    h = ext.host
    ext.web_action('robot', {'name': 'Franka_FR3'})
    h.targets['robot1'], h.controller_running = 'hardware', True
    with pytest.raises(ValueError, match='real robot'):
        ext.web_action('start', {})
    assert not any(c[0] in ('sim', 'scene', 'target') for c in h.calls)
    h.controller_running = False
    ext.web_action('start', {})
    assert ('select', 'Franka_FR3') in h.calls and ('target', 'robot1', 'simulation') in h.calls
    with pytest.raises(ValueError):
        ext.web_action('robot', {'name': 'IIWA_14'})


def test_the_web_files_are_there():
    here = pathlib.Path(__file__).resolve().parent
    assert (here / 'web' / 'page.js').is_file() and (here / 'web' / 'page.css').is_file()
    import json
    assert json.load(open(here / 'manifest.json'))['web']['page'] == 'page.js'


def test_pick_builds_the_sequence_and_plays_it(ext):
    h = ext.host
    run, choices = Host.quick['pick']
    assert choices() == []                                     # nothing to pick before Start
    with pytest.raises(ValueError, match='Start'):
        run('red_cube')
    ext.web_action('start', {})
    ext._poses = {'table': [0.55, 0, 0.1, 1, 0, 0, 0], 'red_cube': [0.5, -0.15, 0.222, 1, 0, 0, 0]}
    assert choices() == ['red_cube'] and ext.web_state()['can_pick']
    with pytest.raises(ValueError, match='Sequences'):
        run('red_cube')
    h.seq = Seq()
    msg = run('red_cube')
    assert 'pick_red_cube' in msg and h.seq.played == ['pick_red_cube']
    wps = h.seq.added['pick_red_cube']
    assert [w['type'] for w in wps] == ['joint', 'gripper', 'cartesian', 'cartesian', 'gripper', 'cartesian']
    with pytest.raises(ValueError, match='No object'):
        ext.web_action('pick', {'name': 'table'})
    h.seq = Seq(hardware=True)
    with pytest.raises(ValueError, match='Simulation'):
        ext.web_action('pick', {'name': 'red_cube'})


def test_a_robot_without_a_gripper_offers_no_pick(ext):
    ext.web_action('robot', {'name': 'Franka_FR3'})
    assert not ext.web_state()['can_pick']
