"""Playground scenes load in MuJoCo for every usable robot, keep the robot's names, and put the
objects on the table; robots the controller cannot drive are listed with the reason."""
import os

import mujoco
import pytest
import yaml

from Extensions.mujoco_playground import scene_builder as sb

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
WS_SRC = os.path.join(REPO, 'fic_ws', 'src')
CONFIGS = yaml.safe_load(open(os.path.join(REPO, 'fic_library', 'config', 'robots_config.yaml')))
CONFIGS = CONFIGS.get('robots', CONFIGS)


def _usable():
    return [r for r in sb.robots(CONFIGS, WS_SRC) if r['ok']]


def test_the_arms_are_usable_and_the_others_say_why():
    rows = {r['name']: r for r in sb.robots(CONFIGS, WS_SRC)}
    assert rows['KinovaGen3_7DOF']['ok'] and rows['KinovaGen3_7DOF']['gripper']
    assert rows['Franka_FR3']['ok'] and not rows['Franka_FR3']['gripper']
    if 'IIWA_14' in rows:                      # robots the public repository may leave out
        assert not rows['IIWA_14']['ok'] and 'torque motors' in rows['IIWA_14']['reason']
    if 'Spot' in rows:
        assert not rows['Spot']['ok'] and rows['Spot']['reason']


@pytest.mark.parametrize('preset', sorted(sb.PRESETS))
def test_every_preset_loads_for_every_usable_robot(tmp_path, preset):
    _, table, objects = sb.PRESETS[preset]
    for r in _usable():
        path = sb.build(r['scene'], objects, table=table, out_path=str(tmp_path / (r['name'] + '.xml')))
        m = mujoco.MjModel.from_xml_path(path)
        robot = mujoco.MjModel.from_xml_path(r['scene'])
        # the robot is unchanged: its joints and actuators keep their names, in order
        assert [m.actuator(i).name for i in range(robot.nu)] == [robot.actuator(i).name for i in range(robot.nu)]
        assert [m.joint(i).name for i in range(robot.njnt)] == [robot.joint(i).name for i in range(robot.njnt)]
        assert {m.body(i).name for i in range(m.nbody)} >= {sb.OBJECT_PREFIX + o['name'] for o in objects}


def test_objects_rest_on_the_table_and_stay_there(tmp_path):
    _, table, objects = sb.PRESETS['shapes']
    kinova = next(r for r in _usable() if r['name'] == 'KinovaGen3_7DOF')
    m = mujoco.MjModel.from_xml_path(sb.build(kinova['scene'], objects, table=table,
                                              out_path=str(tmp_path / 'k.xml')))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    start = {o['name']: d.xpos[m.body(sb.OBJECT_PREFIX + o['name']).id].copy() for o in objects}
    for o in objects:                                   # bottom on the table top (+1 mm)
        half = sb.SHAPES[o['shape']]['half_height'](o['size'])
        assert start[o['name']][2] - half == pytest.approx(sb.TABLE['height'] + 0.001, abs=1e-6)
    m.opt.gravity[:] = (0, 0, -9.81)
    for _ in range(500):                                # 1 s with the arm limp: nothing falls or sinks
        mujoco.mj_step(m, d)
    for o in objects:
        assert abs(d.xpos[m.body(sb.OBJECT_PREFIX + o['name']).id][2] - start[o['name']][2]) < 0.005


def test_bad_objects_are_refused():
    with pytest.raises(ValueError):
        sb.check_object({'name': 'x', 'shape': 'cone'})
    with pytest.raises(ValueError):
        sb.check_object({'name': 'x', 'shape': 'box', 'size': [1, 1, 1]})
    with pytest.raises(ValueError):
        sb.check_object({'name': '', 'shape': 'box'})
    with pytest.raises(ValueError):
        sb.build('/nonexistent.xml', [{'name': 'a'}, {'name': 'a'}])


def test_the_scene_is_written_outside_the_repository(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    kinova = next(r for r in _usable() if r['name'] == 'KinovaGen3_7DOF')
    path = sb.build(kinova['scene'], [], table=True)
    assert path.startswith(str(tmp_path)) and not path.startswith(REPO)
