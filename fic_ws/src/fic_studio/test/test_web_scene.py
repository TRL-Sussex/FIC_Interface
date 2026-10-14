"""The web app's 3D scene: extracted from MuJoCo, and posed the same way MuJoCo does."""
import os
import struct

import pytest

mujoco = pytest.importorskip('mujoco')

from fic_studio.web_remote import scene as sc          # noqa: E402,I100

XML = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'menagerie/model/KinovaGen3_7DOF/xml/scene.xml')
ARM = ['Actuator%d' % (i + 1) for i in range(7)]


@pytest.fixture(scope='module')
def model():
    if not os.path.exists(XML):
        pytest.skip('KinovaGen3_7DOF model not in this workspace')
    return sc.load_model(XML)


def test_scene_has_the_robot_and_its_driven_joints(model):
    s = sc.scene_dict(model, robot='KinovaGen3_7DOF', joint_names=ARM)
    assert s['robot'] == 'KinovaGen3_7DOF'
    assert len(s['bodies']) == model.nbody
    assert [s['joints'][n]['name'] for n in s['drive_joints']] == ARM
    assert all(g['type'] in sc.GEOM_TYPES.values() for g in s['geoms'])
    assert any(g['type'] == 'mesh' and g['mesh'] >= 0 for g in s['geoms'])
    assert all(len(b['quat']) == 4 for b in s['bodies'])


def test_mesh_buffer_layout(model):
    data = sc.mesh_bytes(model, 0)
    magic, fmt, nvert, nface = struct.unpack('<4sIII', data[:16])
    assert magic == b'FICM' and fmt == sc.MESH_FORMAT
    assert nvert == int(model.mesh_vertnum[0]) and nface == int(model.mesh_facenum[0])
    assert len(data) == 16 + nvert * 12 + nface * 12
    with pytest.raises(IndexError):
        sc.mesh_bytes(model, model.nmesh)


def test_bodies_are_posed_exactly_as_mujoco_does(model):
    """The page's maths must match MuJoCo, or the arm on screen is not the arm."""
    s = sc.scene_dict(model, joint_names=ARM)
    data = mujoco.MjData(model)
    angles = [0.3, -0.7, 1.1, -1.9, 0.5, 0.8, -1.2]
    for name, value in zip(ARM, angles):
        data.qpos[model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name)]] = value
    mujoco.mj_forward(model, data)
    values = {n: angles[k] for k, n in enumerate(s['drive_joints'])}
    posed = sc.pose_bodies(s, values)
    for i, (pos, quat) in enumerate(posed):
        assert pos == pytest.approx(list(data.xpos[i]), abs=1e-9), s['bodies'][i]['name']
        got, want = list(quat), list(data.xquat[i])
        if sum(g * w for g, w in zip(got, want)) < 0:
            got = [-v for v in got]                      # q and -q are one rotation
        assert got == pytest.approx(want, abs=1e-9), s['bodies'][i]['name']


def test_zero_pose_matches_too(model):
    s = sc.scene_dict(model, joint_names=ARM)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for i, (pos, _) in enumerate(sc.pose_bodies(s, {})):
        assert pos == pytest.approx(list(data.xpos[i]), abs=1e-9)


def test_gripper_is_found_and_its_open_end_measured(model):
    s = sc.scene_dict(model, joint_names=ARM)
    g = s['gripper']
    assert s['joints'][g['joint']]['name'] == 'left_outer_knuckle'
    assert {s['joints'][c['joint']]['name'] for c in g['coupled']} == {
        'right_outer_knuckle', 'left_inner_knuckle', 'right_inner_knuckle',
        'left_inner_finger', 'right_inner_finger'}
    assert g['closed'] == 0.0 and g['open'] == pytest.approx(0.85)


def test_open_fingers_are_further_apart_than_closed_ones(model):
    """Whatever a model's convention, 'open' must be the wider pose."""
    s = sc.scene_dict(model, joint_names=ARM)
    names = [b['name'] for b in s['bodies']]
    pair = [names.index(s['bodies'][s['joints'][c['joint']]['body']]['name'])
            for c in s['gripper']['coupled']
            if s['joints'][c['joint']]['name'].endswith('inner_finger')]

    def gap(fraction):
        posed = sc.pose_bodies(s, sc.gripper_values(s, fraction))
        a, b = (posed[i][0] for i in pair)
        return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5

    assert gap(1.0) > gap(0.0) + 0.01


def test_gripper_values_cover_every_finger_joint(model):
    s = sc.scene_dict(model, joint_names=ARM)
    values = sc.gripper_values(s, 1.0)
    assert set(values) == {s['gripper']['joint']} | {c['joint'] for c in s['gripper']['coupled']}
    assert sc.gripper_values(s, 0.0)[s['gripper']['joint']] == s['gripper']['closed']
    assert sc.gripper_values(s, 5.0)[s['gripper']['joint']] == s['gripper']['open']   # clamped


def test_reach_is_the_stretched_arm_from_the_shoulder(model):
    s = sc.scene_dict(model, robot='KinovaGen3_7DOF', joint_names=ARM)
    r = sc.reach(s, 'ee_link')
    # Gen3 at zero stands straight up: shoulder pivot 0.285 m, ee_link at 1.19 m.
    assert abs(r['center'][2] - 0.285) < 0.01
    assert abs(r['center'][2] + r['radius'] - 1.19) < 0.01
    assert sc.reach(s, 'no_such_frame') is None
