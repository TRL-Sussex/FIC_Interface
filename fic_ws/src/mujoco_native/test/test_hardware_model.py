"""robots_config.yaml hardware_model: read for a robot (load) and applied to the MuJoCo model (apply)."""
import os

import mujoco
import pytest

from mujoco_native import hardware_model

HERE = os.path.dirname(os.path.abspath(__file__))
KINOVA = os.path.join(HERE, '..', '..', 'menagerie', 'model', 'KinovaGen3_7DOF', 'xml', 'scene.xml')
CFG = os.path.join(HERE, '..', '..', '..', '..', 'fic_library', 'config', 'robots_config.yaml')
NAMES = ['Actuator%d' % k for k in range(1, 8)]


def _yaml(tmp_path, block):
    import yaml
    p = tmp_path / 'robots_config.yaml'
    p.write_text(yaml.safe_dump({'robots': {'R': {'joint_names': NAMES, **block}}}))
    return str(p)


def test_load_absent_disabled_and_switched_off(tmp_path, monkeypatch):
    monkeypatch.delenv('FIC_HARDWARE_MODEL', raising=False)
    assert hardware_model.load('R', _yaml(tmp_path, {})) == {}
    assert hardware_model.load('R', _yaml(tmp_path, {'hardware_model': {'enabled': False, 'joint_damping': [1] * 7}})) == {}
    p = _yaml(tmp_path, {'hardware_model': {'joint_damping': [2.0] * 7}})       # enabled defaults to on
    hm = hardware_model.load('R', p)
    assert hm['joint_damping'] == [2.0] * 7 and hm['joint_names'] == NAMES
    assert hardware_model.load('nobody', p) == {}
    monkeypatch.setenv('FIC_HARDWARE_MODEL', 'off')
    assert hardware_model.load('R', p) == {}


def test_apply_sets_named_joints_only():
    m = mujoco.MjModel.from_xml_path(KINOVA)
    grip_damping = m.dof_damping[7:].copy()
    hm = dict(joint_names=NAMES, joint_friction=[0.1 * k for k in range(1, 8)], joint_damping=[3.0] * 7,
              torque_noise=[0.05, 0, 0.02, 0, 0, 0, 0], command_delay_ms=3.0)
    out = hardware_model.apply(m, hm, 0.001)
    for k, name in enumerate(NAMES):
        dof = m.jnt_dofadr[m.joint(name).id]
        assert m.dof_frictionloss[dof] == pytest.approx(0.1 * (k + 1))
        assert m.dof_damping[dof] == pytest.approx(3.0)
        assert m.dof_armature[dof] == pytest.approx(0.01)                     # no armature list: MJCF's
    assert (m.dof_damping[7:] == grip_damping).all()                           # the gripper untouched
    assert out['applied'] == NAMES and out['delay_steps'] == 3
    assert sorted(out['noise'].values()) == [0.02, 0.05]


def test_apply_empty_changes_nothing():
    m = mujoco.MjModel.from_xml_path(KINOVA)
    before = m.dof_damping.copy()
    assert hardware_model.apply(m, {}, 0.001) == {'noise': {}, 'delay_steps': 0, 'applied': []}
    assert (m.dof_damping == before).all()


def test_kinova_block_in_repository(monkeypatch):
    monkeypatch.delenv('FIC_HARDWARE_MODEL', raising=False)
    hm = hardware_model.load('KinovaGen3_7DOF', CFG)
    assert hm['torque_filter_hz'] == 200.0 and len(hm['joint_friction']) == 7 and len(hm['torque_noise']) == 7
    m = mujoco.MjModel.from_xml_path(KINOVA)
    assert len(hardware_model.apply(m, hm, 0.001)['applied']) == 7
