"""fic_studio.hardware_model: the friction fit, the arm model, writing the robot files."""
import os
import shutil

from fic_studio.hardware_model import config_write, fit
from fic_studio.hardware_model.model import ArmModel, repo_root, robot_cfg
import numpy as np
import pytest
import yaml

REPO = repo_root()


def test_fit_recovers_friction_damping_armature():
    rng = np.random.default_rng(0)
    t = np.arange(0, 16, 0.001)
    qd = 0.2 * np.sin(2 * np.pi * t / 8) + 0.4 * np.sin(2 * np.pi * t / 4) * (t > 8)
    qdd = np.gradient(qd, t)
    r = 0.6 * np.sign(qd) + 15.0 * qd + 0.3 * qdd - 0.2 + rng.normal(0, 0.05, t.size)
    f = fit.fit_joint(qd, qdd, r)
    assert f['friction'] == pytest.approx(0.6, abs=0.03)
    assert f['damping'] == pytest.approx(15.0, rel=0.03)
    assert f['armature'] == pytest.approx(0.3, abs=0.05)
    assert f['offset'] == pytest.approx(-0.2, abs=0.03) and f['r2'] > 0.95
    assert fit.fit_joint(np.zeros(100), np.zeros(100), np.zeros(100)) is None   # never moved


def test_rest_noise_ignores_slow_drift():
    rng = np.random.default_rng(1)
    t = np.arange(0, 4, 0.001)
    tau = np.c_[0.5 * t + rng.normal(0, 0.08, t.size), rng.normal(0, 0.02, t.size)]
    assert fit.rest_noise(tau, 0.001) == pytest.approx([0.08, 0.02], rel=0.1)


def test_block_clips_and_leaves_armature_out_unless_asked():
    fits = [{'friction': -0.1, 'damping': 12.0, 'armature': 0.2, 'offset': 0, 'r2': 1, 'n': 1},
            None]
    b = fit.block(fits, [0.05, 0.01], 200.0, 'test', 2)
    assert b['joint_friction'] == [0.0, 0.0] and b['joint_damping'] == [12.0, 0.0]
    assert 'armature' not in b and b['enabled'] and b['torque_filter_hz'] == 200.0
    assert fit.block(fits, [0.05, 0.01], 200.0, 'test', 2, armature=True)['armature'] == [0.2, 0.0]


def _repo_copy(tmp_path):
    for rel in config_write.FILES:
        os.makedirs(os.path.dirname(tmp_path / rel), exist_ok=True)
        shutil.copy(os.path.join(REPO, rel), tmp_path / rel)
    return str(tmp_path)


def test_write_replaces_kinova_block_and_keeps_the_rest(tmp_path):
    repo = _repo_copy(tmp_path)
    before = open(os.path.join(repo, config_write.FILES[0])).read()
    hm = {'enabled': True, 'source': 'measured: test', 'joint_friction': [0.5] * 7,
          'joint_damping': [9.0] * 7, 'torque_noise': [0.1] * 7, 'torque_filter_hz': 200.0,
          'command_delay_ms': 0.0}
    done = config_write.write(repo, 'KinovaGen3_7DOF', hm)
    assert len(done) == 2
    for p in done:
        d = yaml.safe_load(open(p))['robots']
        assert d['KinovaGen3_7DOF']['hardware_model'] == hm
        assert d['KinovaGen3_7DOF']['admittance_deadbands']['cart_force'] == 4.0
    after = open(os.path.join(repo, config_write.FILES[0])).read()
    # only the block changed: the comments above it and every other robot are kept
    assert after.count('hardware_model:') == before.count('hardware_model:') == 1
    assert '# The real arm as the simulator reproduces it' in after
    assert len(after.splitlines()) == len(before.splitlines())
    old = yaml.safe_load(before)['robots']
    new = yaml.safe_load(after)['robots']
    old.pop('KinovaGen3_7DOF')
    new.pop('KinovaGen3_7DOF')
    assert old == new


def test_write_adds_a_block_to_a_robot_without_one(tmp_path):
    repo = _repo_copy(tmp_path)
    hm = {'enabled': True, 'joint_friction': [0.2] * 7}
    config_write.write(repo, 'Franka_FR3', hm)
    d = yaml.safe_load(open(os.path.join(repo, config_write.FILES[0])))['robots']['Franka_FR3']
    assert d['hardware_model'] == hm and 'admittance_deadbands' in d
    with pytest.raises(KeyError):
        config_write.splice('robots:\n  A:\n    x: 1\n', 'B', hm)


def test_arm_model_gravity_and_inertia():
    """At rest rnea is g(q), about 0 on the vertical first joint; acceleration adds torque."""
    cfg = robot_cfg(REPO, 'KinovaGen3_7DOF')
    am = ArmModel(REPO, cfg)
    q = np.radians(cfg['q_home_deg'])
    g = am.rnea(q, np.zeros(7), np.zeros(7))
    assert abs(g[0]) < 1e-3 and np.abs(g[1:4]).max() > 1.0
    qdd = np.zeros(7)
    qdd[0] = 1.0
    assert am.rnea(q, np.zeros(7), qdd)[0] > 0.05


def test_consistent_flags_disagreeing_or_poor_fits():
    good = {'friction': 0.6, 'damping': 15.0, 'armature': 0.0, 'offset': 0.0, 'r2': 0.98,
            'n': 1000}
    assert fit.consistent(good, [dict(good, friction=0.65), dict(good, damping=14.0)]) == ''
    assert 'disagree' in fit.consistent(good, [dict(good, friction=1.4), good])
    assert 'poor fit' in fit.consistent(dict(good, r2=0.5), [good])
    assert fit.consistent(None, []) == 'too little motion'


def test_even_resampling_of_an_uneven_recording():
    from fic_studio.hardware_model.__main__ import _even
    rng = np.random.default_rng(2)
    t = np.cumsum(rng.uniform(0.0005, 0.002, 4000))
    seg = [(ti, np.array([np.sin(ti)]), np.array([2 * ti])) for ti in t]
    tg, Q, T = _even(seg)
    assert np.allclose(np.diff(tg), 0.001)
    assert np.allclose(Q[:, 0], np.sin(tg), atol=1e-5) and np.allclose(T[:, 0], 2 * tg)
