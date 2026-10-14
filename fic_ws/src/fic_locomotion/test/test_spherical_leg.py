# Copyright 2026 FIC Interface contributors.
# SPDX-License-Identifier: MIT

"""
Kinematics for the rigid leg on a 3-DoF spherical hip.

leg_spherical has no knee, so the foot rides a sphere of fixed radius about the
hip centre. These check the maths against the MJCF itself where MuJoCo is
available, and check the properties that hold regardless.
"""

import math
import random

from fic_locomotion import leg_kinematics as lk

import pytest

FEMUR = lk.SPHERICAL_FEMUR
RADIUS = math.sqrt(sum(c * c for c in FEMUR))


def _leg():
    return lk.LegGeometry('fl', (0.0, 0.0, 0.0), 1.0, femur=FEMUR)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def test_the_registry_offers_both_designs():
    """A robot picks its leg design by name rather than by editing the module."""
    assert set(lk.MODELS) == {'spot', 'leg_spherical_quad'}
    assert lk.model_by_name('leg_spherical_quad') is lk.SPHERICAL_QUAD
    assert lk.model_by_name('nonexistent') is lk.SPOT, 'unknown names fall back to Spot'


def test_spot_is_unchanged_by_the_generalisation():
    """The serial model must survive being wrapped in a LegModel."""
    assert lk.SPOT.joint_names == lk.JOINT_NAMES
    assert lk.SPOT.joint_limits == lk.JOINT_LIMITS
    assert lk.forward_kinematics is lk.serial_forward_kinematics
    assert lk.SPOT.joints_per_leg == 3


def test_the_quadruped_declares_twelve_joints():
    """FIC binds actuator k to joint_names[k], so the vector must be complete."""
    q = lk.SPHERICAL_QUAD
    assert len(q.joint_names) == 12
    assert len(q.joint_limits) == 12
    assert q.joint_names[:3] == ('fl_hx', 'fl_hy', 'fl_hz')
    assert q.joint_names[-3:] == ('hr_hx', 'hr_hy', 'hr_hz')


def test_the_foot_always_lands_on_the_sphere():
    """A rigid femur cannot change length, whatever the hip does."""
    leg = _leg()
    rng = random.Random(0)
    for _ in range(200):
        q = [rng.uniform(-1.5, 1.5) for _ in range(3)]
        foot = lk.spherical_forward_kinematics(leg, q)
        assert abs(math.sqrt(sum(c * c for c in foot)) - RADIUS) < 1e-9


def test_inverse_kinematics_round_trips():
    """Aim the leg at a direction, and forward kinematics must return it."""
    leg = _leg()
    rng = random.Random(1)
    for _ in range(200):
        v = [rng.gauss(0, 1) for _ in range(3)]
        n = math.sqrt(sum(c * c for c in v))
        if n < 1e-6:
            continue
        target = tuple(c / n * RADIUS for c in v)
        q = lk.spherical_inverse_kinematics(leg, target)
        assert _dist(lk.spherical_forward_kinematics(leg, q), target) < 1e-9


def test_unreachable_targets_project_rather_than_raise():
    """A 200 Hz gait loop must always get an answer, never an exception."""
    leg = _leg()
    for target in ((0.0, 0.0, -10.0), (5.0, 5.0, -5.0), (0.0, 0.0, -1e-9)):
        q = lk.spherical_inverse_kinematics(leg, target)
        assert all(math.isfinite(a) for a in q)
        foot = lk.spherical_forward_kinematics(leg, q)
        assert abs(math.sqrt(sum(c * c for c in foot)) - RADIUS) < 1e-9


def test_standing_puts_every_foot_under_its_own_hip():
    """The stand pose has to be a pose the robot can actually hold."""
    q = lk.SPHERICAL_QUAD
    for leg in q.legs:
        foot = q.nominal_stance(leg)
        assert abs(foot[0] - leg.hip_offset[0]) < 1e-3, 'foot is not under the hip in x'
        assert foot[2] < leg.hip_offset[2] - 0.35, 'foot is not below the hip'


def test_the_stand_pose_is_inside_the_joint_limits():
    """Commanding a pose the hips cannot reach would clamp into a lean."""
    q = lk.SPHERICAL_QUAD
    full = q.pose(q.stand_q)
    assert full == q.clamp_to_limits(full)
    full_sit = q.pose(q.sit_q)
    assert full_sit == q.clamp_to_limits(full_sit)


def test_matches_the_mjcf_it_was_measured_from():
    """The femur vector and rotation order are claims about the real model."""
    mujoco = pytest.importorskip('mujoco')
    from pathlib import Path
    mjcf = (Path(__file__).resolve().parents[2]
            / 'menagerie' / 'model' / 'leg_spherical' / 'xml' / 'leg_spherical.xml')
    if not mjcf.is_file():
        pytest.skip('leg_spherical MJCF not reachable')
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    ball = (0.18898, 0.11298, 0.38522)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'foot')
    leg = _leg()
    rng = random.Random(2)
    for _ in range(50):
        q = [rng.uniform(-0.9, 0.9) for _ in range(3)]
        data.qpos[:] = 0.0
        data.qpos[3:6] = q
        mujoco.mj_forward(model, data)
        truth = tuple(data.site_xpos[site][i] - ball[i] for i in range(3))
        assert _dist(lk.spherical_forward_kinematics(leg, q), truth) < 1e-6
