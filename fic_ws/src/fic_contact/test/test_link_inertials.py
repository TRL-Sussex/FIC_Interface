"""Rebuilding a deficient URDF's inertials from its MJCF.

The shipped Franka_FR3 URDF contains no ``<inertial>`` elements at all,
so Pinocchio builds a zero-mass model whose gravity torque is identically
zero. Every residual computed from it is meaningless, and
``ArmModel.gravity_is_degenerate`` exists to refuse that case rather than
report confident nonsense.

The MJCF for the same robot carries complete inertials -- 18.3 kg across
eight bodies. Reading them across is the same move that recovered the
Kinova's missing 1.34 kg tool, generalised from the last link to all of
them.

The load-bearing test is the cross-model gravity check: agreement with
MuJoCo proves both that the inertias were combined correctly and that the
two models' link frames coincide, which is the assumption the transfer
rests on and the one most likely to be silently wrong.
"""

import os

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from fic_contact.residual import ArmModel, link_inertials_from_mjcf

_HERE = os.path.dirname(__file__)
_MODEL = os.path.join(_HERE, '..', '..', 'menagerie', 'model')
_FR3_MJCF = os.path.join(_MODEL, 'Franka_FR3', 'xml', 'scene.xml')
_FR3_URDF = os.path.join(_MODEL, 'Franka_FR3', 'URDF', 'Franka_FR3.urdf')
_FR3_JOINTS = ['fr3_joint%d' % i for i in range(1, 8)]

_KIN_MJCF = os.path.join(_MODEL, 'KinovaGen3_7DOF', 'xml', 'scene.xml')
_KIN_URDF = os.path.join(_MODEL, 'KinovaGen3_7DOF', 'URDF',
                         'KinovaGen3_7DOF.urdf')
_KIN_JOINTS = ['Actuator%d' % i for i in range(1, 8)]

_POSES = [
    [0.0, 0.3, 0.0, -1.8, 0.0, 2.0, 0.0],
    [0.4, -0.4, 0.5, -2.3, 0.3, 1.4, 0.6],
    [-0.6, 0.7, -0.4, -1.2, -0.5, 1.8, -0.3],
]


def _mj_gravity(mjcf, joints, q):
    """Return MuJoCo's gravity torque, the independent reference."""
    model = mujoco.MjModel.from_xml_path(mjcf)
    data = mujoco.MjData(model)
    adr = [model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in joints]
    dof = [model.jnt_dofadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in joints]
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[adr] = q
    mujoco.mj_forward(model, data)
    return np.asarray(data.qfrc_bias)[dof]


def test_franka_urdf_alone_is_degenerate():
    """The bug, stated as a test: no inertials means no gravity."""
    arm = ArmModel(_FR3_URDF, _FR3_JOINTS)
    assert arm.gravity_is_degenerate()


def test_inertials_recovered_match_the_mjcf_masses():
    """Each actuated link's mass comes back from the MJCF."""
    got = link_inertials_from_mjcf(_FR3_MJCF, _FR3_JOINTS)
    assert len(got) == 7
    masses = [entry[0] for entry in got]
    assert all(m > 0.5 for m in masses), masses
    # fr3_link1..7 sum to about 15.5 kg; link0 is the fixed base and
    # contributes no gravity torque, so it is correctly excluded.
    assert 14.0 < sum(masses) < 17.0, sum(masses)


def test_franka_gravity_becomes_non_degenerate():
    """With inertials injected the model produces real gravity torque."""
    arm = ArmModel(_FR3_URDF, _FR3_JOINTS,
                   link_inertials=link_inertials_from_mjcf(
                       _FR3_MJCF, _FR3_JOINTS))
    assert not arm.gravity_is_degenerate()
    assert np.abs(arm.gravity(np.asarray(_POSES[0]))).max() > 1.0


def test_franka_gravity_agrees_with_mujoco():
    """The acceptance criterion, and the frame-alignment check.

    Cross-model rather than self-consistent: MuJoCo is an independent
    description of the same arm, so agreement means the reconstructed
    gravity is right in absolute terms and that the two models' link
    frames coincide. A frame mismatch would show up here as a large,
    pose-dependent disagreement.
    """
    arm = ArmModel(_FR3_URDF, _FR3_JOINTS,
                   link_inertials=link_inertials_from_mjcf(
                       _FR3_MJCF, _FR3_JOINTS))
    worst = max(
        np.abs(_mj_gravity(_FR3_MJCF, _FR3_JOINTS, np.asarray(q))
               - arm.gravity(np.asarray(q))).max()
        for q in _POSES)
    assert worst < 1.0, 'gravity disagreement %.3f Nm' % worst


def test_kinova_is_not_made_worse_by_the_same_treatment():
    """A URDF that already has inertials must not be degraded.

    The Kinova's URDF is complete apart from its tool, so rebuilding from
    the MJCF should land in the same place rather than somewhere new.
    Guards against the reconstruction being tuned to the broken case.
    """
    arm = ArmModel(_KIN_URDF, _KIN_JOINTS,
                   link_inertials=link_inertials_from_mjcf(
                       _KIN_MJCF, _KIN_JOINTS))
    worst = max(
        np.abs(_mj_gravity(_KIN_MJCF, _KIN_JOINTS, np.asarray(q))
               - arm.gravity(np.asarray(q))).max()
        for q in _POSES)
    assert worst < 1.0, 'gravity disagreement %.3f Nm' % worst


def test_rejects_a_joint_the_mjcf_does_not_have():
    """Fail loudly rather than silently returning fewer links."""
    with pytest.raises(ValueError):
        link_inertials_from_mjcf(_FR3_MJCF, ['fr3_joint1', 'nope'])
