"""The gravity model must account for the tool the arm actually carries.

The shipped Kinova URDF stops at ``ee_link`` with ``mass="0"`` and has no
Robotiq gripper at all, while the MJCF the twin and the candidate
geometry already load carries both. That makes ``ArmModel.gravity``
wrong by up to 9.4 Nm on J2 and 6.0 Nm on J4 -- far above the 6-sigma
detection thresholds -- and the error is pose-dependent and changes
sign, so a bias calibration cancels it at one pose only. Everywhere
else a multi-Nm offset survives, and a push that opposes it has to
overcome twice the offset while a push along it fires easily. That is
the directional asymmetry observed on hardware: contact detected in one
rotational direction and not the other.

Gravity torque depends only on mass and centre of mass, never on
rotational inertia, so an equivalent point mass is not an approximation
here -- it is exact for g(q).
"""

import os

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from fic_contact.residual import ArmModel, tool_payload_from_mjcf

_HERE = os.path.dirname(__file__)
_URDF = os.path.join(_HERE, '..', '..', 'menagerie', 'model',
                     'KinovaGen3_7DOF', 'URDF', 'KinovaGen3_7DOF.urdf')
_MJCF = os.path.join(_HERE, '..', '..', 'menagerie', 'model',
                     'KinovaGen3_7DOF', 'xml', 'scene.xml')
_JOINTS = ['Actuator%d' % i for i in range(1, 8)]

#: Poses spanning the workspace: the gravity error is pose-dependent, so
#: a single pose could pass by luck at a node of the error.
_POSES = [
    [0.0, 0.26, -3.14, -2.27, 0.0, 1.0, 1.57],
    [0.0, 0.60, 0.0, 0.30, 0.0, 0.30, 0.0],
    [0.0, -0.50, 0.0, -2.40, 0.0, -1.20, 0.0],
    [0.5, 0.90, 1.0, -1.00, 0.5, 0.80, 0.2],
]


def _mujoco_gravity(model, data, q):
    """Return MuJoCo's gravity torque for the arm joints, ROS order."""
    dof = [model.jnt_dofadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in _JOINTS]
    adr = [model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in _JOINTS]
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[adr] = q
    mujoco.mj_forward(model, data)
    return np.asarray(data.qfrc_bias)[dof]


@pytest.fixture(scope='module')
def mj():
    """The MJCF model/data pair, which includes the tool and gripper."""
    model = mujoco.MjModel.from_xml_path(_MJCF)
    return model, mujoco.MjData(model)


def test_payload_recovers_the_mass_the_urdf_omits(mj):
    """ee_link plus the Robotiq bodies, ~1.34 kg, none of it in the URDF."""
    mass, com = tool_payload_from_mjcf(_MJCF, _JOINTS)
    assert 1.0 < mass < 1.8, mass
    assert np.all(np.isfinite(com)) and len(com) == 3


def test_unpatched_model_is_wrong_by_more_than_the_thresholds(mj):
    """The bug, stated as a test: the error dwarfs 6-sigma thresholds."""
    model, data = mj
    arm = ArmModel(_URDF, _JOINTS)
    worst = max(
        np.abs(_mujoco_gravity(model, data, np.array(q)) - arm.gravity(np.array(q))).max()
        for q in _POSES)
    assert worst > 5.0, worst


def test_payload_brings_gravity_into_agreement(mj):
    """With the tool accounted for, both models agree to well under 1 Nm.

    This is the fix's acceptance criterion, and it is deliberately a
    cross-model check rather than a self-consistency one: MuJoCo's model
    is the independent description of the same physical arm, so
    agreement here means the Pinocchio gravity is right in absolute
    terms, not merely internally tidy.
    """
    model, data = mj
    mass, com = tool_payload_from_mjcf(_MJCF, _JOINTS)
    arm = ArmModel(_URDF, _JOINTS, tool_payload=(mass, com))
    worst = max(
        np.abs(_mujoco_gravity(model, data, np.array(q)) - arm.gravity(np.array(q))).max()
        for q in _POSES)
    assert worst < 1.0, 'residual gravity disagreement %.3f Nm' % worst


def test_zero_payload_changes_nothing(mj):
    """An explicit zero payload must be identical to passing none."""
    q = np.array(_POSES[0])
    plain = ArmModel(_URDF, _JOINTS).gravity(q)
    zero = ArmModel(_URDF, _JOINTS,
                    tool_payload=(0.0, [0.0, 0.0, 0.0])).gravity(q)
    assert np.allclose(plain, zero)


def test_payload_is_rejected_when_malformed():
    """A bad payload must raise, not silently skew every residual."""
    with pytest.raises((ValueError, TypeError)):
        ArmModel(_URDF, _JOINTS, tool_payload=(-1.0, [0.0, 0.0, 0.0]))
    with pytest.raises((ValueError, TypeError)):
        ArmModel(_URDF, _JOINTS, tool_payload=(1.0, [0.0, 0.0]))
    with pytest.raises((ValueError, TypeError)):
        ArmModel(_URDF, _JOINTS,
                 tool_payload=(float('nan'), [0.0, 0.0, 0.0]))
