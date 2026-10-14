"""
The gravity-torque vector must match joint_states for a floating base.

Spot has nv=18 (6 base DOF + 12 leg hinges) but only 12 hinge joints appear in
joint_states. The FIC controller copies the first `num_joints` entries of
whatever arrives, so an unsliced vector feeds it base forces as leg torques.
"""
import pytest

mujoco = pytest.importorskip('mujoco')

from fic_locomotion import leg_kinematics as lk  # noqa: E402


def _spot_model():
    from ament_index_python.packages import get_package_share_path
    path = get_package_share_path('menagerie') / 'model/Spot/xml/scene.xml'
    return mujoco.MjModel.from_xml_path(str(path))


def test_spot_has_more_dofs_than_hinges():
    model = _spot_model()
    hinges = [i for i in range(model.njnt)
              if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    assert model.nv == 18
    assert len(hinges) == 12


def test_sliced_gravity_matches_hinge_count_and_order():
    from mujoco_native.simulation_node import NativeMujocoSimulation
    model = _spot_model()
    hinge_dofadr = [int(model.jnt_dofadr[i]) for i in range(model.njnt)
                    if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    full = [float(i) for i in range(model.nv)]
    hinge_joints = [(0, name, 0, adr)
                    for name, adr in zip(lk.JOINT_NAMES, hinge_dofadr)]
    sliced = NativeMujocoSimulation._hinge_gravity_torques(full, hinge_joints)
    assert len(sliced) == 12
    assert sliced == [float(adr) for adr in hinge_dofadr]


def test_fixed_base_robot_is_unchanged():
    # A fixed-base arm has one DOF per hinge, so slicing must be a no-op.
    from mujoco_native.simulation_node import NativeMujocoSimulation
    full = [1.0, 2.0, 3.0]
    hinge_joints = [(0, 'a', 0, 0), (1, 'b', 1, 1), (2, 'c', 2, 2)]
    assert NativeMujocoSimulation._hinge_gravity_torques(full, hinge_joints) == full
