"""Model-level checks of the simulator's helpers (no ROS graph needed)."""
import os
import types

import mujoco
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('communication')

from mujoco_native.simulation_node import (  # noqa: E402
    NativeMujocoSimulation, find_ft_sensors, find_imu_sensors, load_model,
    model_name)

_MENAGERIE = os.path.join(os.path.dirname(__file__), '..', '..', 'menagerie', 'model')
KINOVA = os.path.join(_MENAGERIE, 'KinovaGen3_7DOF', 'xml', 'scene.xml')

_IMU_XML = """
<mujoco model="imu_probe">
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint name="h" axis="0 0 1"/>
      <geom size="0.1"/>
      <site name="s"/>
    </body>
  </worldbody>
  <sensor>
    <accelerometer site="s"/>
    <gyro site="s"/>
    <framequat objtype="site" objname="s"/>
    <force name="r2_force_sensor" site="s"/>
    <torque name="r2_torque_sensor" site="s"/>
  </sensor>
</mujoco>
"""


def _grav(model, data):
    """Run the node's gravity computation on a bare model/data pair."""
    fake = types.SimpleNamespace(_model=model, _data=data, _gdata=mujoco.MjData(model))
    return NativeMujocoSimulation._compute_gravity_torques(fake)


def test_gravity_is_g_of_q_not_the_command():
    m = load_model(KINOVA)
    d = mujoco.MjData(m)
    d.qpos[1] = 0.8           # shoulder pitched, so gravity loads it
    d.qpos[3] = -1.0
    d.qvel[:] = 0.5           # velocity must not leak into g(q)
    d.ctrl[:7] = 30.0         # nor the actuator command (the old echo bug)
    mujoco.mj_forward(m, d)
    before = (d.qvel.copy(), d.qacc.copy(), d.sensordata.copy())
    tau = np.array(_grav(m, d))

    ref = mujoco.MjData(m)
    ref.qpos[:] = d.qpos
    mujoco.mj_forward(m, ref)
    np.testing.assert_allclose(tau, ref.qfrc_bias, atol=1e-9)
    assert abs(tau[1]) > 1.0
    assert abs(tau[0]) < 1e-3          # joint 1's axis is vertical
    # The stepped state is untouched.
    np.testing.assert_array_equal(d.qvel, before[0])
    np.testing.assert_array_equal(d.qacc, before[1])
    np.testing.assert_array_equal(d.sensordata, before[2])


def test_model_name_is_frame_id():
    assert model_name(load_model(KINOVA)) == 'KinovaGen3_7DOF scene'


def test_ft_sensors_exact_then_suffix():
    assert find_ft_sensors(load_model(KINOVA)) == ('force_sensor', 'torque_sensor')
    m = mujoco.MjModel.from_xml_string(_IMU_XML)
    assert find_ft_sensors(m) == ('r2_force_sensor', 'r2_torque_sensor')


def test_imu_sensors_found():
    m = mujoco.MjModel.from_xml_string(_IMU_XML)
    adr = find_imu_sensors(m)
    assert set(adr) == {'acc', 'gyro', 'quat'}
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    # Body on a vertical hinge at rest: the accelerometer reads +g upward.
    np.testing.assert_allclose(d.sensordata[adr['acc']:adr['acc'] + 3], [0, 0, 9.81], atol=1e-6)
    np.testing.assert_allclose(d.sensordata[adr['quat']:adr['quat'] + 4], [1, 0, 0, 0], atol=1e-9)
    assert find_imu_sensors(load_model(KINOVA)) == {}


def test_loads_compiled_binary_model(tmp_path):
    m = load_model(KINOVA)
    path = tmp_path / 'kinova.mjb'
    mujoco.mj_saveModel(m, str(path), None)
    assert load_model(str(path)).nq == m.nq


def _spawn(model, value):
    """Run the node's start-pose step on a bare model with a parameter value."""
    logs = []
    log = types.SimpleNamespace(info=logs.append, warn=logs.append)
    fake = types.SimpleNamespace(
        _model=model, _data=mujoco.MjData(model), _hold_pose=False,
        get_parameter=lambda name: types.SimpleNamespace(value=value),
        get_logger=lambda: log)
    fake._spawn_qpos = fake._data.qpos.copy()
    NativeMujocoSimulation._apply_initial_joint_positions(fake)
    return fake, logs


def test_initial_joint_positions_spawn_and_hold_the_arm():
    m = load_model(KINOVA)
    home = [0.0, 0.2617993877991494, -3.141592653589793, -2.26543736908864,
            0.0, 1.0402162341886205, 1.5707963267948966]
    fake, _ = _spawn(m, ','.join(str(v) for v in home))
    np.testing.assert_allclose(fake._data.qpos[:7], home)
    np.testing.assert_allclose(fake._spawn_qpos[:7], home)
    assert fake._hold_pose


def test_initial_joint_positions_empty_or_bad_keep_qpos0():
    m = load_model(KINOVA)
    for value in ('', 'a,b', ','.join(['0.1'] * (m.njnt + 1))):
        fake, _ = _spawn(m, value)
        assert not fake._hold_pose, value
        np.testing.assert_allclose(fake._data.qpos, m.qpos0)


def test_kinova_joint_damping():
    """The Kinova model runs with joint damping 5; the wall scenes' probe model keeps 45."""
    np.testing.assert_allclose(load_model(KINOVA).dof_damping[:7], 5.0)
    wall = os.path.join(os.path.dirname(KINOVA), 'scene_wall.xml')
    np.testing.assert_allclose(load_model(wall).dof_damping[:4], 45.0)


_ARM_WITH_OBJECTS = """
<mujoco>
  <worldbody>
    <body name="link1"><joint name="j1" type="hinge"/><geom size="0.05" mass="1"/>
      <body name="finger" pos="0 0 0.2"><joint name="knuckle" type="hinge"/><geom size="0.02" mass="0.1"/></body>
    </body>
    <body name="obj_cube" pos="0.5 0 0.1"><freejoint/><geom type="box" size="0.02 0.02 0.02" mass="0.05"/></body>
  </worldbody>
  <actuator>
    <motor name="torq_j1" joint="j1"/>
    <general name="gripper" joint="knuckle" ctrlrange="0 0.85" ctrllimited="true" biastype="affine" biasprm="0 -1 0"/>
  </actuator>
  <custom><numeric name="start_ctrl_gripper" data="2.0"/><numeric name="start_ctrl_nothing" data="1"/></custom>
</mujoco>
"""

_FLOATING = """
<mujoco>
  <worldbody>
    <body name="trunk"><freejoint/><geom size="0.1" mass="5"/>
      <body name="leg"><joint name="hip" type="hinge"/><geom size="0.02" mass="0.5"/></body>
    </body>
    <body name="obj_ball" pos="1 0 0"><freejoint/><geom size="0.03" mass="0.1"/></body>
  </worldbody>
  <actuator><motor name="hip" joint="hip"/></actuator>
</mujoco>
"""


def test_free_objects_do_not_make_an_arm_floating():
    """Only a free joint carrying the actuated joints is the robot's base."""
    from mujoco_native.simulation_node import base_free_joint
    assert base_free_joint(mujoco.MjModel.from_xml_string(_ARM_WITH_OBJECTS)) == -1
    m = mujoco.MjModel.from_xml_string(_FLOATING)
    assert base_free_joint(m) == 0          # the trunk's, not the ball's


def test_a_servo_starts_at_the_setpoint_its_scene_names():
    """start_ctrl_<actuator>, clamped to the ctrlrange; other actuators start at 0."""
    from mujoco_native.simulation_node import start_ctrl
    m = mujoco.MjModel.from_xml_string(_ARM_WITH_OBJECTS)
    assert list(start_ctrl(m)) == [0.0, 0.85]
