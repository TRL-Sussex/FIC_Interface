# mujoco_native — the FIC Studio MuJoCo simulator (CPU)

The ROS 2 simulator the Studio's **▶ Start** runs in simulation, written in Python on
the official [`mujoco`](https://pypi.org/project/mujoco/) wheel. It runs on
Linux x86-64 and arm64 and on macOS (Apple Silicon), headless or with the
MuJoCo viewer.

`mujoco_newton` is a separate, optional GPU backend with the same topic
contract (Settings → Simulation → Backend).

## Topic contract

Under `robot{station_id}_simulation/`:

| Topic | Type | Rate | Content |
|---|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | 1 kHz | every hinge joint (position, velocity, `qfrc_actuator`); a ball joint as `<joint>_roll/_pitch/_yaw` |
| `gravity_torques` | `std_msgs/Float64MultiArray` | 1 kHz | g(q), one entry per hinge joint (`qfrc_bias` at zero velocity) |
| `contact_torques` | `std_msgs/Float64MultiArray` | 1 kHz | joint torques of the environment on the arm, one entry per hinge (`qfrc_constraint + qfrc_applied`: contacts, joint limits, applied wrenches); `fic_control` estimates H-PMP's external wrench in modes 7/8 from it |
| `imu_data` | `sensor_msgs/Imu` | 500 Hz | the MJCF's accelerometer / gyro / framequat sensors (defaults if none) |
| `force_torque` | `geometry_msgs/Wrench` | 200 Hz | `force_sensor` + `torque_sensor`, or any sensors whose names end in those (multi-robot scenes prefix them) |
| `odom` | `nav_msgs/Odometry` | 50 Hz | base pose/twist of a floating base (zeros for a fixed base) |
| `body_poses`, `wall_force`, `applied_wrench_truth` | — | 200 Hz / per step | test hooks (`experiment_hooks.py`) |
| `actuators_cmds` (sub) | `communication/ActuatorCmds` | — | commands, merged per actuator name across publishers |
| `wall_cmd`, `apply_wrench` (sub) | — | — | test hooks |
| `sim_reset` (srv) | `communication/SimulationReset` | — | reset to a base pose (floating base) and named joint positions; `header.frame_id` must be the model name |

Sensor topics are best effort, depth 1; `actuators_cmds` is best effort, depth
20 (several publishers can share it). `header.frame_id` is the MJCF model name.
Rates are capped by the physics rate: a model with a 2 ms timestep streams its
state at 500 Hz.

## Behaviour

- **Physics** steps at the MJCF `<option timestep>` (1 ms for the arms),
  paced to wall clock against an absolute anchor, with catch-up steps after a
  slow iteration and a re-sync after a stall of more than 0.1 s.
- **Control law**, per commanded actuator: PD+feedforward if the MJCF defines
  `<name>_pos` and `<name>_vel` sensors, else the feedforward `torque`, written
  to `data.ctrl` and clamped to ±100. A torque/motor command not refreshed for
  0.2 s is zeroed, so a dead controller leaves the arm limp. Actuators with a
  bias term (position servos such as the Robotiq `gripper`) keep their last
  setpoint, since the gripper bridge publishes only on change.
- **Gravity torques** come from a forward pass on a scratch `MjData` at the
  current `qpos` with zero velocity, so they are g(q) — never an echo of the
  command — and computing them never disturbs the stepped state.
- **Floating base** (any model with a free joint): starts at keyframe 0 and holds
  the pose until the first actuator command arrives.
- **Start pose of a fixed-base arm**: `initial_joint_positions:=q1,q2,...`
  (radians, the first hinge joints in model order) spawns the arm there and
  holds it until the first actuator command; empty (the default) keeps every
  joint at 0. FIC Studio passes the robot's home pose when its config sets
  `sim_start_at_home: true`.
- **Viewer**: `mujoco.viewer.launch_passive`, redrawn at 60 Hz. Space pauses
  and resumes, Backspace resets to the spawn state. On macOS it needs
  `mjpython` (the launch file uses it). `headless:=true` disables it.
- **Divergence**: with the MJCF's auto-reset disabled, a diverged state pauses
  the simulation; otherwise MuJoCo resets it.
- **Models**: `.xml`/`.urdf` via `MjModel.from_xml_path`, compiled `.mjb` via
  `from_binary_path`. The wheel's bundled plugins (elasticity/cable, sensor,
  actuator, SDF) are registered on import.

## Parameters

| Parameter | Default | |
|---|---|---|
| `simulation/model_file` | — | model path (required) |
| `topic_prefix` | `simulation` | the launch file passes `robot{N}_simulation` |
| `headless` | `false` | no viewer |
| `sim_dt` | `0` | physics step; `0` = the MJCF timestep |
| `publish_rate_hz` | `1000` | joint_states + gravity_torques |
| `imu_rate_hz`, `ft_rate_hz`, `odom_rate_hz`, `body_poses_rate_hz` | 500, 200, 50, 200 | |
| `hardware_model` | `''` | the robot's `robots_config.yaml` `hardware_model` as JSON (the launch file passes it) |

The launch file exposes `robot_name`, `station_id`, `model_file`, `headless`,
`sim_dt` and `publish_rate_hz`.

**Simulating like the real robot.** When the robot named by `robot_name` has an enabled
`hardware_model` block in `robots_config.yaml` ($FIC_ROOT/fic_library/config, then
../fic_library/config from the working directory, then fic_control's share), the launch file
passes it to the node, which sets the friction (`frictionloss`), damping and armature of the
robot's joints, adds the torque-sensor noise to `joint_states`' effort and applies the torque
command `command_delay_ms` late (`mujoco_native/hardware_model.py`). `FIC_HARDWARE_MODEL=off`
in the environment skips it. See fic_docs: Simulation vs. Hardware.

## Build

Requires the `mujoco` Python wheel (`pip install mujoco`; the Docker image pins
3.10.0).

```bash
cd fic_ws
colcon build --packages-select mujoco_native
source install/setup.bash
```

## Run

```bash
ros2 launch mujoco_native simulation_launch.py robot_name:=KinovaGen3_7DOF station_id:=1
# add headless:=true for no viewer; model_file:=/abs/scene.xml for any other robot
ros2 topic hz   /robot1_simulation/joint_states
ros2 topic echo /robot1_simulation/gravity_torques
```

Full stack — start the FIC controller and Studio, then press **SIM** + **FIC**:

```bash
ros2 launch fic_control sim_control_launch.py robot_name:=KinovaGen3_7DOF \
    num_joints:=7 station_id:=1 source_prefix:=robot1_simulation
ros2 run fic_studio fic_studio_pro
```

## Tests

```bash
python3 -m pytest fic_ws/src/mujoco_native/test
```
