# fic_simulation — Simulation Robot Control Node

> **Package:** `fic_simulation`
> **Node name:** `fic_simulation_node_{station_id}` (unique per robot instance)
> **Source:** `src/fic_simulation.cpp`
> **Language:** C++ (templated on `double`), ROS 2 Humble

The `fic_simulation` node runs the FIC (Fractal Impedance Control) library against the **MuJoCo physics simulation**. It receives joint state feedback and gravity torques over ROS 2 topics, computes torque commands through the FIC control pipeline, and publishes them back to the simulator as `ActuatorCmds`.

---

## Package Structure

```
fic_simulation/
├── CMakeLists.txt
├── package.xml
├── include/                          # FIC library headers (shared with fic_hardware)
├── launch/
│   └── fic_launch.py                # Launch file with robot_name + num_joints args
├── scripts/
│   └── plotter_node.py              # Real-time plotting utility
└── src/
    └── fic_simulation.cpp           # Main node implementation
```

### Where It Fits in the Workspace

```
fic_ws/src/
├── communication/          # Custom ROS msgs (ActuatorCmds, Jacobian, etc.)
├── fic_hardware/           # Real robot control (shared memory)
├── fic_simulation/         # ◄ THIS PACKAGE — simulation control
├── fic_studio/             # PyQt5 GUI (control panel, tuning, scripting)
├── menagerie/              # Robot models (URDF + MuJoCo XML)
├── mujoco_native/          # MuJoCo simulator node (CPU, the default)
└── mujoco_newton/          # Optional GPU simulator node (NVIDIA Warp)
```

### Relationship to MuJoCo

`fic_simulation` and the simulator node (`mujoco_native`) are **separate processes** that must both be running. The MuJoCo node handles physics stepping and 3D rendering; this node handles control. They communicate purely over ROS topics:

```
┌────────────────────┐          ROS 2 Topics          ┌────────────────────┐
│                    │                                  │                    │
│  mujoco_native     │  /{prefix}/joint_states ────────▶│  fic_simulation    │
│                    │  /{prefix}/gravity_torques ─────▶│                    │
│  (physics engine   │  /{prefix}/force_torque ────────▶│  (FIC controller)  │
│   + 3D viewer)     │                                  │                    │
│                    │◀── /{prefix}/actuators_cmds ─────│                    │
│                    │                                  │                    │
└────────────────────┘                                  └────────────────────┘
```

---

## Control Modes

Selected with `Int32` messages on `{rid}_control_mode`:

| Mode | Value | FIC Call | Description |
|------|-------|----------|-------------|
| 0 | 0 | Zero torques | Passive (no active control) |
| 1 | 1 | `JC.impedance()` | Pure position stiffness (K0 only, **no velocity damping**) |
| 2 | 2 | `JC.impedanceWithPlanner()` | Impedance + trajectory planner with `damping_ratio` |
| 3 | 3 | `JC.admittance(..., m=1)` | Admittance with InvM, Kv damping, flow/pose limits |
| 4 | 4 | `JC.admittance(..., m=2)` | Haptic-oriented admittance variant |
| 5 | 5 | `RC.impedance()` | Task-space impedance via Jacobian transpose |
| 6 | 6 | `RC.planner()` | Task-space with trajectory planning |
| 7 | 7 | `RC.admittance(..., m=1)` | Task-space admittance |
| 8 | 8 | `RC.admittance(..., m=2)` | Task-space haptic admittance |

The node starts in mode 1 (Joint Impedance). HAP-FIC is switched on with
`{rid}_hapfic_enable` and acts in modes 6-8.

---

## ROS 2 Interface

### Subscribed Topics

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/{prefix}/joint_states` | `sensor_msgs/JointState` | Sensor data | Joint positions, velocities, efforts from MuJoCo |
| `/{prefix}/gravity_torques` | `std_msgs/Float64MultiArray` | Sensor data | Gravity compensation vector from MuJoCo |
| `/{prefix}/contact_torques` | `std_msgs/Float64MultiArray` | Sensor data | Joint torques of the environment on the arm (MuJoCo `qfrc_constraint + qfrc_applied`); HAP-FIC's task-space admittance in modes 7/8 estimates its external wrench from them |
| `/{prefix}/force_torque` | `geometry_msgs/Wrench` | Sensor data | Wrist F/T sensor wrench from MuJoCo; used only with `wrench_from_api` (otherwise the node estimates the external wrench from the joint torques as `pinv(Jᵀ)(τ_meas − g(q))`) |
| `{rid}_control_mode` | `std_msgs/Int32` | Sensor data | Control mode selection (1–8) |
| `command_ee_position_change_{rid}` | `geometry_msgs/Pose` | Sensor data | Cartesian EE target (triggers IK solve) |
| `{rid}_target_joint_angles` | `sensor_msgs/JointState` | Sensor data | Joint-space targets |
| `{rid}_desired_joint_angles` | `std_msgs/Float32MultiArray` | 10 | Joint-space targets (from GUI scripts) |
| `{rid}_selected_robot` | `std_msgs/String` | Sensor data | Robot selection → loads URDF, resets controllers |
| `{rid}_number_of_joints` | `std_msgs/Int32` | Sensor data | Dynamic joint count update |
| `{rid}_gripper_width` | `std_msgs/Float32` | 10 | Gripper aperture command |
| `{rid}_safety/cartesian_limits` | `std_msgs/Float64MultiArray` | 10 | Workspace bounding box |
| `{rid}_safety/joint_limits` | `std_msgs/Float64MultiArray` | 10 | Per-joint position limits |
| `{rid}_safety/velocity_limits` | `std_msgs/Float64MultiArray` | 10 | Max joint & Cartesian velocities |
| `{rid}_active_profile` | `std_msgs/String` | 10 | Tuning profile to use |
| `{rid}_param_update` | `std_msgs/String` | 10 | Re-read the tuning in `config.xml` |
| `{rid}_postural_enable`, `{rid}_elbow_enable` | `std_msgs/Bool` | 10 | Redundancy layers |
| `{rid}_hapfic_enable` | `std_msgs/Bool` | 10 | HAP-FIC on modes 6-8 |
| `{rid}_admittance_planner_enable` | `std_msgs/Bool` | 10 | Planner on the admittance target |
| `{rid}_gravity_compensated`, `{rid}_wrench_gravity_compensated`, `{rid}_wrench_from_api` | `std_msgs/Bool` | 10 | How the measured torque and wrench are read |
| `{rid}_desired_wrench` | `geometry_msgs/Wrench` | 10 | Desired wrench of the Cartesian admittance (modes 7/8, and HAP-FIC on them); zero at start-up |

> **`{rid}`** is the station robot ID, e.g. `robot1`, `robot2`, set by the
> `station_id` parameter. **`{prefix}`** is the `source_prefix` parameter:
> `simulation` by default; the Studio passes `robot<N>_simulation`.

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/{prefix}/actuators_cmds` | `communication/ActuatorCmds` | Torque commands sent to MuJoCo (with velocity damping). Actuator names are `<joint_name_prefix>torq_j1`, `…torq_j2`, etc. |
| `/{prefix}/actuator_torques` | `std_msgs/Float64MultiArray` | The same torques as a plain array |
| `{rid}_trajectory_data` | `std_msgs/Float64MultiArray` | FIC internal state for the plotter and the Studio |
| `/{rid}_fic_torque_commands` | `std_msgs/Float64MultiArray` | Raw FIC torque output (before velocity damping) |
| `{rid}_jacobian` | `communication/Jacobian` | Current end-effector Jacobian |
| `{rid}_ee_current_pose` | `geometry_msgs/Pose` | End-effector pose from forward kinematics |
| `{rid}_admittance_offset` | `std_msgs/Float64MultiArray` | Admittance offset from the target |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `selected_robot` | `string` | `"KinovaGen3_7DOF"` | Robot to load on startup |
| `number_of_joints` | `int` | `7` | Initial joint count |
| `station_id` | `int` | `1` | Station ID (1–4). Determines `robot{N}_*` topic prefix |
| `joint_name_prefix` | `string` | `""` | Prefix for MuJoCo joint/actuator names. Set to `r1_`, `r2_`, etc. in multi-robot scenes |
| `source_prefix` | `string` | `"simulation"` | Namespace of the simulator topics (`/{prefix}/joint_states`, …) |

---

## Torque Output

When the velocity limits are on, the torques sent to MuJoCo include a **velocity damping term** that is not in the raw FIC output:

```
actuator_torque[i] = clamp(fic_torque[i] - kd[i] × joint_velocity[i])
kd[i]              = torque_limit[i] / max_joint_velocity
```

The velocity is a lightly filtered one (see `kDampingCutoffHz` in the header), and the result is clamped to the joint's torque limit. The raw FIC torques are published separately on `/{rid}_fic_torque_commands`.

---

## Signal Filtering

Feedback from MuJoCo is filtered through **second-order Butterworth low-pass filters**:

| Signal | Cutoff Frequency | Sample Rate |
|--------|-----------------|-------------|
| Joint torques (efforts) | 10 Hz | the measured `joint_states` rate |
| Joint positions | 10 Hz | the measured `joint_states` rate |
| Joint velocities | 10 Hz | the measured `joint_states` rate |

---

## Control Loop (1 kHz)

Runs via a 1 ms wall timer. State is snapshot from shared members under a mutex, then processed lock-free.

```
 1. Guard — skip if robot not selected or model not loaded
 2. Snapshot State (under mutex)
      • Copy q_current, q_target, gravity_torques, effort,
        jacobian, ee poses, wrench into local variables
      • External wrench: pinv(Jᵀ)(τ_meas − g(q)) from the effort,
        or the F/T sensor when wrench_from_api is true
 3. Detect Mode Change
      • If mode changed since last tick → set initControllerPLanner = false
 4. Compute EE Velocity
      • ee_velocity = Jacobian × q̇
 5. Initialise Controllers (once per mode change)
      • refreshTuning(): use the fic::TuningSet for this robot/mode,
        read from config.xml on a background thread when the key changes
        or after param_update / a profile change (return until it is ready)
      • Modes 1–4: initJointControllers()
      • Modes 5–8: initCartesianControllers()
 6. Compute Torques
      • Mode 0 → zero torques
      • Modes 1–4 → per-joint JC.impedance / planner / admittance
      • Modes 5–8 → RC.impedance / planner / admittance (with Jacobian)
 7. Publish Results
      • {rid}_trajectory_data
      • /{rid}_fic_torque_commands (raw torques)
      • /{prefix}/actuators_cmds (torques with velocity damping, see Torque Output)
```

**Key difference from `fic_hardware`:** The control loop snapshots all state into local variables first, then releases the mutex before computing. This allows ROS callbacks to update state concurrently without blocking the control computation.

---

## Inverse Kinematics

Iterative **damped Jacobian pseudo-inverse** solver using Pinocchio:

| Parameter | Value |
|-----------|-------|
| Convergence threshold | 1e-6 (SE3 log-map error norm) |
| Max iterations | 5000 |
| Integration step (DT) | 0.01 |
| Method | `Jlog6` local Jacobian, `(JJᵀ)` LDLT solve |

Higher precision than `fic_hardware` (1e-6 vs 1e-4, 5000 vs 1000 iterations) since simulation has no real-time constraint on the IK solve.

---

## Robot Configuration

Same `robots_config.yaml` and `fic::RobotConfigLoader` as `fic_hardware`. Robot models live in the `menagerie` package:

```
menagerie/model/
├── Franka_FR3/     (URDF + MuJoCo xml)
├── KinovaGen3_7DOF/(URDF + MuJoCo xml)
├── IIWA_14/        (URDF + MuJoCo XML)
└── Pendulum/       (URDF + MuJoCo xml)
```

All four robots can be used in simulation (unlike `fic_hardware` which gates on `is_supported`).

---

## Parameter Loading

FIC tuning parameters are loaded from `config.xml` at controller initialisation time:

```
Path: fic_library/config/config.xml
```

**Per-joint parameters** via `fic::ParameterLoader::loadJointParams()`:
- `ScalarParams` — stiffness (K0), diff_effort_max, saturation ratios
- `ListParams` — error_max, force_max bounds
- `PlannerParams` — damping_ratio, planner dynamics
- `AdmittanceParams` — InvM, Kv, dFlow/Flow/Pose limits

**Cartesian parameters** via `fic::ParameterLoader::loadCartesianParams()`:
- Per-axis (X/Y/Z) position stiffness and limits
- Rotation axis parameters
- Elbow controller parameters

---

## Safety System

Same three-layer safety system as `fic_hardware`, receiving limits from `fic_studio`:

- **Cartesian Safety Zones** — Clamps EE target to per-axis bounding box
- **Joint Position Limits** — Clamps joint targets to min/max (radians)
- **Velocity Limits** — Caps joint and Cartesian velocities

See the `fic_hardware` README for message format details.

---

## Custom Message Dependencies

Unlike `fic_hardware` (which uses only standard messages), this node depends on the `communication` package:

| Message | Package | Used For |
|---------|---------|----------|
| `ActuatorCmds` | `communication/msg` | Torque commands to MuJoCo (name + torque per joint) |
| `Jacobian` | `communication/msg` | Publishing the current EE Jacobian |

---

## Build

### Dependencies

| Library | Purpose |
|---------|---------|
| `rclcpp` | ROS 2 C++ client |
| `std_msgs`, `geometry_msgs`, `sensor_msgs` | Standard ROS messages |
| `communication` | Custom messages (ActuatorCmds, Jacobian) |
| `Eigen3` | Linear algebra |
| `pinocchio` | Robot kinematics & dynamics (URDF, FK, IK, Jacobians) |
| `fic_library` | FIC library headers |
| `menagerie` | Robot model files |
| `tinyxml2` | `config.xml` parsing |
| `yaml-cpp` | `robots_config.yaml` parsing |
| `Python3` + `NumPy` | Linked for Pinocchio Python bindings |

### Compile

```bash
cd ~/FIC_Interface/fic_ws
colcon build --symlink-install --packages-select fic_simulation --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Launch

```bash
# Launch FIC controller only (start MuJoCo separately)
ros2 launch fic_simulation fic_launch.py robot_name:=Franka_FR3

# With explicit joint count
ros2 launch fic_simulation fic_launch.py robot_name:=Franka_FR3 num_joints:=7

# Multi-robot: two controllers on different stations with joint name prefixes
ros2 launch fic_simulation fic_launch.py robot_name:=Franka_FR3 station_id:=1 joint_name_prefix:=r1_
ros2 launch fic_simulation fic_launch.py robot_name:=Franka_FR3 station_id:=2 joint_name_prefix:=r2_
```

MuJoCo must be started separately:
```bash
ros2 launch mujoco_native simulation_launch.py robot_name:=Franka_FR3
```

---

## Pre-allocated Loop Variables

The control loop avoids per-tick allocation by pre-sizing vectors at init:

| Variable | Size | Contents |
|----------|------|----------|
| `loop_traj_data_` | `num_joints × 8` | FIC trajectory state (positions, velocities, targets, etc.) |
| `loop_torque_data_` | `num_joints` | Computed torques |
| `loop_current_pose_` | 7 | EE pose [x, y, z, qx, qy, qz, qw] |
| `loop_target_pose_` | 7 | Target EE pose |
| `loop_wrench_data_` | 6 | External EE wrench [Fx, Fy, Fz, Tx, Ty, Tz] (Jacobian estimate, or F/T sensor with `wrench_from_api`) |
| `loop_elbow_current_` | 7 | Current elbow pose |
| `loop_elbow_target_` | 7 | Target elbow pose |

---

## Important Notes

- **Mode 1 has no velocity damping.** Same as `fic_hardware` — the FIC `Controller` class is pure position feedback. To reduce oscillation, decrease K0 stiffness.

- **Gravity torques come from MuJoCo**, not from Pinocchio RNEA. The simulator publishes them on `/{prefix}/gravity_torques`, giving exact model-consistent compensation.

- **Trajectory data is `N×8` floats**, larger than the hardware node's `N×5`. The extra channels carry additional FIC internal state useful for the autotuner.

- **Robot selection triggers full model reload** — new Pinocchio model, resized vectors, fresh filters, controller re-init. Avoid publishing to `selected_robot` during active control.

- **Both `target_joint_angles` (JointState) and `{rid}_desired_joint_angles` (Float32MultiArray) are accepted** as joint targets, for compatibility with both the GUI's tuning panel and script editor.

- **Multi-robot joint matching is name-based.** When `joint_name_prefix` is set, the controller matches joints by name (e.g. `r1_fr3_joint1`) in the shared `/{prefix}/joint_states` message rather than by index. This allows multiple controllers to read from the same joint-state topic without interference.