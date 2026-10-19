# fic_control — Hardware Robot Control Node

> **Package:** `fic_control`
> **Node name:** `fic_hardware_node_<station_id>`, e.g. `fic_hardware_node_2`.
> Per station, so that two arms on one ROS domain do not put two identically
> named nodes on one graph. Topics are `robot<N>_*` (written `{rid}_*` below).
> **Source:** `src/fic_control.cpp`
> **Language:** C++ (templated on `double`), ROS 2 Humble

The `fic_control` node runs the FIC (Fractal Impedance Control) library against **real robot hardware**, communicating with vendor drivers through **POSIX shared memory** for lock-free torque-level control at 1 kHz.

---

## Package Structure

```
fic_control/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── robot_control_launch.py      # Launch file with robot_name argument
├── scripts/
│   └── plotter_node.py              # Real-time plotting utility
└── src/
    └── fic_control.cpp             # Main node implementation
```

### Where It Fits in the Workspace

```
fic_ws/src/
├── communication/          # Custom ROS msgs/srvs (Jacobian, ActuatorCmds, etc.)
├── fic_control/           # ◄ THIS PACKAGE — the controller node + fic_inner_loop (real and simulated robots)
├── fic_studio/             # PyQt5 GUI (control panel, tuning, scripting)
├── menagerie/              # Robot models (URDF + MuJoCo XML)
├── mujoco_native/          # MuJoCo simulator node (CPU, the default)
└── mujoco_newton/          # Optional GPU simulator node (NVIDIA Warp)
```

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                     fic_control  (fic_hardware_node)               │
│                                                                      │
│  ┌───────────────┐    ┌────────────────┐    ┌────────────────────┐  │
│  │  Pinocchio     │    │  FIC Library    │    │  Butterworth       │  │
│  │  URDF → FK     │───▶│  8 control     │───▶│  Filters           │  │
│  │  Jacobian      │    │  modes         │    │  (torque/pos/vel)  │  │
│  │  RNEA gravity  │    │                │    │                    │  │
│  └───────┬───────┘    └───────┬────────┘    └────────────────────┘  │
│          │                    │                                      │
│          ▼                    ▼                                      │
│  ┌───────────────────────────────────────────┐                      │
│  │   POSIX Shared Memory  (lock-free atomic)  │                      │
│  │                                             │                      │
│  │   /robot_feedback_memory                    │                      │
│  │     ← joint_positions[7], torque[7]         │                      │
│  │     ← tool_wrench[6]                        │                      │
│  │     → commanded_torque[7]                   │                      │
│  │     → commanded_gripper_width               │                      │
│  │                                             │                      │
│  │   /desired_joint_angles_memory              │                      │
│  │     ← desired_joint_angles[7]               │                      │
│  └───────────────────────────────────────────┘                      │
│                    ▲               │                                  │
└────────────────────┼───────────────┼──────────────────────────────────┘
                     │               ▼
              ┌──────┴───────────────────────┐
              │    Vendor Robot Driver        │
              │    (Franka libfranka /        │
              │     Kinova Kortex API)        │
              │    Reads/writes shared mem    │
              │    at ~1 kHz                  │
              └──────────────────────────────┘
```

---

## Shared Memory Interface

Defined in `<fic/shared_params.h>`. Two POSIX shared memory segments provide lock-free communication with vendor driver processes.

> **Joint-count sizing.** The shared-memory structs reserve `float[kMaxJoints]` (default `kMaxJoints = 16`) for `torque`, `joint_positions`, `commanded_torque`, and `desired_joint_angles`. The active count is published at runtime in `n_joints`, written by `fic_control` once the URDF is parsed. Readers (driver bridges, plotting nodes) can use `n_joints` to size their own loops instead of hardcoding. The on-disk layout is versioned (`version` + `magic` fields at the top of each struct); upgrading past a layout change is automatic — `open_or_create` detects a size/version mismatch and recreates the segment. If you still see stale-segment issues after an upgrade, stop every process that mapped the old segment and `rm /dev/shm/robot_feedback_memory /dev/shm/desired_joint_angles_memory`.
>
> The vendor driver bridges (`cartesian_impedance_control.cpp` for Franka, `00-Torque_Controller.cpp` for Kinova) still carry their own `kNumJoints` constant — Franka FR3 and Kinova Gen3 are both physically 7-DOF, so the driver-side loops stay at 7 regardless of what `kMaxJoints` is set to.

### `/robot_feedback_memory` — `SharedFeedback`

| Field | Type | Direction | Description |
|-------|------|-----------|-------------|
| `alive` | `atomic<uint32_t>` | Node → Driver | 1 while the torque writer runs. `fic_control` clears it on a clean shutdown; the Kinova driver leaves torque mode when it drops |
| `torque[7]` | `atomic<float>` | Driver → Node | Measured joint torques |
| `joint_positions[7]` | `atomic<float>` | Driver → Node | Measured joint positions (rad) |
| `commanded_torque[7]` | `atomic<float>` | Node → Driver | FIC-computed torque commands |
| `tool_wrench[6]` | `atomic<float>` | Driver → Node | EE forces/torques {Fx, Fy, Fz, Tx, Ty, Tz} |
| `commanded_gripper_width` | `atomic<float>` | Node → Driver | Desired gripper aperture (meters) |

### `/desired_joint_angles_memory` — `SharedDesiredJointState`

| Field | Type | Direction | Description |
|-------|------|-----------|-------------|
| `desired_joint_angles[7]` | `atomic<float>` | GUI → Driver | Target joint angles |

**Initialisation details:**
- `commanded_torque` starts as `NaN` so drivers can detect "no commands yet"
- `commanded_gripper_width` defaults to `0.08 m` (fully open) to prevent gripper snap on startup
- All atomics use `memory_order_acquire` for reads, `memory_order_release` for writes
- Reserved slots per joint array: **`kMaxJoints = 16`**; the runtime `n_joints` field (published by `fic_control` on URDF load) tells readers how many of those slots are meaningful. Slots past `n_joints` are kept zeroed.

---

## Control Modes

Selected with `String` messages on `{rid}_mode_select`:

| Mode | String Message | FIC Call | Description |
|------|---------------|----------|-------------|
| 0 | `"Stop Controller"`, `"Pause Controller"` | Zero torques | Passive (no active control) |
| 1 | `"Joint Impedance"` | `JC.impedance()` | Pure position stiffness (K0 only, **no velocity damping**) |
| 2 | `"Joint Impedance With Planner"` | `JC.impedanceWithPlanner()` | Impedance + trajectory planner with `damping_ratio` |
| 3 | `"Joint Admittance"` | `JC.admittance(..., m=1)` | Admittance with InvM, Kv damping, flow/pose limits |
| 4 | `"Joint Admittance Haptic"` | `JC.admittance(..., m=2)` | Haptic-oriented admittance variant |
| 5 | `"Cartesian Impedance"` | `RC.impedance()` | Task-space impedance via Jacobian transpose |
| 6 | `"Cartesian Planner"` | `RC.planner()` | Task-space with trajectory planning |
| 7 | `"Cartesian Admittance"` | `RC.admittance(..., m=1)` | Task-space admittance |
| 8 | `"Cartesian Admittance Haptic"` | `RC.admittance(..., m=2)` | Task-space haptic admittance |
| — | `"Start"` | N/A | **Launches the vendor robot driver** in a new terminal |

H-PMP is not a mode string: switch it on with `{rid}_hpmp_enable` (`Bool`).
It acts in modes 6-8 and reads the `H-PMP` block (id 100) of `config.xml`.
Modes 5-8 are refused when the robot has no kinematics solver.

**Mode switch:** on any mode message the joint and Cartesian targets snap to the current pose so the robot holds position — but only once real joint feedback has arrived. Before that, `q_current_` is the zero fill from `initSharedMemory()`, so the startup target (`q_home`) is kept instead. The Studio re-sends the mode ~5 s after launching this node, which used to land exactly in that window while the driver was still homing.

**Driver launch:** The `"Start"` command reads `driver_command` from the robot's YAML config, replaces the `{robot_lib}` placeholder with the resolved path to `robot_library/`, and executes it via `gnome-terminal`.

---

## ROS 2 Interface

### Subscribed topics

`{rid}` is `robot<station_id>`.

| Topic | Message type | Description |
|-------|-------------|-------------|
| `{rid}_mode_select` | `std_msgs/String` | Control mode (see the table above) |
| `{rid}_desired_joint_angles` | `std_msgs/Float32MultiArray` | Target joint angles (radians) |
| `command_ee_position_change_{rid}` | `geometry_msgs/Pose` | Cartesian end-effector target (solved by IK) |
| `{rid}_selected_robot` | `std_msgs/String` | Robot selection: loads the URDF, resets the controllers |
| `{rid}_number_of_joints` | `std_msgs/Int32` | Joint count (logged only) |
| `{rid}_active_profile` | `std_msgs/String` | Tuning profile to use |
| `{rid}_param_update` | `std_msgs/String` | Re-read the tuning in `config.xml` |
| `{rid}_gripper_width` | `std_msgs/Float32` | Gripper aperture (metres) |
| `{rid}_desired_wrench` | `geometry_msgs/Wrench` | Desired wrench of the Cartesian admittance (modes 7/8, and H-PMP on them); zero at start-up |
| `{rid}_safety/cartesian_limits` | `std_msgs/Float64MultiArray` | Workspace box per axis |
| `{rid}_safety/joint_limits` | `std_msgs/Float64MultiArray` | Per-joint position limits (radians) |
| `{rid}_safety/velocity_limits` | `std_msgs/Float64MultiArray` | Maximum joint and Cartesian velocities |
| `{rid}_postural_enable`, `{rid}_elbow_enable` | `std_msgs/Bool` | Redundancy layers |
| `{rid}_hpmp_enable` | `std_msgs/Bool` | H-PMP on modes 6-8 |
| `{rid}_admittance_planner_enable` | `std_msgs/Bool` | Planner on the admittance target |
| `{rid}_gravity_compensated`, `{rid}_wrench_gravity_compensated`, `{rid}_wrench_from_api` | `std_msgs/Bool` | How the measured torque and wrench are read |
| `{rid}_wrench_external` | `geometry_msgs/Wrench` | A teleoperation partner's contact, reflected onto this arm |

### Published topics

| Topic | Message type | Description |
|-------|-------------|-------------|
| `{rid}_trajectory_data` | `std_msgs/Float64MultiArray` | FIC internal state for the plotter and the Studio |
| `{rid}_q_current`, `{rid}_q_current_raw` | `std_msgs/Float64MultiArray` | Joint positions, filtered and raw |
| `{rid}_q_target`, `{rid}_q_target_active` | `std_msgs/Float64MultiArray` | Joint target, and the one the controller is tracking |
| `{rid}_ee_current_pose` | `geometry_msgs/Pose` | End-effector pose from forward kinematics |
| `{rid}_force_torque` | `geometry_msgs/Wrench` | Tool wrench from the driver |
| `{rid}_joint_velocities`, `{rid}_joint_torques` | `std_msgs/Float64MultiArray` | Measured joint velocities and torques |
| `{rid}_admittance_offset` | `std_msgs/Float64MultiArray` | Admittance offset from the target |
| `{rid}_gripper_state` | `std_msgs/Float64MultiArray` | Gripper state from the driver |
| `{rid}_inner_loop` | `std_msgs/String` | Which inner loop drives the arm (standard driver or MCU) |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `station_id` | `int` | `1` | Station number: sets `{rid}` and the node name |
| `selected_robot` | `string` | `"Default_Robot"` | Robot to load on startup |
| `mcu_address` | `string` | `192.168.1.20` (or `$FIC_MCU_ADDRESS`) | Address of the inner-loop MCU, if any |

---

## Safety System

Three independent safety layers, all configurable at runtime via ROS topics published by `fic_studio`'s Safety panel.

### Cartesian Safety Zones

Clamps `ee_target` position to a per-axis bounding box before IK solving. Each axis has independent min, max, and enable flag.

**Message format:** `[x_min, x_max, x_enabled, y_min, y_max, y_enabled, z_min, z_max, z_enabled]`

**Defaults:** X ∈ [−1.0, 1.0], Y ∈ [−1.0, 1.0], Z ∈ [0.0, 1.5], all enabled.

### Joint Position Limits

Clamps joint targets to per-joint min/max bounds (radians). Applied to both desired joint angles and IK solutions.

**Message format:** `[j1_min, j1_max, j2_min, j2_max, ..., enabled_flag]`

### Velocity Limits

Caps maximum joint velocity (rad/s) and Cartesian velocity (m/s).

**Message format:** `[max_joint_vel, max_cart_vel, enabled_flag]`

**Defaults:** Joint = 1.0 rad/s, Cartesian = 0.5 m/s.

The utility `isPositionInSafetyZone(x, y, z)` checks whether a point lies within the active Cartesian envelope.

---

## Signal Filtering

All shared-memory feedback passes through **second-order Butterworth low-pass filters** before entering the control loop:

| Signal | Cutoff Frequency | Sample Rate |
|--------|-----------------|-------------|
| Joint torques | 200 Hz | 1000 Hz |
| Joint positions | 10 Hz | 1000 Hz |
| Joint velocities | 10 Hz | 1000 Hz |

Filters are destroyed and recreated whenever the joint count changes (robot switch).

---

## Control Loop (1 kHz)

Runs via a 1 ms wall timer. The entire loop executes under a single mutex lock.

```
 1. Guard — skip if robot not supported or model not loaded
 2. readSharedFeedback()
      • Load raw positions & torques from shared memory
      • NaN-guard each value
      • Filter through Butterworth filters
      • Publish filtered q_current
 3. Forward Kinematics
      • Pinocchio FK + frame placements → ee_current pose
      • Publish ee_current_pose
 4. Compute Jacobians
      • 6×N EE Jacobian (LOCAL_WORLD_ALIGNED)
      • 6×N Elbow Jacobian (if frame exists)
 5. Gravity Compensation
      • Pinocchio RNEA → gravity_torques_
 6. Read Wrench
      • wrench_from_api: tool_wrench[6] from shared memory → wrench_data
      • otherwise: pinv(Jᵀ)(τ_meas − g(q)) from the joint torques → wrench_data
 7. Initialise Controllers (once per mode/target change)
      • Load parameters from config.xml via fic::ParameterLoader
      • Modes 1–4: initJointControllers()
      • Modes 5–8: initCartesianControllers()
 8. Compute Torques
      • Mode 0 → zero torques
      • Modes 1–4 → per-joint JC.impedance / planner / admittance
      • Modes 5–8 → RC.impedance / planner / admittance
 9. Write Torques to Shared Memory
      • commanded_torque[0..6] with memory_order_release + fence
10. Publish /trajectory_data
```

The `initControllerPLanner` flag is reset to `false` on a mode change, a robot selection, a H-PMP toggle (a repeat of the current value is ignored), a new joint target on `{rid}_desired_joint_angles` in mode 2 or, outside mode 1, while the admittance planner is on, and when a new tuning set is installed; a new EE target does not reset it. Since v4.4.0 clearing it only re-seeds the controllers from the tuning already in memory. `config.xml` is read by `refreshTuning()` into a `fic::TuningSet` on a background thread (`std::async`), only when the `fic::TuningKey` (robot, joint count, joint mode, Cartesian mode, postural block) changes or after `{rid}_param_update` / a new `{rid}_active_profile`; the loop sends nothing until the set for the current key is in memory, and on the MCU path the joint parameters are sent to the MCU when a set is installed.

---

## Inverse Kinematics

Iterative **damped Jacobian pseudo-inverse** solver using Pinocchio:

| Parameter | Value |
|-----------|-------|
| Convergence threshold | 1e-4 (SE3 log-map error norm) |
| Max iterations | 1000 |
| Integration step (DT) | 0.01 |
| Damping factor | 1e-6 (Levenberg-Marquardt regularisation) |
| Method | `Jlog6` local Jacobian, `(JJᵀ + λI)` LDLT solve |

IK never runs on the control thread: `targetPositionCallback` only stores the new target, and an IK worker thread (`ikWorker`) solves the newest pending one (a target that arrives during a solve replaces any older pending one). The worker extracts the elbow pose from the solution, clamps the IK result to the joint safety limits, and stores both under `data_mutex_`. The IK result is H-PMP's joint set-point and the postural target; the elbow pose becomes the elbow target while elbow hold is off. Cartesian modes act on a new target at once; only these follow one solve later.

---

## Robot Configuration

Loaded from `robots_config.yaml` via `fic::RobotConfigLoader`, with compiled-in defaults as fallback via `fic::getDefaultConfig()`. Robot models (URDF and MuJoCo XML) live in the `menagerie` package:

```
menagerie/model/
├── Franka_FR3/     (URDF + xml/assets)
├── KinovaGen3_7DOF/(URDF + xml/assets)
├── IIWA_14/        (URDF + XML/assets)
└── Pendulum/       (URDF + xml)
```

Each robot config provides:

| Config Field | Purpose |
|-------------|---------|
| `urdf_path` | Pinocchio model (resolved relative to `<CWD>/src/`) |
| `ee_frame_name` | End-effector frame for FK, Jacobian, IK |
| `elbow_frame_name` | Elbow frame for postural tasks |
| `q_home_rad` | Initial joint target on load |
| `default_ee_position/orientation` | Initial EE target |
| `driver_command` | Shell command to launch vendor driver (`{robot_lib}` placeholder) |
| `is_supported` | Gate for all control — unsupported robots reject all callbacks |
| `has_gripper`, `gripper_range` | Gripper presence and aperture bounds |

### Supported Robots

| Robot | Joints | Driver command | Hardware driver |
|-------|--------|----------------|--------|
| **Franka FR3** | 7 | `libfranka` cartesian_impedance_control (IP 172.16.0.2) | Yes |
| **Kinova Gen3 7DOF** | 7 | Kortex API `01-torque-control_00-Torque_Controller` | Yes |
| **KUKA iiwa 14** | 7 | — | Configuration only (simulation) |
| **Pendulum** | 1 | — | Configuration only (simulation) |

---

## Parameter Loading

FIC tuning parameters are loaded from `config.xml` at controller initialisation time (not every loop):

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

## Build

### Dependencies

| Library | Purpose |
|---------|---------|
| `rclcpp` | ROS 2 C++ client |
| `std_msgs`, `geometry_msgs` | Standard ROS messages |
| `Eigen3` | Linear algebra |
| `pinocchio` | Robot kinematics & dynamics (URDF, FK, IK, RNEA, Jacobians) |
| `fic_library` | FIC library headers (`fic_control.h`, `shared_params.h`, `butterworth_filter.hpp`, `fic_param_loader.hpp`, `robot_config_loader.hpp`) |
| `tinyxml2` | `config.xml` parsing |
| `yaml-cpp` | `robots_config.yaml` parsing |

### Compile

```bash
cd ~/FIC_Interface/fic_ws
colcon build --packages-select fic_control
source install/setup.bash
```

### Launch

```bash
# Default robot
ros2 launch fic_control robot_control_launch.py

# Specific robot
ros2 launch fic_control robot_control_launch.py robot_name:=KinovaGen3_7DOF
```

---

## Important Notes

- **Mode 1 has no velocity damping.** The FIC `Controller` class is pure position feedback. `damping_ratio` only exists in `Planner` (Mode 2), and `Kv_` only in `AdmittanceController` (Modes 3–4). To reduce Mode 1 oscillation, decrease K0 stiffness.

- **Shared memory reserves `kMaxJoints = 16` slots per joint array.** Active joint count is published at runtime in `n_joints`, which is set from `model.nv` after the URDF loads (`publishJointCount()` in `shared_params.h`). Reads and writes in `fic_control` and the vendor driver bridges iterate up to `std::min(num_joints_, kMaxJoints)`. To support more than 16 joints (dual-arm platforms), bump `kMaxJoints` in `shared_params.h`, rebuild `fic_library`, and rebuild every vendor driver that maps the segment.

- **Robot selection triggers a full model reload** under the data mutex — new Pinocchio model, resized vectors, fresh Butterworth filters, and controller re-init. Avoid publishing to `selected_robot` during active control.

- **Controller re-init re-seeds from memory.** A re-init (mode change, H-PMP toggle, some streamed joint targets) no longer reads `config.xml` (v4.4.0); the file is read in the background only when the tuning key changes or after `{rid}_param_update` / a profile change. Before v4.4.0 each re-init re-read the file inside the 1 ms loop, which dropped the loop rate badly while targets streamed.

- **The wrench comes from shared memory only with `wrench_from_api`**, read directly from `tool_wrench[6]` each loop iteration; otherwise it is estimated from the joint torques as `pinv(Jᵀ)(τ_meas − g(q))`. There is no ROS subscriber for the measured force/torque.

- **Only standard ROS message types are used.** All torque output goes through shared memory, not ROS publishers.

- **Config files live in `fic_library/config/`.** Both `config.xml` (tuning parameters) and `robots_config.yaml` (robot definitions) are shared between all packages.