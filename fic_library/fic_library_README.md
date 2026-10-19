# fic_library — Fractal Impedance Control Core Library

> **Location:** `fic_library/`
> **Type:** C++ library (headers, plus `FIC.cpp` included by them) and the shared configuration files
> **Include path:** `<fic/...>`

The `fic_library` package contains the core Fractal Impedance Control algorithm, all supporting utilities, and the shared configuration files used by every other package in the workspace.

---

## Directory Structure

```
fic_library/
├── config/
│   ├── config.xml                  # FIC tuning parameters (per robot, per mode)
│   └── robots_config.yaml          # Robot definitions (URDF paths, frames, drivers)
├── include/fic/
│   ├── FIC.cpp                     # Core FIC algorithm (Controller, Planner, Admittance)
│   ├── fic_control.h               # ficControl wrapper (JC, RC, postural sub-modules)
│   ├── fic_param_loader.hpp        # XML parameter loading into FIC data structures
│   ├── robot_config_loader.hpp     # YAML robot config loading + defaults
│   ├── kinematics_solver.hpp       # Kinematics solver interface
│   ├── solvers/                    # Pinocchio solvers (serial and closed-loop) and the factory
│   ├── butterworth_filter.hpp      # Second-order Butterworth low-pass filter
│   ├── shared_params.h             # POSIX shared memory structs for hardware comms
│   ├── fic_joint_core.h            # The joint layer shared by fic_control and the MCU
│   ├── joint_params_pod.h          # Joint parameters as plain data (for the MCU)
│   ├── inner_loop.h, mcu_node.h    # The inner loop that runs on the MCU
│   ├── fic_link.h, mcu_link.hpp    # The FIC link between fic_control and the MCU (UDP 7350)
│   └── kortex_adapter.h, kortex_min.h  # Kinova Kortex access for the MCU
└── test/                           # host tests (ctest)
```

---

## Core Algorithm — `FIC.cpp`

The `FIC<T>` template class implements the Fractal Impedance Controller with three nested component types per degree of freedom:

### Controller

Pure position-error feedback using a **nonlinear saturation mapping**. Maps position error through configurable breakpoints (`x_b`) and force limits (`E_max`) with multiple saturation stages, producing a stiffness-scaled torque output.

- **Parameters:** `K0` (stiffness), `x_b` (error breakpoints), `E_max` (force limits), `S_Start` (saturation ratios), `n_` (number of saturations)
- **Key property:** No velocity damping — purely position-based

### Planner

Trajectory generator that produces smooth position references given a desired target. Computes acceleration-limited, damping-controlled motion profiles with optional joint wrapping.

- **Parameters:** `stiffness`, `velDesired`, `accMax`, `dampingRatio`, `frequency`, `wrapped` (0/1/2), `posLower`/`posUpper`
- **Wrapping modes:** 0 = none, 1 = [-π, π], 2 = [0, 2π]

### AdmittanceController

Integrates applied force over time to compute compliant motion. Implements a virtual mass-spring-damper: `acceleration = invM × (effort - Kv × velocity)`.

- **Parameters:** `InvM` (inverse mass), `Kv` (damping), `dFlow` (acceleration limits), `Flow` (velocity limits), `Pose` (position limits)
- **Integration:** Euler integration of acceleration → velocity → position, all clamped

### FIC Container API

```cpp
FIC<double> fic(N_s, asymmetric, N_negSat, n_dof, enable_admittance);

fic.control(dof_index, error);        // Controller output
fic.plan(dof_index, posDesired, dt);  // Planner output
fic.admittance(dof_index, dt);        // Admittance output (if enabled)
```

---

## Control Wrapper — `fic_control.h`

The `ficControl<T>` class wraps `FIC<T>` and exposes two high-level sub-modules:

### `JC` — Joint Controller

| Method | Mode | Description |
|--------|------|-------------|
| `JC.impedance(i, ...)` | 1 | Pure FIC stiffness control |
| `JC.impedanceWithPlanner(i, ...)` | 2 | FIC + trajectory planner |
| `JC.admittance(i, ..., m)` | 3/4 | Admittance control (m=1 or m=2) |

### `RC` — Robot (Cartesian) Controller

| Method | Mode | Description |
|--------|------|-------------|
| `RC.impedance(...)` | 5 | Task-space impedance via Jacobian transpose |
| `RC.planner(...)` | 6 | Task-space with trajectory planning |
| `RC.admittance(...)` | 7/8 | Task-space admittance (m=1 or m=2), driven by `setDesiredWrench()` (zero by default) minus the measured wrench |
| `RC.hpmp(...)` | 6-8 | H-PMP: task-space admittance nested over the joint layer (`H-PMP` block, id 100) |

Both sub-modules also provide initialisation methods (`initJointController`, `initJointPlanner`, `initJointAdmittance`, `initCartesianController`, etc.) that configure the underlying FIC components from parameter structs.

---

## Parameter Loader — `fic_param_loader.hpp`

Reads `config.xml` and populates typed parameter structs for controller initialisation.

### Data Structures

| Struct | Fields | Used By |
|--------|--------|---------|
| `ScalarParams` | stiffness, diff_effort_max | Controller |
| `ListParams` | error_max, force_max, saturation_ratios, velocity | Controller |
| `PlannerParams` | stiffness, vel_desired, acc_max, damping_ratio, frequency, wrapped, pos_limits | Planner |
| `AdmittanceParams` | InvM, damping, dFlow, Flow, Pose | AdmittanceController |
| `CartesianAxisParams` | All of the above per Cartesian axis + planner + admittance | RC |
| `CartesianAllParams` | pos_axes[3] (X/Y/Z) + rot_axis (Q) | RC |

### API

```cpp
// Load per-joint parameters for a specific robot and control mode
fic::ParameterLoader::loadJointParams(xml_file, robot_name, joint_id,
    scalars, lists, planners, admittance,
    control_mode /* 1 */, is_postural /* false */, profile_name /* "default" */);

// Load Cartesian parameters
fic::ParameterLoader::loadCartesianParams(xml_file, robot_name, cart_params,
    control_mode /* 5 */, profile_name /* "default" */);

// Fallback: load hardcoded safe defaults
fic::ParameterLoader::loadHardcodedParams(num_joints, scalars, lists, planners, admittance, cart_params);
```

All loading is **fail-safe**: if an XML tag is missing, the parameter retains its memory default. No exceptions are thrown on missing tags.

---

## Robot Config Loader — `robot_config_loader.hpp`

Reads `robots_config.yaml` and provides `RobotConfig` structs containing URDF path, frame names, home positions, driver commands, and more.

```cpp
fic::RobotConfigLoader loader;
loader.loadFromDefault();  // Loads from fic_library/config/robots_config.yaml

auto config = loader.getConfig("Franka_FR3");  // Returns std::optional<RobotConfig>
auto ids    = loader.getRobotIds();             // All loaded robot names

// Fallback for unknown robots
auto fallback = fic::getDefaultConfig("Franka_FR3");

// Path resolution
std::string urdf = fic::resolveUrdfPath(config, workspace_path);
std::string mjcf = fic::resolveMujocoPath(config, workspace_path);
```

Built-in defaults exist for Franka FR3, Kinova Gen3, KUKA iiwa 14, and Pendulum.

---

## Butterworth Filter — `butterworth_filter.hpp`

Second-order IIR low-pass filter used to smooth joint position, velocity, and torque feedback.

```cpp
ButterworthFilter filter(cutoff_hz, sample_rate_hz, initial_value);
double filtered = filter.filter(raw_value);
```

---

## Shared Memory — `shared_params.h`

POSIX shared memory structs for lock-free communication between `fic_control` and vendor robot drivers. See the `fic_control` README for full field documentation.

| Struct | Segment Name | Purpose |
|--------|-------------|---------|
| `SharedFeedback` | `/robot_feedback_memory` | Bidirectional joint data + wrench + gripper |
| `SharedDesiredJointState` | `/desired_joint_angles_memory` | Joint angle targets |

Provides `initSharedMemory()` and `cleanupSharedMemory()` lifecycle functions.

---

## Configuration Files

### `config.xml`

Mode-separated FIC tuning parameters for each robot. Structure:

```xml
<robots>
  <robot name="Franka_FR3">
    <mode id="1" name="Joint Impedance" profile="default">
      <joint id="0">
        <controller>
          <stiffness>600.0</stiffness>
          <error_max>0.05,0.1</error_max>
          <force_max>5,39</force_max>
          <saturation_ratios>0.9</saturation_ratios>
          <n_saturations>2</n_saturations>
          <asymmetric>false</asymmetric>
          <n_neg_sat>1</n_neg_sat>
        </controller>
        <!-- <planner>, <admittance> as siblings when the mode uses them -->
      </joint>
      <!-- joints 1–6 -->
    </mode>
    <!-- modes 2–8 ... -->
    <postural>
      <controller>...</controller>
      <planner>...</planner>
    </postural>
  </robot>
</robots>
```

Each `<joint>`, Cartesian axis (`<x>`/`<y>`/`<z>`/`<q>`), and `<postural>` groups its parameters
into up to three sibling sub-blocks: `<controller>` (FIC inner loop), `<planner>` (trajectory
smoothing), and `<admittance>` (virtual mass / damping). The loader also reads older files
without the `<controller>` nesting.

Read by `fic_param_loader.hpp` (C++) and `xml_manager.py` (Python/GUI).

### `robots_config.yaml`

Robot definitions shared across all packages. Read by `robot_config_loader.hpp` (C++) and `fic_studio` (Python). The keys are documented in `fic_docs/pages/robot-config.html` and `config-xml.html`.

---

## Consumers

| Package | Uses |
|---------|------|
| `fic_control` | All headers + `config/` files |
| `fic_control` | All headers + `config/` files |
| `fic_studio` | `config.xml` and `robots_config.yaml` (via Python `xml_manager.py`) |
| `mujoco_native` | `robots_config.yaml` (`mujoco_xml_path`) |
