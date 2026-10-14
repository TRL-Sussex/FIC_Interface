# FIC Studio Script Control API

**Python:** 3 (the interpreter of your ROS 2 installation)  
**Source:** `Extensions/script_editor/robot_api.py`

---

## Getting Started

The Script Editor extension provides a Python scripting environment with a
`robot` facade object and flat alias functions that map to every GUI action
the studio can perform.

### Opening the Editor

Open the **Script Editor** page from the sidebar (Programming ▸ Script Editor). Open an
example or one of your scripts, edit it, and press **Run** (Ctrl+Enter on the web page);
**Stop** ends it. On the web page a run goes to the Studio's own target
(Simulation / Hardware / VR in the top bar, shown as *Runs on* above the editor) and asks first
on Hardware; the Classic page has its own **Target** dropdown. Any browser that holds control
in the web interface can save and run scripts.

### The `robot` Object

Every script receives a `robot` variable (a `RobotAPI` instance) plus flat
aliases for the most common calls.
Properties (Signature: *property*) are read without brackets: `robot.state.joints`, not
`robot.state.joints()`.

```python
# Grouped namespace
joints = robot.state.joints          # list[float], degrees
robot.motion.move_joints([0, -45, 0, -90, 0, 45, 0], wait=True)
robot.mode.set(2)                    # Joint Impedance With Planner

# Flat aliases (backward-compatible)
joints = get_joint_positions()
move_joints([0, -45, 0, -90, 0, 45, 0], wait=True)
set_mode(2)
```

### Thread-Safety Model

| Operation | Thread |
|-----------|--------|
| `node.publish_*` (motion, mode, tune, topics) | worker thread — direct call |
| `JointStateMixin` reads (`robot.state.*`) | worker thread — direct call |
| GUI / subprocess ops (`process.*`, `target.*`, tuning save) | main thread via `marshal()` |

Calling `marshal()` on a publish method would deadlock — never do that.

### Cooperative Stop

Scripts can be stopped cleanly by pressing **■ Stop** in the toolbar.  The
`is_stopped()` predicate returns `True` once a stop is requested; `wait()` and
`move_joints(wait=True)` poll it at ~20 Hz and return early.  After a 2-second
grace period the runner calls `terminate()` as a last resort.

```python
for wp in WAYPOINTS:
    if is_stopped():
        break
    robot.motion.move_joints(wp, wait=True, timeout=10.0)
    robot.wait(0.5)
```

---

## API Namespaces

### `robot.state` — Read-Only Telemetry

All reads are thread-safe direct calls to `JointStateMixin` helpers.
Angle values are returned in **degrees**.

| Method / Property | Signature | Units | Description |
|---|---|---|---|
| `robot.state.joints` | property | degrees | Current joint positions. |
| `robot.state.joint_velocity` | property | rad/s | Current joint velocities. |
| `robot.state.joint_effort` | property | Nm | Commanded joint control effort (from the FIC controller), NOT a measured torque sensor reading. For the measured torque subscribe to `/robot1_joint_torques` with `robot.topics`. |
| `robot.state.joint_commanded` | property | degrees | Commanded joint positions. |
| `robot.state.ee_pose` | property | m / — | EE pose as `[x, y, z, qx, qy, qz, qw]` or `[]`. |
| `robot.state.wrench` | property | N / Nm | Tool wrench `(Fx, Fy, Fz, Tx, Ty, Tz)` or `None`. |

**Example:**

```python
joints = robot.state.joints
print(f'Joint 1: {joints[0]:.1f} deg')

ee = robot.state.ee_pose
if ee:
    print(f'EE: x={ee[0]:.3f} y={ee[1]:.3f} z={ee[2]:.3f} m')
```

---

### `robot.motion` — Joint-Space and Cartesian Motion

Publishes directly from the worker thread (thread-safe).

| Method | Signature | Units | Description |
|---|---|---|---|
| `robot.motion.move_joints` | `(angles_deg, wait=False, timeout=10.0, tolerance=None)` | degrees | Publish joint angles; optionally block until reached. |
| `robot.motion.move_cartesian` | `(x, y, z, rx=0.0, ry=0.0, rz=0.0)` | m / rad | Publish a Cartesian pose command. |
| `robot.motion.home` | `()` | — | Move to the config home position. |
| `robot.motion.set_gripper` | `(width_m)` | m | Set gripper width clamped to `[0, 0.08]`. |
| `robot.motion.open_gripper` | `()` | — | Open gripper fully (0.08 m). |
| `robot.motion.close_gripper` | `()` | — | Close gripper fully (0.0 m). |

**`move_joints` returns:**
- `None` when `wait=False`.
- `True` when every joint is within tolerance.
- `False` on timeout or cooperative stop.

**Example:**

```python
# Move to home position and wait up to 10 seconds
reached = robot.motion.move_joints(
    [0.0, -45.0, 0.0, -90.0, 0.0, 45.0, 0.0],
    wait=True,
    timeout=10.0,
    tolerance=1.5,
)
print('reached:', reached)

# Cartesian move (metres; rx/ry/rz in radians)
robot.motion.move_cartesian(0.4, 0.0, 0.3, rx=0.0, ry=0.0, rz=0.0)
```

---

### `robot.mode` — Control-Mode Publishing

| Method | Signature | Description |
|---|---|---|
| `robot.mode.set` | `(mode_int)` | Publish integer mode 1–8 and sync the GUI selector. |
| `robot.mode.publish_mode` | `(mode_int)` | Same as `set` — explicit alias. |
| `robot.mode.postural` | `(enabled)` | Enable/disable null-space postural controller. |
| `robot.mode.elbow` | `(enabled)` | Enable/disable null-space elbow constraint. |
| `robot.mode.hapfic` | `(enabled)` | Enable/disable HapFIC hybrid mode (engages in modes 6–8). |
| `robot.mode.admittance_planner` | `(enabled)` | Enable/disable the planner on the admittance target (admittance modes). |
| `robot.mode.active_profile` | `(profile_name)` | Publish a named parameter profile. |

**Control Modes:**

| Mode | Name |
|------|------|
| 1 | Joint Impedance |
| 2 | Joint Impedance With Planner |
| 3 | Joint Admittance |
| 4 | Joint Admittance Haptic |
| 5 | Cartesian Impedance |
| 6 | Cartesian Planner |
| 7 | Cartesian Admittance |
| 8 | Cartesian Admittance Haptic |

**Example:**

```python
robot.mode.set(2)            # Joint Impedance With Planner
robot.mode.postural(True)    # Enable null-space postural
robot.wait(0.5)
robot.mode.postural(False)
```

---

### `robot.process` — Subprocess Control

All methods are marshalled to the GUI thread (they touch Qt widgets and
spawn/kill subprocesses).

| Method | Description |
|---|---|
| `robot.process.start_sim()` | Start the MuJoCo simulation. |
| `robot.process.stop_sim()` | Stop the MuJoCo simulation. |
| `robot.process.start_fic()` | Start the FIC simulation controller. |
| `robot.process.stop_fic()` | Stop the FIC simulation controller. |
| `robot.process.start_controller()` | Start the hardware FIC controller. |
| `robot.process.stop_controller()` | Stop the hardware FIC controller. |
| `robot.process.start_driver()` | Start the hardware driver. |
| `robot.process.stop_driver()` | Stop the hardware driver. |
| `robot.process.start_vr()` | Start the VR relay. |
| `robot.process.stop_vr()` | Stop the VR relay. |
| `robot.process.start_bags()` | Start `ros2 bag record -a`. |
| `robot.process.stop_bags()` | Stop the bag recorder. |
| `robot.process.start_plotter()` | Launch the live plotter. |
| `robot.process.stop_plotter()` | Stop the live plotter. |

**Example:**

```python
robot.process.start_bags()
robot.wait(0.5)
robot.motion.move_joints([0.0, -45.0, 0.0, -90.0, 0.0, 45.0, 0.0], wait=True)
robot.process.stop_bags()
```

---

### `robot.target` — Sim/Hardware/VR Target

| Method / Property | Description |
|---|---|
| `robot.target.current` | Read current target string: `'simulation'`, `'hardware'`, or `'vr'`. |
| `robot.target.set_target(target)` | Change target (marshalled to GUI thread). |

**Example:**

```python
print(f'Target: {robot.target.current}')
robot.target.set_target('simulation')
```

---

### `robot.tune` — Parameter Tuning

| Method | Signature | Description |
|---|---|---|
| `robot.tune.param_update` | `(param_type, value)` | Ask the controller to re-read the tuning in `config.xml`. The arguments are sent along but the value is not applied directly: save the tuning first (`robot.tune.save`). |
| `robot.tune.save` | `(robot_name=None, joint_id=1, mode=1, params=None)` | Save tuning to XML via XMLManager (marshalled). |

**Example:**

```python
# Ask the controller to re-read config.xml (after robot.tune.save or a Tuning Panel save)
robot.tune.param_update(0, 0)
robot.wait(0.2)
```

---

### `robot.safety` — Emergency Stop

| Method | Description |
|---|---|
| `robot.safety.estop()` | Publish mode 1, then ramp every active robot's torque down (`broadcast_stop`). |
| `robot.safety.broadcast_stop()` | Send stop to every active robot (marshalled). |
| `robot.safety.pause()` | Publish mode 1, so this robot holds its pose in Joint Impedance. |

**Example:**

```python
# Abort if force exceeds threshold
wrench = robot.state.wrench
if wrench and abs(wrench[2]) > 20.0:   # Fz > 20 N
    robot.safety.estop()
```

---

### `robot.topics` — Generic ROS Topics

| Method | Signature | Description |
|---|---|---|
| `robot.topics.list_topics` | `()` | Return `[(name, type_str), ...]` from the ROS graph. |
| `robot.topics.subscribe` | `(topic_name)` | Subscribe to a topic (auto-discovers type). |
| `robot.topics.unsubscribe` | `(topic_name)` | Destroy the subscription and clear cache. |
| `robot.topics.get` | `(topic_name)` | Return latest message for topic, or `None`. |
| `robot.topics.cleanup` | `()` | Destroy all subscriptions (called at teardown). |

**Example:**

```python
topics = robot.topics.list_topics()
print(f'Found {len(topics)} topics')

robot.topics.subscribe('/robot1_ee_current_pose')
robot.wait(0.2)
msg = robot.topics.get('/robot1_ee_current_pose')
if msg:
    print(f'EE: {msg.position.x:.3f}')
```

---

### `robot` — Top-Level Helpers

| Method / Property | Signature | Description |
|---|---|---|
| `robot.rid` | property | Robot-id string (e.g. `'robot1'`). |
| `robot.target_mode` | property | Current target string. |
| `robot.wait` | `(seconds)` | Sleep, honouring cooperative stop at ~20 Hz. |
| `robot.is_stopped` | `()` | Return `True` if a stop has been requested. |

---

## Flat Aliases (Backward Compatibility)

These names are injected directly into the script's global namespace so
existing scripts run without modification.

| Alias | Maps to |
|---|---|
| `move_joints(angles_deg, wait=False, ...)` | `robot.motion.move_joints` |
| `move_cartesian(x, y, z, rx, ry, rz)` | `robot.motion.move_cartesian` |
| `home()` | `robot.motion.home` |
| `set_gripper(width_m)` | `robot.motion.set_gripper` |
| `open_gripper()` | `robot.motion.open_gripper` |
| `close_gripper()` | `robot.motion.close_gripper` |
| `get_joint_positions()` | `robot.state.joints` |
| `get_cartesian_pose()` | `robot.state.ee_pose` |
| `get_ee_position()` | `robot.state.ee_pose[:3]` (`None` when unknown) |
| `get_target_mode()` | `robot.target_mode` |
| `is_moving()` | always `False` (reserved) |
| `wait_until_stopped(timeout=10)` | `robot.wait(timeout)`: it waits the whole timeout |
| `get_force_torque()` | `robot.state.wrench` |
| `get_joint_velocity()` | `robot.state.joint_velocity` |
| `get_joint_effort()` | `robot.state.joint_effort` |
| `set_mode(mode)` | `robot.mode.publish_mode` |
| `list_topics()` | `robot.topics.list_topics` |
| `subscribe(topic)` | `robot.topics.subscribe` |
| `unsubscribe(topic)` | `robot.topics.unsubscribe` |
| `get_topic(topic)` | `robot.topics.get` |
| `wait(seconds)` | `robot.wait` |
| `sleep(seconds)` | `time.sleep` |
| `is_stopped()` | `robot.is_stopped` |
| `stop()` | `robot.safety.estop` |

---

## Examples

### Example 01 — Read State

```python
"""01_read_state.py — Read robot state using the new API."""
print('=== Read State Example ===')

joints = robot.state.joints          # list[float], degrees
print(f'Joint positions (deg): {[f"{j:.1f}" for j in joints]}')

ee = robot.state.ee_pose             # [x, y, z, qx, qy, qz, qw] or []
if ee:
    print(f'EE position (m): x={ee[0]:.3f}  y={ee[1]:.3f}  z={ee[2]:.3f}')

wrench = robot.state.wrench
if wrench:
    print(f'Wrench (N/Nm): Fx={wrench[0]:.2f}  Fy={wrench[1]:.2f}  Fz={wrench[2]:.2f}')
```

---

### Example 02 — Joint Trajectory

```python
"""02_joint_trajectory.py — Move through a simple joint-space trajectory."""
WAYPOINTS_DEG = [
    [0.0,  -45.0,  0.0, -90.0, 0.0,  45.0, 0.0],
    [30.0, -30.0, 20.0, -60.0, 0.0,  60.0, 0.0],
    [0.0,  -45.0,  0.0, -90.0, 0.0,  45.0, 0.0],
]

for idx, wp in enumerate(WAYPOINTS_DEG):
    if is_stopped():
        print('Stopped by user.')
        break
    print(f'Moving to waypoint {idx + 1}/{len(WAYPOINTS_DEG)}...')
    reached = robot.motion.move_joints(wp, wait=True, timeout=15.0, tolerance=1.5)
    print(f'  {"Reached" if reached else "Timeout/stopped"}')
    robot.wait(0.3)
```

---

### Example 03 — Cartesian Circle

```python
"""03_cartesian_circle.py — Sweep a small circle in Cartesian space."""
import math

RADIUS_M = 0.05
N_STEPS = 24
CENTRE = (0.4, 0.0, 0.3)

for i in range(N_STEPS + 1):
    if is_stopped():
        break
    angle = 2 * math.pi * i / N_STEPS
    x = CENTRE[0] + RADIUS_M * math.cos(angle)
    y = CENTRE[1] + RADIUS_M * math.sin(angle)
    robot.motion.move_cartesian(x, y, CENTRE[2])
    robot.wait(0.08)
```

---

### Example 04 — Mode and Tune

```python
"""04_mode_and_tune.py — Switch control modes and adjust parameters."""
print(f'Target: {robot.target_mode}')

robot.mode.set(1)          # Joint Impedance
robot.wait(0.5)

robot.mode.postural(True)
robot.wait(0.3)

robot.tune.param_update(0, 0)    # the controller re-reads config.xml
robot.wait(0.2)

robot.mode.postural(False)
print('Done.')
```

---

### Example 05 — Record and Plot

```python
"""05_record_and_plot.py — Start bags and plotter, run a trajectory, then stop."""
robot.process.start_bags()
robot.wait(0.5)
robot.process.start_plotter()
robot.wait(0.5)

WAYPOINTS_DEG = [
    [0.0,  -45.0, 0.0, -90.0, 0.0, 45.0, 0.0],
    [20.0, -30.0, 10.0, -70.0, 0.0, 50.0, 0.0],
    [0.0,  -45.0, 0.0, -90.0, 0.0, 45.0, 0.0],
]

for idx, wp in enumerate(WAYPOINTS_DEG):
    if is_stopped():
        break
    reached = robot.motion.move_joints(wp, wait=True, timeout=12.0)
    robot.wait(0.3)

robot.process.stop_bags()
print('Recording done. Plotter remains open.')
```

---

## Safety Notes

- **E-STOP:** Press Space or the E-STOP button at any time.  From a script,
  call `robot.safety.estop()` or the flat alias `stop()`.
- **Degrees:** The API uses degrees for joint angles throughout.  The
  conversion to radians (for ROS publishes) is handled internally.
- **Blocking waits:** `move_joints(wait=True)` polls at 20 Hz; a tight Python
  loop with no cooperative check cannot be interrupted by the Stop button until
  the current iteration completes.  Add `if is_stopped(): break` inside loops.
- **Sandboxing:** Scripts run in the same Python process as the studio.
  Avoid file system writes to paths outside `~/.fic_studio_scripts/`.
