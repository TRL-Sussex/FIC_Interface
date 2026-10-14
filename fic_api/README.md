# fic_api

Python API for FIC-controlled robots. Use the same functions from standard Python scripts,
Jupyter notebooks, or any IDE — no FIC Studio GUI required.

## Install

```bash
# From the fic_api directory at the repo root:
cd ~/FIC_Interface/fic_api
pip install -e .
```

On Ubuntu 24.04 pip refuses to install into the system Python: add
`--break-system-packages`, or use a virtual environment made with
`--system-site-packages` (so that it sees ROS 2's Python packages).

> **Prerequisite:** Your terminal must have the ROS 2 environment sourced.
> Replace `$ROS_DISTRO` with your installed distro (`humble` or `jazzy`):
>
> ```bash
> source /opt/ros/$ROS_DISTRO/setup.bash
> source ~/FIC_Interface/fic_ws/install/setup.bash
> ```

## Quick Start

```python
import math

from fic_api import Robot

# Connect to simulation (default)
with Robot() as r:
    r.home()
    q = r.get_current_joints()       # [rad] from /robot1_trajectory_data
    print(f"Joints (deg): {[f'{math.degrees(v):.1f}' for v in q]}")

    tau = r.get_control_effort()      # [Nm] per joint
    ee = r.get_ee_position()          # [x,y,z] or None (joint modes)

    r.move_joints([0, -30, 0, -90, 0, 60, 0])
    r.wait_until_stopped()
```

### Robot(...)

```python
Robot(target='simulation', num_joints=7, node_name='fic_api_api',
      home_angles_deg=None, robot_id=1, state='full')
```

- `target`: `'simulation'`, `'hardware'` or `'vr'`.
- `robot_id`: the station number (1 to 4); the topics are `robot<N>_*`.
- `home_angles_deg`: the pose `home()` goes to (default `[0, -45, 0, -90, 0, 45, 0]`).
- `state='lite'`: read the 50 Hz state feed instead of the 1 kHz topics. Use it
  from any PC other than the robot's own.

### Hardware

```python
r = Robot(target='hardware', num_joints=7)
r.set_mode(2)  # Joint Impedance With Planner
r.home()
r.close()
```

## API Reference

### Trajectory Data (live from FIC at ~1 kHz)

All joint values in **radians**, Cartesian in **metres**.

| Function | Returns | Notes |
|---|---|---|
| `get_trajectory()` | `dict` | Full parsed `/robot<N>_trajectory_data` |
| `get_current_joints()` | `[rad, ...]` | Current joint positions |
| `get_target_joints()` | `[rad, ...]` | Joint targets |
| `get_joint_errors()` | `[rad, ...]` | Error (target − current) |
| `get_control_effort()` | `[Nm, ...]` | Torque commands (joint modes) |
| `get_planner_targets()` | `[rad, ...]` | Planner output (joint modes) |
| `get_admittance_targets()` | `[rad, ...]` | Admittance target (modes 3-4) |
| `get_desired_effort()` | `[Nm, ...]` | Gravity/desired (modes 3-4) |
| `get_measured_effort()` | `[Nm, ...]` | Measured torque (modes 3-4) |
| `get_ee_position()` | `[x,y,z]` or `None` | EE pos (Cartesian modes) |
| `get_ee_desired()` | `[x,y,z]` or `None` | Desired EE pos |
| `get_ee_error()` | `[dx,dy,dz]` or `None` | Translational error |
| `get_rotation_error()` | `[rx,ry,rz,angle]` or `None` | Rotation error |
| `get_ee_admittance_target()` | `[x,y,z]` or `None` | Admittance target (Cartesian admittance) |

### Movement

| Function | Description |
|---|---|
| `move_joints(angles_deg, velocity=0.5)` | Move to joint angles (degrees) |
| `move_cartesian(x, y, z, rx=0, ry=0, rz=0)` | Move EE (metres, Euler rad) |
| `home()` | Go to home position |
| `stop()` | Hold current position |
| `set_mode(mode)` | Set control mode 1-8 |

### Status

| Function | Description |
|---|---|
| `get_joint_positions()` | Joints in **degrees** (convenience) |
| `get_joint_velocities()` | `[rad/s]` from JointState |
| `get_cartesian_pose()` | `[x,y,z,qx,qy,qz]` |
| `is_moving(threshold=0.01)` | Any velocity > threshold? |
| `wait_until_stopped(timeout=10)` | Block until stopped |

### Gripper

`set_gripper(width)` (metres, 0.0 to 0.08) / `open_gripper()` / `close_gripper()`

### Generic Topics

```python
r.list_topics()                  # [(name, type), ...]
r.subscribe('/my_topic')        # auto-detect type
data = r.get_topic('/my_topic') # latest msg as dict
r.unsubscribe('/my_topic')

r.get_force_torque()             # /robot1_simulation/force_torque
r.get_torque_commands()          # /robot1_fic_torque_commands
```

## Architecture

`Robot()` creates a ROS 2 node that spins in a background thread. It subscribes
to `/robot<N>_trajectory_data`, the joint states and `robot<N>_ee_current_pose`
(the robot is chosen by `robot_id`). The trajectory data parser understands both the joint-mode (block)
and Cartesian-mode (strided) layouts published by the FIC controller.

When you call `close()` (or the `with` block exits), the node and executor are
destroyed cleanly.
