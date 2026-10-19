# fic_control in simulation

> **Launch:** `sim_control_launch.py` (this package; `robot_control_launch.py` is the same controller for a real robot)
> **Node:** `fic_control`'s executable with `target: simulation`, named `fic_simulation_node_{station_id}`

A simulated arm runs the same two layers as a real one: the task-space layer, the `fic_control` node, and the
joint inner loop, `fic_inner_loop`. This package starts both for one station; the simulator is started separately
(`mujoco_native`'s `simulation_launch.py`, or the Newton backend).

```
FIC Studio / scripts ── robot{N}_* topics ──▶ fic_control (target: simulation)
                                                 │ CMD ▼   ▲ STATE   (FIC-link, UDP)
                                              fic_inner_loop --adapter sim  (127.0.0.1:7200+N)
                                                 │ SimTorque ▼   ▲ SimStep
                                              mujoco_native (one SimStep per physics step)
```

## Launch

```bash
# The controller (the simulator separately)
ros2 launch fic_control sim_control_launch.py robot_name:=Franka_FR3
ros2 launch mujoco_native simulation_launch.py robot_name:=Franka_FR3

# Multi-robot: one controller per station, joint name prefixes for a shared scene
ros2 launch fic_control sim_control_launch.py robot_name:=Franka_FR3 station_id:=1 joint_name_prefix:=r1_
ros2 launch fic_control sim_control_launch.py robot_name:=Franka_FR3 station_id:=2 joint_name_prefix:=r2_
```

| Argument | Default | Meaning |
|---|---|---|
| `robot_name` | `KinovaGen3_7DOF` | Robot from `robots_config.yaml` |
| `station_id` | `1` | The `robotN_*` topics and the inner loop's port, `7200 + N` (Masters 6100+: `fic_link.h` `simInnerLoopPort`) |
| `source_prefix` | `simulation` | The simulator's topic prefix (`robotN_simulation`) |
| `joint_name_prefix` | (empty) | Prefix of the simulator's joint and actuator names |
| `plant` | `auto` | `lockstep` (mujoco_native steps the inner loop), `topics` (the node relays `joint_states` → SimStep and SimTorque → `actuators_cmds`, for Newton), `auto` (chosen from the source prefix) |
| `inner_loop` | `true` | Start the station's `fic_inner_loop --adapter sim` |
| `num_joints` | `7` | Unused: the joint count comes from the robot's configuration |

## How a step runs

- `mujoco_native` sends one SimStep per physics step (simulated time, joint positions, measured joint torques)
  to `127.0.0.1:7200+N` and applies the SimTorque that answers that step (a late reply to an earlier step is
  dropped). While no inner loop answers, it never
  waits and holds the arm the way a robot's own controller would.
- `fic_inner_loop --adapter sim` ticks once per SimStep, on simulated time. It measures the simulator's step from the
  SimSteps and runs at it (1 ms in FIC Studio, 2 ms in the experiment suite), whatever CONFIG says; STATE reports
  the step, and the node's filters follow it.
- The node runs the control law on its 1 kHz timer and sends the latest CMD; its filters step once per STATE.
  `mujoco_native` logs the real-time factor every 10 s (a warning below 0.95: the node's planners and target
  streams run on wall time). If the node stops, the inner loop holds the arm,
  ramps it out of torque and the simulator holds it.

## Topics

The node takes every topic the hardware target takes (see the Topics page of the docs). In simulation it also takes
the simulator's older topics: `robot{N}_control_mode` (`Int32`, `0` stops, `1..8` the modes) and
`robot{N}_target_joint_angles` (`JointState`). It reads `/{source_prefix}/gravity_torques`, `…/contact_torques` and
`…/force_torque`, and publishes `/robot{N}_fic_torque_commands` (the law's torque), `/{source_prefix}/actuator_torques`
and `/{source_prefix}/actuators_cmds` (the inner loop's torque) and `robot{N}_jacobian`.

## Tuning

The node reads `config.xml` from this package's share directory (installed from `fic_library/config/`). Build with
`--symlink-install` so edits made by the Tuning page reach it without a rebuild, and always build Release:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`FIC_HARDWARE_MODEL=off` in the environment runs the ideal simulator whatever the robot's `hardware_model` block
says; `mujoco_native` reads the same variable.

## Plotter

```bash
ros2 run fic_control plotter_node.py --ros-args -p station_id:=1 -p num_joints:=7
```
