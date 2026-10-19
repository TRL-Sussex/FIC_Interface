# fic_studio — FIC Control Studio GUI

> **Package:** `fic_studio`
> **Node name:** `fic_studio_node`
> **Entry points:** `fic_studio_pro`, `fic_dev`
> **Language:** Python 3 (PyQt5 + ROS 2 Jazzy/Humble)

FIC Control Studio is the GUI for operating, tuning and programming robots controlled by the Fractal Impedance Control (FIC) library. It works with real hardware (`fic_control`) and with MuJoCo simulation (`fic_control`), from its own window or from a browser (the web interface).

---

## Package structure

```
fic_studio/
├── package.xml, setup.py, setup.cfg
├── fic_studio/
│   ├── entry_pro.py, entry_dev.py     # entry points: Studio and Dev Console
│   ├── main_window_pro.py             # the Studio's main window (mixins composed here)
│   ├── main_window_dev.py             # the Dev Console
│   ├── ros_node.py                    # FicStudioNode: every publisher the GUI uses
│   ├── config.py, platform_utils.py   # paths and platform differences
│   ├── xml_manager.py                 # reads and writes config.xml (tuning)
│   ├── fic_studio_robots.yaml         # the Studio's robot list
│   ├── mixins/                        # one mixin per page or subsystem
│   ├── components/, widgets.py        # shared widgets
│   ├── theme/, styles.py              # light and dark themes
│   ├── extensions/                    # loads the extensions in Extensions/
│   ├── web_remote/                    # the web interface: server, accounts, ui/ pages
│   ├── robot_import/                  # Add a robot (URDF importer) and driver libraries
│   ├── fleet/, teleop/                # other Studios on the network, teleoperation
│   ├── keyboard/                      # keyboard driving
│   └── experiments/                   # bag recording helpers
├── tools/                             # small development tools
└── test/                              # pytest suite
```

### Where it fits in the workspace

```
fic_ws/src/
├── communication/          # Custom ROS msgs/srvs
├── fic_control/           # Real robot control node
├── fic_control/         # Simulation control node
├── fic_studio/             # ◄ THIS PACKAGE — GUI application
├── menagerie/              # Robot models (URDF + MuJoCo XML)
├── mujoco_native/          # MuJoCo simulator (CPU, the default)
└── mujoco_newton/          # Optional GPU simulator (NVIDIA Warp)
```

---

## Architecture

The main window is built from **mixins**: each page's logic is a mixin class,
composed into the window by multiple inheritance. Optional pages are
**extensions** (`Extensions/` at the repository root): Sequences, Teach Mode
and the Script Editor among them. Each has a Qt page and, if its manifest has a
`web` block, a page of its own in the web interface. `FicStudioNode` holds the
ROS 2 publishers for every robot.

```
FIC Control Studio (PyQt5)  ──  web interface (any browser)
        │
   FicStudioNode (ROS 2)
        │ topics
   ┌────┴─────────────┐
fic_control      mujoco_native
```

## Pages

| Page | Where | What it does |
|------|-------|--------------|
| Dashboard | `DashboardMixin` | Robot, processes, live joints and quick actions |
| Multi-Robot | `MultiRobotMixin` | Up to four robot cards, each with its station, mode and targets |
| Control bar | `ControlBarMixin` | Docked at the bottom: modes, joint and Cartesian targets, gripper |
| Tuning Panel | `TuningMixin` | FIC parameters per joint or axis and mode, read from and saved to `config.xml` |
| Safety | `SafetyMixin` | Cartesian, joint and velocity limits, sent to the controllers |
| Robot Configuration | `RobotConfigMixin` | Robot list and settings; one Save writes `fic_studio_robots.yaml`, `robots_config.yaml` and `config.xml`. **Add a robot** imports a URDF; **Add the driver library** installs a driver |
| Topic Monitor | `TopicMonitorMixin` | Every ROS 2 topic with its type, publishers, subscribers and latest message |
| Settings | `SettingsMixin` | Theme, default robot, simulator, web remote, extensions, shortcuts |
| Help | `HelpMixin` | Searchable help and the documentation |
| Sequences, Teach Mode, Script Editor | `Extensions/sequences`, `teach_mode`, `script_editor` | Waypoint sequences; teaching by hand; Python scripts with the robot API (`docs/scripting/API.md`) |

---

## ROS 2 node: `FicStudioNode`

Defined in `ros_node.py`. For each active robot `N` (1 to 4) it publishes:

| Topic | Type | Used for |
|-------|------|----------|
| `robotN_control_mode` | `Int32` | Mode (simulation) |
| `robotN_mode_select` | `String` | Mode (hardware) |
| `robotN_target_joint_angles` | `JointState` | Joint targets (simulation) |
| `robotN_desired_joint_angles` | `Float32MultiArray` | Joint targets (hardware) |
| `command_ee_position_change_robotN` | `Pose` | Cartesian target |
| `robotN_gripper_width` | `Float32` | Gripper |
| `robotN_selected_robot`, `robotN_number_of_joints` | `String`, `Int32` | Robot selection |
| `robotN_param_update`, `robotN_active_profile` | `String` | Tuning reload and profile |
| `robotN_postural_enable`, `robotN_elbow_enable`, `robotN_hpmp_enable`, `robotN_admittance_planner_enable` | `Bool` | Controller layers |
| `robotN_gravity_compensated`, `robotN_wrench_gravity_compensated`, `robotN_wrench_from_api` | `Bool` | How torque and wrench are measured |
| `robotN_wrench_external` | `Wrench` | A teleoperation partner's contact |
| `robotN_safety/*` | `Float64MultiArray` | Safety limits (from the Safety page) |

---

## Process management

`ProcessMixin` starts and stops the processes behind the toolbar's **▶ Start**
and the BAG and PLT buttons:

| Process | Command |
|---------|---------|
| Controller (hardware) | `ros2 launch fic_control robot_control_launch.py robot_name:=… station_id:=…` |
| Robot driver | The robot's `driver_command` from `robots_config.yaml` |
| Controller (simulation) | `ros2 launch fic_control sim_control_launch.py robot_name:=… num_joints:=… station_id:=…` |
| Simulator | `ros2 launch mujoco_native simulation_launch.py robot_name:=… station_id:=…` (or `mujoco_newton`) |
| Bag recording | `ros2 bag record` |
| Plotter | `ros2 run fic_control plotter_node.py` |

They run in their own process groups. Stop sends SIGTERM, then SIGKILL. On
exit (window close, interpreter exit, SIGTERM, SIGHUP, SIGQUIT) they are
stopped one at a time, **driver first**, so the driver leaves torque mode while
its controller still runs. An exception inside a Qt slot or timer is reported
and does not stop the robot.

## Web remote

`WebRemoteMixin` serves the Studio to browsers on the network (port 8088, or
the next free one; HTTPS and sign-in by default). Commands run on the Qt thread
through the same handlers the buttons use; one browser holds control at a time,
and any browser can E-STOP. Settings are under `web_remote:`. See
`fic_docs/pages/web-remote.html` for the reference and the HTTP API.

---

## XMLManager

`xml_manager.py` reads and writes the FIC tuning in `config.xml`:

- parameters per robot and per mode (1 to 8, and the H-PMP block, id 100),
  with named profiles;
- per-joint scalar, list, planner and admittance parameters;
- per-axis Cartesian parameters (X/Y/Z and rotation);
- atomic save with pretty-printed XML.

---

## Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| `robots_config.yaml` | `fic_library/config/` | Robot definitions (URDF, frames, driver, home poses) |
| `config.xml` | `fic_library/config/` | FIC tuning parameters per robot per mode |
| `.robot_selector_gui_settings.yaml` | `~/` | GUI settings (theme, default robot, extensions, web remote) |
| `accounts.json`, certificate | `~/.fic_studio/` | Web remote accounts and HTTPS certificate |
| scripts | `~/.fic_studio_scripts/` | Script Editor scripts |
| `robot_sequences.yaml` | `~/` | Sequences |

---

## Build and run

Dependencies come from `fic_install/install-deps.sh` (PyQt5 with QtWebEngine,
pyqtgraph, numpy, scipy, PyYAML, cryptography and the ROS 2 Python packages).

```bash
cd ~/FIC_Interface
./build.sh --packages-select fic_studio
source fic_ws/install/setup.bash

ros2 run fic_studio fic_studio_pro    # FIC Control Studio
ros2 run fic_studio fic_dev           # Dev Console: a smaller GUI for a second PC
```

---

## Event loop

Qt and ROS 2 share the main thread. A `QTimer` fires every 10 ms and processes
ROS callbacks for up to 8 ms (`executor.spin_once(timeout_sec=0)` in a loop),
then hands back to Qt (`entry_pro.py`).

---

## Important notes

- **Do not publish to `robotN_selected_robot` during active control.** It
  reloads the robot model in `fic_control` and `fic_control`, which can
  crash a controller mid-motion (Jacobian size mismatch).
- **`config.xml` is shared between the nodes.** Tuning saved in the Studio is
  picked up by `fic_control` and `fic_control` when they re-read it
  (`robotN_param_update`).
