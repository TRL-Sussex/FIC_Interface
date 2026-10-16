# FIC Interface

FIC Interface is the code behind **FIC Control Studio**, software for running
robot arms under Fractal Impedance Control (FIC), in simulation and on real
robots. It contains:

- `fic_library/`: the FIC controller, a C++ library;
- `fic_ws/`: the ROS 2 workspace, with the controller nodes for MuJoCo
  simulation (`fic_simulation`) and for real robots (`fic_hardware`), the
  simulators, the robot models and the Studio itself (`fic_studio`), a desktop
  and web interface for running, tuning and scripting the robots;
- `robot_library/`: the robot drivers (Franka libfranka, Kinova Kortex);
- `docker/`: a ready-to-run simulation image;
- `fic_docs/`: the full documentation.

| Robot | Simulation | Hardware |
|---|---|---|
| Franka FR3 | yes | yes (libfranka) |
| Kinova Gen3 7-DOF | yes | yes (Kortex API) |
| KUKA iiwa 14 | yes | configuration only |
| Pendulum | yes | configuration only |

It runs on Ubuntu 24.04 with ROS 2 Jazzy and on Ubuntu 22.04 with ROS 2
Humble, on macOS (Apple Silicon, simulation only) and in Docker on Windows,
macOS and Linux (simulation only).

---

## Installation

**Docker (Windows, macOS, Linux; simulation only).** Install Docker, then run
`docker/fic-studio.sh` (macOS and Linux) or double-click `docker/fic-studio.cmd`
(Windows). FIC Control Studio opens in your browser with the simulator and the
extensions ready; nothing else is installed on your computer. See
[`docker/README.md`](docker/README.md).

**Native install (Ubuntu 22.04 or 24.04, or macOS on Apple Silicon):**

> **Before you install:** see [everything the installer puts on your computer](fic_install/WHAT-GETS-INSTALLED.md):
> the packages, when it asks for admin rights, the files it creates outside this folder, and how to
> remove them. `bash install.sh --dry-run` shows the plan without changing anything.

```bash
git clone https://github.com/TRL-Sussex/FIC_Interface ~/FIC_Interface
cd ~/FIC_Interface
bash install.sh
```

`install.sh` detects the platform. On Ubuntu it installs ROS 2 (Humble on
22.04, Jazzy on 24.04) and the dependencies, builds the workspace and adds
desktop launchers; on macOS it sets up a RoboStack (conda) environment for
simulation (`fic_install/macos/`). `--dry-run` shows the plan without changing
anything, `--yes` asks no questions and `--update` pulls this checkout first.
See [`fic_install/README.md`](fic_install/README.md).

**By hand (Ubuntu).** With ROS 2 installed:

```bash
cd ~/FIC_Interface
sudo bash fic_install/install-deps.sh
./build.sh
source fic_ws/install/setup.bash
```

Pinocchio (`ros-$ROS_DISTRO-pinocchio`, installed by `install-deps.sh`) is the
default kinematics solver. Without it the workspace still builds, but robots
run in joint space only unless another solver is registered
(see [`docs/custom-kinematics-solver.md`](docs/custom-kinematics-solver.md)).

### Building

Run `./build.sh` from the repository root after a change. It picks the right
`colcon` command for the platform and builds in Release mode:

```bash
./build.sh                                  # the whole workspace
./build.sh --packages-select fic_studio     # extra arguments go to colcon
```

- **Linux:** `colcon build --symlink-install`, with the number of jobs capped
  to the memory.
- **macOS:** activates the `fic_ros` environment if needed. It does not use
  `--symlink-install`, which setuptools 80 and later no longer support.

---

## Running

```bash
source ~/FIC_Interface/fic_ws/install/setup.bash

ros2 run fic_studio fic_studio_pro   # FIC Control Studio
ros2 run fic_studio fic_dev          # Dev Console: a smaller GUI for a second PC
```

The installer also adds both to the desktop's application menu.

On macOS (zsh) the installer puts `FIC Control Studio.app` and `FIC Dev Console.app`
in `~/Applications`. From a terminal:

```zsh
conda activate fic_ros
source ~/FIC_Interface/fic_ws/install/setup.zsh

ros2 run fic_studio fic_studio_pro --web-ui
```

### Simulation

1. Choose the robot in the toolbar.
2. Set **Target** to **Simulation** and click **▶ Start**. The MuJoCo simulator
   starts, then the FIC controller.
3. Choose a control mode and use the joint sliders or the Cartesian inputs.

### Hardware

1. Choose the robot that matches the arm.
2. Set **Target** to **Hardware** and click **▶ Start**. This starts the robot
   driver and the FIC controller.
3. Choose a control mode and command the robot.

> **Warning:** make sure the arm is powered on, the area is clear and the
> robot chosen matches the hardware before sending commands.

### Several robots

Each robot needs its own `fic_hardware` process with a unique `station_id`.
One Studio controls up to four robots with **+ Add Robot**; each card has a
**Station** selector for the robot it drives. The **Teleop** checkbox in the
toolbar locks the local controls so that a robot takes commands only from
outside (another Studio, a teleoperation partner or a gamepad).

### Adding a robot

1. Open **Robot Configuration** in the Studio's sidebar.
2. Drop the robot's `.urdf` (or `.xacro`, its description-package folder, or a
   `.zip` of that folder) on **Add a robot**. The URDF is required; add the
   robot's MuJoCo model (`.xml`) with it if you have one.
3. Check the review screen — joints, limits, torques, end effector, home pose
   and gripper are filled in from the robot's own files — and click
   **Create robot**, then **Try in simulation**.

The Studio writes the robot into `menagerie/model/<RobotID>/` (URDF, meshes,
a MuJoCo model with FIC's motors and sensors) and registers it; nothing needs
rebuilding. Closed-loop (parallel) robots are supported: the loops come from
the MuJoCo model, from markup in the URDF, or from the review screen. What a
URDF needs, and what the importer does not cover (sliding drives, legged
robots), is in `fic_docs/pages/adding-a-robot.html`.

**Its driver:** on the robot's **Hardware** tab in Robot
Configuration, drop its driver library (the vendor SDK with FIC's bridge
program, as a folder or `.zip`) on **Add the driver library**. It is copied
into `robot_library/`, built, and the driver command is pointed at the program
the build made. Then check the torque sign and limits and tick **This robot has
a hardware driver** — the hardware node controls a robot only once that is
ticked. See `fic_docs/pages/adding-a-robot.html` → *Real robot*.

### Web remote (phone or browser)

While the Studio runs it also serves its controls to browsers on the network,
laid out for a phone:

1. The log, the **WEB** indicator in the status bar and **Settings ▸ Web
   Remote** show the address, e.g. `https://192.168.1.50:8088/`.
2. Open it on a phone on the same network. The browser warns once about the
   certificate: the Studio signs its own.
3. Sign in. The first Studio on a network makes an `admin` account and prints
   its password in the log.
4. Tap **Take control** to command the robot; **STOP** works from any browser.

One browser commands at a time, and control drops as soon as the phone locks
or the page is hidden. Sign-in and HTTPS are on by default (Settings ▸ Web
Remote). See `fic_docs/pages/web-remote.html`.

---

## References

### FIC Interface

- A. Takriti, M. Sadeq, Q. Rouxel, D. Cafolla, M. Mistry and C. Tiseo, “Whole-Arm
  Haptic Compliance via a Unified Fractal Impedance Control Interface,” in
  preparation, 2026.

### General

- Tiseo, C., Merkt, W., Wolfslag, W. et al. Safe and compliant control of
  redundant robots using superimposition of passive task-space controllers.
  Nonlinear Dyn (2023). doi:[10.1007/s11071-023-09045-x](https://doi.org/10.1007/s11071-023-09045-x)
- R. Wen, Q. Rouxel, M. Mistry, Z. Li and C. Tiseo, “Collaborative Bimanual
  Manipulation Using Optimal Motion Adaptation and Interaction Control:
  Retargeting Human Commands to Feasible Robot Control References,” in IEEE
  Robotics & Automation Magazine, doi: [10.1109/MRA.2023.3270222](https://doi.org/10.1109/MRA.2023.3270222).
- Tiseo C, Charitos S., and Mistry M. “Harmonic Passive Motion Paradigm,”
  bioRxiv, 2021, doi: [10.1101/2021.07.06.451400](https://doi.org/10.1101/2021.07.06.451400).

### Planner

- C. Tiseo, V. Ivan, W. Merkt, I. Havoutis, M. Mistry and S. Vijayakumar, “A
  Passive Navigation Planning Algorithm for Collision-free Control of Mobile
  Robots,” 2021 IEEE International Conference on Robotics and Automation
  (ICRA), 2021, pp. 8223-8229, doi: [10.1109/ICRA48506.2021.9561377](https://doi.org/10.1109/ICRA48506.2021.9561377).
- C. Tiseo, Q. Rouxel, Z. Li and M. Mistry, “Robust Impedance Control for
  Dexterous Interaction Using Fractal Impedance Controller with
  IK-Optimisation,” 2022 International Conference on Robotics and Automation
  (ICRA), 2022, pp. 840-846, doi: [10.1109/ICRA46639.2022.9812013](https://doi.org/10.1109/ICRA46639.2022.9812013).

### Admittance

- Tiseo C, Merkt W, Babarahmati K K, Wolfslag W, Havoutis I, Vijayakumar S and
  Mistry M, “HapFIC: An Adaptive Force/Position Controller for Safe
  Environment Interaction in Articulated Systems,” in IEEE Transactions on
  Neural Systems and Rehabilitation Engineering,
  doi: [10.1109/TNSRE.2021.3098062](https://doi.org/10.1109/TNSRE.2021.3098062).

### Quaternion

- C. Tiseo, S. R. Charitos and M. Mistry, “Exploiting Spherical Projections To
  Generate Human-Like Wrist Pointing Movements,” 2021 43rd Annual International
  Conference of the IEEE Engineering in Medicine & Biology Society (EMBC), 2021,
  pp. 6192-6197, doi: [10.1109/EMBC46164.2021.9629550](https://doi.org/10.1109/EMBC46164.2021.9629550).

## Contributors

**Main contributors**

- Adnan Takriti (University of Sussex)
- Carlo Tiseo (University of Sussex)

**Supporting contributors**

- Mahmoud Sadeq
- Quentin Rouxel
- Daniele Cafolla
- Michael Mistry

## License

FIC Interface is released under the MIT License (see [`LICENSE`](LICENSE)).
The third-party components in `robot_library/` and `fic_ws/src/` remain under
their own licences; see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).

## Citing

If you use FIC Interface in your work, please cite the paper that describes it
(*Whole-Arm Haptic Compliance via a Unified Fractal Impedance Control
Interface*, under References), the software, and the other papers above that
your work relies on. The software citation is in
[`CITATION.cff`](CITATION.cff) (GitHub's **Cite this repository** button gives
it as BibTeX or APA).

## Questions and bug reports

Open an issue on this repository.
