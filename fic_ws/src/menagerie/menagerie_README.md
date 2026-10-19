# menagerie — Robot Model Library

> **Package:** `menagerie`
> **Type:** ament_cmake (data-only — no executables)

The `menagerie` package stores all robot model files (URDF for Pinocchio, MuJoCo XML for simulation) used by `fic_control`, `fic_control`, and the MuJoCo simulator. It is installed as a ROS 2 package so other packages can declare it as a dependency and resolve model paths at build time.

---

## Directory Structure

```
menagerie/
├── CMakeLists.txt
├── package.xml
└── model/
    ├── Franka_FR3/
    │   ├── URDF/
    │   │   └── Franka_FR3.urdf
    │   └── xml/
    │       ├── Franka_FR3.xml        # MuJoCo model
    │       ├── scene.xml             # MuJoCo scene (floor, lighting)
    │       └── assets/               # .obj and .stl mesh files
    │
    ├── KinovaGen3_7DOF/
    │   ├── URDF/
    │   │   └── KinovaGen3_7DOF.urdf
    │   └── xml/
    │       ├── KinovaGen3_7DOF.xml
    │       ├── gen3/                 # .stl link meshes
    │       └── robotiq/             # Gripper meshes
    │
    ├── IIWA_14/
    │   ├── URDF/                    # .xacro source files
    │   └── XML/
    │       ├── iiwa14.xml
    │       ├── scene.xml
    │       └── assets/              # .obj link meshes
    │
    └── Pendulum/
        ├── URDF/
        │   └── Pendulum.urdf
        └── xml/
            ├── pendulum.xml
            └── common/              # Shared materials, skybox
```

---

## How Paths Are Resolved

Both C++ nodes resolve model paths relative to the workspace working directory:

```
URDF:   <CWD>/src/menagerie/model/<RobotID>/URDF/<RobotID>.urdf
MuJoCo: <CWD>/src/menagerie/model/<RobotID>/xml/scene.xml   (IIWA_14: XML/scene.xml)
```

These paths are stored in `robots_config.yaml` as relative strings like `menagerie/model/Franka_FR3/URDF/Franka_FR3.urdf` and resolved by `fic::resolveUrdfPath()` and `fic::resolveMujocoPath()`.

---

## Included Robots

| Robot | Joints | URDF | MuJoCo XML | Gripper |
|-------|--------|------|------------|---------|
| **Franka FR3** | 7 | Yes | Yes | Franka Hand |
| **Kinova Gen3 7DOF** | 7 | Yes | Yes | Robotiq 2F-85 |
| **KUKA iiwa 14** | 7 | `.xacro` sources only | Yes | None |
| **Pendulum** | 1 | Yes | Yes | None |

---

## Adding a New Robot

### From its URDF (the quick way)

Open **FIC Control Studio → Robot Configuration** and drop the robot's `.urdf`
(or `.xacro`, its description-package folder, or a `.zip` of that folder) on
**Add a robot** — with the robot's own MuJoCo model (`.xml`) if it has one,
which is then used for simulation. Closed-loop (parallel) robots work too:
their loops come from the MuJoCo model's `connect` constraints, from Drake /
onshape-to-robot markup in the URDF, or from the review screen. One review screen later the Studio has written everything in
this folder's layout — URDF, meshes, a MuJoCo model with FIC's motors and
sensors, `scene.xml`, an `import_report.md` — registered the robot in both
robot lists and `config.xml`, and can start it in simulation. Nothing needs
rebuilding: the Studio passes the scene to the simulator as `model_file:=`,
and the controllers read the URDF and `robots_config.yaml` from the source
tree.

What a URDF needs, what is filled in automatically (limits, torques, speeds,
end effector, home pose from a MoveIt SRDF, gripper, loops and driven
joints…) and what is not covered (sliding drives, legged robots) are in the
docs:
`fic_docs/pages/adding-a-robot.html` (**Adding a Robot** in the docs sidebar).

### By hand

For what the importer does not cover, such as a robot with sliding drives:

1. **Create the model directory**

   ```bash
   cd fic_ws/src/menagerie/model/
   mkdir -p MyRobot_7DOF/URDF MyRobot_7DOF/xml/assets
   ```

   Follow the naming convention: `<RobotID>/URDF/<RobotID>.urdf`, `<RobotID>/xml/<RobotID>.xml`
   and `<RobotID>/xml/scene.xml` (copy `_template/xml/scene.xml`).

2. **Add the model files**

   - `URDF/MyRobot_7DOF.urdf` — joint names as in the config, with an end-effector link
   - `xml/MyRobot_7DOF.xml` — the MuJoCo robot: a `<motor>` named `torq_j1`…`torq_jN` per
     joint in config order, `force_torque_site` on the end effector with `force_sensor` and
     `torque_sensor`, `implicitfast` at 1 ms, and joint damping (the shipped arms use 45)
   - `xml/assets/` — the meshes (`.stl`, `.obj`)

   A continuous joint in the URDF is best written as `type="revolute"` with a `<limit>` that
   has `effort` and `velocity` but no `lower`/`upper` — FIC's convention (see
   `KinovaGen3_7DOF.urdf`). Pinocchio then gives it one coordinate, which the controllers need.

3. **Register it** — on the Robot Configuration page click **+ Blank robot**, enter the ID,
   fill in the tabs and **Save**. Save writes `fic_studio_robots.yaml` and
   `fic_library/config/robots_config.yaml` and gives the robot default tuning in `config.xml`.

No rebuild is needed with `colcon build --symlink-install` (this workspace). A copy install
needs `colcon build --packages-select menagerie fic_control` for new model files and the
new `config.xml` entry to reach the install space.

---

## Notes

- **URDF is used by Pinocchio** (in both `fic_control` and `fic_control`) for forward kinematics, Jacobians, inverse kinematics, and gravity compensation (RNEA).
- **MuJoCo XML is used only by the MuJoCo simulator** for physics stepping and 3D rendering.
- **Both models must represent the same robot** with consistent joint ordering, link names, and kinematics.
- **IIWA_14 ships only `.xacro` sources.** `robots_config.yaml` expects `IIWA_14/URDF/IIWA_14.urdf`: generate it with `xacro` before running the FIC controller on it. The other robots ship compiled URDF files.
- **Mesh paths in URDF/MuJoCo XML** should be relative to the model's own directory (e.g. `assets/link0.obj`).