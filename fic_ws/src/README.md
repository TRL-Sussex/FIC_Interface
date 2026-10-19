# fic_ws/src

The ROS 2 packages of FIC Interface. Build them with `./build.sh` from the
repository root (see the top-level README).

| Package | What it is |
|---|---|
| `communication` | Messages and services the other packages share |
| `fic_control` | The FIC controller: the node and its joint inner loop, for simulated and real robots |
| `fic_studio` | FIC Control Studio (desktop and web interface) |
| `mujoco_native` | The MuJoCo simulator (CPU) |
| `mujoco_newton` | An optional GPU simulator (needs an NVIDIA GPU) |
| `menagerie` | Robot models (URDF and MuJoCo scenes) |
| `fic_contact` | Contact estimation from the joint torques |
| `fic_locomotion` | Gait controller for legged robots |
