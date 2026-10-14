# Third-party notices

FIC Interface is released under the MIT License (see [LICENSE](LICENSE)). It
bundles the third-party components below, which remain under their own
licences. Each component's copyright notice and licence text is kept in the
file listed here.

## Robot drivers

| Component | Location | Licence | Copyright | Licence text |
|---|---|---|---|---|
| libfranka | `robot_library/libfranka/` | Apache 2.0 | Franka Robotics GmbH | `robot_library/libfranka/LICENSE`, `NOTICE` |
| Kinova Kortex API | `robot_library/Kinova/` | BSD 3-Clause | Kinova inc. | `robot_library/Kinova/LICENSE` |
| Protocol Buffers (headers bundled with Kortex) | `robot_library/Kinova/api_cpp/examples/kortex_api/` | BSD 3-Clause | Google Inc. | `robot_library/Kinova/LICENSE` and each header |
| cxxopts | `robot_library/Kinova/api_cpp/examples/thirdParty/cxxopts/` | MIT | Jarryd Beck | `.../cxxopts/LICENSE` |

## Simulation

The simulators (`fic_ws/src/mujoco_native`, `fic_ws/src/mujoco_newton`) are FIC
code; MuJoCo itself comes from the `mujoco` Python wheel (Apache 2.0, Google
DeepMind) and is not vendored. 

## ROS 2 package template

| Component | Location | Licence | Copyright | Licence text |
|---|---|---|---|---|
| Lint tests (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`) | `fic_ws/src/fic_studio/test/`, `fic_ws/src/fic_locomotion/test/` | Apache 2.0 | Open Source Robotics Foundation, Inc. | Header of each file |

## Web interface

| Component | Location | Licence | Copyright | Licence text |
|---|---|---|---|---|
| three.js | `fic_ws/src/fic_studio/fic_studio/web_remote/static/three.module.js` | MIT | Three.js Authors | Header of the file |

The Docker image also downloads noVNC (MPL 2.0) and installs websockify (LGPL
3.0) when it is built; neither is part of this repository.

## Robot models

| Model | Location | Licence | Copyright | Licence text |
|---|---|---|---|---|
| Franka FR3 | `fic_ws/src/menagerie/model/Franka_FR3/` | Apache 2.0 | Franka Robotics GmbH | `.../Franka_FR3/xml/LICENSE` |
| KUKA iiwa 14 | `fic_ws/src/menagerie/model/IIWA_14/` | BSD 3-Clause | Robot Locomotion Group @ CSAIL (Drake) | `.../IIWA_14/XML/LICENSE` |
| Kinova Gen3 7-DOF | `fic_ws/src/menagerie/model/KinovaGen3_7DOF/` | BSD 3-Clause | Kinova inc. | `.../KinovaGen3_7DOF/LICENSE` |
| Pendulum | `fic_ws/src/menagerie/model/Pendulum/` | Apache 2.0 | The dm_control Authors | Header in the model files; full text in `.../Pendulum/LICENSE` |
