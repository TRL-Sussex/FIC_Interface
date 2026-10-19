# What the installer puts on your computer

FIC Control Studio needs ROS 2, a physics simulator and a set of libraries. This page lists
everything each way of installing it adds to or changes on your computer, so you can decide
before you run anything. The installer has a dry run that only shows the plan, on Linux and macOS:

```bash
bash install.sh --dry-run
```

| Route | Admin rights | Installs system-wide | Typical download |
|---|---|---|---|
| [Linux, `bash install.sh`](#linux-bash-installsh) | yes (`sudo`, or a password dialog) | apt packages (ROS 2 and libraries) | several hundred MB |
| [Linux by hand, `install-deps.sh`](#linux-by-hand-sudo-bash-fic_installinstall-depssh) | yes (run with `sudo`) | apt packages, possibly tinyxml2 in `/usr/local` | several hundred MB |
| [macOS, `bash install.sh`](#macos-bash-installsh) | no | nothing outside your home folder | a few GB |
| [Docker, `docker/fic-studio.sh`](#docker-dockerfic-studiosh-and-fic-studiocmd) | no (Docker must already be installed) | nothing outside Docker | about 1 GB |

Everything is built **inside the folder you cloned** (`fic_ws/build`, `fic_ws/install`,
`fic_ws/log`). None of the routes edits your shell profile (`~/.bashrc`, `~/.zshrc`), adds
services or anything that starts at login, changes groups or sudoers, or installs udev rules.

---

## Linux: `bash install.sh`

`install.sh` runs `Install_FIC_Studio.desktop` (a bash script). It asks before installing
anything; `--yes` answers every question with yes or the first choice, `--dry-run` prints the
system report and the lists below and exits. Supported: Ubuntu 22.04 (ROS 2 Humble) and 24.04
(ROS 2 Jazzy), amd64 and arm64.

**Admin rights** (`sudo` in a terminal, a password dialog when double-clicked) are used only for
apt and, when needed, the tinyxml2 fix below. pip installs go into your home folder (`--user`).

1. **Python for the installer's dialogs** (only what is missing): `python3`, `python3-pip`,
   `python3-tk`.
2. **ROS 2**, only if none is installed under `/opt/ros` (it asks first):
   - apt: `software-properties-common curl gnupg locales`, the Ubuntu `universe` component, the
     `en_US.UTF-8` locale;
   - the ROS apt source: the `ros2-apt-source` package from
     github.com/ros-infrastructure/ros-apt-source (or, if GitHub's API cannot be reached, the
     key from raw.githubusercontent.com/ros/rosdistro in `/usr/share/keyrings/ros-archive-keyring.gpg`
     and `/etc/apt/sources.list.d/ros2.list` pointing at packages.ros.org);
   - `ros-<distro>-ros-base`, `ros-dev-tools`.
3. **apt packages** (only the missing ones): `cmake build-essential python3-colcon-common-extensions
   git curl libeigen3-dev libtinyxml2-dev libyaml-cpp-dev libboost-dev libboost-serialization-dev
   libboost-system-dev libboost-filesystem-dev libglfw3 python3-pip python3-pyqt5
   python3-pyqt5.qtsvg python3-pyqt5.qtwebengine python3-yaml python3-numpy python3-scipy
   python3-cryptography python3-pyqtgraph python3-matplotlib python3-tk ffmpeg`, `adb` (best-effort: where apt cannot
   install it, as on the Arduino board, `fic_install/adb-private.sh` unpacks it on its own), and
   `ros-<distro>-rclcpp -std-msgs -geometry-msgs -sensor-msgs -nav-msgs -std-srvs
   -urdfdom-headers`.
4. **Pinocchio** (`ros-<distro>-pinocchio`, the kinematics library; optional, and an older one is
   upgraded).
5. **pip, into `~/.local`**: `mujoco` and `numpy<2`, with `--break-system-packages` (Ubuntu
   24.04 refuses `--user` installs otherwise).
6. **Newton GPU simulator** (optional, offered only on NVIDIA machines, about 141 MB):
   `newton[sim,importers]`, `pyglet`, into `~/.local`. With `--yes` it is installed.
7. **tinyxml2 fix**, only if a source-built tinyxml2 conflicts with apt's: one CMake file of the
   source build is deleted and `ldconfig` is run.
8. **Builds**, inside the repository: the workspace, the Kinova driver (amd64) and the MCU host
   tools. Nothing is downloaded by the builds.

Files written outside the repository:

| File | What |
|---|---|
| `~/.local/share/applications/fic-control-studio.desktop`, `fic-dev-console.desktop` | the app-menu launchers (which ones depends on your choice) |
| `~/.config/fic-studio/interface.conf` | `FIC_HEADLESS=0` or `1` (window or web-only) |
| `~/.fic_studio_install_hints.yaml` | whether Newton is available |
| `/tmp/fic-install.log` | the installer's log when double-clicked |

The Studio itself later creates `~/.robot_selector_gui_settings.yaml` (its settings),
`~/.fic_studio/` (web-remote accounts and its HTTPS certificate) and, if you save a DDS setting,
`~/cyclonedds.xml`.

**Not run by the installer** (separate scripts in `fic_install/`, only if you run them):
`install-autostart.sh` (a user service that starts the Studio at boot, and `loginctl
enable-linger`), `setup-hotspot.sh` (a Wi-Fi hotspot that comes up at every boot),
`web-admin-password.sh`, `install-desktop-entry.sh`.

## Linux by hand: `sudo bash fic_install/install-deps.sh`

For a machine that already has ROS 2 (it does not add the ROS apt source).

- apt: `libeigen3-dev libyaml-cpp-dev libglfw3 libtinyxml2-dev python3-dev python3-numpy
  python3-pip python3-pyqt5 python3-pyqt5.qtsvg python3-pyqt5.qtwebengine python3-yaml
  python3-scipy python3-cryptography python3-pyqtgraph python3-opencv python3-matplotlib
  python3-tk git cmake build-essential pkg-config lsb-release curl ffmpeg`, `adb` best-effort (else
  `fic_install/adb-private.sh`), and `ros-dev-tools`,
  `ros-<distro>-std-srvs -urdfdom-headers -nav-msgs -sensor-msgs`, optionally
  `ros-<distro>-pinocchio`.
- If no packaged tinyxml2 CMake config exists: tinyxml2 is cloned from github.com/leethomason
  and installed into `/usr/local`.
- pip, as your user: `mujoco numpy<2`; on amd64 also `pyrealsense2 mediapipe` when the RealSense,
  gaze or battery-sorter extensions are present.
- **Newton is installed without asking** on NVIDIA machines.
- Builds the Kinova driver and the MCU host tools (logs in `/tmp`). It does not build the
  workspace or add launchers: run `./build.sh` afterwards.

## macOS: `bash install.sh`

Runs `fic_install/macos/install-fic-studio-macos.sh`. No admin rights; Homebrew is used if you
have it but is not required. Needs about 10 GB free. `--dry-run` shows the plan.

- **Miniforge** (conda/mamba), if you have no conda: through Homebrew (`brew install --cask
  miniforge`) or downloaded from github.com/conda-forge/miniforge into `~/miniforge3`. `conda
  init` is not run, so your shell profile is not changed.
- **A conda environment `fic_ros`** (about 4.5 GB) from the `conda-forge` and
  `robostack-staging` channels: ROS 2 Humble desktop, compilers, cmake, ninja, colcon, eigen,
  tinyxml2, yaml-cpp, pinocchio, PyQt, pyqtgraph, numpy, PyYAML, scipy, cryptography,
  matplotlib, mujoco and a few build tools.
- **Builds** the workspace inside the repository and writes `.command` launchers into
  `fic_install/macos/`.
- **`~/Applications/FIC Control Studio.app` and `FIC Dev Console.app`** (rebuilt on every run,
  signed locally, registered so Launchpad and Spotlight find them). They log to
  `/tmp/fic_studio_pro.log` and `/tmp/fic_dev.log`.

Nothing is installed outside your home folder, and nothing starts at login.

## Docker: `docker/fic-studio.sh` (and `fic-studio.cmd`)

Docker itself is not installed; the script tells you how if it is missing, and starts Docker
Desktop if it is installed but not running.

- Pulls the image `ghcr.io/trl-sussex/fic_interface:latest` (about 1 GB).
- Creates a container `fic-studio`, a Docker volume `fic-studio-home` (the Studio's home
  folder) and a shared folder `~/FIC_Studio` (Windows: `%USERPROFILE%\FIC_Studio`).
- Listens on `127.0.0.1:6080` (the desktop in your browser) and `127.0.0.1:8088` (the web
  interface); with `--lan`, port 8088 is open to your network.
- `--native` (Linux) lets the container use your screen with `xhost +SI:localuser:<you>`,
  which is not revoked afterwards (`xhost -SI:localuser:$USER` does that).
- The container keeps running until `docker/fic-studio.sh stop`; it does not start at boot.

## Removing it

- **Linux:** delete the launchers and the settings, then the repository:
  ```bash
  rm -f ~/.local/share/applications/fic-control-studio.desktop ~/.local/share/applications/fic-dev-console.desktop
  update-desktop-database ~/.local/share/applications
  rm -rf ~/.config/fic-studio ~/.fic_studio_install_hints.yaml ~/.robot_selector_gui_settings.yaml ~/.fic_studio
  rm -rf ~/FIC_Interface
  ```
  The apt packages, ROS 2 (and its apt source) and the pip packages in `~/.local` stay; remove
  them with `apt` and `pip uninstall` if nothing else uses them. If you ran
  `install-autostart.sh`, run `fic_install/install-autostart.sh --remove` first (and
  `loginctl disable-linger` if you want); for the hotspot, `setup-hotspot.sh --remove`.
- **macOS:** `rm -rf ~/Applications/FIC\ Control\ Studio.app ~/Applications/FIC\ Dev\ Console.app`,
  `conda env remove -n fic_ros`, then the repository; Miniforge is `~/miniforge3` (or
  `brew uninstall --cask miniforge`).
- **Docker:** `docker/fic-studio.sh uninstall` removes the container, the volume and the image;
  `~/FIC_Studio` is kept.
