# Installing FIC Control Studio

How to install FIC Control Studio, the GUI for robot arms under Fractal
Impedance Control, on Ubuntu. For macOS see [`macos/README.md`](macos/README.md);
for Docker see [`../docker/README.md`](../docker/README.md).

---

## Prerequisites

- **Ubuntu 22.04 or 24.04**, **amd64 or arm64** (Arduino UNO Q / VENTUNO Q included)
- Nothing else: the installer installs ROS 2 (Humble on 22.04, Jazzy on 24.04)
  if it is missing, then every dependency, builds the workspace and registers
  the apps. Run `./install.sh` from a checkout, or `bash install.sh` on its own
  to clone the repository first.

`./install.sh --dry-run` prints what it detected and what it would do, and
changes nothing. `bash fic_install/detect-system.sh` prints the detection alone.

What the detection changes (`fic_install/detect-system.sh`, used by the
installer, `build.sh` and the launchers):

| | amd64 | arm64 |
|---|---|---|
| Kinova vendor driver (x86-64 Kortex library) | built | skipped (the Kortex library is x86-64 only) |
| pyrealsense2 / mediapipe (pip) | installed | skipped (no wheels) |
| Library paths (tinyxml2 CMake config) | `/usr/lib/x86_64-linux-gnu` | `/usr/lib/aarch64-linux-gnu` |

Studio's Python stack (PyQt5, QtWebEngine, pyqtgraph, numpy, scipy, matplotlib, yaml) comes from apt;
only `mujoco` comes from pip. Parallel build jobs are capped to RAM/2 GB, apt
waits for the package lock (unattended-upgrades), and root is obtained with
`sudo` in a terminal or `pkexec` when double-clicked.

> The installer resolves all paths relative to the location of the installer
> script, so the repo can live at any path. Set the `FIC_ROOT` environment
> variable to override the auto-detected location if needed. The installer
> handles all other dependencies and the workspace build automatically.

---

## Headless use (e.g. an Arduino VENTUNO Q next to the robot)

- `bash fic_install/install-autostart.sh --now` — Studio starts at boot with no
  monitor or login; use it through its web remote (`https://<board>:8088`).
- `bash fic_install/setup-hotspot.sh` — the board broadcasts its own Wi-Fi
  (starts at next boot; `--now` / `--off`); web remote at `https://10.42.0.1:8088`.
- `bash fic_install/web-admin-password.sh` — set the web remote admin password.

## Installation

### 1. Run the installer

**Option A — Double-click:**

Right-click `Install_FIC_Studio.desktop` in the `FIC_Interface` folder → Properties →
Permissions → tick *Allow executing as program*, then double-click it.

**Option B — Terminal:**

```bash
cd ~/FIC_Interface
./Install_FIC_Studio.desktop            # or: bash install.sh, for any platform
```

**Options** (all optional; the installer asks for anything not given):

| Flag | Does |
|---|---|
| `--yes`, `-y` | accept every default, ask nothing — for terminals and scripts |
| `--headless` | this machine shows **no window**; use the Studio from a browser elsewhere |
| `--with-gui` | this machine shows a window (the default) |
| `--dry-run` | report what would be installed, change nothing |

`--yes` on its own keeps the window, so existing scripted installs behave as before.
`--headless` and `--with-gui` set the choice without a prompt, so a fleet can be installed
without anyone clicking.

### 2. A window, or headless

The installer asks how this machine should show the Studio, because it is a property of the
machine rather than of a session: a robot with no monitor should never draw an interface, and
one on a desk always should. **Re-run the installer to change it.**

- **Window** — the Studio opens its interface in its own Qt window, as normal.
- **Headless** — it draws nothing at all: no Qt window, no QtWebEngine, no WebGL. It still
  runs its ROS node, joins the fleet and serves the web interface, so you work on it from a
  browser on another PC.

The answer is stored in `~/.config/fic-studio/interface.conf` as `FIC_HEADLESS=1` or `0`, and
`fic_install/fic-control-studio.sh` turns it into `QT_QPA_PLATFORM=offscreen` — the mode the
Studio already recognises (`qt_webengine.HEADLESS_PLATFORMS`). Setting `QT_QPA_PLATFORM`
yourself still wins, so nothing here overrides a deliberate choice.

**Pick headless for any machine without a monitor.** Without a screen there is no GPU, so the
Studio's window would be drawn in software on the CPU of a machine whose other job is a 1 kHz
control loop. Headless, it draws nothing and the web remote still answers; a browser on another
PC shows the interface.

### 3. What the installer does

Every package, download and file it adds, per platform: [WHAT-GETS-INSTALLED.md](WHAT-GETS-INSTALLED.md).

1. **Checks for ROS 2** and detects your distro automatically
2. **Scans for missing dependencies** across three categories:
   - **System (apt):** cmake, build-essential, colcon, Eigen, TinyXML2, yaml-cpp, Boost, GLFW (viewer fallback)
   - **ROS 2 (apt):** rclcpp, std-msgs, geometry-msgs, sensor-msgs, nav-msgs, pinocchio
   - **Python (apt):** PyQt5, QtWebEngine, pyqtgraph, numpy, scipy, matplotlib, yaml, tkinter
   - **Python (pip):** mujoco
3. **Prompts to install** anything missing (with a graphical password prompt for apt packages)
4. **Asks window or headless**, unless `--headless`/`--with-gui`/`--yes` already answered it
5. **Builds the ROS 2 workspace** (`./build.sh`, Release) with a progress bar showing each package
6. **Registers the app** in your Ubuntu launcher
7. **Checks that the web interface imports.** A pip PyQt5 in `~/.local` can shadow the system Qt
   and leave QtWebEngine unimportable; if so the installer says how to fix it.

---

## Running

- **From app launcher:** Press Super and search for **FIC**
- **From terminal:**
  ```bash
  bash ~/FIC_Interface/fic_install/fic-control-studio.sh
  ```
- **Direct ROS 2 command** (if workspace is already sourced):
  ```bash
  ros2 run fic_studio fic_studio_pro
  ```

---

## Uninstalling

The full list, including the settings files, the pip packages and macOS/Docker:
[WHAT-GETS-INSTALLED.md, Removing it](WHAT-GETS-INSTALLED.md#removing-it).

```bash
rm ~/.local/share/applications/fic-control-studio.desktop ~/.local/share/applications/fic-dev-console.desktop
update-desktop-database ~/.local/share/applications
bash fic_install/install-autostart.sh --remove    # only if you set up autostart
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| App doesn't appear in launcher | Log out and back in, or run `update-desktop-database ~/.local/share/applications` |
| Build fails during install | Check `/tmp/fic-install.log` — the installer will show the errors in a scrollable dialog |
| Build fails on launch | Run it by hand: `cd ~/FIC_Interface && ./build.sh` |
| Dependency install failed | Check `/tmp/fic-install.log` for details |