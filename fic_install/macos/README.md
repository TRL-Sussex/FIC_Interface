# FIC Studio on macOS (Apple Silicon)

Runs FIC Studio simulation natively on an Apple Silicon Mac. ROS 2 comes from
[RoboStack](https://robostack.github.io/) (conda-forge), and simulation uses the
same CPU MuJoCo simulator as Ubuntu (`mujoco_native`, on the official `mujoco`
wheel), so no Linux binaries or CUDA are needed.

> **Scope:** simulation and the Studio GUI. The real-robot drivers (Franka,
> Kinova) are Linux-only and are not built here.

## Requirements

- macOS 13+ on Apple Silicon (arm64)
- [Homebrew](https://brew.sh) is optional (used for Miniforge if present)
- ~10 GB free disk (ROS 2 + toolchain)
- **Repo must NOT live under `~/Documents`, `~/Desktop`, or `~/Downloads`.**
  macOS privacy protection (TCC) blocks *app-bundle* launches (Finder/Launchpad)
  from reading files there, so the double-clickable `.app` would open and
  immediately quit. Keep the repo elsewhere, e.g. `~/FIC_Interface`. The
  installer refuses to run from a protected folder and tells you how to move it.
  (Moving an already-built repo needs a clean rebuild — colcon bakes absolute
  paths; the installer handles that.)

## Install (one command)

```bash
zsh fic_install/macos/install-fic-studio-macos.sh
```

(`bash install.sh` from the repository root calls this on a Mac.)

The installer is safe to re-run and never uses `sudo`. It:

1. Uses Homebrew if it is installed; it is not required.
2. Installs **Miniforge** (conda/mamba) if absent — via Homebrew, or else
   straight into `~/miniforge3` from its official installer.
3. Creates the **`fic_ros`** conda env with **ROS 2 Humble** + all deps
   (pinocchio, mujoco, PyQt5, compilers, colcon…).
4. Builds the workspace (`communication`, `menagerie`, `fic_simulation`,
   `fic_studio`, `mujoco_native`, …) — the whole workspace.
5. Builds real **`.app` bundles** — *FIC Control Studio.app* and *FIC Dev
   Console.app* — in `~/Applications`, with the FIC icon rendered from the SVG,
   so they show up in Launchpad, Spotlight and the Dock. Also drops `.command`
   launchers as a terminal-visible fallback.

Override the repo location with `FIC_ROOT=/path/to/FIC_Interface zsh …` if you
run the script from elsewhere.

## Distributable installer (`.dmg`)

To hand FIC Studio to someone who doesn't have the repo, build a downloadable
disk image:

```bash
zsh fic_install/macos/build-macos-dmg.sh
# -> fic_install/macos/dist/FIC-Studio-Installer.dmg
```

The `.dmg` bundles the **repo source** (not the ~4.5 GB conda env — deps are
still downloaded at install time). The recipient mounts it and double-clicks
**`Install FIC Studio.command`**, which:

1. Asks where to install (default `~/FIC_Interface`) and **refuses** protected
   folders (`~/Documents`/`~/Desktop`/`~/Downloads`), since the app couldn't
   launch from there.
2. Copies the source to that writable location (a DMG is read-only, and the
   workspace has to be built).
3. Runs `install-fic-studio-macos.sh` in a Terminal, so the multi-GB download
   shows live progress.

Downloaded copies are Gatekeeper-quarantined, so the first launch needs
**right-click → Open**.

## Launch

Open **FIC Control Studio** from Launchpad or Spotlight (`⌘Space` → "FIC"), or
double-click `~/Applications/FIC Control Studio.app` — it opens the web interface in the Studio's own QtWebEngine window
directly, no Terminal window. Drag it to the Dock to keep it there. For a
terminal-visible run (handy for logs), double-click
**`FIC Control Studio.command`** (or **`FIC Control Studio (Classic).command`**
for the classic Qt UI), or from a terminal:

```bash
conda activate fic_ros
cd fic_ws && source install/setup.zsh   # zsh: use setup.zsh, not setup.bash
ros2 run fic_studio fic_studio_pro --web-ui   # --classic for the Qt UI; or: fic_dev
```

In the GUI: pick **KinovaGen3_7DOF** or **Franka_FR3**, press **▶ Start** (a
MuJoCo window opens, then the controller starts), choose a control mode, and
drive the arm with the joint sliders or Cartesian inputs. On macOS the Settings
→ Simulation backend shows **MuJoCo (CPU)**, selected automatically.

## What differs from Ubuntu

| Concern | Ubuntu | macOS (this port) |
|---|---|---|
| ROS 2 | apt under `/opt/ros` | RoboStack conda env `fic_ros` |
| Source env | `source /opt/ros/$DISTRO/setup.bash` | `conda activate fic_ros` |
| Workspace env | `source install/setup.bash` | `source install/setup.zsh` (zsh default) |
| Simulator | `mujoco_native` (CPU), optional `newton` (CUDA) | `mujoco_native` (CPU, native arm64) |
| System libs | `apt install …` | Homebrew / conda-forge |
| Launcher | `.desktop` file | `.app` bundle in `~/Applications` (+ `.command` fallback) |
| CPU count in builds | `nproc` | `sysctl -n hw.ncpu` |
| Fonts | Segoe UI / Consolas | Menlo / SF Pro (auto) |

## Troubleshooting

- **`command not found: colcon` / `ros2`** — you didn't activate the env. Run
  `conda activate fic_ros` first (the `.command` launcher does this for you).
- **`mujoco_native` package not found** — source the workspace with
  `source install/setup.zsh` (zsh), not `setup.bash`.
- **`communication` fails to build referencing `numpy/_core/include`** — you have
  `numpy<2`. RoboStack ros-humble needs **numpy≥2**: `mamba install -n fic_ros 'numpy>=2'`.
- **Rebuild after code changes** — Settings → Developer → *Rebuild Workspace*, or
  `colcon build --packages-select fic_studio` from the activated env. (Don't add
  `--symlink-install` on macOS — modern setuptools rejects it for ament_python.)
- **No `timeout` command when scripting** — macOS lacks it; use `gtimeout`
  (`brew install coreutils`) or `perl -e 'alarm N; exec @ARGV'`.
- **App opens then immediately quits (Dock icon bounces and vanishes)** — the
  repo is under a privacy-protected folder (`~/Documents`/`~/Desktop`/
  `~/Downloads`) and the app can't read the workspace (TCC). Move the repo out
  (e.g. `~/FIC_Interface`) and re-run the installer. Terminal / `.command`
  launches are unaffected, which is why it's easy to miss.
- **macOS blocks the app on first open** — the bundles are ad-hoc signed and
  built locally (not downloaded), so Gatekeeper usually allows them; if it
  doesn't, right-click the app → **Open** once to whitelist it.
- **App shows a generic icon** — a Finder icon-cache delay; the installer nudges
  LaunchServices to refresh. Re-running the installer (or logging out/in) fixes
  it. If the icon never renders, the env's PyQt5 couldn't rasterize the SVG — the
  app still works, just without the custom icon.

See also `fic_ws/src/mujoco_native/README.md` for backend internals.
