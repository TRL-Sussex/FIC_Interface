# FIC Studio in Docker

This runs the whole of FIC Control Studio on **Windows, macOS (Apple Silicon or
Intel) and Linux**: the Studio and the Dev Console, the MuJoCo simulator, the
FIC controller and the extensions that work in simulation. You install only Docker. The Studio's
desktop opens in your web browser.

It is **simulation only**: nothing in it talks to a real robot.

---

## How it works

The Docker image is a complete Linux computer in a box: ROS 2 Jazzy, MuJoCo,
FIC Studio and its extensions, already built. When you start it:

1. **A container starts** from the image `ghcr.io/trl-sussex/fic_interface`
   (downloaded the first time, about 1 GB).
2. **Inside it, a virtual screen starts**, with a small desktop and a taskbar,
   and FIC Control Studio opens on it automatically.
3. **That screen is streamed to your web browser** at `http://localhost:6080`.
   You click and type in the browser as on a normal desktop; the MuJoCo
   windows, the plotter and the docs all open there too.
4. **Nothing is lost when you stop it.** Settings, tuning and robots you add
   are kept in a Docker volume (`fic-studio-home`). Recordings and your own
   extensions go to a normal folder on your computer, `FIC_Studio` in your
   home folder.
5. **It is password-protected.** Every start gets a new random password,
   built into the link the launcher opens, so other users of the same
   computer cannot get in.

The launcher (`fic-studio.sh` on macOS/Linux, `fic-studio.cmd` on Windows)
does all of this: it creates the container, waits until it is ready and
opens the browser.

Docker should have **4 or more CPUs and 8 GB of memory** (Docker Desktop →
Settings → Resources). The simulation draws its 3D view on the CPU, so more
cores make it smoother.

---

## Getting the image

The image comes **precompiled**: GitHub Actions builds it for both Intel/AMD
and ARM (Apple Silicon) and publishes it to `ghcr.io/trl-sussex/fic_interface`.
You do not need to compile anything: the launcher downloads it the first time
you start it (about 1 GB). Go straight to the steps for your system below.

If the download stops with *not allowed to download*, *unauthorized* or *does
not exist (yet)*, the image is not public yet: see
[If the image is not public](#if-the-image-is-not-public) near the end.

---

## Linux (Ubuntu)

**1. Install Docker (once)**

```sh
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
```

Then **log out and back in** (or restart) so your account can use Docker.
Until you do, Docker says `permission denied`; `newgrp docker` in a terminal
works around it for that terminal only.

Ubuntu's own `docker.io` package works too, but building the image also needs
BuildKit, which it does not include: `sudo apt install docker-buildx`. (Running
the published image needs neither.)

**2. Start FIC Studio**

```sh
cd <the repository>/docker
./fic-studio.sh
```

You will see *Waiting for the desktop… ready*, and your browser opens on FIC
Control Studio.

If it stops with *not allowed to download* or *does not exist (yet)*, the
precompiled image is not public yet: see [If the image is not public](#if-the-image-is-not-public).

**3. Run a simulation**

- The robot selector at the top shows **KinovaGen3_7DOF**, and Target is
  **Simulation**.
- Click **▶ Start**. A MuJoCo window opens with the arm, and the Dashboard
  shows *simulator* and *fic controller* running.
- Use the joint controls, or the extensions in the sidebar (☰).
- **■ Stop** stops the simulation.

**4. Stop, and next time**

```sh
./fic-studio.sh stop     # stop (everything is saved)
./fic-studio.sh          # start again, or reopen the link if it is running
```

If you close the browser tab, run `./fic-studio.sh` again: it prints and opens
the link, password included.

**Real windows instead of the browser (Linux only)**

```sh
./fic-studio.sh --native
```

The Studio and MuJoCo windows then open directly on your Ubuntu desktop. Close
the Studio to stop it. See *Linux: windows on your own desktop* below.

---

## Windows 10 / 11

**1. Install Docker Desktop (once)**

1. Download [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
   and run the installer. Keep **Use WSL 2** ticked.
2. Restart Windows if it asks. If it reports that WSL 2 or virtualisation is
   missing, open **PowerShell as Administrator**, run `wsl --install`, and
   restart. Some laptops also need virtualisation turned on in the BIOS/UEFI.
3. Start **Docker Desktop** from the Start menu, accept the terms, and wait
   until it says *Engine running*. You do not need a Docker account.
4. Optional: **Settings → Resources**, give it at least 4 CPUs and 8 GB.
5. On a shared lab PC, your Windows account must be in the **docker-users**
   group (an administrator runs `net localgroup docker-users <your user> /add`;
   then sign out and back in).

**2. Get the launcher**

You need the `docker` folder of this repository (clone it, or download the
repository as a ZIP from GitHub and unzip it). The files that matter are
`fic-studio.cmd` and `fic-studio.ps1`, side by side.

**3. Start FIC Studio**

Double-click **`fic-studio.cmd`**. A console window opens, downloads FIC
Studio the first time (about 1 GB, with progress shown), waits for it, and
opens your browser on FIC Control Studio. If Docker Desktop is not running,
it starts it for you.

If Windows shows *Windows protected your PC* (SmartScreen), click **More info →
Run anyway**. The console window can be closed once the browser is open; FIC
Studio keeps running in Docker.

If it stops with *not allowed to download* or *does not exist (yet)*, the
precompiled image is not public yet: see [If the image is not public](#if-the-image-is-not-public).

**4. Run a simulation**: exactly as on Linux above (▶ Start, ■ Stop).

**5. Stop, and next time**

Open **PowerShell** (or a Command Prompt) in the `docker` folder:

```powershell
.\fic-studio.cmd stop     # stop (everything is saved)
.\fic-studio.cmd          # start again, or reopen the link
```

Double-clicking `fic-studio.cmd` again also reopens the link. Other commands
work the same way: `.\fic-studio.cmd update`, `logs`, `shell`, `status`,
`reset`, `uninstall`.

Your shared folder is `C:\Users\<you>\FIC_Studio`.

---

## macOS (Apple Silicon or Intel)

**1. Install Docker Desktop (once)**

1. Download [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/).
   Pick the build for your chip (Apple menu → About This Mac: *Apple M…* means
   Apple Silicon, *Intel* means Intel).
2. Open the `.dmg`, drag Docker to Applications, and start it. Approve the
   permissions it asks for, accept the terms, and wait until it says *Engine
   running*. You do not need a Docker account.
3. Optional: **Settings → Resources**, give it at least 4 CPUs and 8 GB.

FIC Studio runs natively on Apple Silicon (the image is built for arm64 as
well); Rosetta is not needed.

**2. Get the launcher**

Clone this repository (or download it as a ZIP and unzip it). Keep it outside
*Desktop*, *Documents* and *Downloads* if macOS keeps asking for folder
permissions.

**3. Start FIC Studio**

Open **Terminal** (Applications → Utilities) and run:

```sh
cd <the repository>/docker
./fic-studio.sh
```

If macOS says permission denied, run `chmod +x fic-studio.sh` once. The
launcher starts Docker Desktop if it is not running, downloads FIC Studio the
first time, and opens your browser (Safari, Chrome, …) on FIC Control Studio.

If it stops with *not allowed to download* or *does not exist (yet)*, the
precompiled image is not public yet: see [If the image is not public](#if-the-image-is-not-public).

**4. Run a simulation**: exactly as on Linux above (▶ Start, ■ Stop).

**5. Stop, and next time**

```sh
./fic-studio.sh stop     # stop (everything is saved)
./fic-studio.sh          # start again, or reopen the link
```

Your shared folder is `~/FIC_Studio` (Finder → your home folder).

---

<details>
<summary>Without the launchers: one <code>docker run</code>, on any OS</summary>

```sh
docker run -d --name fic-studio --hostname fic-studio --shm-size 2g \
    -p 127.0.0.1:6080:6080 -p 127.0.0.1:8088:8088 \
    -v fic-studio-home:/home/fic ghcr.io/trl-sussex/fic_interface:latest
```

Then open <http://localhost:6080>. Stop it with `docker stop fic-studio`, and
start it again with `docker start fic-studio`. This desktop has no password.
On a computer that other people also log in to, add
`-e FIC_VNC_PASSWORD=<password>` and open
`http://localhost:6080/vnc.html?password=<password>`.

With Docker Compose: `docker compose up -d` in this folder, using
[`compose.yaml`](compose.yaml).
</details>

## Check that it works

1. The browser shows FIC Control Studio, and **KinovaGen3_7DOF** is the selected robot.
2. Press **▶ Start**. A MuJoCo window opens with the arm in it.
3. The Dashboard shows *simulator* and *fic controller* running, with live joint values.

For a full check (about two minutes), run
`docker exec fic-studio fic-selftest` (any OS). It tests the libraries, the
extensions, the simulator, the controller and the windows.

---

## Using it

**The desktop.** The taskbar at the bottom has launchers for FIC Control
Studio, the Dev Console, a terminal, the file manager and the documentation.
Right-clicking the desktop gives the same menu. If you close the Studio, or it
crashes, start it again from there. The desktop resizes to fit the browser
window.

Give the browser window at least **1280 × 720**. On a smaller window the
right end of the Studio's toolbar, including E-STOP, is cut off.

**Copy and paste** between your computer and the Studio goes through the
clipboard panel in the sidebar, on the left edge of the page.

**Keyboard shortcuts** that the browser reserves (Ctrl+N, Ctrl+T, Ctrl+Q,
Ctrl+Shift+Q, F11) go to the browser, not to the Studio. Use the Studio's
menus for those actions.

**Your files.** The folder **`FIC_Studio`** in your home folder (Windows:
`C:\Users\<you>\FIC_Studio`) is shared with the Studio:

| In `FIC_Studio/` | What goes there |
|---|---|
| `experiments/` | Bag recordings (BAG button) |
| `extensions/` | **Your own extensions**: each folder with a `manifest.json` shows up in the Studio (enable it under Settings → Extensions) |
| anything else | For example a robot's URDF folder, to drop onto **Robot Configuration → Add a robot** |

**What is kept.** Settings, tuning, robot configurations, robots you added
and extensions you made in the Studio all survive stopping, restarting and
updating. `./fic-studio.sh reset` forgets them all. The shared folder is never
touched.

**The web remote** (the Studio's phone page) is at <http://localhost:8088>. In this
image it opens straight away: the port is published on 127.0.0.1 only, so the
sign-in and the certificate the Studio normally insists on are switched off in
`/etc/fic/robot_selector_gui_settings.yaml` (`web_remote.accounts`,
`web_remote.https`). To
use it from a phone on your network, start with `./fic-studio.sh --lan`: that
turns sign-in on (and visitors' own sign-up off), and the launcher prints the address and a password for the
account `admin` (a new one every start; `./fic-studio.sh` shows it again). The
page is plain http, so use `--lan` on a network you trust.

**A terminal inside.** Use `./fic-studio.sh shell`, or the terminal on the
taskbar. ROS 2 and the workspace are already set up there (`ros2 topic list`).
From a script, use `docker exec fic-studio fic <command>`, for example
`docker exec fic-studio fic ros2 topic list`.

## Commands

| macOS / Linux | Windows | |
|---|---|---|
| `./fic-studio.sh` | `fic-studio.cmd` | Start, and open the browser |
| `./fic-studio.sh stop` | `fic-studio.cmd stop` | Stop (settings are kept) |
| `./fic-studio.sh restart` | `fic-studio.cmd restart` | Stop and start again |
| `./fic-studio.sh update` | `fic-studio.cmd update` | Download the newest version and restart |
| `./fic-studio.sh logs` | `fic-studio.cmd logs` | Show what it is doing |
| `./fic-studio.sh shell` | `fic-studio.cmd shell` | A terminal inside it |
| `./fic-studio.sh status` | `fic-studio.cmd status` | Running? Which version? What is saved? |
| `./fic-studio.sh reset` | `fic-studio.cmd reset` | Forget all settings and saved robots |
| `./fic-studio.sh uninstall` | `fic-studio.cmd uninstall` | Remove it from Docker |

Options: `--dev` starts the Dev Console instead of the Studio, and
`--desktop` only the desktop. `--port 6081`
uses another browser port, and `--web-port 8089` another web-remote port.
`--lan` opens the web remote to your network, with sign-in on (as `admin`, with the
password the launcher prints). `--folder DIR` shares a
different folder. `--image REF` runs another image or version. `--no-browser`
skips opening the browser. The Windows equivalents are `-Dev`, `-Desktop`, `-Port 6081`,
`-WebPort 8089`, `-Lan`, `-Folder DIR`, `-Image REF` and `-NoBrowser`.

### Linux: windows on your own desktop

```sh
./fic-studio.sh --native
```

This opens the Studio and the MuJoCo windows directly on your desktop instead
of in the browser. It uses your GPU when Docker can reach it. That is automatic
for Intel and AMD graphics (through `/dev/dri`), and needs the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
for NVIDIA. The container stops when you close the Studio. It shares your
session's shared memory with X11 (`--ipc=host`, which software OpenGL needs to
draw correctly), and keeps its own `/dev/shm` for ROS 2.

---

## Troubleshooting

**`docker: command not found`, or Docker is not running.** Start Docker
Desktop and wait until it says it is running, then open a new terminal. The
scripts try to start Docker Desktop themselves.

On **macOS**, if Docker Desktop is running but a new terminal still says
`command not found`, Docker Desktop put `docker` in `~/.docker/bin`, which
your shell does not search. Add it once:

```sh
echo 'export PATH="$HOME/.docker/bin:$PATH"' >> ~/.zshrc
source ~/.zshrc
docker --version        # should now print a version
```

(Or: Docker Desktop → Settings → Advanced → choose **System** for the CLI
tools, which needs your admin password.)

**`permission denied` on Linux.** Run `sudo usermod -aG docker $USER`, then
log out and back in.

**The page asks for a password.** Open the link that `./fic-studio.sh` (or
`fic-studio.cmd`) prints: it carries the password. Running the script again
while FIC Studio is running prints it again.

**`port is already allocated`.** Something else is using port 6080 (or 8088).
Use another port: `./fic-studio.sh restart --port 6081`.

**The page says "Failed to connect" or keeps reconnecting.** It is still
starting. Wait a few seconds; it reconnects by itself. If it does not, check
`./fic-studio.sh logs`.

**The desktop is empty.** The Studio was closed. Start it from the taskbar or
the right-click menu.

**Several copies of the desktop, or it looks letterboxed.** Use one browser
tab. If the desktop does not fill the window, reload the page.

**Everything is slow.** Give Docker more CPUs (Docker Desktop → Settings →
Resources). Close MuJoCo windows you are not using, since each one is drawn on
the CPU. A smaller browser window also helps.

**`denied` or `unauthorized` when downloading.** The image is not public (yet).
Log in with a GitHub account that has access: `docker login ghcr.io` (the
password is a GitHub token with `read:packages`).

**Start from scratch.** Run `./fic-studio.sh reset`, then `./fic-studio.sh`.

## Limits

- **Simulation only.** There are no robot drivers.
- **No GPU physics.** The Newton backend needs an NVIDIA GPU with CUDA, so it
  is left out. The CPU MuJoCo simulator (`mujoco_native`) is included.
- **Gamepads** only work with Linux hosts, by adding `--device /dev/input/js0`
  to the `docker run` command.

---

## If the image is not public

Until the package is public, `docker pull ghcr.io/trl-sussex/fic_interface:latest`
says `unauthorized` or `denied`. Use one of these instead.

**A. Download it while it is private.** Log in to GitHub's registry once with
an account that can see the TRL-Sussex repository:

1. On GitHub: your picture → **Settings → Developer settings → Personal
   access tokens → Tokens (classic) → Generate new token (classic)**. Tick
   only **`read:packages`**, and copy the token.
2. In a terminal:
   ```sh
   docker login ghcr.io -u <your GitHub username>
   ```
   and paste the token when it asks for a password.
3. Start as usual: `./fic-studio.sh` (Windows: `fic-studio.cmd`). That is
   `ghcr.io/trl-sussex/fic_interface:latest`, built from the `main`
   branch. To pin a release, name it: `--image
   ghcr.io/trl-sussex/fic_interface:v5.6.0` (`-Image` on Windows).

**B. Load a file someone gave you.** If a colleague hands you a
`fic-studio_local.tar.gz` (made with `docker/build.sh --save`, see *For
maintainers*), load it and start it:

```sh
docker load -i fic-studio_local.tar.gz
./fic-studio.sh --image fic-studio:local
```

**C. Build it yourself.** Only if A and B do not work. This compiles ROS 2
packages and the whole FIC workspace. On a 10-core Apple Silicon Mac the
first build took about **7 minutes**; allow 15 to 30 minutes on a laptop with
fewer cores. It needs about **10 GB** of free disk space (4.4 GB for the image,
the rest is build cache that makes later builds faster). Follow
[Building the image yourself, step by step](#building-the-image-yourself-step-by-step)
below; in short:

```sh
cd <the repository>/docker
./build.sh
./fic-studio.sh --image fic-studio:local
```

**Not typing `--image` every time.** Set it once in your shell instead, and
every `./fic-studio.sh` command uses it:

```sh
echo 'export FIC_IMAGE=fic-studio:local' >> ~/.zshrc     # macOS (bash: ~/.bashrc)
```

On Windows: `setx FIC_IMAGE fic-studio:local`, then open a new console.
With a locally built or loaded image, `./fic-studio.sh update` cannot download
a newer one; rebuild (or load the new file) and run `./fic-studio.sh restart`
instead.

---

## Building the image yourself, step by step

This builds `fic-studio:local` from the source code on your own computer and
runs it. You need Docker, Git, about **10 GB** of free disk space and 15 to
30 minutes. Type each line into a terminal, one at a time, and wait for it to
finish before typing the next.

### On Windows 10 / 11

**1. Install Docker Desktop and Git (once).** Install Docker Desktop as in
[Windows 10 / 11](#windows-10--11), step 1, and start it. Install
[Git for Windows](https://git-scm.com/download/win) with its default options;
it includes **Git Bash**, which runs `build.sh`.

**2. Give Docker enough resources.** Docker Desktop → **Settings →
Resources**: at least 4 CPUs and 8 GB of memory (more CPUs build faster).
Click **Apply & restart**.

**3. Open PowerShell** (Start menu → type *PowerShell* → Enter).

**4. Check that Docker works:**

```powershell
docker version
```

It prints a *Client* and a *Server* section. If it says *docker is not
recognized*, Docker Desktop was installed for your user only and did not put
`docker` on PATH. Fix it for this window with:

```powershell
$env:Path = "$env:LOCALAPPDATA\Programs\DockerDesktop\resources\bin;$env:ProgramFiles\Docker\Docker\resources\bin;$env:Path"
docker version
```

If it prints the *Client* part but an error for the *Server*, Docker Desktop
is not running yet: start it and wait for *Engine running*.

**5. Go to the folder you want the code in**, for example your home folder:

```powershell
cd $HOME
```

**6. Download the code with Linux line endings.** The image is Linux, and
scripts with Windows line endings (CRLF) fail inside it with errors such as
`python3\r: No such file or directory`. The two `-c` options keep the files
exactly as they are in the repository and allow its long file names:

```powershell
git clone -c core.autocrlf=false -c core.longpaths=true --branch main https://github.com/TRL-Sussex/FIC_Interface.git FIC_Docker_Build
```

`main` is the branch this image is built from. A warning about
`Pendulum.urdf` and `pendulum.urdf` colliding is harmless. If you already have
a clone made with default Git settings, do not build from it: make this
separate one.

**7. Go into it:**

```powershell
cd FIC_Docker_Build
```

**8. Build the image.** Either with the build script through Git Bash
(recommended: it also runs the self-test):

```powershell
& "$env:ProgramFiles\Git\bin\bash.exe" docker/build.sh --test
```

or directly with Docker (no self-test):

```powershell
docker buildx build --file docker/Dockerfile --tag fic-studio:local --load .
```

The first build downloads about 1 GB and compiles for 15 to 30 minutes. It is
finished when it prints a table with `fic-studio   local` and, with `--test`,
the self-test ends with all checks passed. Later builds reuse the cache and
take a minute or two when little has changed.

**9. Check that the image is there:**

```powershell
docker image ls fic-studio
```

**10. Start it:**

```powershell
.\docker\fic-studio.cmd -Image fic-studio:local
```

The browser opens on FIC Control Studio. Check it as in
[Check that it works](#check-that-it-works).

**11. Optional: make the local image the default**, so `fic-studio.cmd`
(and double-clicking it) uses it without `-Image`:

```powershell
setx FIC_IMAGE fic-studio:local
```

Then open a new PowerShell window.

**12. After you change the code, or pull new code:**

```powershell
git pull
& "$env:ProgramFiles\Git\bin\bash.exe" docker/build.sh
.\docker\fic-studio.cmd restart -Image fic-studio:local
```

`restart` creates the container again from the new image; your settings are
kept.

### On macOS or Linux

**1. Install Docker (once)**: Docker Desktop on macOS (see
[macOS](#macos-apple-silicon-or-intel), step 1), or Docker Engine on Linux (see
[Linux](#linux-ubuntu), step 1). Git is already there on most systems (macOS
offers to install it the first time you type `git`).

**2. Give Docker enough resources** (Docker Desktop → Settings → Resources:
4 or more CPUs, 8 GB or more of memory). Docker Engine on Linux uses the whole
computer.

**3. Open a terminal** and check that Docker works:

```sh
docker version
```

**4. Download the code:**

```sh
cd ~
git clone --branch main https://github.com/TRL-Sussex/FIC_Interface.git FIC_Docker_Build
cd FIC_Docker_Build
```

**5. Build the image and run the self-test:**

```sh
./docker/build.sh --test
```

**6. Check that the image is there:**

```sh
docker image ls fic-studio
```

**7. Start it:**

```sh
./docker/fic-studio.sh --image fic-studio:local
```

**8. After you change the code, or pull new code:**

```sh
git pull
./docker/build.sh
./docker/fic-studio.sh restart --image fic-studio:local
```

### Handing the image to someone else

To give the image you built to someone without internet access to the
registry, write it to a file:

```sh
./docker/build.sh --save        # Windows: & "$env:ProgramFiles\Git\bin\bash.exe" docker/build.sh --save
```

This writes `docker/dist/fic-studio_local.tar.gz` (a little over 1 GB). They load it
with `docker load -i fic-studio_local.tar.gz` and start it with `--image
fic-studio:local` (see [If the image is not public](#if-the-image-is-not-public), option B). An
image built on an Intel/AMD PC runs on Intel/AMD computers; one built on
Apple Silicon runs on ARM computers.

---

## For maintainers

### How the image is built and published

`.github/workflows/docker-image.yml` builds the image for **amd64 and arm64
on native GitHub runners** and runs `fic-selftest` in each. It then publishes
one multi-arch image, **`ghcr.io/trl-sussex/fic_interface`**, with these tags:

- `latest`, from `main` (the branch this image is built from)
- the branch name
- `sha-<commit>`
- the version, for tags like `v1.2.3`

It runs on pushes to `main`, for version tags (`v5.6.0`), every Monday, and
by hand from the Actions tab. Pull requests that touch the image build and
test it without publishing.

The same image and tags also go to `ghcr.io/trl-sussex/fic-studio`, the name
given out at the ROSCon UK 2026 workshop, so older launchers keep working and
keep getting updates.

To let users pull without logging in, the `fic_interface` and `fic-studio`
packages must be public (the organisation's **Packages** → the package →
**Package settings** → **Change visibility**).

### Building locally

```sh
docker/build.sh --test                   # build for this machine, then run the self-test
docker/build.sh --platform linux/arm64   # build for the other architecture (emulated, slow)
docker/build.sh --save                   # also write docker/dist/*.tar.gz for offline hand-out
docker/fic-studio.sh --image fic-studio:local   # run what you built
```

A saved image is loaded with `docker load -i <file>.tar.gz`, so people can
use it without a download.

### How it works

| Piece | Where |
|---|---|
| Image recipe | [`Dockerfile`](Dockerfile) |
| Starts as root, maps the host UID, prepares the home folder | [`rootfs/usr/local/bin/fic-entrypoint`](rootfs/usr/local/bin/fic-entrypoint) |
| The desktop session: virtual display or host display, then the GUI | [`rootfs/usr/local/bin/fic-session`](rootfs/usr/local/bin/fic-session) |
| `fic studio`, `fic dev`, `fic shell`… | [`rootfs/usr/local/bin/fic`](rootfs/usr/local/bin/fic) |
| Carries workspace changes across updates | [`rootfs/usr/local/bin/fic-state`](rootfs/usr/local/bin/fic-state) |
| The self-test | [`rootfs/usr/local/bin/fic-selftest`](rootfs/usr/local/bin/fic-selftest) |
| OpenGL frame pacing | [`rootfs/usr/local/src/fic/fic_gl_pace.c`](rootfs/usr/local/src/fic/fic_gl_pace.c) |
| First-run Studio settings (enabled extensions, skip wizard) | [`rootfs/etc/fic/robot_selector_gui_settings.yaml`](rootfs/etc/fic/robot_selector_gui_settings.yaml) |

- **The workspace is built in place** at `/opt/fic`, with
  `--symlink-install`, and `src/`, `build/` and `install/` all stay in the
  image. The tuning the Studio saves (`fic_library/config`) is the file the
  controller reads through `install/`, which does not survive copying or
  moving the workspace.
- **The simulator is `mujoco_native`** on the `mujoco==3.10.0` wheel, which is
  native on both architectures.
- **The virtual desktop** is TigerVNC (`Xtigervnc`) behind noVNC 1.7.0 and
  websockify, with openbox and tint2. Only this computer can reach it
  (`127.0.0.1`). The launchers also give it a new random password on every
  start (`FIC_VNC_PASSWORD`), carried in the link they open, so other users of
  the same computer cannot use it; on Linux the link goes to the browser
  through a short-lived page only you can read, not on a command line. Plain
  `docker run` and `compose.yaml` have no password unless you set one.
- **Software OpenGL** is Mesa llvmpipe. Without vertical sync MuJoCo's viewers
  would redraw as fast as they can, so a wrapper around `glXSwapBuffers` caps
  the redraw rate (`FIC_GL_MAX_FPS`, 30 by default in the browser desktop), and
  llvmpipe gets a quarter of the CPUs, 1 to 4 threads (`LP_NUM_THREADS`). With
  both, the simulator and the controller hold 1 kHz on 4 CPUs.
- **What is saved where.** `/home/fic` is a volume, and holds settings,
  scripts, sequences and caches. Workspace files the Studio edits are copied
  into `~/.fic_state` when they differ from the image, and copied back when a
  new container starts: `fic_studio_robots.yaml`,
  `fic_library/config/{robots_config.yaml,config.xml}`, robots added with
  "Add a robot", and extensions made in the Studio. Files the user never
  changed are not saved, so image updates still reach them. If an image
  update changes a file the user has edited, the user's version is kept. The
  image's new version is placed next to it as `<file>.from-new-image`, and
  `fic-state status` and the log point it out. `experiments/` is a link into
  the shared folder.

### Environment variables

| Variable | Default | |
|---|---|---|
| `FIC_DISPLAY` | `vnc` | `vnc`: browser desktop. `host`: the X display in `DISPLAY` (Linux) |
| `FIC_SCREEN` | `1600x1000` | Starting size of the browser desktop (it then follows the browser window) |
| `FIC_VNC_PASSWORD` | – | Require this password to open the desktop (the launchers set a new one every start; VNC uses the first 8 characters) |
| `FIC_UID`, `FIC_GID` | – | Run as this user and group, so files in the shared folder are theirs (Linux). `0` for rootless Docker and Docker Desktop for Linux, where the host user is root in the container (the launcher detects both) |
| `FIC_GL_MAX_FPS` | 30 (browser), 60 (host) | Maximum redraw rate of OpenGL windows; `0` for no limit |
| `LP_NUM_THREADS` | ¼ of the CPUs, 1 to 4 | Mesa llvmpipe render threads |
| `FIC_GL` | `auto` | `software` forces CPU rendering in host mode |
| `FIC_WEB_REMOTE_HOSTS` | `localhost` | Addresses the web remote advertises (comma-separated) |
| `FIC_WEB_REMOTE_PORT` | – | The host port the web remote is published on, when it is not 8088 |
| `FIC_WEB_ADMIN_PASSWORD` | – | Turns web sign-in on, with this password for the account `admin` (the launcher's `--lan` sets a new one every start) |
| `FIC_URL` | – | The URL the startup banner shows (the launchers pass the published one) |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | Keeps the ROS 2 graph inside the container. Set `SUBNET` (and share a Docker network) to connect two Studios |
