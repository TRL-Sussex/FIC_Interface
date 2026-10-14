#!/bin/bash
# install-deps.sh — Install all FIC Control Studio dependencies
# Run with: sudo bash install-deps.sh [ROS_DISTRO]
set -e

# shellcheck source=detect-system.sh
. "$(dirname "$0")/detect-system.sh"
ROS_DISTRO="${1:-$FIC_ROS_DISTRO}"
APT="$FIC_APT"

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║   FIC Control Studio — Dependency Installer         ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
fic_system_report
echo ""

# ── Step 1: System packages ──
echo "[1/4] Installing system packages..."
$APT update -qq
$APT install -y \
    libeigen3-dev libyaml-cpp-dev libglfw3 libtinyxml2-dev \
    python3-dev python3-numpy python3-pip \
    python3-pyqt5 python3-pyqt5.qtsvg python3-pyqt5.qtwebengine python3-yaml python3-scipy \
    python3-cryptography python3-pyqtgraph python3-opencv python3-matplotlib python3-tk \
    git cmake build-essential pkg-config \
    lsb-release curl
echo "  -> System packages done."
echo ""

# ── Step 2: ROS 2 packages ──
if [ -n "$ROS_DISTRO" ]; then
    echo "[2/4] Installing ROS 2 ($ROS_DISTRO) packages..."
    # ros-dev-tools brings colcon: install.sh installs it with ROS, but a ROS
    # installed by hand may not have it.
    $APT install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-std-srvs \
        ros-${ROS_DISTRO}-urdfdom-headers \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-sensor-msgs
    # pinocchio backs the DEFAULT kinematics solver but is OPTIONAL: without it
    # the workspace still builds and robots run joint-space-only unless a custom
    # solver is registered (see docs/custom-kinematics-solver.md). Best-effort;
    # apt-get install also upgrades an already-installed but outdated build
    # (the apt lists were refreshed in step 1).
    PINO_PKG="ros-${ROS_DISTRO}-pinocchio"
    PINO_BEFORE=$(dpkg-query -W -f='${Version}' "$PINO_PKG" 2>/dev/null || true)
    $APT install -y "$PINO_PKG" || \
        echo "  -> pinocchio unavailable — building without the built-in solver."
    PINO_AFTER=$(dpkg-query -W -f='${Version}' "$PINO_PKG" 2>/dev/null || true)
    if [ -n "$PINO_AFTER" ]; then
        if [ -n "$PINO_BEFORE" ] && [ "$PINO_BEFORE" != "$PINO_AFTER" ]; then
            echo "  -> pinocchio updated: $PINO_BEFORE -> $PINO_AFTER"
            echo "  -> NOTE: delete fic_ws/build and fic_ws/install before the next"
            echo "     build — stale CMake caches reference the old pinocchio."
        else
            echo "  -> pinocchio: $PINO_AFTER"
        fi
    fi
    echo "  -> ROS 2 packages done."
else
    echo "[2/4] SKIPPED — no ROS distro detected."
fi
echo ""

# ── Step 3: tinyxml2 ──
# The distribution's tinyxml2 is what ROS itself uses (urdfdom, pinocchio's URDF
# parser, ...). Installing another copy into /usr replaces the header, the
# .so symlink and the CMake config, so FIC links the new soname while ROS keeps
# the old one and BOTH end up loaded in one process: fic_hardware then segfaults
# on start-up (seen on a lab's four robots, 2026-09-23: tinyxml2 11 from source
# over Ubuntu 24.04's 10). So use the packaged one whenever it is there, and
# build from source only where it is missing — into /usr/local, never over it.
echo "[3/4] tinyxml2..."
if [ -f "$FIC_LIBDIR/cmake/tinyxml2/tinyxml2-config.cmake" ] \
   || [ -f /usr/lib/cmake/tinyxml2/tinyxml2-config.cmake ]; then
    echo "  -> using the system tinyxml2 (libtinyxml2-dev). Nothing to build."
else
    echo "  -> not packaged here; building from source into /usr/local..."
    TINYXML2_DIR=$(mktemp -d)
    git clone --depth 1 https://github.com/leethomason/tinyxml2.git "$TINYXML2_DIR"
    mkdir -p "$TINYXML2_DIR/build"
    cd "$TINYXML2_DIR/build"
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j"$(nproc)"
    make install
    ldconfig
    cd /
    rm -rf "$TINYXML2_DIR"
    if [ -f /usr/local/lib/cmake/tinyxml2/tinyxml2-config.cmake ]; then
        echo "  -> tinyxml2 built and installed."
    else
        echo "  -> WARNING: tinyxml2 cmake config not found after install!"
    fi
fi
echo ""

# ── Step 4: Python packages (as the real user, not root) ──
echo "[4/4] Installing Python packages..."
REAL_USER="${SUDO_USER:-$USER}"
REAL_HOME=$(eval echo "~$REAL_USER")

# pip is required for the mujoco/newton Python dependencies below. It should
# already be present from Step 1, but verify and install it if missing so the
# pip3 calls that follow don't fail on a fresh system.
if command -v pip3 >/dev/null 2>&1; then
    echo "  -> pip3 already installed ($(pip3 --version 2>/dev/null))."
else
    echo "  -> pip3 not found; installing python3-pip..."
    $APT install -y python3-pip
    if command -v pip3 >/dev/null 2>&1; then
        echo "  -> python3-pip installed."
    else
        echo "  -> ERROR: pip3 still not available after install; aborting Python setup." >&2
        exit 1
    fi
fi
# PyQt5, pyqtgraph, pyyaml, numpy and OpenCV come from apt (step 1): pip has no
# PyQt5 wheels for arm64 and apt's builds match the system Python everywhere.
PIP_PACKAGES="mujoco"
# The Studio's web UI window needs the SYSTEM QtWebEngine (python3-pyqt5.qtwebengine,
# step 1). A pip PyQtWebEngine / PyQt5 wheel in ~/.local shadows it and breaks it
# with an "undefined symbol" error, so say so rather than leave it to be found.
if su - "$REAL_USER" -c "pip3 list --user 2>/dev/null" | grep -qiE '^(PyQtWebEngine|PyQt5)(-Qt5)? '; then
    echo "  -> WARNING: pip PyQt5/PyQtWebEngine wheels found in ~/.local; they break the Studio's web UI." >&2
    echo "     Fix: pip uninstall PyQtWebEngine PyQtWebEngine-Qt5 PyQt5 PyQt5-Qt5 PyQt5-sip" >&2
fi
# Camera / tracking extensions (wheels for x86-64 only), when this checkout has them.
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
if [ "$FIC_ARCH" = "amd64" ] && { [ -d "$REPO_DIR/Extensions/realsense_hand" ] || [ -d "$REPO_DIR/Extensions/gaze" ] \
        || [ -d "$REPO_DIR/Extensions/battery_sorter" ]; }; then
    PIP_PACKAGES="$PIP_PACKAGES pyrealsense2 mediapipe"
fi
# numpy stays below 2: apt's matplotlib, SciPy, OpenCV and ROS are built against
# NumPy 1.x, and a NumPy 2 in ~/.local (mediapipe's OpenCV asks for one) shadows
# theirs and breaks their imports.
PIP_PACKAGES="$PIP_PACKAGES 'numpy<2'"
su - "$REAL_USER" -c "pip3 install --user $PIP_PACKAGES" || {
    echo "  -> pip install failed, trying with --break-system-packages..."
    su - "$REAL_USER" -c "pip3 install --user --break-system-packages $PIP_PACKAGES" || true
}
echo "  -> Python packages done."
echo ""

# ── Optional: Newton (GPU MuJoCo backend) when NVIDIA driver is present ──
NEWTON_HINT_FILE="$REAL_HOME/.fic_studio_install_hints.yaml"
rm -f "$NEWTON_HINT_FILE"
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    echo "[optional] NVIDIA GPU detected ($GPU_NAME) — installing Newton backend..."
    # newton[sim] pins compatible mujoco_warp + mujoco; [importers] adds trimesh;
    # pyglet (>=2.0) is required by newton.viewer.ViewerGL for the OpenGL window.
    NEWTON_PKGS="'newton[sim,importers]' 'pyglet>=2.0' 'numpy<2'"
    su - "$REAL_USER" -c "pip3 install --user $NEWTON_PKGS" || \
        su - "$REAL_USER" -c "pip3 install --user --break-system-packages $NEWTON_PKGS" || true
    if su - "$REAL_USER" -c "python3 -c 'import newton'" >/dev/null 2>&1; then
        echo "newton_available: true" > "$NEWTON_HINT_FILE"
        chown "$REAL_USER:$REAL_USER" "$NEWTON_HINT_FILE" 2>/dev/null || true
        echo "  -> Newton installed. Default simulator backend will be 'newton'."
    else
        echo "  -> Newton install skipped or failed; falling back to standard MuJoCo."
    fi
    echo ""
fi

# ── Optional: the arm's vendor driver ──
# fic_hardware does not talk to the arm itself: a vendor program does, and the two
# share memory through fic_library's headers. A driver built against older headers
# leaves the arm in low-level servoing with no commands ("Waiting for joint feedback
# in shared memory"), so build it here alongside everything else it needs.
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
KINOVA_EX="$REPO_ROOT/robot_library/Kinova/api_cpp/examples"
if [ -d "$KINOVA_EX" ] && [ "$FIC_KINOVA_DRIVER" -eq 0 ]; then
    echo "[optional] Kinova vendor driver: x86-64 only, skipped on $FIC_ARCH."
    echo "  -> The Kortex library is x86-64 only."
    echo ""
elif [ -d "$KINOVA_EX" ]; then
    echo "[optional] Building the Kinova vendor driver..."
    ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
    if su - "$REAL_USER" -c "[ -f '$ROS_SETUP' ] && . '$ROS_SETUP'; cd '$KINOVA_EX' && \
            mkdir -p build-gcc-release && cd build-gcc-release && \
            cmake .. -DCMAKE_BUILD_TYPE=release -DKORTEX_SUB_DIR= -DUSE_CONAN=OFF > /tmp/fic-driver-build.log 2>&1 && \
            make -j$FIC_BUILD_JOBS >> /tmp/fic-driver-build.log 2>&1"; then
        echo "  -> Kinova driver built."
    else
        echo "  -> Kinova driver build failed (see /tmp/fic-driver-build.log);"
        echo "     hardware control will not work until it is built."
    fi
    echo ""
fi

# ── Inner-loop MCU tools (every architecture) ──
FIC_MCU_BUILD="$REPO_ROOT/robot_library/fic_mcu/scripts/build.sh"
if [ -x "$FIC_MCU_BUILD" ]; then
    echo "[optional] Building the inner-loop MCU simulator..."
    su - "$REAL_USER" -c "bash '$FIC_MCU_BUILD'" > /tmp/fic-mcu-build.log 2>&1 \
        && echo "  -> robot_library/fic_mcu/build/ (fic_mcu_sim, fic_mcu_relay) built." \
        || echo "  -> fic_mcu build failed (see /tmp/fic-mcu-build.log)."
    echo ""
fi

echo "╔══════════════════════════════════════════════════════╗"
echo "║   All dependencies installed successfully!           ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""
