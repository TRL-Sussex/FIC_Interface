#!/bin/bash
# detect-system.sh — what machine is this? One answer for every FIC script.
#
# Sourced by Install_FIC_Studio.desktop, install-deps.sh, build.sh and the
# launchers, so they all agree on the platform:
#
#   . fic_install/detect-system.sh        # sets FIC_* variables, no output
#   bash fic_install/detect-system.sh     # prints the report
#
# Nothing here installs or changes anything.

# ── OS / Ubuntu release → ROS 2 distro ──
FIC_OS="$(uname -s)"
FIC_CODENAME="$( . /etc/os-release 2>/dev/null; echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}")"
FIC_OS_PRETTY="$( . /etc/os-release 2>/dev/null; echo "${PRETTY_NAME:-$FIC_OS}")"
case "$FIC_CODENAME" in
    noble) FIC_ROS_TARGET="jazzy" ;;    # Ubuntu 24.04
    jammy) FIC_ROS_TARGET="humble" ;;   # Ubuntu 22.04
    *)     FIC_ROS_TARGET="" ;;
esac

# Installed ROS 2 (an already-sourced one wins, then the one matching this
# Ubuntu release, then whatever is under /opt/ros).
FIC_ROS_DISTRO="${ROS_DISTRO:-}"
if [ -z "$FIC_ROS_DISTRO" ] || [ ! -f "/opt/ros/$FIC_ROS_DISTRO/setup.bash" ]; then
    FIC_ROS_DISTRO=""
    for d in "$FIC_ROS_TARGET" humble jazzy iron rolling; do
        if [ -n "$d" ] && [ -f "/opt/ros/$d/setup.bash" ]; then FIC_ROS_DISTRO="$d"; break; fi
    done
fi

# ── CPU architecture ──
# dpkg names (amd64/arm64) pick packages; the multiarch triplet
# (x86_64-linux-gnu / aarch64-linux-gnu) is where libraries and their CMake
# configs live — never hard-code the x86 one.
FIC_MACHINE="$(uname -m)"
FIC_ARCH="$(dpkg --print-architecture 2>/dev/null || echo "$FIC_MACHINE")"
FIC_MULTIARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || gcc -dumpmachine 2>/dev/null || echo "${FIC_MACHINE}-linux-gnu")"
FIC_LIBDIR="/usr/lib/$FIC_MULTIARCH"

# ── Board ──
# Arduino UNO Q / VENTUNO Q: Linux MPU + an STM32 MCU that can run the FIC
# inner loop. Identified from the devicetree.
FIC_BOARD="generic"
FIC_BOARD_NAME=""
if [ -r /sys/firmware/devicetree/base/compatible ]; then
    _compat="$(tr '\0' '\n' < /sys/firmware/devicetree/base/compatible 2>/dev/null | head -1)"
    case "$_compat" in
        arduino,imola) FIC_BOARD="uno-q";     FIC_BOARD_NAME="Arduino UNO Q" ;;
        arduino,monza) FIC_BOARD="ventuno-q"; FIC_BOARD_NAME="Arduino VENTUNO Q" ;;
        arduino,*)     FIC_BOARD="arduino";   FIC_BOARD_NAME="Arduino board ($_compat)" ;;
    esac
    unset _compat
fi
FIC_HAS_MCU=0
case "$FIC_BOARD" in uno-q|ventuno-q|arduino) FIC_HAS_MCU=1 ;; esac

# ── What cannot build on this architecture ──
# Every workspace package builds on amd64 and arm64 (the simulator is the
# pure-Python mujoco_native on the pip `mujoco` wheel, which has arm64 builds),
# so FIC_SKIP_PACKAGES is empty; it stays as the hook for any future
# architecture-specific package. The Kinova C++ API ships an x86-64 static
# library only; on other architectures the inner-loop MCU, when there is one,
# drives the arm, so no Linux-side driver is needed.
# (Kinova does publish ARM builds of the API, should one ever be wanted.)
FIC_SKIP_PACKAGES=""
FIC_KINOVA_DRIVER=0
if [ "$FIC_ARCH" = "amd64" ]; then
    FIC_KINOVA_DRIVER=1
fi

# ── Resources ──
FIC_NPROC="$(nproc 2>/dev/null || echo 2)"
FIC_MEM_GB="$(awk '/MemTotal/ {printf "%d", $2/1048576}' /proc/meminfo 2>/dev/null || echo 0)"
# colcon runs one compiler per package; fic_hardware + pinocchio templates need
# ~2 GB each. Cap parallel jobs on small boards so the build is not OOM-killed.
FIC_BUILD_JOBS="$FIC_NPROC"
if [ "$FIC_MEM_GB" -gt 0 ] && [ $((FIC_MEM_GB / 2)) -lt "$FIC_BUILD_JOBS" ]; then
    FIC_BUILD_JOBS=$((FIC_MEM_GB / 2))
    [ "$FIC_BUILD_JOBS" -lt 1 ] && FIC_BUILD_JOBS=1
fi

FIC_NVIDIA=0
FIC_GPU_NAME=""
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    FIC_NVIDIA=1
    FIC_GPU_NAME="$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)"
fi

# ── How to talk to the user, how to get root ──
FIC_HAS_DISPLAY=0
{ [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; } && FIC_HAS_DISPLAY=1
# In a terminal, sudo asks for the password right there. Without one (double-
# clicked from the desktop) pkexec shows the graphical prompt — but pkexec
# needs a polkit agent, which a terminal or SSH session does not have.
if [ "$(id -u)" -eq 0 ]; then
    FIC_SUDO=""
elif [ -t 0 ] || [ "$FIC_HAS_DISPLAY" -eq 0 ]; then
    FIC_SUDO="sudo"
elif command -v pkexec >/dev/null 2>&1; then
    FIC_SUDO="pkexec"
else
    FIC_SUDO="sudo"
fi

# apt waits for the lock instead of failing while unattended-upgrades runs.
FIC_APT="apt-get -o DPkg::Lock::Timeout=900"

fic_colcon_args() {
    # colcon arguments for this machine: skipped packages + job cap.
    local args="--symlink-install --parallel-workers $FIC_BUILD_JOBS"
    [ -n "$FIC_SKIP_PACKAGES" ] && args="$args --packages-skip $FIC_SKIP_PACKAGES"
    echo "$args"
}

fic_system_report() {
    echo "System:        $FIC_OS_PRETTY ($FIC_CODENAME)"
    echo "Architecture:  $FIC_ARCH ($FIC_MULTIARCH), $FIC_NPROC cores, ${FIC_MEM_GB} GB RAM"
    [ -n "$FIC_BOARD_NAME" ] && echo "Board:         $FIC_BOARD_NAME — MCU available for the FIC inner loop"
    if [ -n "$FIC_ROS_DISTRO" ]; then
        echo "ROS 2:         $FIC_ROS_DISTRO (installed)"
    elif [ -n "$FIC_ROS_TARGET" ]; then
        echo "ROS 2:         not installed — will install $FIC_ROS_TARGET"
    else
        echo "ROS 2:         not installed, and '$FIC_CODENAME' has no supported ROS 2 release"
    fi
    [ "$FIC_NVIDIA" -eq 1 ] && echo "GPU:           $FIC_GPU_NAME (Newton backend available)"
    [ -n "$FIC_SKIP_PACKAGES" ] && echo "Not built:     $FIC_SKIP_PACKAGES"
    [ "$FIC_KINOVA_DRIVER" -eq 0 ] && echo "Kinova driver: x86-64 only (the Kortex library is built for x86-64); the inner-loop MCU drives the arm"
    echo "Build jobs:    $FIC_BUILD_JOBS"
    echo "Privileges:    ${FIC_SUDO:-root}"
}

# Executed rather than sourced: print the report.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    fic_system_report
fi
