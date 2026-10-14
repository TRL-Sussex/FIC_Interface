#!/usr/bin/env bash
# build.sh — build the FIC ROS 2 workspace with the right flags for this OS.
#
# Detects the platform and runs the correct colcon invocation:
#   macOS (Apple Silicon / RoboStack conda env)
#       - activates the `fic_ros` env if it isn't already active
#       - puts the conda prefix on the linker search path (see LIBRARY_PATH below)
#       - no --symlink-install (setuptools>=80 dropped `setup.py develop`)
#   Ubuntu Linux (amd64 / arm64)
#       - sources /opt/ros/$ROS_DISTRO/setup.bash if the env isn't sourced
#       - colcon build --symlink-install with jobs capped to the RAM (and any
#         per-architecture skip fic_install/detect-system.sh names; none today)
#
# Extra arguments are passed straight through, e.g.
#   ./build.sh --packages-select fic_studio
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
WS_PATH="$HERE/fic_ws"
ENV_NAME="${FIC_CONDA_ENV:-fic_ros}"

[[ -d "$WS_PATH" ]] || { echo "No workspace at $WS_PATH" >&2; exit 1; }
cd "$WS_PATH"

case "$(uname -s)" in
  Darwin)
    # Activate the RoboStack env unless the caller already did.
    if [[ "${CONDA_DEFAULT_ENV:-}" != "$ENV_NAME" ]]; then
      CONDA_BASE="${CONDA_BASE:-/opt/homebrew/Caskroom/miniforge/base}"
      [[ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]] || CONDA_BASE="$(conda info --base 2>/dev/null || true)"
      if [[ ! -f "$CONDA_BASE/etc/profile.d/conda.sh" ]]; then
        echo "Could not find conda. Run fic_install/macos/install-fic-studio-macos.sh first." >&2
        exit 1
      fi
      # conda's activate.d hooks (gfortran, etc.) reference unset variables, so
      # they abort under `set -u`. Relax it just for the activation.
      set +u
      # shellcheck disable=SC1091
      source "$CONDA_BASE/etc/profile.d/conda.sh"
      conda activate "$ENV_NAME"
      set -u
    fi

    # conda's `rcutils` exports a bare `-latomic`, inherited by every dependent
    # target. libatomic.dylib IS in the env, but colcon links everything else by
    # absolute path, so the link line carries no `-L` and Apple's ld fails with
    #     ld: library 'atomic' not found
    # clang forwards LIBRARY_PATH to the linker as a search path, which fixes
    # every target at once without a CMake reconfigure.
    export LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"

    export CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
    echo "→ macOS build in '$ENV_NAME'"
    exec colcon build \
      --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
      "$@"
    ;;

  Linux)
    # shellcheck source=fic_install/detect-system.sh
    . "$HERE/fic_install/detect-system.sh"
    if [[ -z "${ROS_DISTRO:-}" && -n "$FIC_ROS_DISTRO" ]]; then
      # ROS setup.bash trips `set -u` on unset AMENT_* variables.
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/$FIC_ROS_DISTRO/setup.bash"
      set -u
    fi
    if [[ -z "${ROS_DISTRO:-}" ]]; then
      echo "No ROS 2 installation found under /opt/ros. Run ./install.sh first." >&2
      exit 1
    fi

    # Optimised builds unless asked otherwise: an unoptimised controller misses
    # its 1 kHz deadlines. (CMake reads this on a package's first configure.)
    export CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

    # Per-architecture flags from detect-system.sh: cap parallel jobs to the
    # RAM on small boards (and skip anything it lists as unbuildable).
    # shellcheck disable=SC2046
    echo "→ Linux build (ROS 2 $ROS_DISTRO, $FIC_ARCH, $CMAKE_BUILD_TYPE): colcon build $(fic_colcon_args)"
    # shellcheck disable=SC2046
    exec colcon build $(fic_colcon_args) "$@"
    ;;

  *)
    echo "Unsupported OS: $(uname -s)." >&2
    echo "Supported: macOS (Apple Silicon) and Ubuntu Linux (22.04/24.04)." >&2
    exit 1
    ;;
esac
