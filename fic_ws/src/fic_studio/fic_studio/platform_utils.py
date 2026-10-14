"""Cross-platform helpers so the GUI behaves correctly on macOS and Linux.

FIC Studio was originally Ubuntu-only. These helpers centralise the handful of
places where Linux and macOS differ (opening files/URLs, monospace fonts, the
CPU-count shell expression used in build commands) so callers don't have to
sprinkle ``sys.platform`` checks everywhere.
"""

from __future__ import annotations

import subprocess
import sys

IS_MACOS = sys.platform == 'darwin'
IS_LINUX = sys.platform.startswith('linux')
IS_WINDOWS = sys.platform.startswith('win')


def default_mono_font() -> str:
    """Preferred monospace font family for the current platform.

    macOS ships Menlo/SF Mono, not Consolas; Windows has Consolas; most Linux
    desktops have DejaVu Sans Mono.
    """
    if IS_MACOS:
        return 'Menlo'
    if IS_WINDOWS:
        return 'Consolas'
    return 'DejaVu Sans Mono'


def mono_font_choices() -> list:
    """Monospace font options to offer in Settings, platform-appropriate first."""
    if IS_MACOS:
        return ['Menlo', 'SF Mono', 'Monaco', 'Courier New', 'Roboto Mono']
    if IS_WINDOWS:
        return ['Consolas', 'Courier New', 'Cascadia Mono', 'Roboto Mono']
    return ['DejaVu Sans Mono', 'Consolas', 'Courier New', 'Roboto Mono', 'Ubuntu Mono']


# CSS/Qt-stylesheet font stacks. Listing every platform's fonts lets Qt pick the
# first that exists, so the same stylesheet renders correctly everywhere.
UI_FONT_STACK = "'SF Pro Text', 'Helvetica Neue', 'Segoe UI', 'Ubuntu', 'Roboto', sans-serif"
MONO_FONT_STACK = "'Menlo', 'SF Mono', 'Consolas', 'DejaVu Sans Mono', monospace"


def open_path(path: str) -> None:
    """Open a file or folder in the OS file manager (Finder / Nautilus / Explorer)."""
    if IS_MACOS:
        subprocess.Popen(['open', path])
    elif IS_WINDOWS:
        subprocess.Popen(['explorer', path])
    else:
        subprocess.Popen(['xdg-open', path])


def open_url(url: str) -> None:
    """Open a URL in the default browser (works on all platforms)."""
    import webbrowser
    webbrowser.open(url)


# Every workspace package builds on every supported platform (Linux x86-64 and
# arm64, macOS). Until v4.5.0 the C++ `mujoco` simulator had to be skipped on
# macOS (it linked `rt`) and off x86-64 (it vendored an x86-64 libmujoco.so);
# it was retired in favour of the pure-Python `mujoco_native`, so the build
# commands no longer carry a skip list.


# conda's `rcutils` exports a bare `-latomic` link flag (rcutilsExport.cmake sets
# INTERFACE_LINK_LIBRARIES "atomic"), which every dependent target inherits.
# libatomic.dylib *is* present in the RoboStack env, but colcon links every other
# library by absolute path, so the link line carries no `-L` at all and Apple's ld
# searches only /usr/lib and the SDK:
#     ld: library 'atomic' not found
# Putting the conda prefix on LIBRARY_PATH — which clang forwards to the linker as
# a search path — fixes every target at once and needs no CMake reconfigure.
MACOS_LINK_ENV = 'LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"'


def colcon_build_command(packages=None) -> str:
    """Platform-appropriate ``colcon build`` command for the GUI build buttons.

    - **Linux:** ``colcon build --symlink-install [--packages-select ...]``.
    - **macOS/RoboStack:** no ``--symlink-install`` (modern setuptools dropped
      ``setup.py develop --editable``, which it relies on); put the conda
      prefix on the linker search path; and pass the conda Python so the C++
      packages resolve NumPy/Pinocchio headers.
    """
    scope = (' --packages-select ' + ' '.join(packages)) if packages else ''
    if IS_MACOS:
        return (MACOS_LINK_ENV + ' colcon build' + scope +
                ' --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"')
    return 'colcon build --symlink-install' + scope


def nproc_expr() -> str:
    """Shell expression that expands to the CPU count on macOS and Linux.

    ``nproc`` doesn't exist on macOS (use ``sysctl -n hw.ncpu``); fall back to
    ``nproc`` on Linux and 4 if neither is available.
    """
    return '$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)'
