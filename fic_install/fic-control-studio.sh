#!/bin/bash
# fic-control-studio.sh — Launch FIC Control Studio GUI
# This script sources the ROS2 workspace and runs the GUI entry point.

set -e

# ── Configuration ──
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIC_ROOT="${FIC_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
WS_PATH="$FIC_ROOT/fic_ws"
BUILD_GUI="$SCRIPT_DIR/build_gui.py"
INSTALLER_GUI="$SCRIPT_DIR/installer_gui.py"

# ~/.local/bin, which a desktop session only has on PATH if it existed at login:
# tools installed there later (adb from adb-private.sh) are found all the same.
if [ -d "$HOME/.local/bin" ]; then
    case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) export PATH="$HOME/.local/bin:$PATH" ;; esac
fi

# ── GUI helper (tkinter → zenity → echo fallback) ──
gui_error() {
    if python3 -c "import tkinter" &>/dev/null && [ -f "$INSTALLER_GUI" ]; then
        python3 "$INSTALLER_GUI" error "$1" "$2"
    elif command -v zenity &>/dev/null; then
        zenity --error --title="$1" --text="$2" --width=400 2>/dev/null
    else
        echo "ERROR: $1 — $2" >&2
    fi
}

# ── Source ROS2 (auto-detect distro) ──
if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    ROS_DISTRO=$(ls -1 /opt/ros/ 2>/dev/null | tail -n1)
    if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source "/opt/ros/$ROS_DISTRO/setup.bash"
    else
        gui_error "FIC Control Studio" "No ROS2 installation found in /opt/ros/"
        exit 1
    fi
fi
echo "Using ROS2 distro: $ROS_DISTRO"

# ── Build workspace if needed ──
if [ ! -f "$WS_PATH/install/setup.bash" ]; then
    BUILD_OK=0

    # Try PyQt5 build dialog first
    if python3 "$BUILD_GUI" "$WS_PATH" "bash '$FIC_ROOT/build.sh'" \
            --title "FIC Control Studio — First Launch Build" 2>/dev/null; then
        BUILD_OK=1
    else
        BUILD_EXIT=$?
        if [ "$BUILD_EXIT" -eq 3 ]; then
            # PyQt5 not available — try tkinter progress dialog
            if python3 -c "import tkinter" &>/dev/null && [ -f "$INSTALLER_GUI" ]; then
                python3 "$INSTALLER_GUI" progress \
                    "FIC Control Studio — First Launch Build" \
                    "Building workspace..." \
                    "bash '$FIC_ROOT/build.sh'"
                [ -f "$WS_PATH/install/setup.bash" ] && BUILD_OK=1
            elif command -v zenity &>/dev/null; then
                (
                    echo "10"; sleep 0.5
                    bash "$FIC_ROOT/build.sh" 2>&1
                    echo "100"
                ) | zenity --progress --title="FIC Control Studio" \
                    --text="Building workspace for first launch...\nThis may take a few minutes." \
                    --pulsate --auto-close --no-cancel --width=400 2>/dev/null
                [ -f "$WS_PATH/install/setup.bash" ] && BUILD_OK=1
            else
                echo "Workspace not built. Building now (this may take a few minutes)..."
                bash "$FIC_ROOT/build.sh" && BUILD_OK=1
            fi
        fi
    fi

    if [ "$BUILD_OK" -ne 1 ] || [ ! -f "$WS_PATH/install/setup.bash" ]; then
        gui_error "FIC Control Studio" \
            "Build failed. Please run manually:\n\n$FIC_ROOT/build.sh"
        exit 1
    fi
fi

# ── Source workspace ──
source "$WS_PATH/install/setup.bash"

# ── Load user env vars (ROS_DOMAIN_ID, DDS config, etc.) ──
# Must come AFTER sourcing ROS2/workspace, which can reset env vars.
# ~/.bashrc's interactive guard blocks direct sourcing, so extract exports.
if [ -f "$HOME/.bashrc" ]; then
    eval "$(grep -E '^\s*export\s+ROS_DOMAIN_ID|^\s*export\s+DDS_|^\s*export\s+FASTRTPS_|^\s*export\s+CYCLONE_|^\s*export\s+RMW_' "$HOME/.bashrc" 2>/dev/null)" || true
fi

# ── Headless, if this machine was installed that way ──
# Written by the installer (Install_FIC_Studio.desktop). Headless means the
# Studio draws nothing at all: no Qt window, no QtWebEngine, no WebGL. It still
# runs its ROS node, joins the fleet and serves the web interface, so you work
# on it from a browser on another PC.
#
# This is what a machine with no monitor should do. These robots have no screen
# on any output, so X runs on the "dummy" driver and Mesa falls back to
# llvmpipe; Chromium then blocklists WebGL and Three.js renders in software on
# the CPU. Measured on a lab robot (9 Oct 2026): the windowed Studio took 121%
# CPU with another 24% in QtWebEngineProc, against 5.4% headless, on a machine
# whose other job is a 1 kHz control loop. A browser on a PC that has a GPU
# renders the same interface properly and costs the robot nothing.
#
# QT_QPA_PLATFORM is how the Studio already detects this (qt_webengine.py,
# HEADLESS_PLATFORMS), so setting it is all that is needed.
FIC_INTERFACE_CONF="${XDG_CONFIG_HOME:-$HOME/.config}/fic-studio/interface.conf"
if [ -f "$FIC_INTERFACE_CONF" ]; then
    . "$FIC_INTERFACE_CONF"
fi
if [ "${FIC_HEADLESS:-0}" = "1" ] && [ -z "${QT_QPA_PLATFORM:-}" ]; then
    export QT_QPA_PLATFORM=offscreen
    echo "FIC Studio: headless (installed with no interface). Open the web" \
         "interface from another PC; re-run the installer to get a window."
fi

# ── Launch GUI ──
cd "$WS_PATH"
# One Studio per station. A window opened on a machine whose headless Studio
# runs as the user service fic-studio (the MCU board; a robot PC started over
# SSH) takes over from it: the service stops first, and an installed one (not
# a transient systemd-run unit, which cannot be started again) comes back when
# the window closes.
if [ -z "${QT_QPA_PLATFORM:-}" ] && command -v systemctl >/dev/null 2>&1 \
        && systemctl --user is-active --quiet fic-studio 2>/dev/null; then
    unit_file="$(systemctl --user show fic-studio -p FragmentPath --value 2>/dev/null)"
    echo "FIC Studio: stopping the headless Studio (fic-studio service) for this window."
    systemctl --user stop fic-studio
    case "$unit_file" in
        ""|/run/*) ;;
        *)
            ros2 run fic_studio fic_studio_pro --web-ui "$@"
            rc=$?
            echo "FIC Studio: window closed; starting the headless Studio again."
            systemctl --user start fic-studio
            exit $rc
            ;;
    esac
fi
# The app opens the web UI in its own Qt window (QtWebEngine), never in a
# browser. "$@" lets `--classic` (the "Classic interface" action) override it.
exec ros2 run fic_studio fic_studio_pro --web-ui "$@"
