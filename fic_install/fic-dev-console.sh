#!/bin/bash
# fic-dev-console.sh — Launch FIC Dev Console (lightweight hardware teleop GUI)
# This script sources the ROS2 workspace and runs the dev console entry point.

set -e

# ── Configuration ──
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FIC_ROOT="${FIC_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
WS_PATH="$FIC_ROOT/fic_ws"
BUILD_GUI="$SCRIPT_DIR/build_gui.py"
INSTALLER_GUI="$SCRIPT_DIR/installer_gui.py"

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
        gui_error "FIC Dev Console" "No ROS2 installation found in /opt/ros/"
        exit 1
    fi
fi
echo "Using ROS2 distro: $ROS_DISTRO"

# ── Build workspace if needed ──
if [ ! -f "$WS_PATH/install/setup.bash" ]; then
    BUILD_OK=0

    # Try PyQt5 build dialog first
    if python3 "$BUILD_GUI" "$WS_PATH" "bash '$FIC_ROOT/build.sh'" \
            --title "FIC Dev Console — First Launch Build" 2>/dev/null; then
        BUILD_OK=1
    else
        BUILD_EXIT=$?
        if [ "$BUILD_EXIT" -eq 3 ]; then
            # PyQt5 not available — try tkinter progress dialog
            if python3 -c "import tkinter" &>/dev/null && [ -f "$INSTALLER_GUI" ]; then
                python3 "$INSTALLER_GUI" progress \
                    "FIC Dev Console — First Launch Build" \
                    "Building workspace..." \
                    "bash '$FIC_ROOT/build.sh'"
                [ -f "$WS_PATH/install/setup.bash" ] && BUILD_OK=1
            elif command -v zenity &>/dev/null; then
                (
                    echo "10"; sleep 0.5
                    bash "$FIC_ROOT/build.sh" 2>&1
                    echo "100"
                ) | zenity --progress --title="FIC Dev Console" \
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
        gui_error "FIC Dev Console" \
            "Build failed. Please run manually:\n\n$FIC_ROOT/build.sh"
        exit 1
    fi
fi

# ── Source workspace ──
source "$WS_PATH/install/setup.bash"

# ── Load user env vars (ROS_DOMAIN_ID, DDS config, etc.) ──
# Must come AFTER sourcing ROS2/workspace, which can reset env vars.
if [ -f "$HOME/.bashrc" ]; then
    eval "$(grep -E '^\s*export\s+ROS_DOMAIN_ID|^\s*export\s+DDS_|^\s*export\s+FASTRTPS_|^\s*export\s+CYCLONE_|^\s*export\s+RMW_' "$HOME/.bashrc" 2>/dev/null)" || true
fi

# ── Launch Dev Console ──
cd "$WS_PATH"
exec ros2 run fic_studio fic_dev
