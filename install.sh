#!/bin/bash
# install.sh — FIC Control Studio installer, all platforms.
#
# Gets the code, then hands off to the platform installer, which detects the
# machine (fic_install/detect-system.sh) and installs ROS 2, every dependency,
# builds the workspace and registers the apps:
#   macOS (Apple Silicon) → fic_install/macos/install-fic-studio-macos.sh
#                           (RoboStack conda env + native CPU MuJoCo backend)
#   Ubuntu 22.04 / 24.04  → Install_FIC_Studio.desktop
#     amd64 or arm64        (ROS 2 Humble / Jazzy, apt deps, build, launchers;
#                            Arduino UNO Q / VENTUNO Q boards included)
#
# Usage:
#   ./install.sh                  install from this checkout
#   ./install.sh --update         git pull this checkout first
#   ./install.sh --dry-run        show the system and the plan, change nothing
#   ./install.sh --yes            no questions (terminal / scripted installs)
#   bash install.sh               run on its own (no checkout next to it):
#                                 clones FIC_REPO_URL into FIC_ROOT first
#
#   FIC_REPO_URL  default https://github.com/TRL-Sussex/FIC_Interface.git
#   FIC_BRANCH    branch to clone (default: the repository's default branch)
#   FIC_ROOT      where to clone (default ~/FIC_Interface)
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
FIC_REPO_URL="${FIC_REPO_URL:-https://github.com/TRL-Sussex/FIC_Interface.git}"

UPDATE=0
PASS=()
for a in "$@"; do
    case "$a" in
        --update) UPDATE=1 ;;
        *) PASS+=("$a") ;;
    esac
done

# ── Get the code ──
if [ ! -d "$HERE/fic_ws" ] || [ ! -f "$HERE/Install_FIC_Studio.desktop" ]; then
    # Run outside a checkout (e.g. downloaded on its own): clone one.
    TARGET="${FIC_ROOT:-$HOME/FIC_Interface}"
    command -v git >/dev/null 2>&1 || {
        echo "git is required to download FIC Studio:  sudo apt install git" >&2
        exit 1
    }
    if [ -d "$TARGET/.git" ]; then
        echo "Using the existing checkout at $TARGET"
        UPDATE=1
    else
        echo "Downloading FIC Studio into $TARGET ..."
        git clone ${FIC_BRANCH:+--branch "$FIC_BRANCH"} "$FIC_REPO_URL" "$TARGET" || {
            echo "Clone failed. Check the network and access to $FIC_REPO_URL." >&2
            exit 1
        }
    fi
    HERE="$TARGET"
fi

if [ "$UPDATE" -eq 1 ] && [ -d "$HERE/.git" ]; then
    echo "Updating $HERE ($(git -C "$HERE" rev-parse --abbrev-ref HEAD)) ..."
    if ! git -C "$HERE" diff --quiet || ! git -C "$HERE" diff --cached --quiet; then
        echo "  Local changes present — not pulling. Commit or stash them, then re-run with --update." >&2
    else
        git -C "$HERE" pull --ff-only || echo "  git pull failed; installing the code as it is." >&2
    fi
fi

# ── Hand off to the platform installer ──
# ${PASS[@]+"${PASS[@]}"}: macOS's bash 3.2 calls an empty "${PASS[@]}" unbound under set -u.
case "$(uname -s)" in
    Darwin)
        exec zsh "$HERE/fic_install/macos/install-fic-studio-macos.sh" ${PASS[@]+"${PASS[@]}"}
        ;;
    Linux)
        exec bash "$HERE/Install_FIC_Studio.desktop" ${PASS[@]+"${PASS[@]}"}
        ;;
    *)
        echo "Unsupported OS: $(uname -s)." >&2
        echo "Supported: macOS (Apple Silicon) and Ubuntu Linux 22.04/24.04 (amd64, arm64)." >&2
        exit 1
        ;;
esac
