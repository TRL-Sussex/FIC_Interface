#!/bin/bash
# install-desktop-entry.sh — Install FIC Control Studio as an Ubuntu app
# Run once: bash install-desktop-entry.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
USER_HOME="$HOME"
FIC_ROOT="${FIC_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
LAUNCH_SCRIPT="$SCRIPT_DIR/fic-control-studio.sh"
ICON_FILE="$SCRIPT_DIR/fic-control-studio.svg"
DESKTOP_FILE="$USER_HOME/.local/share/applications/fic-control-studio.desktop"

echo "=== Installing FIC Control Studio Desktop Entry ==="

# 1. Ensure launch script is executable
chmod +x "$LAUNCH_SCRIPT"
echo "  ✓ Launch script → $LAUNCH_SCRIPT"

# 2. Icon already at $ICON_FILE
echo "  ✓ Icon → $ICON_FILE"

# 2. Create .desktop file with correct home path
mkdir -p "$USER_HOME/.local/share/applications"
sed "s|__FIC_INSTALL__|$SCRIPT_DIR|g" "$SCRIPT_DIR/fic-control-studio.desktop" > "$DESKTOP_FILE"
chmod +x "$DESKTOP_FILE"
echo "  ✓ Desktop entry → $DESKTOP_FILE"

# 3. Update desktop database
update-desktop-database "$USER_HOME/.local/share/applications" 2>/dev/null || true
echo "  ✓ Desktop database updated"

echo ""
echo "Done! 'FIC Control Studio' should now appear in your app launcher."
echo "If not visible immediately, log out and back in."
