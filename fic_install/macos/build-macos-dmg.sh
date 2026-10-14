#!/bin/zsh
# ============================================================================
#  build-macos-dmg.sh — package FIC Studio into a distributable macOS .dmg
# ----------------------------------------------------------------------------
#  Produces  fic_install/macos/dist/FIC-Studio-Installer.dmg  — a read-only
#  disk image a user can download, mount, and double-click to install.
#
#  The image bundles the REPO SOURCE (not the ~4.5 GB conda env — deps are
#  downloaded at install time). Because a DMG is read-only and the workspace
#  must be BUILT (which writes into the tree), the bundled installer first
#  COPIES the source to a writable, NON-privacy-protected location
#  (default ~/FIC_Interface) and builds there. That copy step is also what
#  keeps the repo out of ~/Documents/~/Desktop/~/Downloads, where macOS TCC
#  would stop the resulting .app from launching.
#
#  Usage:   zsh fic_install/macos/build-macos-dmg.sh
#  Requires: hdiutil (built into macOS). No sudo.
# ============================================================================
set -e

SCRIPT_DIR="${0:A:h}"
FIC_ROOT="${SCRIPT_DIR:h:h}"                 # macos/ -> fic_install/ -> repo root
DIST_DIR="$SCRIPT_DIR/dist"
VOL_NAME="FIC Studio Installer"
DMG_PATH="$DIST_DIR/FIC-Studio-Installer.dmg"
SRC_SUBDIR=".fic_studio_source"              # hidden inside the DMG (clean window)

echo "════════════════════════════════════════════════════════════════"
echo "  Building FIC Studio macOS installer image"
echo "  Repo:   $FIC_ROOT"
echo "  Output: $DMG_PATH"
echo "════════════════════════════════════════════════════════════════"

command -v hdiutil >/dev/null 2>&1 || { echo "✗ hdiutil not found (needs macOS)."; exit 1; }

# --- 1. Stage the image contents in a temp dir ----------------------------
STAGE="$(mktemp -d)/dmg"
mkdir -p "$STAGE/$SRC_SUBDIR"
echo "→ Copying repo source (excluding .git / build artifacts)…"
# rsync keeps it fast and lets us prune the heavy, non-portable bits.
rsync -a \
  --exclude='.git' \
  --exclude='fic_ws/build' \
  --exclude='fic_ws/install' \
  --exclude='fic_ws/log' \
  --exclude='**/__pycache__' \
  --exclude='.DS_Store' \
  --exclude='fic_install/macos/dist' \
  --exclude='fic_install/macos/*.command' \
  "$FIC_ROOT/" "$STAGE/$SRC_SUBDIR/"

# --- 2. Bootstrap installer the user double-clicks ------------------------
# A .command opens in Terminal, so the multi-GB dependency download shows live
# progress (a .pkg wizard would look frozen). It copies the bundled source to a
# writable, non-protected destination and hands off to the real installer.
cat > "$STAGE/Install FIC Studio.command" <<'BOOT'
#!/bin/zsh
set -e
HERE="${0:A:h}"
SRC="$HERE/.fic_studio_source"
DEFAULT_DEST="$HOME/FIC_Interface"

clear
echo "════════════════════════════════════════════════════════════════"
echo "  FIC Studio — macOS Installer"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "This installs FIC Studio (simulation) natively on Apple Silicon."
echo "It downloads ROS 2 + dependencies (~a few GB) — internet required."
echo ""

# Destination must be writable and NOT under a TCC-protected folder, or the
# resulting .app can't read its own files and would crash on open.
DEST="$DEFAULT_DEST"
echo "Install location [$DEST]"
echo -n "Press Return to accept, or type a different path: "
read REPLY
[[ -n "$REPLY" ]] && DEST="${REPLY/#\~/$HOME}"

for p in "$HOME/Documents" "$HOME/Desktop" "$HOME/Downloads"; do
  if [[ "$DEST" == "$p"* ]]; then
    echo ""
    echo "✗ $DEST is inside a macOS privacy-protected folder."
    echo "   App launches can't read files there. Choose somewhere else"
    echo "   (e.g. $HOME/FIC_Interface) and run this installer again."
    echo ""; echo "Press Return to close."; read _; exit 1
  fi
done

if [[ -e "$DEST" ]]; then
  echo ""
  echo "Warning: $DEST already exists — it will be updated in place."
  echo -n "Continue? [y/N] "; read ans
  [[ "$ans" == [yY]* ]] || { echo "Aborted."; echo "Press Return to close."; read _; exit 1; }
fi

echo ""
echo "→ Copying FIC Studio to $DEST …"
mkdir -p "$DEST"
rsync -a "$SRC/" "$DEST/"

echo "→ Launching the installer…"
echo ""
exec zsh "$DEST/fic_install/macos/install-fic-studio-macos.sh"
BOOT
chmod +x "$STAGE/Install FIC Studio.command"

# --- 3. A short read-me shown in the mounted image ------------------------
cat > "$STAGE/READ ME FIRST.txt" <<'RM'
FIC Studio — macOS Installer
============================

1. Double-click  "Install FIC Studio.command".
   (First time: if macOS blocks it, right-click it -> Open -> Open.)

2. A Terminal window opens. Accept the install location (default:
   ~/FIC_Interface) or type another. Do NOT install into Documents,
   Desktop or Downloads — macOS privacy rules stop the app from
   reading files there.

3. The installer downloads ROS 2 + dependencies (a few GB, needs
   internet), builds the workspace, and creates the app:

       ~/Applications/FIC Control Studio.app
       ~/Applications/FIC Dev Console.app

   Find them in Launchpad / Spotlight, or drag to the Dock.

Requirements: macOS 13+ on Apple Silicon, Homebrew (brew.sh), ~10 GB free.

This installs into your user account only — no admin password needed.
RM

# --- 4. Build the compressed, read-only DMG -------------------------------
echo "→ Creating the disk image…"
mkdir -p "$DIST_DIR"
rm -f "$DMG_PATH"
hdiutil create \
  -volname "$VOL_NAME" \
  -srcfolder "$STAGE" \
  -fs HFS+ \
  -format UDZO \
  -imagekey zlib-level=9 \
  -ov "$DMG_PATH" >/dev/null

echo "════════════════════════════════════════════════════════════════"
echo "  ✓ Built: $DMG_PATH"
echo "     size:  $(du -h "$DMG_PATH" | awk '{print $1}')"
echo ""
echo "  Share this .dmg. Recipients mount it and double-click"
echo "  \"Install FIC Studio.command\". (Downloaded copies are quarantined —"
echo "   right-click -> Open the first time.)"
echo "════════════════════════════════════════════════════════════════"
