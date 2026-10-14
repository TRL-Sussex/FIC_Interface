#!/bin/zsh
# ============================================================================
#  FIC Studio — macOS (Apple Silicon) installer
# ----------------------------------------------------------------------------
#  Sets up everything needed to run FIC Studio SIMULATION natively on macOS:
#    1. Homebrew, if it is installed (it is optional)
#    2. Miniforge (conda/mamba) — through Homebrew if there, else directly
#    3. A RoboStack ROS 2 Humble conda env `fic_ros` with all build/sim deps
#    4. Builds the ROS 2 workspace (native mujoco_native backend)
#    5. Generates double-clickable .command launchers
#
#  This installer is idempotent — safe to re-run. It never uses sudo; all
#  installs are user-scoped (conda, and Homebrew if present).
#
#  Usage:   zsh install-fic-studio-macos.sh
#  Override repo location:   FIC_ROOT=/path/to/FIC_Interface zsh install-fic-studio-macos.sh
# ============================================================================
set -e

ENV_NAME="fic_ros"
ROS_DISTRO="humble"

# --- Resolve FIC_ROOT (repo root) relative to this script, unless overridden.
SCRIPT_DIR="${0:A:h}"
FIC_ROOT="${FIC_ROOT:-${SCRIPT_DIR:h:h}}"   # macos/ -> fic_install/ -> repo root
WS_PATH="$FIC_ROOT/fic_ws"

echo "════════════════════════════════════════════════════════════════"
echo "  FIC Studio — macOS installer"
echo "  Repo:      $FIC_ROOT"
echo "  Workspace: $WS_PATH"
echo "════════════════════════════════════════════════════════════════"

if [[ "$(uname -m)" != "arm64" ]]; then
  echo "Warning: This installer targets Apple Silicon (arm64). Detected: $(uname -m)."
  echo "    It may still work on Intel Macs but is untested there."
fi

# --- Guard: the repo must NOT live under a TCC-protected folder ------------
# macOS blocks app bundles (Finder/Launchpad launches) from READING files in
# ~/Documents, ~/Desktop and ~/Downloads. Since we ship a double-clickable
# .app that reads the workspace at launch, a repo under those folders makes the
# app "open then immediately crash" (Terminal launches still work, because they
# inherit Terminal's file access). Refuse early with a clear fix.
for _prot in "$HOME/Documents" "$HOME/Desktop" "$HOME/Downloads"; do
  if [[ "$FIC_ROOT" == "$_prot"* ]]; then
    echo ""
    echo "✗ The repo is inside a macOS privacy-protected folder:"
    echo "     $FIC_ROOT"
    echo "   App-bundle launches can't read files there, so the FIC Studio app"
    echo "   would crash on open. Move the repo somewhere NOT under"
    echo "   ~/Documents, ~/Desktop or ~/Downloads, e.g.:"
    echo ""
    echo "     mv \"$FIC_ROOT\" \"$HOME/$(basename "$FIC_ROOT")\""
    echo "     zsh \"$HOME/$(basename "$FIC_ROOT")/fic_install/macos/install-fic-studio-macos.sh\""
    echo ""
    echo "   (A clean rebuild from the new location is required — colcon bakes"
    echo "    absolute paths into the build. This installer does that for you.)"
    exit 1
  fi
done

# --- 1-2. Miniforge (conda/mamba) ------------------------------------------
# Homebrew is optional: without it, Miniforge is installed straight into
# ~/miniforge3 from its official installer (user-scoped, no sudo).
MINIFORGE_DIR="$HOME/miniforge3"
if ! command -v conda >/dev/null 2>&1 && [[ ! -f "$MINIFORGE_DIR/etc/profile.d/conda.sh" ]]; then
  if command -v brew >/dev/null 2>&1; then
    echo "→ Installing Miniforge via Homebrew…"
    brew install --cask miniforge
  else
    echo "→ Installing Miniforge into $MINIFORGE_DIR…"
    MF_SH="$(mktemp -t miniforge).sh"
    curl -fsSL -o "$MF_SH" \
      "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-MacOSX-arm64.sh"
    bash "$MF_SH" -b -p "$MINIFORGE_DIR"
    rm -f "$MF_SH"
  fi
fi
# Locate and source conda for this shell.
CONDA_BASE=""
command -v brew >/dev/null 2>&1 && CONDA_BASE="$(brew --prefix)/Caskroom/miniforge/base"
[[ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]] || CONDA_BASE="$MINIFORGE_DIR"
[[ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]] || CONDA_BASE="$(conda info --base 2>/dev/null)"
source "$CONDA_BASE/etc/profile.d/conda.sh"
echo "✓ conda: $(conda --version)  (base: $CONDA_BASE)"

# --- 3. RoboStack ROS 2 env + dependencies ---------------------------------
export CONDA_CHANNEL_PRIORITY=flexible
if conda env list | grep -qE "^\s*${ENV_NAME}\s"; then
  echo "✓ conda env '${ENV_NAME}' already exists — ensuring deps are present."
else
  echo "→ Creating conda env '${ENV_NAME}' with ROS 2 ${ROS_DISTRO} (this downloads a few GB)…"
  mamba create -n "$ENV_NAME" -y -c conda-forge -c robostack-staging "ros-${ROS_DISTRO}-desktop"
fi

echo "→ Installing build + simulation + FIC dependencies…"
# NOTE: numpy>=2 is REQUIRED — RoboStack ros-humble on osx-arm64 is built
# against NumPy 2.x; pinning numpy<2 breaks the `communication` package build.
#       setuptools<80 is REQUIRED — setuptools 80 removed the legacy
#       `setup.py develop/install --uninstall` commands that colcon's
#       ament_python build (and the GUI's Rebuild buttons) depend on.
#       pyobjc-framework-cocoa lets script-launched GUI helpers (e.g. the
#       plotter) promote themselves to a foreground app so their window shows.
#       pyqtwebengine (conda-forge, matches the env's qt-main) runs the web UI
#       that the app opens with `--web-ui`.
mamba install -n "$ENV_NAME" -y -c conda-forge -c robostack-staging \
  compilers cmake ninja pkg-config colcon-common-extensions \
  "ros-${ROS_DISTRO}-ament-cmake" eigen tinyxml2 yaml-cpp pinocchio \
  pyqt pyqtgraph "numpy>=2" pyyaml scipy cryptography matplotlib "mujoco>=3.3" "setuptools<80" \
  pyobjc-framework-cocoa pyqtwebengine

conda activate "$ENV_NAME"
echo "✓ Environment ready: python $(python --version 2>&1 | awk '{print $2}'), ROS_DISTRO=$ROS_DISTRO"

# --- 4. Build the workspace ------------------------------------------------
# Build the whole workspace (every package builds on macOS since the C++
# `mujoco` simulator, which linked the Linux-only librt, was retired in v4.5.0).
echo "→ Building the ROS 2 workspace (colcon)…"
cd "$WS_PATH"
# NOTE: we deliberately do NOT pass --symlink-install. Modern setuptools (in the
# RoboStack env) dropped `setup.py develop --editable`, which colcon's symlink
# install for ament_python relies on, so it errors out. The packages are instead
# copy-installed; fic_studio/config.py resolves the repo root from the `fic_ws`
# path component (not a fixed walk), so SIM + rebuild buttons work regardless.
#
# LIBRARY_PATH: conda's `rcutils` exports a bare `-latomic` that every dependent
# target inherits. libatomic.dylib is in the env, but colcon links everything else
# by absolute path, so the link line has no `-L` and Apple's ld cannot find it:
#     ld: library 'atomic' not found
LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}" \
colcon build \
  --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" -DBUILD_TESTING=OFF

if grep -q "pinocchio not found" "$WS_PATH/log/latest_build/"*/stderr.log 2>/dev/null; then
  echo "Warning: Pinocchio was NOT found — Cartesian control will be disabled."
else
  echo "✓ Build complete (Pinocchio found → Cartesian control enabled)."
fi

# --- 5. Generate .command launchers ---------------------------------------
echo "→ Generating launchers…"
MACOS_DIR="$FIC_ROOT/fic_install/macos"
make_launcher() {
  local file="$1" entry="$2" title="$3" args="${4:-}"
  cat > "$file" <<LAUNCHER
#!/bin/zsh
# $title — double-click to launch (auto-generated by install-fic-studio-macos.sh)
# FIC_ROOT pins the repo root so the GUI resolves fic_ws/, robot_library/, etc.
# correctly regardless of how the workspace was built.
export FIC_ROOT="$FIC_ROOT"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate $ENV_NAME
cd "$WS_PATH"
source install/setup.zsh
exec ros2 run fic_studio $entry $args
LAUNCHER
  chmod +x "$file"
  echo "   • $file"
}
make_launcher "$MACOS_DIR/FIC Control Studio.command" "fic_studio_pro" "FIC Control Studio" "--web-ui"
make_launcher "$MACOS_DIR/FIC Control Studio (Classic).command" "fic_studio_pro" "FIC Control Studio (Classic)" "--classic"
make_launcher "$MACOS_DIR/FIC Dev Console.command"    "fic_dev"        "FIC Dev Console"

# --- 6. Build real macOS .app bundles in ~/Applications --------------------
# A proper .app makes FIC Studio show up in Launchpad, Spotlight and the Dock
# with the FIC icon, launching the web UI directly (no Terminal window). The
# .command files above remain as a terminal-visible fallback for viewing logs.
echo "→ Building macOS app bundles…"
APP_DIR="$HOME/Applications"
mkdir -p "$APP_DIR"

# 6a. Render the SVG icon into a real .icns (PyQt5 + built-in sips/iconutil).
#     Best-effort: if anything fails the apps still build, just with a generic
#     icon. Uses the env's PyQt5 (offscreen) — no extra tools to install.
ICON_SVG="$FIC_ROOT/fic_install/fic-control-studio.svg"
ICNS_PATH="$(mktemp -d)/AppIcon.icns"
build_icns() {
  local svg="$1" out="$2"
  [[ -f "$svg" ]] || return 1
  local iconset="$(mktemp -d)/AppIcon.iconset"
  mkdir -p "$iconset"
  QT_QPA_PLATFORM=offscreen python - "$svg" "$iconset" <<'PY' || return 1
import os, sys
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5.QtGui import QGuiApplication, QImage, QPainter
from PyQt5.QtCore import Qt
from PyQt5.QtSvg import QSvgRenderer
svg, iconset = sys.argv[1], sys.argv[2]
app = QGuiApplication([])
renderer = QSvgRenderer(svg)
if not renderer.isValid():
    sys.exit("invalid svg")
specs = [(16, "icon_16x16.png"), (32, "icon_16x16@2x.png"),
         (32, "icon_32x32.png"), (64, "icon_32x32@2x.png"),
         (128, "icon_128x128.png"), (256, "icon_128x128@2x.png"),
         (256, "icon_256x256.png"), (512, "icon_256x256@2x.png"),
         (512, "icon_512x512.png"), (1024, "icon_512x512@2x.png")]
for size, name in specs:
    img = QImage(size, size, QImage.Format_ARGB32)
    img.fill(Qt.transparent)
    p = QPainter(img)
    renderer.render(p)
    p.end()
    if not img.save(os.path.join(iconset, name)):
        sys.exit("save failed: " + name)
PY
  iconutil -c icns "$iconset" -o "$out"
}
if build_icns "$ICON_SVG" "$ICNS_PATH"; then
  echo "   ✓ Icon generated from fic-control-studio.svg"
else
  echo "   Warning: Could not generate .icns — apps will use a generic icon."
  ICNS_PATH=""
fi

# 6b. Assemble each .app bundle (Info.plist + launcher executable + icon).
make_app() {
  local title="$1" entry="$2" bundle_id="$3" args="${4:-}"
  local app="$APP_DIR/$title.app"
  rm -rf "$app"
  mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources"
  cat > "$app/Contents/MacOS/launch" <<LAUNCH
#!/bin/zsh
# $title — launched by macOS (LaunchServices), so it inherits no shell env;
# everything the GUI needs is sourced explicitly here.
export FIC_ROOT="$FIC_ROOT"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate $ENV_NAME
cd "$WS_PATH"
source install/setup.zsh
exec ros2 run fic_studio $entry $args >"/tmp/$entry.log" 2>&1
LAUNCH
  chmod +x "$app/Contents/MacOS/launch"
  [[ -n "$ICNS_PATH" && -f "$ICNS_PATH" ]] && cp "$ICNS_PATH" "$app/Contents/Resources/AppIcon.icns"
  cat > "$app/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>$title</string>
  <key>CFBundleDisplayName</key><string>$title</string>
  <key>CFBundleIdentifier</key><string>$bundle_id</string>
  <key>CFBundleExecutable</key><string>launch</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleInfoDictionaryVersion</key><string>6.0</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>CFBundleVersion</key><string>1.0</string>
  <key>LSMinimumSystemVersion</key><string>13.0</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>LSApplicationCategoryType</key><string>public.app-category.developer-tools</string>
</dict></plist>
PLIST
  # Ad-hoc sign so the bundle has a stable identity (smoother Gatekeeper "Open"
  # on first launch, and a consistent code identity for any TCC grants).
  codesign --force --deep --sign - "$app" >/dev/null 2>&1 || true
  touch "$app"
  echo "   • $app"
}
make_app "FIC Control Studio" "fic_studio_pro" "ai.fic.control-studio" "--web-ui"
make_app "FIC Dev Console"     "fic_dev"        "ai.fic.dev-console"

# 6c. Nudge LaunchServices so Finder/Spotlight pick up the new apps + icons now.
LSREGISTER="/System/Library/Frameworks/CoreServices.framework/Frameworks/LaunchServices.framework/Support/lsregister"
[[ -x "$LSREGISTER" ]] && "$LSREGISTER" -f "$APP_DIR/FIC Control Studio.app" "$APP_DIR/FIC Dev Console.app" 2>/dev/null || true

echo "════════════════════════════════════════════════════════════════"
echo "  ✓ Installation complete."
echo ""
echo "  FIC Studio is now a macOS app:"
echo "     • $APP_DIR/FIC Control Studio.app"
echo "     • $APP_DIR/FIC Dev Console.app"
echo "  Find it in Launchpad / Spotlight (⌘Space → \"FIC\"), or drag it to the Dock."
echo ""
echo "  Terminal-visible fallback (shows logs):"
echo "     $MACOS_DIR/FIC Control Studio.command"
echo ""
echo "  Or run directly:"
echo "     conda activate $ENV_NAME"
echo "     cd $WS_PATH && source install/setup.zsh"
echo "     ros2 run fic_studio fic_studio_pro --web-ui    (--classic for the Qt UI)"
echo ""
echo "  In the GUI: pick a robot, press SIM then FIC, choose a control mode."
echo "════════════════════════════════════════════════════════════════"
