#!/usr/bin/env bash
# ============================================================================
# adb for a machine whose apt cannot install it: the distribution's adb with
# its own Android libraries, in a folder of its own.
#
# On the Arduino Ventuno Q the board's adbd (its own USB debug link, used by
# App Lab) pins Arduino's android-libcutils; Ubuntu's adb pins another version,
# so `apt-get install adb` would remove adbd, and apt refuses. This unpacks
# adb and the libraries of its version (android-libcutils, -libbase,
# -libziparchive) into <prefix>/lib/fic-adb and puts a small `adb` in
# <prefix>/bin that points it at them. The system's packages are not touched.
#
#   fic_install/adb-private.sh            # as root: /usr/local; as a user: ~/.local
#   fic_install/adb-private.sh <prefix>
#
# Afterwards: adb version.
# ============================================================================
set -euo pipefail

if [ -n "${1:-}" ]; then PREFIX="$1"
elif [ "$(id -u)" -eq 0 ]; then PREFIX=/usr/local
else PREFIX="$HOME/.local"; fi
HERE="$PREFIX/lib/fic-adb"

VERSION=$(apt-cache policy adb 2>/dev/null | awk '/Candidate:/ {print $2}')
if [ -z "$VERSION" ] || [ "$VERSION" = "(none)" ]; then
    echo "adb-private: apt has no adb package for this system" >&2
    exit 1
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
(cd "$WORK" && apt-get download -q "adb=$VERSION" "android-libcutils=$VERSION" \
    "android-libbase=$VERSION" "android-libziparchive=$VERSION")

rm -rf "$HERE"
mkdir -p "$HERE" "$PREFIX/bin"
for deb in "$WORK"/*.deb; do dpkg-deb -x "$deb" "$HERE"; done
BIN=$(find "$HERE" -path '*platform-tools/adb' -type f | head -1)
LIBS=$(dirname "$(find "$HERE" -name 'libcutils.so.0' | head -1)")
[ -x "$BIN" ] && [ -d "$LIBS" ] || { echo "adb-private: unexpected package layout" >&2; exit 1; }

cat > "$PREFIX/bin/adb" <<EOF
#!/bin/sh
# FIC: adb $VERSION with its own Android libraries (fic_install/adb-private.sh).
LD_LIBRARY_PATH="$LIBS\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}" exec "$BIN" "\$@"
EOF
chmod 755 "$PREFIX/bin/adb"
# USB access to Android devices for the user at the screen (udev rules; the apt
# adb would have brought them). Only as root; it clashes with nothing.
if [ "$(id -u)" -eq 0 ]; then
    apt-get -o DPkg::Lock::Timeout=900 install -y android-sdk-platform-tools-common || \
        echo "adb-private: no udev rules for Android devices (adb may say 'no permissions')"
fi
"$PREFIX/bin/adb" version | head -2
echo "adb-private: $PREFIX/bin/adb"
