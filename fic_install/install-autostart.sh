#!/bin/bash
# install-autostart.sh — start FIC Control Studio at boot, headless.
#
#   bash fic_install/install-autostart.sh            install + enable (starts at next boot)
#   bash fic_install/install-autostart.sh --now      ... and start it now
#   bash fic_install/install-autostart.sh --remove   disable and remove
#
# Runs Studio as a systemd *user* service with Qt's offscreen platform: no
# monitor or desktop login needed. Its web remote (https://<board>:8088) is the
# way in — from a phone on the board's own Wi-Fi (setup-hotspot.sh) or the LAN.
# "Lingering" lets the user service start at boot without anyone logging in.
#
#   journalctl --user -u fic-studio -f         Studio's log (first web admin password)
#   systemctl --user stop|start fic-studio
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
FIC_ROOT="$(cd "$HERE/.." && pwd)"
UNIT_DIR="$HOME/.config/systemd/user"
UNIT="$UNIT_DIR/fic-studio.service"

if [ "${1:-}" = "--remove" ]; then
    systemctl --user disable --now fic-studio.service 2>/dev/null
    rm -f "$UNIT"
    systemctl --user daemon-reload
    echo "FIC Studio autostart removed."
    exit 0
fi

mkdir -p "$UNIT_DIR"
cat > "$UNIT" <<UNITEOF
[Unit]
Description=FIC Control Studio (headless, web remote on :8088)
After=default.target

[Service]
Type=simple
Environment=QT_QPA_PLATFORM=offscreen
Environment=FIC_ROOT=$FIC_ROOT
# The launcher sources ROS 2 and the workspace (building it on first run).
ExecStart=/bin/bash -lc 'exec "$FIC_ROOT/fic_install/fic-control-studio.sh"'
Restart=on-failure
RestartSec=5
KillMode=control-group
TimeoutStopSec=15

[Install]
WantedBy=default.target
UNITEOF

# Start at boot without a login session.
if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != "yes" ]; then
    loginctl enable-linger "$USER" 2>/dev/null || sudo loginctl enable-linger "$USER"
fi
systemctl --user daemon-reload
systemctl --user enable fic-studio.service
echo "FIC Studio will start at boot (user service fic-studio, headless)."
if [ "${1:-}" = "--now" ]; then
    systemctl --user restart fic-studio.service
    echo "Started. Log: journalctl --user -u fic-studio -f"
fi
