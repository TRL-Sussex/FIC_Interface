#!/usr/bin/env bash
# ============================================================================
# Put this board's Linux side on the arm's network (192.168.1.0/24) — the
# "one switch" layout: Kinova arm, the MCU's Ethernet shield and this board on
# one switch, so Linux reaches the arm (camera stream, setup, homing) and the
# MCU (FIC-link over UDP) directly.
#
#   fic_install/setup-arm-network.sh              # pick the wired port with a link
#   fic_install/setup-arm-network.sh enx00e04c..  # or name it
#   fic_install/setup-arm-network.sh --dry-run    # only show what it would do
#
# Gives that port 192.168.1.30/24 with no default route (Wi-Fi keeps the
# internet) through a NetworkManager profile "FIC arm network (<port>)".
# The board's own port (end0) only links at 100 Mbit and not with every
# gigabit switch; a USB Ethernet adapter (enx...) is the reliable choice.
# Addresses: arm 192.168.1.10, MCU shield 192.168.1.20, this board .30.
# Check afterwards: robot_library/fic_mcu/scripts/check_switch_layout.sh
# ============================================================================
set -euo pipefail

ADDR="192.168.1.30/24"
DRY=0
IFACE=""
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) IFACE="$a" ;;
  esac
done

wired() {   # wired Ethernet ports (USB adapters first), not CAN/Wi-Fi/virtual
  for d in /sys/class/net/*; do
    n="$(basename "$d")"
    [ -e "$d/device" ] || continue
    [ "$(cat "$d/type" 2>/dev/null)" = "1" ] || continue
    [ -d "$d/wireless" ] && continue
    echo "$n"
  done | sort -r   # enx* sorts before end0 in reverse
}

if [ -z "$IFACE" ]; then
  for n in $(wired); do
    [ "$(cat /sys/class/net/$n/carrier 2>/dev/null)" = "1" ] && { IFACE="$n"; break; }
  done
fi
if [ -z "$IFACE" ]; then
  echo "No wired port has a link. Plug this board (a USB Ethernet adapter, or end0) into the switch," >&2
  echo "or name the port: $0 <port>. Wired ports here: $(wired | tr '\n' ' ')" >&2
  exit 1
fi
[ -e "/sys/class/net/$IFACE" ] || { echo "No such port: $IFACE" >&2; exit 1; }

NAME="FIC arm network ($IFACE)"
echo "Port:    $IFACE (link: $(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo ?))"
echo "Address: $ADDR, no default route; profile \"$NAME\""

run() { if [ "$DRY" -eq 1 ]; then echo "  would run: $*"; else "$@"; fi; }

if nmcli -t -f NAME connection show | grep -qxF "$NAME"; then
  run nmcli connection modify "$NAME" ipv4.method manual ipv4.addresses "$ADDR" ipv4.never-default yes ipv6.method disabled
else
  run nmcli connection add type ethernet ifname "$IFACE" con-name "$NAME" \
      ipv4.method manual ipv4.addresses "$ADDR" ipv4.never-default yes ipv6.method disabled \
      connection.autoconnect yes connection.autoconnect-priority 50
fi
run nmcli connection up "$NAME"
[ "$DRY" -eq 1 ] || echo "Done. Now: robot_library/fic_mcu/scripts/check_switch_layout.sh"
