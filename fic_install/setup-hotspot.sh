#!/bin/bash
# setup-hotspot.sh — make the board its own Wi-Fi access point, for the web remote.
#
#   bash fic_install/setup-hotspot.sh [--ssid NAME] [--password PW] [--now]
#   bash fic_install/setup-hotspot.sh --off      back to the saved Wi-Fi networks
#   bash fic_install/setup-hotspot.sh --remove   delete the hotspot profile
#
# Creates the NetworkManager profile "FIC-Hotspot" (WPA2, 2.4 GHz, the board is
# 10.42.0.1 and hands out addresses) with a higher autoconnect priority than the
# saved networks, so it comes up at every boot. Phones join it and open
#     https://10.42.0.1:8088
#
# NOTE: the board has ONE Wi-Fi radio. While the hotspot is up the board is not
# on any other Wi-Fi (no internet over Wi-Fi). Without --now nothing changes
# until the next boot; `--off` switches back to the saved networks.
set -u
IFACE="$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1; exit}')"
CON="FIC-Hotspot"
SSID="FIC-$(hostname)"
PW=""
NOW=0
while [ $# -gt 0 ]; do
    case "$1" in
        --ssid) SSID="$2"; shift ;;
        --password) PW="$2"; shift ;;
        --now) NOW=1 ;;
        --off)
            nmcli connection down "$CON" 2>/dev/null
            nmcli connection modify "$CON" connection.autoconnect no 2>/dev/null
            nmcli device wifi rescan 2>/dev/null; nmcli device connect "$IFACE"
            echo "Hotspot off (and not started at boot). Re-enable: $0 --now"; exit 0 ;;
        --remove) nmcli connection delete "$CON"; exit $? ;;
        *) echo "unknown option $1" >&2; exit 2 ;;
    esac
    shift
done
[ -n "$IFACE" ] || { echo "No Wi-Fi device found." >&2; exit 1; }
nmcli -f WIFI-PROPERTIES.AP device show "$IFACE" | grep -q yes || { echo "$IFACE cannot be an access point." >&2; exit 1; }
if [ -z "$PW" ]; then
    PW="$(nmcli -s -g 802-11-wireless-security.psk connection show "$CON" 2>/dev/null)"
    [ -n "$PW" ] || PW="$(tr -dc 'A-HJ-NP-Za-km-z2-9' < /dev/urandom | head -c 12)"
fi
[ ${#PW} -ge 8 ] || { echo "Password must be at least 8 characters." >&2; exit 1; }

nmcli connection delete "$CON" >/dev/null 2>&1
nmcli connection add type wifi ifname "$IFACE" con-name "$CON" ssid "$SSID" \
    802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
    wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp \
    wifi-sec.psk "$PW" ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled \
    connection.autoconnect yes connection.autoconnect-priority 100 >/dev/null || exit 1

echo "Hotspot profile '$CON' ready:"
echo "    Wi-Fi name : $SSID"
echo "    Password   : $PW"
echo "    Web remote : https://10.42.0.1:8088   (accept the self-signed certificate)"
if [ "$NOW" -eq 1 ]; then
    echo "Switching $IFACE to the hotspot now (this drops the board's current Wi-Fi) ..."
    nmcli connection up "$CON"
else
    echo "It starts at the next boot (or now: $0 --now). Back to normal Wi-Fi: $0 --off"
fi
