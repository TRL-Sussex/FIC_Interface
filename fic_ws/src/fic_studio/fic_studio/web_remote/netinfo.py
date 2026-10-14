"""The addresses a phone could reach this Studio on."""
import ipaddress
import json
import os
import socket
import subprocess

#: Carrier-grade NAT range; Tailscale hands out addresses from it.
_CGNAT = ipaddress.ip_network('100.64.0.0/10')

#: Interfaces that never face a phone: container and VM bridges.
_SKIP_IFACE_PREFIXES = ('lo', 'docker', 'br-', 'veth', 'virbr', 'vmnet')


def _ip_command_addresses():
    """IPv4 addresses from iproute2, skipping loopback and virtual bridges."""
    out = subprocess.run(['ip', '-j', '-4', 'addr', 'show'],
                         capture_output=True, text=True, timeout=1.0).stdout
    found = []
    for iface in json.loads(out or '[]'):
        name = iface.get('ifname', '')
        if name.startswith(_SKIP_IFACE_PREFIXES):
            continue
        for info in iface.get('addr_info', []):
            ip = info.get('local')
            if ip and not ip.startswith('127.'):
                found.append(ip)
    return found


def _default_route_address():
    """Return the address the default route leaves from. No packet is sent."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect(('10.255.255.255', 1))
        return sock.getsockname()[0]


def network_label(ip):
    """Name the network an address is on: Tailscale for CGNAT, else LAN."""
    if ip == 'localhost':
        return 'this computer'
    try:
        addr = ipaddress.ip_address(ip)
    except ValueError:
        return 'LAN'
    if addr.is_loopback:
        return 'this computer'
    return 'Tailscale' if addr in _CGNAT else 'LAN'


def lan_addresses():
    """Reachable IPv4 addresses, private LAN first, then Tailscale, then public.

    iproute2 lists every interface; where it is missing (macOS) fall back to
    the default-route address. Never raises: an empty list means none found.

    FIC_WEB_REMOTE_HOSTS (comma-separated host names or addresses) replaces
    the detected ones. In a container the interfaces are the container's own,
    which a phone or the host's browser cannot reach, so the Docker image
    sets this to the addresses the port is published on.
    """
    override = os.environ.get('FIC_WEB_REMOTE_HOSTS', '')
    hosts = [h.strip() for h in override.split(',') if h.strip()]
    if hosts:
        return list(dict.fromkeys(hosts))

    found = []
    for probe in (_ip_command_addresses, lambda: [_default_route_address()]):
        try:
            found = probe()
        except Exception:
            found = []
        if found:
            break

    def rank(ip):
        addr = ipaddress.ip_address(ip)
        if addr in _CGNAT:
            return 1
        return 0 if addr.is_private else 2

    unique = list(dict.fromkeys(found))
    return sorted(unique, key=rank)


def published_port(bound):
    """Return the port browsers reach a server bound to ``bound`` on.

    FIC_WEB_REMOTE_PORT is the host port that the container's web-remote port
    (FIC_WEB_REMOTE_CONTAINER_PORT, default 8088) is published on, when that
    differs. Any other port, or no variable, is its own answer.
    """
    try:
        published = int(os.environ.get('FIC_WEB_REMOTE_PORT', '') or 0)
        container = int(os.environ.get('FIC_WEB_REMOTE_CONTAINER_PORT', '') or 8088)
    except ValueError:
        return bound
    return published if published and bound == container else bound
