#!/usr/bin/env python3
"""
optitrack_windows_sender.py — Bridge OptiTrack/Motive rigid bodies from
Windows (no ROS2 required) to FIC Studio's OptiTrack panel via UDP.

The FIC Studio OptiTrack panel (Source = "UDP (Windows Python)") binds a
UDP socket on the Linux robot machine and consumes JSON packets in the
shape below.  Fields are optional — if a rigid body isn't tracked in a
given frame, leave it out and FIC Studio will keep the last value.

Run:
    python optitrack_windows_sender.py ^
        --host 192.168.1.42 ^
        --port 5555 ^
        --ee-rb-id 1 ^
        --f1-rb-id 2 ^
        --f2-rb-id 3

Requirements:
    - Motive streaming enabled (Data Streaming pane in Motive)
    - NatNet SDK 4.x (`pip install NatNetClient` or use the SDK's bundled
      NatNetClient.py in the same folder)

Protocol (one datagram per frame):
    {
        "ee": {"pos": [x, y, z], "quat": [qx, qy, qz, qw]},
        "f1": {"pos": [x, y, z], "quat": [qx, qy, qz, qw]},
        "f2": {"pos": [x, y, z], "quat": [qx, qy, qz, qw]}
    }

Units: meters and unit quaternion.  Frame: whatever Motive streams
(typically Y-up meters).  FIC Studio handles axis remap + origin offset.
"""

import argparse
import json
import socket
import sys
import time

try:
    from NatNetClient import NatNetClient
except ImportError:
    print(
        "ERROR: NatNetClient.py not found.\n"
        "Copy it from the Motive NatNet SDK Samples/PythonClient folder, "
        "or `pip install NatNetClient`.",
        file=sys.stderr,
    )
    sys.exit(1)


class OptiTrackUDPSender:
    def __init__(self, host, port, ee_id, f1_id, f2_id, send_rate_hz=120):
        self.host = host
        self.port = port
        self.rb_map = {ee_id: 'ee', f1_id: 'f1', f2_id: 'f2'}
        self.latest = {'ee': None, 'f1': None, 'f2': None}
        self.send_dt = 1.0 / float(send_rate_hz)
        self._last_send = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)

    def on_rigid_body(self, new_id, position, rotation):
        """NatNet callback. `rotation` is (qx, qy, qz, qw) from Motive."""
        role = self.rb_map.get(new_id)
        if role is None:
            return
        self.latest[role] = {
            "pos": [float(position[0]), float(position[1]), float(position[2])],
            "quat": [
                float(rotation[0]), float(rotation[1]),
                float(rotation[2]), float(rotation[3]),
            ],
        }
        self._maybe_send()

    def _maybe_send(self):
        now = time.monotonic()
        if now - self._last_send < self.send_dt:
            return
        self._last_send = now
        payload = {role: body for role, body in self.latest.items() if body is not None}
        if not payload:
            return
        try:
            self.sock.sendto(json.dumps(payload).encode('utf-8'), (self.host, self.port))
        except OSError as e:
            print(f"[send fail] {e}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True, help='Robot machine IP (Linux running FIC Studio)')
    ap.add_argument('--port', type=int, default=5555)
    ap.add_argument('--ee-rb-id', type=int, required=True, help='Motive rigid body ID for EE')
    ap.add_argument('--f1-rb-id', type=int, required=True, help='Motive rigid body ID for finger 1')
    ap.add_argument('--f2-rb-id', type=int, required=True, help='Motive rigid body ID for finger 2')
    ap.add_argument('--server-ip', default='127.0.0.1', help='Motive server IP')
    ap.add_argument('--client-ip', default='127.0.0.1', help='This machine IP on Motive\'s network')
    ap.add_argument('--multicast', default='239.255.42.99', help='Motive multicast address')
    ap.add_argument('--send-rate', type=int, default=120, help='UDP send rate cap (Hz)')
    args = ap.parse_args()

    sender = OptiTrackUDPSender(
        args.host, args.port, args.ee_rb_id, args.f1_rb_id, args.f2_rb_id,
        send_rate_hz=args.send_rate,
    )

    client = NatNetClient()
    client.set_server_address(args.server_ip)
    client.set_client_address(args.client_ip)
    client.set_use_multicast(True)
    client.set_multicast_address(args.multicast)
    client.rigid_body_listener = sender.on_rigid_body

    if not client.run():
        print("NatNet client failed to start. Check Motive streaming & IPs.", file=sys.stderr)
        sys.exit(1)

    print(f"Streaming to udp://{args.host}:{args.port}  "
          f"(EE={args.ee_rb_id}, F1={args.f1_rb_id}, F2={args.f2_rb_id})")
    print("Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        client.shutdown()


if __name__ == '__main__':
    main()
