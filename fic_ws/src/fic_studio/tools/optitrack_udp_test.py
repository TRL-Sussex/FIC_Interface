#!/usr/bin/env python3
"""
optitrack_udp_test.py — Test-only sender for FIC Studio's OptiTrack UDP input.
Generates a slow circular EE motion and pulsing finger distance, with no
Motive/NatNet required.  Lets you verify the pipeline end-to-end before
plugging in the real OptiTrack.

Usage (on any machine that can reach the FIC Studio host):
    python optitrack_udp_test.py --host 192.168.1.42 --port 5555
    python optitrack_udp_test.py --host 127.0.0.1 --port 5555   # same machine
"""

import argparse
import json
import math
import socket
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5555)
    ap.add_argument('--rate', type=float, default=100.0, help='Send rate (Hz)')
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.monotonic()
    dt = 1.0 / args.rate

    print(f"Sending test frames to udp://{args.host}:{args.port} at {args.rate:.0f} Hz (Ctrl+C to stop)")

    try:
        while True:
            t = time.monotonic() - t0

            # EE: slow 10 cm circle around an origin 0.4 m in front of the robot
            ee_x = 0.4 + 0.10 * math.cos(0.5 * t)
            ee_y = 0.0 + 0.10 * math.sin(0.5 * t)
            ee_z = 0.4

            # Fingers: pulsing 20 mm → 80 mm distance, 0.5 Hz
            gap = 0.02 + 0.03 * (1 + math.sin(2 * math.pi * 0.5 * t))
            f1 = [ee_x, ee_y - gap / 2, ee_z - 0.05]
            f2 = [ee_x, ee_y + gap / 2, ee_z - 0.05]

            payload = {
                "ee": {"pos": [ee_x, ee_y, ee_z], "quat": [0.0, 0.0, 0.0, 1.0]},
                "f1": {"pos": f1, "quat": [0.0, 0.0, 0.0, 1.0]},
                "f2": {"pos": f2, "quat": [0.0, 0.0, 0.0, 1.0]},
            }
            sock.sendto(json.dumps(payload).encode('utf-8'), (args.host, args.port))
            time.sleep(dt)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
