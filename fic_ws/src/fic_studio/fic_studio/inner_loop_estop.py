"""E-STOP straight to the joint inner loop (the MCU board or fic_inner_loop on this PC).

One FIC-link ``Estop`` datagram (fic_link.h: MsgType::Estop, EstopMsg). The inner loop latches it like a
command's e-stop flag and ramps the arm down on its own, so the stop works even when fic_control is stuck.
Never blocks and never raises: the E-STOP path must not wait on a missing listener.
"""
import itertools
import os
import socket
import struct

MAGIC = 0x4C434946          # "FICL"
VERSION = 3
MSG_ESTOP = 7
MSG_HELLO = 1
MSG_HELLO_ACK = 2
PC_INNER_LOOP = '127.0.0.1:7360'
TORQUING = (3, 4, 5)        # McuState Running, Hold, Ramp: the inner loop still drives the arm

_seq = itertools.count(1)


def _crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def datagram(seq=None, reason=1):
    """The Estop datagram, byte for byte as fic_link.h encodes it."""
    seq = next(_seq) if seq is None else seq
    body = struct.pack('<I', reason)
    head = struct.pack('<IHHIHH', MAGIC, VERSION, MSG_ESTOP, seq & 0xFFFFFFFF, len(body), 0)
    return head + body + struct.pack('<H', _crc16(head + body))


def _frame(msg_type, body, seq):
    head = struct.pack('<IHHIHH', MAGIC, VERSION, msg_type, seq & 0xFFFFFFFF, len(body), 0)
    return head + body + struct.pack('<H', _crc16(head + body))


def probe_state(address, timeout_s=0.1):
    """The inner loop's McuState from one HELLO (fic_link.h HelloAck.state), or None (no answer, bad address).
    Never raises."""
    host, _, port = (address or '').rpartition(':')
    try:
        port = int(port)
    except ValueError:
        return None
    if not host:
        return None
    nonce = next(_seq) & 0xFFFFFFFF
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(timeout_s)
            s.sendto(_frame(MSG_HELLO, struct.pack('<I', nonce), nonce), (host, port))
            while True:
                data = s.recv(256)
                if len(data) < 16 + 40 + 2:
                    continue
                magic, version, mtype, _sq, length, _r = struct.unpack_from('<IHHIHH', data, 0)
                if magic != MAGIC or version != VERSION or mtype != MSG_HELLO_ACK or length != 40:
                    continue
                if struct.unpack_from('<H', data, 16 + length)[0] != _crc16(data[:16 + length]):
                    continue
                ack_nonce, _fw, _ad, _mj, state = struct.unpack_from('<IIIHB', data, 16)
                if ack_nonce == nonce:
                    return state
    except OSError:
        return None


def sim_inner_loop_port(station):
    """The UDP port of a simulated station's inner loop on 127.0.0.1 (fic_link.h simInnerLoopPort): robot
    stations 1-99 at 7200 + N, 1001-1099 at 7100 + (N - 1000), Masters' stations 100-999 at
    6100 + (N - 100); below the MCU (7350), the PC inner loop (7360) and DDS (7400 and up)."""
    station = int(station)
    if 1001 <= station <= 1099:
        return 7100 + (station - 1000)
    if 100 <= station <= 999:
        return 6100 + (station - 100)
    return 7200 + (station % 100)


def send(address):
    """Send one Estop to 'host:port'. True if it left this machine's stack; False on a bad address."""
    host, _, port = (address or '').rpartition(':')
    try:
        port = int(port)
    except ValueError:
        return False
    if not host:
        return False
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setblocking(False)
            s.sendto(datagram(), (host, port))
        return True
    except OSError:
        return False


def local_inner_loops(proc='/proc'):
    """'127.0.0.1:port' of every fic_inner_loop running on this PC (Linux /proc), whoever started it: the real
    robot's (7360) and each simulated station's (sim_inner_loop_port). Sorted; [] where /proc cannot tell."""
    try:
        inodes = set()
        for pid in os.listdir(proc):
            if not pid.isdigit():
                continue
            try:
                with open(os.path.join(proc, pid, 'comm')) as f:
                    if f.read().strip() != 'fic_inner_loop':
                        continue
                fd_dir = os.path.join(proc, pid, 'fd')
                for fd in os.listdir(fd_dir):
                    link = os.readlink(os.path.join(fd_dir, fd))
                    if link.startswith('socket:['):
                        inodes.add(link[8:-1])
            except OSError:
                continue
        ports = set()
        with open(os.path.join(proc, 'net', 'udp')) as f:
            next(f, None)
            for line in f:
                cols = line.split()
                if len(cols) > 9 and cols[9] in inodes:
                    ports.add(int(cols[1].rsplit(':', 1)[1], 16))
        return ['127.0.0.1:%d' % p for p in sorted(ports)]
    except (OSError, ValueError, IndexError):
        return []
