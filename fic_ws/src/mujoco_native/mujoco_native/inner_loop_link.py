"""The simulator's side of lockstep with the joint inner loop (fic_inner_loop --adapter sim).

Each physics step sends one FIC-link SimStep (simulated time, joint positions, measured torques, wrench) and
uses the SimTorque the inner loop answers: the same inner loop that drives the real arm, ticking once per step
on simulated time. With no inner loop answering, a step never waits (a simulator on its own, or the
linear-impedance baseline driving actuators_cmds, runs as before); once one answers, each step waits for its
torque (bounded, so a stalled inner loop slows the simulation but never freezes it).
"""
import binascii
import itertools
import socket
import struct
import time

MAGIC = 0x4C434946          # "FICL"
VERSION = 3
MSG_SIM_STEP = 8
MSG_SIM_TORQUE = 9
_HEADER = struct.Struct('<IHHIHH')

_seq = itertools.count(1)


def _crc16(data):
    """fic_link.h's crc16 (CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF), in C: it runs twice per physics step."""
    return binascii.crc_hqx(data, 0xFFFF)


def _frame(msg_type, payload, seq):
    head = _HEADER.pack(MAGIC, VERSION, msg_type, seq & 0xFFFFFFFF, len(payload), 0)
    return head + payload + struct.pack('<H', _crc16(head + payload))


def encode_sim_step(t_us, q, tau, wrench, seq=None):
    """SimStep, byte for byte as fic_link.h encodes it."""
    n = len(q)
    payload = (struct.pack('<QH', int(t_us), n) + struct.pack('<%df' % n, *q) +
               struct.pack('<%df' % n, *tau) + struct.pack('<6f', *wrench))
    return _frame(MSG_SIM_STEP, payload, next(_seq) if seq is None else seq)


def encode_sim_torque(step, t_us, torque_mode, tau, seq=None):
    """SimTorque (the inner loop's answer to the SimStep at t_us); used by tests and fakes."""
    n = len(tau)
    payload = struct.pack('<IQBH', step, int(t_us), 1 if torque_mode else 0, n) + struct.pack('<%df' % n, *tau)
    return _frame(MSG_SIM_TORQUE, payload, next(_seq) if seq is None else seq)


def decode_sim_torque(data):
    """(step, t_us, torque_mode, [tau]) or None (not a valid SimTorque)."""
    if len(data) < _HEADER.size + 2:
        return None
    magic, version, msg_type, _seq_, length, _r = _HEADER.unpack_from(data, 0)
    if magic != MAGIC or version != VERSION or msg_type != MSG_SIM_TORQUE:
        return None
    if _HEADER.size + length + 2 != len(data):
        return None
    (crc,) = struct.unpack_from('<H', data, _HEADER.size + length)
    if _crc16(data[:_HEADER.size + length]) != crc:
        return None
    step, t_us, mode, n = struct.unpack_from('<IQBH', data, _HEADER.size)
    if 15 + 4 * n != length:
        return None
    tau = list(struct.unpack_from('<%df' % n, data, _HEADER.size + 15))
    return step, t_us, bool(mode), tau


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


def hold_torque(bias, q_hold, q, qd, kp=200.0, kd=20.0):
    """How a robot's own controller holds a joint when the inner loop does not torque it (fic_mcu_hostsim)."""
    return bias + kp * (q_hold - q) - kd * qd


class InnerLoopLink:
    """One SimStep / SimTorque per physics step with the inner loop at 127.0.0.1:port."""

    def __init__(self, port, host='127.0.0.1', wait_s=0.02, alive_s=0.5):
        self._addr = (host, int(port))
        self._wait_s = wait_s
        self._alive_s = alive_s
        self._last_reply = float('-inf')
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def alive(self):
        """The inner loop answered recently."""
        return time.monotonic() - self._last_reply < self._alive_s

    def step(self, t_s, q, tau, wrench):
        """Send this step; (torque_mode, [tau]) answering it, or None (no answer to it in time).

        Only the reply that names this step (its t_us, echoed) is used: a late reply to an earlier step is
        dropped, so a step never applies a torque computed from an older state. Any reply keeps the link
        alive."""
        t_us = round(t_s * 1e6)
        try:
            self._sock.sendto(encode_sim_step(t_us, q, tau, wrench), self._addr)
        except OSError:
            return None
        start = time.monotonic()
        deadline = start + (self._wait_s if self.alive() else 0.0)
        while True:
            reply = self._recv(max(0.0, deadline - time.monotonic()))
            if reply is not None:
                self._last_reply = time.monotonic()
                if reply[1] == t_us:
                    return reply[2], reply[3]
                deadline = max(deadline, start + self._wait_s)   # it answers: wait for this step's reply
                continue
            if time.monotonic() >= deadline:
                return None

    def _recv(self, timeout):
        try:
            if timeout > 0:
                self._sock.settimeout(timeout)
            else:
                self._sock.setblocking(False)
            while True:
                data = self._sock.recv(2048)
                r = decode_sim_torque(data)
                if r is not None:
                    return r
        except (BlockingIOError, socket.timeout, OSError):
            return None

    def close(self):
        self._sock.close()


class RtfMeter:
    """The real-time factor (simulated time / wall time) over windows of period_s wall seconds. In lockstep the
    physics stays consistent below 1, but the task node's planners and target streams run on wall time, so a
    run below ~0.95 is not the run it says it is: logged so it is on record."""

    def __init__(self, period_s=10.0):
        self._period = period_s
        self._start = None                            # (sim, wall) at the window's start
        self._last_sim = None

    def update(self, sim_t, wall_t):
        """The factor when a window ends, else None. Simulated time going back (a reset) starts a new window."""
        if self._start is None or (self._last_sim is not None and sim_t < self._last_sim):
            self._start = (sim_t, wall_t)
            self._last_sim = sim_t
            return None
        self._last_sim = sim_t
        s0, w0 = self._start
        if wall_t - w0 < self._period:
            return None
        self._start = (sim_t, wall_t)
        return (sim_t - s0) / (wall_t - w0)
