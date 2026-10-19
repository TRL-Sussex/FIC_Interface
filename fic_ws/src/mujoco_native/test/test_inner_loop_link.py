"""The simulator's side of lockstep with the joint inner loop (mujoco_native.inner_loop_link).

SimStep / SimTorque bytes are pinned against fic_library/include/fic/fic_link.h (compiled 19 Oct 2026). A
simulator with no inner loop listening must never wait per step; once one answers, each step waits for its
torque. The hold torque is how a robot's own controller holds the arm when the inner loop does not torque it.
"""
import socket
import struct
import threading
import time

import pytest

from mujoco_native import inner_loop_link as ill

STEP_HEX = ('4649434c03000800050000003200000040e201000000000002000000003f000080be0000803f00000040'
            '000000000000000000004040000000000000000000000000e2de')
TORQUE_HEX = '4649434c030009000f000000170000000900000040e2010000000000010200000060400000a0bf03e7'


def test_simstep_bytes_match_fic_link():
    d = ill.encode_sim_step(123456, [0.5, -0.25], [1.0, 2.0], [0, 0, 3.0, 0, 0, 0], seq=5)
    assert d.hex() == STEP_HEX


def test_simtorque_decodes_fic_link_bytes():
    r = ill.decode_sim_torque(bytes.fromhex(TORQUE_HEX))
    assert r == (9, 123456, True, [3.5, -1.25])
    assert ill.decode_sim_torque(bytes.fromhex(TORQUE_HEX)[:-1]) is None      # bad CRC / length
    assert ill.decode_sim_torque(bytes.fromhex(STEP_HEX)) is None             # another message


def test_no_inner_loop_never_waits():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()                                         # nothing listens there
    link = ill.InnerLoopLink(port)
    t0 = time.monotonic()
    for k in range(200):
        assert link.step(k * 0.002, [0.1] * 7, [0.0] * 7, [0.0] * 6) is None
    assert time.monotonic() - t0 < 0.2                # no per-step wait
    link.close()


def test_waits_for_a_live_inner_loop():
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(('127.0.0.1', 0))
    port = srv.getsockname()[1]
    stop = threading.Event()

    def echo():                                       # a fake inner loop: answers each step after 3 ms
        srv.settimeout(0.1)
        k = 0
        while not stop.is_set():
            try:
                data, addr = srv.recvfrom(2048)
            except socket.timeout:
                continue
            k += 1
            time.sleep(0.003)
            n = struct.unpack_from('<H', data, 16 + 8)[0]
            t_us = struct.unpack_from('<Q', data, 16)[0]
            srv.sendto(ill.encode_sim_torque(k, t_us, True, [float(k)] * n, seq=k), addr)
    th = threading.Thread(target=echo, daemon=True)
    th.start()
    link = ill.InnerLoopLink(port)
    got = []
    for k in range(30):                               # paced like a real-time simulator (2 ms steps)
        got.append(link.step(k * 0.002, [0.1] * 7, [0.0] * 7, [0.0] * 6))
        time.sleep(0.002)
    stop.set()
    th.join()
    link.close()
    live = [g for g in got if g is not None]
    assert len(live) >= 25                            # after the first answer, every step got its torque
    assert all(g[0] for g in live) and len(live[-1][1]) == 7


def test_hold_torque():
    import pytest
    assert ill.hold_torque(5.0, 0.3, 0.25, 0.1) == pytest.approx(5.0 + 200 * 0.05 - 20 * 0.1)


def test_each_step_uses_the_reply_to_that_step():
    """A late reply (a busy PC) is dropped, never applied to a later step: the torque a step applies was computed
    from that step's state, however many replies came late before it."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(('127.0.0.1', 0))
    port = srv.getsockname()[1]
    stop = threading.Event()

    def echo():                                       # answers each step with its own time; three replies late
        srv.settimeout(0.1)
        k = 0
        while not stop.is_set():
            try:
                data, addr = srv.recvfrom(2048)
            except socket.timeout:
                continue
            k += 1
            if k in (10, 20, 30):
                time.sleep(0.03)                      # longer than the link waits
            t_us = struct.unpack_from('<Q', data, 16)[0]
            n = struct.unpack_from('<H', data, 24)[0]
            srv.sendto(ill.encode_sim_torque(k, t_us, True, [t_us * 1e-6] * n, seq=k), addr)
    th = threading.Thread(target=echo, daemon=True)
    th.start()
    link = ill.InnerLoopLink(port)
    got = []
    for k in range(60):
        t = k * 0.002
        r = link.step(t, [0.1] * 7, [0.0] * 7, [0.0] * 6)
        got.append((t, r))
        time.sleep(0.002)
    stop.set()
    th.join()
    link.close()
    used = [(t, r[1][0]) for t, r in got if r is not None]
    assert len(used) >= 50
    stale = [(round(t, 3), round(v, 3)) for t, v in used if abs(v - t) > 1e-6]
    assert not stale, stale


def test_real_time_factor_meter():
    m = ill.RtfMeter(period_s=10.0)
    assert m.update(0.0, 100.0) is None                # starts the window
    assert m.update(4.0, 105.0) is None                # window not over
    assert m.update(9.0, 110.0) == pytest.approx(0.9)  # 9 s simulated in 10 s
    assert m.update(5.0, 112.0) is None                # simulated time went back (a reset): a new window
    assert m.update(15.0, 122.0) == pytest.approx(1.0)


#: The station -> port table every copy of the mapping follows (fic_link.h simInnerLoopPort, the Studio's).
PORTS = {1: 7201, 2: 7202, 35: 7235, 99: 7299, 100: 6100, 545: 6545, 999: 6999, 1001: 7101, 1099: 7199}


def test_sim_inner_loop_port_per_station():
    for station, port in PORTS.items():
        assert ill.sim_inner_loop_port(station) == port
    assert all(1024 < ill.sim_inner_loop_port(n) < 7350 for n in range(1, 1100))


def test_crc_is_crc16_ccitt_false():
    """The FIC-link checksum (fic_link.h crc16): CRC-16/CCITT-FALSE, check value 0x29B1."""
    assert ill._crc16(b'123456789') == 0x29B1


def test_a_step_round_trip_costs_little():
    """Encode a SimStep and decode a SimTorque every physics step: the checksum was a bit loop in
    Python (~17 % of the step loop on a PC, more on the MCU board, whose simulator ran at a
    real-time factor of 0.33). 2000 round trips: ~140 ms with that loop, ~4 ms in C; bound 40 ms."""
    import time
    q = [0.1] * 7
    reply = ill.encode_sim_torque(1, 1000, True, q)
    t0 = time.perf_counter()
    for k in range(2000):
        ill.encode_sim_step(k, q, q, [0.0] * 6)
        ill.decode_sim_torque(reply)
    assert time.perf_counter() - t0 < 0.04
