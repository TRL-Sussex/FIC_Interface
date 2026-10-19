"""E-STOP straight to the inner loop: one FIC-link Estop datagram (fic_studio.inner_loop_estop).

The bytes must be exactly what fic_link.h's encodeFixed(MsgType::Estop, EstopMsg{1}, seq) produces:
Header {magic "FICL", version 3, type 7, seq, len 4, reserved 0}, EstopMsg {reason 1}, CRC16-CCITT-FALSE
over header + payload. Sending never blocks and never raises (E-STOP must not wait on a missing listener).
"""
import socket
import struct

from fic_studio import inner_loop_estop


def _crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def test_datagram_matches_fic_link():
    d = inner_loop_estop.datagram(seq=42)
    head = struct.pack('<IHHIHH', 0x4C434946, 3, 7, 42, 4, 0)
    body = struct.pack('<I', 1)
    assert d == head + body + struct.pack('<H', _crc16(head + body))
    assert len(d) == 16 + 4 + 2
    # Pinned from fic_link.h itself (encodeFixed(MsgType::Estop, EstopMsg{1}, 42, ...), compiled 19 Oct 2026).
    assert d.hex() == '4649434c030007002a00000004000000010000002bac'


def test_send_reaches_a_listener_and_never_raises():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(('127.0.0.1', 0))
    rx.settimeout(1.0)
    port = rx.getsockname()[1]
    assert inner_loop_estop.send('127.0.0.1:%d' % port)
    data, _ = rx.recvfrom(64)
    assert data[8:12] != b'' and struct.unpack('<H', data[6:8])[0] == 7
    rx.close()
    # nothing listening, a bad address, empty: no exception, False or True without blocking
    inner_loop_estop.send('127.0.0.1:1')
    assert inner_loop_estop.send('not an address') is False
    assert inner_loop_estop.send('') is False


def test_estop_goes_to_the_pc_inner_loop_and_the_mcu(monkeypatch):
    """The Studio's E-STOP sends it to fic_inner_loop on this PC and, when one was found, to the MCU board."""
    from types import SimpleNamespace
    from fic_studio.mixins.inner_loop_mixin import InnerLoopMixin
    sent = []
    monkeypatch.setattr(inner_loop_estop, 'send', lambda a: sent.append(a) or True)
    monkeypatch.setattr(inner_loop_estop, 'local_inner_loops', lambda proc='/proc': [])
    w = SimpleNamespace(_mcu_info={'address': '192.168.1.20'}, log_message=lambda *a: None,
                        node=SimpleNamespace(station_id=2))
    InnerLoopMixin.estop_inner_loops(w)
    # the real robot's inner loop, this station's simulated one (sim_inner_loop_port), the MCU board
    assert sent == ['127.0.0.1:7360', '127.0.0.1:7202', '192.168.1.20:7350']
    assert w._estop_targets == sent                   # the E-STOP then waits for these to release the arm
    sent.clear()
    InnerLoopMixin.estop_inner_loops(SimpleNamespace(_mcu_info=None, log_message=lambda *a: None))
    assert sent == ['127.0.0.1:7360']


def test_estop_handler_sends_it_first():
    """_emergency_stop: the inner-loop E-STOP right after the Kortex stop, before the latch and process kills."""
    import inspect
    from fic_studio.main_window_pro import ProfessionalRobotGUI
    src = inspect.getsource(ProfessionalRobotGUI._emergency_stop)
    assert 'estop_inner_loops' in src
    assert src.index('estop_inner_loops') < src.index('_estop_latch_set')


def _fake_proc(root, procs, udp_lines):
    """procs: {pid: (comm, [socket inodes])}; udp_lines: /proc/net/udp rows (port, inode)."""
    import os
    os.makedirs(os.path.join(root, 'net'))
    with open(os.path.join(root, 'net', 'udp'), 'w') as f:
        f.write('  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout '
                'inode ref pointer drops\n')
        for i, (port, inode) in enumerate(udp_lines):
            f.write('  %d: 00000000:%04X 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 '
                    '%d 2 0000000000000000 0\n' % (i, port, inode))
    for pid, (comm, inodes) in procs.items():
        d = os.path.join(root, str(pid))
        os.makedirs(os.path.join(d, 'fd'))
        with open(os.path.join(d, 'comm'), 'w') as f:
            f.write(comm + '\n')
        for k, ino in enumerate(inodes):
            os.symlink('socket:[%d]' % ino, os.path.join(d, 'fd', str(k + 3)))


def test_finds_every_inner_loop_on_this_pc(tmp_path):
    """Every fic_inner_loop on this PC, whoever started it: the real robot's (7360) and each simulated
    station's (7370+N). Other processes' sockets (DDS, the Studio's own) are not E-STOP targets."""
    _fake_proc(str(tmp_path),
               {101: ('fic_inner_loop', [5001]), 102: ('fic_inner_loop', [5002]),
                103: ('fic_inner_loop', [5003]), 104: ('python3', [5004])},
               [(7360, 5001), (7371, 5002), (7374, 5003), (7400, 5004)])
    assert inner_loop_estop.local_inner_loops(str(tmp_path)) == [
        '127.0.0.1:7360', '127.0.0.1:7371', '127.0.0.1:7374']
    assert inner_loop_estop.local_inner_loops(str(tmp_path / 'missing')) == []


def test_estop_from_any_studio_reaches_every_inner_loop_on_this_pc(monkeypatch):
    """A Studio running a simulation and one running the real robot on one PC: either's E-STOP stops both."""
    from types import SimpleNamespace
    from fic_studio.mixins.inner_loop_mixin import InnerLoopMixin
    sent = []
    monkeypatch.setattr(inner_loop_estop, 'send', lambda a: sent.append(a) or True)
    monkeypatch.setattr(inner_loop_estop, 'local_inner_loops',
                        lambda proc='/proc': ['127.0.0.1:7360', '127.0.0.1:7373'])
    w = SimpleNamespace(_mcu_info=None, log_message=lambda *a: None, node=SimpleNamespace(station_id=1))
    InnerLoopMixin.estop_inner_loops(w)
    assert sent == ['127.0.0.1:7360', '127.0.0.1:7201', '127.0.0.1:7373']


def _hello_responder(state):
    """A fake inner loop answering HELLO with HelloAck in the given McuState; returns (port, stop)."""
    import threading
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(('127.0.0.1', 0))
    srv.settimeout(0.05)
    stop = threading.Event()

    def run():
        while not stop.is_set():
            try:
                data, addr = srv.recvfrom(256)
            except socket.timeout:
                continue
            if struct.unpack_from('<H', data, 6)[0] != 1:
                continue
            nonce = struct.unpack_from('<I', data, 16)[0]
            body = struct.pack('<IIIHBB24s', nonce, 1, 1 << 5, 16, state[0], 0, b'fake')
            head = struct.pack('<IHHIHH', 0x4C434946, 3, 2, 1, len(body), 0)
            srv.sendto(head + body + struct.pack('<H', _crc16(head + body)), addr)
        srv.close()
    threading.Thread(target=run, daemon=True).start()
    return srv.getsockname()[1], stop


def test_probe_state_reads_the_inner_loops_state():
    state = [5]                                        # Ramp
    port, stop = _hello_responder(state)
    try:
        assert inner_loop_estop.probe_state('127.0.0.1:%d' % port) == 5
        state[0] = 2                                   # Ready: out of torque
        assert inner_loop_estop.probe_state('127.0.0.1:%d' % port) == 2
    finally:
        stop.set()
    assert inner_loop_estop.probe_state('127.0.0.1:1', timeout_s=0.05) is None
    assert inner_loop_estop.probe_state('nonsense') is None


def _wait_host(targets, driver_exits_at, log):
    import time
    from types import SimpleNamespace
    t0 = time.monotonic()
    proc = SimpleNamespace(poll=lambda: 0 if time.monotonic() - t0 >= driver_exits_at else None)
    return SimpleNamespace(_estop_targets=targets, driver_process=proc, _mcu_driver=False, _ESTOP_RAMP_MS=1,
                           _ESTOP_RELEASE_TIMEOUT_S=2.0, log_message=lambda *a: log.append(a)), t0


def test_estop_waits_for_the_arm_back_in_position_mode(monkeypatch):
    """Before anything is killed: every inner loop has finished its ramp (out of torque) and this Studio's
    vendor driver has left torque mode by itself (the Kinova driver re-engages position mode, then exits)."""
    import time
    from PyQt5.QtWidgets import QApplication
    from fic_studio.main_window_pro import ProfessionalRobotGUI
    QApplication.instance() or QApplication([])
    log = []
    host, t0 = _wait_host(['127.0.0.1:7360'], 0.5, log)
    monkeypatch.setattr(inner_loop_estop, 'probe_state',
                        lambda a, timeout_s=0.1: 5 if time.monotonic() - t0 < 0.3 else 2)
    ProfessionalRobotGUI._estop_ramp_wait(host)
    took = time.monotonic() - t0
    assert 0.5 <= took < 1.5, took


def test_estop_wait_gives_up_after_its_timeout(monkeypatch):
    import time
    from PyQt5.QtWidgets import QApplication
    from fic_studio.main_window_pro import ProfessionalRobotGUI
    QApplication.instance() or QApplication([])
    log = []
    host, t0 = _wait_host(['127.0.0.1:7360'], 99.0, log)
    host._ESTOP_RELEASE_TIMEOUT_S = 0.4
    monkeypatch.setattr(inner_loop_estop, 'probe_state', lambda a, timeout_s=0.1: 3)
    ProfessionalRobotGUI._estop_ramp_wait(host)
    assert time.monotonic() - t0 < 1.0
    assert any('still' in str(m) for m in log)


def test_estop_does_not_wait_on_a_driver_no_inner_loop_will_release(monkeypatch):
    """No inner loop answered: nothing raises the stop flag, so the driver will not leave torque mode on its
    own; the E-STOP goes straight on to stopping it instead of sitting out the timeout."""
    import time
    from PyQt5.QtWidgets import QApplication
    from fic_studio.main_window_pro import ProfessionalRobotGUI
    QApplication.instance() or QApplication([])
    log = []
    host, t0 = _wait_host(['127.0.0.1:7360'], 99.0, log)
    monkeypatch.setattr(inner_loop_estop, 'probe_state', lambda a, timeout_s=0.1: None)
    ProfessionalRobotGUI._estop_ramp_wait(host)
    assert time.monotonic() - t0 < 0.5



def test_sim_inner_loop_port_matches_the_simulator_and_the_library():
    """The Studio's copy of the mapping (E-STOP targets) is the simulator's and fic_link.h's."""
    table = {1: 7201, 2: 7202, 35: 7235, 99: 7299, 100: 6100, 545: 6545, 999: 6999, 1001: 7101, 1099: 7199}
    for station, port in table.items():
        assert inner_loop_estop.sim_inner_loop_port(station) == port
