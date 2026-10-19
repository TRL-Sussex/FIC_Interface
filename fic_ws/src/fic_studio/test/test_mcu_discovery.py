"""Inner-loop MCU discovery and status, the same for every robot.

A fake MCU answers FIC-link HELLO on a local UDP port, standing in for one on
Ethernet or behind the Bridge relay.
"""
import socket
import struct
import threading

import pytest

from fic_studio import mcu_probe
from fic_studio.mixins.inner_loop_mixin import parse_status
from fic_studio.mixins.robot_config_mixin import controller_robots


class FakeMcu:
    """Answers HELLO with a HELLO_ACK naming ``adapters``."""

    def __init__(self, adapters=('kortex', 'hostsim'), fw='fic_mcu_test'):
        self.mask = sum(1 << mcu_probe.ADAPTERS[a] for a in adapters)
        self.fw = fw.encode()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.1)
        self.address = '127.0.0.1:%d' % self.sock.getsockname()[1]
        self._run = True
        self._t = threading.Thread(target=self._serve, daemon=True)
        self._t.start()

    def _serve(self):
        while self._run:
            try:
                data, peer = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            nonce = struct.unpack_from('<I', data, 16)[0]
            payload = struct.pack('<IIIHBB', nonce, 1, self.mask, 7, 0, 0) + self.fw.ljust(24, b'\0')
            head = struct.pack('<IHHIHH', mcu_probe._MAGIC, 1, mcu_probe._HELLO_ACK, 1, len(payload), 0)
            msg = head + payload
            self.sock.sendto(msg + struct.pack('<H', mcu_probe._crc16(msg)), peer)

    def close(self):
        self._run = False
        self._t.join()
        self.sock.close()


class FakeBridge:
    def __init__(self, possible=True, ok=True):
        self._possible, self._ok, self.ensured = possible, ok, 0

    def possible(self):
        return self._possible

    def ensure(self):
        self.ensured += 1
        return self._ok, 'relay running' if self._ok else 'no router'


@pytest.fixture
def places(monkeypatch):
    """Point 'Ethernet' and 'Bridge' at addresses the test controls."""
    def set_places(ethernet, bridge):
        monkeypatch.setattr(mcu_probe, 'ENV_ADDRESS', '')
        monkeypatch.setattr(mcu_probe, 'ETHERNET_ADDRESS', ethernet)
        monkeypatch.setattr(mcu_probe, 'BRIDGE_ADDRESS', bridge)
    return set_places


def _dead_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return '127.0.0.1:%d' % port


def test_robot_without_adapter_is_never_probed():
    bridge = FakeBridge()
    info = mcu_probe.discover({'mcu': {}}, bridge=bridge)
    assert not info['use'] and info['adapter'] == '' and bridge.ensured == 0


def test_unknown_adapter_is_refused():
    info = mcu_probe.discover({'mcu': {'adapter': 'warp'}}, bridge=FakeBridge())
    assert not info['use'] and 'unknown' in info['detail']


def test_ethernet_mcu_wins(places):
    mcu = FakeMcu()
    try:
        places(mcu.address, _dead_address())
        bridge = FakeBridge()
        info = mcu_probe.discover({'mcu': {'adapter': 'kortex'}}, bridge=bridge)
        assert info['use'] and info['via'] == 'ethernet' and bridge.ensured == 0
        assert info['adapters'] == ['kortex', 'hostsim']
    finally:
        mcu.close()


def test_bridge_mcu_found_when_ethernet_silent(places):
    mcu = FakeMcu()
    try:
        places(_dead_address(), mcu.address)
        bridge = FakeBridge()
        info = mcu_probe.discover({'mcu': {'adapter': 'kortex'}}, bridge=bridge)
        assert info['use'] and info['via'] == 'bridge' and bridge.ensured == 1
        assert info['address'] == mcu.address
    finally:
        mcu.close()


def test_board_without_bridge_skips_it(places):
    places(_dead_address(), _dead_address())
    bridge = FakeBridge(possible=False)
    info = mcu_probe.discover({'mcu': {'adapter': 'kortex'}}, bridge=bridge)
    assert not info['use'] and bridge.ensured == 0 and 'no MCU answered' in info['detail']


def test_mcu_without_the_robots_adapter_means_standard(places):
    mcu = FakeMcu(adapters=('hostsim',))
    try:
        places(mcu.address, _dead_address())
        info = mcu_probe.discover({'mcu': {'adapter': 'kortex'}}, bridge=FakeBridge(possible=False))
        assert not info['use'] and 'does not support' in info['detail'] and info['adapters'] == ['hostsim']
    finally:
        mcu.close()


def test_status_line_parsing():
    path, state, rest = parse_status(
        'mcu fic_mcu_ventunoq v1 at 127.0.0.1:7350 | running | state 134 Hz, age 0 ms | faults 0x0')
    assert (path, state) == ('mcu', 'running') and 'faults 0x0' in rest
    assert parse_status('standard: vendor driver via shared memory')[0] == 'standard'
    assert parse_status('') == ('', '', '')


def test_status_line_parsing_pc_inner_loop():
    """fic_control's lines for the PC inner loop (fic_inner_loop) and for no inner loop at all."""
    path, state, rest = parse_status(
        'inner loop fic_inner_loop v3 at 127.0.0.1:7360 | running | state 1000 Hz, age 0 ms | faults 0x0')
    assert (path, state) == ('pc', 'running') and 'faults 0x0' in rest
    path, state, rest = parse_status('no inner loop: nothing answered at 127.0.0.1:7360')
    assert (path, state) == ('none', '') and 'nothing answered' in rest


def test_controller_copy_takes_the_mcu_block_from_the_studio():
    old = {'R': {'mcu': {'adapter': 'kortex', 'torque_rate_limit': [2, 2]}}}
    out = controller_robots({'R': {'mcu': {'adapter': 'kortex', 'torque_rate_limit': [2, 2],
                                           'arm_ip': '10.0.0.5'}}}, old)
    assert out['R']['mcu']['arm_ip'] == '10.0.0.5'
    # Edited in the controllers' copy only: kept.
    assert controller_robots({'R': {}}, old)['R']['mcu']['adapter'] == 'kortex'
    # Adapter removed in the Studio: standard path only.
    assert controller_robots({'R': {'mcu': {}}}, old)['R']['mcu'] == {}


def test_every_python_fic_link_sender_speaks_fic_link_h_version():
    """The Studio's MCU probe, the inner-loop E-STOP and the simulator's lockstep link all send FIC-link
    datagrams by hand: each must use fic_link.h's kVersion, or a firmware/inner loop of that version ignores
    them (19 Oct 2026: the probe still said v2 to firmware 3, so the Studio found no MCU)."""
    import os
    import re
    from fic_studio import inner_loop_estop, mcu_probe
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, '..', '..', '..', '..'))
    header = open(os.path.join(root, 'fic_library', 'include', 'fic', 'fic_link.h')).read()
    k = int(re.search(r'kVersion\s*=\s*(\d+)', header).group(1))
    assert mcu_probe._VERSION == k
    assert inner_loop_estop.VERSION == k
    sim = open(os.path.join(root, 'fic_ws', 'src', 'mujoco_native', 'mujoco_native', 'inner_loop_link.py')).read()
    assert int(re.search(r'^VERSION\s*=\s*(\d+)', sim, re.M).group(1)) == k


def test_relay_is_found_where_the_studio_finds_the_repository(monkeypatch, tmp_path):
    """The relay binary comes from the repository the Studio runs from (fic_studio.config, FIC_ROOT), not from
    this file's location: in a plain (non-symlink) colcon install the module lives under install/, and the
    Studio then never tried the board's Bridge (19 Oct 2026)."""
    import importlib
    import os
    from fic_studio import config, mcu_probe
    monkeypatch.setenv('FIC_ROOT', str(tmp_path))
    try:
        importlib.reload(config)
        importlib.reload(mcu_probe)
        assert mcu_probe.RELAY_BINARY == os.path.join(
            str(tmp_path.resolve()), 'robot_library', 'fic_mcu', 'build', 'fic_mcu_relay')
    finally:
        monkeypatch.delenv('FIC_ROOT')
        importlib.reload(config)
        importlib.reload(mcu_probe)


def test_robot_config_yaml_is_found_where_the_studio_finds_the_repository(monkeypatch, tmp_path):
    """Robot Config reads fic_library/config/robots_config.yaml from the repository the Studio runs from
    (fic_studio.config), not relative to its own file (wrong in a plain colcon install)."""
    import importlib
    import os
    from fic_studio import config
    from fic_studio.mixins import robot_config_mixin
    monkeypatch.setenv('FIC_ROOT', str(tmp_path))
    try:
        importlib.reload(config)
        importlib.reload(robot_config_mixin)
        assert robot_config_mixin.RobotConfigMixin._rc_fic_config_yaml_path() == os.path.join(
            str(tmp_path.resolve()), 'fic_library', 'config', 'robots_config.yaml')
    finally:
        monkeypatch.delenv('FIC_ROOT')
        importlib.reload(config)
        importlib.reload(robot_config_mixin)
