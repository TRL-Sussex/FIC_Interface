"""Is there an FIC inner-loop MCU that can drive this robot?

The same test fic_hardware makes (fic_hardware/mcu_backend.hpp), from the
Studio, so a hardware Start does not launch the robot's vendor driver when an
MCU is going to drive the arm (both would command it). One FIC-link HELLO
(fic_library/include/fic/fic_link.h) to the MCU; a robot without an ``mcu:``
block in robots_config.yaml is never probed.

Where the MCU is (``discover``), for every robot the same way:

  * ``FIC_MCU_ADDRESS`` set: there, and only there;
  * otherwise on Ethernet at 192.168.1.20 (the shield and this machine on one
    network), then on this board's own Bridge: an Arduino UNO Q / VENTUNO Q
    whose MCU carries the shield, cabled only to the arm. Linux then reaches
    the MCU through ``fic_mcu_relay`` on 127.0.0.1, started here on demand.

The address found is what fic_hardware is given (``FIC_MCU_ADDRESS``), so the
Studio and the controller always talk to the same MCU.
"""
import atexit
import errno
import os
import random
import signal
import socket
import struct
import subprocess
import threading

import yaml

ENV_ADDRESS = os.environ.get('FIC_MCU_ADDRESS', '')
ETHERNET_ADDRESS = '192.168.1.20'
BRIDGE_ADDRESS = '127.0.0.1'
DEFAULT_ADDRESS = ENV_ADDRESS or ETHERNET_ADDRESS
DEFAULT_PORT = 7350
ROUTER_SOCKET = '/var/run/arduino-router.sock'
_FIC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), *(['..'] * 4)))
RELAY_BINARY = os.path.join(_FIC_ROOT, 'robot_library', 'fic_mcu', 'build', 'fic_mcu_relay')
# _VERSION: fic/fic_link.h kVersion (2 since FIC 5.11.0, when CONFIG gained the FIC law options).
_MAGIC, _VERSION, _HELLO, _HELLO_ACK = 0x4C434946, 2, 1, 2
ADAPTERS = {'loopback': 1, 'kortex': 2, 'canfd': 3, 'hostsim': 4}


def _crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def probe(address=None, timeout=0.3, attempts=3, link_version=_VERSION):
    """HELLO -> dict(fw, version, adapters, max_joints, state), or None. A board answers only a
    HELLO of its own FIC-link version (``link_version``)."""
    host, _, port = (address or DEFAULT_ADDRESS).partition(':')
    port = int(port) if port else DEFAULT_PORT
    nonce = random.getrandbits(32)
    head = struct.pack('<IHHIHH', _MAGIC, link_version, _HELLO, 1, 4, 0) + struct.pack('<I', nonce)
    datagram = head + struct.pack('<H', _crc16(head))
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout / max(1, attempts))
    try:
        for _ in range(attempts):
            try:
                s.sendto(datagram, (host, port))
                while True:
                    data, _ = s.recvfrom(2048)
                    if len(data) < 16 + 40 + 2:
                        continue
                    magic, ver, typ, _seq, length, _r = struct.unpack_from('<IHHIHH', data)
                    if magic != _MAGIC or typ != _HELLO_ACK or \
                            _crc16(data[:16 + length]) != struct.unpack_from('<H', data, 16 + length)[0]:
                        continue
                    got_nonce, fw_version, adapters, max_joints, state, _ = struct.unpack_from('<IIIHBB', data, 16)
                    if got_nonce != nonce:
                        continue
                    name = data[32:56].split(b'\0')[0].decode(errors='replace')
                    return dict(fw=name, version=fw_version, adapters=adapters,
                                max_joints=max_joints, state=state, address='%s:%d' % (host, port))
            except socket.timeout:
                continue
            except OSError:
                return None
    finally:
        s.close()
    return None


def controller_mcu_settings(robot_name):
    """The robot's ``mcu:`` block from fic_library/config/robots_config.yaml —
    the file fic_hardware reads (the Studio's own robot list does not carry it)."""
    try:
        from fic_studio.config import WS_PATH
        path = os.path.join(os.path.dirname(WS_PATH), 'fic_library', 'config', 'robots_config.yaml')
        with open(path) as f:
            return ((yaml.safe_load(f) or {}).get('robots', {}).get(robot_name) or {}).get('mcu') or {}
    except (OSError, ImportError, yaml.YAMLError, AttributeError):
        return {}


# ── The board's own MCU, over the Bridge ──────────────────────────────────

def _relay_preexec():
    """Own process group (async-signal-safe).

    No PR_SET_PDEATHSIG: it fires when the *thread* that forked exits, and the
    relay is usually started from a short-lived probe thread. The Studio stops
    it at exit instead (atexit), and an orphaned relay is harmless — it idles
    and the next Studio reuses it.
    """
    os.setsid()


class BridgeRelay:
    """``fic_mcu_relay``: FIC-link on 127.0.0.1:7350 <-> this board's MCU.

    Started only when the board has an Arduino router (UNO Q / VENTUNO Q) and
    the relay is built; left running for the Studio's lifetime (it idles on a
    socket until fic_hardware or a probe speaks). One that is already running
    — another Studio's, a test's — is used as it is.
    """

    def __init__(self, binary=RELAY_BINARY, router_socket=ROUTER_SOCKET):
        self.binary = binary
        self.router_socket = router_socket
        self.process = None
        self._lock = threading.Lock()

    def possible(self):
        """This board has a Bridge to an MCU, and the relay is built."""
        return os.path.exists(self.router_socket) and os.access(self.binary, os.X_OK)

    @staticmethod
    def _port_taken(port=DEFAULT_PORT):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind((BRIDGE_ADDRESS, port))
            return False
        except OSError as e:
            return e.errno == errno.EADDRINUSE
        finally:
            s.close()

    def ensure(self):
        """Running (ours or someone else's)? Start ours if not. -> (ok, detail)"""
        with self._lock:
            if self.process is not None and self.process.poll() is None:
                return True, 'relay running'
            if self._port_taken():
                return True, 'relay already running'
            if not os.path.exists(self.router_socket):
                return False, 'no Arduino router on this board'
            if not os.access(self.binary, os.X_OK):
                return False, 'fic_mcu_relay not built (robot_library/fic_mcu/scripts/build.sh)'
            try:
                self.process = subprocess.Popen(
                    [self.binary], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    preexec_fn=_relay_preexec)
            except OSError as e:
                return False, 'fic_mcu_relay failed to start: %s' % e
            return True, 'relay started'

    def stop(self):
        with self._lock:
            if self.process is not None and self.process.poll() is None:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    pass
            self.process = None


_RELAY = BridgeRelay()
atexit.register(_RELAY.stop)


def adapter_name(bit_mask):
    """Adapter names in an MCU's HELLO_ACK bit mask."""
    return [n for n, b in ADAPTERS.items() if bit_mask & (1 << b)]


def discover(robot_config, robot_name=None, bridge=None):
    """Find an MCU that can drive this robot, wherever it is.

    Returns a dict: ``use`` (bool), ``detail`` (one line for people),
    ``adapter`` (the robot's, or ''), and when an MCU answered ``address``
    ('host'), ``via`` ('configured' | 'ethernet' | 'bridge'), ``fw``,
    ``adapters`` (names the firmware supports) and ``state``.
    Never raises; takes up to ~1 s when nothing answers.
    """
    mcu = (robot_config or {}).get('mcu') or (controller_mcu_settings(robot_name) if robot_name else {})
    adapter = (mcu or {}).get('adapter') or ''
    out = {'use': False, 'adapter': adapter, 'address': '', 'via': ''}
    if not adapter:
        out['detail'] = 'robot has no mcu adapter configured'
        return out
    if adapter not in ADAPTERS:
        out['detail'] = "unknown mcu adapter '%s'" % adapter
        return out
    bridge = _RELAY if bridge is None else bridge
    if ENV_ADDRESS:
        places = [('configured', ENV_ADDRESS)]
    else:
        places = [('ethernet', ETHERNET_ADDRESS)]
        if bridge and bridge.possible():
            places.append(('bridge', BRIDGE_ADDRESS))
    misses = []
    for via, address in places:
        if via == 'bridge':
            ok, why = bridge.ensure()
            if not ok:
                misses.append('bridge: %s' % why)
                continue
            ack = probe(address, timeout=1.0)          # a Bridge round trip is a few ms each way
        else:
            ack = probe(address)
        if ack is None:
            if probe(address, timeout=0.3, attempts=1, link_version=_VERSION - 1) is not None:
                out.update(address=address, via=via)
                out['detail'] = ('the MCU at %s (%s) runs older firmware (FIC-link v%d, this FIC speaks v%d): '
                                 'flash it again with robot_library/fic_mcu/scripts/deploy_app.sh'
                                 % (address, via, _VERSION - 1, _VERSION))
                return out
            misses.append('%s %s: no answer' % (via, address))
            continue
        out.update(address=address, via=via, fw=ack['fw'], state=ack['state'],
                   adapters=adapter_name(ack['adapters']))
        if not ack['adapters'] & (1 << ADAPTERS[adapter]):
            out['detail'] = "%s (%s) does not support adapter '%s' — it supports %s" % (
                ack['fw'], via, adapter, ', '.join(out['adapters']) or 'none')
            return out
        out['use'] = True
        out['detail'] = "%s v%d via %s (%s), adapter '%s'" % (ack['fw'], ack['version'], via, address, adapter)
        return out
    out['detail'] = 'no MCU answered (%s)' % '; '.join(misses)
    return out
