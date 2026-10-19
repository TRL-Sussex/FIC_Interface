"""Which inner loop drives the robot: the FIC inner-loop MCU, or the standard
vendor-driver path — shown the same way for every robot.

Two sources, one summary (``inner_loop_summary``) for the status bar and the
web remote:

  * while fic_control runs: its ``{rid}_inner_loop`` topic (~2 Hz), which says
    what it actually drives — "mcu ... | running | state 1000 Hz ... | faults 0x0"
    or "standard: ...";
  * otherwise: a background ``mcu_probe.discover`` for the selected robot every
    ``IDLE_PROBE_S``, so the Studio shows before Start whether an MCU will take
    the arm, and where it is (Ethernet or this board's Bridge).

A robot without an ``mcu:`` adapter is never probed and always shows
"standard". The inner-loop part commands nothing.

Kortex supervisor (robots with ``mcu: {adapter: kortex}``): once the arm answers
on its Kortex port from this machine — the one-switch layout — the Studio starts
``kortex_supervisor.py`` for the station: arm state and faults, camera reachable,
and Clear faults / Home / Kortex e-stop (``kortex_command``). It never commands
torque; the MCU (or the vendor driver) keeps the arm. Arm not reachable: nothing
is started and everything else works as before.
"""
import json
import threading
import time

from std_msgs.msg import String

from fic_studio import mcu_probe

#: Seconds between idle probes (each is one HELLO, ~1 ms on Ethernet, a few on the Bridge).
IDLE_PROBE_S = 10.0
#: A status message older than this means fic_control is gone.
TOPIC_STALE_S = 3.0
#: Seconds between checks that the arm answers (to start the Kortex supervisor).
KORTEX_CHECK_S = 10.0
#: The supervisor publishes ~1 Hz; older than this, it is gone.
KORTEX_STALE_S = 5.0


def parse_status(text):
    """fic_control's status line -> (path, state, rest)."""
    if not text:
        return '', '', ''
    if text.startswith('standard'):
        return 'standard', '', text.partition(':')[2].strip()
    if text.startswith('no inner loop'):
        return 'none', '', text.partition(':')[2].strip()
    parts = [p.strip() for p in text.split('|')]
    state = parts[1] if len(parts) > 1 else ''
    return ('pc' if text.startswith('inner loop') else 'mcu'), state, ' · '.join(parts[2:])


class InnerLoopMixin:
    """Status-bar indicator + summary of the robot's inner loop."""

    def _init_inner_loop(self):
        self._il_msg = None                 # (text, monotonic time) from fic_control
        self._il_probe = None               # last idle discover() result
        self._il_probe_at = float('-inf')
        self._il_probe_busy = False
        self._il_sub = None
        self._kx_msg = None                 # (dict, monotonic time) from the Kortex supervisor
        self._kx_check_at = float('-inf')
        self._kx_check_busy = False
        self._kx_note = ''                  # why there is no supervisor
        self._kx_pub = None
        try:
            rid = self.node.station_robot_id
            self._il_sub = self.node.create_subscription(
                String, rid + '_inner_loop', self._il_on_status, self.node._qos_tl)
            self._kx_sub = self.node.create_subscription(
                String, rid + '_kortex_status', self._kx_on_status, self.node._qos_tl)
            self._kx_pub = self.node.create_publisher(String, rid + '_kortex_cmd', 10)
        except Exception as exc:                          # pragma: no cover
            self.log_message('WARN', 'Inner-loop status unavailable: %s' % exc, 'Process')

    def _il_on_status(self, msg):
        """ROS thread: keep the newest line."""
        self._il_msg = (msg.data, time.monotonic())

    # ── Idle probe (background thread; never blocks the GUI) ─────────────
    def _il_maybe_probe(self, robot):
        if self._il_probe_busy or time.monotonic() - self._il_probe_at < IDLE_PROBE_S:
            return
        if getattr(self, 'driver_running', False) or getattr(self, 'controller_running', False):
            return                                        # the topic says it during a session
        cfg = self.get_robot_config(robot) if hasattr(self, 'get_robot_config') else {}
        self._il_probe_busy = True

        def run():
            try:
                info = mcu_probe.discover(cfg, robot_name=robot)
                info['robot'] = robot
                self._il_probe = info
            finally:
                self._il_probe_at = time.monotonic()
                self._il_probe_busy = False
        threading.Thread(target=run, name='mcu-probe', daemon=True).start()

    def il_refresh_probe(self):
        """Probe again at the next status tick (robot changed, config saved)."""
        self._il_probe_at = float('-inf')
        self._il_probe = None

    # ── E-STOP straight to the inner loop ─────────────────────────────────
    def estop_inner_loops(self):
        """E-STOP to every fic_inner_loop on this PC (the real robot's and every simulated station's, whichever
        Studio started them, so a simulation's E-STOP stops the real robot and the real robot's stops the
        simulations) and, when one was found, the MCU board: each ramps its arm down on its own, even if the
        controller node is stuck. Never blocks, never raises."""
        from fic_studio import inner_loop_estop
        targets = [inner_loop_estop.PC_INNER_LOOP]
        sid = getattr(getattr(self, 'node', None), 'station_id', None)
        if sid is not None:                       # this station's simulated arm (fic_inner_loop --adapter sim)
            try:
                targets.append('127.0.0.1:%d' % inner_loop_estop.sim_inner_loop_port(sid))
            except (TypeError, ValueError):
                pass
        for t in inner_loop_estop.local_inner_loops():   # the other stations' on this PC
            if t not in targets:
                targets.append(t)
        addr = ((getattr(self, '_mcu_info', None) or {}).get('address') or '').strip()
        if addr:
            targets.append(addr if ':' in addr else addr + ':7350')
        self._estop_targets = list(targets)          # the E-STOP then waits for these to release the arm
        for t in targets:
            try:
                inner_loop_estop.send(t)
            except Exception as exc:                      # pragma: no cover - send() never raises
                self.log_message('WARN', 'Inner-loop E-STOP to %s not sent: %s' % (t, exc), 'Safety')

    # ── Summary ───────────────────────────────────────────────────────────
    def inner_loop_summary(self):
        """dict(path, state, detail, live, where, adapters, robot) for the GUI and web.

        path: 'mcu' (the MCU board) | 'pc' (fic_inner_loop on this PC) | 'none' (no inner loop: the arm is
        not driven) | 'standard' (an older controller) | 'unknown';  live: from the running controller
        (True) or from a probe before Start (False).
        """
        robot = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
        msg = getattr(self, '_il_msg', None)
        if msg and getattr(self, 'controller_running', False) and time.monotonic() - msg[1] < TOPIC_STALE_S:
            path, state, rest = parse_status(msg[0])
            info = getattr(self, '_mcu_info', None) or {}
            return {'path': path, 'state': state or ('running' if path == 'standard' else ''),
                    'detail': rest, 'live': True, 'where': info.get('via', ''),
                    'adapters': info.get('adapters', []), 'robot': robot, 'text': msg[0]}
        probe = getattr(self, '_il_probe', None)
        if probe and probe.get('robot') == robot:
            if probe['use']:
                return {'path': 'mcu', 'state': 'available', 'detail': probe['detail'], 'live': False,
                        'where': probe.get('via', ''), 'adapters': probe.get('adapters', []),
                        'robot': robot, 'text': probe['detail']}
            # No MCU for this robot: the PC inner loop will drive it.
            return {'path': 'pc', 'state': '', 'detail': probe['detail'], 'live': False,
                    'where': probe.get('via', ''), 'adapters': probe.get('adapters', []),
                    'robot': robot, 'text': probe['detail']}
        return {'path': 'unknown', 'state': '', 'detail': 'checking ...', 'live': False,
                'where': '', 'adapters': [], 'robot': robot, 'text': ''}

    # ── Status bar ───────────────────────────────────────────────────────
    def _il_update_statusbar(self):
        if not hasattr(self, 'il_led'):
            return
        robot = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
        if robot:
            self._il_maybe_probe(robot)
        self._kx_update_statusbar(robot)
        s = self.inner_loop_summary()
        if s['path'] == 'mcu':
            where = ' (%s)' % s['where'] if s['where'] else ''
            fault = 'fault' in s['state'] or ('faults' in s['detail'] and 'faults 0x0' not in s['detail'])
            self.il_led.set_status('disconnected' if fault else
                                   'connected' if s['live'] else 'warning')
            self.il_label.setText('MCU%s · %s' % (where, s['state'] or 'available'))
        elif s['path'] == 'pc':
            fault = 'fault' in s['state'] or ('faults' in s['detail'] and 'faults 0x0' not in s['detail'])
            self.il_led.set_status('disconnected' if fault else 'connected' if s['live'] else 'idle')
            self.il_label.setText('PC · %s' % (s['state'] or 'inner loop'))
        elif s['path'] == 'none':
            self.il_led.set_status('disconnected')
            self.il_label.setText('no inner loop')
        elif s['path'] == 'standard':
            self.il_led.set_status('connected' if s['live'] else 'idle')
            self.il_label.setText('standard')
        else:
            self.il_led.set_status('idle')
            self.il_label.setText('…')
        self.il_label.setToolTip('Inner loop for %s\n%s' % (robot, s['text'] or s['detail']))

    # ── Kortex supervisor ────────────────────────────────────────────────
    def _kx_on_status(self, msg):
        """ROS thread: keep the newest status."""
        try:
            self._kx_msg = (json.loads(msg.data), time.monotonic())
        except ValueError:
            pass

    def _kx_settings(self, robot):
        cfg = self.get_robot_config(robot) if hasattr(self, 'get_robot_config') else {}
        return (cfg or {}).get('mcu') or mcu_probe.controller_mcu_settings(robot) or {}

    def _kx_manage(self, robot):
        """Every KORTEX_CHECK_S, in a thread: start / stop the supervisor."""
        if self._kx_check_busy or time.monotonic() - self._kx_check_at < KORTEX_CHECK_S:
            return
        self._kx_check_busy = True
        mcu = self._kx_settings(robot)
        station = self.node.station_id

        def run():
            sup = mcu_probe.kortex_supervisor()
            try:
                if mcu.get('adapter') != 'kortex':
                    if sup.running():
                        sup.stop()
                    self._kx_note = 'not a Kortex robot'
                    return
                ip = mcu.get('arm_ip') or '192.168.1.10'
                if sup.running() and sup.key == (station, ip):
                    return
                if not mcu_probe.arm_reachable(ip):
                    self._kx_note = 'arm %s not on this network (MCU-only: Bridge layout)' % ip
                    return
                ok, why = sup.ensure(station, ip)
                self._kx_note = '' if ok else why
                if ok and why == 'started':
                    self.log_message('INFO', 'Kortex supervisor started: arm %s answers from this machine '
                                     '(status, clear faults, home, Kortex e-stop)' % ip, 'Process')
            finally:
                self._kx_check_at = time.monotonic()
                self._kx_check_busy = False
        threading.Thread(target=run, name='kortex-check', daemon=True).start()

    def kortex_summary(self):
        """dict for the GUI and web: available, reachable, state, fault, faults, camera, busy, last, note."""
        sup = mcu_probe.kortex_supervisor()
        msg = getattr(self, '_kx_msg', None)
        if not sup.running() or not msg or time.monotonic() - msg[1] > KORTEX_STALE_S:
            return {'available': False, 'reachable': False, 'state': '', 'fault': False, 'faults': '',
                    'camera': None, 'busy': '', 'last': '',
                    'note': getattr(self, '_kx_note', '') or 'starting ...' if sup.running() else
                    getattr(self, '_kx_note', '') or 'no Kortex supervisor'}
        d = msg[0]
        state = (d.get('arm_state') or '').replace('ARMSTATE_', '')
        base = d.get('base_faults') or [0, 0]
        act = d.get('actuator_faults') or []
        faults = []
        if any(base):
            faults.append('base 0x%x/0x%x' % (base[0], base[1]))
        faults += ['joint %d 0x%x/0x%x' % tuple(a) for a in act]
        return {'available': True, 'reachable': bool(d.get('reachable')), 'state': state,
                'servoing': (d.get('servoing') or '').replace('_SERVOING', ''),
                'fault': state == 'IN_FAULT' or bool(faults), 'faults': ', '.join(faults),
                'camera': d.get('camera'), 'busy': d.get('busy') or '', 'last': d.get('last') or '',
                'note': '' if d.get('reachable') else 'arm not reachable from this machine'}

    def kortex_command(self, op):
        """'clear_faults' | 'home' | 'estop' -> (sent, why). The supervisor gates and reports in 'last'."""
        if op not in ('clear_faults', 'home', 'estop'):
            return False, 'unknown Kortex command'
        if self._kx_pub is None or not mcu_probe.kortex_supervisor().running():
            return False, self.kortex_summary()['note'] or 'no Kortex supervisor'
        self._kx_pub.publish(String(data=op))
        return True, ''

    def _kx_update_statusbar(self, robot):
        if robot:
            self._kx_manage(robot)
        if not hasattr(self, 'kx_led'):
            return
        k = self.kortex_summary()
        if not k['available']:
            self.kx_led.set_status('idle')
            self.kx_btn.setText('ARM · –')
        elif not k['reachable']:
            self.kx_led.set_status('warning')
            self.kx_btn.setText('ARM · not reachable')
        else:
            self.kx_led.set_status('disconnected' if k['fault'] else 'connected')
            self.kx_btn.setText('ARM · %s' % (k['busy'] or k['state'].lower().replace('_', ' ') or '?'))
        tip = ['Kortex (the arm\'s own API, from Linux)']
        if k['available']:
            tip += ['state: %s, servoing: %s' % (k['state'] or '?', k.get('servoing') or '?'),
                    'faults: %s' % (k['faults'] or 'none'),
                    'camera stream: %s' % {True: 'reachable', False: 'NOT reachable', None: '?'}[k['camera']]]
            if k['last']:
                tip.append('last: %s' % k['last'])
        else:
            tip.append(k['note'])
        self.kx_btn.setToolTip('\n'.join(tip))
        for act in getattr(self, '_kx_actions', []):
            act.setEnabled(k['available'] and k['reachable'])

    def _kx_menu(self, op):
        """Status-bar ARM menu."""
        from PyQt5.QtWidgets import QMessageBox
        if op == 'home':
            if QMessageBox.question(
                    self, 'Home the arm',
                    "Move the arm to Kinova's Home pose now?\n\nIt moves on its own. Keep the space clear "
                    "and the e-stop in reach.") != QMessageBox.Yes:
                return
        sent, why = self.kortex_command(op)
        label = {'clear_faults': 'Clear faults', 'home': 'Home', 'estop': 'Kortex e-stop'}[op]
        if sent:
            self.log_message('WARN' if op != 'clear_faults' else 'INFO',
                             'Kortex: %s sent (the result shows on ARM)' % label, 'Process')
        else:
            self.log_message('WARN', 'Kortex: %s not sent: %s' % (label, why), 'Process')
