"""Which inner loop drives the robot: the FIC inner-loop MCU, or the standard
vendor-driver path — shown the same way for every robot.

Two sources, one summary (``inner_loop_summary``) for the status bar and the
web remote:

  * while fic_hardware runs: its ``{rid}_inner_loop`` topic (~2 Hz), which says
    what it actually drives — "mcu ... | running | state 1000 Hz ... | faults 0x0"
    or "standard: ...";
  * otherwise: a background ``mcu_probe.discover`` for the selected robot every
    ``IDLE_PROBE_S``, so the Studio shows before Start whether an MCU will take
    the arm, and where it is (Ethernet or this board's Bridge).

A robot without an ``mcu:`` adapter is never probed and always shows
"standard". Nothing here commands anything.
"""
import threading
import time

from std_msgs.msg import String

from fic_studio import mcu_probe

#: Seconds between idle probes (each is one HELLO, ~1 ms on Ethernet, a few on the Bridge).
IDLE_PROBE_S = 10.0
#: A status message older than this means fic_hardware is gone.
TOPIC_STALE_S = 3.0


def parse_status(text):
    """fic_hardware's status line -> (path, state, rest)."""
    if not text:
        return '', '', ''
    if text.startswith('standard'):
        return 'standard', '', text.partition(':')[2].strip()
    parts = [p.strip() for p in text.split('|')]
    state = parts[1] if len(parts) > 1 else ''
    return 'mcu', state, ' · '.join(parts[2:])


class InnerLoopMixin:
    """Status-bar indicator + summary of the robot's inner loop."""

    def _init_inner_loop(self):
        self._il_msg = None                 # (text, monotonic time) from fic_hardware
        self._il_probe = None               # last idle discover() result
        self._il_probe_at = float('-inf')
        self._il_probe_busy = False
        self._il_sub = None
        try:
            rid = self.node.station_robot_id
            self._il_sub = self.node.create_subscription(
                String, rid + '_inner_loop', self._il_on_status, self.node._qos_tl)
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

    # ── Summary ───────────────────────────────────────────────────────────
    def inner_loop_summary(self):
        """dict(path, state, detail, live, where, adapters, robot) for the GUI and web.

        path: 'mcu' | 'standard' | 'unknown';  live: from the running
        controller (True) or from a probe before Start (False).
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
            return {'path': 'standard', 'state': '', 'detail': probe['detail'], 'live': False,
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
        s = self.inner_loop_summary()
        if s['path'] == 'mcu':
            where = ' (%s)' % s['where'] if s['where'] else ''
            fault = 'fault' in s['state'] or ('faults' in s['detail'] and 'faults 0x0' not in s['detail'])
            self.il_led.set_status('disconnected' if fault else
                                   'connected' if s['live'] else 'warning')
            self.il_label.setText('MCU%s · %s' % (where, s['state'] or 'available'))
        elif s['path'] == 'standard':
            self.il_led.set_status('connected' if s['live'] else 'idle')
            self.il_label.setText('standard')
        else:
            self.il_led.set_status('idle')
            self.il_label.setText('…')
        self.il_label.setToolTip('Inner loop for %s\n%s' % (robot, s['text'] or s['detail']))
