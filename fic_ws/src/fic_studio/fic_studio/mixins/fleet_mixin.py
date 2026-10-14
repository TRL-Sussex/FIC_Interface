# mixins/fleet_mixin.py — The fleet and teleoperation, on the Multi-Robot page
"""
Fleet mixin: every Studio on the network, lab-wide stops, and teleoperation.

The Multi-Robot page opens with:

* Fleet: a card per Studio on the ROS network (found by ``fleet.presence``,
  not added by hand): its station, host, robot, simulation or hardware,
  whether it runs, its mode and any teleop pairing. Buttons: Drive (point the
  control area at that station) and Web (its web remote). E-STOP ALL and
  PAUSE ALL reach every station (``fleet.commands``).
* Teleoperation: pick a leader and a follower station and start. Any Studio
  can pair any two stations; the follower's Studio agrees (at once in
  simulation, its operator is asked on hardware). See ``fic_studio.teleop``.

The control area below them is the one that was already there. This
replaces the teleop extension.
"""

import math
import os
import time

from fic_studio import theme
from fic_studio.fleet.stations import is_master, station_label, station_rid
from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QMenu,
    QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

FLEET_REFRESH_MS = 500
#: Refuse a remote start this soon after an E-stop (as the web remote does).
ESTOP_HOLDOFF_S = 3.0
#: How long a card shows the answer to a command sent from here.
RESULT_SHOW_S = 20.0
TELEOP_TICK_MS = 10
CARD_COLUMNS = 3


class _TeleopHooks:
    """What a TeleopManager needs from the Studio, for this Studio's own station."""

    def __init__(self, mw):
        self.mw = mw
        self._planner_before = None

    @property
    def rid(self):
        return self.mw.node.station_robot_id

    def robot_type(self):
        return self.mw.robot_dropdown.currentText()

    def joint_count(self):
        return int(self.mw.robots.get(self.robot_type(), 7))

    def joint_positions(self):
        return list(self.mw.joint_positions_for(self.rid) or [])

    def joint_limits(self):
        """Return (lower, upper) in radians from robots_config, or (None, None)."""
        cfg = (getattr(self.mw, 'robot_configs', {}) or {}).get(self.robot_type(), {})
        lim = cfg.get('joint_limits_deg') or {}
        n = self.joint_count()
        lower, upper = lim.get('lower') or [], lim.get('upper') or []
        if len(lower) < n or len(upper) < n:
            return None, None
        return ([math.radians(v) for v in lower[:n]], [math.radians(v) for v in upper[:n]])

    def target(self):
        return getattr(self.mw, 'control_target', 'simulation')

    def begin_following(self, session):
        """Hold the arm in Joint Impedance, planner off: streamed targets are tracked."""
        toggle = getattr(self.mw, '_rc_admittance_planner_toggle', None)
        self._planner_before = toggle.isChecked() if toggle is not None else None
        self.mw.node.publish_admittance_planner(False, self.rid)
        self.mw.node.publish_mode(1, robot_id=self.rid)
        self._sync_mode_ui()

    def set_mode(self, mode):
        """Put this station's arm into FIC control mode 1-8, as the Studio would.

        Same two steps as begin_following: publish it, then bring the desktop's
        own mode tabs into line, so the Studio never shows a mode the arm is not
        in. Used when the lab is driven from the control PC rather than from
        this machine's own screen.
        """
        self.mw.node.publish_mode(int(mode), robot_id=self.rid)
        self._sync_mode_ui()

    def publish_target(self, q):
        self.mw.node.publish_real_joint_angles(self.rid, q)

    def wrench(self):
        """This arm's measured contact, as [fx, fy, fz, tx, ty, tz], or None.

        Read from `current_wrench_by_robot`, which joint_state_mixin fills from
        whichever source is actually publishing: `{rid}_force_torque` on
        hardware, `/robotN_simulation/force_torque` in simulation, or the VR
        bridge. Asking it rather than picking a topic is what stops the force
        channel quietly dying when a pairing moves from simulation to real
        arms, which is exactly how the old teleop extension broke.
        """
        w = (getattr(self.mw, 'current_wrench_by_robot', {}) or {}).get(self.rid)
        return list(w)[:6] if w else None

    def publish_wrench(self, w6):
        """Put a partner's contact onto this arm. Zeros release it."""
        self.mw.node.publish_wrench_external(self.rid, w6)

    def end_following(self, session, reason):
        """Hold where the arm is (a mode message snaps the target to it); restore the planner."""
        self.mw.node.publish_mode(1, robot_id=self.rid)
        if self._planner_before is not None:
            self.mw.node.publish_admittance_planner(self._planner_before, self.rid)
        self._planner_before = None
        self._sync_mode_ui()

    def _sync_mode_ui(self):
        try:
            self.mw.update_mode_ui_only('Joint Impedance', self.rid)
        except Exception:
            pass

    def confirm_request(self, session):
        self.mw._fleet_confirm_request(session)

    def on_change(self):
        self.mw._fleet_dirty = True

    def log(self, level, text):
        self.mw.log_message(level, text, 'Teleop')


class FleetMixin:
    """Mixin: fleet cards, lab-wide stops and teleoperation on the Multi-Robot page."""

    # ── Lifecycle ────────────────────────────────────────────────────────
    def _init_fleet(self):
        """Start presence, lab-wide commands and teleop. Call once the UI exists."""
        from fic_studio.fleet.commands import FleetCommands
        from fic_studio.fleet.presence import ANNOUNCE_S, ClientList, FleetDirectory
        from fic_studio.ros_node import _host_tag
        from fic_studio.teleop.session import TeleopManager
        host = _host_tag()
        station = self.node.station_id
        self._fleet_studio_id = '%s:%d' % (host, os.getpid())
        self._fleet_remote_estop = False
        self._fleet_dirty = True
        self._fleet_cards = {}
        self._fleet_dir = FleetDirectory(self.node, self._fleet_card)
        self._fleet_cmd = FleetCommands(self.node, station, host, self._fleet_on_command)
        self._fleet_clients = ClientList(self.node)
        self._fleet_graph = {}          # station -> (time, [node names]) cache
        self._teleop = TeleopManager(self.node, station, host, _TeleopHooks(self))
        self._fleet_announce_timer = QTimer(self)
        self._fleet_announce_timer.timeout.connect(self._fleet_dir.announce)
        self._fleet_announce_timer.start(int(ANNOUNCE_S * 1000))
        self._teleop_timer = QTimer(self)
        self._teleop_timer.timeout.connect(self._teleop.tick)
        self._teleop_timer.start(TELEOP_TICK_MS)
        # The light state feed: this station's state at 50 Hz for other machines, and
        # the feed of every remote robot this Studio drives (fleet.lite).
        from fic_studio.fleet import lite
        rid = self.node.station_robot_id
        self._lite_pub = lite.LitePublisher(
            self.node, rid,
            lambda: self.joint_positions_for(rid),
            lambda: self.joint_velocity_for(rid) if hasattr(self, 'joint_velocity_for') else [],
            lambda: self.joint_effort_for(rid) if hasattr(self, 'joint_effort_for') else [],
            lambda: self.ee_pose_for(rid))
        self._lite_timer = QTimer(self)
        self._lite_timer.timeout.connect(self._lite_pub.publish)
        self._lite_timer.start(int(1000 / lite.RATE_HZ))
        self._lite_subs = {}            # rid -> subscriptions to a remote robot's feed
        self._fleet_ui_timer = QTimer(self)
        self._fleet_ui_timer.timeout.connect(self._fleet_refresh)
        self._fleet_ui_timer.start(FLEET_REFRESH_MS)
        self._fleet_dir.announce()

    def _teardown_fleet(self):
        for name in ('_fleet_announce_timer', '_teleop_timer', '_fleet_ui_timer', '_lite_timer'):
            timer = getattr(self, name, None)
            if timer is not None:
                timer.stop()
        for name in ('_teleop', '_fleet_cmd', '_fleet_clients', '_fleet_dir', '_lite_pub'):
            obj = getattr(self, name, None)
            if obj is not None:
                try:
                    obj.close()
                except Exception:
                    pass

    # ── This Studio's presence card ──────────────────────────────────────
    def _fleet_card(self):
        from fic_studio.mixins.robot_control_mixin import CONTROL_MODE_NAMES
        from fic_studio.ros_node import _host_tag
        robot = self.robot_dropdown.currentText()
        cfg = (getattr(self, 'robot_configs', {}) or {}).get(robot, {})
        idx = int(getattr(self, '_rc_selected_mode_idx', 0))
        try:
            running = bool(self._tb_session_running())
        except Exception:
            running = False
        teleop = getattr(self, '_teleop', None)
        pairs = [s.as_dict() for s in teleop.visible()] if teleop is not None else []
        return {
            'studio_id': getattr(self, '_fleet_studio_id', _host_tag()),
            'host': _host_tag(),
            'station': self.node.station_id,
            'robot': robot,
            'robot_display': cfg.get('display_name') or robot,
            'joints': int(self.robots.get(robot, 7)),
            'target': getattr(self, 'control_target', 'simulation'),
            'running': running,
            'has_joints': bool(self.joint_positions_for(self.node.station_robot_id)),
            'mode': CONTROL_MODE_NAMES[idx] if idx < len(CONTROL_MODE_NAMES) else '',
            'teleop': [p for p in pairs if p['state'] in ('active', 'pending', 'requested')],
            'urls': list(getattr(self, '_web_urls', []) or []),
            'version': getattr(self, 'APP_VERSION', ''),
            'last_cmd': getattr(self, '_fleet_last_cmd', None),
            # robots this Studio has command publishers for (Drive), besides its own
            'driving': [n for n in self.node.active_robots if n != self.node.station_id],
        }

    # ── Lab-wide commands ────────────────────────────────────────────────
    def _fleet_on_command(self, cmd, info):
        """Run a lab-wide command addressed to this station."""
        who = '%s (%s)' % (station_label(info.get('from_station')), info.get('from_host', '?'))
        if cmd == 'session':
            self._fleet_on_session(info, who)
            return
        if cmd == 'estop':
            self._fleet_estop_time = time.monotonic()
            self.log_message('ERROR', 'E-STOP from %s%s' % (
                who, ': ' + info['reason'] if info.get('reason') else ''), 'Fleet')
            self._fleet_remote_estop = True
            try:
                self._emergency_stop()
            finally:
                self._fleet_remote_estop = False
        elif cmd == 'pause':
            self.log_message('WARN', 'PAUSE from %s' % who, 'Fleet')
            self.pause_for(self.node.station_robot_id)

    def _fleet_on_session(self, info, who):
        """Start or stop this station's session for another Studio (web remote's checks)."""
        op, target = info.get('op'), info.get('target')
        running = bool(self._tb_session_running())

        def answer(ok, text):
            self._fleet_last_cmd = {'id': info.get('id'), 'from_host': info.get('from_host'),
                                    'ok': ok, 'text': text, 'ts': time.time()}
            self.log_message('INFO' if ok else 'WARN', '%s (asked by %s)' % (text, who), 'Fleet')
            self._fleet_dir.announce()

        if op == 'stop':
            if not running:
                return answer(True, 'already stopped')
            self._tb_toggle_start()
            return answer(True, 'stopped %s' % getattr(self, 'control_target', ''))
        if running:
            if target == getattr(self, 'control_target', None):
                return answer(True, 'already running (%s)' % target)
            return answer(False, 'refused: a %s session is running; stop it first'
                          % getattr(self, 'control_target', ''))
        if time.monotonic() - getattr(self, '_fleet_estop_time', float('-inf')) < ESTOP_HOLDOFF_S:
            return answer(False, 'refused: E-stop just fired')
        if target == 'hardware' and not self._resolve_driver_command(
                self.robot_dropdown.currentText()):
            return answer(False, 'refused: no hardware driver configured for %s'
                          % self.robot_dropdown.currentText())
        if target != getattr(self, 'control_target', None):
            self._tb_set_target(target)
        self._tb_toggle_start()
        answer(True, 'started %s' % target)

    def _fleet_session(self, n, op, target=None):
        """Ask station ``n``'s Studio to start (``target``) or stop its session."""
        card = self._fleet_dir.station(n) or {}
        if op == 'start' and target == 'hardware':
            reply = QMessageBox.warning(
                self, 'Start hardware',
                'Start %s on HARDWARE (%s)?\n\nIts controller and driver start, and the arm is '
                'held by the controller. Make sure the area around it is clear.'
                % (station_label(n), card.get('host', '?')),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        self._fleet_session_send(n, op, target)

    def _fleet_session_send(self, n, op, target=None):
        """Send the start / stop request (no dialog: the caller has confirmed)."""
        self._fleet_cmd.send('session', [n], op=op, target=target)
        self.log_message('INFO', 'Asked %s to %s%s' % (
            station_label(n), op, ' (%s)' % target if target else ''), 'Fleet')

    def _fleet_estop_all(self):
        self._fleet_cmd.send('estop', 'all', 'E-STOP ALL')

    def _fleet_pause_all(self):
        self._fleet_cmd.send('pause', 'all')

    def _fleet_on_local_estop(self):
        """Call from the E-stop path: end teleop, and stop the partner arms too."""
        self._fleet_estop_time = time.monotonic()
        teleop = getattr(self, '_teleop', None)
        if teleop is None:
            return
        partners = teleop.partner_stations()
        teleop.stop_all('E-stop on %s' % station_label(self.node.station_id))
        if partners and not getattr(self, '_fleet_remote_estop', False):
            self._fleet_cmd.send('estop', partners, 'teleop partner %s E-stopped'
                                 % station_label(self.node.station_id))

    # ── Teleop requests addressed to us ──────────────────────────────────
    def _fleet_confirm_request(self, s):
        box = QMessageBox(self)
        box.setWindowTitle('Teleoperation request')
        box.setIcon(QMessageBox.Warning)
        box.setText('%s wants %s to lead this robot (%s, on %s).\n\n'
                    "This arm will copy the leader's motion (scale %.2f, max %.0f°/s).\n"
                    'Accept?' % (s.from_host or 'A Studio', station_label(s.leader),
                                 station_label(s.follower),
                                 str(self._teleop.hooks.target()).upper(), s.scale,
                                 math.degrees(s.max_speed)))
        accept = box.addButton('Accept', QMessageBox.AcceptRole)
        box.addButton('Refuse', QMessageBox.RejectRole)
        box.setModal(False)

        def answered(_button):
            if box.clickedButton() is accept:
                self._teleop.accept(s.sid)
            elif s.state == 'pending':
                self._teleop.reject(s.sid, 'refused by the operator')
        box.buttonClicked.connect(answered)
        box.show()

    # ── The page sections ────────────────────────────────────────────────
    def _build_fleet_panel(self) -> QWidget:
        """Return the Fleet + Teleoperation sections for the Multi-Robot page."""
        panel = QWidget()
        outer = QVBoxLayout(panel)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(10)

        head = QHBoxLayout()
        title = QLabel('Fleet')
        title.setObjectName('SectionLabel')
        head.addWidget(title)
        self._fleet_summary = QLabel('')
        self._fleet_summary.setObjectName('HintLabel')
        head.addWidget(self._fleet_summary)
        head.addStretch()
        pause = self._btn('PAUSE ALL', 'BtnWarn')
        pause.setToolTip("Pause every station's controller (hardware). Over the network.")
        pause.clicked.connect(self._fleet_pause_all)
        head.addWidget(pause)
        estop = self._btn('E-STOP ALL', 'BtnStop')
        estop.setToolTip('E-stop every Studio on the network, this one included.\n'
                         "Needs the network: each robot's own E-stop stays the primary stop.")
        estop.clicked.connect(self._fleet_estop_all)
        head.addWidget(estop)
        outer.addLayout(head)

        self._fleet_warning = QLabel('')
        self._fleet_warning.setWordWrap(True)
        theme.bind(self._fleet_warning, lambda t: 'color: %s; font-weight: 600;'
                   % t['accent_red'])
        self._fleet_warning.setVisible(False)
        outer.addWidget(self._fleet_warning)

        self._fleet_grid_host = QWidget()
        self._fleet_grid = QGridLayout(self._fleet_grid_host)
        self._fleet_grid.setContentsMargins(0, 0, 0, 0)
        self._fleet_grid.setSpacing(8)
        for col in range(CARD_COLUMNS):
            self._fleet_grid.setColumnStretch(col, 1)
        outer.addWidget(self._fleet_grid_host)

        outer.addWidget(self._build_teleop_section())
        return panel

    def _build_teleop_section(self) -> QWidget:
        box = QFrame()
        box.setObjectName('Card')
        theme.bind(box, lambda t: 'QFrame#Card { border: 1px solid %s; border-radius: 8px; }'
                   % t['border_subtle'])
        lay = QVBoxLayout(box)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(8)
        title = QLabel('Teleoperation')
        title.setObjectName('SectionLabel')
        lay.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(QLabel('Leader'))
        self._teleop_leader = QComboBox()
        self._teleop_leader.setMinimumWidth(170)
        row.addWidget(self._teleop_leader)
        arrow = QLabel('→')
        row.addWidget(arrow)
        row.addWidget(QLabel('Follower'))
        self._teleop_follower = QComboBox()
        self._teleop_follower.setMinimumWidth(170)
        row.addWidget(self._teleop_follower)
        row.addSpacing(12)
        row.addWidget(QLabel('Scale'))
        self._teleop_scale = QDoubleSpinBox()
        self._teleop_scale.setRange(0.05, 2.0)
        self._teleop_scale.setSingleStep(0.05)
        self._teleop_scale.setValue(1.0)
        self._teleop_scale.setToolTip('Follower motion = scale × leader motion.')
        row.addWidget(self._teleop_scale)
        row.addWidget(QLabel('Max speed'))
        self._teleop_speed = QDoubleSpinBox()
        self._teleop_speed.setRange(5.0, 90.0)
        self._teleop_speed.setSingleStep(5.0)
        self._teleop_speed.setValue(35.0)
        self._teleop_speed.setSuffix(' °/s')
        self._teleop_speed.setToolTip('No follower joint target moves faster than this.')
        row.addWidget(self._teleop_speed)
        row.addStretch()
        start = self._btn('Start teleop', 'BtnPrimary')
        start.clicked.connect(self._teleop_start_clicked)
        row.addWidget(start)
        lay.addLayout(row)

        hint = QLabel('Joint space, same robot type. The follower starts where it is and copies '
                      "the leader's motion from then on. The leader's mode is not changed: set "
                      'it to one you can move the arm in. An E-stop on either side stops both.')
        hint.setObjectName('HintLabel')
        hint.setWordWrap(True)
        lay.addWidget(hint)

        self._teleop_rows_host = QWidget()
        self._teleop_rows = QVBoxLayout(self._teleop_rows_host)
        self._teleop_rows.setContentsMargins(0, 0, 0, 0)
        self._teleop_rows.setSpacing(4)
        lay.addWidget(self._teleop_rows_host)
        return box

    # ── Refresh ──────────────────────────────────────────────────────────
    def _lite_follow_remote_robots(self):
        """Feed every remote robot this Studio drives from its light feed, not 1 kHz topics."""
        from fic_studio.fleet import lite
        node = getattr(self.node, 'state_node', None) or self.node
        own = self.node.station_id
        wanted = {station_rid(n) for n in self.node.active_robots if n != own}
        for rid in wanted - set(self._lite_subs):
            self._lite_subs[rid] = lite.subscribe(
                node, rid,
                lambda q, dq, eff, r=rid: self._on_lite_joints(r, q, dq, eff),
                lambda pose, r=rid: self.current_ee_pose_by_robot.__setitem__(r, pose))
        for rid in set(self._lite_subs) - wanted:
            for sub in self._lite_subs.pop(rid):
                try:
                    node.destroy_subscription(sub)
                except Exception:
                    pass

    def _on_lite_joints(self, rid, q, dq, eff):
        """Store a remote robot's joints (state thread: data only)."""
        self.current_joint_positions_by_robot[rid] = q
        if dq:
            self.current_joint_velocities_by_robot[rid] = dq
        if eff:
            self.current_joint_efforts_by_robot[rid] = eff
        self.current_joint_update_time_by_robot[rid] = time.monotonic()

    def _fleet_refresh(self):
        self._lite_follow_remote_robots()
        if not hasattr(self, '_fleet_grid'):
            return
        studios = self._fleet_dir.studios()
        seen = set()
        for i, card in enumerate(studios):
            key = card['studio_id']
            seen.add(key)
            widget = self._fleet_cards.get(key)
            if widget is None:
                widget = self._fleet_make_card(card)
                self._fleet_cards[key] = widget
            self._fleet_grid.addWidget(widget, i // CARD_COLUMNS, i % CARD_COLUMNS)
            self._fleet_fill_card(widget, card)
        for key in [k for k in self._fleet_cards if k not in seen]:
            widget = self._fleet_cards.pop(key)
            widget.setParent(None)
            widget.deleteLater()

        running = sum(1 for c in studios if c.get('running'))
        pairs = {p['sid'] for c in studios for p in c.get('teleop', [])
                 if p.get('state') == 'active'}
        self._fleet_summary.setText('%d Studio%s · %d running · %d teleop pair%s' % (
            len(studios), '' if len(studios) == 1 else 's', running,
            len(pairs), '' if len(pairs) == 1 else 's'))
        conflicts = self._fleet_dir.conflicts()
        self._fleet_warning.setVisible(bool(conflicts))
        self._fleet_warning.setText('  '.join(
            '%s is claimed by %s: both command the same topics. Give each machine its own '
            'station (toolbar → Robot).' % (station_label(n), ' and '.join(hosts))
            for n, hosts in sorted(conflicts.items())))
        self._teleop_fill_pickers(studios)
        sig = [(x.sid, x.state, x.holding, x.reason) for x in self._teleop.visible()]
        if self._fleet_dirty or sig != getattr(self, '_teleop_rows_sig', None):
            self._fleet_dirty = False
            self._teleop_rows_sig = sig
            self._teleop_fill_rows()

    def _fleet_make_card(self, card):
        frame = QFrame()
        frame.setObjectName('FleetCard')
        theme.bind(frame, lambda t: 'QFrame#FleetCard { border: 1px solid %s; '
                   'border-radius: 8px; background: %s; }' % (t['border_subtle'], t['bg_raised']))
        lay = QVBoxLayout(frame)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(3)
        top = QHBoxLayout()
        frame._title = QLabel()
        frame._title.setObjectName('DashCardTitle')
        top.addWidget(frame._title)
        top.addStretch()
        frame._state = QLabel()
        top.addWidget(frame._state)
        lay.addLayout(top)
        frame._line1 = QLabel()
        frame._line1.setObjectName('HintLabel')
        lay.addWidget(frame._line1)
        frame._line2 = QLabel()
        lay.addWidget(frame._line2)
        frame._line3 = QLabel()
        lay.addWidget(frame._line3)
        frame._cmdby = QLabel()
        frame._cmdby.setObjectName('HintLabel')
        frame._cmdby.setWordWrap(True)
        lay.addWidget(frame._cmdby)
        frame._result = QLabel()
        frame._result.setWordWrap(True)
        lay.addWidget(frame._result)
        buttons = QHBoxLayout()
        frame._drive = QPushButton('Drive')
        frame._drive.setCursor(Qt.PointingHandCursor)
        frame._drive.setToolTip("Point this Studio's control area at that station.")
        frame._drive.clicked.connect(lambda: self._fleet_drive(frame._station))
        buttons.addWidget(frame._drive)
        frame._start = QPushButton('Start ▾')
        frame._start.setCursor(Qt.PointingHandCursor)
        frame._start.setToolTip("Start that station's session: its own Studio launches it.")
        menu = QMenu(frame._start)
        menu.addAction('Simulation', lambda: self._fleet_session(frame._station, 'start',
                                                                 'simulation'))
        menu.addAction('Hardware…', lambda: self._fleet_session(frame._station, 'start',
                                                                'hardware'))
        frame._start.setMenu(menu)
        buttons.addWidget(frame._start)
        frame._stop = QPushButton('Stop')
        frame._stop.setCursor(Qt.PointingHandCursor)
        frame._stop.setToolTip("Stop that station's session.")
        frame._stop.clicked.connect(lambda: self._fleet_session(frame._station, 'stop'))
        buttons.addWidget(frame._stop)
        frame._web = QPushButton('Web')
        frame._web.setCursor(Qt.PointingHandCursor)
        frame._web.setToolTip("Open that Studio's web remote in a browser.")
        frame._web.clicked.connect(lambda: frame._url and QDesktopServices.openUrl(
            QUrl(frame._url)))
        buttons.addWidget(frame._web)
        buttons.addStretch()
        lay.addLayout(buttons)
        return frame

    def _fleet_fill_card(self, frame, card):
        n = card.get('station')
        frame._station = n
        frame._url = (card.get('urls') or [None])[0]
        own = card.get('self')
        frame._title.setText('%s%s' % (station_label(n), '  · this Studio' if own else ''))
        t = theme.current()
        if card.get('running'):
            state, colour = '● running', t['accent_green']
        elif card.get('has_joints'):
            state, colour = '● live', t['accent_teal']
        else:
            state, colour = '○ idle', t['text_secondary']
        frame._state.setText(state)
        frame._state.setStyleSheet('color: %s;' % colour)
        target = {'simulation': 'SIM', 'hardware': 'HARDWARE', 'vr': 'VR'}.get(
            card.get('target'), str(card.get('target', '')).upper())
        frame._line1.setText('%s · %s · %s' % (
            card.get('host', '?'), card.get('robot_display', ''), target))
        frame._line2.setText('Mode: %s' % (card.get('mode') or '—'))
        roles = []
        for p in card.get('teleop', []):
            if p.get('leader') == n:
                roles.append('leads %s' % station_label(p.get('follower')))
            elif p.get('follower') == n:
                roles.append('follows %s%s' % (station_label(p.get('leader')),
                                               ' (holding)' if p.get('holding') else ''))
        frame._line3.setText('Teleop: %s' % ', '.join(roles) if roles else '')
        frame._line3.setVisible(bool(roles))
        by = self._fleet_commanded_by(n) if not is_master(n) else []
        frame._cmdby.setText('Commanded by: %s' % ', '.join(by) if by else '')
        frame._cmdby.setVisible(bool(by))
        remote_robot = not own and not is_master(n)
        frame._drive.setVisible(remote_robot)
        frame._start.setVisible(remote_robot and not card.get('running'))
        frame._stop.setVisible(remote_robot and bool(card.get('running')))
        last = card.get('last_cmd') or {}
        from fic_studio.ros_node import _host_tag
        mine = (last.get('from_host') == _host_tag()
                and time.time() - last.get('ts', 0) < RESULT_SHOW_S)
        frame._result.setVisible(bool(mine))
        if mine:
            frame._result.setText(('✓ ' if last.get('ok') else '✗ ') + str(last.get('text', '')))
            colour = t['accent_green'] if last.get('ok') else t['accent_red']
            frame._result.setStyleSheet('color: %s;' % colour)
        frame._web.setVisible(bool(frame._url))

    #: Command topics of a robot station; a publisher on one can move the arm.
    _COMMAND_TOPICS = ('{rid}_desired_joint_angles', '{rid}_mode_select', '{rid}_control_mode',
                       'command_ee_position_change_{rid}', '{rid}_target_joint_angles',
                       '{rid}_gripper_width')

    def _fleet_commanded_by(self, n):
        """Return who can command robot station ``n``, besides its own Studio.

        Studios that Drive it (their presence card), FIC API scripts aimed at it
        (their announcements), and any other ROS node publishing on its command
        topics (from the ROS graph, by node name; refreshed every 2 s).
        """
        out = []
        for card in self._fleet_dir.studios():
            if n in (card.get('driving') or []) and card.get('station') != n:
                out.append('%s (%s)' % (station_label(card.get('station')), card.get('host', '?')))
        clients = self._fleet_clients.clients(n)
        for c in clients:
            out.append('%s on %s' % (c.get('script', 'script'), c.get('host', '?')))
        known = {c.get('node') for c in clients}
        cached = self._fleet_graph.get(n)
        if cached is None or time.monotonic() - cached[0] > 2.0:
            names = set()
            rid = station_rid(n)
            for fmt in self._COMMAND_TOPICS:
                try:
                    infos = self.node.get_publishers_info_by_topic('/' + fmt.format(rid=rid))
                except Exception:
                    infos = []
                names.update(i.node_name for i in infos)
            cached = (time.monotonic(), sorted(names))
            self._fleet_graph[n] = cached
        for name in cached[1]:
            if name.startswith(('fic_studio_', '_ros2cli_daemon')) or name in known:
                continue        # Studios are listed above; the ROS CLI daemon only listens
            out.append('node %s' % name)
        return out

    def _fleet_drive(self, n):
        """Point the control area at station ``n`` (another Studio's robot)."""
        card = self._fleet_dir.station(n) or {}
        if card.get('target') == 'hardware':
            reply = QMessageBox.question(
                self, 'Drive another robot',
                'Drive %s from here?\n\nIts arm is on HARDWARE, run by %s. Commands from this '
                'control area will move it.' % (station_label(n), card.get('host', '?')),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        self._fleet_drive_now(n)

    def _fleet_drive_now(self, n):
        """Point the control area at station ``n`` (no dialog: the caller has confirmed)."""
        card = self._fleet_dir.station(n) or {}
        self.node.add_robot(n)
        self._rc_active_rid = station_rid(n)
        self._rc_rebuild_robot_selector()
        self._rebuild_rc_control_area()
        self.log_message('WARN', 'Control area now drives %s (%s)'
                         % (station_label(n), card.get('host', '?')), 'Fleet')

    # ── Teleop controls ──────────────────────────────────────────────────
    def _teleop_fill_pickers(self, studios):
        for combo in (self._teleop_leader, self._teleop_follower):
            keep = combo.currentData()
            wanted = [(c['station'], '%s — %s' % (station_label(c['station']), c.get('host', '')))
                      for c in studios]
            current = [(combo.itemData(i), combo.itemText(i)) for i in range(combo.count())]
            if wanted == current:
                continue
            combo.blockSignals(True)
            combo.clear()
            for n, text in wanted:
                combo.addItem(text, n)
            idx = combo.findData(keep) if keep is not None else -1
            combo.setCurrentIndex(max(0, idx))
            combo.blockSignals(False)

    def _teleop_start_clicked(self):
        leader = self._teleop_leader.currentData()
        follower = self._teleop_follower.currentData()
        if leader is None or follower is None:
            return
        if leader == follower:
            QMessageBox.information(self, 'Teleoperation', 'Pick two different stations.')
            return
        lcard = self._fleet_dir.station(leader) or {}
        fcard = self._fleet_dir.station(follower) or {}
        if lcard.get('robot') and fcard.get('robot') and lcard['robot'] != fcard['robot']:
            QMessageBox.warning(self, 'Teleoperation', '%s is a %s and %s a %s: teleop between '
                                'different robots is not supported yet.' % (
                                    station_label(leader), lcard['robot'],
                                    station_label(follower), fcard['robot']))
            return
        self._teleop.request(leader, follower, scale=self._teleop_scale.value(),
                             max_speed=math.radians(self._teleop_speed.value()),
                             leader_robot=lcard.get('robot', ''),
                             leader_joints=int(lcard.get('joints', 0) or 0))
        self.log_message('INFO', 'Teleop requested: %s leads %s' % (
            station_label(leader), station_label(follower)), 'Teleop')

    def _teleop_fill_rows(self):
        while self._teleop_rows.count():
            item = self._teleop_rows.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        for s in sorted(self._teleop.visible(), key=lambda s: s.created, reverse=True):
            row = QWidget()
            lay = QHBoxLayout(row)
            lay.setContentsMargins(0, 0, 0, 0)
            text = '%s → %s · %s' % (station_label(s.leader), station_label(s.follower), s.state)
            if s.state == 'active' and s.follower == self.node.station_id:
                text += ' · holding (no leader data)' if s.holding else ' · following'
            if s.reason:
                text += ' · %s' % s.reason
            label = QLabel(text)
            lay.addWidget(label)
            lay.addStretch()
            if s.state in ('active', 'pending', 'requested'):
                stop = self._btn('Stop', 'BtnStop')
                stop.clicked.connect(lambda _=False, sid=s.sid: self._teleop.stop(
                    sid, 'stopped from %s' % station_label(self.node.station_id)))
                lay.addWidget(stop)
            self._teleop_rows.addWidget(row)

    def teleop_role(self):
        """Return 'leader', 'follower', 'waiting' or '' for the status bar."""
        teleop = getattr(self, '_teleop', None)
        if teleop is None:
            return ''
        if teleop.following():
            return 'follower'
        if teleop.leading():
            return 'leader'
        if any(s.state in ('requested', 'pending') and self.node.station_id in
               (s.leader, s.follower) for s in teleop.sessions.values()):
            return 'waiting'
        return ''
