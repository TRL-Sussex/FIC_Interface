"""Sequences extension — record + play robot waypoint sequences.

Migrated from fic_studio.mixins.sequence_mixin to the extension contract.
Joint state subscribers were extracted to the built-in JointStateMixin so
disabling this extension still leaves shared `current_joint_positions` /
`current_ee_pose` populated for safety and other readers.
"""
import math
import os
import re
import time

import yaml
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QInputDialog, QLabel, QListView,
    QListWidget, QListWidgetItem, QMessageBox, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)

from fic_studio.extensions import Extension as ExtensionBase
from Extensions._shared.theme_helpers import status_color

from . import trajectories as traj


SEQUENCES_YAML_PATH = os.path.expanduser("~/robot_sequences.yaml")


class Extension(ExtensionBase):
    ID = "sequences"
    NAME = "Sequences"
    DESCRIPTION = (
        "Record waypoints (joint, Cartesian, gripper), build motion sequences, "
        "and play them back with configurable hold times."
    )
    SIDEBAR_ICON = "▷"
    SIDEBAR_SECTION = "Programming"

    def __init__(self, host):
        super().__init__(host)
        self.sequences = {}
        self.current_sequence_name = None
        self.is_playing = False
        self.play_index = 0
        self.target_waypoint = None
        self.wait_start_time = None
        self.hold_start_time = None
        self.move_start_time = 0.0

        self.joint_tolerance_rad = math.radians(3.0)     # the page shows 3.0°
        self.position_tolerance_m = 0.015
        self.timeout_seconds = 30.0
        self._cart_settle_last_pos = None
        self._cart_settle_since = None

        self.play_timer = QTimer()
        self.play_timer.timeout.connect(self._check_position_reached)

        # Trajectory templates (trajectories.py): a stream of Cartesian targets at 60 Hz.
        self._traj = None
        self._traj_timer = QTimer()
        self._traj_timer.timeout.connect(self._traj_tick)

        # Per-station sequence-specific publisher (lazy-created in _execute_waypoint)
        self._page = None

    # ── Lifecycle ─────────────────────────────────────────────────────
    # ── Public API (used by the global control bar) ───────────────────
    #
    # The bar's Quick Actions and Snapshot buttons drive sequences through
    # these five methods. They are thin wrappers over the page's own
    # handlers on purpose: playback is a state machine (play_index, hold
    # timers, _check_position_reached) and there must be exactly one of it.

    def list_sequences(self) -> list:
        """Return the saved sequence names, sorted."""
        return sorted(self.sequences.keys())

    def play(self, name: str, loop: bool = False) -> None:
        """Select ``name`` and start playing it."""
        if name not in self.sequences:
            return
        self.sequence_dropdown.setCurrentText(name)
        self.seq_loop_checkbox.setChecked(bool(loop))
        self._play_sequence()

    def stop(self) -> None:
        """Stop playback (a sequence or a trajectory)."""
        self._stop_sequence()
        self.stop_trajectory()

    def new_sequence(self, name: str) -> str:
        """Make an empty sequence ``name`` and select it. Returns the name."""
        name = str(name or '').strip()
        if not name or name in self.sequences:
            raise ValueError('Give a new, unused name')
        self.sequences[name] = []
        if hasattr(self, 'sequence_dropdown'):
            self._refresh_sequence_dropdown()
            self.sequence_dropdown.setCurrentText(name)
        else:
            self.current_sequence_name = name
        self._write_yaml()
        return name

    def step(self, name: str) -> int:
        """Select ``name`` and move to its next waypoint (the first after the last). Returns its index."""
        if name not in self.sequences:
            raise ValueError('No such sequence')
        if self.is_playing or self._traj is not None:
            raise ValueError('Stop first')
        self.sequence_dropdown.setCurrentText(name)
        wps = self.sequences[name]
        if not wps:
            raise ValueError('This sequence has no waypoints')
        i = self.seq_waypoint_list.currentRow() + 1
        if not 0 <= i < len(wps):
            i = 0
        self.seq_waypoint_list.setCurrentRow(i)
        self._execute_waypoint(wps[i])
        self.seq_status_label.setText(f"Moving to waypoint {i + 1}")
        return i

    def speed(self) -> float:
        return float(self.seq_speed_scale.value()) if hasattr(self, 'seq_speed_scale') else 1.0

    def set_speed(self, value: float) -> None:
        """Playback speed scale (0.1 to 3): hold times are divided by it."""
        value = float(value)
        if not 0.1 <= value <= 3.0:
            raise ValueError('Speed 0.1 to 3')
        self.seq_speed_scale.setValue(value)

    def target_is_hardware(self) -> bool:
        return hasattr(self, 'seq_target') and self.seq_target.currentText() != 'Simulation'

    # ── Trajectory templates ──────────────────────────────────────────
    def run_trajectory(self, kind, size, period, cycles, plane, hardware_ok=False) -> float:
        """Trace a trajectory template (trajectories.py) from the end effector's current pose,
        holding its orientation, on the sequences' target. Returns its duration (s)."""
        if self.is_playing or self._traj is not None:
            raise ValueError('Stop first')
        hw = self.target_is_hardware()
        if hw and hardware_ok is not True:
            raise ValueError('Confirm moving the real robot')
        kind, size, period, cycles, plane = traj.check(kind, size, period, cycles, plane,
                                                       'hardware' if hw else 'simulation')
        ee = self.host.current_ee_pose
        if ee is None:
            raise ValueError('No end-effector pose yet: start the controller first')
        mode = self.host.control_mode() if hasattr(self.host, 'control_mode') else ''
        if mode and not mode.startswith('Cartesian'):
            if hw:
                raise ValueError('Switch the real robot to a Cartesian mode first (Cart panel)')
            self.host.set_mode('Cartesian Planner')
            self.host.log('INFO', 'Trajectory: switched to Cartesian Impedance with Planner', 'Sequence')
        start = [ee.position.x, ee.position.y, ee.position.z]
        q = ee.orientation
        self._traj = {'kind': kind, 'samples': traj.samples(start, kind, size, period, cycles, plane),
                      'quat': (q.x, q.y, q.z, q.w), 't0': time.monotonic(), 'last': -1}
        self._traj_timer.start(int(1000 / traj.RATE_HZ))
        self.host.log('INFO', '▶ Trajectory: %s, %.0f mm, %.1f s period, %d cycle(s), plane %s%s'
                      % (traj.KINDS[kind], size * 1e3, period, cycles, plane, ' — REAL ROBOT' if hw else ''),
                      'Sequence')
        return traj.duration(period, cycles)

    def stop_trajectory(self) -> None:
        if self._traj is not None:
            self._traj_timer.stop()
            self._traj = None
            self.host.log('WARN', '■ Trajectory stopped', 'Sequence')

    def trajectory_state(self) -> dict:
        t = self._traj
        if t is None:
            return {'running': False}
        n = len(t['samples'])
        return {'running': True, 'kind': t['kind'], 'progress': round(max(0, t['last']) / max(1, n - 1), 3)}

    def save_trajectory(self, name, kind, size, plane, per_cycle=16) -> str:
        """One cycle of a template as a sequence of Cartesian waypoints from the current pose, to
        compare a sequence (reach each point, then the next) with the streamed trajectory."""
        kind, size, _, _, plane = traj.check(kind, size, traj.PERIOD[1], 1, plane)
        ee = self.host.current_ee_pose
        if ee is None:
            raise ValueError('No end-effector pose yet: start the controller first')
        aa = self._quat_to_axis_angle_deg(ee.orientation)
        return self.add_sequence(name, traj.waypoints([ee.position.x, ee.position.y, ee.position.z], aa,
                                                      kind, size, plane, per_cycle=int(per_cycle)))

    @staticmethod
    def _quat_to_axis_angle_deg(q):
        qw = max(-1.0, min(1.0, q.w))
        angle = 2.0 * math.acos(qw)
        sn = math.sqrt(max(0.0, 1.0 - qw * qw))
        axis = (1.0, 0.0, 0.0) if sn < 0.001 else (q.x / sn, q.y / sn, q.z / sn)
        return [axis[0], axis[1], axis[2], math.degrees(angle)]

    def _traj_tick(self):
        t = self._traj
        if t is None:
            self._traj_timer.stop()
            return
        i = int((time.monotonic() - t['t0']) * traj.RATE_HZ)          # late ticks skip ahead
        if i >= len(t['samples']):
            i = len(t['samples']) - 1
        if i != t['last']:
            from geometry_msgs.msg import Pose, Point, Quaternion
            p = t['samples'][i][1]
            qx, qy, qz, qw = t['quat']
            self._publish_pose(Pose(position=Point(x=p[0], y=p[1], z=p[2]),
                                    orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)))
            t['last'] = i
        if i >= len(t['samples']) - 1:
            self._traj_timer.stop()
            self._traj = None
            self.host.log('INFO', '✓ Trajectory finished', 'Sequence')

    def _publish_pose(self, pose):
        """A Cartesian target to the sequences' target (the simulator or a real robot)."""
        node = self.host.node
        if not self.target_is_hardware():
            node.publish_position(pose)
            return True
        rid = self._target_topic_rid() or node.station_robot_id
        pub = node.pubs.get(rid, {}).get('position')
        if pub is None:
            self.host.log("ERROR", f"No Cart pose publisher for {rid}", rid)
            return False
        pub.publish(pose)
        return True

    def add_sequence(self, name: str, waypoints: list) -> str:
        """Store ``waypoints`` as the sequence ``name`` (replacing one of that
        name), select it and save the file. No dialogs: Teach Mode calls this
        from its web page too. Returns the name."""
        name = str(name or '').strip()
        if not name:
            raise ValueError('Give the sequence a name')
        if self.is_playing:
            raise ValueError('Stop the sequence first')
        self.sequences[name] = [dict(w) for w in waypoints]
        if hasattr(self, 'sequence_dropdown'):
            self._refresh_sequence_dropdown()
            self.sequence_dropdown.setCurrentText(name)
        else:
            self.current_sequence_name = name
        self._write_yaml()
        return name

    def capture(self, kind: str, into: str = None) -> None:
        """Append the live pose to ``into`` as a ``kind`` waypoint.

        ``kind`` is one of joint / cartesian / gripper, matching the
        capture-mode dropdown on the page.
        """
        if into is not None:
            if into not in self.sequences:
                return
            self.sequence_dropdown.setCurrentText(into)
        self.seq_capture_mode.setCurrentText(kind.capitalize())
        self._capture_position()

    def teardown(self) -> None:
        self._stop_sequence()
        self.stop_trajectory()

    def on_emergency_stop(self) -> None:
        if self.is_playing:
            self.host.log("WARN", "Emergency stop — halting sequence", "Sequence")
            self._stop_sequence()
        if self._traj is not None:
            self.host.log("WARN", "Emergency stop — halting trajectory", "Sequence")
            self.stop_trajectory()

    @property
    def state_running(self) -> bool:
        return self.is_playing

    # ── Web page (manifest "web"; web/page.js) ────────────────────────
    # The web page drives the same state machine as the Qt page and keeps it in
    # sync (the dropdown, list and status follow). No dialog boxes here: a
    # browser or a headless board cannot click them, so a refusal is a
    # ValueError the page shows.

    def web_state(self) -> dict:
        wps = self.sequences.get(self.current_sequence_name, []) if self.current_sequence_name else []
        page_built = hasattr(self, 'seq_target')
        target = self.seq_target.currentText() if page_built else 'Simulation'
        return {
            'sequences': [{'name': n, 'count': len(self.sequences[n])} for n in sorted(self.sequences)],
            'current': self.current_sequence_name,
            'waypoints': [self._web_waypoint(i, wp) for i, wp in enumerate(wps)],
            'playing': self.is_playing,
            'play_index': self.play_index if self.is_playing else None,
            'selected': self.seq_waypoint_list.currentRow() if page_built else -1,
            'status': self.seq_status_label.text() if page_built else ('Playing' if self.is_playing else 'Ready'),
            'loop': self.seq_loop_checkbox.isChecked() if page_built else False,
            'targets': [self.seq_target.itemText(i) for i in range(self.seq_target.count())] if page_built else ['Simulation'],
            'target': target,
            'hardware': target != 'Simulation',
            'joints': self.host.robots.get(self.host.current_robot_name(), 7),
            'speed': self.seq_speed_scale.value() if page_built else 1.0,
            'joint_tol_deg': round(math.degrees(self.joint_tolerance_rad), 2),
            'pos_tol_mm': round(self.position_tolerance_m * 1000.0, 1),
            'timeout_s': self.timeout_seconds,
            'file': SEQUENCES_YAML_PATH,
            'trajectory': self.trajectory_state(),
            'trajectory_kinds': [{'value': k, 'label': v} for k, v in traj.KINDS.items()],
            'trajectory_limits': {'size': traj.SIZE, 'period': traj.PERIOD, 'cycles': traj.CYCLES,
                                  'peak_speed': traj.PEAK_SPEED},
        }

    @staticmethod
    def _web_waypoint(i, wp):
        """One waypoint for the page: its values as well as its one-line text."""
        out = {'index': i, 'type': wp.get('type', 'joint'), 'hold': wp.get('hold_time', 1.0),
               'text': re.sub(r'^(JOINT|CART|GRIPPER)\s*', '', Extension._format_waypoint(i, wp).split('. ', 1)[-1]).rsplit('  (', 1)[0]}
        for key in ('values', 'position', 'axis_angle', 'width'):
            if key in wp:
                out[key] = wp[key]
        return out

    @staticmethod
    def _web_parse_waypoint(args, joints):
        """A waypoint typed on the page: {'type', values | position + axis_angle | width, 'hold'}."""
        kind = str(args.get('type') or '')
        try:
            hold = float(args.get('hold', 1.0))
        except (TypeError, ValueError):
            raise ValueError('Hold time must be a number')
        if not 0.0 <= hold <= 600.0:
            raise ValueError('Hold time must be between 0 and 600 s')

        def numbers(key, count):
            vals = args.get(key)
            if not isinstance(vals, (list, tuple)) or len(vals) != count:
                raise ValueError(f'{key} needs {count} numbers')
            try:
                out = [float(v) for v in vals]
            except (TypeError, ValueError):
                raise ValueError(f'{key} needs {count} numbers')
            if not all(math.isfinite(v) for v in out):
                raise ValueError(f'{key} needs {count} numbers')
            return out

        if kind == 'joint':
            values = numbers('values', joints)
            if any(abs(v) > 360.0 for v in values):
                raise ValueError('Joint angles are in degrees, within ±360')
            return {'type': 'joint', 'values': values, 'hold_time': hold}
        if kind == 'cartesian':
            pos = numbers('position', 3)
            aa = numbers('axis_angle', 4)
            if any(abs(v) > 3.0 for v in pos):
                raise ValueError('Positions are in metres, within ±3 m')
            if math.sqrt(sum(a * a for a in aa[:3])) < 1e-6:
                raise ValueError('The rotation axis cannot be zero')
            return {'type': 'cartesian', 'position': pos, 'axis_angle': aa, 'hold_time': hold}
        if kind == 'gripper':
            width = numbers('width', 1)[0] if isinstance(args.get('width'), list) else args.get('width')
            try:
                width = float(width)
            except (TypeError, ValueError):
                raise ValueError('Gripper width must be a number')
            if not 0.0 <= width <= 0.08:
                raise ValueError('Gripper width must be between 0 and 0.08 m')
            return {'type': 'gripper', 'width': width, 'hold_time': hold}
        raise ValueError('The waypoint type is joint, cartesian or gripper')

    def web_action(self, name: str, args: dict):
        seq = lambda: self.sequences.get(self.current_sequence_name, [])  # noqa: E731

        def index():
            i = int(args.get('index', -1))
            if not 0 <= i < len(seq()):
                raise ValueError('No such waypoint')
            return i

        def busy():
            if self.is_playing:
                raise ValueError('Stop the sequence first')

        def moves_robot():
            # Read the target now: the page asked with the target of its last poll.
            if self.seq_target.currentText() != 'Simulation' and args.get('hardware_ok') is not True:
                raise ValueError('Confirm moving the real robot')

        if name == 'select':
            if args.get('name') not in self.sequences:
                raise ValueError('No such sequence')
            busy()
            self.sequence_dropdown.setCurrentText(args['name'])
        elif name == 'new':
            new = str(args.get('name') or '').strip()
            if not new or new in self.sequences:
                raise ValueError('Give a new, unused name')
            self.sequences[new] = []
            self._refresh_sequence_dropdown()
            self.sequence_dropdown.setCurrentText(new)
        elif name == 'rename':
            busy()
            new = str(args.get('name') or '').strip()
            if not self.current_sequence_name or not new or new in self.sequences:
                raise ValueError('Give a new, unused name')
            self.sequences[new] = self.sequences.pop(self.current_sequence_name)
            self._refresh_sequence_dropdown()
            self.sequence_dropdown.setCurrentText(new)
        elif name == 'delete_sequence':
            busy()
            if len(self.sequences) <= 1:
                raise ValueError('The last sequence cannot be deleted')
            del self.sequences[self.current_sequence_name]
            self._refresh_sequence_dropdown()
        elif name == 'play':
            busy()
            if not seq():
                raise ValueError('This sequence has no waypoints')
            moves_robot()
            self.seq_loop_checkbox.setChecked(bool(args.get('loop', self.seq_loop_checkbox.isChecked())))
            self._play_sequence()
        elif name == 'step':
            busy()
            if not seq():
                raise ValueError('This sequence has no waypoints')
            moves_robot()
            i = self.seq_waypoint_list.currentRow() + 1
            if not 0 <= i < len(seq()):
                i = 0
            self.seq_waypoint_list.setCurrentRow(i)
            self._execute_waypoint(seq()[i])
            self.seq_status_label.setText(f"Moving to waypoint {i + 1}")
            return {'index': i}
        elif name == 'stop':
            self.stop()
        elif name == 'trajectory':
            return {'duration': self.run_trajectory(args.get('kind'), args.get('size'), args.get('period'),
                                                    args.get('cycles'), args.get('plane'),
                                                    hardware_ok=args.get('hardware_ok') is True)}
        elif name == 'save_trajectory':
            busy()
            return {'name': self.save_trajectory(args.get('name'), args.get('kind'), args.get('size'),
                                                 args.get('plane'), int(args.get('per_cycle', 16)))}
        elif name == 'loop':
            self.seq_loop_checkbox.setChecked(bool(args.get('value')))
        elif name == 'target':
            busy()
            if self.seq_target.findText(str(args.get('value'))) < 0:
                raise ValueError('No such target')
            self.seq_target.setCurrentText(str(args['value']))
        elif name == 'goto':
            busy()
            i = index()
            moves_robot()
            self._execute_waypoint(seq()[i])
            self.seq_waypoint_list.setCurrentRow(i)
            self.seq_status_label.setText(f"Moving to waypoint {i + 1}")
        elif name == 'capture':
            busy()
            wp = self._web_live_waypoint(str(args.get('kind') or 'joint'), float(args.get('hold', 1.0)))
            seq().append(wp)
            self._refresh_waypoint_list()
            self.host.log("INFO", f"Captured {wp['type']} waypoint to '{self.current_sequence_name}' (web)", "Sequence")
        elif name in ('add', 'update'):
            busy()
            if not self.current_sequence_name:
                raise ValueError('Make a sequence first')
            wp = self._web_parse_waypoint(args.get('waypoint') or {}, self.host.robots.get(self.host.current_robot_name(), 7))
            if name == 'add':
                seq().append(wp)
                i = len(seq()) - 1
            else:
                i = index()
                seq()[i] = wp
            self._refresh_waypoint_list()
            self.seq_waypoint_list.setCurrentRow(i)
            return {'index': i}
        elif name == 'settings':
            ranges = {'speed': (self.seq_speed_scale, 0.1, 3.0), 'joint_tol_deg': (self.seq_joint_tol, 0.5, 20.0),
                      'pos_tol_mm': (self.seq_pos_tol, 1.0, 50.0), 'timeout_s': (self.seq_timeout, 5.0, 120.0)}
            key = str(args.get('key'))
            if key not in ranges:
                raise ValueError('No such setting')
            box, lo, hi = ranges[key]
            try:
                value = float(args.get('value'))
            except (TypeError, ValueError):
                raise ValueError('Give a number')
            if not lo <= value <= hi:
                raise ValueError(f'Between {lo:g} and {hi:g}')
            box.setValue(value)                 # its valueChanged sets the tolerance or timeout
        elif name == 'delete':
            busy()
            seq().pop(index())
            self._refresh_waypoint_list()
        elif name == 'move':
            busy()
            i, j = index(), index() + (-1 if args.get('dir') == 'up' else 1)
            if not 0 <= j < len(seq()):
                raise ValueError('Already at the end')
            seq()[i], seq()[j] = seq()[j], seq()[i]
            self._refresh_waypoint_list()
        elif name == 'hold':
            try:
                hold = float(args.get('value'))
            except (TypeError, ValueError):
                raise ValueError('Hold time must be a number')
            if not 0.0 <= hold <= 600.0:
                raise ValueError('Hold time must be between 0 and 600 s')
            seq()[index()]['hold_time'] = hold
            self._refresh_waypoint_list()
        elif name == 'save':
            self._write_yaml()
        elif name == 'load':
            busy()
            if not os.path.exists(SEQUENCES_YAML_PATH):
                raise ValueError(f'No file at {SEQUENCES_YAML_PATH}')
            self._load_sequences_from_yaml(silent=True)
        else:
            raise ValueError(f'No such action: {name}')
        return {}

    def _web_live_waypoint(self, kind: str, hold: float) -> dict:
        """A waypoint from the robot's live state (the Qt page's Read + Capture, without dialogs)."""
        if not 0.0 <= hold <= 600.0:
            raise ValueError('Hold time must be between 0 and 600 s')
        if kind == 'joint':
            joints = self.host.current_joint_positions
            if not joints:
                raise ValueError('No joint feedback yet: start the controller first')
            return {'type': 'joint', 'values': [round(math.degrees(q), 3) for q in joints], 'hold_time': hold}
        if kind == 'cartesian':
            ee = self.host.current_ee_pose
            if ee is None:
                raise ValueError('No end-effector pose yet: start the controller first')
            qx, qy, qz, qw = ee.orientation.x, ee.orientation.y, ee.orientation.z, ee.orientation.w
            angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
            sn = math.sqrt(max(0.0, 1.0 - qw * qw))
            axis = (1.0, 0.0, 0.0) if sn < 0.001 else (qx / sn, qy / sn, qz / sn)
            return {'type': 'cartesian', 'position': [ee.position.x, ee.position.y, ee.position.z],
                    'axis_angle': [axis[0], axis[1], axis[2], math.degrees(angle)], 'hold_time': hold}
        if kind == 'gripper':
            if self.seq_target.currentText() == "Simulation":
                raise ValueError('No gripper reading in simulation')
            slider = getattr(self.host.main_window, f'{self._get_target_robot_id()}_gripper_slider', None)
            if slider is None:
                raise ValueError('No gripper on this robot')
            return {'type': 'gripper', 'width': (slider.value() / 100.0) * 0.08, 'hold_time': hold}
        raise ValueError('Capture joint, cartesian or gripper')

    # ── Build ─────────────────────────────────────────────────────────
    def build_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(12)
        layout.setAlignment(Qt.AlignTop)

        title_row = QHBoxLayout()
        title_row.addWidget(self.host.make_page_title("Sequence Programming"))
        title_row.addStretch()
        layout.addLayout(title_row)

        # Sequence management row
        seq_mgmt_row = QHBoxLayout()
        seq_mgmt_row.setSpacing(8)
        seq_mgmt_row.addWidget(QLabel("Sequence:"))
        self.sequence_dropdown = QComboBox()
        self.sequence_dropdown.setMinimumWidth(200)
        self.sequence_dropdown.currentTextChanged.connect(self._on_sequence_selected)
        seq_mgmt_row.addWidget(self.sequence_dropdown, 1)

        new_seq_btn = self.host.make_button("+ NEW", "BtnTeal")
        new_seq_btn.clicked.connect(self._create_new_sequence)
        seq_mgmt_row.addWidget(new_seq_btn)
        rename_seq_btn = self.host.make_button("RENAME")
        rename_seq_btn.clicked.connect(self._rename_sequence)
        seq_mgmt_row.addWidget(rename_seq_btn)
        delete_seq_btn = self.host.make_button("DELETE", "BtnStop")
        delete_seq_btn.clicked.connect(self._delete_sequence)
        seq_mgmt_row.addWidget(delete_seq_btn)
        seq_mgmt_row.addStretch()
        save_yaml_btn = self.host.make_button("SAVE TO YAML", "BtnWarn")
        save_yaml_btn.clicked.connect(self._save_sequences_to_yaml)
        seq_mgmt_row.addWidget(save_yaml_btn)
        load_yaml_btn = self.host.make_button("LOAD FROM YAML", "BtnPrimary")
        load_yaml_btn.clicked.connect(self._load_sequences_from_yaml)
        seq_mgmt_row.addWidget(load_yaml_btn)
        layout.addLayout(seq_mgmt_row)

        # Two-column body
        content_row = QHBoxLayout()
        content_row.setSpacing(15)

        # LEFT — capture
        left_col = QVBoxLayout()
        left_col.setSpacing(10)

        capture_grp = QGroupBox("CAPTURE POSITION")
        capture_layout = QVBoxLayout(capture_grp)
        capture_layout.setSpacing(8)

        target_row = QHBoxLayout()
        target_row.addWidget(QLabel("Target:"))
        self.seq_target = QComboBox()
        self.seq_target.addItems(["Simulation", "Robot 1", "Robot 2", "Robot 3", "Robot 4"])
        self.seq_target.setToolTip("Select where to send commands: simulation or real robot")
        target_row.addWidget(self.seq_target)
        target_row.addStretch()
        capture_layout.addLayout(target_row)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Mode:"))
        self.seq_capture_mode = QComboBox()
        self.seq_capture_mode.addItems(["Joint", "Cartesian", "Gripper"])
        self.seq_capture_mode.currentTextChanged.connect(self._on_capture_mode_changed)
        mode_row.addWidget(self.seq_capture_mode)
        mode_row.addStretch()
        capture_layout.addLayout(mode_row)

        self.seq_instruction_label = QLabel(
            "1. Set target to real robot and enable Joint Admittance mode\n"
            "2. Physically move robot to desired position\n"
            "3. Click READ FROM ROBOT to capture current position\n"
            "4. Click CAPTURE to save as waypoint"
        )
        self.seq_instruction_label.setWordWrap(True)
        self.seq_instruction_label.setObjectName("HintLabel")
        capture_layout.addWidget(self.seq_instruction_label)

        # Joint inputs
        self.seq_joint_widget = QWidget()
        joint_layout = QGridLayout(self.seq_joint_widget)
        joint_layout.setSpacing(5)
        joint_layout.setContentsMargins(0, 5, 0, 5)
        self.seq_joint_inputs = []
        for i in range(7):
            lbl = QLabel(f"J{i+1}:")
            lbl.setObjectName('HintLabel')
            spin = QDoubleSpinBox()
            spin.setRange(-360.0, 360.0); spin.setDecimals(2); spin.setSuffix("°"); spin.setValue(0.0)
            row, col = divmod(i, 2)
            joint_layout.addWidget(lbl, row, col * 2)
            joint_layout.addWidget(spin, row, col * 2 + 1)
            self.seq_joint_inputs.append(spin)
        capture_layout.addWidget(self.seq_joint_widget)

        # Cartesian inputs
        self.seq_cart_widget = QWidget()
        cart_layout = QGridLayout(self.seq_cart_widget)
        cart_layout.setSpacing(5)
        cart_layout.setContentsMargins(0, 5, 0, 5)
        cart_layout.addWidget(QLabel("X:"), 0, 0)
        self.seq_cart_x = QDoubleSpinBox(); self.seq_cart_x.setRange(-2.0, 2.0); self.seq_cart_x.setDecimals(4); self.seq_cart_x.setSuffix(" m")
        cart_layout.addWidget(self.seq_cart_x, 0, 1)
        cart_layout.addWidget(QLabel("Y:"), 0, 2)
        self.seq_cart_y = QDoubleSpinBox(); self.seq_cart_y.setRange(-2.0, 2.0); self.seq_cart_y.setDecimals(4); self.seq_cart_y.setSuffix(" m")
        cart_layout.addWidget(self.seq_cart_y, 0, 3)
        cart_layout.addWidget(QLabel("Z:"), 1, 0)
        self.seq_cart_z = QDoubleSpinBox(); self.seq_cart_z.setRange(-2.0, 2.0); self.seq_cart_z.setDecimals(4); self.seq_cart_z.setSuffix(" m")
        cart_layout.addWidget(self.seq_cart_z, 1, 1)
        cart_layout.addWidget(QLabel("Axis X:"), 2, 0)
        self.seq_cart_ax = QDoubleSpinBox(); self.seq_cart_ax.setRange(-1.0, 1.0); self.seq_cart_ax.setDecimals(4)
        cart_layout.addWidget(self.seq_cart_ax, 2, 1)
        cart_layout.addWidget(QLabel("Axis Y:"), 2, 2)
        self.seq_cart_ay = QDoubleSpinBox(); self.seq_cart_ay.setRange(-1.0, 1.0); self.seq_cart_ay.setDecimals(4)
        cart_layout.addWidget(self.seq_cart_ay, 2, 3)
        cart_layout.addWidget(QLabel("Axis Z:"), 3, 0)
        self.seq_cart_az = QDoubleSpinBox(); self.seq_cart_az.setRange(-1.0, 1.0); self.seq_cart_az.setDecimals(4); self.seq_cart_az.setValue(1.0)
        cart_layout.addWidget(self.seq_cart_az, 3, 1)
        cart_layout.addWidget(QLabel("Angle:"), 3, 2)
        self.seq_cart_angle = QDoubleSpinBox(); self.seq_cart_angle.setRange(-360.0, 360.0); self.seq_cart_angle.setDecimals(2); self.seq_cart_angle.setSuffix("°")
        cart_layout.addWidget(self.seq_cart_angle, 3, 3)
        capture_layout.addWidget(self.seq_cart_widget)
        self.seq_cart_widget.hide()

        # Gripper inputs
        self.seq_gripper_widget = QWidget()
        gripper_layout = QHBoxLayout(self.seq_gripper_widget)
        gripper_layout.setContentsMargins(0, 5, 0, 5); gripper_layout.setSpacing(10)
        gripper_layout.addWidget(QLabel("Gripper Width:"))
        self.seq_gripper_width = QDoubleSpinBox()
        self.seq_gripper_width.setRange(0.0, 0.08); self.seq_gripper_width.setDecimals(4)
        self.seq_gripper_width.setSuffix(" m"); self.seq_gripper_width.setValue(0.08); self.seq_gripper_width.setSingleStep(0.005)
        gripper_layout.addWidget(self.seq_gripper_width)
        self.seq_gripper_pct = QLabel("100%")
        self.seq_gripper_pct.setObjectName("AccentValue")
        gripper_layout.addWidget(self.seq_gripper_pct)
        self.seq_gripper_width.valueChanged.connect(
            lambda v: self.seq_gripper_pct.setText(f"{(v / 0.08) * 100:.0f}%"))
        open_btn = self.host.make_button("OPEN")
        open_btn.clicked.connect(lambda: self.seq_gripper_width.setValue(0.08))
        gripper_layout.addWidget(open_btn)
        close_btn = self.host.make_button("CLOSE")
        close_btn.clicked.connect(lambda: self.seq_gripper_width.setValue(0.0))
        gripper_layout.addWidget(close_btn)
        gripper_layout.addStretch()
        capture_layout.addWidget(self.seq_gripper_widget)
        self.seq_gripper_widget.hide()

        # Hold time
        timing_row = QHBoxLayout()
        timing_row.addWidget(QLabel("Hold Time:"))
        self.seq_hold_time = QDoubleSpinBox()
        self.seq_hold_time.setRange(0.0, 60.0); self.seq_hold_time.setDecimals(1); self.seq_hold_time.setValue(1.0); self.seq_hold_time.setSuffix(" s")
        timing_row.addWidget(self.seq_hold_time)
        timing_row.addStretch()
        capture_layout.addLayout(timing_row)

        capture_btn_row = QHBoxLayout()
        read_robot_btn = self.host.make_button("READ FROM ROBOT", "BtnPrimary")
        read_robot_btn.clicked.connect(self._read_robot_position)
        capture_btn_row.addWidget(read_robot_btn)
        capture_btn = self.host.make_button("CAPTURE POSITION", "BtnTeal")
        capture_btn.clicked.connect(self._capture_position)
        capture_btn_row.addWidget(capture_btn)
        capture_layout.addLayout(capture_btn_row)
        left_col.addWidget(capture_grp)

        # Mode controls
        mode_grp = QGroupBox("ROBOT MODE (Real Robot Only)")
        mode_layout = QVBoxLayout(mode_grp)
        mode_layout.setSpacing(8)
        mode_info = QLabel(
            "Teaching: Enable Joint Admittance to physically move robot\n"
            "Playback: Use Joint Impedance with Planner for smooth motion")
        mode_info.setObjectName("HintLabel"); mode_info.setWordWrap(True)
        mode_layout.addWidget(mode_info)
        mode_btn_row = QHBoxLayout()
        teach_mode_btn = self.host.make_button("TEACH MODE", "BtnWarn")
        teach_mode_btn.setToolTip(
            "Joint capture mode → Joint Admittance; Cartesian capture mode → Cartesian Admittance.\n"
            "Lets you move the robot by hand to teach waypoints in the active mode.")
        teach_mode_btn.clicked.connect(self._set_teach_mode)
        mode_btn_row.addWidget(teach_mode_btn)
        playback_mode_btn = self.host.make_button("PLAYBACK MODE", "BtnPrimary")
        playback_mode_btn.setToolTip(
            "Joint capture mode → Joint Impedance With Planner; Cartesian capture mode → Cartesian Planner.\n"
            "Smooth automated motion in the active mode for replaying captured waypoints.")
        playback_mode_btn.clicked.connect(self._set_playback_mode)
        mode_btn_row.addWidget(playback_mode_btn)
        mode_layout.addLayout(mode_btn_row)
        left_col.addWidget(mode_grp)

        # Playback controls
        playback_grp = QGroupBox("PLAYBACK")
        playback_layout = QVBoxLayout(playback_grp)
        playback_layout.setSpacing(8)
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed Scale:"))
        self.seq_speed_scale = QDoubleSpinBox()
        self.seq_speed_scale.setRange(0.1, 3.0); self.seq_speed_scale.setDecimals(1); self.seq_speed_scale.setValue(1.0); self.seq_speed_scale.setSuffix("x")
        speed_row.addWidget(self.seq_speed_scale)
        self.seq_loop_checkbox = QCheckBox("Loop")
        speed_row.addWidget(self.seq_loop_checkbox)
        speed_row.addStretch()
        playback_layout.addLayout(speed_row)

        tol_row = QHBoxLayout()
        tol_row.addWidget(QLabel("Joint Tol:"))
        self.seq_joint_tol = QDoubleSpinBox()
        self.seq_joint_tol.setRange(0.5, 20.0); self.seq_joint_tol.setDecimals(1); self.seq_joint_tol.setValue(3.0); self.seq_joint_tol.setSuffix("°")
        self.seq_joint_tol.setToolTip("Joint position tolerance in degrees")
        self.seq_joint_tol.valueChanged.connect(
            lambda v: setattr(self, 'joint_tolerance_rad', math.radians(v)))
        tol_row.addWidget(self.seq_joint_tol)
        tol_row.addWidget(QLabel("Pos Tol:"))
        self.seq_pos_tol = QDoubleSpinBox()
        self.seq_pos_tol.setRange(1.0, 50.0); self.seq_pos_tol.setDecimals(0); self.seq_pos_tol.setValue(15.0); self.seq_pos_tol.setSuffix("mm")
        self.seq_pos_tol.setToolTip("Cartesian position tolerance in mm.")
        self.seq_pos_tol.valueChanged.connect(
            lambda v: setattr(self, 'position_tolerance_m', v / 1000.0))
        tol_row.addWidget(self.seq_pos_tol)
        tol_row.addWidget(QLabel("Timeout:"))
        self.seq_timeout = QDoubleSpinBox()
        self.seq_timeout.setRange(5.0, 120.0); self.seq_timeout.setDecimals(0); self.seq_timeout.setValue(30.0); self.seq_timeout.setSuffix("s")
        self.seq_timeout.setToolTip("Max time to wait for position")
        self.seq_timeout.valueChanged.connect(
            lambda v: setattr(self, 'timeout_seconds', v))
        tol_row.addWidget(self.seq_timeout)
        tol_row.addStretch()
        playback_layout.addLayout(tol_row)

        play_btn_row = QHBoxLayout()
        self.seq_play_btn = self.host.make_button("▶ PLAY", "BtnTeal")
        self.seq_play_btn.clicked.connect(self._play_sequence)
        play_btn_row.addWidget(self.seq_play_btn)
        self.seq_stop_btn = self.host.make_button("■ STOP", "BtnStop")
        self.seq_stop_btn.clicked.connect(self._stop_sequence)
        self.seq_stop_btn.setEnabled(False)
        play_btn_row.addWidget(self.seq_stop_btn)
        step_btn = self.host.make_button("STEP →")
        step_btn.clicked.connect(self._step_sequence)
        play_btn_row.addWidget(step_btn)
        playback_layout.addLayout(play_btn_row)

        self.seq_status_label = QLabel("Ready")
        self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('ready'))
        playback_layout.addWidget(self.seq_status_label)

        left_col.addWidget(playback_grp)
        left_col.addStretch()
        content_row.addLayout(left_col)

        # RIGHT — waypoint list
        right_col = QVBoxLayout()
        right_col.setSpacing(8)
        waypoints_grp = QGroupBox("WAYPOINTS")
        waypoints_layout = QVBoxLayout(waypoints_grp)
        waypoints_layout.setSpacing(8)
        self.seq_waypoint_list = QListWidget()
        self.seq_waypoint_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.seq_waypoint_list.setMinimumHeight(300)
        self.seq_waypoint_list.itemClicked.connect(self._on_waypoint_selected)
        self.seq_waypoint_list.itemDoubleClicked.connect(self._goto_waypoint)
        waypoints_layout.addWidget(self.seq_waypoint_list)

        wp_btn_row = QHBoxLayout()
        for label, style, slot in [
            ("↑ UP", None, self._move_waypoint_up),
            ("↓ DOWN", None, self._move_waypoint_down),
            ("UPDATE", "BtnPrimary", self._update_waypoint),
            ("DELETE", "BtnStop", self._delete_waypoint),
        ]:
            btn = self.host.make_button(label, style)
            btn.clicked.connect(slot)
            wp_btn_row.addWidget(btn)
        waypoints_layout.addLayout(wp_btn_row)

        goto_row = QHBoxLayout()
        goto_btn = self.host.make_button("GO TO SELECTED", "BtnTeal")
        goto_btn.clicked.connect(self._goto_waypoint)
        goto_row.addWidget(goto_btn)
        goto_row.addStretch()
        waypoints_layout.addLayout(goto_row)

        right_col.addWidget(waypoints_grp)
        content_row.addLayout(right_col, 1)
        layout.addLayout(content_row, 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(page)

        # Initialise sequence data — was _init_sequence_data
        self._load_initial_sequences()
        self._page = scroll
        return scroll

    def _load_initial_sequences(self):
        if os.path.exists(SEQUENCES_YAML_PATH):
            try:
                self._load_sequences_from_yaml(silent=True)
            except Exception as e:
                print(f"Could not load sequences: {e}")
        if not self.sequences:
            self.sequences["Default Sequence"] = []
            self._refresh_sequence_dropdown()
            self.sequence_dropdown.setCurrentText("Default Sequence")

    # ── Helpers ───────────────────────────────────────────────────────
    def _refresh_sequence_dropdown(self):
        current = self.sequence_dropdown.currentText()
        self.sequence_dropdown.blockSignals(True)
        self.sequence_dropdown.clear()
        self.sequence_dropdown.addItems(sorted(self.sequences.keys()))
        if current in self.sequences:
            self.sequence_dropdown.setCurrentText(current)
        elif self.sequences:
            self.sequence_dropdown.setCurrentIndex(0)
        self.sequence_dropdown.blockSignals(False)
        self._on_sequence_selected(self.sequence_dropdown.currentText())

    def _on_sequence_selected(self, name):
        self.current_sequence_name = name
        self._refresh_waypoint_list()

    def _refresh_waypoint_list(self):
        self.seq_waypoint_list.clear()
        if not self.current_sequence_name:
            return
        for i, wp in enumerate(self.sequences.get(self.current_sequence_name, [])):
            self.seq_waypoint_list.addItem(QListWidgetItem(self._format_waypoint(i, wp)))

    @staticmethod
    def _format_waypoint(index, wp):
        wp_type = wp.get('type', 'joint')
        hold = wp.get('hold_time', 1.0)
        if wp_type == 'joint':
            values = wp.get('values', [])
            if len(values) > 4:
                vals_str = f"[{values[0]:.1f}°, {values[1]:.1f}°, ... {values[-1]:.1f}°]"
            else:
                vals_str = "[" + ", ".join(f"{v:.1f}°" for v in values) + "]"
            return f"{index+1}. JOINT {vals_str}  ({hold:.1f}s)"
        if wp_type == 'cartesian':
            pos = wp.get('position', [0, 0, 0])
            return f"{index+1}. CART ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})  ({hold:.1f}s)"
        if wp_type == 'gripper':
            width = wp.get('width', 0.08)
            return f"{index+1}. GRIPPER {width:.4f}m ({(width / 0.08) * 100:.0f}%)  ({hold:.1f}s)"
        return f"{index+1}. UNKNOWN"

    def _on_capture_mode_changed(self, mode):
        self.seq_joint_widget.setVisible(mode == "Joint")
        self.seq_cart_widget.setVisible(mode == "Cartesian")
        self.seq_gripper_widget.setVisible(mode == "Gripper")

    def _get_target_robot_id(self):
        target = self.seq_target.currentText()
        if target == "Simulation":
            return "simulation"
        return target.lower().replace(" ", "_")

    def _target_topic_rid(self):
        """Topic-prefix rid ('robotN') of the seq_target, or None if it's set
        to 'Simulation'. This is what publish callsites should use to address
        a specific robot's per-robot publishers."""
        wid = self._get_target_robot_id()
        if wid == "simulation":
            return None
        if wid.startswith("robot_") and wid[len("robot_"):].isdigit():
            return f"robot{wid[len('robot_'):]}"
        return wid

    def _read_robot_position(self):
        mode = self.seq_capture_mode.currentText()
        target = self.seq_target.currentText()
        robot_id = self._get_target_robot_id()
        joints = self.host.current_joint_positions
        ee = self.host.current_ee_pose

        if mode == "Joint":
            if joints:
                num_joints = len(joints)
                for i in range(min(num_joints, len(self.seq_joint_inputs))):
                    self.seq_joint_inputs[i].setValue(math.degrees(joints[i]))
                self.host.log("INFO", f"Read {num_joints} joint positions from robot feedback", "Sequence")
            else:
                # Fallback to MainWindow sliders (used when no FIC controller is publishing)
                mw = self.host.main_window
                sliders = (getattr(mw, 'joint_sliders', []) if target == "Simulation"
                           else mw.real_joint_sliders.get(robot_id, []))
                if sliders:
                    for i, sld in enumerate(sliders):
                        if i < len(self.seq_joint_inputs):
                            self.seq_joint_inputs[i].setValue(math.degrees(sld.value() / 100.0))
                    self.host.log("INFO", "Read joint positions from sliders (no feedback)", "Sequence")
                else:
                    QMessageBox.warning(self._page, "No Data",
                        "No joint feedback received.\n\n"
                        "Make sure FIC controller is running and publishing q_current / joint_states / trajectory_data.")
        elif mode == "Cartesian":
            if ee is not None:
                self.seq_cart_x.setValue(ee.position.x)
                self.seq_cart_y.setValue(ee.position.y)
                self.seq_cart_z.setValue(ee.position.z)
                qx, qy, qz, qw = ee.orientation.x, ee.orientation.y, ee.orientation.z, ee.orientation.w
                angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
                s = math.sqrt(1.0 - qw * qw)
                if s < 0.001:
                    ax, ay, az = 1.0, 0.0, 0.0
                else:
                    ax, ay, az = qx / s, qy / s, qz / s
                self.seq_cart_ax.setValue(ax); self.seq_cart_ay.setValue(ay); self.seq_cart_az.setValue(az)
                self.seq_cart_angle.setValue(math.degrees(angle))
                self.host.log("INFO", "Read Cartesian pose from robot feedback", "Sequence")
            else:
                QMessageBox.warning(self._page, "No Data",
                    "No Cartesian feedback received.\n\n"
                    "Make sure FIC controller is running and publishing ee_current_pose.")
        elif mode == "Gripper":
            if target == "Simulation":
                QMessageBox.information(self._page, "Simulation",
                    "Gripper not available in simulation. Set value manually.")
            else:
                mw = self.host.main_window
                gripper_slider = getattr(mw, f'{robot_id}_gripper_slider', None)
                if gripper_slider:
                    width = (gripper_slider.value() / 100.0) * 0.08
                    self.seq_gripper_width.setValue(width)
                    self.host.log("INFO", f"Read gripper width from {robot_id}: {width:.4f}m", "Sequence")
                else:
                    QMessageBox.warning(self._page, "No Data", f"No gripper slider found for {robot_id}.")

    def _capture_position(self):
        if not self.current_sequence_name:
            QMessageBox.warning(self._page, "No Sequence", "Please create or select a sequence first.")
            return
        mode = self.seq_capture_mode.currentText()
        hold_time = self.seq_hold_time.value()
        if mode == "Joint":
            num_joints = self.host.robots.get(self.host.current_robot_name(), 7)
            values = [self.seq_joint_inputs[i].value() for i in range(min(num_joints, len(self.seq_joint_inputs)))]
            wp = {'type': 'joint', 'values': values, 'hold_time': hold_time}
        elif mode == "Cartesian":
            wp = {'type': 'cartesian',
                  'position': [self.seq_cart_x.value(), self.seq_cart_y.value(), self.seq_cart_z.value()],
                  'axis_angle': [self.seq_cart_ax.value(), self.seq_cart_ay.value(),
                                 self.seq_cart_az.value(), self.seq_cart_angle.value()],
                  'hold_time': hold_time}
        elif mode == "Gripper":
            wp = {'type': 'gripper', 'width': self.seq_gripper_width.value(), 'hold_time': hold_time}
        else:
            return
        self.sequences[self.current_sequence_name].append(wp)
        self._refresh_waypoint_list()
        self.host.log("INFO", f"Captured {mode} waypoint to '{self.current_sequence_name}'", "Sequence")

    def _on_waypoint_selected(self, item):
        index = self.seq_waypoint_list.row(item)
        wps = self.sequences.get(self.current_sequence_name, [])
        if not (0 <= index < len(wps)):
            return
        wp = wps[index]
        wp_type = wp.get('type', 'joint')
        if wp_type == 'joint':
            self.seq_capture_mode.setCurrentText("Joint")
        elif wp_type == 'cartesian':
            self.seq_capture_mode.setCurrentText("Cartesian")
        elif wp_type == 'gripper':
            self.seq_capture_mode.setCurrentText("Gripper")
        self.seq_hold_time.setValue(wp.get('hold_time', 1.0))
        if wp_type == 'joint':
            for i, val in enumerate(wp.get('values', [])):
                if i < len(self.seq_joint_inputs):
                    self.seq_joint_inputs[i].setValue(val)
        elif wp_type == 'cartesian':
            pos = wp.get('position', [0, 0, 0])
            aa = wp.get('axis_angle', [0, 0, 1, 0])
            self.seq_cart_x.setValue(pos[0]); self.seq_cart_y.setValue(pos[1]); self.seq_cart_z.setValue(pos[2])
            self.seq_cart_ax.setValue(aa[0]); self.seq_cart_ay.setValue(aa[1]); self.seq_cart_az.setValue(aa[2])
            self.seq_cart_angle.setValue(aa[3])
        elif wp_type == 'gripper':
            self.seq_gripper_width.setValue(wp.get('width', 0.08))

    def _update_waypoint(self):
        selected = self.seq_waypoint_list.currentRow()
        wps = self.sequences.get(self.current_sequence_name, [])
        if selected < 0 or selected >= len(wps):
            QMessageBox.warning(self._page, "No Selection", "Please select a waypoint to update.")
            return
        mode = self.seq_capture_mode.currentText()
        hold_time = self.seq_hold_time.value()
        if mode == "Joint":
            num_joints = self.host.robots.get(self.host.current_robot_name(), 7)
            values = [self.seq_joint_inputs[i].value() for i in range(min(num_joints, len(self.seq_joint_inputs)))]
            wps[selected] = {'type': 'joint', 'values': values, 'hold_time': hold_time}
        elif mode == "Cartesian":
            wps[selected] = {'type': 'cartesian',
                             'position': [self.seq_cart_x.value(), self.seq_cart_y.value(), self.seq_cart_z.value()],
                             'axis_angle': [self.seq_cart_ax.value(), self.seq_cart_ay.value(),
                                            self.seq_cart_az.value(), self.seq_cart_angle.value()],
                             'hold_time': hold_time}
        elif mode == "Gripper":
            wps[selected] = {'type': 'gripper', 'width': self.seq_gripper_width.value(), 'hold_time': hold_time}
        self._refresh_waypoint_list()
        self.seq_waypoint_list.setCurrentRow(selected)

    def _delete_waypoint(self):
        selected = self.seq_waypoint_list.currentRow()
        wps = self.sequences.get(self.current_sequence_name, [])
        if selected < 0 or selected >= len(wps):
            QMessageBox.warning(self._page, "No Selection", "Please select a waypoint to delete.")
            return
        if QMessageBox.question(self._page, "Delete Waypoint", f"Delete waypoint {selected + 1}?",
                                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            wps.pop(selected)
            self._refresh_waypoint_list()

    def _move_waypoint_up(self):
        selected = self.seq_waypoint_list.currentRow()
        wps = self.sequences.get(self.current_sequence_name, [])
        if 0 < selected < len(wps):
            wps[selected], wps[selected - 1] = wps[selected - 1], wps[selected]
            self._refresh_waypoint_list()
            self.seq_waypoint_list.setCurrentRow(selected - 1)

    def _move_waypoint_down(self):
        selected = self.seq_waypoint_list.currentRow()
        wps = self.sequences.get(self.current_sequence_name, [])
        if 0 <= selected < len(wps) - 1:
            wps[selected], wps[selected + 1] = wps[selected + 1], wps[selected]
            self._refresh_waypoint_list()
            self.seq_waypoint_list.setCurrentRow(selected + 1)

    def _goto_waypoint(self, item=None):
        selected = self.seq_waypoint_list.currentRow()
        wps = self.sequences.get(self.current_sequence_name, [])
        if selected < 0 or selected >= len(wps):
            QMessageBox.warning(self._page, "No Selection", "Please select a waypoint.")
            return
        self._execute_waypoint(wps[selected])
        self.seq_status_label.setText(f"Moving to waypoint {selected + 1}")

    #: The mode a waypoint is sent in when the robot is in the other kind of mode (simulation).
    WAYPOINT_MODE = {'joint': 'Joint Impedance With Planner', 'cartesian': 'Cartesian Planner'}
    MODE_SWITCH_MS = 400

    def _mode_for(self, wp_type):
        """In simulation, switch to a mode that follows ``wp_type`` waypoints when the robot is in
        the other kind (a joint waypoint in a Cartesian mode does nothing). Returns True if it
        switched. A real robot is never switched: the waypoint is sent as it is."""
        want = self.WAYPOINT_MODE.get(wp_type)
        if want is None or self.target_is_hardware() or not hasattr(self.host, 'control_mode'):
            return False
        mode = self.host.control_mode()
        if not mode or mode.startswith('Cartesian') == (wp_type == 'cartesian'):
            return False
        self.host.set_mode(want)
        self.host.log('INFO', 'Switched to %s for a %s waypoint' % (want, wp_type), 'Sequence')
        return True

    def _execute_waypoint(self, wp):
        """Send one waypoint, first switching to a mode that follows it (simulation); the target is
        sent MODE_SWITCH_MS later then, since a mode switch snaps the target to the current pose."""
        if self._mode_for(wp.get('type', 'joint')):
            QTimer.singleShot(self.MODE_SWITCH_MS, lambda w=dict(wp): self._send_waypoint(w))
            return
        self._send_waypoint(wp)

    def _send_waypoint(self, wp):
        wp_type = wp.get('type', 'joint')
        target = self.seq_target.currentText()
        robot_id = self._get_target_robot_id()
        node = self.host.node
        if wp_type == 'joint':
            values = wp.get('values', [])
            radians = [math.radians(v) for v in values]
            if target == "Simulation":
                robot_name = self.host.current_robot_name()
                names = [f"{robot_name}_j{i+1}" for i in range(len(radians))]
                node.publish_joint_angles(radians, names=names)
                deg_str = ", ".join([f"{v:.1f}°" for v in values])
                self.host.log("INFO", f"Joint WP: [{deg_str}]", "Sequence")
            else:
                from std_msgs.msg import Float32MultiArray
                rid = self._target_topic_rid() or node.station_robot_id
                pub = node.real_joint_publishers.get(rid)
                if pub is None:
                    self.host.log("ERROR", f"No joint publisher for {rid}", robot_id)
                    return
                msg = Float32MultiArray()
                msg.data = [float(r) for r in radians]
                pub.publish(msg)
                deg_str = ", ".join([f"{v:.1f}°" for v in values])
                self.host.log("INFO", f"Joint WP: [{deg_str}]", robot_id)
        elif wp_type == 'cartesian':
            from geometry_msgs.msg import Pose, Point, Quaternion
            from fic_studio.math_helpers import _axis_angle_to_quat
            pos = wp.get('position', [0, 0, 0])
            aa = wp.get('axis_angle', [0, 0, 1, 0])
            qx, qy, qz, qw = _axis_angle_to_quat(aa[0], aa[1], aa[2], aa[3], "deg")
            pose = Pose(position=Point(x=pos[0], y=pos[1], z=pos[2]),
                        orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
            if self._publish_pose(pose):
                self.host.log("INFO", f"Cart WP: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})",
                              "Sequence" if target == "Simulation" else robot_id)
        elif wp_type == 'gripper':
            width = wp.get('width', 0.08)
            pct = (width / 0.08) * 100
            if target == "Simulation":
                # the simulator's gripper follows {rid}_gripper_width through the gripper bridge
                node.publish_gripper_width(node.station_robot_id, float(width))
                self.host.log("INFO", f"Gripper WP: {width:.4f}m ({pct:.0f}%) [sim]", "Sequence")
            else:
                from std_msgs.msg import Float32
                rid = self._target_topic_rid() or node.station_robot_id
                pub = node.gripper_publishers.get(rid)
                if pub:
                    msg = Float32()
                    msg.data = float(width)
                    pub.publish(msg)
                self.host.log("INFO", f"Gripper WP: {width:.4f}m ({pct:.0f}%)", robot_id)

    def _play_sequence(self):
        wps = self.sequences.get(self.current_sequence_name, [])
        if not wps:
            QMessageBox.warning(self._page, "Empty Sequence", "This sequence has no waypoints.")
            return
        self.is_playing = True
        self.play_index = 0
        self.target_waypoint = None
        self.wait_start_time = None
        self.hold_start_time = None
        self.seq_play_btn.setEnabled(False)
        self.seq_stop_btn.setEnabled(True)
        self.seq_status_label.setText("Playing...")
        self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('playing'))
        self.host.log("INFO",
            f"▶ Started '{self.current_sequence_name}' ({len(wps)} WPs) → {self.seq_target.currentText()}",
            "Sequence")
        self._play_next_waypoint()

    def _play_next_waypoint(self):
        if not self.is_playing:
            return
        wps = self.sequences.get(self.current_sequence_name, [])
        if self.play_index >= len(wps):
            if self.seq_loop_checkbox.isChecked():
                self.play_index = 0
                self.host.log("INFO", "↻ Looping sequence...", "Sequence")
            else:
                self._stop_sequence()
                self.seq_status_label.setText("Sequence Complete")
                self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('ready'))
                self.host.log("INFO", f"✓ Sequence '{self.current_sequence_name}' complete", "Sequence")
                return
        wp = wps[self.play_index]
        self.target_waypoint = wp
        self.wait_start_time = None
        self.hold_start_time = None
        self._execute_waypoint(wp)
        self.seq_waypoint_list.setCurrentRow(self.play_index)
        self.seq_status_label.setText(f"Moving to waypoint {self.play_index + 1}/{len(wps)}...")
        self.move_start_time = time.time()
        self.play_timer.start(50)

    def _check_position_reached(self):
        if not self.is_playing or not self.target_waypoint:
            self.play_timer.stop()
            return
        wp = self.target_waypoint
        wp_type = wp.get('type', 'joint')
        wps = self.sequences.get(self.current_sequence_name, [])
        elapsed = time.time() - self.move_start_time
        # a waypoint may carry its own timeout_s and tolerance_mm (e.g. a grasp that stops on the object)
        if elapsed > float(wp.get('timeout_s', self.timeout_seconds)):
            self.host.log("WARN", f"Timeout at WP {self.play_index + 1} ({elapsed:.1f}s)", "Sequence")
            self.seq_status_label.setText(f"Timeout at waypoint {self.play_index + 1} - continuing...")
            self._proceed_to_next_waypoint()
            return
        reached = False
        if wp_type == 'joint':
            reached = self._check_joint_position_reached(wp)
        elif wp_type == 'cartesian':
            reached = self._check_cartesian_position_reached(wp)
        elif wp_type == 'gripper':
            reached = (elapsed > 0.3)
        if reached:
            if self.hold_start_time is None:
                self.hold_start_time = time.time()
                hold_time = wp.get('hold_time', 1.0)
                self.seq_status_label.setText(
                    f"Waypoint {self.play_index + 1}/{len(wps)} reached - holding {hold_time:.1f}s")
                self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('ready'))
            adjusted_hold = wp.get('hold_time', 1.0) / self.seq_speed_scale.value()
            if (time.time() - self.hold_start_time) >= adjusted_hold:
                self._proceed_to_next_waypoint()
        else:
            self.seq_status_label.setText(
                f"Moving to waypoint {self.play_index + 1}/{len(wps)}... ({elapsed:.1f}s)")
            self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('playing'))

    def _check_joint_position_reached(self, wp):
        target_values = wp.get('values', [])
        joints = self.host.current_joint_positions
        if not target_values or not joints:
            return False
        target_rad = [math.radians(v) for v in target_values]
        for target, current in zip(target_rad, joints):
            if abs(target - current) > self.joint_tolerance_rad:
                return False
        return True

    def _check_cartesian_position_reached(self, wp):
        ee = self.host.current_ee_pose
        if ee is None:
            return False
        target_pos = wp.get('position', [0, 0, 0])
        current_pos = [ee.position.x, ee.position.y, ee.position.z]
        pos_error = math.sqrt(sum((t - c) ** 2 for t, c in zip(target_pos, current_pos)))
        tol = float(wp['tolerance_mm']) / 1000.0 if wp.get('tolerance_mm') else self.position_tolerance_m
        if pos_error <= tol:
            self._cart_settle_last_pos = None
            self._cart_settle_since = None
            return True
        near_band = tol * 3.0
        stall_step_m = 0.001
        stall_window_s = 1.0
        now = time.time()
        if pos_error <= near_band:
            if self._cart_settle_last_pos is None:
                self._cart_settle_last_pos = current_pos
                self._cart_settle_since = now
            else:
                step = math.sqrt(sum((a - b) ** 2 for a, b in zip(current_pos, self._cart_settle_last_pos)))
                if step > stall_step_m:
                    self._cart_settle_last_pos = current_pos
                    self._cart_settle_since = now
                elif self._cart_settle_since and (now - self._cart_settle_since) >= stall_window_s:
                    self.host.log("INFO",
                        f"Cartesian waypoint settled at {pos_error*1000:.1f}mm "
                        f"(beyond {tol*1000:.0f}mm tolerance) — advancing",
                        "Sequence")
                    self._cart_settle_last_pos = None
                    self._cart_settle_since = None
                    return True
        else:
            self._cart_settle_last_pos = None
            self._cart_settle_since = None
        return False

    def _proceed_to_next_waypoint(self):
        self.play_timer.stop()
        elapsed = time.time() - self.move_start_time
        self.host.log("INFO", f"✓ WP {self.play_index + 1} reached ({elapsed:.1f}s)", "Sequence")
        self.play_index += 1
        self.target_waypoint = None
        self.hold_start_time = None
        self._cart_settle_last_pos = None
        self._cart_settle_since = None
        QTimer.singleShot(100, self._play_next_waypoint)

    def _stop_sequence(self):
        was_playing = self.is_playing
        self.is_playing = False
        self.play_timer.stop()
        self.play_index = 0
        self.target_waypoint = None
        self.hold_start_time = None
        self._cart_settle_last_pos = None
        self._cart_settle_since = None
        if hasattr(self, 'seq_play_btn'):
            self.seq_play_btn.setEnabled(True)
            self.seq_stop_btn.setEnabled(False)
            self.seq_status_label.setText("Stopped")
            self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('stopped'))
        if was_playing:
            self.host.log("WARN", f"■ Sequence '{self.current_sequence_name}' stopped", "Sequence")

    def _step_sequence(self):
        wps = self.sequences.get(self.current_sequence_name, [])
        selected = self.seq_waypoint_list.currentRow()
        next_idx = selected + 1 if selected >= 0 else 0
        if next_idx < len(wps):
            self.seq_waypoint_list.setCurrentRow(next_idx)
            self._goto_waypoint()

    def _create_new_sequence(self):
        name, ok = QInputDialog.getText(self._page, "New Sequence", "Sequence name:")
        if ok and name:
            if name in self.sequences:
                QMessageBox.warning(self._page, "Exists", f"Sequence '{name}' already exists.")
                return
            self.sequences[name] = []
            self._refresh_sequence_dropdown()
            self.sequence_dropdown.setCurrentText(name)

    def _rename_sequence(self):
        if not self.current_sequence_name:
            return
        new_name, ok = QInputDialog.getText(self._page, "Rename Sequence", "New name:",
                                            text=self.current_sequence_name)
        if ok and new_name and new_name != self.current_sequence_name:
            if new_name in self.sequences:
                QMessageBox.warning(self._page, "Exists", f"Sequence '{new_name}' already exists.")
                return
            self.sequences[new_name] = self.sequences.pop(self.current_sequence_name)
            self._refresh_sequence_dropdown()
            self.sequence_dropdown.setCurrentText(new_name)

    def _delete_sequence(self):
        if not self.current_sequence_name:
            return
        if len(self.sequences) <= 1:
            QMessageBox.warning(self._page, "Cannot Delete", "Cannot delete the last sequence.")
            return
        if QMessageBox.question(self._page, "Delete Sequence",
                                f"Delete sequence '{self.current_sequence_name}'?",
                                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            del self.sequences[self.current_sequence_name]
            self._refresh_sequence_dropdown()

    def _write_yaml(self):
        """Write every sequence to SEQUENCES_YAML_PATH (no dialogs)."""
        data = {'sequences': [{'name': n, 'waypoints': wps} for n, wps in self.sequences.items()]}
        with open(SEQUENCES_YAML_PATH, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.host.log("INFO", f"Saved sequences to {SEQUENCES_YAML_PATH}", "Sequence")

    def _save_sequences_to_yaml(self):
        try:
            self._write_yaml()
            QMessageBox.information(self._page, "Saved", f"Sequences saved to:\n{SEQUENCES_YAML_PATH}")
        except Exception as e:
            QMessageBox.critical(self._page, "Save Error", f"Could not save sequences:\n{e}")

    def _load_sequences_from_yaml(self, silent=False):
        try:
            with open(SEQUENCES_YAML_PATH, 'r') as f:
                data = yaml.safe_load(f)
            if not data or 'sequences' not in data:
                if not silent:
                    QMessageBox.warning(self._page, "Invalid File", "No sequences found in file.")
                return
            self.sequences = {}
            for seq_data in data['sequences']:
                self.sequences[seq_data.get('name', 'Unnamed')] = seq_data.get('waypoints', [])
            self._refresh_sequence_dropdown()
            if not silent:
                QMessageBox.information(self._page, "Loaded",
                    f"Loaded {len(self.sequences)} sequence(s) from:\n{SEQUENCES_YAML_PATH}")
            self.host.log("INFO",
                f"Loaded {len(self.sequences)} sequences from {SEQUENCES_YAML_PATH}", "Sequence")
        except FileNotFoundError:
            if not silent:
                QMessageBox.warning(self._page, "Not Found", f"File not found:\n{SEQUENCES_YAML_PATH}")
        except Exception as e:
            if not silent:
                QMessageBox.critical(self._page, "Load Error", f"Could not load sequences:\n{e}")

    def _set_teach_mode(self):
        target = self.seq_target.currentText()
        if target == "Simulation":
            QMessageBox.information(self._page, "Simulation",
                "Teaching mode is for real robots only.\n"
                "In simulation, use the joint sliders to set positions.")
            return
        robot_id = self._get_target_robot_id()
        capture_mode = self.seq_capture_mode.currentText()
        mode_name = "Cartesian Admittance" if capture_mode == "Cartesian" else "Joint Admittance"
        from std_msgs.msg import String
        pub = getattr(self.host.node, f'{robot_id}_mode_publisher', None)
        if pub:
            msg = String(); msg.data = mode_name
            pub.publish(msg)
            self.seq_status_label.setText(f"{target}: TEACH MODE ({mode_name})")
            self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('teach'))
            self.host.log("INFO", f"Set {robot_id} to {mode_name} (teach mode)", "Sequence")
        else:
            QMessageBox.warning(self._page, "No Publisher", f"Mode publisher not found for {robot_id}")

    def _set_playback_mode(self):
        target = self.seq_target.currentText()
        if target == "Simulation":
            QMessageBox.information(self._page, "Simulation",
                "Playback mode is for real robots only.\n"
                "Simulation will receive commands directly.")
            return
        robot_id = self._get_target_robot_id()
        capture_mode = self.seq_capture_mode.currentText()
        mode_name = "Cartesian Planner" if capture_mode == "Cartesian" else "Joint Impedance With Planner"
        from std_msgs.msg import String
        pub = getattr(self.host.node, f'{robot_id}_mode_publisher', None)
        if pub:
            msg = String(); msg.data = mode_name
            pub.publish(msg)
            self.seq_status_label.setText(f"{target}: PLAYBACK MODE ({mode_name})")
            self.seq_status_label.setStyleSheet('color:%s; font-weight:bold;' % status_color('playing'))
            self.host.log("INFO", f"Set {robot_id} to {mode_name} (playback mode)", "Sequence")
        else:
            QMessageBox.warning(self._page, "No Publisher", f"Mode publisher not found for {robot_id}")
