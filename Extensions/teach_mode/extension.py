"""Teach Mode extension — guide the robot by hand and record waypoints.

Migrated from fic_studio.mixins.teach_mixin to the extension contract.
Joint state subscribers extracted to JointStateMixin; reads go through
host.current_joint_positions / host.current_ee_pose. Waypoints can be
saved into the Sequences extension (cross-extension lookup via host).
"""
import math
import time
from datetime import datetime
from functools import partial

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox,
    QPushButton, QScrollArea, QSplitter, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from fic_studio import theme
from fic_studio.extensions import Extension as ExtensionBase
from Extensions._shared.theme_helpers import set_toggle_button_state, status_color


class _TeachSignals(QObject):
    waypoint_recorded = pyqtSignal(dict)
    position_updated = pyqtSignal(list, list)


class Extension(ExtensionBase):
    ID = "teach_mode"
    NAME = "Teach Mode"
    DESCRIPTION = (
        "Put the robot in zero-torque admittance, guide it by hand, and record "
        "waypoints (joint, Cartesian, gripper) into the sequence library."
    )
    SIDEBAR_ICON = "✎"
    SIDEBAR_SECTION = "Programming"

    def __init__(self, host):
        super().__init__(host)
        self.teach_signals = _TeachSignals()
        self.teach_signals.waypoint_recorded.connect(self._on_waypoint_recorded)
        self.teach_signals.position_updated.connect(self._update_position_display)

        self.teach_mode_active = False
        self.recorded_waypoints = []
        self.teach_start_time = None
        self.teach_target_mode = "Joint"

        self.current_cartesian_pose = []
        self.current_ee_quat = (0.0, 0.0, 0.0, 1.0)

        self.auto_record_enabled = False
        self.auto_record_interval = 1.0
        self.movement_threshold_mm = 5.0
        self.last_auto_record_time = 0
        self.gripper_width = 0.08
        self.sequence_name = f"taught_{datetime.now().strftime('%H%M%S')}"

        self.teach_position_timer = QTimer()
        self.teach_position_timer.timeout.connect(self._poll_robot_position)
        self._page = None

    # ── Lifecycle ─────────────────────────────────────────────────────
    def teardown(self) -> None:
        if self.teach_mode_active:
            self._stop_teach_mode()

    def on_emergency_stop(self) -> None:
        if self.teach_mode_active:
            self.host.log("WARN", "Emergency stop — exiting teach mode", "Teach")
            self._stop_teach_mode()

    @property
    def state_active(self) -> bool:
        return self.teach_mode_active

    # ── Build ─────────────────────────────────────────────────────────
    def build_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(12)

        title_row = QHBoxLayout()
        title_row.addWidget(self.host.make_page_title("Teach Mode"))
        title_row.addStretch()
        title_row.addWidget(QLabel("Target Robot:"))
        title_row.addWidget(self.host.make_target_robot_combo())
        title_row.addSpacing(12)
        title_row.addWidget(QLabel("Mode:"))
        self.teach_mode_selector = QComboBox()
        self.teach_mode_selector.addItems(["Joint", "Cartesian"])
        self.teach_mode_selector.setCurrentText("Joint")
        self.teach_mode_selector.setMinimumWidth(110)
        self.teach_mode_selector.setToolTip(
            "Joint: hand-guide via Joint Admittance, save joint-angle waypoints.\n"
            "Cartesian: hand-guide via Cartesian Admittance, save EE-pose waypoints.")
        self.teach_mode_selector.currentTextChanged.connect(self._on_teach_mode_changed)
        title_row.addWidget(self.teach_mode_selector)

        self.teach_toggle_btn = self.host.make_button("▶ START TEACHING", "BtnTeal")
        self.teach_toggle_btn.setMinimumWidth(150)
        self.teach_toggle_btn.setMinimumHeight(32)
        self.teach_toggle_btn.clicked.connect(self._toggle_teach_mode)
        title_row.addWidget(self.teach_toggle_btn)
        layout.addLayout(title_row)

        # Status banner
        self.teach_status_frame = QFrame()
        theme.bind(
            self.teach_status_frame,
            lambda t: (
                'QFrame { background:%s; border:1px solid %s;'
                ' border-radius:6px; padding:10px; }'
            ) % (t['bg_raised'], t['border_subtle'])
        )
        status_layout = QHBoxLayout(self.teach_status_frame)
        status_layout.setContentsMargins(10, 5, 10, 5)
        self.teach_status_icon = QLabel('○')
        self.teach_status_icon.setStyleSheet('font-size: 18px; color: %s;' % status_color('idle'))
        status_layout.addWidget(self.teach_status_icon)
        self.teach_status_label = QLabel("Teach mode inactive. Click 'Start Teaching' to begin.")
        self.teach_status_label.setObjectName('HintLabel')
        status_layout.addWidget(self.teach_status_label, 1)
        self.waypoint_count_label = QLabel('Waypoints: 0')
        theme.bind(
            self.waypoint_count_label,
            lambda t: 'color:%s; font-weight:bold;' % t['accent_blue']
        )
        status_layout.addWidget(self.waypoint_count_label)
        layout.addWidget(self.teach_status_frame)

        splitter = QSplitter(Qt.Horizontal)

        # LEFT — current position
        position_widget = QWidget()
        position_widget.setMinimumWidth(320)
        position_layout = QVBoxLayout(position_widget)
        position_layout.setContentsMargins(0, 0, 8, 0)
        position_layout.setSpacing(12)

        joint_grp = QGroupBox("CURRENT JOINT POSITIONS")
        joint_layout = QGridLayout(joint_grp)
        joint_layout.setContentsMargins(12, 16, 12, 12)
        joint_layout.setSpacing(6)
        self.teach_joint_labels = []
        for i in range(7):
            name_label = QLabel(f'Joint {i+1}:')
            name_label.setObjectName('HintLabel')
            value_label = QLabel('---')
            theme.bind(
                value_label,
                lambda t: 'font-family:monospace; color:%s; font-weight:bold;' % t['accent_blue']
            )
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            joint_layout.addWidget(name_label, i, 0)
            joint_layout.addWidget(value_label, i, 1)
            self.teach_joint_labels.append((name_label, value_label))
        position_layout.addWidget(joint_grp)

        cart_grp = QGroupBox("CURRENT END-EFFECTOR POSE")
        cart_layout = QGridLayout(cart_grp)
        cart_layout.setContentsMargins(12, 16, 12, 12)
        cart_layout.setSpacing(6)
        self.teach_cart_labels = {}
        for i, (name, unit) in enumerate([('X', 'mm'), ('Y', 'mm'), ('Z', 'mm'),
                                          ('Roll', '°'), ('Pitch', '°'), ('Yaw', '°')]):
            name_lbl = QLabel(f'{name}:')
            name_lbl.setObjectName('HintLabel')
            cart_layout.addWidget(name_lbl, i, 0)
            value_label = QLabel('---')
            theme.bind(
                value_label,
                lambda t: 'font-family:monospace; color:%s; font-weight:bold;' % t['accent_teal']
            )
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            cart_layout.addWidget(value_label, i, 1)
            unit_lbl = QLabel(unit)
            unit_lbl.setObjectName('HintLabel')
            cart_layout.addWidget(unit_lbl, i, 2)
            self.teach_cart_labels[name] = value_label
        position_layout.addWidget(cart_grp)

        record_grp = QGroupBox("RECORD WAYPOINT")
        record_layout = QVBoxLayout(record_grp)
        record_layout.setContentsMargins(12, 16, 12, 12)
        record_layout.setSpacing(10)
        self.record_waypoint_btn = self.host.make_button("● RECORD CURRENT POSITION", "BtnPrimary")
        self.record_waypoint_btn.setMinimumHeight(35)
        self.record_waypoint_btn.setEnabled(False)
        self.record_waypoint_btn.clicked.connect(self._record_waypoint_clicked)
        record_layout.addWidget(self.record_waypoint_btn)

        auto_row = QHBoxLayout()
        self.auto_record_check = QCheckBox("Auto-record every")
        self.auto_record_check.toggled.connect(self._toggle_auto_record)
        auto_row.addWidget(self.auto_record_check)
        self.auto_record_spin = QDoubleSpinBox()
        self.auto_record_spin.setRange(0.5, 10.0); self.auto_record_spin.setValue(1.0)
        self.auto_record_spin.setSuffix(" sec"); self.auto_record_spin.setDecimals(1); self.auto_record_spin.setMinimumWidth(75)
        self.auto_record_spin.setValue(self.auto_record_interval)
        self.auto_record_spin.valueChanged.connect(lambda v: setattr(self, 'auto_record_interval', float(v)))
        auto_row.addWidget(self.auto_record_spin)
        auto_row.addStretch()
        record_layout.addLayout(auto_row)

        thresh_row = QHBoxLayout()
        thresh_row.addWidget(QLabel("Movement threshold:"))
        self.movement_threshold_spin = QDoubleSpinBox()
        self.movement_threshold_spin.setRange(0.1, 50.0); self.movement_threshold_spin.setValue(5.0)
        self.movement_threshold_spin.setSuffix(" mm"); self.movement_threshold_spin.setDecimals(1); self.movement_threshold_spin.setMinimumWidth(75)
        self.movement_threshold_spin.setValue(self.movement_threshold_mm)
        self.movement_threshold_spin.valueChanged.connect(lambda v: setattr(self, 'movement_threshold_mm', float(v)))
        thresh_row.addWidget(self.movement_threshold_spin)
        thresh_row.addStretch()
        record_layout.addLayout(thresh_row)
        position_layout.addWidget(record_grp)

        # Gripper
        gripper_grp = QGroupBox("GRIPPER")
        gripper_layout = QVBoxLayout(gripper_grp)
        gripper_layout.setContentsMargins(12, 16, 12, 12)
        gripper_layout.setSpacing(8)
        width_row = QHBoxLayout()
        width_row.addWidget(QLabel("Width:"))
        self.teach_gripper_width = QDoubleSpinBox()
        self.teach_gripper_width.setRange(0.0, 0.08); self.teach_gripper_width.setDecimals(4)
        self.teach_gripper_width.setSingleStep(0.005); self.teach_gripper_width.setSuffix(" m"); self.teach_gripper_width.setValue(0.08)
        width_row.addWidget(self.teach_gripper_width)
        self.teach_gripper_pct = QLabel('100%')
        theme.bind(
            self.teach_gripper_pct,
            lambda t: 'font-family:monospace; color:%s; font-weight:bold;' % t['accent_teal']
        )
        self.teach_gripper_width.setValue(self.gripper_width)
        self.teach_gripper_width.valueChanged.connect(
            lambda v: self.teach_gripper_pct.setText(f"{(v / 0.08) * 100:.0f}%"))
        self.teach_gripper_width.valueChanged.connect(lambda v: setattr(self, 'gripper_width', float(v)))
        width_row.addWidget(self.teach_gripper_pct)
        width_row.addStretch()
        gripper_layout.addLayout(width_row)

        actuate_row = QHBoxLayout()
        for label, val in [("OPEN", 0.08), ("CLOSE", 0.0)]:
            btn = self.host.make_button(label)
            btn.setToolTip(f"Set gripper to {val} m and send to robot")
            btn.clicked.connect(partial(self._set_and_publish_gripper, val))
            actuate_row.addWidget(btn)
        apply_btn = self.host.make_button("APPLY", "BtnPrimary")
        apply_btn.setToolTip("Send the spinbox width to the robot now (does not record)")
        apply_btn.clicked.connect(self._apply_gripper_width)
        actuate_row.addWidget(apply_btn)
        actuate_row.addStretch()
        gripper_layout.addLayout(actuate_row)

        self.record_gripper_btn = self.host.make_button("● RECORD GRIPPER WAYPOINT", "BtnTeal")
        self.record_gripper_btn.setMinimumHeight(32)
        self.record_gripper_btn.setEnabled(False)
        self.record_gripper_btn.setToolTip(
            "Drop a gripper-only waypoint into the sequence at the current width.")
        self.record_gripper_btn.clicked.connect(self._record_gripper_waypoint)
        gripper_layout.addWidget(self.record_gripper_btn)
        position_layout.addWidget(gripper_grp)
        position_layout.addStretch(1)
        splitter.addWidget(position_widget)

        # RIGHT — recorded waypoints
        waypoints_widget = QWidget()
        waypoints_widget.setMinimumWidth(400)
        waypoints_layout = QVBoxLayout(waypoints_widget)
        waypoints_layout.setContentsMargins(8, 0, 0, 0)
        waypoints_layout.setSpacing(12)

        wp_grp = QGroupBox("RECORDED WAYPOINTS")
        wp_layout = QVBoxLayout(wp_grp)
        wp_layout.setContentsMargins(12, 16, 12, 12)
        self.waypoints_table = QTableWidget()
        self.waypoints_table.setColumnCount(4)
        self.waypoints_table.setHorizontalHeaderLabels(["#", "Time", "Position (mm)", "Actions"])
        for col, mode in enumerate([QHeaderView.Fixed, QHeaderView.Fixed,
                                    QHeaderView.Stretch, QHeaderView.Fixed]):
            self.waypoints_table.horizontalHeader().setSectionResizeMode(col, mode)
        self.waypoints_table.setColumnWidth(0, 40); self.waypoints_table.setColumnWidth(1, 70); self.waypoints_table.setColumnWidth(3, 100)
        wp_layout.addWidget(self.waypoints_table)

        wp_actions = QHBoxLayout()
        clear_wp_btn = self.host.make_button("Clear All")
        clear_wp_btn.clicked.connect(self._clear_waypoints)
        wp_actions.addWidget(clear_wp_btn)
        undo_btn = self.host.make_button("↩ Undo Last")
        undo_btn.clicked.connect(self._undo_last_waypoint)
        wp_actions.addWidget(undo_btn)
        wp_actions.addStretch()
        wp_layout.addLayout(wp_actions)
        waypoints_layout.addWidget(wp_grp, 1)

        save_grp = QGroupBox("SAVE TRAJECTORY")
        save_layout = QVBoxLayout(save_grp)
        save_layout.setContentsMargins(12, 16, 12, 12)
        save_layout.setSpacing(10)
        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Sequence Name:"))
        self.teach_sequence_name = QLineEdit()
        self.teach_sequence_name.setPlaceholderText("e.g., pick_and_place_v1")
        self.teach_sequence_name.setText(self.sequence_name)
        self.teach_sequence_name.textChanged.connect(lambda t: setattr(self, 'sequence_name', t))
        name_row.addWidget(self.teach_sequence_name, 1)
        save_layout.addLayout(name_row)

        save_btn_row = QHBoxLayout()
        save_seq_btn = self.host.make_button("SAVE AS SEQUENCE", "BtnTeal")
        save_seq_btn.clicked.connect(self._save_as_sequence)
        save_btn_row.addWidget(save_seq_btn)
        export_btn = self.host.make_button("Export YAML")
        export_btn.clicked.connect(self._export_waypoints)
        save_btn_row.addWidget(export_btn)
        save_btn_row.addStretch()
        save_layout.addLayout(save_btn_row)
        waypoints_layout.addWidget(save_grp)
        splitter.addWidget(waypoints_widget)
        splitter.setSizes([350, 500])
        layout.addWidget(splitter, 1)

        instructions = QLabel(
            "<b>How to use:</b> Pick <i>Joint</i> or <i>Cartesian</i> mode, then click 'Start Teaching' "
            "to put the robot in the matching admittance mode. Guide the robot by hand and click "
            "'Record Current Position' to save motion waypoints. Use the <b>Gripper</b> controls "
            "to actuate the gripper while teaching, and 'Record Gripper Waypoint' to drop a "
            "gripper-only step into the sequence. When done, save as a sequence (requires the "
            "Sequences extension to be enabled)."
        )
        instructions.setWordWrap(True)
        instructions.setObjectName('HintLabel')
        layout.addWidget(instructions)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(page)
        self._page = scroll
        return scroll

    # ── Web page (manifest "web"; web/page.js) ────────────────────────
    # The web page and the Qt page drive the same state below. No dialog boxes
    # on this path: a browser or a headless board cannot click them, so a
    # refusal is a ValueError the page shows, and the page asks before moving.

    def web_state(self) -> dict:
        rid = self.host.target_robot_id
        joints = self.host.current_joint_positions or []
        cart = self.current_cartesian_pose if len(self.current_cartesian_pose) >= 6 else []
        if self.host.current_ee_pose is not None and not self.teach_mode_active:
            cart = self._cartesian_now()
        return {
            'teaching': self.teach_mode_active,
            'mode': self.teach_target_mode,
            'elapsed_s': round(time.time() - self.teach_start_time, 1) if self.teach_mode_active and self.teach_start_time else 0.0,
            'target': self.host.robot_target(rid),
            'robot': self._robot_label(rid),
            'robot_id': rid,
            'robots': [{'value': r, 'label': self._robot_label(r)} for r in (self.host.active_robot_rids() or [rid])],
            'joints_deg': [round(math.degrees(q), 2) for q in joints],
            'cart': ([round(v * 1000.0, 1) for v in cart[:3]] + [round(math.degrees(v), 2) for v in cart[3:6]]) if cart else [],
            'auto_record': self.auto_record_enabled,
            'auto_interval_s': self.auto_record_interval,
            'threshold_mm': self.movement_threshold_mm,
            'gripper_width': self.gripper_width,
            'waypoints': [{'index': i, 'mode': wp.get('mode', 'Joint'), 'time': round(wp.get('time', 0.0), 1),
                           'text': self._waypoint_text(wp)} for i, wp in enumerate(self.recorded_waypoints)],
            'sequence_name': self.sequence_name,
            'sequences': self._sequences_ext() is not None,
        }

    def web_action(self, name: str, args: dict):
        def hardware_ok():
            # The target read now: the page asked with the one of its last poll.
            if self.host.robot_target(self.host.target_robot_id) == 'hardware' and args.get('hardware_ok') is not True:
                raise ValueError('Confirm moving the real robot')

        def index():
            i = int(args.get('index', -1))
            if not 0 <= i < len(self.recorded_waypoints):
                raise ValueError('No such waypoint')
            return i

        if name == 'start':
            if self.teach_mode_active:
                raise ValueError('Already teaching')
            hardware_ok()
            self.begin_teaching()
        elif name == 'stop':
            self._stop_teach_mode()
        elif name == 'mode':
            if self.teach_mode_active:
                raise ValueError('Stop teaching first')
            mode = str(args.get('value'))
            if mode not in ('Joint', 'Cartesian'):
                raise ValueError('The mode is Joint or Cartesian')
            if hasattr(self, 'teach_mode_selector'):
                self.teach_mode_selector.setCurrentText(mode)     # its signal sets teach_target_mode
            self.teach_target_mode = mode
        elif name == 'robot':
            if self.teach_mode_active:
                raise ValueError('Stop teaching first')
            if str(args.get('value')) not in self.host.active_robot_rids():
                raise ValueError('No such robot')
            self.host.set_target_robot(str(args['value']))
        elif name == 'record':
            if not self.teach_mode_active:
                raise ValueError('Start teaching first')
            self._record_waypoint()
        elif name == 'record_gripper':
            if not self.teach_mode_active:
                raise ValueError('Start teaching first')
            self._record_gripper_waypoint()
        elif name == 'auto':
            if 'interval' in args:
                self._set_bounded('auto_record_interval', 'auto_record_spin', args['interval'], 0.5, 10.0)
            if 'threshold' in args:
                self._set_bounded('movement_threshold_mm', 'movement_threshold_spin', args['threshold'], 0.1, 50.0)
            if 'enabled' in args:
                if hasattr(self, 'auto_record_check'):
                    self.auto_record_check.setChecked(bool(args['enabled']))   # its signal calls _toggle_auto_record
                else:
                    self._toggle_auto_record(bool(args['enabled']))
        elif name == 'gripper':
            self._set_bounded('gripper_width', 'teach_gripper_width', args.get('width'), 0.0, 0.08)
            if args.get('send'):
                hardware_ok()
                self.send_gripper(self.gripper_width)
        elif name == 'go':
            i = index()
            hardware_ok()
            self.move_to_waypoint(i)
        elif name == 'delete':
            self._delete_waypoint(index())
        elif name == 'undo':
            self._undo_last_waypoint()
        elif name == 'clear':
            self.clear_waypoints()
        elif name == 'name':
            self.sequence_name = str(args.get('value') or '').strip()
            if hasattr(self, 'teach_sequence_name'):
                self.teach_sequence_name.setText(self.sequence_name)
        elif name == 'save_sequence':
            saved, count = self.save_as_sequence(str(args.get('name') or self.sequence_name))
            return {'name': saved, 'count': count}
        else:
            raise ValueError(f'No such action: {name}')
        return {}

    def _set_bounded(self, attr, widget, value, lo, hi):
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError('Give a number')
        if not (lo <= value <= hi) or not math.isfinite(value):
            raise ValueError(f'Between {lo:g} and {hi:g}')
        setattr(self, attr, value)
        if hasattr(self, widget):
            getattr(self, widget).setValue(value)

    def _robot_label(self, rid):
        """'Robot 1 (KinovaGen3_7DOF)': which robot the page drives."""
        number = rid[len('robot'):] if rid.startswith('robot') else rid
        try:
            kind = self.host.current_robot_name()
        except Exception:  # noqa: BLE001
            kind = ''
        return f'Robot {number}' + (f' ({kind})' if kind else '')

    def _sequences_ext(self):
        mw = self.host.main_window
        return mw.get_extension("sequences") if hasattr(mw, "get_extension") else None

    @staticmethod
    def _waypoint_text(wp):
        mode = wp.get('mode', 'Joint')
        if mode == 'Gripper':
            w = wp.get('gripper_width', 0.0)
            return f"Gripper {w * 1000:.0f} mm ({(w / 0.08) * 100:.0f}%)"
        if mode == 'Joint' and wp.get('joint_positions'):
            return 'J ' + ', '.join(f"{math.degrees(q):.1f}°" for q in wp['joint_positions'])
        if wp.get('cartesian_pose') and len(wp['cartesian_pose']) >= 3:
            x, y, z = [v * 1000 for v in wp['cartesian_pose'][:3]]
            return f"X {x:.0f}  Y {y:.0f}  Z {z:.0f} mm"
        return '—'

    # ── Teaching ──────────────────────────────────────────────────────
    def _on_teach_mode_changed(self, mode):
        self.teach_target_mode = mode
        if not self.teach_mode_active:
            self.teach_status_label.setText(
                f"Teach mode inactive ({mode}). Click 'Start Teaching' to begin.")

    def _publish_teach_mode_string(self, mode_string):
        node = self.host.node
        if node is None:
            return False
        rid = self.host.target_robot_id
        pubs = node.pubs.get(rid, {})
        pub = pubs.get('mode_select')
        if pub is None:
            return False
        from std_msgs.msg import String
        msg = String(); msg.data = mode_string
        pub.publish(msg)
        return True

    def send_gripper(self, width_m):
        """Send a gripper width to the target robot now (does not record)."""
        node = self.host.node
        if node is None or not hasattr(node, 'publish_gripper_width'):
            raise ValueError('No gripper publisher: start the Studio session first')
        rid = self.host.target_robot_id
        node.publish_gripper_width(rid, float(width_m))
        self.host.log("INFO", f"Gripper → {width_m:.4f}m ({(width_m / 0.08) * 100.0:.0f}%)", "Teach")

    def _set_and_publish_gripper(self, width_m):
        self.teach_gripper_width.setValue(width_m)
        self._apply_gripper_width()

    def _apply_gripper_width(self):
        try:
            self.send_gripper(float(self.teach_gripper_width.value()))
        except ValueError as e:
            self.host.log("ERROR", str(e), "Teach")

    def _toggle_teach_mode(self):
        if not self.teach_mode_active:
            self._start_teach_mode()
        else:
            self._stop_teach_mode()

    def _start_teach_mode(self):
        active_mode = self.teach_target_mode
        admittance_mode = "Cartesian Admittance" if active_mode == "Cartesian" else "Joint Admittance"
        reply = QMessageBox.question(
            self._page, "Start Teach Mode",
            f"Switch the robot to {admittance_mode} so you can guide it by hand?\n\n"
            f"Waypoints will be recorded as {active_mode.lower()} targets.\n\n"
            "Make sure the area is clear. Continue?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        try:
            self.begin_teaching()
        except Exception as e:
            QMessageBox.critical(self._page, "Error", f"Failed to start teach mode: {e}")
            self.host.log("ERROR", f"Failed to start teach mode: {e}", "Teach")

    def begin_teaching(self):
        """Switch the target robot to admittance and start recording (no dialog)."""
        active_mode = self.teach_target_mode
        admittance_mode = "Cartesian Admittance" if active_mode == "Cartesian" else "Joint Admittance"
        if not self._publish_teach_mode_string(admittance_mode):
            self.host.log("WARN",
                "No mode publisher on the GUI node — robot mode not switched. "
                "Start fic_control first, then re-arm teach mode.", "Teach")
        else:
            self.host.log("INFO", f"Switched to {admittance_mode}", "Teach")
        self.teach_mode_active = True
        self.teach_start_time = time.time()
        self.last_auto_record_time = time.time()
        if hasattr(self, 'teach_toggle_btn'):
            self.teach_mode_selector.setEnabled(False)
            self.teach_toggle_btn.setText('■ STOP TEACHING')
            set_toggle_button_state(self.teach_toggle_btn, True)
            self.teach_status_icon.setText('●')
            self.teach_status_icon.setStyleSheet(
                'font-size: 18px; color: %s;' % status_color('active'))
            self.teach_status_label.setText(
                f'Teach mode ACTIVE ({active_mode}) — guide the robot and record waypoints')
            theme.bind(
                self.teach_status_frame,
                lambda t: (
                    'QFrame { background:%s; border:1px solid %s;'
                    ' border-radius:6px; padding:8px; }'
                ) % (t['bg_raised'], t['accent_teal'])
            )
            self.record_waypoint_btn.setEnabled(True)
            self.record_gripper_btn.setEnabled(True)
        self.teach_position_timer.start(50)
        self.host.log("INFO", f"Teach mode started ({active_mode})", "Teach")

    def _stop_teach_mode(self):
        self.teach_mode_active = False
        self.teach_position_timer.stop()
        playback_mode = ("Cartesian Planner" if self.teach_target_mode == "Cartesian"
                         else "Joint Impedance With Planner")
        try:
            self._publish_teach_mode_string(playback_mode)
        except Exception:
            pass
        if hasattr(self, 'auto_record_check'):
            self.auto_record_check.setChecked(False)
        self.auto_record_enabled = False
        if hasattr(self, 'teach_mode_selector'):
            self.teach_mode_selector.setEnabled(True)
        if hasattr(self, 'teach_toggle_btn'):
            self.teach_toggle_btn.setText('▶ START TEACHING')
            set_toggle_button_state(self.teach_toggle_btn, False)
            self.teach_status_icon.setText('○')
            self.teach_status_icon.setStyleSheet(
                'font-size: 18px; color: %s;' % status_color('idle'))
            self.teach_status_label.setText(
                f'Teach mode stopped. Recorded {len(self.recorded_waypoints)} waypoints.')
            theme.bind(
                self.teach_status_frame,
                lambda t: (
                    'QFrame { background:%s; border:1px solid %s;'
                    ' border-radius:6px; padding:8px; }'
                ) % (t['bg_raised'], t['border_subtle'])
            )
            self.record_waypoint_btn.setEnabled(False)
            self.record_gripper_btn.setEnabled(False)
        self.host.log("INFO",
            f"Teach mode stopped. {len(self.recorded_waypoints)} waypoints recorded.", "Teach")

    def _cartesian_now(self):
        """[x, y, z, roll, pitch, yaw] (m, rad) of the live end-effector pose; [] if none."""
        ee = self.host.current_ee_pose
        if ee is None:
            return []
        qx, qy, qz, qw = ee.orientation.x, ee.orientation.y, ee.orientation.z, ee.orientation.w
        self.current_ee_quat = (qx, qy, qz, qw)
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        sinp = 2 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return [ee.position.x, ee.position.y, ee.position.z, roll, pitch, yaw]

    def _poll_robot_position(self):
        try:
            joints = self.host.current_joint_positions
            joint_pos = list(joints) if joints else []
            cart_pose = self._cartesian_now() or [0.0] * 6
            self.current_cartesian_pose = cart_pose
            self.teach_signals.position_updated.emit(joint_pos, cart_pose)
            if self.auto_record_enabled and self.teach_mode_active:
                now = time.time()
                if now - self.last_auto_record_time >= self.auto_record_interval:
                    if self._position_changed_enough():
                        self._record_waypoint()
                    self.last_auto_record_time = now
        except Exception:
            pass

    def _position_changed_enough(self):
        if not self.recorded_waypoints or not self.current_cartesian_pose:
            return True
        last_wp = self.recorded_waypoints[-1]
        if not last_wp.get('cartesian_pose'):
            return True
        last_xyz = last_wp['cartesian_pose'][:3]
        curr_xyz = self.current_cartesian_pose[:3]
        distance_mm = math.sqrt(sum((a - b) ** 2 for a, b in zip(curr_xyz, last_xyz))) * 1000
        return distance_mm >= self.movement_threshold_mm

    def _update_position_display(self, joint_pos, cart_pose):
        if not hasattr(self, 'teach_joint_labels'):
            return
        num_joints = len(joint_pos)
        for i, (name_lbl, value_lbl) in enumerate(self.teach_joint_labels):
            if i < num_joints:
                name_lbl.setVisible(True); value_lbl.setVisible(True)
                value_lbl.setText(f"{math.degrees(joint_pos[i]):7.2f}°")      # joint_pos is in radians
            else:
                name_lbl.setVisible(False); value_lbl.setVisible(False)
        if len(cart_pose) >= 6:
            multipliers = [1000, 1000, 1000, 57.2958, 57.2958, 57.2958]
            for i, name in enumerate(["X", "Y", "Z", "Roll", "Pitch", "Yaw"]):
                if name in self.teach_cart_labels:
                    val = cart_pose[i] * multipliers[i] if i < len(cart_pose) else 0
                    self.teach_cart_labels[name].setText(f"{val:8.2f}")

    # ── Waypoints ─────────────────────────────────────────────────────
    def _record_waypoint(self):
        if not self.teach_mode_active:
            return
        elapsed = time.time() - self.teach_start_time if self.teach_start_time else 0
        joints = self.host.current_joint_positions or []
        cart = self._cartesian_now() or list(self.current_cartesian_pose)
        if not joints and not cart:
            raise ValueError('No robot feedback yet: start the controller first')
        wp = {
            'index': len(self.recorded_waypoints) + 1,
            'time': elapsed, 'mode': self.teach_target_mode,
            'joint_positions': list(joints),
            'cartesian_pose': list(cart),
            'cartesian_quat': list(self.current_ee_quat),
            'timestamp': datetime.now().isoformat(),
        }
        self.recorded_waypoints.append(wp)
        self.teach_signals.waypoint_recorded.emit(wp)
        self.host.log("INFO", f"Waypoint {wp['index']} ({wp['mode']}) recorded", "Teach")

    def _record_waypoint_clicked(self):
        try:
            self._record_waypoint()
        except ValueError as e:
            QMessageBox.warning(self._page, "Record Waypoint", str(e))

    def _record_gripper_waypoint(self):
        if not self.teach_mode_active:
            return
        elapsed = time.time() - self.teach_start_time if self.teach_start_time else 0
        width_m = float(self.gripper_width)
        wp = {
            'index': len(self.recorded_waypoints) + 1,
            'time': elapsed, 'mode': 'Gripper',
            'gripper_width': width_m,
            'timestamp': datetime.now().isoformat(),
        }
        self.recorded_waypoints.append(wp)
        self.teach_signals.waypoint_recorded.emit(wp)
        pct = (width_m / 0.08) * 100.0
        self.host.log("INFO",
            f"Waypoint {wp['index']} (Gripper) recorded — {width_m:.4f}m ({pct:.0f}%)", "Teach")

    def _on_waypoint_recorded(self, wp):
        self._refresh_table()

    def _refresh_table(self):
        """Redraw the Qt table from recorded_waypoints, so each row's buttons
        address the waypoint now in that row."""
        for i, wp in enumerate(self.recorded_waypoints):
            wp['index'] = i + 1
        if not hasattr(self, 'waypoints_table'):
            return
        self.waypoint_count_label.setText(f"Waypoints: {len(self.recorded_waypoints)}")
        self.waypoints_table.setRowCount(0)
        for row, wp in enumerate(self.recorded_waypoints):
            self.waypoints_table.insertRow(row)
            idx_item = QTableWidgetItem(str(wp['index'])); idx_item.setTextAlignment(Qt.AlignCenter)
            self.waypoints_table.setItem(row, 0, idx_item)
            time_item = QTableWidgetItem(f"{wp['time']:.1f}s"); time_item.setTextAlignment(Qt.AlignCenter)
            self.waypoints_table.setItem(row, 1, time_item)
            self.waypoints_table.setItem(row, 2, QTableWidgetItem(
                f"[{wp.get('mode', 'Joint')}] {self._waypoint_text(wp)}"))
            actions_widget = QWidget()
            actions_layout = QHBoxLayout(actions_widget)
            actions_layout.setContentsMargins(2, 2, 2, 2); actions_layout.setSpacing(2)
            goto_btn = self.host.make_button('Go', 'BtnPrimary')
            goto_btn.setMaximumWidth(35)
            goto_btn.clicked.connect(partial(self._goto_waypoint, row))
            actions_layout.addWidget(goto_btn)
            del_btn = self.host.make_button('✕', 'BtnWarn')
            del_btn.setMaximumWidth(25)
            del_btn.clicked.connect(partial(self._delete_waypoint, row))
            actions_layout.addWidget(del_btn)
            self.waypoints_table.setCellWidget(row, 3, actions_widget)
        self.waypoints_table.scrollToBottom()

    def _goto_waypoint(self, row):
        if row < 0 or row >= len(self.recorded_waypoints):
            return
        wp = self.recorded_waypoints[row]
        if QMessageBox.question(self._page, "Move to Waypoint",
                                f"Move robot to waypoint {wp['index']} ({wp.get('mode', 'Joint')})?",
                                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        try:
            self.move_to_waypoint(row)
        except ValueError as e:
            self.host.log("ERROR", str(e), "Teach")

    def move_to_waypoint(self, row):
        """Send recorded waypoint ``row`` to the target robot (no dialog)."""
        wp = self.recorded_waypoints[row]
        wp_mode = wp.get('mode', 'Joint')
        node = self.host.node
        if node is None:
            raise ValueError('No ROS node: start the Studio session first')
        target_rid = self.host.target_robot_id
        if wp_mode == "Gripper":
            self.send_gripper(float(wp.get('gripper_width', 0.0)))
        elif wp_mode == "Cartesian":
            quat = wp.get('cartesian_quat')
            pose_rpy = wp.get('cartesian_pose') or []
            if not quat or len(pose_rpy) < 3:
                raise ValueError(f"Waypoint {wp['index']} has no Cartesian pose")
            from geometry_msgs.msg import Pose, Point, Quaternion
            pose = Pose(position=Point(x=pose_rpy[0], y=pose_rpy[1], z=pose_rpy[2]),
                        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
            pub = node.pubs.get(target_rid, {}).get('position')
            if pub is None:
                raise ValueError(f"No Cartesian pose publisher for {target_rid}")
            pub.publish(pose)
            self.host.log("INFO", f"Moving to waypoint {wp['index']} (Cartesian)", "Teach")
        else:
            joint_pos_rad = [float(v) for v in (wp.get('joint_positions') or [])]
            if not joint_pos_rad:
                raise ValueError(f"Waypoint {wp['index']} has no joint angles")
            # Hardware takes desired_joint_angles; simulation the JointState target.
            if self.host.robot_target(target_rid) == 'hardware':
                node.publish_real_joint_angles(target_rid, joint_pos_rad)
            else:
                names = [f'joint_{i + 1}' for i in range(len(joint_pos_rad))]
                node.publish_joint_angles(joint_pos_rad, names, target_rid)
            self.host.log("INFO", f"Moving to waypoint {wp['index']} (Joint)", "Teach")

    def _delete_waypoint(self, row):
        if row < 0 or row >= len(self.recorded_waypoints):
            return
        self.recorded_waypoints.pop(row)
        self._refresh_table()

    def _undo_last_waypoint(self):
        if self.recorded_waypoints:
            self._delete_waypoint(len(self.recorded_waypoints) - 1)

    def _clear_waypoints(self):
        if not self.recorded_waypoints:
            return
        if QMessageBox.question(self._page, "Clear Waypoints",
                                f"Delete all {len(self.recorded_waypoints)} recorded waypoints?",
                                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.clear_waypoints()

    def clear_waypoints(self):
        self.recorded_waypoints.clear()
        self._refresh_table()
        self.host.log("INFO", "Cleared all waypoints", "Teach")

    def _toggle_auto_record(self, enabled):
        self.auto_record_enabled = bool(enabled)
        self.last_auto_record_time = time.time()
        if enabled:
            self.host.log("INFO",
                f"Auto-record enabled ({self.auto_record_interval}s interval)", "Teach")

    def _sequence_steps(self):
        """The recorded waypoints in the Sequences extension's format."""
        from fic_studio.math_helpers import _quat_to_axis_angle
        steps = []
        for wp in self.recorded_waypoints:
            wp_mode = wp.get('mode', 'Joint')
            if wp_mode == "Gripper":
                step = {'type': 'gripper', 'width': float(wp.get('gripper_width', 0.0)),
                        'hold_time': 0.5, 'label': f"Waypoint {wp['index']}"}
            elif wp_mode == "Cartesian":
                pose_rpy = wp.get('cartesian_pose') or [0.0] * 6
                quat = wp.get('cartesian_quat') or [0.0, 0.0, 0.0, 1.0]
                ax, ay, az, angle_deg = _quat_to_axis_angle(quat[0], quat[1], quat[2], quat[3], "deg")
                step = {'type': 'cartesian',
                        'position': [pose_rpy[0], pose_rpy[1], pose_rpy[2]],
                        'axis_angle': [ax, ay, az, angle_deg],
                        'hold_time': 0.5, 'label': f"Waypoint {wp['index']}"}
            else:
                joint_pos_deg = [math.degrees(r) for r in (wp.get('joint_positions') or [])]
                step = {'type': 'joint', 'values': joint_pos_deg,
                        'hold_time': 0.5, 'label': f"Waypoint {wp['index']}"}
            steps.append(step)
        return steps

    def save_as_sequence(self, name):
        """Store the recorded waypoints as a sequence and save the file (no
        dialog). Returns (name, count)."""
        if not self.recorded_waypoints:
            raise ValueError('No waypoints recorded yet')
        seq_ext = self._sequences_ext()
        if seq_ext is None:
            raise ValueError('Turn on the Sequences extension (Settings › Extensions) to save sequences')
        name = (name or '').strip() or f"taught_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        steps = self._sequence_steps()
        if hasattr(seq_ext, 'add_sequence'):
            seq_ext.add_sequence(name, steps)
        else:                                   # an older Sequences extension
            seq_ext.sequences[name] = steps
        self.host.log("INFO", f"Saved sequence '{name}' with {len(steps)} steps", "Teach")
        return name, len(steps)

    def _save_as_sequence(self):
        try:
            name, count = self.save_as_sequence(self.teach_sequence_name.text())
        except ValueError as e:
            QMessageBox.warning(self._page, "Save as Sequence", str(e))
            return
        except Exception as e:  # noqa: BLE001 — the file write
            QMessageBox.critical(self._page, "Save as Sequence", f"Could not save: {e}")
            return
        QMessageBox.information(self._page, "Sequence Saved",
            f"Saved {count} waypoints as sequence '{name}'.\n\n"
            f"Go to the Sequences page to play it back.")

    def _export_waypoints(self):
        if not self.recorded_waypoints:
            QMessageBox.warning(self._page, "No Waypoints", "No waypoints to export.")
            return
        import yaml
        filename, _ = QFileDialog.getSaveFileName(
            self._page, "Export Waypoints",
            f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.yaml",
            "YAML Files (*.yaml *.yml)")
        if not filename:
            return
        try:
            export_data = {
                'name': self.teach_sequence_name.text() or 'unnamed',
                'created': datetime.now().isoformat(),
                'robot': self.host.current_robot_name(),
                'waypoints': self.recorded_waypoints,
            }
            with open(filename, 'w') as f:
                yaml.dump(export_data, f, default_flow_style=False)
            QMessageBox.information(self._page, "Exported", f"Waypoints exported to:\n{filename}")
            self.host.log("INFO", f"Exported waypoints to {filename}", "Teach")
        except Exception as e:
            QMessageBox.critical(self._page, "Error", f"Failed to export: {e}")
