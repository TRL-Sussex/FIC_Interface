# mixins/control_bar_mixin.py — Global bottom control bar
"""
Control bar mixin: the Studio's robot-driving surface, on every page.

Replaces the Robot Control page. Modelled on the Kinova Kortex Web App's
robot control panel — a persistent strip along the bottom whose buttons
slide panels up from beneath.

The eight FIC control modes collapse into two expanding menus. Each menu
owns its four modes, the modifiers that apply to its kind, and the config
profile selector, so a modifier can no longer be visible for a mode it
does not apply to.

The *engine* stays in ``robot_control_mixin``: this module builds panels
and wires them to the existing handlers rather than reimplementing them.
The control surface (joint fields or EE pose fields) is shared and
re-parented into whichever panel is open — ``_rc_joint_fields`` and
``_rc_pose_fields`` are singular state that ``_rc_publish_joints`` reads,
so there can only be one surface.
"""

from functools import partial

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QHBoxLayout, QInputDialog, QLabel,
    QMessageBox, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

from fic_studio import theme
from fic_studio.components.control_panel_host import ControlPanelHost
from fic_studio.mixins.robot_control_mixin import (
    CONTROL_MODE_NAMES, CONTROL_MODE_TABS, mode_display_name,
)

try:
    from fic_contact.residual import LINK_NAMES
except ImportError:  # pragma: no cover
    # fic_contact is a workspace-local package, only importable once
    # install/setup.bash has been sourced (unlike std_srvs above, which
    # ships inside the conda env itself). Unit tests build the control
    # bar without a sourced workspace, so fall back to the same 7 Kinova
    # Gen3 link names fic_contact.residual.LINK_NAMES defines, keeping the
    # ground-truth combo populated either way.
    LINK_NAMES = (
        'Shoulder_Link', 'HalfArm1_Link', 'HalfArm2_Link', 'ForeArm_Link',
        'SphericalWrist1_Link', 'SphericalWrist2_Link', 'Bracelet_Link',
    )

#: Menu key → (button label, the CONTROL_MODE_TABS indices it owns).
MODE_MENUS = {
    'joint': ('Joint', [0, 1, 2, 3]),
    'cart': ('Cart', [4, 5, 6, 7]),
}

#: Row labels inside a menu. The menu button already says Joint / Cart, so
#: the rows drop the redundant prefix. Published names stay CONTROL_MODE_NAMES.
MODE_ROW_LABELS = ['Impedance', 'Imp. + Planner', 'Admittance', 'HAP-FIC']

NO_SEQUENCES = ('The Sequences extension is disabled — enable it in '
                'Settings ▸ Extensions to play or record sequences.')

NO_CARTESIAN = ('This robot has no kinematics solver, so Cartesian modes '
                'cannot run on it.')

VR_DRIVEN = ('The VR target drives this robot from the headset — local '
             'commanding is disabled while VR is selected.')

#: Width reserved for the status readout, mirrored on the left so the
#: button group centres in the bar rather than sitting left of centre.
STATUS_WIDTH = 230

#: One height for every control on the menu line, so nothing sits proud of
#: its neighbours or overlaps them.
ROW_HEIGHT = 26


def _hint(text):
    """Return a dim explanatory label."""
    q = QLabel(text)
    theme.bind(q, lambda T: (
        'background: transparent; border: none; font-size: 11px; color: %s;'
        % T['text_disabled']
    ))
    return q


def _separator():
    """Return a thin vertical rule dividing the groups on the menu line."""
    line = QFrame()
    line.setFrameShape(QFrame.VLine)
    line.setFixedWidth(1)
    line.setFixedHeight(ROW_HEIGHT - 4)
    theme.bind(line, lambda T: 'background: %s; border: none;' % T['border_subtle'])
    return line


def _caption(text):
    """Return a small fixed-width caption label for a panel row."""
    q = QLabel(text)
    q.setFixedWidth(58)
    theme.bind(q, lambda T: (
        'background: transparent; border: none; font-size: 10px; color: %s;'
        % T['text_disabled']
    ))
    return q


class ControlBarMixin:
    """Mixin providing the global bottom control bar and its panels."""

    # ── Construction ──────────────────────────────────────────────────────

    def _build_control_bar(self):
        """Build the control bar and return its ControlPanelHost."""
        self._cb_mode_buttons = {}
        self._cb_profile_combos = {}
        self._cb_syncing_profile = False

        # The bar is built before the Multi-Robot page, so it seeds the
        # control state the shared surface and the publish handlers read.
        node = getattr(self, 'node', None)
        default_rid = getattr(node, 'station_robot_id', 'robot1')
        self._rc_active_rid = getattr(self, '_rc_active_rid', default_rid)
        self._rc_joint_fields = getattr(self, '_rc_joint_fields', [])
        self._rc_pose_fields = getattr(self, '_rc_pose_fields', {})
        self._rc_selected_mode_idx = getattr(self, '_rc_selected_mode_idx', 0)
        self._rc_selected_kind = getattr(self, '_rc_selected_kind', 'joint')
        self._robot_targets = getattr(self, '_robot_targets', {})
        self.control_target = getattr(self, 'control_target', 'simulation')

        bar = ControlPanelHost()
        self._control_bar = bar

        # Hidden mode dropdown kept for publisher_mixin, which reads
        # robot_1_mode_dropdown to resolve the active mode.
        self._rc_mode_dropdown = QComboBox()
        self._rc_mode_dropdown.addItems(CONTROL_MODE_NAMES)
        self._rc_mode_dropdown.hide()
        self.robot_1_mode_dropdown = self._rc_mode_dropdown

        # Shared control surface, re-parented into the open panel.
        self._rc_control_area = self._build_rc_control_area()

        bar.add_panel('joint', '▲  Joint', self._cb_build_mode_panel('joint'))
        bar.add_panel('cart', '▲  Cart', self._cb_build_mode_panel('cart'))
        bar.panel_changed.connect(self._cb_on_panel_changed)
        # One surface, shown under either menu. A mode is always selected,
        # so the controls are always there — no placeholder, no empty panel.
        bar.set_shared(self._rc_control_area, ('joint', 'cart'))

        self._rc_gripper_group = self._build_rc_gripper_group()
        bar.add_panel('gripper', '▲  Gripper', self._cb_wrap(self._rc_gripper_group))

        bar.add_action('home', '⌂  Home', self._cb_on_home)

        # The viewer launch/stop toggle lives in the top toolbar now
        # (main_window_pro's tb_viewer_btn, beside ▶ PLT) -- hardware-only,
        # since driving a viewer of the real arm only makes sense while
        # commanding hardware. _cb_on_contact_twin stays here and is what
        # that button calls; only its own control-bar action button (this
        # one used to be it) moved out. The Contact panel -- Calibrate and
        # the ground-truth combo -- stays exactly where it was.
        bar.add_panel('contact', '◈  Contact', self._cb_build_contact_panel())

        bar.add_panel('snapshot', '◉  Snapshot', self._cb_build_snapshot_panel())
        bar.add_panel('quick', '▲  Quick Actions', self._cb_build_quick_panel())

        # Centre the button group. The trailing stretch alone would push it
        # left of centre by the width of the status readout, so the same
        # width is reserved on the far left.
        bar.add_stretch()
        bar.add_leading_stretch()
        bar.add_left_spacer(STATUS_WIDTH)

        self._cb_status_lbl = QLabel('--')
        self._cb_status_lbl.setFixedWidth(STATUS_WIDTH)
        self._cb_status_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        theme.bind(self._cb_status_lbl, lambda T: (
            'background: transparent; font-size: 11px; color: %s;'
            " font-family: 'JetBrains Mono', 'Consolas', monospace;"
            % T['text_secondary']
        ))
        bar.add_widget(self._cb_status_lbl)

        self._cb_contact_lbl = QLabel('--')
        self._cb_contact_lbl.setFixedWidth(STATUS_WIDTH)
        self._cb_contact_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        theme.bind(self._cb_contact_lbl, lambda T: (
            'background: transparent; font-size: 11px; color: %s;'
            " font-family: 'JetBrains Mono', 'Consolas', monospace;"
            % T['text_secondary']
        ))
        bar.add_widget(self._cb_contact_lbl)

        # The indicator reports live telemetry, so it needs its own tick:
        # before this it only refreshed on mode/target/robot changes and sat
        # frozen the rest of the time. 5 Hz is enough to read and costs far
        # less than the dashboard's 20 Hz.
        self._cb_status_timer = QTimer()
        self._cb_status_timer.timeout.connect(self._cb_refresh_status)
        self._cb_status_timer.start(200)

        self._cb_place_surface()
        self._cb_sync_mode_buttons()
        self._cb_apply_availability()
        self._cb_refresh_status()
        return bar

    @staticmethod
    def _cb_wrap(widget):
        """Wrap a widget in a plain container suitable for a panel body."""
        holder = QWidget()
        lay = QVBoxLayout(holder)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(widget)
        return holder

    def _cb_build_mode_panel(self, kind):
        """Build the Joint or Cart menu body.

        Mode, Options and Config sit on one line. As three stacked rows they
        each ran a few hundred pixels into a bar well over a thousand wide,
        and the short rows left the panel taller than it needed to be.
        """
        body = QWidget()
        row = QHBoxLayout(body)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        row.setAlignment(Qt.AlignVCenter)

        # ── Mode ──
        row.addWidget(_caption('Mode'), 0, Qt.AlignVCenter)
        buttons = []
        for slot, tab_idx in enumerate(MODE_MENUS[kind][1]):
            btn = QPushButton(MODE_ROW_LABELS[slot])
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setObjectName('ModeTab')
            btn.setFixedHeight(ROW_HEIGHT)
            btn.clicked.connect(partial(self._cb_on_mode_row, kind, tab_idx))
            row.addWidget(btn, 0, Qt.AlignVCenter)
            buttons.append(btn)
        self._cb_mode_buttons[kind] = buttons

        # ── Options — permanently scoped to this menu's kind ──
        row.addWidget(_separator(), 0, Qt.AlignVCenter)
        row.addWidget(_caption('Options'), 0, Qt.AlignVCenter)
        for box in self._cb_build_modifiers(kind):
            box.setFixedHeight(ROW_HEIGHT)
            row.addWidget(box, 0, Qt.AlignVCenter)

        # ── Config ──
        row.addWidget(_separator(), 0, Qt.AlignVCenter)
        row.addWidget(_caption('Config'), 0, Qt.AlignVCenter)
        combo = QComboBox()
        combo.setFixedHeight(ROW_HEIGHT)
        combo.setMinimumWidth(170)
        combo.setToolTip('Active tuning config (profile) for the selected '
                         'control mode. Create new configs in the Tuning panel.')
        row.addWidget(combo, 0, Qt.AlignVCenter)
        self._cb_profile_combos[kind] = combo

        row.addStretch(1)

        # The joint combo is the canonical one the rest of the app reads.
        if kind == 'joint':
            self._rc_profile_dropdown = combo
        combo.currentIndexChanged.connect(partial(self._cb_on_profile_changed, kind))
        return body

    def _cb_build_modifiers(self, kind):
        """Create this menu's modifier checkboxes and return them."""
        if kind == 'joint':
            self._rc_admittance_planner_toggle = QCheckBox('Admittance planner')
            self._rc_admittance_planner_toggle.setToolTip(
                'Route the joint admittance target through the trajectory '
                'planner instead of commanding it directly.')
            self._rc_admittance_planner_toggle.toggled.connect(
                self._on_rc_admittance_planner_toggled)
            return [self._rc_admittance_planner_toggle]

        self._rc_postural_toggle = QCheckBox('Postural')
        self._rc_postural_toggle.setToolTip(
            'Null-space joint-target tracking (uses q_target_ik_). '
            'Stabilises redundant joint configuration during Cartesian modes '
            'without affecting the EE target.')
        self._rc_postural_toggle.toggled.connect(self._on_rc_postural_toggled)

        self._rc_elbow_toggle = QCheckBox('Elbow')
        self._rc_elbow_toggle.setToolTip(
            "Null-space elbow hold. On enable, the robot's current elbow "
            'position is captured as the target and held without disturbing '
            'the EE task.')
        self._rc_elbow_toggle.toggled.connect(self._on_rc_elbow_toggled)

        self._rc_hpmp_toggle = QCheckBox('H-PMP')
        self._rc_hpmp_toggle.setToolTip(
            'H-PMP: nests the Cartesian controller (modes 5-8) inside the joint '
            'HAP-FIC, so every joint yields to a push while the end-effector '
            'tracks its goal. On by default; off sends the Cartesian torques '
            'directly to the joints.')
        self._rc_hpmp_toggle.setChecked(True)      # on by default (ros_node.add_robot sends it)
        self._rc_hpmp_toggle.toggled.connect(self._on_rc_hpmp_toggled)

        self._rc_desired_wrench_btn = QPushButton('Wrench…')
        self._rc_desired_wrench_btn.setToolTip(
            'Desired wrench of the Cartesian admittance (modes 7 and 8, and '
            'H-PMP on them): the goal moves until the measured wrench '
            'equals it. Zero at controller start-up.')
        self._rc_desired_wrench_btn.clicked.connect(self._open_desired_wrench_dialog)

        return [self._rc_postural_toggle, self._rc_elbow_toggle,
                self._rc_hpmp_toggle, self._rc_desired_wrench_btn]

    def _cb_build_quick_panel(self):
        """Build the Quick Actions body: a sequence (new, play, step, stop, loop, speed), a
        trajectory template (line, circle, figure-eight: run it, or save it as a sequence) and the
        actions extensions add (ExtensionHost.add_quick_action, e.g. the Playground's Pick)."""
        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        def button(label, slot, tip=''):
            b = QPushButton(label)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(slot)
            if tip:
                b.setToolTip(tip)
            return b

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(_caption('Sequence'))
        self._cb_sequence_combo = QComboBox()
        self._cb_sequence_combo.setMinimumWidth(200)
        row.addWidget(self._cb_sequence_combo)
        row.addWidget(button('New…', self._cb_new_sequence, 'Make an empty sequence; Snapshot then adds waypoints to it'))
        self._cb_sequence_info = _hint('')
        row.addWidget(self._cb_sequence_info)
        row.addStretch()
        for label, slot, tip in (('▶  Play', self._cb_play_sequence, ''),
                                 ('⏭  Step', self._cb_step_sequence, 'Move to the next waypoint'),
                                 ('■  Stop', self._cb_stop_sequence, 'Stop the sequence or the trajectory'),
                                 ('↻  Loop', self._cb_loop_sequence, '')):
            row.addWidget(button(label, slot, tip))
        row.addWidget(_hint('Speed'))
        self._cb_speed = QDoubleSpinBox()
        self._cb_speed.setRange(0.1, 3.0)
        self._cb_speed.setSingleStep(0.1)
        self._cb_speed.setDecimals(1)
        self._cb_speed.setSuffix('×')
        self._cb_speed.setValue(1.0)
        self._cb_speed.setToolTip('Playback speed: hold times are divided by it')
        self._cb_speed.valueChanged.connect(self._cb_set_speed)
        row.addWidget(self._cb_speed)
        lay.addLayout(row)

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(_caption('Trajectory'))
        self._cb_traj_kind = QComboBox()
        for key, label in (('line', 'Line'), ('circle', 'Circle'), ('eight', 'Figure-eight')):
            self._cb_traj_kind.addItem(label, key)
        row.addWidget(self._cb_traj_kind)
        self._cb_traj_size = QSpinBox()
        self._cb_traj_size.setRange(10, 150)
        self._cb_traj_size.setValue(50)
        self._cb_traj_size.setSuffix(' mm')
        self._cb_traj_size.setToolTip("The line's half-length, the circle's radius, the figure-eight's half-width")
        row.addWidget(self._cb_traj_size)
        self._cb_traj_period = QDoubleSpinBox()
        self._cb_traj_period.setRange(2.0, 60.0)
        self._cb_traj_period.setValue(8.0)
        self._cb_traj_period.setSuffix(' s')
        self._cb_traj_period.setToolTip('Time for one cycle')
        row.addWidget(self._cb_traj_period)
        self._cb_traj_cycles = QSpinBox()
        self._cb_traj_cycles.setRange(1, 10)
        self._cb_traj_cycles.setValue(2)
        self._cb_traj_cycles.setSuffix(' ×')
        row.addWidget(self._cb_traj_cycles)
        self._cb_traj_plane = QComboBox()
        self._cb_traj_plane.addItems(['xy', 'xz', 'yz'])
        self._cb_traj_plane.setToolTip('The plane of the base frame it is traced in')
        row.addWidget(self._cb_traj_plane)
        row.addStretch()
        row.addWidget(button('▶  Run', self._cb_run_trajectory,
                             'Trace it from where the end effector is, holding its orientation'))
        row.addWidget(button('Save as sequence…', self._cb_save_trajectory,
                             'One cycle as Cartesian waypoints, to compare a sequence with the trajectory'))
        lay.addLayout(row)

        self._cb_ext_actions_box = QVBoxLayout()
        self._cb_ext_actions_box.setSpacing(6)
        lay.addLayout(self._cb_ext_actions_box)
        self._cb_rebuild_quick_actions()

        self._cb_sequence_status = _hint('')
        lay.addWidget(self._cb_sequence_status)
        lay.addStretch()
        self._cb_refresh_sequences()
        return body

    # ── Quick Actions: sequence and trajectory controls ───────────────────

    def _cb_say(self, text, level='INFO'):
        status = getattr(self, '_cb_sequence_status', None)
        if status is not None:
            status.setText(text)
        if level != 'INFO':
            self._cb_log(level, text)

    def _cb_new_sequence(self):
        ext = self._cb_sequences()
        if ext is None:
            return self._cb_say('The Sequences extension is off: turn it on in Settings, Extensions.', 'WARN')
        name, ok = QInputDialog.getText(self, 'New sequence', 'Name:')
        if not ok:
            return
        try:
            name = ext.new_sequence(name)
        except Exception as exc:
            return self._cb_say(str(exc), 'WARN')
        self._cb_refresh_sequences(select=name)
        self._cb_say('New sequence "%s": move the arm and press Snapshot for each waypoint.' % name)

    def _cb_step_sequence(self):
        ext = self._cb_sequences()
        name = self._cb_selected_sequence()
        if ext is None or name is None:
            return
        try:
            i = ext.step(name)
            self._cb_say('Moving to waypoint %d of "%s"' % (i + 1, name))
        except Exception as exc:
            self._cb_say(str(exc), 'WARN')

    def _cb_set_speed(self, value):
        ext = self._cb_sequences()
        if ext is not None and hasattr(ext, 'set_speed'):
            try:
                ext.set_speed(value)
            except Exception as exc:
                self._cb_say(str(exc), 'WARN')

    def _cb_traj_args(self):
        return (self._cb_traj_kind.currentData(), self._cb_traj_size.value() / 1000.0,
                self._cb_traj_period.value(), self._cb_traj_cycles.value(),
                self._cb_traj_plane.currentText())

    def _cb_run_trajectory(self):
        ext = self._cb_sequences()
        if ext is None or not hasattr(ext, 'run_trajectory'):
            return self._cb_say('The Sequences extension is off: turn it on in Settings, Extensions.', 'WARN')
        hw = ext.target_is_hardware()
        if hw and QMessageBox.question(
                self, 'Move the real robot?',
                'The trajectory goes to the REAL robot (the Sequences target). Run it?') != QMessageBox.Yes:
            return
        try:
            secs = ext.run_trajectory(*self._cb_traj_args(), hardware_ok=hw)
            self._cb_say('Tracing the %s: %.0f s. Stop ends it.' % (self._cb_traj_kind.currentText().lower(), secs))
        except Exception as exc:
            self._cb_say(str(exc), 'WARN')

    def _cb_save_trajectory(self):
        ext = self._cb_sequences()
        if ext is None or not hasattr(ext, 'save_trajectory'):
            return
        kind, size, _, _, plane = self._cb_traj_args()
        name, ok = QInputDialog.getText(self, 'Save as sequence', 'Name:',
                                        text='%s_%dmm' % (kind, round(size * 1000)))
        if not ok:
            return
        try:
            name = ext.save_trajectory(name, kind, size, plane)
        except Exception as exc:
            return self._cb_say(str(exc), 'WARN')
        self._cb_refresh_sequences(select=name)
        self._cb_say('Saved "%s": play it to see the arm go waypoint by waypoint instead.' % name)

    # ── Quick Actions added by extensions ─────────────────────────────────

    def _cb_rebuild_quick_actions(self):
        """(Re)draw the rows of the actions extensions added (ExtensionHost.add_quick_action)."""
        box = getattr(self, '_cb_ext_actions_box', None)
        if box is None:
            return
        while box.count():
            item = box.takeAt(0)
            lay = item.layout()
            if lay is not None:
                while lay.count():
                    w = lay.takeAt(0).widget()
                    if w is not None:
                        w.deleteLater()
        self._cb_ext_choice = {}
        for key, act in sorted(getattr(self, '_quick_actions', {}).items()):
            row = QHBoxLayout()
            row.setSpacing(10)
            row.addWidget(_caption(act['label']))
            combo = None
            if act.get('choices') is not None:
                combo = QComboBox()
                combo.setMinimumWidth(160)
                row.addWidget(combo)
                self._cb_ext_choice[key] = combo
            if act.get('hint'):
                row.addWidget(_hint(act['hint']))
            row.addStretch()
            btn = QPushButton(act.get('button') or '▶  Run')
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(partial(self._cb_run_quick_action, key))
            row.addWidget(btn)
            box.addLayout(row)
        self._cb_refresh_quick_choices()

    def _cb_refresh_quick_choices(self):
        for key, combo in getattr(self, '_cb_ext_choice', {}).items():
            act = getattr(self, '_quick_actions', {}).get(key)
            if act is None:
                continue
            try:
                choices = [str(c) for c in (act['choices']() or [])]
            except Exception:
                choices = []
            keep = combo.currentText()
            if [combo.itemText(i) for i in range(combo.count())] != choices:
                combo.clear()
                combo.addItems(choices)
                if keep in choices:
                    combo.setCurrentText(keep)

    def _cb_run_quick_action(self, key):
        act = getattr(self, '_quick_actions', {}).get(key)
        if act is None:
            return
        combo = getattr(self, '_cb_ext_choice', {}).get(key)
        choice = combo.currentText() if combo is not None else None
        if combo is not None and not choice:
            return self._cb_say('%s: nothing to choose yet' % act['label'], 'WARN')
        try:
            msg = act['run'](choice)
            self._cb_say(str(msg) if msg else '%s: done' % act['label'])
            self._cb_refresh_sequences()
        except Exception as exc:
            self._cb_say(str(exc), 'WARN')

    def _cb_build_snapshot_panel(self):
        """Build the Snapshot body: capture the live pose as a waypoint."""
        body = QWidget()
        lay = QVBoxLayout(body)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(_caption('Capture as'))
        self._cb_snapshot_kind = QComboBox()
        self._cb_snapshot_kind.addItems(['cartesian', 'joint', 'gripper'])
        row.addWidget(self._cb_snapshot_kind)
        row.addStretch()
        btn = QPushButton('◉  Capture')
        btn.setCursor(Qt.PointingHandCursor)
        btn.clicked.connect(self._cb_capture_snapshot)
        row.addWidget(btn)
        lay.addLayout(row)
        lay.addWidget(_hint('Appends a waypoint to the sequence selected in '
                            'Quick Actions.'))
        lay.addStretch()
        return body

    def _cb_build_contact_panel(self):
        """Build the Contact panel: calibrate and a ground-truth combo."""
        body = QWidget()
        row = QHBoxLayout(body)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        row.addWidget(_caption('Contact'))
        btn = QPushButton('◉  Calibrate')
        btn.setCursor(Qt.PointingHandCursor)
        btn.clicked.connect(self._cb_on_calibrate)
        row.addWidget(btn)
        row.addWidget(_separator())
        row.addWidget(_caption('Ground truth'))
        self._cb_gt_combo = QComboBox()
        self._cb_gt_combo.addItem('none')
        self._cb_gt_combo.addItems(list(LINK_NAMES))
        row.addWidget(self._cb_gt_combo)
        row.addStretch(1)
        row.addWidget(_hint(
            'Hold the arm still and clear of contact while calibrating.'))
        # Connected only after every item is added: QComboBox emits
        # currentTextChanged as soon as the first addItem gives it a
        # current index, and firing the (blocking, up to 1 s) parameter
        # call at construction time -- before anyone has touched the
        # combo -- would stall Studio startup for no reason.
        self._cb_gt_combo.currentTextChanged.connect(
            self._cb_on_ground_truth_changed)
        return body

    # ── Mode selection ────────────────────────────────────────────────────

    def _cb_on_panel_changed(self, key):
        """Opening a mode menu commits to that kind of control.

        Pressing Joint while a Cartesian mode is running selects Joint
        Impedance, and Cart selects Cartesian Impedance — the base mode of
        the menu you just opened, so the controls below it are live
        immediately rather than commanding the other kind.

        A menu already holding the active mode is left alone: opening Joint
        while Joint Admittance is running must not silently drop you back
        to Impedance.
        """
        if key in ('quick', 'snapshot'):
            self._cb_refresh_sequences()
            self._cb_refresh_quick_choices()
            return
        if key not in MODE_MENUS:
            return
        indices = MODE_MENUS[key][1]
        if getattr(self, '_rc_selected_mode_idx', 0) in indices:
            return
        base = indices[0]
        self._on_rc_mode_selected(base, CONTROL_MODE_TABS[base][0])

    def _cb_on_mode_row(self, kind, tab_idx):
        """Handle a click on one of a menu's mode rows."""
        name = CONTROL_MODE_TABS[tab_idx][0]
        self._on_rc_mode_selected(tab_idx, name)

    def _cb_sync_mode_buttons(self):
        """Mark the row matching the active mode; clear every other.

        Uses the ModeTab stylesheet's own active_j / active_c properties
        rather than :checked, so the bar's rows look exactly like the mode
        tabs everywhere else in the Studio.
        """
        active = getattr(self, '_rc_selected_mode_idx', 0)
        for kind, (_, indices) in MODE_MENUS.items():
            buttons = self._cb_mode_buttons.get(kind) or []
            for slot, tab_idx in enumerate(indices):
                if slot >= len(buttons):
                    continue
                btn = buttons[slot]
                on = (tab_idx == active)
                btn.setChecked(on)
                btn.setProperty('active_j', on and kind == 'joint')
                btn.setProperty('active_c', on and kind == 'cart')
                btn.style().unpolish(btn)
                btn.style().polish(btn)

    def _cb_place_surface(self):
        """Keep the shared control surface in step with the active mode.

        The surface itself never moves — it is shared by both menus — so
        this only has to make sure it reflects the mode now selected.
        """
        area = getattr(self, '_rc_control_area', None)
        if area is not None:
            area.setVisible(True)
        bar = getattr(self, '_control_bar', None)
        if bar is not None and bar.current() is not None:
            bar.open_panel(bar.current())

    # ── Profiles ──────────────────────────────────────────────────────────

    def _cb_on_profile_changed(self, kind, _index=0):
        """Mirror a profile choice into the other menu, then publish it."""
        if self._cb_syncing_profile:
            return
        source = self._cb_profile_combos.get(kind)
        if source is None:
            return
        self._cb_syncing_profile = True
        try:
            for other, combo in self._cb_profile_combos.items():
                if other != kind and combo.count() == source.count():
                    combo.setCurrentIndex(source.currentIndex())
        finally:
            self._cb_syncing_profile = False
        if hasattr(self, '_rc_note_profile_choice'):
            self._rc_note_profile_choice()
        if hasattr(self, '_publish_rc_profile'):
            self._publish_rc_profile()

    def _cb_mirror_profiles(self):
        """Copy the canonical profile list into every menu's Config combo."""
        source = self._cb_profile_combos.get('joint')
        if source is None:
            return
        self._cb_syncing_profile = True
        try:
            for kind, combo in self._cb_profile_combos.items():
                if combo is source:
                    continue
                combo.clear()
                for i in range(source.count()):
                    combo.addItem(source.itemText(i), source.itemData(i))
                combo.setCurrentIndex(source.currentIndex())
        finally:
            self._cb_syncing_profile = False

    # ── Sequences (delegated to the extension) ────────────────────────────

    def _cb_sequences(self):
        """Return the loaded sequences extension, or None."""
        getter = getattr(self, 'get_extension', None)
        return getter('sequences') if getter is not None else None

    def _cb_refresh_sequences(self, select=None):
        """Repopulate the sequence dropdown from the extension, keeping the chosen one (or
        choosing ``select``). The bar is built before the extensions load their sequences, so it
        also runs whenever Quick Actions or Snapshot opens."""
        combo = getattr(self, '_cb_sequence_combo', None)
        if combo is None:
            return
        ext = self._cb_sequences()
        keep = select or combo.currentText()
        if ext is None:
            combo.clear()
            return
        try:
            names = list(ext.list_sequences())
        except Exception as exc:                                # pragma: no cover
            self._cb_log('WARN', 'Could not list sequences: %s' % exc)
            return
        if [combo.itemText(i) for i in range(combo.count())] != names:
            combo.clear()
            combo.addItems(names)
        if keep in names:
            combo.setCurrentText(keep)
        speed = getattr(self, '_cb_speed', None)
        if speed is not None and hasattr(ext, 'speed'):
            speed.blockSignals(True)
            speed.setValue(ext.speed())
            speed.blockSignals(False)

    def _cb_selected_sequence(self):
        """Return the chosen sequence name, or None."""
        combo = getattr(self, '_cb_sequence_combo', None)
        if combo is None or not combo.count():
            return None
        return combo.currentText()

    def _cb_play_sequence(self, loop=False):
        """Play the selected sequence through the extension."""
        ext = self._cb_sequences()
        name = self._cb_selected_sequence()
        if ext is None or name is None:
            return
        try:
            ext.play(name, loop=loop)
        except Exception as exc:                                # pragma: no cover
            self._cb_log('ERROR', 'Could not play "%s": %s' % (name, exc))

    def _cb_loop_sequence(self):
        """Play the selected sequence on repeat."""
        self._cb_play_sequence(loop=True)

    def _cb_stop_sequence(self):
        """Stop sequence playback."""
        ext = self._cb_sequences()
        if ext is None:
            return
        try:
            ext.stop()
        except Exception as exc:                                # pragma: no cover
            self._cb_log('ERROR', 'Could not stop sequence: %s' % exc)

    def _cb_capture_snapshot(self):
        """Append the live pose to the selected sequence as a waypoint."""
        ext = self._cb_sequences()
        name = self._cb_selected_sequence()
        if ext is None or name is None:
            return
        kind = self._cb_snapshot_kind.currentText()
        try:
            ext.capture(kind, into=name)
        except Exception as exc:                                # pragma: no cover
            self._cb_log('ERROR', 'Could not capture waypoint: %s' % exc)

    # ── Direct actions ────────────────────────────────────────────────────

    def _cb_on_home(self):
        """Send the home pose, asking first if a sequence is playing."""
        ext = self._cb_sequences()
        if ext is not None and getattr(ext, 'is_playing', False):
            from PyQt5.QtWidgets import QMessageBox
            reply = QMessageBox.question(
                self._control_bar, 'Sequence running',
                'A sequence is playing. Home anyway?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply != QMessageBox.Yes:
                return
        self._rc_on_home()

    def _cb_on_contact_twin(self):
        """Toggle the MuJoCo contact twin viewer."""
        if getattr(self, 'contact_twin_running', False):
            self._stop_contact_twin()
        else:
            self._start_contact_twin()

    def _cb_on_calibrate(self):
        """Call the contact node's calibration service for the active robot."""
        rid = self._rc_active_rid
        client = self.node.create_client(Trigger, f'{rid}/contact/calibrate')
        if not client.wait_for_service(timeout_sec=1.0):
            self._cb_log('WARN', 'contact node is not running')
            return
        client.call_async(Trigger.Request())
        self._cb_log('INFO', 'calibrating — do not touch the arm')

    @staticmethod
    def _cb_contact_node_name(rid):
        """Map a robot id ('robot1') to its contact node's ROS graph name.

        The calibrate/record services above are named directly off `rid`
        (e.g. 'robot1/contact/calibrate') and never need the contact
        node's own ROS name at all. The ground-truth combo below cannot
        take that shortcut -- it has to reach the node's *built-in*
        parameter-set service, which ROS always names off the node's own
        graph name. `fic_contact/launch/contact_launch.py` renames each
        station's node to `contact_localization_<station_id>`, the
        inverse of `ContactNode`'s own `self._rid = 'robot%s' % sid`, so
        this reverses that same substitution.
        """
        sid = rid[len('robot'):] if rid.startswith('robot') else rid
        return 'contact_localization_%s' % sid

    def _cb_on_ground_truth_changed(self, text):
        """Set the active robot's declared ground-truth link on its node.

        Sets the `link_truth` parameter via the contact node's standard
        ROS `set_parameters` service -- never by reaching into the node
        object directly, which was never actually an option once the
        Studio and the contact node are separate processes. `'none'`
        clears the declared truth back to "no ground truth declared",
        kept distinct from the operator declaring "no contact" (link 0,
        which this combo never offers) -- see
        `ContactNode.set_link_truth`.
        """
        node_name = self._cb_contact_node_name(self._rc_active_rid)
        client = self.node.create_client(
            SetParameters, '%s/set_parameters' % node_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self._cb_log('WARN', 'contact node is not running')
            return
        request = SetParameters.Request(parameters=[Parameter(
            name='link_truth',
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=text))])
        client.call_async(request)
        self._cb_log('INFO', 'ground truth -> %s' % text)

    # ── Availability + status ─────────────────────────────────────────────

    def _cb_apply_availability(self):
        """Enable or hide bar buttons according to robot and extensions."""
        bar = getattr(self, '_control_bar', None)
        if bar is None:
            return
        cfg = {}
        if hasattr(self, 'robot_configs') and hasattr(self, 'robot_dropdown'):
            cfg = self.robot_configs.get(self.robot_dropdown.currentText(), {})

        legged = cfg.get('robot_category') == 'legged'
        bar.set_enabled('cart', not legged, NO_CARTESIAN)

        gripper_btn = bar.button('gripper')
        if gripper_btn is not None:
            gripper_btn.setVisible(bool(cfg.get('has_gripper', True)))

        has_sequences = self._cb_sequences() is not None
        bar.set_enabled('quick', has_sequences, NO_SEQUENCES)
        bar.set_enabled('snapshot', has_sequences, NO_SEQUENCES)

        # Target mapping. Simulation and Hardware both command locally, so
        # the menus stay live; VR hands driving to the headset, so the
        # command surface locks the same way the Teleop checkbox locks it.
        target = getattr(self, 'control_target', 'simulation')
        vr = (target == 'vr')
        for key in ('joint', 'cart', 'gripper', 'home'):
            if key == 'cart' and legged:
                continue
            bar.set_enabled(key, not vr, VR_DRIVEN)
        area = getattr(self, '_rc_control_area', None)
        if area is not None:
            area.setEnabled(not vr)

        # The live contact readout is only meaningful while driving real
        # every target, matching the viewer launch button in the top
        # toolbar (main_window_pro._refresh_toolbar_launch). Both were
        # briefly hardware-only, but control_target defaults to
        # 'simulation', so a fresh launch hid them and the feature looked
        # missing rather than inapplicable. The readout's own four states
        # ('--' no data, UNCAL, 'no contact', link name) already say
        # whether an estimate is available, so gating on target as well
        # only removed information.
        clbl = getattr(self, '_cb_contact_lbl', None)
        if clbl is not None:
            clbl.setVisible(True)

    def _cb_refresh_status(self):
        """Update the always-visible mode / rate / target readout."""
        lbl = getattr(self, '_cb_status_lbl', None)
        if lbl is None:
            return
        idx = getattr(self, '_rc_selected_mode_idx', 0)
        toggle = getattr(self, '_rc_hpmp_toggle', None)
        mode = mode_display_name(idx, bool(toggle is not None and toggle.isChecked()))
        rid = self.node.station_robot_id if hasattr(self, 'node') else 'robot1'
        target = self.robot_target(rid) if hasattr(self, 'robot_target') else '--'
        rate, age_ms, health = 0.0, None, 'offline'
        if hasattr(self, 'telemetry_health'):
            rate, age_ms, health = self.telemetry_health(rid)

        ext = self._cb_sequences()
        playing = ext is not None and getattr(ext, 'is_playing', False)

        labels = {'simulation': 'SIM', 'hardware': 'HW', 'vr': 'VR'}
        parts = [mode, labels.get(target, target)]
        if health == 'offline':
            parts.append('offline')
        else:
            parts.append('%.0f Hz' % rate)
            if health == 'stale':
                parts.append('stale %.0f ms' % age_ms)
        glyph = '▶' if playing else '●'
        lbl.setText('%s  %s' % (glyph, ' · '.join(parts)))

        token = {'live': 'accent_green', 'stale': 'accent_orange'}.get(
            health, 'text_disabled')
        if playing:
            token = 'accent_teal'
        t = theme.current()
        lbl.setStyleSheet(
            'background: transparent; border: none; font-size: 11px;'
            " font-family: 'JetBrains Mono', 'Consolas', monospace;"
            ' color: %s;' % t[token])

        self._cb_refresh_contact_readout(rid, t)

    def _cb_refresh_contact_readout(self, rid, t):
        """Update the contact-link readout, distinguishing all four states.

        ``link_index`` has three regimes and "no message at all" is a
        fourth — absence must never look like a real measurement:

        * no {rid}_contact_estimate message has ever arrived  -> ``--``
        * ``link_index == -1`` (uncalibrated / gated off)      -> ``UNCAL``
        * ``link_index == 0`` (a genuine "no contact" reading) -> ``no contact``
        * ``link_index in 1..n``                                -> the link name
        """
        clbl = getattr(self, '_cb_contact_lbl', None)
        if clbl is None:
            return
        active_rid = getattr(self, '_rc_active_rid', rid)
        est = self.contact_estimate_for(active_rid) if hasattr(
            self, 'contact_estimate_for') else None
        if est is None:
            clbl.setText('--')
            ctoken = 'text_disabled'
        elif est['link_index'] < 0:
            clbl.setText('UNCAL')
            ctoken = 'accent_orange'
        elif est['link_index'] == 0:
            clbl.setText('no contact')
            ctoken = 'accent_green'
        else:
            clbl.setText('%s %d%%' % (
                est['link_name'].replace('_Link', ''),
                round(est['confidence'] * 100)))
            ctoken = 'accent_red'
        clbl.setStyleSheet(
            'background: transparent; font-size: 11px;'
            " font-family: 'JetBrains Mono', 'Consolas', monospace;"
            ' color: %s;' % t[ctoken])

    def _cb_log(self, level, message):
        """Log through the host when it can, else stay silent."""
        if hasattr(self, 'log_message'):
            self.log_message(level, message, 'Control')
