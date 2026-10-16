# mixins/robot_control_mixin.py — Unified simulation + hardware robot control
# Merges the former simulation_mixin.py and control_mixin.py into a single page
# with a Simulation / Hardware toggle.

import os
import math

from fic_studio import theme
from fic_studio.pages import page_showing
from fic_studio.fleet.stations import station_label
from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QBrush, QPen
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit,
    QComboBox, QCheckBox, QGroupBox, QGridLayout, QScrollArea, QFrame,
    QSlider, QSizePolicy, QMessageBox, QFormLayout, QDialog, QDoubleSpinBox,
)

#: Desired-wrench fields: (label, unit, limit, step). Forces in N, moments in N m.
DESIRED_WRENCH_AXES = (
    ('Fx', 'N', 200.0, 0.5), ('Fy', 'N', 200.0, 0.5), ('Fz', 'N', 200.0, 0.5),
    ('Mx', 'N m', 50.0, 0.05), ('My', 'N m', 50.0, 0.05), ('Mz', 'N m', 50.0, 0.05),
)


# ═══════════════════════════════════════════════════════════════════════════════
#  Virtual Joystick Widget
# ═══════════════════════════════════════════════════════════════════════════════

class VirtualJoystick(QWidget):
    """Virtual joystick widget that emits (dx, dy) values from -1.0 to 1.0."""
    joystickMoved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(100, 100)
        self.is_grabbed = False
        self.max_radius = 40
        self.stick_radius = 15
        self.stick_offset = QPointF(0, 0)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.emit_position)
        self.timer.start(50)
        theme.signals.theme_changed.connect(self.update)

    def paintEvent(self, event):
        from fic_studio import theme
        t = theme.current()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = QPointF(self.width() / 2, self.height() / 2)
        painter.setBrush(QBrush(QColor(t['bg_base'])))
        painter.setPen(QPen(QColor(t['border_subtle']), 2))
        painter.drawEllipse(center, self.max_radius, self.max_radius)
        stick_color = (QColor(t['accent_teal']) if self.is_grabbed
                       else QColor(t['border_medium']))
        painter.setBrush(QBrush(stick_color))
        painter.setPen(Qt.NoPen)
        stick_pos = center + self.stick_offset
        painter.drawEllipse(stick_pos, self.stick_radius, self.stick_radius)

    def mousePressEvent(self, event):
        center = QPointF(self.width() / 2, self.height() / 2)
        if (event.pos() - center).manhattanLength() < self.max_radius:
            self.is_grabbed = True
            self.update_stick(event.pos(), center)

    def mouseMoveEvent(self, event):
        if self.is_grabbed:
            center = QPointF(self.width() / 2, self.height() / 2)
            self.update_stick(event.pos(), center)

    def mouseReleaseEvent(self, event):
        self.is_grabbed = False
        self.stick_offset = QPointF(0, 0)
        self.update()
        self.joystickMoved.emit(0.0, 0.0)

    def update_stick(self, pos, center):
        vec = pos - center
        dist = math.hypot(vec.x(), vec.y())
        if dist > self.max_radius:
            vec.setX(vec.x() * self.max_radius / dist)
            vec.setY(vec.y() * self.max_radius / dist)
        self.stick_offset = vec
        self.update()

    def emit_position(self):
        if not self.is_grabbed:
            return
        dx = self.stick_offset.x() / self.max_radius
        dy = -self.stick_offset.y() / self.max_radius
        if abs(dx) < 0.05: dx = 0.0
        if abs(dy) < 0.05: dy = 0.0
        if dx != 0.0 or dy != 0.0:
            self.joystickMoved.emit(dx, dy)


# ═══════════════════════════════════════════════════════════════════════════════
#  Mode definitions
# ═══════════════════════════════════════════════════════════════════════════════

CONTROL_MODE_TABS = [
    ("Joint Imp",     "joint"),   # Mode 1
    ("Joint Planner", "joint"),   # Mode 2
    ("Joint Adm",     "joint"),   # Mode 3
    ("Joint HAP-FIC", "joint"),   # Mode 4
    ("Cart Imp",      "cart"),    # Mode 5
    ("Cart Planner",  "cart"),    # Mode 6
    ("Cart Adm",      "cart"),    # Mode 7
    ("Cart HAP-FIC",  "cart"),    # Mode 8
]

# Tooltip text shown when hovering a mode tab — aligned with the Control Modes
# docs page. Keep these one sentence so the hover stays readable.
CONTROL_MODE_TOOLTIPS = [
    "Mode 1 — Joint Impedance. Pure position stiffness, no velocity damping. "
    "Fast response, can oscillate if K0 is high. Use after you've validated tuning.",
    "Mode 2 — Joint Impedance + Planner. Smooth ramp between targets with damping. "
    "Default mode on startup; safest for first motions.",
    "Mode 3 — Joint Admittance. Robot yields to external forces in joint space.",
    "Mode 4 — Joint HAP-FIC. Joint admittance bounded by each joint's FIC law: yields to a push, returns on release.",
    "Mode 5 — Cartesian Impedance. Spring-anchored end-effector pose, no damping.",
    "Mode 6 — Cartesian Impedance with Planner. Smooth EE ramp with damping. Good precision-positioning default.",
    "Mode 7 — Cartesian Admittance. EE yields to measured wrench. "
    "Postural null-space is gated out in this mode; Elbow still works.",
    "Mode 8 — Cartesian HAP-FIC. Cartesian admittance bounded by the FIC law (HapFIC). "
    "Postural null-space gated out; Elbow still works.",
]

# The modes' internal names: the hidden mode dropdown, extensions' mode strings and logs key on
# these, so they stay as they are. What a user reads is CONTROL_MODE_DISPLAY (mode_display_name).
CONTROL_MODE_NAMES = [
    "Joint Impedance",
    "Joint Impedance With Planner",
    "Joint Admittance",
    "Joint Admittance Haptic",
    "Cartesian Impedance",
    "Cartesian Planner",
    "Cartesian Admittance",
    "Cartesian Admittance Haptic",
]

# The modes as the Studio shows them (the Whole-Arm paper's names). HAP-FIC is mode 4 (joints) and
# mode 8 (Cartesian); H-PMP is not a mode but the toggle that nests a Cartesian mode (5-8) in the
# joint HAP-FIC.
CONTROL_MODE_DISPLAY = [
    "Joint Impedance",
    "Joint Impedance with Planner",
    "Joint Admittance",
    "Joint HAP-FIC",
    "Cartesian Impedance",
    "Cartesian Impedance with Planner",
    "Cartesian Admittance",
    "Cartesian HAP-FIC",
]


def mode_display_name(idx, hpmp=False):
    """What the Studio calls mode idx (0-based), with the H-PMP toggle on or off. H-PMP acts on the
    Cartesian modes only: on Cartesian HAP-FIC (mode 8) it is H-PMP itself, the controller of the
    Whole-Arm experiments; on modes 5-7 that mode nested in the joint HAP-FIC."""
    if not 0 <= idx < len(CONTROL_MODE_DISPLAY):
        return '--'
    if hpmp and idx == 7:
        return 'H-PMP'
    if hpmp and idx >= 4:
        return CONTROL_MODE_DISPLAY[idx] + ' + H-PMP'
    return CONTROL_MODE_DISPLAY[idx]


# ═══════════════════════════════════════════════════════════════════════════════
#  Unified Robot Control Mixin
# ═══════════════════════════════════════════════════════════════════════════════

class RobotControlMixin:
    """Unified mixin for simulation and hardware robot control.

    Provides a single page with a Simulation / Hardware toggle.
    Shared controls: mode buttons, joint sliders, Cartesian pose + joysticks.
    Hardware-only: gripper, pause/stop, multi-robot cards.
    Simulation-only: preset poses.
    """

    #: Skip periodic work while this page is hidden.
    _page_showing = page_showing

    # ──────────────────────────────────────────────────────────────────────
    #  Page builder
    # ──────────────────────────────────────────────────────────────────────

    def _build_multi_robot_page(self) -> QScrollArea:
        """Build and return the Robot Control page as a QScrollArea.

        Layout: header (title + robot selector), target toggle,
        mode-button row, adaptive control area, gripper group
        (gated on has_gripper), a CollapsibleSection('Advanced') that
        holds null-space controls + config + preset row + deg/rad toggle
        + joint sliders + joysticks, and an action bar (Home, Reset,
        Live-publish checkbox, Pause/Stop hw-only, Send command).
        """
        self._robot_targets: dict[str, str] = {}
        self.control_target = 'simulation'

        # ── active robot for the control area ──
        self._rc_active_rid: str = self.node.station_robot_id
        # Field stores initialised on first _rebuild_rc_control_area call
        self._rc_joint_fields: list = []
        self._rc_pose_fields: dict = {}

        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(12)
        layout.setAlignment(Qt.AlignTop)

        # ── Header: title + robot selector (right-aligned) ──
        title_row = QHBoxLayout()
        self._rc_title = self._page_title(
            'Multi-Robot — %s — Simulation' % station_label(self.node.station_id))
        title_row.addWidget(self._rc_title)
        title_row.addStretch()

        # Robot selector
        self._rc_robot_selector = QComboBox()
        self._rc_robot_selector.setToolTip(
            'Select the active robot for the control area. '
            'Choose "＋ Add robot…" to add a new station.')
        self._rc_rebuild_robot_selector()
        self._rc_robot_selector.currentTextChanged.connect(self._rc_on_robot_selected)
        title_row.addWidget(self._rc_robot_selector)

        layout.addLayout(title_row)

        # ── Fleet + Teleoperation (fleet_mixin): every Studio on the network ──
        if hasattr(self, '_build_fleet_panel'):
            layout.addWidget(self._build_fleet_panel())
            _control_title = QLabel('Control — %s' % station_label(self.node.station_id))
            _control_title.setObjectName('SectionLabel')
            layout.addWidget(_control_title)

        # ── Target toggle (Simulation / Hardware / VR) ──
        toggle_row = QHBoxLayout()
        toggle_frame = QFrame()
        theme.bind(toggle_frame,
                   lambda t: 'border:1px solid %s; border-radius:6px;' % t['border_subtle'])
        toggle_inner = QHBoxLayout(toggle_frame)
        toggle_inner.setContentsMargins(0, 0, 0, 0)
        toggle_inner.setSpacing(0)

        self._rc_sim_toggle = QPushButton('Simulation')
        self._rc_sim_toggle.setObjectName('UnitBtn')
        self._rc_sim_toggle.setProperty('active', True)
        self._rc_sim_toggle.setCursor(Qt.PointingHandCursor)
        self._rc_sim_toggle.clicked.connect(lambda: self._set_control_target('simulation'))

        self._rc_hw_toggle = QPushButton('Hardware')
        self._rc_hw_toggle.setObjectName('UnitBtn')
        self._rc_hw_toggle.setProperty('active', False)
        self._rc_hw_toggle.setCursor(Qt.PointingHandCursor)
        self._rc_hw_toggle.clicked.connect(lambda: self._set_control_target('hardware'))

        self._rc_vr_toggle = QPushButton('VR')
        self._rc_vr_toggle.setObjectName('UnitBtn')
        self._rc_vr_toggle.setProperty('active', False)
        self._rc_vr_toggle.setCursor(Qt.PointingHandCursor)
        self._rc_vr_toggle.clicked.connect(lambda: self._set_control_target('vr'))

        toggle_inner.addWidget(self._rc_sim_toggle)
        toggle_inner.addWidget(self._rc_hw_toggle)
        toggle_inner.addWidget(self._rc_vr_toggle)
        toggle_row.addWidget(toggle_frame)
        toggle_row.addStretch()
        layout.addLayout(toggle_row)

        # Mode selection, the modifiers (Postural / Elbow / Hpmp /
        # Admittance planner) and the config profile selector all live on the
        # global control bar now — see control_bar_mixin. Only the selection
        # state is seeded here, because the publish handlers below read it.
        self._rc_selected_mode_idx = getattr(self, '_rc_selected_mode_idx', 0)
        self._rc_selected_kind = getattr(self, '_rc_selected_kind', 'joint')

        # Preset poses (Cartesian, simulation only)
        self._rc_preset_row = self._build_rc_preset_row()
        layout.addWidget(self._rc_preset_row)

        # Legacy control widgets (custom pose + joysticks, joint sliders,
        # send-pose) are retained OFF-SCREEN so existing Home/Reset/publish
        # paths keep working, but are no longer shown — the adaptive control
        # area replaces them.
        self._rc_legacy_holder = QWidget()
        self._rc_legacy_holder.setVisible(False)
        _legacy_l = QVBoxLayout(self._rc_legacy_holder)
        _legacy_l.setContentsMargins(0, 0, 0, 0)
        self._rc_custom_group = self._build_rc_custom_pose_group()
        _legacy_l.addWidget(self._rc_custom_group)
        self._rc_joint_group = self._build_rc_joint_group()
        _legacy_l.addWidget(self._rc_joint_group)
        self._rc_pose_row = QWidget()
        pose_row_l = QHBoxLayout(self._rc_pose_row)
        pose_row_l.setContentsMargins(0, 0, 0, 0)
        pose_row_l.addStretch()
        send_pose_btn = self._btn('SEND POSE / RESUME', 'BtnTeal')
        send_pose_btn.clicked.connect(self._publish_rc_pose)
        pose_row_l.addWidget(send_pose_btn)
        _legacy_l.addWidget(self._rc_pose_row)
        layout.addWidget(self._rc_legacy_holder)

        # ── Action bar ──
        action_bar = QHBoxLayout()
        action_bar.setSpacing(8)

        _home_btn = self._btn('Home')
        _home_btn.setToolTip('Send the robot to its home joint configuration.')
        _home_btn.clicked.connect(self._rc_on_home)
        action_bar.addWidget(_home_btn)

        _reset_btn = self._btn('Reset')
        _reset_btn.setToolTip('Reset all control fields to their default values.')
        _reset_btn.clicked.connect(self._reset_rc_defaults)
        action_bar.addWidget(_reset_btn)

        action_bar.addStretch()

        self._rc_live_publish = QCheckBox('Live publish')
        self._rc_live_publish.setChecked(False)
        self._rc_live_publish.setToolTip(
            'When checked, every field edit streams a command immediately.\n'
            'When unchecked, only the Send command button publishes.')
        action_bar.addWidget(self._rc_live_publish)

        # Hardware-only pause/stop (visibility managed by _update_target_visibility)
        self._rc_pause_btn = self._btn('PAUSE', 'BtnWarn')
        self._rc_pause_btn.clicked.connect(self.pause_robot_1)
        action_bar.addWidget(self._rc_pause_btn)

        self._rc_stop_btn = self._btn('STOP', 'BtnStop')
        self._rc_stop_btn.clicked.connect(self.stop_robot_1)
        action_bar.addWidget(self._rc_stop_btn)

        _send_btn = self._btn('▶  Send command', 'BtnPrimary')
        _send_btn.setToolTip(
            'Publish the current joint or Cartesian target to the active robot.')
        _send_btn.clicked.connect(self._rc_on_send_command)
        action_bar.addWidget(_send_btn)

        layout.addLayout(action_bar)

        # ── Multi-robot cards: kept as no-op (not built, not displayed) ──
        # _add_robot_card() / _remove_robot_card() still work for external
        # callers; the _add_robot_btn is created hidden so
        # _update_target_visibility can toggle it without AttributeError.
        self._robot_cards = []
        self._robot_card_count = 0
        self._add_robot_btn = QPushButton('＋  Add Robot')
        self._add_robot_btn.setObjectName('BtnPrimary')
        self._add_robot_btn.setCursor(Qt.PointingHandCursor)
        self._add_robot_btn.setFixedWidth(180)
        self._add_robot_btn.clicked.connect(self._add_robot_card)
        self._add_robot_btn.setVisible(False)

        layout.addStretch()

        # ── Initial visibility ──
        self._update_rc_section_visibility('joint')
        self._update_target_visibility()

        # ── start the ~10 Hz actual-state refresh timer ──
        self._start_rc_refresh_timer()

        # ── Initial profile list (current robot, mode 1) ──
        self._refresh_rc_profile_dropdown()

        # ── Robot dropdown change ──
        self.robot_dropdown.currentTextChanged.connect(self._on_rc_robot_changed)

        # ── Register primary widgets as robot_1 for publisher_mixin ──
        self._register_robot_1_widgets()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(page)
        return scroll

    def _rc_on_home(self):
        """Send the active robot to its home joint configuration.

        In a joint mode this commands q_home_deg; in a Cartesian mode it
        stays in Cartesian and commands default_ee_target instead.

        Resets the joint targets only. It used to call _reset_rc_defaults,
        which also forced the control mode back to Joint Impedance — going
        home is not a reason to change how the robot is being controlled,
        and that call additionally reached for the mode-tab strip that the
        control bar replaced, so pressing Home raised inside a Qt slot and
        never got as far as setting the commanded values.
        """
        if getattr(self, '_rc_selected_kind', 'joint') == 'cart':
            # Cartesian mode: stay in Cartesian and send the home pose from
            # default_ee_target. Publishing joint angles here would command
            # the robot in a different space from the one it is running in.
            self._rc_home_cartesian()
            return

        self._reset_rc_joint_defaults()
        fields = getattr(self, '_rc_joint_fields', [])
        if fields:
            meta = getattr(self, '_rc_joint_meta', None)
            if meta and len(meta) == len(fields):
                # Meta-aware: set home only on actuated fields, by actuated index
                # (view order may differ from actuated order for parallel robots).
                robot = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
                home_deg = getattr(self, 'robot_configs', {}).get(robot, {}).get('q_home_deg', [])
                for m, field in zip(meta, fields):
                    ci = m.get('cmd_index')
                    if m.get('actuated') and ci is not None and ci < len(home_deg):
                        field.set_commanded(float(home_deg[ci]))
            else:
                rads = self._gather_joint_angles_radians()
                for i, field in enumerate(fields):
                    if i < len(rads):
                        field.set_commanded(math.degrees(rads[i]))
            self._rc_publish_joints()
        else:
            self._publish_rc_joint_angles()
        self.log_message('INFO', 'Home position commanded', 'Control')

    def _rc_on_send_command(self):
        """Publish the active kind (joints or Cartesian pose) to the robot.

        Prefers the adaptive MetricField rows if populated;
        falls back to the legacy slider/input publishers for the Advanced
        custom-pose / joint-slider area.
        """
        kind = getattr(self, '_rc_selected_kind', 'joint')
        if kind == 'joint':
            if getattr(self, '_rc_joint_fields', []):
                self._rc_publish_joints()
            else:
                self._publish_rc_joint_angles()
        else:
            if getattr(self, '_rc_pose_fields', {}):
                self._rc_publish_pose_from_fields()
            else:
                self._publish_rc_pose()

    # ──────────────────────────────────────────────────────────────────────
    #  Target toggle
    # ──────────────────────────────────────────────────────────────────────

    def robot_target(self, rid: str = None) -> str:
        """Return 'simulation', 'hardware', or 'vr' for the given robot ID.

        Falls back to the global toggle for robots without a per-robot
        override. This is the single source of truth — all code that
        previously read self.control_target should call this instead.
        """
        if rid is None:
            rid = self.node.station_robot_id
        return self._robot_targets.get(rid, self.control_target)

    def set_robot_target(self, rid: str, target: str) -> None:
        """Set a per-robot sim/hardware/vr target. Updates UI as needed."""
        if target not in ("simulation", "hardware", "vr"):
            return
        self._robot_targets[rid] = target
        self.log_message("INFO", f"{rid} target → {target}", "Control")

    def _set_control_target(self, target: str):
        if target == self.control_target:
            return
        self.control_target = target

        self._rc_sim_toggle.setProperty("active", target == "simulation")
        self._rc_hw_toggle.setProperty("active", target == "hardware")
        self._rc_vr_toggle.setProperty("active", target == "vr")
        for btn in (self._rc_sim_toggle, self._rc_hw_toggle, self._rc_vr_toggle):
            btn.style().unpolish(btn)
            btn.style().polish(btn)

        labels = {"simulation": "Simulation", "hardware": "Hardware", "vr": "VR"}
        label = labels.get(target, target)
        self._rc_title.setText(
            f"Multi-Robot — {station_label(self.node.station_id)} — {label}")
        self._update_target_visibility()
        self.log_message("INFO", f"Target switched to {label}", "Control")
        if hasattr(self, '_refresh_toolbar_launch'):
            self._refresh_toolbar_launch()
        # The config follows the target ('sim' in simulation) until the user
        # picks one.
        if getattr(self, '_rc_profile_choice', None) is None:
            self._refresh_rc_profile_dropdown()
            self._publish_rc_profile()

    def _update_target_visibility(self):
        """Refresh widget visibility after a target or mode change."""
        # Keep the control bar in step: it gates its command surface on the
        # target and prints the target in its status readout.
        for hook in ('_cb_apply_availability', '_cb_refresh_status'):
            fn = getattr(self, hook, None)
            if fn is not None:
                fn()
        if not hasattr(self, '_rc_preset_row'):
            return
        is_sim = (self.control_target == 'simulation')
        is_hw = (self.control_target == 'hardware')
        is_cart = (self._rc_selected_kind == 'cart')

        # Simulation-only: preset poses (inside Advanced)
        self._rc_preset_row.setVisible(is_sim and is_cart)

        # Hardware-only: pause/stop in action bar + send-pose row (inside Advanced)
        self._rc_pause_btn.setVisible(is_hw)
        self._rc_stop_btn.setVisible(is_hw)
        self._rc_pose_row.setVisible(is_hw and is_cart)

        # Gripper visibility is set once at build time from has_gripper; we
        # never hide it here (leave it to the build-time gate).

        # Extra robot cards (legacy path — normally empty in the new layout)
        for card in self._robot_cards:
            card.setVisible(True)
            n = card.property('robot_index')
            if n is not None:
                rid = 'robot_%d' % n
                mode_idx = getattr(self, '%s_selected_mode_idx' % rid, 0)
                self._update_extra_section_visibility(
                    rid, CONTROL_MODE_NAMES[mode_idx])

    # ──────────────────────────────────────────────────────────────────────
    #  Mode selection
    # ──────────────────────────────────────────────────────────────────────

    def _on_rc_mode_selected(self, mode_idx: int, mode_name: str):
        kind = "cart" if "Cart" in mode_name else "joint"
        self._rc_selected_mode_idx = mode_idx
        self._rc_selected_kind = kind
        page_tabs = getattr(self, "_rc_mode_buttons", None)
        if page_tabs and mode_idx < len(page_tabs):
            self._highlight_rc_mode_tab(page_tabs[mode_idx], kind)
        self._update_rc_section_visibility(kind)

        # Update hidden dropdown for publisher_mixin
        self._rc_mode_dropdown.setCurrentIndex(mode_idx)

        # Publish mode to the first active robot (card 1)
        self._send_hpmp_for_mode(mode_idx + 1)
        self.node.publish_mode(mode_idx + 1)

        # Profiles are robot-scoped — the list is stable — but the
        # "(fallback)" indicator depends on the mode, so refresh to
        # re-decorate. Selection is preserved via itemData.
        self._refresh_rc_profile_dropdown()

        # Re-publish active profile so the controller's next reload picks
        # the right tuning set for the new mode.
        self._publish_rc_profile()

        # Rebuild the adaptive control area for joint ↔ cart switch.
        self._rebuild_rc_control_area()

        self.log_message("INFO",
            f"Mode → {CONTROL_MODE_DISPLAY[mode_idx]} (Mode {mode_idx + 1}) / {self._rc_current_profile()}",
            "Control")

        # Keep the Robot ▸ Control Mode menu checkmark in sync.
        if hasattr(self, "_sync_mode_menu_check"):
            self._sync_mode_menu_check()

        # Resync the control bar: its mode rows, the shared control surface
        # (which follows joint/cart), and the always-visible status readout.
        for hook in ("_cb_sync_mode_buttons", "_cb_place_surface",
                     "_cb_refresh_status"):
            fn = getattr(self, hook, None)
            if fn is not None:
                fn()

    def update_mode_ui_only(self, mode_name: str, robot_id: str = None) -> None:
        """Sync the mode-selector UI to match a programmatic mode change.

        Does NOT publish anything — the caller is responsible for having
        already published the mode to fic_hardware.  Used by extensions that
        send their own mode_select messages (e.g. the teleop extension) so
        the UI reflects the active mode without triggering a second publish.
        """
        try:
            idx = CONTROL_MODE_NAMES.index(mode_name)
        except ValueError:
            return
        kind = "cart" if "Cart" in mode_name else "joint"
        rid = robot_id or self.node.station_robot_id
        n = int(rid[len("robot"):]) if rid.startswith("robot") else self.node.station_id
        first_n = self.node.station_id
        if n == first_n:
            self._rc_selected_mode_idx = idx
            self._rc_selected_kind = kind
            self._rc_mode_dropdown.setCurrentIndex(idx)
            # The control-bar GUI has no page tab strip, so this is
            # guarded exactly as _on_rc_mode_selected guards it. Unguarded,
            # it raised AttributeError there and took the caller down with
            # it -- the Following page's START, and teleop's mode sync.
            page_tabs = getattr(self, '_rc_mode_buttons', None)
            if page_tabs and idx < len(page_tabs):
                self._highlight_rc_mode_tab(page_tabs[idx], kind)
            # ...and it has mode rows of its own to mark instead.
            sync = getattr(self, '_cb_sync_mode_buttons', None)
            if callable(sync):
                sync()
            self._update_rc_section_visibility(kind)
        else:
            hidden_dd = getattr(self, f'{rid}_mode_dropdown', None)
            if hidden_dd is not None:
                hidden_dd.setCurrentIndex(idx)
            setattr(self, f'{rid}_selected_mode_idx', idx)
            buttons = getattr(self, f'{rid}_mode_buttons', [])
            if buttons and idx < len(buttons):
                self._highlight_extra_mode_tab(rid, buttons[idx], kind)
            self._update_extra_section_visibility(rid, CONTROL_MODE_NAMES[idx])

    # ──────────────────────────────────────────────────────────────────────
    #  Null-space toggle handlers
    # ──────────────────────────────────────────────────────────────────────
    def _on_rc_postural_toggled(self, checked: bool):
        self.node.publish_postural_enable(bool(checked))
        self.log_message("INFO", f"Postural {'ON' if checked else 'OFF'}", "Control")

    def _on_rc_elbow_toggled(self, checked: bool):
        self.node.publish_elbow_enable(bool(checked))
        self.log_message("INFO", f"Elbow {'ON' if checked else 'OFF'}", "Control")

    def _send_hpmp_for_mode(self, mode, robot_id=None, toggle=None):
        """Before a Cartesian mode (5-8) is sent, send the H-PMP toggle as it shows.

        The controllers start with H-PMP off and hear only the toggle messages sent after they
        start (the Studio's first one, at its own start, is missed), so a Cart press would
        otherwise run the plain Cartesian mode while the toggle reads H-PMP. A repeat changes
        nothing in the controller. ``toggle`` is a robot card's own checkbox (default card 1's).
        """
        if not 5 <= int(mode) <= 8:
            return
        if toggle is None:
            toggle = getattr(self, '_rc_hpmp_toggle', None)
        checked = getattr(toggle, 'isChecked', None)
        publish = getattr(self.node, 'publish_hpmp_enable', None)
        if callable(publish):
            publish(bool(checked()) if callable(checked) else True, robot_id=robot_id)

    def _on_rc_hpmp_toggled(self, checked: bool):
        self.node.publish_hpmp_enable(bool(checked))
        self.log_message("INFO", f"H-PMP {'ON' if checked else 'OFF'}", "Control")
        if hasattr(self, '_cb_refresh_status'):      # the mode readout names H-PMP when it is on
            self._cb_refresh_status()

    def _on_rc_admittance_planner_toggled(self, checked: bool):
        self._publish_admittance_planner(checked)
        self.log_message("INFO", f"Admittance planner {'ON' if checked else 'OFF'}", "Control")

    def _publish_admittance_planner(self, enabled: bool, robot_id=None):
        rid = robot_id or self.node.station_robot_id
        self.node.publish_admittance_planner(enabled, rid)

    # ──────────────────────────────────────────────────────────────────────
    #  Desired wrench of the Cartesian admittance (modes 7/8, H-PMP on them)
    # ──────────────────────────────────────────────────────────────────────
    def desired_wrench_for(self, robot_id=None):
        """Return the desired wrench last sent to a robot (zero if none was sent)."""
        rid = self.node._resolve_rid(robot_id)
        return list(getattr(self, '_desired_wrench_sent', {}).get(rid, [0.0] * 6))

    def _publish_desired_wrench(self, values, robot_id=None):
        """Send the admittance's desired wrench [Fx, Fy, Fz, Mx, My, Mz] (N, N m).

        The admittance of modes 7 and 8 (and of H-PMP on them) settles where the
        measured wrench equals this one; zero, the controller's start-up value, is
        the plain compliant behaviour.
        """
        values = [float(v) for v in values]
        if len(values) != 6 or not all(math.isfinite(v) for v in values):
            raise ValueError('a desired wrench is six finite numbers')
        rid = self.node._resolve_rid(robot_id)
        self.node.publish_desired_wrench(values, rid)
        if not hasattr(self, '_desired_wrench_sent'):
            self._desired_wrench_sent = {}
        self._desired_wrench_sent[rid] = values
        self.log_message('INFO', 'Desired wrench → F (%.2f, %.2f, %.2f) N, M (%.2f, %.2f, %.2f) N m'
                         % tuple(values), 'Control')
        dlg = getattr(self, '_desired_wrench_dialog', None)
        if dlg is not None and getattr(dlg, 'rid', None) == rid:
            dlg.show_values(values)

    def _open_desired_wrench_dialog(self):
        """Open the desired-wrench editor for the robot the control bar drives."""
        rid = self.node._resolve_rid(None)
        dlg = getattr(self, '_desired_wrench_dialog', None)
        if dlg is None or dlg.rid != rid:
            if dlg is not None:
                dlg.close()
            dlg = self._desired_wrench_dialog = DesiredWrenchDialog(self, rid)
        dlg.show_values(self.desired_wrench_for(rid))
        dlg.show()
        dlg.raise_()

    # ──────────────────────────────────────────────────────────────────────
    #  Adaptive control area + robot selector
    # ──────────────────────────────────────────────────────────────────────

    def _rc_command_rid(self) -> str:
        """Return the robot-id string used for publishing from the control area."""
        return self._rc_active_rid

    def _build_rc_control_area(self) -> QFrame:
        """Build and return the adaptive control-area frame.

        The frame holds a QVBoxLayout saved as ``self._rc_area_layout``.
        Delegates to ``_rebuild_rc_control_area()`` for the initial population.
        """
        frame = QFrame()
        frame.setObjectName('RcControlArea')
        theme.bind(
            frame,
            lambda t: (
                'QFrame#RcControlArea {'
                ' background: %s;'
                ' border: 1px solid %s;'
                ' border-left: 3px solid %s;'
                ' border-radius: 10px;'
                '}' % (t['bg_raised'], t['border_subtle'], t['accent_blue'])
            ),
        )
        self._rc_area_layout = QVBoxLayout(frame)
        self._rc_area_layout.setContentsMargins(16, 12, 16, 12)
        self._rc_area_layout.setSpacing(6)
        self._rebuild_rc_control_area()
        return frame

    def _rebuild_rc_control_area(self):
        """Clear and repopulate the adaptive control area for the active kind/robot.

        Joint kind: one ``CommandedVsActual`` row per joint, limits from config.
        Cartesian kind: X/Y/Z ``CommandedVsActual`` rows + axis-angle
        ``MetricField`` rows (axis X/Y/Z and angle).
        """
        from fic_studio.components.control_widgets import CommandedVsActual, MetricField
        from functools import partial as _partial

        layout = getattr(self, '_rc_area_layout', None)
        if layout is None:
            return

        # Clear existing widgets — hide + schedule deletion rather than
        # calling deleteLater() inside the takeAt loop, which can cause
        # the layout itself to be collected before the loop finishes.
        #
        # Recursive on purpose: the joint and Cartesian rows live in nested
        # QGridLayouts, and an item holding a layout returns None from
        # .widget(). Clearing only direct widgets left every previous row
        # parented to the frame but in no layout, so Qt kept painting them
        # at their old positions — rows visibly overlapping after a rebuild.
        def _clear(lay):
            while lay.count():
                item = lay.takeAt(0)
                w = item.widget()
                if w is not None:
                    w.hide()
                    w.setParent(None)
                    continue
                child = item.layout()
                if child is not None:
                    _clear(child)
                    child.deleteLater()

        _clear(layout)

        self._rc_joint_fields = []
        self._rc_pose_fields = {}

        kind = getattr(self, '_rc_selected_kind', 'joint')

        # Resolve robot config from the global dropdown (the active robot type)
        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})

        if kind == 'joint':
            import math
            # Display rows = every joint in view_joint_names (all hinges +
            # platform orientation for a parallel robot like CSPM), or J1..Jn
            # for a conventional robot. Only actuated joints are controllable;
            # the rest are shown greyed/view-only with live data.
            disp = self._rc_display_joints(robot_name, cfg)
            self._rc_joint_meta = disp
            home_deg = cfg.get('q_home_deg', [])
            rid = self.node.station_robot_id
            live_q = self.joint_positions_for(rid)
            sim_by_name = (self.sim_joint_by_name(rid)
                           if hasattr(self, 'sim_joint_by_name') else {})

            # Two per row, compact. Seven full-height rows stacked vertically
            # ran to ~384 px, which overflowed the control-bar panel and put
            # the lower joints out of reach.
            grid = QGridLayout()
            grid.setHorizontalSpacing(14)
            grid.setVerticalSpacing(4)
            grid.setContentsMargins(0, 0, 0, 0)
            grid.setColumnStretch(0, 1)
            grid.setColumnStretch(1, 1)
            layout.addLayout(grid)

            for i, dj in enumerate(disp):
                field = CommandedVsActual(
                    dj['label'], dj['lo'], dj['hi'], unit='°', decimals=1,
                    compact=True,
                )
                if dj['actuated']:
                    # committed → always publish; edited → gate on live_publish
                    field.committed.connect(_partial(self._rc_on_joint_committed, i))
                    field.edited.connect(_partial(self._rc_on_joint_edited, i))
                else:
                    field.setEnabled(False)  # passive/view-only: greyed, no command
                grid.addWidget(field, i // 2, i % 2)
                self._rc_joint_fields.append(field)
                # Pre-fill commanded (actuated joints only) from live q_current
                # by actuated index, else q_home_deg. Seed actual from the sim's
                # joint_states (by name) so view-only rows already show data.
                ci = dj['cmd_index']
                if dj['actuated'] and ci is not None:
                    if live_q and ci < len(live_q):
                        field.set_commanded(math.degrees(live_q[ci]))
                    elif ci < len(home_deg):
                        field.set_commanded(float(home_deg[ci]))
                sv = sim_by_name.get(dj['name'])
                if sv is not None:
                    sv_deg = math.degrees(sv[0])
                    field.set_actual(sv_deg)
                    if not dj['actuated']:
                        field.set_commanded(sv_deg)

        else:
            # Cartesian kind: X / Y / Z position rows
            _pos_specs = [
                ('X', -1.0, 1.0, 'm', 3),
                ('Y', -1.0, 1.0, 'm', 3),
                ('Z', -1.0, 1.0, 'm', 3),
            ]
            cart_grid = QGridLayout()
            cart_grid.setHorizontalSpacing(14)
            cart_grid.setVerticalSpacing(4)
            cart_grid.setContentsMargins(0, 0, 0, 0)
            cart_grid.setColumnStretch(0, 1)
            cart_grid.setColumnStretch(1, 1)
            layout.addLayout(cart_grid)

            # Position down the left column, orientation down the right, so
            # the two halves of a pose read as two things rather than one
            # seven-item list.
            for ci, (label, lo, hi, unit, dec) in enumerate(_pos_specs):
                field = CommandedVsActual(label, lo, hi, unit=unit,
                                          decimals=dec, compact=True)
                field.committed.connect(self._rc_on_pose_committed)
                field.edited.connect(self._rc_on_pose_edited)
                cart_grid.addWidget(field, ci, 0)
                self._rc_pose_fields[label] = field

            # Axis-angle orientation rows (command-only MetricFields)
            _aa_specs = [
                ('axis X', -1.0, 1.0, '', 2),
                ('axis Y', -1.0, 1.0, '', 2),
                ('axis Z', -1.0, 1.0, '', 2),
                ('angle', -180.0, 180.0, '°', 1),
            ]
            for ai, (label, lo, hi, unit, dec) in enumerate(_aa_specs):
                field = MetricField(label, 0.0, lo, hi, unit=unit,
                                    decimals=dec, compact=True)
                field.committed.connect(self._rc_on_pose_committed)
                field.edited.connect(self._rc_on_pose_edited)
                cart_grid.addWidget(field, ai, 1)
                self._rc_pose_fields[label] = field

            # Pre-fill commanded values with the robot's current EE pose so
            # the fields match where the robot actually is (the controller
            # snaps ee_target to ee_current on mode switch). Falls back to
            # default_ee_target from config if no live pose is available yet.
            rid = self.node.station_robot_id
            live_pose = self.ee_pose_for(rid)
            if live_pose is not None:
                pos = [live_pose.position.x, live_pose.position.y, live_pose.position.z]
                quat = [live_pose.orientation.x, live_pose.orientation.y,
                        live_pose.orientation.z, live_pose.orientation.w]
            else:
                ee = cfg.get('default_ee_target', {})
                pos = ee.get('position', [])
                quat = ee.get('orientation', [])
            self._rc_fill_pose_fields(pos, quat)

    def _rc_fill_pose_fields(self, pos, quat):
        """Write a position + quaternion into the Cartesian control fields.

        Orientation is entered as axis-angle, so the quaternion is converted
        and pushed into the canonical hemisphere first (angle <= 180 deg).
        """
        fields = getattr(self, '_rc_pose_fields', {})
        for j, lbl in enumerate(('X', 'Y', 'Z')):
            if j < len(pos) and lbl in fields:
                fields[lbl].set_commanded(float(pos[j]))
        if len(quat) != 4:
            return
        from fic_studio.math_helpers import _quat_to_axis_angle
        qx, qy, qz, qw = (float(quat[0]), float(quat[1]),
                          float(quat[2]), float(quat[3]))
        if qw < 0.0:                   # canonical hemisphere (angle <= 180 deg)
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        ax, ay, az, ang = _quat_to_axis_angle(qx, qy, qz, qw, 'deg')
        for lbl, val in (('axis X', ax), ('axis Y', ay),
                         ('axis Z', az), ('angle', ang)):
            if lbl in fields:
                fields[lbl].set_value(val)

    def _rc_home_cartesian(self):
        """Command the robot's home *pose* without leaving Cartesian mode."""
        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        ee = cfg.get('default_ee_target', {})
        pos = ee.get('position', [])
        quat = ee.get('orientation', [])
        if not pos:
            self.log_message(
                'WARN',
                'No default_ee_target for %s — nothing to home to' % robot_name,
                'Control')
            return
        self._rc_fill_pose_fields(pos, quat)
        if getattr(self, '_rc_pose_fields', {}):
            self._rc_publish_pose_from_fields()
        else:
            self._publish_rc_pose()
        self.log_message('INFO', 'Home pose commanded', 'Control')

    def _rc_rebuild_robot_selector(self):
        """Repopulate the robot selector combo with active robots + Add entry."""
        sel = getattr(self, '_rc_robot_selector', None)
        if sel is None:
            return
        sel.blockSignals(True)
        sel.clear()
        active = getattr(self.node, '_active_robots', [])
        for n in active:
            sel.addItem(station_label(n), n)
        sel.addItem('＋ Add robot…')
        # Select the entry matching _rc_active_rid
        rid = getattr(self, '_rc_active_rid', self.node.station_robot_id)
        try:
            n_current = int(rid.replace('robot', ''))
            idx = active.index(n_current)
            sel.setCurrentIndex(idx)
        except (ValueError, IndexError):
            sel.setCurrentIndex(0)
        sel.blockSignals(False)

    def _rc_on_robot_selected(self, text: str):
        """Handle robot selector change — switch active rid or add a new robot."""
        if not text:
            return
        add_entry = '＋ Add robot…'
        if text == add_entry:
            # Prompt for the next station number
            active = list(getattr(self.node, '_active_robots', []))
            next_n = (max(active) + 1) if active else 2
            try:
                from PyQt5.QtWidgets import QInputDialog
                n, ok = QInputDialog.getInt(
                    None,
                    'Add Robot',
                    'Station number:',
                    next_n, 1, 99,
                )
                if ok:
                    self.node.add_robot(n)
                    self._rc_active_rid = 'robot%d' % n
                    self._rc_rebuild_robot_selector()
                    self._rebuild_rc_control_area()
                    self.log_message('INFO', 'Added robot%d via selector' % n, 'Control')
            except Exception as exc:
                self.log_message('WARN', 'Add robot failed: %s' % exc, 'Control')
            # Reset the combo back to the current active robot entry
            self._rc_rebuild_robot_selector()
            return
        # The chosen entry's data is its station number (text: "Robot N" / "Master").
        sel = getattr(self, '_rc_robot_selector', None)
        idx = sel.findText(text) if sel is not None else -1
        n = sel.itemData(idx) if idx >= 0 else None
        if n is None:
            try:
                n = int(text.split()[-1])
            except (ValueError, IndexError):
                return
        self._rc_active_rid = 'robot%d' % n
        self._rc_rebuild_robot_selector()
        self._rebuild_rc_control_area()
        self.log_message('INFO', 'Active robot → %s' % self._rc_active_rid, 'Control')

    # ─────────────────────────────────────────────────────────────────────
    #  Joint field handlers (committed vs edited)
    # ─────────────────────────────────────────────────────────────────────

    def _rc_on_joint_field(self, joint_index: int, value: float = 0.0):
        """Backward-compat alias: treat as a committed signal from joint field.

        Retained so that existing test helpers calling the old stub signature
        continue to work without error.  Real wiring uses the split
        ``_rc_on_joint_committed`` / ``_rc_on_joint_edited`` handlers.
        """
        self._rc_on_joint_committed(joint_index, value)

    def _rc_on_joint_committed(self, joint_index: int, value: float = 0.0):
        """Called when a joint CommandedVsActual emits ``committed``.

        Publishes the current joint positions unconditionally (committed always
        publishes regardless of the Live-publish toggle).
        """
        self._rc_publish_joints()

    def _rc_on_joint_edited(self, joint_index: int, value: float = 0.0):
        """Called when a joint CommandedVsActual emits ``edited``.

        Publishes only when the Live-publish checkbox is checked.
        """
        lp = getattr(self, '_rc_live_publish', None)
        if lp is not None and lp.isChecked():
            self._rc_publish_joints()

    def _rc_publish_joints(self):
        """Gather joint-field values (deg → rad) and publish to the active robot.

        Routes to ``node.publish_joint_angles`` for simulation/VR or
        ``node.publish_real_joint_angles`` for hardware, using
        ``_rc_command_rid()`` as the target robot.
        """
        fields = getattr(self, '_rc_joint_fields', [])
        if not fields:
            return
        rid = self._rc_command_rid()
        # Publish ONLY the actuated joints, in actuated order, named with the
        # real joint names from the config (meta). For conventional robots the
        # meta is num_joints all-actuated 'J1..Jn' rows named '<robot>_jN', so
        # this reproduces the previous behaviour exactly.
        meta = getattr(self, '_rc_joint_meta', None)
        if meta and len(meta) == len(fields):
            pairs = [(m, f) for m, f in zip(meta, fields) if m.get('actuated')]
            pairs.sort(key=lambda mf: (mf[0].get('cmd_index') if mf[0].get('cmd_index') is not None else 0))
            rads = [math.radians(f.commanded()) for _, f in pairs]
            names = [m['name'] for m, _ in pairs]
        else:
            rads = [math.radians(f.commanded()) for f in fields]
            names = None
        target = getattr(self, 'control_target', 'simulation')
        if target in ('simulation', 'vr'):
            if names is None:
                robot_type = self.robot_dropdown.currentText()
                names = ['%s_j%d' % (robot_type, k + 1) for k in range(len(rads))]
            self.node.publish_joint_angles(rads, names=names, robot_id=rid)
        else:
            self.node.publish_real_joint_angles(rid, rads)
        degs = ', '.join(['%.1f°' % math.degrees(r) for r in rads])
        self.log_message('INFO', 'Joints → [%s]' % degs, 'Control')

    # ─────────────────────────────────────────────────────────────────────
    #  Pose field handlers (committed vs edited)
    # ─────────────────────────────────────────────────────────────────────

    def _rc_on_pose_field(self, value: float = 0.0):
        """Backward-compat alias: treat as a committed signal from a pose field.

        Retained so that existing test helpers calling the old stub signature
        continue to work without error.  Real wiring uses the split
        ``_rc_on_pose_committed`` / ``_rc_on_pose_edited`` handlers.
        """
        self._rc_on_pose_committed(value)

    def _rc_on_pose_committed(self, value: float = 0.0):
        """Called when a pose MetricField emits ``committed``.

        Publishes the full Cartesian pose unconditionally.
        """
        self._rc_publish_pose_from_fields()

    def _rc_on_pose_edited(self, value: float = 0.0):
        """Called when a pose MetricField emits ``edited``.

        Publishes only when the Live-publish checkbox is checked.
        """
        lp = getattr(self, '_rc_live_publish', None)
        if lp is not None and lp.isChecked():
            self._rc_publish_pose_from_fields()

    def _rc_publish_pose_from_fields(self):
        """Build a Pose from the Cartesian MetricFields and publish it.

        Uses axis-angle orientation from the 'axis X/Y/Z' and 'angle' fields;
        routes to ``node.publish_position`` (sim/VR) or ``send_pose_for``
        (hardware).
        """
        from geometry_msgs.msg import Pose, Point, Quaternion
        from fic_studio.math_helpers import _axis_angle_to_quat

        pose_fields = getattr(self, '_rc_pose_fields', {})
        if not pose_fields:
            return
        rid = self._rc_command_rid()
        try:
            x = pose_fields['X'].commanded()
            y = pose_fields['Y'].commanded()
            z = pose_fields['Z'].commanded()
            ax = pose_fields['axis X'].value()
            ay = pose_fields['axis Y'].value()
            az = pose_fields['axis Z'].value()
            angle = pose_fields['angle'].value()
        except KeyError:
            return
        qx, qy, qz, qw = _axis_angle_to_quat(ax, ay, az, angle, 'deg')
        pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )
        target = getattr(self, 'control_target', 'simulation')
        if target in ('simulation', 'vr'):
            self.node.publish_position(pose, robot_id=rid)
        else:
            # Pass the pose built from THIS panel's fields. Without it
            # send_pose_for re-derives one from the per-robot card
            # widgets, which sit at default_ee_target -- see its
            # docstring for the measurement.
            self.send_pose_for(rid, pose=pose)
        self.log_message(
            'INFO', 'Pose → (%.3f, %.3f, %.3f)' % (x, y, z), 'Control')

    # ─────────────────────────────────────────────────────────────────────
    #  ~10 Hz refresh — push actual state into the control fields
    # ─────────────────────────────────────────────────────────────────────

    def _start_rc_refresh_timer(self):
        """Start the ~10 Hz timer that refreshes actual-state overlays.

        Idempotent: a second call simply restarts the existing timer.
        Creates ``self._rc_refresh_timer`` if it does not yet exist.
        """
        if not hasattr(self, '_rc_refresh_timer'):
            self._rc_refresh_timer = QTimer(self if hasattr(self, 'timerEvent')
                                             else None)
            self._rc_refresh_timer.timeout.connect(self._rc_on_refresh_tick)
        self._rc_refresh_timer.start(100)   # 100 ms ≈ 10 Hz

    def _rc_on_refresh_tick(self):
        """Refresh actual-state overlays in the adaptive control area.

        Reads ``joint_positions_for(rid)`` (radians → degrees) and
        ``ee_pose_for(rid).position`` from the JointStateMixin and pushes
        the values into ``CommandedVsActual.set_actual``.  Returns early
        when the Robot Control page is not currently visible.
        """
        # Page-visible guard: skip when the main_stack is not on page 1.
        stack = getattr(self, 'main_stack', None)
        if not self._page_showing('multi_robot'):
            return

        rid = self._rc_command_rid()

        joint_fields = getattr(self, '_rc_joint_fields', [])
        if joint_fields:
            meta = getattr(self, '_rc_joint_meta', None)
            positions = []
            if hasattr(self, 'joint_positions_for'):
                positions = self.joint_positions_for(rid)
            sim_by_name = (self.sim_joint_by_name(rid)
                           if hasattr(self, 'sim_joint_by_name') else {})
            for i, field in enumerate(joint_fields):
                m = meta[i] if meta and i < len(meta) else None
                if m is not None:
                    # Prefer the sim's per-name joint_states (covers passive /
                    # view-only joints the controller's q_current omits).
                    sv = sim_by_name.get(m['name'])
                    if sv is not None:
                        val = math.degrees(sv[0])
                    else:
                        ci = m.get('cmd_index')
                        val = (math.degrees(positions[ci])
                               if ci is not None and ci < len(positions) else None)
                    if val is not None:
                        field.set_actual(val)
                        if not m.get('actuated'):
                            # Passive/view-only row: there is no command to show,
                            # so display the live value as the number too.
                            field.set_commanded(val)
                elif i < len(positions):
                    field.set_actual(math.degrees(positions[i]))

        pose_fields = getattr(self, '_rc_pose_fields', {})
        if pose_fields:
            pose = None
            if hasattr(self, 'ee_pose_for'):
                pose = self.ee_pose_for(rid)
            if pose is not None:
                pos = pose.position
                for label, val in (('X', pos.x), ('Y', pos.y), ('Z', pos.z)):
                    if label in pose_fields:
                        pose_fields[label].set_actual(val)

    def _highlight_rc_mode_tab(self, active_btn: QPushButton, kind: str):
        """Mark one mode tab active.

        The control bar drives mode selection through its own rows, so the
        old page's tab strip may not exist. Absent widgets are not an error
        here: an exception raised inside a Qt slot aborts the interpreter
        rather than surfacing, so this stays defensive on purpose.
        """
        buttons = getattr(self, "_rc_mode_buttons", None) or []
        if active_btn is None:
            return
        for b in buttons:
            b.setProperty("active_j", False)
            b.setProperty("active_c", False)
            b.style().unpolish(b)
            b.style().polish(b)
        if kind == "joint":
            active_btn.setProperty("active_j", True)
        elif kind == "cart":
            active_btn.setProperty("active_c", True)
        active_btn.style().unpolish(active_btn)
        active_btn.style().polish(active_btn)

    def _rc_apply_mode_availability(self, robot_name: str):
        """
        Enable only the modes a robot can actually run.

        Legged robots configure `kinematics_solver: none`, so the C++
        node rejects Cartesian modes outright. Disabling the buttons
        makes that refusal visible in the UI instead of surfacing as a
        log error. If a Cartesian mode is currently selected, fall back
        to Joint Impedance through the normal selection path so no stale
        Cartesian selection survives the switch.
        """
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        legged = cfg.get('robot_category') == 'legged'
        for button, (_, kind) in zip(getattr(self, '_rc_mode_buttons', []),
                                    CONTROL_MODE_TABS):
            button.setEnabled(not (legged and kind == 'cart'))
        # The control bar gates the whole Cart menu rather than four tabs.
        if hasattr(self, '_cb_apply_availability'):
            self._cb_apply_availability()
        if legged and getattr(self, '_rc_selected_kind', 'joint') == 'cart':
            self._on_rc_mode_selected(0, 'Joint Imp')

    def _update_rc_section_visibility(self, kind: str):
        """Show/hide mode-dependent sections inside Advanced for joint or cart."""
        show_cart = (kind == 'cart')
        show_joint = (kind == 'joint')

        def _show(attr, visible):
            w = getattr(self, attr, None)
            if w is not None:
                w.setVisible(visible)

        _show('_rc_joint_group', show_joint)
        _show('_rc_custom_group', show_cart)

        # Preset row and send-pose row depend on both kind and target
        is_hw = (self.control_target == 'hardware')
        _show('_rc_preset_row', show_cart and not is_hw)
        _show('_rc_pose_row', show_cart and is_hw)

        # On the control bar each menu permanently owns the modifiers that
        # apply to its kind, so there is nothing to show or hide: the
        # toggles only need hiding while the old single-page layout exists.
        if getattr(self, '_rc_nullspace_label', None) is not None:
            _show('_rc_postural_toggle', show_cart)
            _show('_rc_elbow_toggle', show_cart)
            _show('_rc_hpmp_toggle', show_cart)
            _show('_rc_nullspace_label', show_cart)
            _show('_rc_admittance_planner_toggle', show_joint)
            self._rc_admittance_planner_toggle.setEnabled(show_joint)

    # ──────────────────────────────────────────────────────────────────────
    #  Preset poses (simulation, Cartesian only)
    # ──────────────────────────────────────────────────────────────────────

    def _build_rc_preset_row(self) -> QWidget:
        from geometry_msgs.msg import Pose, Point, Quaternion
        self.presets = [
            Pose(position=Point(x=0.0, y=0.0,  z=0.0),  orientation=Quaternion(x=0.3, y=0.4, z=0.3, w=1.0)),
            Pose(position=Point(x=0.0, y=0.0,  z=1.31), orientation=Quaternion(x=0.0, y=0.0, z=0.3, w=1.0)),
            Pose(position=Point(x=1.0, y=0.2,  z=1.0),  orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=1.0)),
            Pose(position=Point(x=0.5, y=-0.5, z=0.5),  orientation=Quaternion(x=0.6, y=0.1, z=0.0, w=1.0)),
        ]
        container = QWidget()
        row = QHBoxLayout(container)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        row.addWidget(self._section_header("Preset Poses"))
        for i, p in enumerate(self.presets):
            label = f"({p.position.x:.1f}, {p.position.y:.1f}, {p.position.z:.2f})"
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, i=i: self.node.publish_position(self.presets[i]))
            row.addWidget(b)
        row.addStretch()
        return container

    # ──────────────────────────────────────────────────────────────────────
    #  Custom pose + joysticks (Cartesian)
    # ──────────────────────────────────────────────────────────────────────

    def _build_rc_custom_pose_group(self) -> QGroupBox:
        grp = QGroupBox("Custom Pose  —  Position + Axis–Angle")
        vbox = QVBoxLayout(grp)
        vbox.setSpacing(6)

        # Position + axis-angle side by side
        side_row = QHBoxLayout()

        # ── Position ──
        pos_grp = QGroupBox("Position (m)")
        pos_vbox = QVBoxLayout(pos_grp)
        pos_vbox.setSpacing(5)

        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        ee = cfg.get('default_ee_target', {})
        pos = ee.get('position', [0.38, 0.0, 0.432])

        _cart_tt = (
            "End-effector target in the robot's base frame (metres).\n"
            "Used by Cartesian modes 5–8. The IK solver converts this to joint angles; "
            "if the target is out of reach or at a singularity the log shows "
            "'IK: no solution' and the previous target is kept."
        )
        self.x_input = QLineEdit(str(pos[0]) if len(pos) > 0 else "0.0")
        self.x_input.setPlaceholderText("X")
        self.x_input.setToolTip(_cart_tt + "\n\nX axis (forward, +ve away from the robot base).")
        self.y_input = QLineEdit(str(pos[1]) if len(pos) > 1 else "0.0")
        self.y_input.setPlaceholderText("Y")
        self.y_input.setToolTip(_cart_tt + "\n\nY axis (lateral, +ve to the robot's left).")
        self.z_input = QLineEdit(str(pos[2]) if len(pos) > 2 else "0.0")
        self.z_input.setPlaceholderText("Z")
        self.z_input.setToolTip(_cart_tt + "\n\nZ axis (vertical, +ve up).")

        pos_form = QFormLayout()
        pos_form.setLabelAlignment(Qt.AlignLeft)
        pos_form.setSpacing(5)
        pos_form.addRow("X (m):", self.x_input)
        pos_form.addRow("Y (m):", self.y_input)
        pos_form.addRow("Z (m):", self.z_input)
        pos_vbox.addLayout(pos_form)

        # Joysticks
        joy_layout = QHBoxLayout()
        xy_layout = QVBoxLayout()
        xy_lbl = QLabel("X / Y")
        xy_lbl.setAlignment(Qt.AlignCenter)
        theme.bind(xy_lbl,
                   lambda t: 'color:%s; font-size:10px; font-weight: 600;' % t['text_secondary'])
        self.joy_xy = VirtualJoystick()
        xy_layout.addWidget(xy_lbl)
        xy_layout.addWidget(self.joy_xy)
        z_layout = QVBoxLayout()
        z_lbl = QLabel("Z (Up/Down)")
        z_lbl.setAlignment(Qt.AlignCenter)
        theme.bind(z_lbl,
                   lambda t: 'color:%s; font-size:10px; font-weight: 600;' % t['text_secondary'])
        self.joy_z = VirtualJoystick()
        z_layout.addWidget(z_lbl)
        z_layout.addWidget(self.joy_z)
        joy_layout.addLayout(xy_layout)
        joy_layout.addLayout(z_layout)
        pos_vbox.addLayout(joy_layout)

        self.joy_autopublish = QCheckBox("Live Publish on Joystick Move")
        self.joy_autopublish.setChecked(True)
        # Superseded by the action-bar Live-publish toggle; kept for back-compat
        # but hidden so users don't see a duplicate, non-functional control.
        self.joy_autopublish.setVisible(False)
        pos_vbox.addWidget(self.joy_autopublish)

        side_row.addWidget(pos_grp)

        # ── Axis-angle ──
        if robot_name == "Franka_FR3":
            def_ax, def_ay, def_az, def_angle = "-1.0", "0.0", "0.0", "180.0"
        elif robot_name == "KinovaGen3_7DOF":
            def_ax, def_ay, def_az, def_angle = "0.5773503", "0.5773503", "0.5773503", "119.999999863"
        else:
            def_ax, def_ay, def_az, def_angle = "0.0", "0.0", "1.0", "0.0"

        aa_grp = QGroupBox("Orientation — Axis / Angle")
        aa_layout = QGridLayout(aa_grp)
        aa_layout.setSpacing(5)

        _aa_tt = (
            "End-effector orientation as axis-angle: a unit vector (ax, ay, az) and a rotation θ about it. "
            "Converted to quaternion before publishing (see QUAT preview below). "
            "The defaults above are home-pose orientations — keep them if you only want to move the EE position."
        )
        self.ax_input = QLineEdit(def_ax)
        self.ax_input.setPlaceholderText("ax")
        self.ax_input.setToolTip(_aa_tt)
        self.ay_input = QLineEdit(def_ay)
        self.ay_input.setPlaceholderText("ay")
        self.ay_input.setToolTip(_aa_tt)
        self.az_input = QLineEdit(def_az)
        self.az_input.setPlaceholderText("az")
        self.az_input.setToolTip(_aa_tt)
        self.angle_input = QLineEdit(def_angle)
        self.angle_input.setPlaceholderText("angle")
        self.angle_input.setToolTip(
            "Rotation angle θ about the axis above. Unit picker on the right (deg or rad)."
        )
        self.unit_combo = QComboBox()
        self.unit_combo.addItems(["deg", "rad"])
        self.unit_combo.setToolTip("Unit for the θ angle field above.")

        for i, (lbl_text, widget) in enumerate([
            ("ax:", self.ax_input), ("ay:", self.ay_input),
            ("az:", self.az_input), ("θ:", self.angle_input),
        ]):
            aa_layout.addWidget(QLabel(lbl_text), i, 0)
            aa_layout.addWidget(widget, i, 1)
        aa_layout.addWidget(QLabel("Units:"), 4, 0)
        aa_layout.addWidget(self.unit_combo, 4, 1)

        self.q_preview = QLabel("QUAT  [ 0.000000,  0.000000,  0.000000,  1.000000 ]")
        self.q_preview.setObjectName("QuatPreview")
        aa_layout.addWidget(self.q_preview, 5, 0, 1, 2)

        side_row.addWidget(aa_grp)
        vbox.addLayout(side_row)

        # Buttons
        btn_row = QHBoxLayout()
        reset_pose_btn = self._btn("RESET DEFAULTS")
        reset_pose_btn.clicked.connect(self._reset_rc_pose_defaults)
        btn_row.addWidget(reset_pose_btn)
        btn_row.addStretch()
        pub_btn = self._btn("PUBLISH CUSTOM POSE", "BtnTeal")
        pub_btn.clicked.connect(self._publish_rc_pose)
        btn_row.addWidget(pub_btn)
        vbox.addLayout(btn_row)

        # Wiring
        for w in (self.ax_input, self.ay_input, self.az_input, self.angle_input):
            w.textChanged.connect(self._update_quat_preview)
        self.unit_combo.currentIndexChanged.connect(self._update_quat_preview)
        for w in (self.x_input, self.y_input, self.z_input,
                  self.ax_input, self.ay_input, self.az_input, self.angle_input):
            w.returnPressed.connect(self._publish_rc_pose)

        self.joy_xy.joystickMoved.connect(self._on_joy_xy_moved)
        self.joy_z.joystickMoved.connect(self._on_joy_z_moved)

        self._update_quat_preview()
        return grp

    # ──────────────────────────────────────────────────────────────────────
    #  Joint sliders
    # ──────────────────────────────────────────────────────────────────────

    def _build_rc_joint_group(self) -> QGroupBox:
        grp = QGroupBox("Joint Angle Control")
        vbox = QVBoxLayout(grp)
        vbox.setSpacing(6)

        top = QHBoxLayout()
        top.setSpacing(8)

        # Unit toggle
        unit_frame = QFrame()
        theme.bind(unit_frame,
                   lambda t: 'border:1px solid %s; border-radius:6px;' % t['border_subtle'])
        unit_row = QHBoxLayout(unit_frame)
        unit_row.setContentsMargins(0, 0, 0, 0)
        unit_row.setSpacing(0)
        self.joint_units_rad = QPushButton("RAD")
        self.joint_units_rad.setObjectName("UnitBtn")
        self.joint_units_deg = QPushButton("DEG")
        self.joint_units_deg.setObjectName("UnitBtn")
        self.joint_units_deg.setProperty("active", True)
        self.joint_units_rad.clicked.connect(lambda: self._rc_set_unit("rad"))
        self.joint_units_deg.clicked.connect(lambda: self._rc_set_unit("deg"))
        unit_row.addWidget(self.joint_units_rad)
        unit_row.addWidget(self.joint_units_deg)

        self.joint_units = QComboBox()  # Hidden, for logic compat
        self.joint_units.addItems(["deg", "rad"])
        self.joint_units.hide()

        self.joint_autopublish = QCheckBox("Auto-publish")
        top.addWidget(unit_frame)
        top.addWidget(self.joint_autopublish)
        top.addStretch()

        reset_joints_btn = self._btn("RESET DEFAULTS")
        reset_joints_btn.clicked.connect(self._reset_rc_joint_defaults)
        top.addWidget(reset_joints_btn)

        pub_btn = self._btn("PUBLISH JOINTS", "BtnPrimary")
        pub_btn.clicked.connect(self._publish_rc_joint_angles)
        top.addWidget(pub_btn)
        vbox.addLayout(top)

        self.joint_sliders_container = QWidget()
        self.joint_sliders_layout = QVBoxLayout(self.joint_sliders_container)
        self.joint_sliders_layout.setContentsMargins(0, 0, 0, 0)
        self.joint_sliders_layout.setSpacing(4)
        vbox.addWidget(self.joint_sliders_container)

        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        num_joints = cfg.get('num_joints', self.robots.get(robot_name, 7))
        q_home = cfg.get('q_home_deg', None)
        self._build_joint_sliders(num_joints, default_deg_values=q_home)
        return grp

    # ──────────────────────────────────────────────────────────────────────
    #  Gripper (hardware only)
    # ──────────────────────────────────────────────────────────────────────

    def _build_rc_gripper_group(self) -> QGroupBox:
        grp = QGroupBox("Gripper Control (0% Closed - 100% Open)")
        layout = QHBoxLayout(grp)
        layout.setSpacing(10)

        gripper_lbl = QLabel("Width:")
        gripper_lbl.setObjectName("HintLabel")

        self._rc_gripper_slider = QSlider(Qt.Horizontal)
        self._rc_gripper_slider.setRange(0, 100)
        self._rc_gripper_slider.setValue(100)
        self._rc_gripper_slider.setSingleStep(1)

        self._rc_gripper_val = QLabel("100 %")
        self._rc_gripper_val.setFixedWidth(45)
        self._rc_gripper_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._rc_gripper_val.setObjectName("AccentValue")

        self._rc_gripper_slider.valueChanged.connect(self._on_rc_gripper_changed)

        layout.addWidget(gripper_lbl)
        layout.addWidget(self._rc_gripper_slider, 1)
        layout.addWidget(self._rc_gripper_val)
        return grp

    def _on_rc_gripper_changed(self, val):
        """Publish gripper width to the active robot when the slider moves."""
        self._rc_gripper_val.setText('%d %%' % val)
        actual_width = (val / 100.0) * 0.08
        self.node.publish_gripper_width(self._rc_command_rid(), actual_width)

    # ──────────────────────────────────────────────────────────────────────
    #  Joint slider building & interaction
    # ──────────────────────────────────────────────────────────────────────

    def _build_joint_sliders(self, num_joints: int, default_deg_values=None):
        if default_deg_values is None:
            robot_name = self.robot_dropdown.currentText()
            cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
            default_deg_values = cfg.get('q_home_deg', None)

        layout = self.joint_sliders_layout
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        self.joint_sliders = []
        self.joint_value_labels = []
        self.joint_default_degrees = []

        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        limits = cfg.get('joint_limits_deg', {})
        lower_list = limits.get('lower', [])
        upper_list = limits.get('upper', [])

        for i in range(num_joints):
            default_deg = default_deg_values[i] if default_deg_values and i < len(default_deg_values) else 0.0
            self.joint_default_degrees.append(default_deg)

            lo_deg = lower_list[i] if i < len(lower_list) else -180
            hi_deg = upper_list[i] if i < len(upper_list) else 180
            lo_cr = int(math.radians(lo_deg) * 100)
            hi_cr = int(math.radians(hi_deg) * 100)

            row_w = QWidget()
            row = QHBoxLayout(row_w)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(8)

            lbl_name = QLabel(f"J{i+1}")
            lbl_name.setFixedWidth(28)
            theme.bind(lbl_name,
                       lambda t: 'color:%s;font-size:10px;' % t['text_secondary'])
            row.addWidget(lbl_name)

            sld = QSlider(Qt.Horizontal)
            sld.setRange(lo_cr, hi_cr)
            sld.setSingleStep(1)
            sld.setPageStep(10)
            default_cr = max(lo_cr, min(hi_cr, int(math.radians(default_deg) * 100)))
            sld.setValue(default_cr)
            sld.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            sld.setToolTip(
                f"Joint {i+1} target position. Range: [{lo_deg:.1f}° to {hi_deg:.1f}°].\n"
                "Handle turns orange when you've moved it off the home pose.\n"
                "Publishes to robot{N}_desired_joint_angles (joint modes 1–4) "
                "or feeds the IK solver (Cartesian modes 5–8)."
            )
            row.addWidget(sld, 1)

            val_lbl = QLabel()
            val_lbl.setFixedWidth(65)
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            theme.bind(val_lbl,
                       lambda t: 'color:%s;font-weight: 600;font-size:10px;' % t['text_primary'])
            row.addWidget(val_lbl)

            layout.addWidget(row_w)
            self.joint_sliders.append(sld)
            self.joint_value_labels.append(val_lbl)

            sld.valueChanged.connect(
                lambda v, idx=i, lbl=val_lbl: self._on_joint_slider_changed(idx, v, lbl))
            self._on_joint_slider_changed(i, sld.value(), val_lbl)

        layout.addStretch(1)

    def _on_joint_slider_changed(self, index: int, value_centi_rad: int, lbl: QLabel):
        rad_val = value_centi_rad / 100.0
        units = self.joint_units.currentText()
        if units == "deg":
            display = math.degrees(rad_val)
            lbl.setText(f"{display:.2f}°")
        else:
            lbl.setText(f"{rad_val:.4f}")

        default_deg = self.joint_default_degrees[index] if index < len(self.joint_default_degrees) else 0.0
        default_centi = int(math.radians(default_deg) * 100)
        is_mod = (value_centi_rad != default_centi)
        sld = self.joint_sliders[index]
        _t = theme.current()
        _sld_color = _t['accent_orange'] if is_mod else _t['accent_blue']
        sld.setStyleSheet(
            'QSlider::handle:horizontal{background:%s;}'
            'QSlider::sub-page:horizontal{background:%s;}' % (_sld_color, _sld_color)
        )

        if hasattr(self, "joint_autopublish") and self.joint_autopublish.isChecked():
            self._publish_rc_joint_angles()

    def _rc_set_unit(self, unit: str):
        self.joint_units.setCurrentText(unit)
        for btn, u in ((self.joint_units_rad, "rad"), (self.joint_units_deg, "deg")):
            btn.setProperty("active", u == unit)
            btn.style().unpolish(btn)
            btn.style().polish(btn)
        for i, sld in enumerate(getattr(self, "joint_sliders", [])):
            lbl = self.joint_value_labels[i]
            self._on_joint_slider_changed(i, sld.value(), lbl)

    def _gather_joint_angles_radians(self):
        return [s.value() / 100.0 for s in getattr(self, "joint_sliders", [])]

    # ──────────────────────────────────────────────────────────────────────
    #  Unified publish methods
    # ──────────────────────────────────────────────────────────────────────

    def _publish_rc_joint_angles(self):
        if not getattr(self, "joint_sliders", []):
            QMessageBox.warning(self, "No Sliders", "No joint sliders to publish.")
            return
        radians = self._gather_joint_angles_radians()

        if self.control_target in ("simulation", "vr"):
            names = [f"{self.robot_dropdown.currentText()}_j{i+1}" for i in range(len(radians))]
            self.node.publish_joint_angles(radians, names=names)
        else:
            self.node.publish_real_joint_angles(self.node.station_robot_id, radians)

        degrees = [math.degrees(r) for r in radians]
        deg_str = ", ".join([f"{d:.1f}°" for d in degrees])
        self.log_message("INFO", f"Joints: [{deg_str}]", "Control")

    def _publish_rc_pose(self):
        """Publish Cartesian pose using axis-angle inputs."""
        from geometry_msgs.msg import Pose, Point, Quaternion
        from fic_studio.math_helpers import _axis_angle_to_quat

        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
            ax = float(self.ax_input.text())
            ay = float(self.ay_input.text())
            az = float(self.az_input.text())
            angle = float(self.angle_input.text())
            units = self.unit_combo.currentText()
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Please enter valid numeric values.")
            return

        qx, qy, qz, qw = _axis_angle_to_quat(ax, ay, az, angle, units)
        pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)
        )

        if self.control_target in ("simulation", "vr"):
            self.node.publish_position(pose)
        else:
            self.send_robot_1_pose(pose=pose)

        self.log_message("INFO", f"Pose: ({x:.3f}, {y:.3f}, {z:.3f})", "Control")

    # Keep the old name as an alias for backward compat
    def publish_custom_pose_axis_angle(self):
        self._publish_rc_pose()

    # ──────────────────────────────────────────────────────────────────────
    #  Joystick handlers
    # ──────────────────────────────────────────────────────────────────────

    def _on_joy_xy_moved(self, dx: float, dy: float):
        """Joystick X/Y: nudge position MetricFields and optionally publish.

        Updates the adaptive Cartesian MetricFields (X/Y) when in cart mode;
        falls back to the legacy x_input/y_input when those fields are absent
        (e.g. in the Advanced custom-pose group with no MetricFields built).
        Publishes under the same Live-publish gate as edited events.
        """
        speed = 0.005
        pose_fields = getattr(self, '_rc_pose_fields', {})
        if 'X' in pose_fields and 'Y' in pose_fields:
            x_field = pose_fields['X']
            y_field = pose_fields['Y']
            # CommandedVsActual wraps a MetricField at ._field;
            # plain MetricField is its own value store.
            _x_inner = getattr(x_field, '_field', x_field)
            _y_inner = getattr(y_field, '_field', y_field)
            _x_inner.set_value(_x_inner.value() + dx * speed)
            _y_inner.set_value(_y_inner.value() + dy * speed)
            lp = getattr(self, '_rc_live_publish', None)
            if lp is not None and lp.isChecked():
                self._rc_publish_pose_from_fields()
        else:
            try:
                curr_x = float(self.x_input.text())
                curr_y = float(self.y_input.text())
                self.x_input.setText('%.4f' % (curr_x + dx * speed))
                self.y_input.setText('%.4f' % (curr_y + dy * speed))
                lp = getattr(self, '_rc_live_publish', None)
                if lp is not None and lp.isChecked():
                    self._publish_rc_pose()
            except (ValueError, AttributeError):
                pass

    def _on_joy_z_moved(self, dx: float, dy: float):
        """Joystick Z: nudge position MetricField Z and optionally publish.

        Updates the adaptive Cartesian MetricField (Z) when in cart mode;
        falls back to the legacy z_input when that field is absent.
        Publishes under the same Live-publish gate as edited events.
        """
        speed = 0.005
        pose_fields = getattr(self, '_rc_pose_fields', {})
        if 'Z' in pose_fields:
            z_field = pose_fields['Z']
            _z_inner = getattr(z_field, '_field', z_field)
            _z_inner.set_value(_z_inner.value() + dy * speed)
            lp = getattr(self, '_rc_live_publish', None)
            if lp is not None and lp.isChecked():
                self._rc_publish_pose_from_fields()
        else:
            try:
                curr_z = float(self.z_input.text())
                self.z_input.setText('%.4f' % (curr_z + dy * speed))
                lp = getattr(self, '_rc_live_publish', None)
                if lp is not None and lp.isChecked():
                    self._publish_rc_pose()
            except (ValueError, AttributeError):
                pass

    # ──────────────────────────────────────────────────────────────────────
    #  Quat preview
    # ──────────────────────────────────────────────────────────────────────

    def _update_quat_preview(self):
        from fic_studio.math_helpers import _axis_angle_to_quat
        try:
            ax = float(self.ax_input.text())
            ay = float(self.ay_input.text())
            az = float(self.az_input.text())
            ang = float(self.angle_input.text())
            units = self.unit_combo.currentText()
            qx, qy, qz, qw = _axis_angle_to_quat(ax, ay, az, ang, units)
            self.q_preview.setText(f"QUAT  [ {qx:.6f},  {qy:.6f},  {qz:.6f},  {qw:.6f} ]")
        except ValueError:
            self.q_preview.setText("QUAT  [ invalid ]")

    # ──────────────────────────────────────────────────────────────────────
    #  Reset defaults
    # ──────────────────────────────────────────────────────────────────────

    def _reset_rc_defaults(self):
        """Reset everything to robot config defaults."""
        self._reset_rc_joint_defaults()
        self._reset_rc_pose_defaults()
        # Reset mode to first
        self._rc_selected_mode_idx = 0
        self._rc_selected_kind = "joint"
        page_tabs = getattr(self, "_rc_mode_buttons", None)
        if page_tabs:
            self._highlight_rc_mode_tab(page_tabs[0], "joint")
        self._rc_mode_dropdown.setCurrentIndex(0)
        self._update_rc_section_visibility("joint")
        self._rebuild_rc_control_area()
        for hook in ("_cb_sync_mode_buttons", "_cb_place_surface",
                     "_cb_refresh_status"):
            fn = getattr(self, hook, None)
            if fn is not None:
                fn()
        self.log_message("INFO", "All controls reset to defaults", "Control")

    def _reset_rc_joint_defaults(self):
        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        num_joints = cfg.get('num_joints', self.robots.get(robot_name, 7))
        q_home = cfg.get('q_home_deg', None)
        self._build_joint_sliders(num_joints, default_deg_values=q_home)
        self.log_message("INFO", "Joint sliders reset to defaults", "Control")

    def _reset_rc_pose_defaults(self):
        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        ee = cfg.get('default_ee_target', {})
        pos = ee.get('position', [0.0, 0.0, 0.0])

        self.x_input.setText(str(pos[0]) if len(pos) > 0 else "0.0")
        self.y_input.setText(str(pos[1]) if len(pos) > 1 else "0.0")
        self.z_input.setText(str(pos[2]) if len(pos) > 2 else "0.0")

        if robot_name == "Franka_FR3":
            self.ax_input.setText("-1.0"); self.ay_input.setText("0.0")
            self.az_input.setText("0.0"); self.angle_input.setText("180.0")
        elif robot_name == "KinovaGen3_7DOF":
            self.ax_input.setText("0.5773503"); self.ay_input.setText("0.5773503")
            self.az_input.setText("0.5773503"); self.angle_input.setText("119.999999863")
        else:
            self.ax_input.setText("0.0"); self.ay_input.setText("0.0")
            self.az_input.setText("1.0"); self.angle_input.setText("0.0")

        self.unit_combo.setCurrentIndex(0)
        self._update_quat_preview()
        self.log_message("INFO", "Custom pose reset to defaults", "Control")

    def _on_rc_robot_changed(self, robot_name: str):
        if not robot_name:
            return
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        num_joints = cfg.get('num_joints', self.robots.get(robot_name, 7))
        q_home = cfg.get('q_home_deg', None)
        self._build_joint_sliders(num_joints, default_deg_values=q_home)
        self._reset_rc_pose_defaults()
        # Rebuild the adaptive control surface too. Without this the joint
        # rows and EE fields stay as they were built at start-up, when the
        # robot dropdown was still empty — i.e. empty.
        self._rebuild_rc_control_area()
        for hook in ('_cb_place_surface', '_cb_apply_availability',
                     '_cb_refresh_status'):
            fn = getattr(self, hook, None)
            if fn is not None:
                fn()
        # Profiles are per (robot, mode) — refresh and re-publish.
        self._refresh_rc_profile_dropdown()
        self._publish_rc_profile()

    # ──────────────────────────────────────────────────────────────────────
    #  Config (tuning profile) selector
    # ──────────────────────────────────────────────────────────────────────

    def _rc_current_mode_id(self) -> int:
        """Return the 1-based control-mode id the control panel is set to."""
        idx = getattr(self, "_rc_selected_mode_idx", 0)
        return int(idx) + 1

    def _rc_current_profile(self) -> str:
        dd = getattr(self, "_rc_profile_dropdown", None)
        if dd is None:
            return "default"
        # Display text may carry a "(fallback)" suffix — itemData holds the clean name.
        data = dd.currentData()
        if data:
            return data
        name = dd.currentText()
        return name if name else "default"

    def _rc_target_default_profile(self, profiles):
        """Return the config used until the user picks one: 'sim' on the
        Simulation target when the robot defines it, else 'default'. A robot
        simulated like the real one (its hardware_model) uses 'default' in
        simulation too: the tuning tried there is the one the arm runs."""
        target = (self.robot_target() if hasattr(self, 'robot_target')
                  else getattr(self, 'control_target', 'simulation'))
        if target != 'simulation' or 'sim' not in profiles:
            return 'default'
        return 'default' if self._rc_simulated_like_hardware() else 'sim'

    def _rc_simulated_like_hardware(self):
        """The selected robot has an enabled hardware_model (robots_config.yaml) and
        FIC_HARDWARE_MODEL does not switch it off for this Studio's simulator."""
        env = dict(os.environ, **(getattr(self, '_sim_env_override', None) or {}))
        if env.get('FIC_HARDWARE_MODEL', '').strip().lower() in ('off', '0', 'false', 'ideal'):
            return False
        dd = getattr(self, 'robot_dropdown', None)
        robot = dd.currentText() if dd is not None else ''
        cfg = (getattr(self, 'robot_configs', {}) or {}).get(robot) or {}
        hm = cfg.get('hardware_model')
        return isinstance(hm, dict) and bool(hm.get('enabled', True))

    def _rc_note_profile_choice(self):
        """Remember a config the user picked; the target's own default is not
        a choice, so the selection keeps following the target."""
        dd = getattr(self, "_rc_profile_dropdown", None)
        if dd is None:
            return
        names = [dd.itemData(i) for i in range(dd.count())]
        current = self._rc_current_profile()
        self._rc_profile_choice = (
            None if current == self._rc_target_default_profile(names) else current)

    def _refresh_rc_profile_dropdown(self):
        """Populate the Config dropdown with the robot-scoped profile union.
        Profiles are robot-scoped: picking 'Stiff' here activates 'Stiff' for
        whichever mode is currently selected. The loader auto-falls back to
        'default' for any mode that doesn't have a Stiff override."""
        dd = getattr(self, "_rc_profile_dropdown", None)
        if dd is None:
            return
        try:
            from fic_studio.xml_manager import XMLManager
            mgr = XMLManager()
            robot = self.robot_dropdown.currentText()
            mode = self._rc_current_mode_id()
            profiles = mgr.get_all_profiles(robot) or ["default"]
            with_override = set(mgr.get_available_profiles(robot, mode))
        except Exception:
            profiles = ["default"]
            with_override = {"default"}

        # Until the user picks a config, the target decides: 'sim' (when the
        # robot has one) on the Simulation target, 'default' otherwise.
        chosen = getattr(self, '_rc_profile_choice', None)
        prev = chosen if chosen in profiles else self._rc_target_default_profile(profiles)
        dd.blockSignals(True)
        dd.clear()
        for p in profiles:
            label = p if p in with_override else f"{p}  (fallback → default)"
            dd.addItem(label, p)
        idx = dd.findData(prev) if prev else -1
        if idx < 0:
            idx = dd.findData("default")
        dd.setCurrentIndex(max(0, idx))
        dd.blockSignals(False)

        # The control bar shows a Config selector in each menu; mirror the
        # canonical list into the others so Cart is not left with an empty
        # dropdown.
        if hasattr(self, '_cb_mirror_profiles'):
            self._cb_mirror_profiles()

    def _on_rc_profile_changed(self):
        """User picked a config in the control panel → publish it."""
        self._publish_rc_profile()
        self.log_message("INFO",
            f"Active config → {self._rc_current_profile()}", "Control")

    def _publish_rc_profile(self):
        """Publish the currently selected profile on active_profile_publisher_."""
        try:
            from std_msgs.msg import String
            node = getattr(self, "node", None)
            pub = getattr(node, "active_profile_publisher_", None) if node else None
            if pub is None:
                return
            msg = String()
            msg.data = self._rc_current_profile() or "default"
            pub.publish(msg)
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────
    #  Publisher_mixin compatibility — register widgets as robot_1
    # ──────────────────────────────────────────────────────────────────────

    def _register_robot_1_widgets(self):
        """Register the primary control widgets as robot_1 attributes
        so that publisher_mixin.get_pose_from_inputs('robot_1') works."""
        self.robot_1_pos_x = self.x_input
        self.robot_1_pos_y = self.y_input
        self.robot_1_pos_z = self.z_input
        self.robot_1_axis_x = self.ax_input
        self.robot_1_axis_y = self.ay_input
        self.robot_1_axis_z = self.az_input
        self.robot_1_angle = self.angle_input
        self.robot_1_angle_units = self.unit_combo
        self.robot_1_q_preview = self.q_preview
        self.robot_1_gripper_slider = self._rc_gripper_slider
        self.robot_1_joy_xy = self.joy_xy
        self.robot_1_joy_z = self.joy_z
        self.robot_1_joy_autopub = self.joy_autopublish

    # Keep convert_axis_angle_preview for publisher_mixin compat
    def convert_axis_angle_preview(self, robot_id: str):
        from fic_studio.math_helpers import _axis_angle_to_quat
        try:
            ax  = float(getattr(self, f'{robot_id}_axis_x').text())
            ay  = float(getattr(self, f'{robot_id}_axis_y').text())
            az  = float(getattr(self, f'{robot_id}_axis_z').text())
            ang = float(getattr(self, f'{robot_id}_angle').text())
            units = getattr(self, f'{robot_id}_angle_units').currentText()
            qx, qy, qz, qw = _axis_angle_to_quat(ax, ay, az, ang, units)
            preview = getattr(self, f'{robot_id}_q_preview')
            preview.setText(f"QUAT  [ {qx:.6f},  {qy:.6f},  {qz:.6f},  {qw:.6f} ]")
        except ValueError:
            preview = getattr(self, f'{robot_id}_q_preview')
            preview.setText("QUAT  [ invalid ]")

    # ──────────────────────────────────────────────────────────────────────
    #  Multi-robot hardware cards (Robot 2, 3, 4)
    # ──────────────────────────────────────────────────────────────────────

    def _rc_refresh_joint_count(self):
        """Rebuild the joint control area if its joint count is stale.

        Called when the Robot Control page opens so the slider count matches
        the active robot immediately, without waiting for a mode switch.
        """
        if getattr(self, '_rc_selected_kind', 'joint') != 'joint':
            return
        robot = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
        cfg = getattr(self, 'robot_configs', {}).get(robot, {})
        expected = len(self._rc_display_joints(robot, cfg))
        if len(getattr(self, '_rc_joint_fields', [])) != expected:
            self._rebuild_rc_control_area()

    def _rc_display_joints(self, robot_name, cfg=None):
        """Rows to show in the joint control area.

        Returns a list of dicts ``{name, label, actuated, lo, hi, cmd_index}``.
        A robot with a ``view_joint_names`` list (e.g. the CSPM parallel robot)
        lists ALL those joints, but only the ones also in ``joint_names`` (the
        actuated set) are controllable; the rest render greyed/view-only.
        ``cmd_index`` is the joint's position in the actuated list (used for
        publish + limits) or ``None`` for view-only joints. Robots WITHOUT
        ``view_joint_names`` behave exactly as before: ``num_joints`` controllable
        ``J1..Jn`` rows named ``<robot>_jN``.
        """
        if cfg is None:
            cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        limits = cfg.get('joint_limits_deg', {}) or {}
        lower = limits.get('lower', []) or []
        upper = limits.get('upper', []) or []
        actuated = cfg.get('joint_names', []) or []
        act_index = {nm: k for k, nm in enumerate(actuated) if nm}
        view = cfg.get('view_joint_names') or []
        rows = []
        if view:
            for nm in view:
                ci = act_index.get(nm)
                is_act = ci is not None
                lo = lower[ci] if (is_act and ci < len(lower)) else -180.0
                hi = upper[ci] if (is_act and ci < len(upper)) else 180.0
                rows.append({'name': nm, 'label': nm, 'actuated': is_act,
                             'lo': lo, 'hi': hi, 'cmd_index': ci})
        else:
            num_joints = cfg.get('num_joints', getattr(self, 'robots', {}).get(robot_name, 7))
            for i in range(int(num_joints)):
                lo = lower[i] if i < len(lower) else -180.0
                hi = upper[i] if i < len(upper) else 180.0
                rows.append({'name': '%s_j%d' % (robot_name, i + 1),
                             'label': 'J%d' % (i + 1), 'actuated': True,
                             'lo': lo, 'hi': hi, 'cmd_index': i})
        return rows

    def show_robot_control_panel(self):
        """Navigate to the Robot Control page and refresh the joint count."""
        self._show_page('multi_robot')
        self._set_active_sidebar_btn(self._side_buttons[1])
        self._rc_refresh_joint_count()

    def launch_robot_node(self):
        """Legacy alias — shows the robot control panel."""
        self.show_robot_control_panel()


class DesiredWrenchDialog(QDialog):
    """Six fields for the admittance's desired wrench, with Apply and Zero."""

    def __init__(self, host, rid):
        super().__init__(host if isinstance(host, QWidget) else None)
        self.host, self.rid = host, rid
        self.setWindowTitle('Desired wrench — %s' % rid)
        lay = QVBoxLayout(self)
        note = QLabel('The Cartesian admittance (modes 7 and 8, and H-PMP on them) '
                      'moves the goal until the measured wrench equals this one. '
                      'Zero is the plain compliant behaviour.')
        note.setWordWrap(True)
        lay.addWidget(note)
        grid = QGridLayout()
        self.fields = []
        for i, (label, unit, limit, step) in enumerate(DESIRED_WRENCH_AXES):
            box = QDoubleSpinBox()
            box.setRange(-limit, limit)
            box.setSingleStep(step)
            box.setDecimals(2)
            box.setSuffix(' ' + unit)
            grid.addWidget(QLabel(label), i % 3, 2 * (i // 3))
            grid.addWidget(box, i % 3, 2 * (i // 3) + 1)
            self.fields.append(box)
        lay.addLayout(grid)
        row = QHBoxLayout()
        zero = QPushButton('Zero')
        zero.setToolTip('Send a zero desired wrench')
        zero.clicked.connect(lambda: self.host._publish_desired_wrench([0.0] * 6, self.rid))
        apply_btn = QPushButton('Apply')
        apply_btn.setDefault(True)
        apply_btn.clicked.connect(
            lambda: self.host._publish_desired_wrench(self.values(), self.rid))
        close = QPushButton('Close')
        close.clicked.connect(self.close)
        row.addWidget(zero)
        row.addStretch(1)
        row.addWidget(apply_btn)
        row.addWidget(close)
        lay.addLayout(row)

    def values(self):
        """Return the six field values."""
        return [box.value() for box in self.fields]

    def show_values(self, values):
        """Fill the fields."""
        for box, v in zip(self.fields, values):
            box.setValue(float(v))
