# mixins/multi_robot_mixin.py — Per-station robot cards (Robot 2-4)
"""
Multi-robot mixin: the "＋ Add Robot" cards for stations 2 to 4.

Extracted from ``robot_control_mixin`` when the Robot Control page was
dismantled into the global control bar. The bar drives whichever robot is
active; these cards are for commanding several stations side by side, and
live on their own page.

``card_robot_type`` is part of the extension API (``ExtensionHost.
robot_type_for``) and ``_add_robot_btn`` is read directly by the
mujoco_playground extension. Both keep resolving because
``ProfessionalRobotGUI`` inherits this mixin alongside the others.
"""

import math
from functools import partial

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QSizePolicy, QSlider,
    QSpinBox, QVBoxLayout, QWidget,
)

from fic_studio.mixins.robot_control_mixin import (
    CONTROL_MODE_NAMES, CONTROL_MODE_TABS, VirtualJoystick,
)


class MultiRobotMixin:
    """Mixin providing the per-station robot cards and their page."""

    def _add_robot_card(self):
        """Add an extra robot card (Robot 2–4) for simulation or hardware.

        Brings the new robot online: creates publishers/subscribers on the
        node, then builds the UI card bound to that robot's rid.
        """
        if self._robot_card_count >= 3:
            return

        self._robot_card_count += 1
        n = self._robot_card_count + 1  # Robot 2, 3, 4
        robot_name = f"Robot {n}"
        robot_id = f"robot_{n}"

        # Default: card N controls station N
        self._card_station[robot_id] = n

        # ── Bring this robot online on the ROS node ──
        # Only create publishers (for sending commands). Do NOT subscribe to
        # the remote robot's high-frequency state topics (q_current, ee_pose)
        # — that forces its hardware node to stream 1 kHz data to this GUI
        # over the network, causing DDS contention and control-loop lag.
        self.node.add_robot(n)

        grp = self._create_extra_robot_section(robot_name)
        # Tag the card so we can find/remove it later by robot index.
        grp.setProperty("robot_index", n)
        self._robot_cards.append(grp)

        # Find the position before the stretch at the end.
        # Guard: in this layout the button has no parent widget (created
        # hidden, never added to a layout), so parentWidget() returns None.
        _btn_parent = self._add_robot_btn.parentWidget()
        if _btn_parent is not None:
            parent_layout = _btn_parent.layout()
            idx = parent_layout.indexOf(self._add_robot_btn)
            parent_layout.insertWidget(idx + 1, grp)

        if self._robot_card_count >= 3:
            self._add_robot_btn.setVisible(False)

        if hasattr(self, '_notify_extensions_robots_changed'):
            self._notify_extensions_robots_changed()

    def _remove_robot_card(self, n: int):
        """Tear down robot N's card and its node-level publishers/subscriptions."""
        robot_id = f"robot_{n}"
        station = self._card_station.pop(robot_id, n)
        # Remove from node using the actual station, not the card number.
        if hasattr(self, '_remove_robot_subscriptions'):
            self._remove_robot_subscriptions(station)
        if hasattr(self, '_remove_robot_safety_publishers'):
            self._remove_robot_safety_publishers(station)
        self.node.remove_robot(station)

        # Find and detach the matching card.
        target = None
        for card in self._robot_cards:
            if card.property("robot_index") == n:
                target = card
                break
        if target is not None:
            self._robot_cards.remove(target)
            target.setParent(None)
            target.deleteLater()

        self._robot_card_count = max(0, self._robot_card_count - 1)
        self._add_robot_btn.setVisible(self._robot_card_count < 3)

        if hasattr(self, '_notify_extensions_robots_changed'):
            self._notify_extensions_robots_changed()

    def card_robot_type(self, robot_id: str) -> str:
        """Return the effective robot type for a card.

        Card 1 (`robot_1`) follows the global dropdown; cards 2–4 use their
        own per-card override stored in `_card_robot_types`."""
        if robot_id == "robot_1":
            return self.robot_dropdown.currentText()
        return self._card_robot_types.get(
            robot_id, self.robot_dropdown.currentText())

    def _card_topic_rid(self, widget_id: str) -> str:
        """Return the ROS topic prefix ('robotN') for a card's current station."""
        n = self._card_station.get(widget_id, int(widget_id.split("_")[1]))
        return f"robot{n}"

    def _create_extra_robot_section(self, robot_name) -> QGroupBox:
        """Build a self-contained robot card (for Robot 2-4)."""
        robot_id = robot_name.lower().replace(" ", "_")
        selected_type = self.robot_dropdown.currentText()
        station_n = self._card_station.get(
            robot_id, int(robot_id.split("_")[1]))
        # Seed the per-card type with whatever the global dropdown is showing
        # right now. The card's own combo (added below) can change it later.
        self._card_robot_types[robot_id] = selected_type

        cfg = getattr(self, 'robot_configs', {}).get(selected_type, {})
        num_joints = cfg.get('num_joints', 7)
        ee = cfg.get('default_ee_target', {})
        _pos = ee.get('position', [0.0, 0.0, 0.0])
        def_pos = [str(_pos[0]), str(_pos[1]), str(_pos[2])]
        def_joints = cfg.get('q_home_deg', [0.0] * num_joints)
        if len(def_joints) < num_joints:
            def_joints = def_joints + [0.0] * (num_joints - len(def_joints))

        if selected_type == "Franka_FR3":
            def_axis, def_angle_str = ["-1.0", "0.0", "0.0"], "180.0"
        elif selected_type == "KinovaGen3_7DOF":
            def_axis, def_angle_str = ["0.5773503", "0.5773503", "0.5773503"], "119.999999863"
        else:
            def_axis, def_angle_str = ["0.0", "0.0", "1.0"], "0.0"

        grp = QGroupBox(f"{robot_name} — Station {station_n}")
        grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        grp_layout = QVBoxLayout(grp)
        grp_layout.setSpacing(8)

        # Mode tab buttons
        mode_row = QHBoxLayout()
        mode_row.setSpacing(4)
        mode_buttons = []
        for idx, (label, kind) in enumerate(CONTROL_MODE_TABS):
            btn = QPushButton(label)
            btn.setObjectName("ModeTab")
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(partial(
                self._on_extra_mode_selected, robot_id, idx, kind))
            mode_row.addWidget(btn)
            mode_buttons.append(btn)
        mode_row.addStretch()

        pause_btn = self._btn("PAUSE", "BtnWarn")
        stop_btn = self._btn("STOP", "BtnStop")
        pause_btn.clicked.connect(
            lambda _=False, wid=robot_id: self.pause_for(self._card_topic_rid(wid)))
        stop_btn.clicked.connect(
            lambda _=False, wid=robot_id: self.stop_for(self._card_topic_rid(wid)))
        mode_row.addWidget(pause_btn)
        mode_row.addWidget(stop_btn)
        setattr(self, f'{robot_id}_pause_btn', pause_btn)
        setattr(self, f'{robot_id}_stop_btn', stop_btn)

        close_btn = QPushButton("✕")
        close_btn.setObjectName("CloseBtn")
        close_btn.setFixedSize(28, 28)
        close_btn.setCursor(Qt.PointingHandCursor)
        close_btn.setToolTip(f"Remove {robot_name}")
        n_idx = int(robot_id.split("_")[1])
        close_btn.clicked.connect(
            lambda _=False, _n=n_idx: self._remove_robot_card(_n))
        mode_row.addWidget(close_btn)
        grp_layout.addLayout(mode_row)

        setattr(self, f'{robot_id}_mode_buttons', mode_buttons)
        setattr(self, f'{robot_id}_selected_mode_idx', 0)
        self._highlight_extra_mode_tab(robot_id, mode_buttons[0], "joint")

        mode_dd = QComboBox()
        mode_dd.addItems(CONTROL_MODE_NAMES)
        mode_dd.hide()

        # Reset button + station selector + per-card robot-type combo
        util_row = QHBoxLayout()
        reset_btn = self._btn("RESET DEFAULTS")
        reset_btn.clicked.connect(lambda: self._reset_extra_robot_defaults(robot_id))
        util_row.addWidget(reset_btn)

        station_lbl = QLabel("Station:")
        station_lbl.setObjectName("HintLabel")
        station_sb = QSpinBox()
        station_sb.setRange(1, 4)
        station_sb.setValue(station_n)
        station_sb.setToolTip("Which robot station this card controls")
        station_sb.valueChanged.connect(
            lambda new_n, wid=robot_id, g=grp: self._on_card_station_changed(wid, new_n, g))
        util_row.addWidget(station_lbl)
        util_row.addWidget(station_sb)
        setattr(self, f'{robot_id}_station_spinbox', station_sb)

        type_lbl = QLabel("Type:")
        type_lbl.setObjectName("HintLabel")
        type_dd = QComboBox()
        type_dd.addItems(list(self.robots.keys()))
        if selected_type in self.robots:
            type_dd.setCurrentText(selected_type)
        type_dd.currentTextChanged.connect(
            lambda t, rid=robot_id:
                self._on_card_type_changed(rid, self._card_topic_rid(rid), t))
        util_row.addWidget(type_lbl)
        util_row.addWidget(type_dd)
        setattr(self, f'{robot_id}_type_dropdown', type_dd)

        # Per-card tuning profile (parity with card 1's _rc_profile_dropdown).
        profile_lbl = QLabel("Config:")
        profile_lbl.setObjectName("HintLabel")
        profile_dd = QComboBox()
        profile_dd.setMinimumWidth(180)
        profile_dd.setToolTip(
            "Tuning profile for the currently selected mode. Profiles are\n"
            "robot-scoped; '(fallback)' means the chosen profile has no\n"
            "override for this mode and the loader will use 'default'.")
        profile_dd.currentIndexChanged.connect(
            lambda _i, rid=robot_id: self._on_extra_profile_changed(rid))
        util_row.addWidget(profile_lbl)
        util_row.addWidget(profile_dd)
        setattr(self, f'{robot_id}_profile_dropdown', profile_dd)

        util_row.addStretch()
        grp_layout.addLayout(util_row)

        # Null-space helpers (per-card)
        ns_row = QHBoxLayout()
        ns_row.setSpacing(8)
        ns_label = QLabel("Null-space:")
        ns_row.addWidget(ns_label)
        setattr(self, f'{robot_id}_nullspace_label', ns_label)

        post_cb = QCheckBox("Postural")
        post_cb.setToolTip("Null-space joint-target tracking for this robot.")
        post_cb.toggled.connect(
            lambda c, wid=robot_id: self.node.publish_postural_enable(
                bool(c), self._card_topic_rid(wid)))
        ns_row.addWidget(post_cb)
        setattr(self, f'{robot_id}_postural_toggle', post_cb)

        elbow_cb = QCheckBox("Elbow")
        elbow_cb.setToolTip("Null-space elbow hold for this robot.")
        elbow_cb.toggled.connect(
            lambda c, wid=robot_id: self.node.publish_elbow_enable(
                bool(c), self._card_topic_rid(wid)))
        ns_row.addWidget(elbow_cb)
        setattr(self, f'{robot_id}_elbow_toggle', elbow_cb)

        hpmp_cb = QCheckBox("H-PMP")
        hpmp_cb.setToolTip("H-PMP: the Cartesian modes (5-8) nested inside the joint HAP-FIC. "
                             "On by default; off sends the Cartesian torques directly.")
        hpmp_cb.setChecked(True)
        hpmp_cb.toggled.connect(
            lambda c, wid=robot_id: self.node.publish_hpmp_enable(
                bool(c), self._card_topic_rid(wid)))
        ns_row.addWidget(hpmp_cb)
        setattr(self, f'{robot_id}_hpmp_toggle', hpmp_cb)

        ns_row.addStretch()
        grp_layout.addLayout(ns_row)

        # Position + Joysticks
        pos_grp = QGroupBox("Position (m) & Joysticks")
        pos_vbox = QVBoxLayout(pos_grp)
        pos_vbox.setSpacing(8)

        pos_grid = QGridLayout()
        pos_grid.setSpacing(5)
        pos_x = QLineEdit(def_pos[0])
        pos_y = QLineEdit(def_pos[1])
        pos_z = QLineEdit(def_pos[2])
        for i, (lbl_text, widget) in enumerate([("X", pos_x), ("Y", pos_y), ("Z", pos_z)]):
            pos_grid.addWidget(QLabel(lbl_text + ":"), i, 0)
            pos_grid.addWidget(widget, i, 1)
        pos_vbox.addLayout(pos_grid)

        joy_row = QHBoxLayout()
        xy_col = QVBoxLayout()
        xy_lbl = QLabel("X / Y")
        xy_lbl.setAlignment(Qt.AlignCenter)
        xy_lbl.setObjectName("JoystickLabel")
        joy_xy = VirtualJoystick()
        xy_col.addWidget(xy_lbl)
        xy_col.addWidget(joy_xy)
        z_col = QVBoxLayout()
        z_lbl = QLabel("Z (Up/Down)")
        z_lbl.setAlignment(Qt.AlignCenter)
        z_lbl.setObjectName("JoystickLabel")
        joy_z = VirtualJoystick()
        z_col.addWidget(z_lbl)
        z_col.addWidget(joy_z)
        joy_row.addLayout(xy_col)
        joy_row.addLayout(z_col)
        pos_vbox.addLayout(joy_row)

        joy_autopub = QCheckBox("Live Publish on Joystick Move")
        joy_autopub.setChecked(False)
        pos_vbox.addWidget(joy_autopub)

        def handle_joy_xy(dx, dy, wid=robot_id):
            speed = 0.005
            try:
                cx = float(pos_x.text())
                cy = float(pos_y.text())
                pos_x.setText(f"{cx + (dx * speed):.4f}")
                pos_y.setText(f"{cy + (dy * speed):.4f}")
                if joy_autopub.isChecked():
                    self.send_pose_for(self._card_topic_rid(wid))
            except ValueError:
                pass

        def handle_joy_z(dx, dy, wid=robot_id):
            speed = 0.005
            try:
                cz = float(pos_z.text())
                pos_z.setText(f"{cz + (dy * speed):.4f}")
                if joy_autopub.isChecked():
                    self.send_pose_for(self._card_topic_rid(wid))
            except ValueError:
                pass

        joy_xy.joystickMoved.connect(handle_joy_xy)
        joy_z.joystickMoved.connect(handle_joy_z)

        # Axis-angle group
        aa_grp = QGroupBox("Orientation — Axis / Angle")
        aa_layout = QGridLayout(aa_grp)
        aa_layout.setSpacing(5)
        axis_x = QLineEdit(def_axis[0])
        axis_y = QLineEdit(def_axis[1])
        axis_z = QLineEdit(def_axis[2])
        angle_le = QLineEdit(def_angle_str)
        angle_units = QComboBox()
        angle_units.addItems(["deg", "rad"])
        q_preview = QLabel("QUAT  [ 0.000000,  0.000000,  0.000000,  1.000000 ]")
        q_preview.setObjectName("QuatPreview")
        convert_btn = self._btn("PREVIEW  AXIS→QUAT", "BtnTeal")

        for i, (lbl_text, widget) in enumerate([
            ("ax:", axis_x), ("ay:", axis_y), ("az:", axis_z), ("θ:", angle_le)
        ]):
            aa_layout.addWidget(QLabel(lbl_text), i, 0)
            aa_layout.addWidget(widget, i, 1)
        aa_layout.addWidget(QLabel("Units:"), 4, 0)
        aa_layout.addWidget(angle_units, 4, 1)
        aa_layout.addWidget(convert_btn, 5, 0, 1, 2)
        aa_layout.addWidget(q_preview, 6, 0, 1, 2)

        side_row = QHBoxLayout()
        side_row.addWidget(pos_grp)
        side_row.addWidget(aa_grp)
        grp_layout.addLayout(side_row)

        # Joint angles
        joint_grp = QGroupBox("Joint Angles")
        joint_layout = QVBoxLayout(joint_grp)
        joint_layout.setSpacing(5)

        jt_bar = QHBoxLayout()
        jt_bar.setSpacing(8)

        unit_frame = QFrame()
        unit_frame.setObjectName("UnitFrame")
        unit_row_l = QHBoxLayout(unit_frame)
        unit_row_l.setContentsMargins(0, 0, 0, 0)
        unit_row_l.setSpacing(0)
        u_rad = QPushButton("RAD"); u_rad.setObjectName("UnitBtn")
        u_deg = QPushButton("DEG"); u_deg.setObjectName("UnitBtn")
        u_deg.setProperty("active", True)
        unit_row_l.addWidget(u_rad)
        unit_row_l.addWidget(u_deg)

        units_cb = QComboBox()
        units_cb.addItems(["deg", "rad"])
        units_cb.hide()

        def _toggle_unit(unit, r=u_rad, d=u_deg, cb=units_cb, rid=robot_id):
            cb.setCurrentText(unit)
            r.setProperty("active", unit == "rad")
            d.setProperty("active", unit == "deg")
            r.style().unpolish(r); r.style().polish(r)
            d.style().unpolish(d); d.style().polish(d)
            self._update_extra_joint_display_units(rid)

        u_rad.clicked.connect(lambda: _toggle_unit("rad"))
        u_deg.clicked.connect(lambda: _toggle_unit("deg"))

        jt_bar.addWidget(unit_frame)
        autopub = QCheckBox("Auto-publish")
        jt_bar.addWidget(autopub)
        jt_bar.addStretch()

        count_label = QLabel("Joints:")
        count_label.setObjectName("HintLabel")
        count_sb = QSpinBox()
        count_sb.setRange(1, 12)
        count_sb.setValue(num_joints)
        jt_bar.addWidget(count_label)
        jt_bar.addWidget(count_sb)

        pub_joints_btn = self._btn("PUBLISH JOINTS", "BtnPrimary")
        jt_bar.addWidget(pub_joints_btn)
        joint_layout.addLayout(jt_bar)

        container = QWidget()
        jvbox = QVBoxLayout(container)
        jvbox.setContentsMargins(0, 0, 0, 0)
        jvbox.setSpacing(4)
        joint_layout.addWidget(container)
        grp_layout.addWidget(joint_grp)

        # Gripper
        gripper_grp = QGroupBox("Gripper Control (0% Closed - 100% Open)")
        gripper_layout = QHBoxLayout(gripper_grp)
        gripper_layout.setSpacing(10)
        gripper_lbl = QLabel("Width:"); gripper_lbl.setObjectName("HintLabel")
        gripper_slider = QSlider(Qt.Horizontal)
        gripper_slider.setRange(0, 100); gripper_slider.setValue(100); gripper_slider.setSingleStep(1)
        gripper_val_lbl = QLabel("100 %")
        gripper_val_lbl.setFixedWidth(45)
        gripper_val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        gripper_val_lbl.setObjectName("AccentValue")

        def on_gripper_changed(val, wid=robot_id):
            gripper_val_lbl.setText(f"{val} %")
            actual_width = (val / 100.0) * 0.08
            self.node.publish_gripper_width(self._card_topic_rid(wid), actual_width)

        gripper_slider.valueChanged.connect(on_gripper_changed)
        gripper_layout.addWidget(gripper_lbl)
        gripper_layout.addWidget(gripper_slider, 1)
        gripper_layout.addWidget(gripper_val_lbl)
        grp_layout.addWidget(gripper_grp)

        # Send Pose row
        pose_row_widget = QWidget()
        pose_row_inner = QHBoxLayout(pose_row_widget)
        pose_row_inner.setContentsMargins(0, 0, 0, 0)
        pose_row_inner.addStretch()
        send_pose_btn = self._btn("SEND POSE / RESUME", "BtnTeal")
        send_pose_btn.clicked.connect(
            lambda _=False, wid=robot_id: self.send_pose_for(self._card_topic_rid(wid)))
        pose_row_inner.addWidget(send_pose_btn)
        grp_layout.addWidget(pose_row_widget)

        # Store references
        setattr(self, f'{robot_id}_position_group', pos_grp)
        setattr(self, f'{robot_id}_aa_group', aa_grp)
        setattr(self, f'{robot_id}_joint_group', joint_grp)
        setattr(self, f'{robot_id}_gripper_group', gripper_grp)
        setattr(self, f'{robot_id}_pose_row', pose_row_widget)
        setattr(self, f'{robot_id}_mode_dropdown', mode_dd)
        setattr(self, f'{robot_id}_pos_x', pos_x)
        setattr(self, f'{robot_id}_pos_y', pos_y)
        setattr(self, f'{robot_id}_pos_z', pos_z)
        setattr(self, f'{robot_id}_axis_x', axis_x)
        setattr(self, f'{robot_id}_axis_y', axis_y)
        setattr(self, f'{robot_id}_axis_z', axis_z)
        setattr(self, f'{robot_id}_angle', angle_le)
        setattr(self, f'{robot_id}_angle_units', angle_units)
        setattr(self, f'{robot_id}_q_preview', q_preview)
        setattr(self, f'{robot_id}_gripper_slider', gripper_slider)
        setattr(self, f'{robot_id}_joy_xy', joy_xy)
        setattr(self, f'{robot_id}_joy_z', joy_z)
        setattr(self, f'{robot_id}_joy_autopub', joy_autopub)

        self.real_joint_units[robot_id] = units_cb
        self.real_joint_autopublish[robot_id] = autopub
        self.real_joint_count[robot_id] = count_sb
        self.real_joint_container[robot_id] = container
        self.real_joint_layout[robot_id] = jvbox
        self.real_joint_sliders[robot_id] = []
        self.real_joint_spinboxes[robot_id] = []

        # Wiring
        convert_btn.clicked.connect(
            lambda _=False, rid=robot_id: self.convert_axis_angle_preview(rid))
        count_sb.valueChanged.connect(
            lambda n, rid=robot_id: self._build_extra_joint_sliders(
                rid, n, robot_type=self.card_robot_type(rid)))
        pub_joints_btn.clicked.connect(
            lambda _=False, rid=robot_id:
                self._publish_extra_joint_angles(rid, self._card_topic_rid(rid)))

        self._build_extra_joint_sliders(robot_id, count_sb.value(),
                                         default_values=def_joints,
                                         robot_type=selected_type)
        self._update_extra_section_visibility(robot_id, CONTROL_MODE_NAMES[0])
        self._refresh_extra_card_profile_dropdown(robot_id)
        return grp

    # ──────────────────────────────────────────────────────────────────────
    #  Extra robot mode / visibility
    # ──────────────────────────────────────────────────────────────────────

    def _on_extra_mode_selected(self, robot_id, mode_idx, kind):
        hidden_dd = getattr(self, f'{robot_id}_mode_dropdown', None)
        if hidden_dd:
            hidden_dd.setCurrentIndex(mode_idx)
        setattr(self, f'{robot_id}_selected_mode_idx', mode_idx)
        buttons = getattr(self, f'{robot_id}_mode_buttons', [])
        if buttons:
            self._highlight_extra_mode_tab(robot_id, buttons[mode_idx], kind)
        self._update_extra_section_visibility(robot_id, CONTROL_MODE_NAMES[mode_idx])
        topic_rid = self._card_topic_rid(robot_id)
        self.node.publish_mode(mode_idx + 1, robot_id=topic_rid)
        # Profile is robot-scoped — list is stable, but the "(fallback)"
        # decoration depends on mode, so refresh and re-publish for parity
        # with card 1's _on_rc_mode_selected.
        self._refresh_extra_card_profile_dropdown(robot_id)
        self._publish_extra_card_profile(robot_id)
        self.log_message("INFO", f"Mode → {CONTROL_MODE_NAMES[mode_idx]}", robot_id)

    def _highlight_extra_mode_tab(self, robot_id, active_btn, kind):
        buttons = getattr(self, f'{robot_id}_mode_buttons', [])
        for b in buttons:
            b.setProperty("active_j", False); b.setProperty("active_c", False)
            b.style().unpolish(b); b.style().polish(b)
        if kind == "joint":
            active_btn.setProperty("active_j", True)
        elif kind == "cart":
            active_btn.setProperty("active_c", True)
        active_btn.style().unpolish(active_btn); active_btn.style().polish(active_btn)

    def _update_extra_section_visibility(self, robot_id, mode_text):
        kind = self._classify_mode(mode_text)
        is_hw = (self.control_target == "hardware")
        show_cart = (kind == "cart")
        show_joint = (kind == "joint")
        for attr, show in [
            (f'{robot_id}_position_group', show_cart),
            (f'{robot_id}_aa_group',       show_cart),
            (f'{robot_id}_pose_row',       show_cart and is_hw),
            (f'{robot_id}_joint_group',    show_joint),
            (f'{robot_id}_pause_btn',      is_hw),
            (f'{robot_id}_stop_btn',       is_hw),
            (f'{robot_id}_postural_toggle', show_cart),
            (f'{robot_id}_elbow_toggle',    show_cart),
            (f'{robot_id}_hpmp_toggle',   show_cart),
            (f'{robot_id}_nullspace_label', show_cart),
        ]:
            widget = getattr(self, attr, None)
            if widget:
                widget.setVisible(show)

    def _classify_mode(self, mode_text: str) -> str:
        t = mode_text.lower()
        if any(x in t for x in ["cartesian", "cart"]):
            return "cart"
        return "joint"

    # ──────────────────────────────────────────────────────────────────────
    #  Extra robot joint sliders
    # ──────────────────────────────────────────────────────────────────────

    def _build_extra_joint_sliders(self, robot_id, num_joints,
                                    default_values=None, robot_type=None):
        if robot_type is None:
            robot_type = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_type, {})
        limits = cfg.get('joint_limits_deg', {})
        lower_list = limits.get('lower', [])
        upper_list = limits.get('upper', [])

        vbox = self.real_joint_layout[robot_id]
        while vbox.count():
            item = vbox.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

        self.real_joint_sliders[robot_id] = []
        self.real_joint_spinboxes[robot_id] = []

        for i in range(num_joints):
            lo_deg = lower_list[i] if i < len(lower_list) else -180
            hi_deg = upper_list[i] if i < len(upper_list) else 180
            lo_cr = int(math.radians(lo_deg) * 100)
            hi_cr = int(math.radians(hi_deg) * 100)

            row_w = QWidget()
            row = QHBoxLayout(row_w)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(8)

            lbl = QLabel(f"J{i+1}")
            lbl.setFixedWidth(24)
            lbl.setObjectName("HintLabel")
            row.addWidget(lbl)

            sld = QSlider(Qt.Horizontal)
            sld.setRange(lo_cr, hi_cr)
            sld.setSingleStep(1); sld.setPageStep(10); sld.setValue(0)
            sld.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            sld.setToolTip(f"J{i+1}: [{lo_deg:.1f}° to {hi_deg:.1f}°]")
            row.addWidget(sld, 1)

            spin = QDoubleSpinBox()
            spin.setRange(lo_deg, hi_deg)
            spin.setSingleStep(1.0); spin.setDecimals(2)
            spin.setMinimumWidth(80); spin.setAlignment(Qt.AlignRight)
            spin.setValue(0.0)
            row.addWidget(spin)

            if default_values and i < len(default_values):
                initial = max(lo_deg, min(hi_deg, default_values[i]))
                spin.setValue(initial)
                sld.setValue(max(lo_cr, min(hi_cr, int(math.radians(initial) * 100.0))))

            sld.valueChanged.connect(
                lambda val, r=robot_id, s=spin, sl=sld:
                    self._on_extra_slider_changed(r, val, s, sl))
            spin.valueChanged.connect(
                lambda val, r=robot_id, s=spin, sl=sld:
                    self._on_extra_spinbox_changed(r, val, s, sl))

            vbox.addWidget(row_w)
            self.real_joint_sliders[robot_id].append(sld)
            self.real_joint_spinboxes[robot_id].append(spin)
        vbox.addStretch(1)

    def _on_extra_slider_changed(self, robot_id, value_centi_rad, spinbox, slider):
        rad_val = value_centi_rad / 100.0
        units = self.real_joint_units[robot_id].currentText()
        display = math.degrees(rad_val) if units == "deg" else rad_val
        spinbox.blockSignals(True)
        spinbox.setValue(display)
        spinbox.blockSignals(False)
        if self.real_joint_autopublish[robot_id].isChecked():
            self._publish_extra_joint_angles(robot_id, self._card_topic_rid(robot_id))

    def _on_extra_spinbox_changed(self, robot_id, value, spinbox, slider):
        units = self.real_joint_units[robot_id].currentText()
        rad_val = math.radians(value) if units == "deg" else value
        slider.blockSignals(True)
        slider.setValue(int(rad_val * 100.0))
        slider.blockSignals(False)
        if self.real_joint_autopublish[robot_id].isChecked():
            self._publish_extra_joint_angles(robot_id, self._card_topic_rid(robot_id))

    def _update_extra_joint_display_units(self, robot_id):
        units = self.real_joint_units[robot_id].currentText()
        sliders = self.real_joint_sliders.get(robot_id, [])
        spinboxes = self.real_joint_spinboxes.get(robot_id, [])
        for sld, spin in zip(sliders, spinboxes):
            rad_val = sld.value() / 100.0
            display = math.degrees(rad_val) if units == "deg" else rad_val
            spin.blockSignals(True)
            spin.setValue(display)
            spin.blockSignals(False)

    def _publish_extra_joint_angles(self, robot_id, topic_robot_id):
        positions_rad = [s.value() / 100.0 for s in self.real_joint_sliders.get(robot_id, [])]
        self.node.publish_real_joint_angles(topic_robot_id, positions_rad)
        degrees = [math.degrees(r) for r in positions_rad]
        deg_str = ", ".join([f"{d:.1f}°" for d in degrees])
        self.log_message("INFO", f"Joints: [{deg_str}]", robot_id)

    def _on_card_type_changed(self, robot_id: str, topic_robot_id: str, new_type: str):
        """User picked a different robot type for this card. Update sliders,
        joint count, and default position to match the new type."""
        self._card_robot_types[robot_id] = new_type
        cfg = getattr(self, 'robot_configs', {}).get(new_type, {})
        num_joints = cfg.get('num_joints', 7)
        def_joints = cfg.get('q_home_deg', [0.0] * num_joints)
        if len(def_joints) < num_joints:
            def_joints = def_joints + [0.0] * (num_joints - len(def_joints))

        count_sb = self.real_joint_count.get(robot_id)
        if count_sb is not None:
            count_sb.blockSignals(True)
            count_sb.setValue(num_joints)
            count_sb.blockSignals(False)

        self._build_extra_joint_sliders(robot_id, num_joints,
                                         default_values=def_joints,
                                         robot_type=new_type)

        ee = cfg.get('default_ee_target', {})
        _pos = ee.get('position', [0.0, 0.0, 0.0])
        if hasattr(self, f'{robot_id}_pos_x'):
            getattr(self, f'{robot_id}_pos_x').setText(str(_pos[0]))
            getattr(self, f'{robot_id}_pos_y').setText(str(_pos[1]))
            getattr(self, f'{robot_id}_pos_z').setText(str(_pos[2]))

        # Profile list is robot-scoped — repopulate for the new robot type.
        self._refresh_extra_card_profile_dropdown(robot_id)
        self._publish_extra_card_profile(robot_id)

        self.log_message("INFO", f"Type → {new_type}", self._card_topic_rid(robot_id))

    def _on_card_station_changed(self, robot_id: str, new_station: int, grp: QGroupBox):
        """User changed which station this card controls."""
        old_station = self._card_station.get(robot_id)
        if old_station == new_station:
            return
        self._card_station[robot_id] = new_station

        # Publishers only — no subscriptions (see _add_robot_card comment).
        self.node.add_robot(new_station)

        card_n = robot_id.split("_")[1]
        grp.setTitle(f"Robot {card_n} — Station {new_station}")
        self.log_message(
            "INFO",
            f"Card {card_n} now targets Station {new_station}",
            f"robot{new_station}")

    # ── Per-card tuning profile helpers (parity with card 1) ───────────────
    def _extra_card_current_profile(self, robot_id: str) -> str:
        dd = getattr(self, f'{robot_id}_profile_dropdown', None)
        if dd is None:
            return "default"
        # Display text may carry "(fallback)" — itemData holds the clean name.
        data = dd.currentData()
        if data:
            return data
        name = dd.currentText()
        return name if name else "default"

    def _refresh_extra_card_profile_dropdown(self, robot_id: str):
        """Populate this card's profile combo for its robot type and current mode."""
        dd = getattr(self, f'{robot_id}_profile_dropdown', None)
        if dd is None:
            return
        try:
            from fic_studio.xml_manager import XMLManager
            mgr = XMLManager()
            robot = self.card_robot_type(robot_id)
            mode_idx = getattr(self, f'{robot_id}_selected_mode_idx', 0)
            mode = mode_idx + 1
            profiles = mgr.get_all_profiles(robot) or ["default"]
            with_override = set(mgr.get_available_profiles(robot, mode))
        except Exception:
            profiles = ["default"]
            with_override = {"default"}

        prev = dd.currentData() or dd.currentText()
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

    def _on_extra_profile_changed(self, robot_id: str):
        self._publish_extra_card_profile(robot_id)
        self.log_message("INFO",
            f"Active config → {self._extra_card_current_profile(robot_id)}",
            robot_id)

    def _publish_extra_card_profile(self, robot_id: str):
        """Publish to robot N's own active_profile topic (not card 1's)."""
        try:
            from std_msgs.msg import String
            topic_rid = self._card_topic_rid(robot_id)
            pubs = self.node.pubs.get(topic_rid, {})
            pub = pubs.get('active_profile')
            if pub is None:
                return
            msg = String()
            msg.data = self._extra_card_current_profile(robot_id) or "default"
            pub.publish(msg)
        except Exception:
            pass

    def _reset_extra_robot_defaults(self, robot_id):
        selected_type = self.card_robot_type(robot_id)
        cfg = getattr(self, 'robot_configs', {}).get(selected_type, {})
        num_joints = cfg.get('num_joints', 7)
        ee = cfg.get('default_ee_target', {})
        _pos = ee.get('position', [0.0, 0.0, 0.0])
        def_joints = cfg.get('q_home_deg', [0.0] * num_joints)

        if selected_type == "Franka_FR3":
            def_axis, def_angle_str = ["-1.0", "0.0", "0.0"], "180.0"
        elif selected_type == "KinovaGen3_7DOF":
            def_axis, def_angle_str = ["0.5773503", "0.5773503", "0.5773503"], "119.999999863"
        else:
            def_axis, def_angle_str = ["0.0", "0.0", "1.0"], "0.0"

        getattr(self, f'{robot_id}_pos_x').setText(str(_pos[0]))
        getattr(self, f'{robot_id}_pos_y').setText(str(_pos[1]))
        getattr(self, f'{robot_id}_pos_z').setText(str(_pos[2]))
        getattr(self, f'{robot_id}_axis_x').setText(def_axis[0])
        getattr(self, f'{robot_id}_axis_y').setText(def_axis[1])
        getattr(self, f'{robot_id}_axis_z').setText(def_axis[2])
        getattr(self, f'{robot_id}_angle').setText(def_angle_str)

        count = self.real_joint_count[robot_id].value()
        self._build_extra_joint_sliders(robot_id, count,
                                         default_values=def_joints,
                                         robot_type=selected_type)

        buttons = getattr(self, f'{robot_id}_mode_buttons', [])
        if buttons:
            self._highlight_extra_mode_tab(robot_id, buttons[0], "joint")
        hidden_dd = getattr(self, f'{robot_id}_mode_dropdown', None)
        if hidden_dd:
            hidden_dd.setCurrentIndex(0)
        self._update_extra_section_visibility(robot_id, CONTROL_MODE_NAMES[0])
        self.convert_axis_angle_preview(robot_id)

    # ──────────────────────────────────────────────────────────────────────
    #  Navigation
    # ──────────────────────────────────────────────────────────────────────

