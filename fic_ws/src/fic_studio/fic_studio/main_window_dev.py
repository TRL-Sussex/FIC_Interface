# main_window_dev.py — FIC Dev Console: lightweight hardware-focused GUI
#
# A stripped-down interface for direct hardware control, tuning, and
# diagnostics, for a second PC sitting next to a real robot.
#
# Teleoperation is NOT here any more. It lives in the Pro window's Multi-Robot
# page (fic_studio/teleop/), which is the only implementation.

import os
import math
import subprocess

from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QFrame, QTabWidget, QToolBar, QStatusBar, QAction,
    QSizePolicy, QMessageBox, QGroupBox, QGridLayout, QLineEdit,
    QDoubleSpinBox, QScrollArea, QListView,
)

from fic_studio.ros_node import FicStudioNode
from fic_studio.widgets import StatusLED
from fic_studio import theme

# Mixins
from fic_studio.mixins.tuning_mixin import TuningMixin
from fic_studio.mixins.process_mixin import ProcessMixin
from fic_studio.mixins.publisher_mixin import PublisherMixin
from fic_studio.mixins.settings_mixin import SettingsMixin, LogViewer
from fic_studio.mixins.robot_config_mixin import RobotConfigMixin
from fic_studio.mixins.safety_mixin import SafetyMixin


# ═══════════════════════════════════════════════════════════════════════════════
#  CONTROL MODE NAMES
# ═══════════════════════════════════════════════════════════════════════════════

CONTROL_MODES = {
    1: "Joint Impedance",
    2: "Joint Impedance + Planner",
    3: "Joint Admittance",
    4: "Joint Admittance Haptic",
    5: "Cartesian Impedance",
    6: "Cartesian Planner",
    7: "Cartesian Admittance",
    8: "Cartesian Admittance Haptic",
}


# ═══════════════════════════════════════════════════════════════════════════════
#  FIC DEV CONSOLE
# ═══════════════════════════════════════════════════════════════════════════════

class FICDevConsole(
    QMainWindow, TuningMixin, ProcessMixin, PublisherMixin,
    SettingsMixin, RobotConfigMixin, SafetyMixin,
):
    APP_NAME = "FIC Dev Console"
    APP_VERSION = "5.10.0"

    def __init__(self, node: FicStudioNode):
        super().__init__()
        self.node = node
        self.setWindowTitle(f"{self.APP_NAME} — {self.APP_VERSION}")
        self.resize(1100, 700)
        self.setMinimumSize(800, 450)

        # ── Settings & theme ──
        self._init_settings()
        self.apply_theme(self.current_theme)

        # ── Process state (hardware only — no simulation_process) ──
        self.controller_process = None
        self.fic_process = None
        self.vr_relay_process = None
        self.vr_fic_process = None
        self.bags_process = None
        self.plotter_process = None
        self.driver_process = None
        self.current_bag_path = None

        self.controller_running = False
        self.fic_running = False
        self.vr_relay_running = False
        self.vr_fic_running = False
        self.bags_running = False
        self.plotter_running = False
        self.driver_running = False
        # ProcessMixin expects these even if unused
        self.simulation_process = None
        self.simulation_running = False

        # ── Robot configs ──
        self.robots = {
            "Franka_FR3": 7, "KinovaGen3_7DOF": 7,
        }
        self.real_joint_sliders = {}
        self.real_joint_spinboxes = {}
        self.real_joint_units = {}
        self.real_joint_autopublish = {}
        self.real_joint_count = {}
        self.real_joint_container = {}
        self.real_joint_layout = {}
        self._side_buttons = []

        self._setup_gripper_publishers()

        self.robot_configs = self._load_robot_configs()
        if self.robot_configs:
            self.robots = {k: v.get('num_joints', 7) for k, v in self.robot_configs.items()}
        try:
            from fic_studio.xml_manager import XMLManager
            XMLManager().ensure_all_robots(self.robots)
        except Exception as e:
            print(f"[XMLManager] Warning: {e}")

        # ── Mixin inits ──
        self._init_safety()

        # ── Live data state ──
        self._num_joints = 7
        self._current_mode = 1
        self._live_q = [0.0] * 7
        self._live_ee = [0.0, 0.0, 0.0]

        # SafetyMixin expects these for limit checking
        self.current_joint_positions = [0.0] * 7
        self.current_ee_pose = None

        # ── ROS2 subscribers (station-specific) ──
        from std_msgs.msg import Float64MultiArray, Float32MultiArray
        rid = self.node.station_robot_id
        self.node.create_subscription(
            Float64MultiArray, f'{rid}_trajectory_data',
            self._on_trajectory_data, 10)
        self.node.create_subscription(
            Float32MultiArray, f'{rid}_q_current',
            self._on_q_current, 10)

        # ── Build UI ──
        self._build_ui()

        # ── Periodic status update ──
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(500)

        # ── Live readout timer (20 Hz) ──
        self._readout_timer = QTimer()
        self._readout_timer.timeout.connect(self._refresh_live_readout)
        self._readout_timer.start(50)

        # ── Select default robot ──
        self._refresh_robots_dropdown()
        default_robot = self.settings['general']['default_robot']
        idx = self.robot_dropdown.findText(default_robot)
        if idx >= 0:
            self.robot_dropdown.setCurrentIndex(idx)

        self.log_message("INFO", f"{self.APP_NAME} started", "System")
        self.log_message("INFO", f"Loaded {len(self.robot_configs)} robot(s)", "System")

    # ══════════════════════════════════════════════════════════════════════════
    #  UI HELPERS (same pattern as main_window_pro.py)
    # ══════════════════════════════════════════════════════════════════════════

    def _hr(self):
        line = QFrame()
        line.setObjectName("Divider")
        line.setFrameShape(QFrame.HLine)
        return line

    def _section_label(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setObjectName("SideSection")
        return lbl

    def _section_header(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setObjectName("SectionLabel")
        return lbl

    def _page_title(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setObjectName("PageTitle")
        return lbl

    def _btn(self, text: str, style_id: str = None) -> QPushButton:
        btn = QPushButton(text)
        btn.setCursor(Qt.PointingHandCursor)
        if style_id:
            btn.setObjectName(style_id)
        return btn

    # ══════════════════════════════════════════════════════════════════════════
    #  BUILD UI
    # ══════════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        self._build_toolbar()
        self._build_central()
        self._build_statusbar()

    # ── TOOLBAR ──────────────────────────────────────────────────────────────

    @staticmethod
    def _tb_styles(t):
        """Return toolbar-button style strings keyed by state, using theme tokens."""
        run = theme.button_qss(t, 'accent', t["accent_red"])
        idle = theme.button_qss(t, 'secondary')
        return {
            'hw': idle, 'hw_run': run, 'vr': idle, 'vr_run': run,
            'util': idle, 'util_run': run,
        }

    def _restyle_btn(self, btn: QPushButton, text: str, running: bool, kind: str):
        style_key = f'{kind}_run' if running else kind
        new_style = self._tb_styles(theme.current())[style_key]
        if btn.styleSheet() != new_style:
            btn.setStyleSheet(new_style)
        if btn.text() != text:
            btn.setText(text)

    def _build_toolbar(self):
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        toolbar.setIconSize(QSize(20, 20))
        self.addToolBar(toolbar)
        self.main_toolbar = toolbar

        # Robot dropdown
        toolbar.addWidget(QLabel(" Robot: "))
        self.robot_dropdown = QComboBox()
        self.robot_dropdown.setMinimumWidth(130)
        self.robot_dropdown.setFixedHeight(26)
        self.robot_dropdown.addItems(self.robots.keys())
        self.robot_dropdown.currentTextChanged.connect(self._on_robot_changed)
        toolbar.addWidget(self.robot_dropdown)
        toolbar.addSeparator()

        # Mode dropdown
        toolbar.addWidget(QLabel(" Mode: "))
        self.mode_dropdown = QComboBox()
        for mode_id, name in CONTROL_MODES.items():
            self.mode_dropdown.addItem(f"{mode_id}: {name}")
        self.mode_dropdown.setFixedHeight(26)
        self.mode_dropdown.setMinimumWidth(180)
        self.mode_dropdown.currentIndexChanged.connect(self._on_mode_changed)
        toolbar.addWidget(self.mode_dropdown)

        send_mode_btn = QPushButton("Send")
        send_mode_btn.setFixedSize(55, 26)
        send_mode_btn.clicked.connect(self._send_mode)
        toolbar.addWidget(send_mode_btn)
        toolbar.addSeparator()

        # Process buttons
        hw_label = QLabel(" HW: ")
        hw_label.setObjectName("BlueLabel")
        toolbar.addWidget(hw_label)

        self.tb_driver_btn = QPushButton("▶ API")
        theme.bind(
            self.tb_driver_btn,
            lambda t, s=self: self._tb_styles(t)[
                'hw_run' if s.driver_running else 'hw'
            ],
        )
        self.tb_driver_btn.setToolTip("Start/Stop Robot Driver (F9)")
        self.tb_driver_btn.setFixedSize(80, 26)
        self.tb_driver_btn.clicked.connect(self._toggle_driver)
        toolbar.addWidget(self.tb_driver_btn)

        self.tb_ctrl_btn = QPushButton("▶ FIC")
        theme.bind(
            self.tb_ctrl_btn,
            lambda t, s=self: self._tb_styles(t)[
                'hw_run' if s.controller_running else 'hw'
            ],
        )
        self.tb_ctrl_btn.setToolTip("Start/Stop Hardware Controller (F8)")
        self.tb_ctrl_btn.setFixedSize(80, 26)
        self.tb_ctrl_btn.clicked.connect(self._toggle_controller)
        toolbar.addWidget(self.tb_ctrl_btn)
        toolbar.addSeparator()

        self.tb_bags_btn = QPushButton("▶ BAG")
        theme.bind(
            self.tb_bags_btn,
            lambda t, s=self: self._tb_styles(t)[
                'util_run' if s.bags_running else 'util'
            ],
        )
        self.tb_bags_btn.setToolTip("Start/Stop ROS Bag Recording")
        self.tb_bags_btn.setFixedSize(80, 26)
        self.tb_bags_btn.clicked.connect(self._toggle_bags)
        toolbar.addWidget(self.tb_bags_btn)

        self.tb_plot_btn = QPushButton("▶ PLT")
        theme.bind(
            self.tb_plot_btn,
            lambda t, s=self: self._tb_styles(t)[
                'util_run' if s.plotter_running else 'util'
            ],
        )
        self.tb_plot_btn.setToolTip("Start/Stop Plotter")
        self.tb_plot_btn.setFixedSize(80, 26)
        self.tb_plot_btn.clicked.connect(self._toggle_plotter)
        toolbar.addWidget(self.tb_plot_btn)
        toolbar.addSeparator()

        self.tb_vr_btn = QPushButton("▶ VR")
        theme.bind(
            self.tb_vr_btn,
            lambda t, s=self: self._tb_styles(t)[
                'vr_run' if getattr(s, 'vr_relay_running', False) else 'vr'
            ],
        )
        self.tb_vr_btn.setToolTip("Auto-detect from headset")
        self.tb_vr_btn.setFixedSize(90, 26)
        self.tb_vr_btn.clicked.connect(self._toggle_vr)
        toolbar.addWidget(self.tb_vr_btn)
        toolbar.addSeparator()

        # Spacer
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        spacer.setAttribute(Qt.WA_TransparentForMouseEvents)
        toolbar.addWidget(spacer)

        # E-STOP
        self.estop_btn = QPushButton("■ E-STOP")
        theme.bind(
            self.estop_btn,
            lambda t: theme.button_qss(t, 'safety'),
        )
        self.estop_btn.setFixedSize(95, 26)
        self.estop_btn.setToolTip("Emergency Stop (Escape)")
        self.estop_btn.clicked.connect(self._emergency_stop)
        toolbar.addWidget(self.estop_btn)

        # Aliases for ProcessMixin compatibility
        self.controller_toggle_btn = self.tb_ctrl_btn
        self.send_start_mode_btn = self.tb_driver_btn
        self.bags_toggle_btn = self.tb_bags_btn
        self.plotter_toggle_btn = self.tb_plot_btn
        # Sim buttons — create dummies so ProcessMixin doesn't crash
        self.fic_toggle_btn = QPushButton()
        self.simulation_toggle_btn = QPushButton()
        self.vr_toggle_btn = self.tb_vr_btn

    # ── CENTRAL AREA ─────────────────────────────────────────────────────────

    def _build_central(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_control_tab(), "Control")
        self.tabs.addTab(self._build_tuning_page(), "Tuning")
        self.tabs.addTab(self._build_settings_page(), "Settings")
        root.addWidget(self.tabs, 1)

        self.log_viewer = LogViewer()
        self.log_viewer.setFixedHeight(self.settings['general']['log_viewer_height'])
        self.log_viewer.setVisible(self.settings['general']['show_log_viewer'])
        root.addWidget(self.log_viewer)

    # ── CONTROL TAB ──────────────────────────────────────────────────────────

    def _build_control_tab(self) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 16, 20, 16)
        layout.setSpacing(12)

        layout.addWidget(self._page_title("Hardware Control"))

        # ── Joint Target Group ──
        self._joint_group = QGroupBox("JOINT TARGETS (degrees)")
        jg_layout = QVBoxLayout(self._joint_group)

        self._joint_grid = QGridLayout()
        self._joint_grid.setSpacing(6)
        self._joint_fields = []
        for i in range(7):
            lbl = QLabel(f"J{i+1}:")
            lbl.setFixedWidth(30)
            field = QLineEdit("0.0")
            field.setFixedWidth(80)
            self._joint_grid.addWidget(lbl, i // 4, (i % 4) * 2)
            self._joint_grid.addWidget(field, i // 4, (i % 4) * 2 + 1)
            self._joint_fields.append(field)
        jg_layout.addLayout(self._joint_grid)

        jg_btn_row = QHBoxLayout()
        send_joints_btn = self._btn("SEND JOINTS", "BtnPrimary")
        send_joints_btn.setFixedWidth(140)
        send_joints_btn.clicked.connect(self._send_joint_targets)
        jg_btn_row.addWidget(send_joints_btn)

        reset_joints_btn = self._btn("RESET TO HOME")
        reset_joints_btn.setFixedWidth(140)
        reset_joints_btn.setToolTip("Fill fields with home position from robot config")
        reset_joints_btn.clicked.connect(self._reset_joint_home)
        jg_btn_row.addWidget(reset_joints_btn)
        jg_btn_row.addStretch()
        jg_layout.addLayout(jg_btn_row)

        layout.addWidget(self._joint_group)

        # ── Cartesian Target Group ──
        self._cart_group = QGroupBox("CARTESIAN TARGET (metres / radians)")
        cg_layout = QGridLayout(self._cart_group)
        cg_layout.setSpacing(6)

        cart_labels = ["X:", "Y:", "Z:", "Rx:", "Ry:", "Rz:"]
        self._cart_fields = []
        for i, lbl_text in enumerate(cart_labels):
            lbl = QLabel(lbl_text)
            lbl.setFixedWidth(25)
            field = QLineEdit("0.0")
            field.setFixedWidth(80)
            cg_layout.addWidget(lbl, i // 3, (i % 3) * 2)
            cg_layout.addWidget(field, i // 3, (i % 3) * 2 + 1)
            self._cart_fields.append(field)

        cart_btn_row = QHBoxLayout()
        send_cart_btn = self._btn("SEND POSE", "BtnPrimary")
        send_cart_btn.setFixedWidth(140)
        send_cart_btn.clicked.connect(self._send_cartesian_target)
        cart_btn_row.addWidget(send_cart_btn)

        reset_cart_btn = self._btn("RESET TO HOME")
        reset_cart_btn.setFixedWidth(140)
        reset_cart_btn.setToolTip("Fill fields with default EE target from robot config")
        reset_cart_btn.clicked.connect(self._reset_cart_home)
        cart_btn_row.addWidget(reset_cart_btn)
        cart_btn_row.addStretch()

        cart_btn_container = QWidget()
        cart_btn_container.setLayout(cart_btn_row)
        cg_layout.addWidget(cart_btn_container, 2, 0, 1, 6)

        self._cart_group.setVisible(False)  # hidden by default (joint mode 1)
        layout.addWidget(self._cart_group)

        # ── Gripper ──
        gripper_group = QGroupBox("GRIPPER")
        gg_layout = QHBoxLayout(gripper_group)
        gg_layout.setSpacing(8)

        gg_layout.addWidget(QLabel("Width (m):"))
        self._gripper_spin = QDoubleSpinBox()
        self._gripper_spin.setRange(0.0, 0.08)
        self._gripper_spin.setDecimals(4)
        self._gripper_spin.setSingleStep(0.005)
        self._gripper_spin.setValue(0.04)
        self._gripper_spin.setFixedWidth(90)
        gg_layout.addWidget(self._gripper_spin)

        send_grip_btn = self._btn("Send", "BtnPrimary")
        send_grip_btn.setFixedWidth(60)
        send_grip_btn.clicked.connect(self._send_gripper)
        gg_layout.addWidget(send_grip_btn)

        open_btn = self._btn("Open", "BtnGreen")
        open_btn.setFixedWidth(60)
        open_btn.clicked.connect(lambda: self._send_gripper_preset(0.08))
        gg_layout.addWidget(open_btn)

        close_btn = self._btn("Close", "BtnRed")
        close_btn.setFixedWidth(60)
        close_btn.clicked.connect(lambda: self._send_gripper_preset(0.0))
        gg_layout.addWidget(close_btn)

        gg_layout.addStretch()
        layout.addWidget(gripper_group)

        # ── Live Readout ──
        readout_group = QGroupBox("LIVE READOUT")
        rg_layout = QGridLayout(readout_group)
        rg_layout.setSpacing(4)

        self._readout_labels = []
        for i in range(7):
            name_lbl = QLabel(f"J{i+1}:")
            name_lbl.setFixedWidth(30)
            theme.bind(
                name_lbl,
                lambda t: "color: %s; font-size: 11px;" % t["text_secondary"],
            )
            val_lbl = QLabel("0.000°")
            val_lbl.setFixedWidth(80)
            theme.bind(
                val_lbl,
                lambda t: (
                    "color: %s; font-size: 12px; font-family: monospace;"
                    % t["text_primary"]
                ),
            )
            rg_layout.addWidget(name_lbl, i // 4, (i % 4) * 2)
            rg_layout.addWidget(val_lbl, i // 4, (i % 4) * 2 + 1)
            self._readout_labels.append(val_lbl)

        # EE position row
        rg_layout.addWidget(QLabel(""), 2, 0)  # spacer row
        ee_labels = ["EE X:", "EE Y:", "EE Z:"]
        self._ee_readout_labels = []
        for i, txt in enumerate(ee_labels):
            lbl = QLabel(txt)
            lbl.setFixedWidth(40)
            theme.bind(
                lbl,
                lambda t: "color: %s; font-size: 11px;" % t["text_secondary"],
            )
            val = QLabel("0.000 m")
            val.setFixedWidth(80)
            theme.bind(
                val,
                lambda t: (
                    "color: %s; font-size: 12px; font-family: monospace;"
                    % t["text_primary"]
                ),
            )
            rg_layout.addWidget(lbl, 3, i * 2)
            rg_layout.addWidget(val, 3, i * 2 + 1)
            self._ee_readout_labels.append(val)

        layout.addWidget(readout_group)

        layout.addStretch()
        scroll.setWidget(page)
        return scroll

    # ── STATUS BAR ───────────────────────────────────────────────────────────

    def _build_statusbar(self):
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)

        conn_widget = QWidget()
        conn_layout = QHBoxLayout(conn_widget)
        conn_layout.setContentsMargins(8, 0, 8, 0)
        conn_layout.setSpacing(6)

        self.ros_led = StatusLED()
        self.ros_led.set_status('connected')
        conn_layout.addWidget(self.ros_led)
        conn_layout.addWidget(QLabel("ROS2"))
        conn_layout.addWidget(QLabel("  "))

        self.ctrl_led = StatusLED()
        conn_layout.addWidget(self.ctrl_led)
        conn_layout.addWidget(QLabel("FIC"))
        conn_layout.addWidget(QLabel("  "))

        self.driver_led = StatusLED()
        conn_layout.addWidget(self.driver_led)
        conn_layout.addWidget(QLabel("API"))
        conn_layout.addWidget(QLabel("  "))

        self.bags_led = StatusLED()
        conn_layout.addWidget(self.bags_led)
        conn_layout.addWidget(QLabel("BAGS"))

        self.statusbar.addWidget(conn_widget)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.statusbar.addWidget(spacer)

        self.robot_status_label = QLabel("Robot: —")
        self.statusbar.addPermanentWidget(self.robot_status_label)

    # ══════════════════════════════════════════════════════════════════════════
    #  STATUS UPDATE
    # ══════════════════════════════════════════════════════════════════════════

    def _update_status(self):
        self._restyle_btn(self.tb_driver_btn,
            "■ API" if self.driver_running else "▶ API",
            self.driver_running, 'hw')
        self._restyle_btn(self.tb_ctrl_btn,
            "■ FIC" if self.controller_running else "▶ FIC",
            self.controller_running, 'hw')
        self._restyle_btn(self.tb_bags_btn,
            "■ BAG" if self.bags_running else "▶ BAG",
            self.bags_running, 'util')
        self._restyle_btn(self.tb_plot_btn,
            "■ PLT" if self.plotter_running else "▶ PLT",
            self.plotter_running, 'util')

        vr_running = getattr(self, 'vr_relay_running', False)
        self._update_vr_label()
        self._restyle_btn(self.tb_vr_btn, self.tb_vr_btn.text(), vr_running, 'vr')
        if vr_running:
            if hasattr(self, '_poll_vr_scene_config'):
                self._poll_vr_scene_config()
            if hasattr(self, '_poll_vr_mode'):
                self._poll_vr_mode()

        self.ctrl_led.set_status('connected' if self.controller_running else 'idle')
        self.driver_led.set_status('connected' if self.driver_running else 'idle')
        self.bags_led.set_status('warning' if self.bags_running else 'idle')

    # ══════════════════════════════════════════════════════════════════════════
    #  ROS2 CALLBACKS
    # ══════════════════════════════════════════════════════════════════════════

    def _on_trajectory_data(self, msg):
        """Parse /trajectory_data and update live state."""
        from fic_studio.mixins.script_mixin import _parse_trajectory_data
        data = _parse_trajectory_data(list(msg.data), self._num_joints)
        if data is None:
            return
        self._live_q = data.get('q_current', self._live_q)
        self.current_joint_positions = list(self._live_q)
        if data.get('ee_position'):
            self._live_ee = data['ee_position']
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x = self._live_ee[0]
            pose.position.y = self._live_ee[1]
            pose.position.z = self._live_ee[2]
            self.current_ee_pose = pose

    def _on_q_current(self, msg):
        """Fallback: update live joint positions from q_current topic."""
        if msg.data:
            self._live_q = list(msg.data[:self._num_joints])
            self.current_joint_positions = list(self._live_q)

    def _refresh_live_readout(self):
        """Update the joint & EE readout labels (called at 20 Hz)."""
        for i in range(min(len(self._readout_labels), len(self._live_q))):
            deg = math.degrees(self._live_q[i])
            self._readout_labels[i].setText(f"{deg:+8.3f}\u00b0")
        for i in range(3):
            if i < len(self._live_ee):
                self._ee_readout_labels[i].setText(f"{self._live_ee[i]:+.4f} m")

    # ══════════════════════════════════════════════════════════════════════════
    #  CONTROL TAB ACTIONS
    # ══════════════════════════════════════════════════════════════════════════

    def _send_joint_targets(self):
        """Read the 7 joint fields (deg), convert to rad, publish."""
        try:
            values_deg = [float(f.text()) for f in self._joint_fields[:self._num_joints]]
        except ValueError:
            QMessageBox.warning(self, "Invalid Input",
                                "All joint fields must be valid numbers (degrees).")
            return
        values_rad = [math.radians(d) for d in values_deg]
        self.node.publish_real_joint_angles(self.node.station_robot_id, values_rad)
        self.log_message("INFO",
            f"Sent joints (deg): {[f'{d:.1f}' for d in values_deg]}", "Control")

    def _send_cartesian_target(self):
        """Read XYZ + Rx Ry Rz fields, build Pose, publish."""
        try:
            vals = [float(f.text()) for f in self._cart_fields]
        except ValueError:
            QMessageBox.warning(self, "Invalid Input",
                                "All Cartesian fields must be valid numbers.")
            return
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = vals[0]
        pose.position.y = vals[1]
        pose.position.z = vals[2]
        # Axis-angle → quaternion (simplified: store in orientation fields)
        # For a proper conversion use scipy, but for dev console raw values suffice
        pose.orientation.x = vals[3]
        pose.orientation.y = vals[4]
        pose.orientation.z = vals[5]
        pose.orientation.w = 1.0
        self.node.publish_position(pose)
        self.log_message("INFO",
            f"Sent pose: xyz=({vals[0]:.4f}, {vals[1]:.4f}, {vals[2]:.4f})", "Control")

    def _send_gripper(self):
        """Publish the gripper width from the spinbox."""
        width = self._gripper_spin.value()
        self.publish_gripper_width(self.node.station_robot_id, width)

    def _send_gripper_preset(self, width: float):
        """Open/Close shortcut."""
        self._gripper_spin.setValue(width)
        self.publish_gripper_width(self.node.station_robot_id, width)

    def _reset_joint_home(self):
        """Fill joint fields with home position from the active robot config."""
        robot_name = self.robot_dropdown.currentText()
        cfg = self.robot_configs.get(robot_name, {})
        home_deg = cfg.get('q_home_deg', [0.0] * self._num_joints)
        for i, field in enumerate(self._joint_fields[:self._num_joints]):
            val = home_deg[i] if i < len(home_deg) else 0.0
            field.setText(f"{val:.1f}")
        self.log_message("INFO", f"Joint fields reset to {robot_name} home", "Control")

    def _reset_cart_home(self):
        """Fill Cartesian fields with default_ee_target from robot config."""
        robot_name = self.robot_dropdown.currentText()
        cfg = self.robot_configs.get(robot_name, {})
        ee_target = cfg.get('default_ee_target', {})
        pos = ee_target.get('position', [0.0, 0.0, 0.0])
        ori = ee_target.get('orientation', [0.0, 0.0, 0.0, 0.0])
        # Fields: X, Y, Z, Rx, Ry, Rz
        defaults = [
            pos[0] if len(pos) > 0 else 0.0,
            pos[1] if len(pos) > 1 else 0.0,
            pos[2] if len(pos) > 2 else 0.0,
            ori[0] if len(ori) > 0 else 0.0,
            ori[1] if len(ori) > 1 else 0.0,
            ori[2] if len(ori) > 2 else 0.0,
        ]
        for i, field in enumerate(self._cart_fields):
            field.setText(f"{defaults[i]:.4f}")
        self.log_message("INFO", f"Cartesian fields reset to {robot_name} default EE target", "Control")

    def _send_mode(self):
        """Publish the currently selected control mode."""
        mode_id = self.mode_dropdown.currentIndex() + 1
        self.node.publish_mode(mode_id)
        self.log_message("INFO",
            f"Sent control mode: {mode_id} — {CONTROL_MODES.get(mode_id, '?')}", "Control")

    def _on_mode_changed(self, idx):
        """Toggle joint/Cartesian field visibility based on mode."""
        self._current_mode = idx + 1
        is_cart = self._current_mode >= 5
        self._joint_group.setVisible(not is_cart)
        self._cart_group.setVisible(is_cart)

    # ══════════════════════════════════════════════════════════════════════════
    #  ROBOT CHANGED
    # ══════════════════════════════════════════════════════════════════════════

    def _on_robot_changed(self, robot_name: str):
        if not robot_name:
            return
        num_joints = self.robots.get(robot_name, 7)
        self._num_joints = num_joints

        # Update joint field visibility
        for i, field in enumerate(self._joint_fields):
            visible = i < num_joints
            field.setVisible(visible)
            # Also hide the label in the grid (column to the left)
            lbl_item = self._joint_grid.itemAtPosition(i // 4, (i % 4) * 2)
            if lbl_item and lbl_item.widget():
                lbl_item.widget().setVisible(visible)

        # Update readout label visibility
        for i, lbl in enumerate(self._readout_labels):
            lbl.setVisible(i < num_joints)

        self._update_tuning_joint_count()
        if hasattr(self, 'load_robot_safety_limits'):
            self.load_robot_safety_limits(robot_name)
        self.robot_status_label.setText(f"Robot: {robot_name} ({num_joints} DOF)")
        self.log_message("INFO", f"Selected robot: {robot_name}", "Robot")

    def _select_robot_from_menu(self, robot_name: str):
        idx = self.robot_dropdown.findText(robot_name)
        if idx >= 0:
            self.robot_dropdown.setCurrentIndex(idx)

    # ══════════════════════════════════════════════════════════════════════════
    #  EMERGENCY STOP
    # ══════════════════════════════════════════════════════════════════════════

    def _emergency_stop(self):
        self.log_message("ERROR", "E-STOP ACTIVATED!", "Safety")

        for attr, stop_method in [
            ('controller_running', '_stop_controller'),
            ('driver_running', '_stop_driver'),
            ('fic_running', '_stop_fic_simulation'),
            ('bags_running', '_stop_bags'),
            ('plotter_running', '_stop_plotter'),
        ]:
            if getattr(self, attr, False) and hasattr(self, stop_method):
                getattr(self, stop_method)()

        # Send current joint positions to hold
        if self._live_q:
            from std_msgs.msg import Float32MultiArray
            msg = Float32MultiArray()
            msg.data = [float(j) for j in self._live_q]
            pub = self.node.real_joint_publishers.get(self.node.station_robot_id)
            if pub:
                pub.publish(msg)

        self.robot_status_label.setText("E-STOP ACTIVE — ALL PROCESSES KILLED")
        self.robot_status_label.setStyleSheet(
            "color: %s; font-weight: 600;" % theme.current()["accent_red"]
        )

        try:
            subprocess.run(["pkill", "-9", "-f", "cartesian_impedance_control"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    #  NAVIGATION STUBS (mixins may call these)
    # ══════════════════════════════════════════════════════════════════════════

    def show_tuning_panel(self):
        self.tabs.setCurrentIndex(1)
        self._load_tuning_parameters()

    def show_settings_panel(self):
        self.tabs.setCurrentIndex(2)

    # Stubs for methods mixins may reference
    def show_dashboard(self): pass
    def show_robot_control_panel(self): pass
    def show_autotuning_panel(self): pass
    def show_sequence_panel(self): pass
    def show_script_editor_panel(self): pass
    def show_teach_mode_panel(self): pass
    def show_experiment_panel(self): pass
    def show_robot_config_panel(self): pass
    def show_topic_monitor_panel(self): pass

    # ══════════════════════════════════════════════════════════════════════════
    #  LOGGING
    # ══════════════════════════════════════════════════════════════════════════

    def log_message(self, level: str, message: str, source: str = ""):
        if hasattr(self, 'log_viewer') and self.log_viewer:
            self.log_viewer.log(level, message, source)

    # ══════════════════════════════════════════════════════════════════════════
    #  CLOSE
    # ══════════════════════════════════════════════════════════════════════════

    def closeEvent(self, event):
        # SIGTERM-then-SIGKILL the entire process group of every tracked
        # subprocess. Replaces the old per-attr proc.terminate() loop, which
        # only signalled the bash wrapper and left ros2 launch grandchildren
        # orphaned on the robot.
        self._kill_all_processes(reason="window closed")
        event.accept()
