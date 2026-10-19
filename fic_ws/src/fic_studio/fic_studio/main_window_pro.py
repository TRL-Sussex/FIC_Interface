# main_window_pro.py — Professional IDE-style Main Window
# Uses QMainWindow with menus, toolbars, status bar while keeping all working functionality

import math

from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QFrame, QStackedWidget, QSplitter, QDockWidget, QAction,
    QToolBar, QStatusBar, QMenu, QActionGroup, QSizePolicy, QMessageBox,
    QFileDialog, QListView, QCheckBox, QToolButton,
)

# Local imports
from fic_studio import version
from fic_studio.ros_node import FicStudioNode
from fic_studio.fleet.stations import all_stations, picker_value, station_label
from fic_studio.theme.stylesheet import build_stylesheet, scale_overlay_qss
from fic_studio.widgets import StatusLED
from fic_studio import theme

# Import mixins
from fic_studio.mixins.dashboard_mixin import DashboardMixin
from fic_studio.pages import PageRegistry
from fic_studio.mixins.control_bar_mixin import ControlBarMixin
from fic_studio.mixins.multi_robot_mixin import MultiRobotMixin
from fic_studio.mixins.fleet_mixin import FleetMixin
from fic_studio.mixins.robot_control_mixin import RobotControlMixin
from fic_studio.mixins.tuning_mixin import TuningMixin
from fic_studio.mixins.process_mixin import ProcessMixin
from fic_studio.mixins.publisher_mixin import PublisherMixin
from fic_studio.mixins.settings_mixin import SettingsMixin, LogViewer
from fic_studio.mixins.robot_config_mixin import RobotConfigMixin
from fic_studio.mixins.topic_monitor_mixin import TopicMonitorMixin
from fic_studio.mixins.safety_mixin import SafetyMixin
from fic_studio.mixins.joint_state_mixin import JointStateMixin
from fic_studio.mixins.gamepad_mixin import GamepadMixin
from fic_studio.mixins.keyboard_mixin import KeyboardMixin
from fic_studio.mixins.help_mixin import HelpMixin
from fic_studio.mixins.web_remote_mixin import WebRemoteMixin
from fic_studio.mixins.web_ui_mixin import WebUIMixin
from fic_studio.mixins.locomotion_mixin import LocomotionMixin
from fic_studio.mixins.inner_loop_mixin import InnerLoopMixin

# Extension system
from fic_studio.config import FIC_INTERFACE_DIR
from fic_studio.extensions import (
    ExtensionHost, ExtensionLoader, discover_extensions,
)
from fic_studio.extensions.settings_panel import ExtensionsSettingsPanel


# ═══════════════════════════════════════════════════════════════════════════════
#  PROFESSIONAL MAIN WINDOW
# ═══════════════════════════════════════════════════════════════════════════════

# Toolbar buttons are quiet outlines; theme.button_qss builds them from
# tokens. Only the E-stop is solid (kind='safety').
def _tb_filled(t, accent):
    """Return the outline toolbar-button QSS in ``accent``."""
    return theme.button_qss(t, 'accent', accent)


#: Toolbar QSS per (use, theme name): the status timer asks for it every 500 ms.
_TB_QSS = {}


def _set_qss(widget, qss):
    """Set ``qss`` only when it differs: a restyle re-polishes the widget, which is not cheap."""
    if widget.styleSheet() != qss:
        widget.setStyleSheet(qss)


class ProfessionalRobotGUI(
    QMainWindow, DashboardMixin, ControlBarMixin, RobotControlMixin,
    MultiRobotMixin,
    FleetMixin,
    TuningMixin, ProcessMixin,
    PublisherMixin, SettingsMixin, RobotConfigMixin,
    TopicMonitorMixin, SafetyMixin, JointStateMixin, LocomotionMixin,
    GamepadMixin, KeyboardMixin, HelpMixin, WebRemoteMixin, WebUIMixin, InnerLoopMixin,
):
    APP_NAME = "FIC Control Studio"
    APP_VERSION = version.RELEASE
    
    def __init__(self, node: FicStudioNode):
        super().__init__()
        self.node = node
        self.setWindowTitle(f"{self.APP_NAME} - {version.display()}")
        self.resize(1500, 800)
        self.setMinimumSize(1000, 500)
        
        # Initialize settings first (loads from file or defaults)
        self._init_settings()

        # Apply saved theme (dark/light from settings)
        self.apply_theme(self.current_theme)

        # Process handles
        self.simulation_process = None
        self.controller_process = None
        self.fic_process = None
        self.vr_relay_process = None
        self.bags_process = None
        self.plotter_process = None
        self.driver_process = None
        self.locomotion_process = None
        self.current_bag_path = None

        self.controller_running = False
        self.simulation_running = False
        self.fic_running = False
        self.vr_relay_running = False
        self.vr_fic_running = False
        self.bags_running = False
        self.plotter_running = False
        self.driver_running = False
        self.locomotion_running = False

        # First-launch wizard (before the UI is built, after the process state: its test launch
        # reads and sets it)
        self._wizard_first_action = None
        if not self.settings.get('general', {}).get('wizard_completed', False):
            self._run_first_launch_wizard()
        
        self.robots = {
            "Franka_FR3": 7, "KinovaGen3_7DOF": 7,
            "UR5e": 6, "XArm7": 7,
        }
        
        self.real_joint_sliders = {}
        self.real_joint_spinboxes = {}
        self.real_joint_units = {}
        self.real_joint_autopublish = {}
        self.real_joint_count = {}
        self.real_joint_container = {}
        self.real_joint_layout = {}
        # Per-card robot type override (widget prefix → type name).
        # Card 1 falls back to the global self.robot_dropdown selection.
        self._card_robot_types = {}
        # Per-card station mapping (widget prefix → station number).
        # E.g. {"robot_2": 3} means card 2 controls station/robot 3.
        self._card_station = {}
        
        self._setup_gripper_publishers()
        self._side_buttons = []
        
        self.robot_configs = self._load_robot_configs()
        if self.robot_configs:
            self.robots = {k: v.get('num_joints', 7) for k, v in self.robot_configs.items()}
        try:
            from fic_studio.xml_manager import XMLManager
            XMLManager().ensure_all_robots(self.robots)
        except Exception as e:
            print(f"[XMLManager] Warning: could not ensure robots in config: {e}")
        
        self._init_topic_monitor()
        self._init_safety()
        self._init_joint_state()
        self._init_dashboard()
        self._init_gamepad()
        self._init_keyboard()
        self._init_locomotion()
        self._init_inner_loop()

        self._init_extensions()

        self._setup_menubar()
        self._setup_toolbar()
        self._setup_control_bar()
        self._setup_central_widget()
        self._setup_statusbar()
        
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(500)
        
        self._refresh_robots_dropdown()
        default_robot = self.settings['general']['default_robot']
        idx = self.robot_dropdown.findText(default_robot)
        if idx >= 0:
            self.robot_dropdown.setCurrentIndex(idx)
        
        self.log_message("INFO", f"{self.APP_NAME} started", "System")
        self.log_message("INFO", f"Loaded {len(self.robot_configs)} robot(s)", "System")
        self._dash_scan_recent_experiments()
        # Wire theme-change listeners so toolbar and dashboard recolour on switch
        theme.signals.theme_changed.connect(lambda *_: self._restyle_toolbar())
        if hasattr(self, '_restyle_dash_theme'):
            theme.signals.theme_changed.connect(lambda *_: self._restyle_dash_theme())

        # Last: its commands drive the control bar, which must exist by now.
        self._init_web_remote()
        self._init_web_ui()     # --web-ui: the redesigned interface, opt-in
        self._init_fleet()
        # Settings > VR: the background relay, so headsets find this Studio without VR > Start
        self._vr_settings_ready = True
        QTimer.singleShot(1500, self._apply_vr_settings)
        # ...and back after an E-STOP (it drives no robot) or if it stops by itself
        self._vr_bg_timer = QTimer(self)
        self._vr_bg_timer.timeout.connect(self._vr_bg_watchdog)
        self._vr_bg_timer.start(5000)

        # Navigate to wizard-chosen first action (deferred so window is shown)
        if self._wizard_first_action:
            action = self._wizard_first_action
            if action == "dashboard":
                QTimer.singleShot(100, self.show_dashboard)
            elif action == "sim":
                QTimer.singleShot(100, lambda: (self._dash_toggle_sim(), self.show_robot_control_panel()))
            elif action == "tuning":
                QTimer.singleShot(100, self.show_tuning_panel)
            elif action == "settings":
                QTimer.singleShot(100, self.show_settings_panel)

    # ══════════════════════════════════════════════════════════════════════════
    #  FIRST-LAUNCH WIZARD
    # ══════════════════════════════════════════════════════════════════════════

    def _run_first_launch_wizard(self):
        from fic_studio.mixins.first_launch_wizard import FirstLaunchWizard

        # robot_configs may not be loaded yet at this point in __init__,
        # so load them directly
        from fic_studio.mixins.robot_config_mixin import DEFAULT_ROBOT_CONFIGS
        configs = DEFAULT_ROBOT_CONFIGS
        if hasattr(self, 'robot_configs') and self.robot_configs:
            configs = self.robot_configs

        wizard = FirstLaunchWizard(
            parent=self,
            robot_configs=configs,
            settings=self.settings,
        )

        if wizard.exec_() == wizard.Accepted:
            results = wizard.get_results()
            self.settings['general']['default_robot'] = results['robot']
            self.settings['general']['wizard_completed'] = True
            self._save_settings()
            self._wizard_first_action = results['first_action']
        else:
            # Cancelled — mark completed so it doesn't loop
            self.settings['general']['wizard_completed'] = True
            self._save_settings()

    def _wizard_run_test(self, status_callback):
        """Run test launch sequence for the wizard."""
        status_callback("sim", "progress", "Starting...")

        if not hasattr(self, 'simulation_running'):
            status_callback("sim", "fail", "Process system not initialized")
            return

        self._start_simulation()

        def check_sim():
            if self.simulation_running:
                status_callback("sim", "ok", "Running")
                status_callback("fic", "progress", "Starting...")
                self._start_fic_simulation()
                QTimer.singleShot(3000, check_fic)
            else:
                status_callback("sim", "fail", "Failed to start")

        def check_fic():
            if self.fic_running:
                status_callback("fic", "ok", "Running")
                status_callback("topic", "progress", "Checking...")
                QTimer.singleShot(2000, check_topic)
            else:
                status_callback("fic", "fail", "Failed to start")

        def check_topic():
            try:
                topics = self.node.get_topic_names_and_types()
                has_traj = any('/trajectory_data' in name for name, _ in topics)
            except Exception:
                has_traj = False
            if has_traj:
                status_callback("topic", "ok", "Publishing")
            else:
                status_callback("topic", "fail", "Not publishing")
            QTimer.singleShot(1000, self._wizard_cleanup_test)

        QTimer.singleShot(3000, check_sim)

    def _wizard_cleanup_test(self):
        """Stop processes launched during wizard test."""
        if getattr(self, 'fic_running', False):
            self._stop_fic_simulation()
        if getattr(self, 'simulation_running', False):
            self._stop_simulation()

    def _rerun_wizard(self):
        """Re-run the first-launch wizard from the Help menu."""
        self.settings['general']['wizard_completed'] = False
        self._run_first_launch_wizard()
        # Re-running the wizard means the user is re-onboarding — restore the
        # Getting Started card in case they'd previously dismissed it.
        if hasattr(self, '_dash_show_getting_started'):
            self._dash_show_getting_started()
        # Apply robot selection
        default_robot = self.settings['general']['default_robot']
        idx = self.robot_dropdown.findText(default_robot)
        if idx >= 0:
            self.robot_dropdown.setCurrentIndex(idx)
        # Navigate to chosen action (already running, so do it immediately)
        action = self._wizard_first_action
        if action == "dashboard":
            self.show_dashboard()
        elif action == "sim":
            self._dash_toggle_sim()
            self.show_robot_control_panel()
        elif action == "tuning":
            self.show_tuning_panel()
        elif action == "settings":
            self.show_settings_panel()

    # ══════════════════════════════════════════════════════════════════════════
    #  UI HELPER METHODS
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
    
    def _mk_sidebar_btn(self, text: str, callback, active: bool = False) -> QPushButton:
        btn = QPushButton(text)
        btn.setObjectName("SideButton")
        btn.setCursor(Qt.PointingHandCursor)
        btn.setProperty("active", active)
        btn.clicked.connect(callback)
        return btn
    
    # ══════════════════════════════════════════════════════════════════════════
    #  MENU BAR — no inline styles, all from global theme
    # ══════════════════════════════════════════════════════════════════════════
    
    def _setup_menubar(self):
        menubar = self.menuBar()

        # ═══ File Menu ═══
        file_menu = menubar.addMenu("&File")

        new_script = QAction("New Script", self)
        new_script.setShortcut(QKeySequence.New)
        new_script.triggered.connect(self._new_script)
        file_menu.addAction(new_script)

        open_menu = file_menu.addMenu("Open")
        open_experiment = QAction("Experiment Spec...", self)
        open_experiment.triggered.connect(self._open_experiment_spec)
        open_menu.addAction(open_experiment)
        open_sequence = QAction("Sequence File...", self)
        open_sequence.triggered.connect(self._open_sequence_file)
        open_menu.addAction(open_sequence)
        open_config = QAction("Config Snapshot...", self)
        open_config.triggered.connect(self._open_config_snapshot)
        open_menu.addAction(open_config)

        file_menu.addSeparator()

        save_action = QAction("Save", self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self._save_current)
        file_menu.addAction(save_action)

        export_menu = file_menu.addMenu("Export")
        export_config = QAction("Current Config as Snapshot...", self)
        export_config.triggered.connect(self._export_config_snapshot)
        export_menu.addAction(export_config)
        export_params = QAction("FIC Parameters as YAML...", self)
        export_params.triggered.connect(self._export_params_yaml)
        export_menu.addAction(export_params)

        file_menu.addSeparator()

        self._recent_menu = file_menu.addMenu("Recent Experiments")
        self._populate_recent_experiments()

        file_menu.addSeparator()

        exit_action = QAction("Exit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # ═══ View Menu ═══
        view_menu = menubar.addMenu("&View")

        dashboard_action = QAction("Dashboard", self)
        dashboard_action.setShortcut("Ctrl+0")
        dashboard_action.triggered.connect(self.show_dashboard)
        view_menu.addAction(dashboard_action)
        view_menu.addSeparator()

        panel_actions = [
            ("Robot Control",  "Ctrl+1", self.show_robot_control_panel),
            ("Tuning Panel",   "Ctrl+3", self.show_tuning_panel),
        ]
        for label, shortcut, callback in panel_actions:
            action = QAction(label, self)
            action.setShortcut(shortcut)
            action.triggered.connect(callback)
            view_menu.addAction(action)

        view_menu.addSeparator()

        self.toggle_toolbar_action = QAction("Show Toolbar", self)
        self.toggle_toolbar_action.setShortcut("Ctrl+T")
        self.toggle_toolbar_action.setCheckable(True)
        self.toggle_toolbar_action.setChecked(True)
        self.toggle_toolbar_action.triggered.connect(self._toggle_toolbar)
        view_menu.addAction(self.toggle_toolbar_action)

        self.keyboard_control_action = QAction("Keyboard Control", self)
        self.keyboard_control_action.setShortcut("Ctrl+K")
        self.keyboard_control_action.setCheckable(True)
        self.keyboard_control_action.setToolTip(
            "Drive the arm from the keyboard: W/S/A/D/Q/E, Tab to swap space")
        self.keyboard_control_action.triggered.connect(self.set_keyboard_control)
        view_menu.addAction(self.keyboard_control_action)

        self.toggle_control_bar_action = QAction("Show Control Bar", self)
        self.toggle_control_bar_action.setShortcut("Ctrl+Shift+B")
        self.toggle_control_bar_action.setCheckable(True)
        self.toggle_control_bar_action.setChecked(True)
        self.toggle_control_bar_action.triggered.connect(self._toggle_control_bar)
        view_menu.addAction(self.toggle_control_bar_action)

        self.toggle_sidebar_action = QAction("Show Sidebar", self)
        self.toggle_sidebar_action.setShortcut("Ctrl+\\")
        self.toggle_sidebar_action.setCheckable(True)
        self.toggle_sidebar_action.setChecked(True)
        self.toggle_sidebar_action.triggered.connect(self._toggle_sidebar)
        view_menu.addAction(self.toggle_sidebar_action)

        self.view_log_action = QAction("Logs", self)
        self.view_log_action.setShortcut("Ctrl+L")
        self.view_log_action.triggered.connect(self.show_logs_panel)
        view_menu.addAction(self.view_log_action)

        self.toggle_statusbar_action = QAction("Show Status Bar", self)
        self.toggle_statusbar_action.setCheckable(True)
        self.toggle_statusbar_action.setChecked(True)
        self.toggle_statusbar_action.triggered.connect(self._toggle_statusbar)
        view_menu.addAction(self.toggle_statusbar_action)
        view_menu.addSeparator()

        self.compact_mode_action = QAction("Compact Mode", self)
        self.compact_mode_action.setShortcut("Ctrl+M")
        self.compact_mode_action.setCheckable(True)
        self.compact_mode_action.triggered.connect(self._toggle_compact_mode)
        view_menu.addAction(self.compact_mode_action)

        self.minimal_mode_action = QAction("Minimal Mode", self)
        self.minimal_mode_action.setShortcut("Ctrl+Shift+M")
        self.minimal_mode_action.setCheckable(True)
        self.minimal_mode_action.triggered.connect(self._toggle_minimal_mode)
        view_menu.addAction(self.minimal_mode_action)
        view_menu.addSeparator()

        scale_menu = view_menu.addMenu("UI Scale")
        self.scale_action_group = QActionGroup(self)
        self._ui_scale = 100
        for scale in [75, 85, 100, 110, 125, 150]:
            action = QAction(f"{scale}%", self)
            action.setCheckable(True)
            action.setChecked(scale == 100)
            action.triggered.connect(lambda checked, s=scale: self._set_ui_scale(s))
            scale_menu.addAction(action)
            self.scale_action_group.addAction(action)
        view_menu.addSeparator()

        fullscreen = QAction("Toggle Fullscreen", self)
        fullscreen.setShortcut("F11")
        fullscreen.triggered.connect(self._toggle_fullscreen)
        view_menu.addAction(fullscreen)

        # ═══ Robot Menu ═══
        robot_menu = menubar.addMenu("&Robot")

        home_action = QAction("Move to Home", self)
        home_action.setShortcut("Ctrl+H")
        home_action.triggered.connect(self._move_home)
        robot_menu.addAction(home_action)

        estop_action = QAction("Emergency Stop", self)
        estop_action.triggered.connect(self._emergency_stop)
        robot_menu.addAction(estop_action)
        robot_menu.addSeparator()

        robot_select_menu = robot_menu.addMenu("Select Robot")
        self._robot_select_menu = robot_select_menu   # rebuilt when robots change
        self.robot_action_group = QActionGroup(self)
        for robot_name in self.robots.keys():
            action = QAction(robot_name, self)
            action.setCheckable(True)
            action.triggered.connect(
                lambda checked, r=robot_name: self._select_robot_from_menu(r))
            robot_select_menu.addAction(action)
            self.robot_action_group.addAction(action)
        robot_menu.addSeparator()

        mode_menu = robot_menu.addMenu("Control Mode")
        self._mode_action_group = QActionGroup(self)
        control_modes = {
            1: "Joint Impedance",
            2: "Joint Impedance + Planner",
            3: "Joint Admittance",
            4: "Joint Admittance Haptic",
            5: "Cartesian Impedance",
            6: "Cartesian Planner",
            7: "Cartesian Admittance",
            8: "Cartesian Admittance Haptic",
        }
        for mode_id, mode_name in control_modes.items():
            action = QAction(f"{mode_id}: {mode_name}", self)
            action.setCheckable(True)
            action.triggered.connect(
                lambda checked, m=mode_id: self._set_mode_from_menu(m))
            mode_menu.addAction(action)
            self._mode_action_group.addAction(action)
        robot_menu.addSeparator()

        robot_info = QAction("Robot Info...", self)
        robot_info.triggered.connect(self._show_robot_info)
        robot_menu.addAction(robot_info)

        # ═══ Simulation Menu ═══
        sim_menu = menubar.addMenu("&Simulation")

        start_sim = QAction("Toggle Simulation", self)
        start_sim.setShortcut("F5")
        start_sim.triggered.connect(self._toggle_simulation)
        sim_menu.addAction(start_sim)

        start_fic = QAction("Toggle FIC Controller", self)
        start_fic.setShortcut("F6")
        start_fic.triggered.connect(self._toggle_fic_simulation)
        sim_menu.addAction(start_fic)
        sim_menu.addSeparator()

        launch_all = QAction("Launch Sim + FIC", self)
        launch_all.setShortcut("F7")
        launch_all.triggered.connect(self._dash_toggle_sim)
        sim_menu.addAction(launch_all)

        stop_all = QAction("Stop All Processes", self)
        stop_all.setShortcut("Ctrl+Shift+Q")
        stop_all.triggered.connect(self._stop_all_processes)
        sim_menu.addAction(stop_all)
        sim_menu.addSeparator()

        hw_header = QAction("── Hardware ──", self)
        hw_header.setEnabled(False)
        sim_menu.addAction(hw_header)

        toggle_ctrl = QAction("Toggle Controller", self)
        toggle_ctrl.setShortcut("F8")
        toggle_ctrl.triggered.connect(self._toggle_controller)
        sim_menu.addAction(toggle_ctrl)

        toggle_driver = QAction("Toggle Robot Driver", self)
        toggle_driver.setShortcut("F9")
        toggle_driver.triggered.connect(self._toggle_driver)
        sim_menu.addAction(toggle_driver)
        sim_menu.addSeparator()

        rosbag_action = QAction("Quick Experiment...", self)
        rosbag_action.triggered.connect(self._quick_start_experiment)
        sim_menu.addAction(rosbag_action)
        # Note: rosbag_action is connected to _quick_start_experiment which is
        # now a thin shim that proxies to the experiments extension if loaded.

        plotter_action = QAction("Toggle Plotter", self)
        plotter_action.triggered.connect(self._toggle_plotter)
        sim_menu.addAction(plotter_action)

        # ═══ Tools Menu ═══
        tools_menu = menubar.addMenu("&Tools")

        topic_monitor = QAction("Topic Monitor", self)
        topic_monitor.triggered.connect(self.show_topic_monitor_panel)
        tools_menu.addAction(topic_monitor)
        robot_config = QAction("Robot Configuration", self)
        robot_config.triggered.connect(self.show_robot_config_panel)
        tools_menu.addAction(robot_config)
        tools_menu.addSeparator()

        open_exp_folder = QAction("Open Experiments Folder", self)
        open_exp_folder.triggered.connect(self._open_experiments_folder)
        tools_menu.addAction(open_exp_folder)
        tools_menu.addSeparator()

        compile_action = QAction("Compile Workspace", self)
        compile_action.setShortcut("Ctrl+B")
        compile_action.triggered.connect(self._open_compile_terminal)
        tools_menu.addAction(compile_action)

        compile_pkg = QAction("Compile Package...", self)
        compile_pkg.triggered.connect(self._open_compile_package_terminal)
        tools_menu.addAction(compile_pkg)

        clean_build = QAction("Clean Build", self)
        clean_build.triggered.connect(self._clean_and_rebuild)
        tools_menu.addAction(clean_build)

        # ═══ Help Menu ═══
        help_menu = menubar.addMenu("&Help")

        help_page_action = QAction("Help", self)
        help_page_action.setShortcut("F1")
        help_page_action.triggered.connect(self.show_help_panel)
        help_menu.addAction(help_page_action)

        docs_web_action = QAction("Open Documentation (web)", self)
        docs_web_action.triggered.connect(lambda: self._open_docs_page("home"))
        help_menu.addAction(docs_web_action)
        help_menu.addSeparator()

        shortcuts_ref = QAction("Keyboard Shortcuts", self)
        shortcuts_ref.setShortcut("Ctrl+/")
        shortcuts_ref.triggered.connect(self._show_shortcuts_reference)
        help_menu.addAction(shortcuts_ref)
        help_menu.addSeparator()

        settings_action = QAction("Settings", self)
        settings_action.setShortcut("Ctrl+,")
        settings_action.triggered.connect(self.show_settings_panel)
        help_menu.addAction(settings_action)
        help_menu.addSeparator()

        wizard_action = QAction("Re-run Setup Wizard...", self)
        wizard_action.triggered.connect(self._rerun_wizard)
        help_menu.addAction(wizard_action)
        help_menu.addSeparator()

        about_action = QAction(f"About {self.APP_NAME}", self)
        about_action.triggered.connect(self._show_about)
        help_menu.addAction(about_action)

        # Initialise the checkable Robot/Mode menu groups to the active state.
        self._sync_robot_menu_check()
        self._sync_mode_menu_check()

    def _open_docs_page(self, page_id: str):
        """Open a page of the bundled documentation in the user's browser.

        Routed through HelpMixin.serve_docs rather than opening the file
        directly: the docs fetch pages/*.html, which a browser refuses from a
        file:// origin, so the direct route only ever rendered "Could not
        load: pages/<id>.html".
        """
        self.serve_docs(page_id)
    
    # ══════════════════════════════════════════════════════════════════════════
    #  TOOLBAR — no inline styles, all from global theme
    # ══════════════════════════════════════════════════════════════════════════
    
    def _setup_control_bar(self):
        """Dock the global control bar along the bottom of the window.

        Kinova's robot control panel sits on the bottom of every screen and
        slides panels up from it; this is the same idea as a second,
        bottom-docked QToolBar, so it spans the full width and picks up the
        View-menu show/hide the other chrome already has.
        """
        self.control_bar_toolbar = QToolBar("Control Bar")
        self.control_bar_toolbar.setObjectName("ControlBarToolBar")
        self.control_bar_toolbar.setMovable(False)
        self.control_bar_toolbar.setFloatable(False)
        self.addToolBar(Qt.BottomToolBarArea, self.control_bar_toolbar)
        self.control_bar_toolbar.addWidget(self._build_control_bar())

    def _toggle_control_bar(self, checked: bool):
        """Show or hide the bottom control bar."""
        self.control_bar_toolbar.setVisible(checked)

    def _setup_toolbar(self):
        self.main_toolbar = QToolBar("Main Toolbar")
        self.main_toolbar.setMovable(False)
        self.main_toolbar.setIconSize(QSize(20, 20))
        self.addToolBar(self.main_toolbar)
        toolbar = self.main_toolbar
        
        # === LAUNCH: target selector + single Start/Stop ===
        tgt_label = QLabel("Target")
        tgt_label.setObjectName("BlueLabel")
        toolbar.addWidget(tgt_label)

        self._tb_target_btns = {}
        tgt_frame = QWidget()
        tgt_row = QHBoxLayout(tgt_frame)
        tgt_row.setContentsMargins(0, 0, 0, 0)
        tgt_row.setSpacing(0)
        for key, text in (("simulation", "Simulation"),
                          ("hardware", "Hardware"), ("vr", "VR")):
            b = QPushButton(text)
            b.setFixedHeight(26)
            b.setCursor(Qt.PointingHandCursor)
            b.clicked.connect(lambda _=False, k=key: self._tb_set_target(k))
            self._tb_target_btns[key] = b
            tgt_row.addWidget(b)
        toolbar.addWidget(tgt_frame)

        self._tb_start_btn = QPushButton("▶ Start")
        self._tb_start_btn.setFixedHeight(26)
        self._tb_start_btn.setMinimumWidth(96)
        self._tb_start_btn.setCursor(Qt.PointingHandCursor)
        self._tb_start_btn.clicked.connect(self._tb_toggle_start)
        toolbar.addWidget(self._tb_start_btn)
        toolbar.addSeparator()

        # Hidden sink: process_mixin styles the old launch aliases directly.
        self._tb_sink_btn = QPushButton()
        self._tb_sink_btn.setVisible(False)

        # === TOOLS SECTION ===
        # Toolbar EXP button — visible only when the experiments extension is
        # enabled (toggleable in Settings → Extensions).
        self.tb_exp_btn = QPushButton("▶ EXP")
        self.tb_exp_btn.setStyleSheet(self._tb_styles()['util'])
        self.tb_exp_btn.setToolTip("Quick-Start Experiment Recording (requires Experiments extension)")
        self.tb_exp_btn.setFixedSize(90, 26)
        self.tb_exp_btn.clicked.connect(self._quick_start_experiment)
        toolbar.addWidget(self.tb_exp_btn)

        self.tb_bags_btn = QPushButton("▶ BAG")
        self.tb_bags_btn.setStyleSheet(self._tb_styles()['util'])
        self.tb_bags_btn.setToolTip("Manual ROS Bag Record (independent of experiments)")
        self.tb_bags_btn.setFixedSize(90, 26)
        self.tb_bags_btn.clicked.connect(self._toggle_bags)
        toolbar.addWidget(self.tb_bags_btn)

        self.tb_plot_btn = QPushButton("▶ PLT")
        self.tb_plot_btn.setStyleSheet(self._tb_styles()['util'])
        self.tb_plot_btn.setToolTip("Start/Stop Plotter")
        self.tb_plot_btn.setFixedSize(90, 26)
        self.tb_plot_btn.clicked.connect(self._toggle_plotter)
        toolbar.addWidget(self.tb_plot_btn)

        # Contact viewer launch/stop toggle -- hardware-only (see
        # _refresh_toolbar_launch), since mirroring the arm and
        # highlighting the pushed link only makes sense while driving
        # real hardware. The Contact panel (Calibrate, ground truth)
        # stays in the control bar; only this launch button lives here.
        self.tb_viewer_btn = QPushButton("▶ VIEW")
        self.tb_viewer_btn.setStyleSheet(self._tb_styles()['util'])
        self.tb_viewer_btn.setToolTip(
            "Start/Stop the contact viewer — mirrors the arm and "
            "highlights the pushed link")
        self.tb_viewer_btn.setFixedSize(90, 26)
        self.tb_viewer_btn.clicked.connect(self._cb_on_contact_twin)
        toolbar.addWidget(self.tb_viewer_btn)
        toolbar.addSeparator()
        
        # === ROBOT SECTION ===
        robot_label = QLabel("Robot:")
        toolbar.addWidget(robot_label)
        
        self._tb_robot_id = QComboBox()
        self._tb_robot_id.setFixedHeight(26)
        # Robot 1..4, then Master (the control seat: no arm; see fleet.stations).
        for n in all_stations():
            self._tb_robot_id.addItem(station_label(n), n)
        self._tb_robot_id.setCurrentIndex(
            max(0, self._tb_robot_id.findData(picker_value(self.node.station_id))))
        # No font-weight here, and combo_popup_qss is not optional decoration.
        # A widget-level stylesheet propagates to the widget's children, and a
        # combo's drop-down list is one of them: "font-weight: 600" set the
        # combo's *font*, which the list then inherited, so the robot picker
        # opened in bold while every other dropdown and every right-click menu
        # was normal weight. QSS cannot undo it from the list side — neither
        # QComboBox QAbstractItemView nor ::item overrides an inherited font —
        # so the weight has to come off the box. The accent left border is what
        # makes this combo stand out; it does not need bold as well.
        theme.bind(self._tb_robot_id, lambda t: (
            'QComboBox { background-color: %s; color: %s; border: 1px solid %s;'
            ' border-left: 3px solid %s; border-radius: 5px; padding: 5px 9px;'
            ' font-size: 12px; }'
        ) % (t['bg_overlay'], t['text_primary'], t['border_subtle'],
             t['accent_blue']) + theme.combo_popup_qss(t))
        # The list is only as wide as the box unless told otherwise, and the box
        # is sized for "Robot 1" — which clipped every entry in the popup.
        self._tb_robot_id.view().setMinimumWidth(140)
        self._tb_robot_id.currentIndexChanged.connect(self._tb_on_robot_id_changed)
        toolbar.addWidget(self._tb_robot_id)

        self.robot_dropdown = QComboBox()
        self.robot_dropdown.setMinimumWidth(130)
        self.robot_dropdown.setFixedHeight(26)
        self.robot_dropdown.addItems(self.robots.keys())
        self.robot_dropdown.currentTextChanged.connect(self._on_robot_changed)
        self.robot_dropdown.currentTextChanged.connect(lambda _t: self.il_refresh_probe())
        toolbar.addWidget(self.robot_dropdown)
        toolbar.addSeparator()
        
        # Spacer
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        spacer.setAttribute(Qt.WA_TransparentForMouseEvents)
        toolbar.addWidget(spacer)

        self.estop_btn = QPushButton("■ E-STOP")
        theme.bind(
            self.estop_btn,
            lambda t: theme.button_qss(t, 'safety'),
        )
        self.estop_btn.setFixedSize(95, 26)
        self.estop_btn.setToolTip("Emergency Stop (Space)")
        self.estop_btn.clicked.connect(self._emergency_stop)
        toolbar.addWidget(self.estop_btn)
        
        # Aliases for process_mixin compatibility. The four launch buttons
        # no longer exist — point their style targets at a hidden sink so
        # _apply_btn_style(...) never raises. Utility buttons keep real ones.
        self.fic_toggle_btn = self._tb_sink_btn
        self.simulation_toggle_btn = self._tb_sink_btn
        self.vr_toggle_btn = self._tb_sink_btn
        self.controller_toggle_btn = self._tb_sink_btn
        self.send_start_mode_btn = self._tb_sink_btn
        self.bags_toggle_btn = self.tb_bags_btn
        self.plotter_toggle_btn = self.tb_plot_btn
        self.exp_toggle_btn = self.tb_exp_btn

    # ══════════════════════════════════════════════════════════════════════════
    #  CENTRAL WIDGET
    # ══════════════════════════════════════════════════════════════════════════
    
    def _filter_sidebar(self, text: str):
        text = text.lower().strip()
        for btn in self._side_buttons:
            if btn is None:
                continue
            btn.setVisible(text == "" or text in btn.text().lower())
        for parts in self._sidebar_sections.values():
            label = parts.get("label")
            divider = parts.get("divider")
            if label is None:
                continue
            if text == "":
                label.setVisible(True)
                if divider:
                    divider.setVisible(True)
            else:
                label.setVisible(False)
                if divider:
                    divider.setVisible(False)

    def _setup_central_widget(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── SIDEBAR ──
        from PyQt5.QtWidgets import QScrollArea, QLineEdit
        self.sidebar = QWidget()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(230)
        sidebar_outer_layout = QVBoxLayout(self.sidebar)
        sidebar_outer_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_outer_layout.setSpacing(0)

        # Collapse button — inside sidebar at top
        self._sidebar_collapse_btn = QPushButton("☰")
        self._sidebar_collapse_btn.setObjectName("SidebarCollapse")
        self._sidebar_collapse_btn.setCursor(Qt.PointingHandCursor)
        self._sidebar_collapse_btn.setFixedSize(36, 36)
        self._sidebar_collapse_btn.clicked.connect(self._toggle_sidebar_collapse)
        collapse_row = QHBoxLayout()
        collapse_row.setContentsMargins(12, 10, 12, 0)
        collapse_row.addWidget(self._sidebar_collapse_btn, 0, Qt.AlignLeft | Qt.AlignTop)
        collapse_row.addStretch()
        sidebar_outer_layout.addLayout(collapse_row)

        # Collapsible content — everything below ☰ in one widget
        self._sidebar_content = QWidget()
        self._sidebar_content.setObjectName("Sidebar")
        sidebar_content_layout = QVBoxLayout(self._sidebar_content)
        sidebar_content_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_content_layout.setSpacing(0)

        # Search bar
        self._sidebar_search = QLineEdit()
        self._sidebar_search.setObjectName("SidebarSearch")
        self._sidebar_search.setPlaceholderText("Search")
        self._sidebar_search.setClearButtonEnabled(True)
        self._sidebar_search.textChanged.connect(self._filter_sidebar)
        search_wrap = QHBoxLayout()
        search_wrap.setContentsMargins(12, 8, 12, 8)
        search_wrap.addWidget(self._sidebar_search)
        sidebar_content_layout.addLayout(search_wrap)

        # Scrollable navigation area
        sidebar_scroll = QScrollArea()
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sidebar_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        sidebar_scroll.setFrameShape(QFrame.NoFrame)

        sidebar_inner = QWidget()
        sidebar_inner.setObjectName("Sidebar")
        sidebar_layout = QVBoxLayout(sidebar_inner)
        sidebar_layout.setContentsMargins(0, 4, 0, 0)
        sidebar_layout.setSpacing(0)

        dashboard_btn = self._mk_sidebar_btn("  ⊞  Dashboard", self.show_dashboard, active=True)
        sidebar_layout.addWidget(dashboard_btn)

        robot_ctrl_btn = self._mk_sidebar_btn("  ⊞  Multi-Robot", self.show_multi_robot_panel)
        sidebar_layout.addWidget(robot_ctrl_btn)

        self._locomotion_sidebar_btn = self._mk_sidebar_btn(
            "  ⊿  Locomotion", self.show_locomotion_panel)
        sidebar_layout.addWidget(self._locomotion_sidebar_btn)

        self._sidebar_section_dividers = {}
        self._sidebar_sections = {}

        def _open_section(name):
            label = self._section_label(name)
            sidebar_layout.addWidget(label)
            self._sidebar_sections[name] = {"label": label, "divider": None}
            return label

        def _close_section(name):
            hr = self._hr()
            sidebar_layout.addWidget(hr)
            self._sidebar_section_dividers[name] = hr
            if name in self._sidebar_sections:
                self._sidebar_sections[name]["divider"] = hr

        _open_section("Tuning")
        tuning_btn = self._mk_sidebar_btn("  ⊿  Tuning Panel", self.show_tuning_panel)
        sidebar_layout.addWidget(tuning_btn)
        _close_section("Tuning")

        _open_section("Programming")
        _close_section("Programming")

        _open_section("Teleoperation")
        _close_section("Teleoperation")

        _open_section("Experiments")
        _close_section("Experiments")

        system_label = _open_section("System")
        self._sidebar_new_section_anchor = system_label
        robot_config_btn = self._mk_sidebar_btn("  ⊕  Robot Config", self.show_robot_config_panel)
        sidebar_layout.addWidget(robot_config_btn)
        topic_monitor_btn = self._mk_sidebar_btn("  ◉  Topic Monitor", self.show_topic_monitor_panel)
        sidebar_layout.addWidget(topic_monitor_btn)
        self._safety_btn = self._mk_sidebar_btn("  ⊘  Safety", self.show_safety_panel)
        sidebar_layout.addWidget(self._safety_btn)
        _close_section("System")

        ext_sidebar_buttons = self._add_extension_sidebar_entries(sidebar_layout)

        self._hide_empty_sidebar_sections(sidebar_layout)

        sidebar_layout.addStretch(1)

        sidebar_scroll.setWidget(sidebar_inner)
        sidebar_content_layout.addWidget(sidebar_scroll, 1)

        # Settings pinned at bottom of sidebar (inside collapsible content)
        settings_btn = self._mk_sidebar_btn("  ≡  Settings", self.show_settings_panel)
        settings_btn.setObjectName("SideButtonBottom")
        settings_hr = self._hr()
        sidebar_content_layout.addWidget(settings_hr)
        sidebar_content_layout.addWidget(settings_btn)

        # Logs pinned above Help.
        self._logs_sidebar_btn = self._mk_sidebar_btn("  ▤  Logs", self.show_logs_panel)
        self._logs_sidebar_btn.setObjectName("SideButtonBottom")
        sidebar_content_layout.addWidget(self._logs_sidebar_btn)

        # Help pinned at bottom of sidebar (below Logs)
        help_btn = self._mk_sidebar_btn("  ?  Help", self.show_help_panel)
        help_btn.setObjectName("SideButtonBottom")
        sidebar_content_layout.addWidget(help_btn)

        sidebar_outer_layout.addWidget(self._sidebar_content, 1)

        # Kinova hides its pages menu behind the ☰ by default; so does this.
        # Always collapsed on launch — ☰ reveals it for the session.
        self._sidebar_expanded = True
        self._toggle_sidebar_collapse()

        self._side_buttons = [
            dashboard_btn,                                      # 0
            robot_ctrl_btn, tuning_btn, None,                   # 1, 2, 3 (autotuning — extension)
            None, None, None,                                   # 4, 5, 6 (sequences/teach/script — extensions)
            None,                                               # 7 (teleop — now built into Multi-Robot)
            None,                                               # 8 (experiments — extension)
            settings_btn, robot_config_btn, topic_monitor_btn,  # 9, 10, 11
            None,                                               # 12 (tracker — extension)
            *ext_sidebar_buttons,                               # 13+ (extensions)
            self._logs_sidebar_btn,                             # Logs (pinned bottom)
            help_btn,                                           # last: Help (pinned bottom)
            self._safety_btn,                                   # Safety (active-state mgmt)
            self._locomotion_sidebar_btn,                       # Locomotion (legged robots only)
        ]
        self._help_sidebar_btn = help_btn

        # The stack does not exist yet, so record the pairing and bind it
        # once the pages are registered, further down this method.
        self._pending_page_buttons = (
            ('dashboard', dashboard_btn), ('multi_robot', robot_ctrl_btn),
            ('tuning', tuning_btn), ('settings', settings_btn),
            ('robot_config', robot_config_btn),
            ('topic_monitor', topic_monitor_btn),
            ('logs', self._logs_sidebar_btn), ('help', help_btn),
            ('safety', self._safety_btn),
            ('locomotion', self._locomotion_sidebar_btn),
        )

        self._loco_refresh_visibility()
        
        # ── MAIN CONTENT AREA ──
        main_content = QWidget()
        main_content_layout = QVBoxLayout(main_content)
        main_content_layout.setContentsMargins(0, 0, 0, 0)
        main_content_layout.setSpacing(0)
        
        self.main_stack = QStackedWidget()
        self.main_stack.setObjectName("MainContent")
        # Pages are addressed by name. The seven empty placeholder slots this
        # replaced existed only to reserve indices for extensions, which have
        # appended to the end of the stack for some time — they were reserving
        # nothing, and made every index below them load-bearing.
        self.pages = PageRegistry(self.main_stack)
        self.pages.register('dashboard', self._build_dashboard_page())
        self.pages.register('multi_robot', self._build_multi_robot_page())
        self.pages.register('tuning', self._build_tuning_page())
        self.pages.register('settings', self._build_settings_page())
        self.pages.register('robot_config', self._build_robot_config_page())
        self.pages.register('topic_monitor', self._build_topic_monitor_page())
        self.pages.register('help', self._build_help_page())
        self.pages.register('safety', self._build_safety_page())
        self.pages.register('locomotion', self._build_locomotion_page())

        # Extension pages (built from instances created in _init_extensions).
        self._add_extension_stack_pages()

        self.pages.register('logs', self._build_logs_page())

        for _page, _btn in self._pending_page_buttons:
            self.pages.bind_button(_page, _btn)
        for _eid, _btn in getattr(self, '_extension_sidebar_buttons', {}).items():
            self.pages.bind_button(_eid, _btn)

        main_content_layout.addWidget(self.main_stack, 1)
        
        root.addWidget(self.sidebar)
        root.addWidget(main_content, 1)
        self.pages.show('dashboard')
    
    # ══════════════════════════════════════════════════════════════════════════
    #  STATUS BAR — no inline styles, all from global theme
    # ══════════════════════════════════════════════════════════════════════════
    
    def _build_logs_page(self):
        """Build the Logs page.

        The log viewer used to be a fixed-height strip pinned under every
        page, competing for vertical space with the content and, since the
        control bar landed, with the bar as well. On its own page it gets
        the whole area and needs no collapse affordance.
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(22, 20, 22, 12)
        layout.setSpacing(12)
        layout.addWidget(self._page_title("Logs"))

        self.log_viewer = LogViewer()
        self.log_viewer.log_text.setVisible(True)
        # No collapsing on a full page — the ☰ only made sense on a strip.
        self.log_viewer.collapse_btn.setVisible(False)
        layout.addWidget(self.log_viewer, 1)
        return page

    def _setup_statusbar(self):
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
        self.fic_led = StatusLED()
        conn_layout.addWidget(self.fic_led)
        conn_layout.addWidget(QLabel("FIC"))
        conn_layout.addWidget(QLabel("  "))
        self.sim_led = StatusLED()
        conn_layout.addWidget(self.sim_led)
        conn_layout.addWidget(QLabel("SIM"))
        conn_layout.addWidget(QLabel("  "))
        self.bags_led = StatusLED()
        conn_layout.addWidget(self.bags_led)
        conn_layout.addWidget(QLabel("BAGS"))
        conn_layout.addWidget(QLabel("  "))
        # Inner loop: the FIC MCU (Ethernet / Bridge) or the standard path.
        self.il_led = StatusLED()
        conn_layout.addWidget(self.il_led)
        conn_layout.addWidget(QLabel("INNER"))
        self.il_label = QLabel("…")
        theme.bind(
            self.il_label,
            lambda t: "color: %s; font-size: 11px;" % t["text_secondary"],
        )
        conn_layout.addWidget(self.il_label)
        conn_layout.addWidget(QLabel("  "))
        # The arm's own Kortex API from Linux (one-switch layout only): state,
        # faults, camera; menu: clear faults, home, Kortex e-stop.
        self.kx_led = StatusLED()
        conn_layout.addWidget(self.kx_led)
        self.kx_btn = QToolButton()
        self.kx_btn.setText("ARM · –")
        self.kx_btn.setAutoRaise(True)
        self.kx_btn.setPopupMode(QToolButton.InstantPopup)
        kx_menu = QMenu(self.kx_btn)
        self._kx_actions = [
            kx_menu.addAction("Clear faults", lambda: self._kx_menu("clear_faults")),
            kx_menu.addAction("Home the arm…", lambda: self._kx_menu("home")),
        ]
        kx_menu.addSeparator()
        self._kx_actions.append(kx_menu.addAction("Kortex e-stop", lambda: self._kx_menu("estop")))
        self.kx_btn.setMenu(kx_menu)
        conn_layout.addWidget(self.kx_btn)
        conn_layout.addWidget(QLabel("  "))
        self.teleop_led = StatusLED()
        conn_layout.addWidget(self.teleop_led)
        conn_layout.addWidget(QLabel("TELEOP"))
        self.teleop_status_bar_label = QLabel("")
        theme.bind(
            self.teleop_status_bar_label,
            lambda t: "color: %s; font-size: 11px;" % t["text_secondary"],
        )
        conn_layout.addWidget(self.teleop_status_bar_label)
        conn_layout.addWidget(QLabel("  "))
        # Gamepad indicator — green when at least one USB pad is plugged in.
        # Driven by GamepadMixin.gamepad_signals.connected_changed; the mixin
        # is initialised before _setup_statusbar in __init__, so the signal
        # is already wired by the time the first scan completes.
        self.gamepad_led = StatusLED()
        self.gamepad_led.set_status(
            'connected' if getattr(self, '_gp_last_connected', False) else 'idle')
        conn_layout.addWidget(self.gamepad_led)
        self.gamepad_state_label = QLabel("GAMEPAD")
        theme.bind(
            self.gamepad_state_label,
            lambda t: "color: %s; font-family: monospace; font-size: 10px;" % t["text_secondary"],
        )
        conn_layout.addWidget(self.gamepad_state_label)
        if hasattr(self, "gamepad_signals"):
            self.gamepad_signals.connected_changed.connect(
                lambda c: self.gamepad_led.set_status('connected' if c else 'idle'))
            self.gamepad_signals.state_changed.connect(
                lambda s: self.gamepad_state_label.setText(s if s else "GAMEPAD"))
        self.statusbar.addWidget(conn_widget)
        
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.statusbar.addWidget(spacer)
        
        self.robot_status_label = QLabel("Robot: Franka_FR3 (7 DOF)")
        self.statusbar.addPermanentWidget(self.robot_status_label)
    
    # ══════════════════════════════════════════════════════════════════════════
    #  STATUS UPDATE
    # ══════════════════════════════════════════════════════════════════════════
    #  TOOLBAR BUTTON STYLES (inline — QToolBar overrides global QSS on children)
    # ══════════════════════════════════════════════════════════════════════════

    # These MUST be inline setStyleSheet because Qt's QToolBar aggressively
    # re-applies its own styling to child widgets, overriding objectName-based
    # and property-based selectors from the global stylesheet.
    def _tb_styles(self):
        """Build toolbar button styles from the active theme tokens."""
        key = ('styles', theme.current_name())
        if key in _TB_QSS:
            return _TB_QSS[key]
        t = theme.current()

        def _idle(_accent):
            return theme.button_qss(t, 'secondary')

        run = _tb_filled(t, t['accent_red'])
        _TB_QSS[key] = {
            'hw': _idle(t['accent_teal']), 'hw_run': run,
            'sim': _idle(t['accent_blue']), 'sim_run': run,
            'vr': _idle(t['accent_green']), 'vr_run': run,
            'util': theme.button_qss(t, 'secondary'), 'util_run': run,
        }
        return _TB_QSS[key]

    def _restyle_toolbar(self):
        """Re-apply theme-derived styles to the toolbar buttons."""
        styles = self._tb_styles()
        for btn in (self.tb_exp_btn, self.tb_bags_btn, self.tb_plot_btn,
                    self.tb_viewer_btn):
            key = 'util_run' if '■' in btn.text() else 'util'
            btn.setStyleSheet(styles[key])
        self._refresh_toolbar_launch()

    def _restyle_btn(self, btn: QPushButton, text: str, running: bool, kind: str):
        """Apply inline style to a toolbar button (only way that sticks in QToolBar)."""
        style_key = f'{kind}_run' if running else kind
        new_style = self._tb_styles()[style_key]
        if btn.styleSheet() != new_style:
            btn.setStyleSheet(new_style)
        if btn.text() != text:
            btn.setText(text)

    def _tb_session_running(self):
        """True if the current target's processes are up."""
        t = self.control_target
        if t == "simulation":
            return self.simulation_running or self.fic_running
        if t == "hardware":
            return self.driver_running or self.controller_running
        if t == "vr":
            return getattr(self, "vr_relay_running", False)
        return False

    def _tb_toggle_start(self):
        """Start/stop the current target's processes via the combined toggles."""
        t = self.control_target
        if t == "simulation":
            self._dash_toggle_sim()
        elif t == "hardware":
            self._dash_toggle_hw()
        elif t == "vr":
            self._toggle_vr()
        self._update_status()

    def _tb_set_target(self, target):
        """Toolbar target click → set the shared control target (locked while running)."""
        if self._tb_session_running():
            return
        self._set_control_target(target)
        self._refresh_toolbar_launch()

    def _tb_target_styles(self):
        """Return (on, off) QSS for the target segments from the active theme."""
        key = ('target', theme.current_name())
        if key not in _TB_QSS:
            t = theme.current()
            _TB_QSS[key] = (_tb_filled(t, t['accent_blue']), theme.button_qss(t, 'quiet'))
        return _TB_QSS[key]

    def _tb_startstop_styles(self):
        """Return (start, stop) QSS for the Start/Stop button from the active theme."""
        key = ('startstop', theme.current_name())
        if key not in _TB_QSS:
            t = theme.current()
            _TB_QSS[key] = (_tb_filled(t, t['accent_blue']), _tb_filled(t, t['accent_red']))
        return _TB_QSS[key]

    def _refresh_toolbar_launch(self):
        """Sync the launch cluster to control_target + running state."""
        running = self._tb_session_running()
        target = self.control_target
        on, off = self._tb_target_styles()
        for key, b in self._tb_target_btns.items():
            _set_qss(b, on if key == target else off)
            b.setEnabled(not running)
        start, stop = self._tb_startstop_styles()
        self._tb_start_btn.setText("■ Stop" if running else "▶ Start")
        _set_qss(self._tb_start_btn, stop if running else start)
        if hasattr(self, "_tb_robot_id"):
            self._tb_robot_id.setEnabled(not running)
        self.robot_dropdown.setEnabled(not running)

        # The contact viewer button stays visible on every target. It was
        # briefly hidden unless target == 'hardware', but control_target
        # defaults to 'simulation' (robot_control_mixin.py), so on a fresh
        # launch the button was invisible — indistinguishable from "the
        # feature isn't installed". The viewer also genuinely works in
        # simulation: fic_control drives the same joint-admittance path
        # and publishes {rid}_admittance_offset, so there is nothing to
        # protect the user from here.
        if hasattr(self, 'tb_viewer_btn'):
            self.tb_viewer_btn.setVisible(True)

    def _tb_on_robot_id_changed(self, idx):
        """Switch which station this GUI drives: persist + restart to re-point."""
        import os
        import sys
        new_sid = self._tb_robot_id.itemData(idx)
        current = self._tb_robot_id.findData(picker_value(self.node.station_id))
        if new_sid is None or new_sid == picker_value(self.node.station_id):
            return
        if self._tb_session_running():
            self._tb_robot_id.setCurrentIndex(current)
            return
        reply = QMessageBox.question(
            self, 'Switch Robot',
            'Switch this Studio to %s?\n\nThe app will restart to re-point its '
            'ROS topics.' % station_label(new_sid),
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if reply != QMessageBox.Yes:
            self._tb_robot_id.setCurrentIndex(current)
            return
        self.settings['general']['station_id'] = new_sid
        self._save_settings()
        os.execv(sys.executable, [sys.executable] + sys.argv)

    def _update_status(self):
        self.fic_led.set_status('connected' if self.fic_running else 'idle')
        self.sim_led.set_status('connected' if self.simulation_running else 'idle')
        vr_running = getattr(self, 'vr_relay_running', False)
        self._update_vr_label()
        if vr_running:
            if hasattr(self, '_poll_vr_scene_config'):
                self._poll_vr_scene_config()
            if hasattr(self, '_poll_vr_mode'):
                self._poll_vr_mode()
        self._refresh_toolbar_launch()

        # Toolbar EXP button: state from the experiments extension if loaded.
        exp_ext = self.get_extension("experiments")
        exp_running = bool(exp_ext is not None and getattr(exp_ext, "state_running", False))
        if exp_ext is None:
            self.tb_exp_btn.setVisible(False)
        else:
            self.tb_exp_btn.setVisible(True)
            self._restyle_btn(self.tb_exp_btn,
                "■ EXP" if exp_running else "▶ EXP",
                exp_running, 'util')

        self.bags_led.set_status('warning' if self.bags_running else 'idle')
        self._il_update_statusbar()
        self._restyle_btn(self.tb_bags_btn,
            "■ BAG" if self.bags_running else "▶ BAG",
            self.bags_running, 'util')

        self._restyle_btn(self.tb_plot_btn,
            "■ PLT" if self.plotter_running else "▶ PLT",
            self.plotter_running, 'util')

        if hasattr(self, '_dash_update_action_labels'):
            self._dash_update_action_labels()

        # Teleop status LED + label — built-in teleop (Multi-Robot page).
        role = self.teleop_role() if hasattr(self, 'teleop_role') else ''
        if role:
            _t = theme.current()
            if role == "leader":
                self.teleop_led.set_status('active')
                self.teleop_status_bar_label.setText("LEADER")
                self.teleop_status_bar_label.setStyleSheet(
                    "color: %s; font-size: 11px; font-weight: 600;"
                    % _t["accent_blue"]
                )
            elif role == "follower":
                self.teleop_led.set_status('connected')
                self.teleop_status_bar_label.setText("FOLLOWER")
                self.teleop_status_bar_label.setStyleSheet(
                    "color: %s; font-size: 11px; font-weight: 600;"
                    % _t["accent_green"]
                )
            elif role == "waiting":
                self.teleop_led.set_status('warning')
                self.teleop_status_bar_label.setText("WAITING")
                self.teleop_status_bar_label.setStyleSheet(
                    "color: %s; font-size: 11px;" % _t["accent_orange"]
                )
            else:
                self.teleop_led.set_status('idle')
                self.teleop_status_bar_label.setText("")
        else:
            self.teleop_led.set_status('idle')
            self.teleop_status_bar_label.setText("")
    
    # ══════════════════════════════════════════════════════════════════════════
    #  NAVIGATION
    # ══════════════════════════════════════════════════════════════════════════

    def _show_page(self, name: str) -> bool:
        """Navigate to a registered page and mark its sidebar entry active."""
        if not self.pages.show(name):
            self.log_message('WARN', 'No page registered as %r' % name, 'Navigation')
            return False
        button = self.pages.button(name)
        if button is not None:
            self._set_active_sidebar_btn(button)
        return True

    def is_page_visible(self, name: str) -> bool:
        """True when ``name`` is the page on screen.

        Periodic refreshers ask this rather than comparing an index, so
        navigation and throttling cannot disagree about what is showing.
        """
        registry = getattr(self, 'pages', None)
        return bool(registry and registry.is_visible(name))

    def _set_active_sidebar_btn(self, active_btn: QPushButton):
        for btn in self._side_buttons:
            if btn is None:
                continue
            btn.setProperty("active", btn == active_btn)
            btn.style().unpolish(btn)
            btn.style().polish(btn)
            btn.update()
    
    def show_dashboard(self):
        self._show_page('dashboard')
    def show_multi_robot_panel(self):
        self._show_page('multi_robot')

    def show_robot_control_panel(self):
        """Open the Joint control menu on the bar.

        Robot Control is no longer a page. Kept because settings_mixin, the
        first-launch wizard and main_window_dev all still navigate here.
        """
        bar = getattr(self, '_control_bar', None)
        if bar is not None:
            bar.open_panel('joint')
        self._rc_refresh_joint_count()
    def show_tuning_panel(self):
        self._show_page('tuning')
        self._load_tuning_parameters()
    # Shims for the menu / file-open paths that previously called methods on
    # the now-extension mixins. Each routes via the loaded extension if any.
    def show_sequence_panel(self) -> bool:
        if self.get_extension("sequences") is not None:
            self._show_extension_panel("sequences")
            return True
        return False

    def show_experiment_panel(self) -> bool:
        if self.get_extension("experiments") is not None:
            self._show_extension_panel("experiments")
            return True
        return False

    def _quick_start_experiment(self):
        ext = self.get_extension("experiments")
        if ext is None:
            self.log_message("WARN",
                "Experiments extension not enabled — turn it on in Settings → Extensions.",
                "Experiment")
            return
        try:
            ext.quick_start()
        except Exception as e:
            self.log_message("ERROR", f"quick_start failed: {e}", "Experiment")

    def show_settings_panel(self):
        self._show_page('settings')
    def show_robot_config_panel(self):
        self._show_page('robot_config')
    def show_topic_monitor_panel(self):
        self._show_page('topic_monitor')
        if hasattr(self, '_refresh_topic_list'):
            self._refresh_topic_list()

    def show_logs_panel(self):
        """Open the Logs page."""
        self._show_page('logs')
        return
        self._set_active_sidebar_btn(self._logs_sidebar_btn)

    def show_help_panel(self):
        """Navigate to the Help page."""
        self._show_page('help')
        return
        self._set_active_sidebar_btn(self._help_sidebar_btn)

    def show_locomotion_panel(self):
        """Open the locomotion page."""
        self._show_page('locomotion')
        return
        self._set_active_sidebar_btn(self._locomotion_sidebar_btn)
    
    # ══════════════════════════════════════════════════════════════════════════
    #  ACTION HANDLERS
    # ══════════════════════════════════════════════════════════════════════════
    
    def _on_robot_changed(self, robot_name: str):
        if not robot_name:
            return
        num_joints = self.robots.get(robot_name, 7)
        self._js_num_joints = num_joints    # read by the state-thread callbacks
        if hasattr(self, 'joint_sliders_layout'):
            self._build_joint_sliders(num_joints)
        self._update_tuning_joint_count()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        def_joints = cfg.get('q_home_deg', [0.0] * num_joints)
        # Update extra hardware robot cards (Robot 2-4)
        for robot_id in getattr(self, 'real_joint_layout', {}):
            self._build_extra_joint_sliders(robot_id, num_joints, default_values=def_joints, robot_type=robot_name)
            count_sb = self.real_joint_count.get(robot_id)
            if count_sb:
                count_sb.blockSignals(True)
                count_sb.setValue(num_joints)
                count_sb.blockSignals(False)
        if hasattr(self, 'tuning_robot_label'):
            self.tuning_robot_label.setText(f"Robot: {robot_name}")
        if hasattr(self, 'load_robot_safety_limits'):
            self.load_robot_safety_limits(robot_name)
        self.robot_status_label.setText(f"Robot: {robot_name} ({num_joints} DOF)")
        self.log_message("INFO", f"Selected robot: {robot_name}", "Robot")
        self._sync_robot_menu_check()
        self._loco_refresh_visibility()
        if self.locomotion_running and not self._loco_is_legged(robot_name):
            self._stop_locomotion()
            self.log_message(
                "INFO",
                f"Locomotion controller stopped: {robot_name} is not a legged robot",
                "Robot")
        if hasattr(self, '_rc_apply_mode_availability'):
            self._rc_apply_mode_availability(robot_name)
    
    def _select_robot_from_menu(self, robot_name: str):
        idx = self.robot_dropdown.findText(robot_name)
        if idx >= 0:
            self.robot_dropdown.setCurrentIndex(idx)

    def _sync_robot_menu_check(self):
        """Tick the Robot ▸ Select Robot entry that matches the active robot.

        setChecked() on a QActionGroup member does not re-emit triggered, so
        there's no risk of feedback into _select_robot_from_menu.
        """
        group = getattr(self, 'robot_action_group', None)
        dropdown = getattr(self, 'robot_dropdown', None)
        if group is None or dropdown is None:
            return
        current = dropdown.currentText()
        for action in group.actions():
            action.setChecked(action.text() == current)

    def _sync_mode_menu_check(self):
        """Tick the Robot ▸ Control Mode entry that matches the active mode.

        The live mode source of truth is the (hidden) Robot Control panel mode
        dropdown _rc_mode_dropdown (0-indexed). tb_mode_dropdown is honoured
        first if it ever exists, per the menu-sync contract. setChecked() does
        not re-emit triggered, so this can't loop back into _set_mode_from_menu.
        """
        if not hasattr(self, '_mode_action_group'):
            return
        dropdown = (getattr(self, 'tb_mode_dropdown', None)
                    or getattr(self, '_rc_mode_dropdown', None))
        if dropdown is None:
            return
        mode = dropdown.currentIndex() + 1
        for action in self._mode_action_group.actions():
            action.setChecked(action.text().startswith(f"{mode}:"))

    def _emergency_stop(self):
        self.log_message("ERROR", "E-STOP ACTIVATED! HALTING ALL SYSTEMS AND PROCESSES.", "Safety")
        # A second stop path straight to the arm (Kortex ApplyEmergencyStop),
        # when the Kortex supervisor runs (one-switch layout). First, so nothing
        # below can delay it; without the supervisor this does nothing.
        if hasattr(self, 'kortex_command'):
            try:
                sent, _ = self.kortex_command('estop')
                if sent:
                    self.log_message("ERROR", "Kortex e-stop sent to the arm.", "Safety")
            except Exception as exc:                      # never block the E-stop
                self.log_message("WARN", "Kortex e-stop not sent: %s" % exc, "Safety")
        # Straight to the joint inner loop (fic_inner_loop on this PC, the MCU board): it ramps the arm
        # down on its own, even if fic_control is stuck. One UDP datagram each; never blocks.
        if hasattr(self, 'estop_inner_loops'):
            self.estop_inner_loops()
        # latched until Reset (who fired it: set by the web, fleet and gamepad paths)
        if hasattr(self, '_estop_latch_set'):
            self._estop_latch_set(getattr(self, '_estop_source', '') or 'Studio window')
        self._estop_source = ''
        # Fan out E-stop to every loaded extension (sequences, teach_mode,
        # script_editor, experiments, autotuning, teleop, tracker, ...).
        # Each extension's on_emergency_stop() halts its own activity.
        self._estop_extensions()
        # End teleop, and E-stop the partner arms (a follower must not keep
        # copying, and a leader's operator must know the follower stopped).
        if hasattr(self, '_fleet_on_local_estop'):
            self._fleet_on_local_estop()
        # 1) Command the controller's GRACEFUL torque ramp-down FIRST, so the
        #    robot decays to a safe hold instead of freezing its last torque
        #    when its process is killed below (WS1.1 safety fix).
        self._estop_command_safe_stop()
        # 2) Give the controller a brief window to ramp, then tear everything down.
        self._estop_ramp_wait()
        self._estop_kill_processes()

    def _estop_command_safe_stop(self):
        """Issue the graceful stop before any process is killed.

        Publishes mode 0 ('Stop Controller') to every active robot so the C++
        controller ramps its torque down to a safe idle, then locks each robot
        at its last-known joint pose. This is the safe-stop contract; the
        process teardown that follows is only a backstop.
        """
        try:
            if hasattr(self, 'broadcast_stop'):
                self.broadcast_stop()
        except Exception as e:
            self.log_message("ERROR", f"E-STOP graceful stop failed: {e}", "Safety")
        from std_msgs.msg import Float32MultiArray
        positions_by_robot = getattr(self, 'current_joint_positions_by_robot', {}) or {}
        for rid, pub in self.node.real_joint_publishers.items():
            pos = positions_by_robot.get(rid)
            if not pos and rid == self.node.station_robot_id:
                pos = getattr(self, 'current_joint_positions', None)
            if not pos:
                continue
            msg = Float32MultiArray()
            msg.data = [float(j) for j in pos]
            pub.publish(msg)

    def _estop_ramp_wait(self):
        """Before anything is killed, let the arm leave torque mode the way Stop does, so it neither jerks nor
        drops: every inner loop the E-STOP reached finishes its ramp (out of torque, the stop flag raised), then
        this Studio's vendor driver leaves torque mode by itself (the Kinova driver re-engages position mode and
        servoing, then exits). Killed while still in torque mode, a driver stops without the ramp. At least
        ``_ESTOP_RAMP_MS`` (250 ms; 0 skips the wait, for tests), at most ``_ESTOP_RELEASE_TIMEOUT_S`` (5 s; the
        ramp from 20 N m takes about 1.2 s, the driver's switch a few hundred ms)."""
        ms = getattr(self, '_ESTOP_RAMP_MS', 250)
        if ms <= 0:
            return
        import time
        from PyQt5.QtWidgets import QApplication
        from fic_studio import inner_loop_estop
        t0 = time.monotonic()
        deadline = t0 + getattr(self, '_ESTOP_RELEASE_TIMEOUT_S', 5.0)
        targets = list(getattr(self, '_estop_targets', None) or [])
        driver = None if getattr(self, '_mcu_driver', False) else getattr(self, 'driver_process', None)
        busy, answered = list(targets), False
        while time.monotonic() < deadline:
            states = {t: inner_loop_estop.probe_state(t, 0.05) for t in busy}
            answered = answered or any(v is not None for v in states.values())
            busy = [t for t, v in states.items() if v in inner_loop_estop.TORQUING]
            driver_up = driver is not None and driver.poll() is None
            if not busy and time.monotonic() - t0 >= ms / 1000.0:
                if not driver_up:
                    return
                if not answered:
                    break        # no inner loop raises the stop flag, so the driver will not leave on its own
            QApplication.processEvents()
            time.sleep(0.01)
        still = list(busy)
        if driver is not None and driver.poll() is None:
            still.append('the vendor driver')
        if still:
            self.log_message("ERROR", "E-STOP: still in torque mode after %.1f s (%s); stopping the processes "
                             "anyway" % (time.monotonic() - t0, ", ".join(still)), "Safety")

    def _estop_kill_processes(self):
        """Hard teardown backstop: stop every process this Studio started, then SIGKILL whatever is
        left in their sessions. Runs after the graceful ramp window. Only this Studio's processes:
        other users' simulators and launches on the same PC are left alone."""
        sessions = self._owned_sessions()
        if hasattr(self, 'controller_running') and self.controller_running:
            if hasattr(self, '_stop_controller'):
                self._stop_controller()
        if hasattr(self, 'driver_running') and self.driver_running:
            if hasattr(self, '_stop_driver'):
                self._stop_driver()
        if hasattr(self, 'fic_running') and self.fic_running:
            if hasattr(self, '_stop_fic_simulation'):
                self._stop_fic_simulation()
        if hasattr(self, 'simulation_running') and self.simulation_running:
            if hasattr(self, '_stop_simulation'):
                self._stop_simulation()
        if hasattr(self, 'bags_running') and self.bags_running:
            if hasattr(self, '_stop_bags'):
                self._stop_bags()
        if hasattr(self, 'plotter_running') and self.plotter_running:
            if hasattr(self, '_stop_plotter'):
                self._stop_plotter()
        if hasattr(self, 'robot_status_label'):
            self.robot_status_label.setText("E-STOP ACTIVE - ALL PROCESSES KILLED")
            self.robot_status_label.setObjectName("EStopLabel")
            self.robot_status_label.style().unpolish(self.robot_status_label)
            self.robot_status_label.style().polish(self.robot_status_label)
        from fic_studio.mixins.process_mixin import _kill_session
        self._kill_all_processes(reason="E-STOP")
        left = sum(_kill_session(sid) for sid in sessions)
        # the background VR relay drives no robot: headsets keep finding this Studio
        if hasattr(self, '_vr_bg_watchdog'):
            QTimer.singleShot(1000, self._vr_bg_watchdog)
        self.log_message("WARN", "E-STOP: this Studio's processes stopped"
                         + (f" ({left} left over were killed)" if left else ""), "Safety")

    def _move_home(self):
        robot_name = self.robot_dropdown.currentText()
        config = self.robot_configs.get(robot_name, {})
        home_angles = config.get('q_home_rad', [0, -0.785, 0, -1.57, 0, 0.785, 0])
        radians = [float(a) for a in home_angles]
        if self.control_target in ("simulation", "vr"):
            names = [f"{robot_name}_j{i+1}" for i in range(len(radians))]
            self.node.publish_joint_angles(radians, names=names)
        else:
            self.node.publish_real_joint_angles(self.node.station_robot_id, radians)
        deg = ", ".join(f"{math.degrees(a):.1f}°" for a in radians)
        self.log_message("INFO", f"Moving to home: [{deg}]", "Control")
    
    def _toggle_sidebar(self, checked: bool):
        self._sidebar_expanded = checked
        self._sidebar_content.setVisible(checked)
        self.sidebar.setFixedWidth(230 if checked else 60)
    def _toggle_toolbar(self, checked: bool):
        self.main_toolbar.setVisible(checked)
    def _toggle_statusbar(self, checked: bool):
        self.statusbar.setVisible(checked)
    
    def _toggle_minimal_mode(self, checked: bool):
        self._minimal_mode = checked
        if checked:
            self._pre_minimal_states = {
                'toolbar': self.main_toolbar.isVisible(),
                'sidebar': self.sidebar.isVisible(),
                'statusbar': self.statusbar.isVisible(),
            }
            self.main_toolbar.setVisible(False)
            self.sidebar.setVisible(False)
            self.statusbar.setVisible(False)
            self.toggle_toolbar_action.setChecked(False)
            self.toggle_sidebar_action.setChecked(False)
            self.toggle_statusbar_action.setChecked(False)
        else:
            states = getattr(self, '_pre_minimal_states', {})
            self.main_toolbar.setVisible(states.get('toolbar', True))
            self.sidebar.setVisible(states.get('sidebar', True))
            self.statusbar.setVisible(states.get('statusbar', True))
            self.toggle_toolbar_action.setChecked(states.get('toolbar', True))
            self.toggle_sidebar_action.setChecked(states.get('sidebar', True))
            self.toggle_statusbar_action.setChecked(states.get('statusbar', True))
            self._update_log_toggle_label()
        self.log_message("INFO", f"Minimal mode: {'ON' if checked else 'OFF'}", "View")
    
    def _toggle_compact_mode(self, checked: bool):
        self._compact_mode = checked
        if checked:
            self.sidebar.setFixedWidth(180)
            btn_height = 22
            btn_widths = {'ctrl': 58, 'mode': 58, 'fic': 50, 'sim': 50, 'exp': 50, 'bags': 50, 'plot': 50, 'estop': 65}
            dropdown_height = 22
        else:
            self.sidebar.setFixedWidth(230)
            btn_height = 26
            btn_widths = {'ctrl': 70, 'mode': 70, 'fic': 60, 'sim': 60, 'exp': 60, 'bags': 60, 'plot': 60, 'estop': 80}
            dropdown_height = 26
        self.tb_ctrl_btn.setFixedSize(btn_widths['ctrl'], btn_height)
        self.tb_mode_btn.setFixedSize(btn_widths['mode'], btn_height)
        self.tb_fic_btn.setFixedSize(btn_widths['fic'], btn_height)
        self.tb_sim_btn.setFixedSize(btn_widths['sim'], btn_height)
        self.tb_exp_btn.setFixedSize(btn_widths['exp'], btn_height)
        self.tb_bags_btn.setFixedSize(btn_widths['bags'], btn_height)
        self.tb_plot_btn.setFixedSize(btn_widths['plot'], btn_height)
        self.estop_btn.setFixedSize(btn_widths['estop'], btn_height)
        self.robot_dropdown.setFixedHeight(dropdown_height)
        self.log_message("INFO", f"Compact mode: {'ON' if checked else 'OFF'}", "View")
        
    def _set_ui_scale(self, scale: int):
        self._ui_scale = scale
        factor = scale / 100.0
        self.setStyleSheet(build_stylesheet(
            self.current_theme, scale_overlay_qss(scale)))
        self.sidebar.setFixedWidth(int(230 * factor))
        self.log_message("INFO", f"UI Scale: {scale}%", "View")

    def _toggle_sidebar_collapse(self):
        """Collapse or expand the sidebar.

        Deliberately not persisted. Every session starts with the sidebar
        collapsed, the way Kinova's pages menu is hidden until you click the
        menu icon; the toggle is a per-session convenience, not a stored
        preference. Persisting it also meant writing the settings file as a
        side effect of building the window, which stamped whatever happened
        to be in the settings dict at that moment on every launch.
        """
        self._sidebar_expanded = not self._sidebar_expanded
        expanded = self._sidebar_expanded
        self._sidebar_content.setVisible(expanded)
        self.sidebar.setFixedWidth(230 if expanded else 60)
        self.toggle_sidebar_action.setChecked(expanded)

    def _toggle_log_collapse(self):
        """Open the Logs page (the collapsible strip no longer exists)."""
        self.show_logs_panel()

    def _update_log_toggle_label(self):
        pass

    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
    
    def _new_script(self):
        # File > New Script — route through the script_editor extension if loaded.
        ext = self.get_extension("script_editor")
        if ext is None:
            self.log_message("WARN",
                "Script Editor extension not enabled — turn it on in Settings → Extensions.",
                "File")
            return
        self._show_extension_panel("script_editor")
        if hasattr(ext, "_new_script"):
            ext._new_script()

    def _save_current(self):
        # File > Save — context-sensitive on the currently visible page.
        #  - Script Editor page → save the current script.
        #  - Settings page (index 9) → save settings from the UI.
        #  - anything else → there's nothing this action can save.
        ext = self.get_extension("script_editor")
        if (self.is_page_visible("script_editor")
                and ext is not None and hasattr(ext, "_save_script")):
            ext._save_script()
        elif current_idx == 9:
            self._save_settings_from_ui()
        else:
            self.log_message(
                "WARN",
                "Nothing to save on this page — use File ▸ Export for "
                "config/parameters, or open the Script Editor.",
                "File")
    
    def _show_about(self):
        QMessageBox.about(self, f"About {self.APP_NAME}",
            f"<h2>{self.APP_NAME}</h2><p>Version {version.display()}</p>"
            f"<p>Professional Robot Control Interface</p><p>Built with PyQt5 and ROS2</p>")

    # ══════════════════════════════════════════════════════════════════════════
    #  MENU BAR HANDLER METHODS
    # ══════════════════════════════════════════════════════════════════════════

    def _open_experiment_spec(self):
        """File > Open > Experiment Spec — delegates to experiment mixin."""
        if not self.show_experiment_panel():
            self.log_message("WARN",
                "Experiments extension not enabled — turn it on in Settings → Extensions.",
                "File")

    def _open_sequence_file(self):
        """File > Open > Sequence File — delegates to the sequences extension."""
        if not self.show_sequence_panel():
            self.log_message("WARN",
                "Sequences extension not enabled — turn it on in Settings → Extensions.",
                "File")
            return
        seq_ext = self.get_extension("sequences")
        if seq_ext is not None and hasattr(seq_ext, "_load_sequences_from_yaml"):
            try:
                seq_ext._load_sequences_from_yaml()
            except Exception:
                pass

    def _open_config_snapshot(self):
        """File > Open > Config Snapshot — load an XML snapshot over the active config."""
        import shutil
        from fic_studio.config import XML_FILE_PATH
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Config Snapshot", "", "XML Files (*.xml);;All Files (*)")
        if not path:
            return
        try:
            shutil.copy2(path, XML_FILE_PATH)
            self.log_message("INFO", f"Loaded config snapshot: {path}", "Config")
            if hasattr(self, '_load_tuning_parameters'):
                self._load_tuning_parameters()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to load snapshot:\n{e}")

    def _export_config_snapshot(self):
        """File > Export > Config Snapshot — save a copy of the active config.xml."""
        import shutil
        from fic_studio.config import XML_FILE_PATH
        path, _ = QFileDialog.getSaveFileName(
            self, "Export Config Snapshot", "config_snapshot.xml",
            "XML Files (*.xml);;All Files (*)")
        if not path:
            return
        try:
            shutil.copy2(XML_FILE_PATH, path)
            self.log_message("INFO", f"Exported config snapshot: {path}", "Config")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to export snapshot:\n{e}")

    def _export_params_yaml(self):
        """File > Export > FIC Parameters as YAML — dump current params for the active robot."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Export FIC Parameters as YAML", "fic_params.yaml",
            "YAML Files (*.yaml *.yml);;All Files (*)")
        if not path:
            return
        try:
            import yaml
            from fic_studio.xml_manager import XMLManager
            xm = XMLManager()
            robot_name = self.robot_dropdown.currentText()
            num_joints = self.robots.get(robot_name, 7)
            data = {'robot': robot_name, 'modes': {}}
            for mode in xm.get_available_modes(robot_name):
                mode_data = {'joints': {}}
                for j in range(num_joints):
                    jp = xm.load_joint_params(robot_name, j, mode)
                    mode_data['joints'][j] = {
                        k: v for k, v in jp.__dict__.items() if v is not None
                    }
                data['modes'][mode] = mode_data
            with open(path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.log_message("INFO", f"Exported parameters to {path}", "Config")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to export YAML:\n{e}")

    def _populate_recent_experiments(self):
        """File > Recent Experiments — scan experiments directory and build submenu."""
        import os
        import yaml as _yaml
        self._recent_menu.clear()
        from fic_studio.config import EXPERIMENTS_ROOT
        exp_dir = EXPERIMENTS_ROOT
        if not os.path.isdir(exp_dir):
            no_exp = QAction("(no experiments found)", self)
            no_exp.setEnabled(False)
            self._recent_menu.addAction(no_exp)
            return
        # Find experiment.yaml files, sort by mtime descending
        experiments = []
        for entry in os.listdir(exp_dir):
            meta_path = os.path.join(exp_dir, entry, 'experiment.yaml')
            if os.path.isfile(meta_path):
                experiments.append((os.path.getmtime(meta_path), os.path.join(exp_dir, entry), meta_path))
        experiments.sort(reverse=True)
        if not experiments:
            no_exp = QAction("(no experiments found)", self)
            no_exp.setEnabled(False)
            self._recent_menu.addAction(no_exp)
            return
        for _, directory, fp in experiments[:10]:
            try:
                with open(fp) as f:
                    meta = _yaml.safe_load(f)
                label = meta.get('name', os.path.basename(directory))
            except Exception:
                label = os.path.basename(directory)
            action = QAction(label, self)
            action.triggered.connect(
                lambda checked, d=directory: self._open_experiment_dir(d))
            self._recent_menu.addAction(action)

    def _open_experiment_dir(self, directory: str):
        """Open an experiment directory in the system file manager."""
        from fic_studio.platform_utils import open_path
        try:
            open_path(directory)
        except Exception as e:
            self.log_message("ERROR", f"Could not open directory: {e}", "System")

    def _open_experiments_folder(self):
        """Tools > Open Experiments Folder."""
        import os
        from fic_studio.config import EXPERIMENTS_ROOT
        os.makedirs(EXPERIMENTS_ROOT, exist_ok=True)
        self._open_experiment_dir(EXPERIMENTS_ROOT)

    def _set_mode_from_menu(self, mode_id: int):
        """Robot > Control Mode > (1-8) — publish mode and update the dropdown."""
        self._send_hpmp_for_mode(mode_id)
        self.node.publish_mode(mode_id)
        # Sync the Robot Control panel dropdown (0-indexed)
        if hasattr(self, '_rc_mode_dropdown'):
            self._rc_mode_dropdown.setCurrentIndex(mode_id - 1)
        self.log_message("INFO", f"Control mode set to {mode_id}", "Control")
        self._sync_mode_menu_check()

    def _show_robot_info(self):
        """Robot > Robot Info — show a summary dialog for the selected robot."""
        robot_name = self.robot_dropdown.currentText()
        cfg = self.robot_configs.get(robot_name, {})
        num_joints = self.robots.get(robot_name, 7)
        info_lines = [
            f"<b>Robot:</b> {robot_name}",
            f"<b>Joints:</b> {num_joints}",
            f"<b>Model file:</b> {cfg.get('model_xml', 'N/A')}",
            f"<b>URDF:</b> {cfg.get('urdf_path', 'N/A')}",
            f"<b>Driver:</b> {cfg.get('driver_command', 'N/A')}",
            f"<b>Home (deg):</b> {cfg.get('q_home_deg', 'N/A')}",
        ]
        QMessageBox.information(self, f"Robot Info — {robot_name}",
                                "<br>".join(info_lines))

    def _stop_all_processes(self):
        """Simulation > Stop All Processes — confirm then stop everything."""
        reply = QMessageBox.question(
            self, "Stop All Processes",
            "Stop all running processes (SIM, FIC, Controller, Driver, Bags, Plotter)?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        for attr, stop_method in [
            ('simulation_running', '_stop_simulation'),
            ('fic_running', '_stop_fic_simulation'),
            ('controller_running', '_stop_controller'),
            ('driver_running', '_stop_driver'),
            ('bags_running', '_stop_bags'),
            ('plotter_running', '_stop_plotter'),
        ]:
            if getattr(self, attr, False) and hasattr(self, stop_method):
                getattr(self, stop_method)()
        self.log_message("INFO", "All processes stopped", "System")

    def _show_shortcuts_reference(self):
        """Help > Keyboard Shortcuts — quick-reference dialog.

        Mirrors the ACTUAL bindings: the menu-bar accelerators defined in
        _setup_menubar plus the QShortcuts registered in
        SettingsMixin._setup_keyboard_shortcuts. Keep this in sync if either
        of those changes.
        """
        shortcuts = [
            ("Space", "Emergency Stop (E-STOP)"),
            ("Escape", "Stop Sequence"),
            ("Ctrl+R", "Play Sequence"),
            ("", ""),
            ("Ctrl+0", "Dashboard"),
            ("Ctrl+1", "Robot Control"),
            ("Ctrl+3", "Tuning Panel"),
            ("Ctrl+4", "Sequences"),
            ("Ctrl+5", "Settings"),
            ("", ""),
            ("F5", "Toggle Simulation"),
            ("F6", "Toggle FIC Controller"),
            ("F7", "Launch Sim + FIC"),
            ("F8", "Toggle Hardware Controller"),
            ("F9", "Toggle Robot Driver"),
            ("Ctrl+Shift+Q", "Stop All Processes"),
            ("", ""),
            ("Ctrl+H", "Move to Home"),
            ("Ctrl+B", "Compile Workspace"),
            ("Ctrl+S", "Save (context-aware)"),
            ("Ctrl+,", "Settings"),
            ("Ctrl+/", "This shortcuts reference"),
            ("F1", "Help"),
            ("", ""),
            ("Ctrl+L", "Toggle Log Panel"),
            ("Ctrl+\\", "Toggle Sidebar"),
            ("Ctrl+T", "Toggle Toolbar"),
            ("Ctrl+M", "Compact Mode"),
            ("Ctrl+Shift+M", "Minimal Mode"),
            ("F11", "Toggle Fullscreen"),
        ]
        rows = []
        for key, desc in shortcuts:
            if not key:
                rows.append("<tr><td colspan='2'>&nbsp;</td></tr>")
            else:
                rows.append(f"<tr><td><code>{key}</code></td><td>{desc}</td></tr>")
        html = (
            "<h3>Keyboard Shortcuts</h3>"
            "<table cellpadding='4'>" + "".join(rows) + "</table>"
        )
        QMessageBox.information(self, "Keyboard Shortcuts", html)

    def log_message(self, level: str, message: str, source: str = ""):
        if hasattr(self, 'log_viewer') and self.log_viewer:
            self.log_viewer.log(level, message, source)

    # ══════════════════════════════════════════════════════════════════════════
    #  EXTENSION SYSTEM
    # ══════════════════════════════════════════════════════════════════════════

    def _init_extensions(self):
        ext_root = FIC_INTERFACE_DIR / "Extensions"
        self._extensions_manifests = discover_extensions(ext_root)
        self._extension_loader = ExtensionLoader(ext_root)
        enabled_ids = list(self.settings.get("extensions", {}).get("enabled", []))
        self._extension_instances = self._extension_loader.load_enabled(
            self._extensions_manifests,
            enabled_ids,
            host_factory=lambda eid: ExtensionHost(self, eid),
        )
        # Filled in during _setup_central_widget.
        self._extension_panel_indices = {}
        self._extension_sidebar_buttons = {}

    def _add_extension_sidebar_entries(self, sidebar_layout):
        # Group loaded extensions by SIDEBAR_SECTION (default "Extensions").
        # Extensions targeting an existing built-in section merge into it via
        # the dividers dict; extensions with a brand-new section name get a
        # new section block inserted just before the System anchor.
        if not self._extension_instances:
            return []
        sections: dict[str, list] = {}
        for ext_id, inst in self._extension_instances.items():
            section = (
                getattr(inst, "SIDEBAR_SECTION", None)
                or "Extensions"
            )
            sections.setdefault(section, []).append((ext_id, inst))

        buttons: list = []
        for section_name, items in sections.items():
            divider = self._sidebar_section_dividers.get(section_name)
            if divider is not None:
                # Merge into existing section: insert each button just before
                # its hr. indexOf is recomputed each iteration so subsequent
                # buttons land in order after the previous one.
                for ext_id, inst in items:
                    btn = self._make_extension_btn(ext_id, inst)
                    idx = sidebar_layout.indexOf(divider)
                    sidebar_layout.insertWidget(idx, btn)
                    self._extension_sidebar_buttons[ext_id] = btn
                    buttons.append(btn)
            else:
                # New section: insert label + buttons + hr just before the
                # System anchor (so the new section lands between Experiments
                # and System in the sidebar).
                anchor = self._sidebar_new_section_anchor
                anchor_idx = sidebar_layout.indexOf(anchor)

                label = self._section_label(section_name)
                sidebar_layout.insertWidget(anchor_idx, label)
                anchor_idx += 1
                for ext_id, inst in items:
                    btn = self._make_extension_btn(ext_id, inst)
                    sidebar_layout.insertWidget(anchor_idx, btn)
                    anchor_idx += 1
                    self._extension_sidebar_buttons[ext_id] = btn
                    buttons.append(btn)
                new_hr = self._hr()
                sidebar_layout.insertWidget(anchor_idx, new_hr)
                self._sidebar_section_dividers[section_name] = new_hr
        return buttons

    def _hide_empty_sidebar_sections(self, sidebar_layout):
        # For each named section, count QPushButton widgets between its
        # label and its trailing hr. If zero, hide both so the sidebar
        # doesn't show a header with nothing under it.
        for name, parts in self._sidebar_sections.items():
            label = parts.get("label")
            divider = parts.get("divider")
            if label is None or divider is None:
                continue
            label_idx = sidebar_layout.indexOf(label)
            divider_idx = sidebar_layout.indexOf(divider)
            if label_idx < 0 or divider_idx < 0:
                continue
            has_button = False
            for i in range(label_idx + 1, divider_idx):
                item = sidebar_layout.itemAt(i)
                if item is None:
                    continue
                w = item.widget()
                if isinstance(w, QPushButton):
                    has_button = True
                    break
            if not has_button:
                label.setVisible(False)
                divider.setVisible(False)

    def _make_extension_btn(self, ext_id, inst):
        icon = getattr(inst, "SIDEBAR_ICON", "") or ""
        label_text = getattr(inst, "NAME", ext_id)
        btn_label = f"  {icon}  {label_text}".rstrip() if icon else f"  {label_text}"
        return self._mk_sidebar_btn(
            btn_label,
            lambda checked=False, eid=ext_id: self._show_extension_panel(eid),
        )

    def _add_extension_stack_pages(self):
        for ext_id, inst in self._extension_instances.items():
            try:
                page = inst.build_page()
            except Exception as e:
                print(f"[extensions] {ext_id}.build_page() failed: {e}")
                continue
            idx = self.pages.register(ext_id, page)
            self._extension_panel_indices[ext_id] = idx

    def _show_extension_panel(self, ext_id: str):
        if not self.pages.show(ext_id):
            return
        btn = self._extension_sidebar_buttons.get(ext_id)
        if btn is not None:
            self._set_active_sidebar_btn(btn)
        inst = self._extension_instances.get(ext_id)
        if inst is not None:
            try:
                inst.on_activate()
            except Exception as e:
                print(f"[extensions] {ext_id}.on_activate() failed: {e}")

    def _build_extensions_settings_tab(self):
        return ExtensionsSettingsPanel(
            manifests=getattr(self, "_extensions_manifests", []),
            enabled_ids=list(self.settings.get("extensions", {}).get("enabled", [])),
            on_apply=self._save_extensions_enabled,
        )

    def _save_extensions_enabled(self, enabled_ids):
        self.settings.setdefault("extensions", {})["enabled"] = list(enabled_ids)
        self._save_settings()

    def get_extension(self, ext_id: str):
        # Returns the loaded Extension instance or None if disabled / not found.
        # Used by core code that needs to query an extension's runtime state
        # (e.g. teleop status LED in the status bar).
        return getattr(self, "_extension_instances", {}).get(ext_id)

    def _notify_extensions_robots_changed(self):
        for inst in getattr(self, "_extension_instances", {}).values():
            try:
                inst.host.notify_robots_changed()
            except Exception:
                pass

    def _estop_extensions(self):
        # Fan E-stop out to every loaded extension's on_emergency_stop hook.
        for ext_id, inst in getattr(self, "_extension_instances", {}).items():
            try:
                inst.on_emergency_stop()
            except Exception as e:
                print(f"[extensions] {ext_id}.on_emergency_stop failed: {e}")

    def closeEvent(self, event):
        # Stop taking web commands before tearing down what they command.
        if hasattr(self, "_teardown_web_remote"):
            self._teardown_web_remote()
        if hasattr(self, "_teardown_web_ui"):
            self._teardown_web_ui()
        if hasattr(self, "_teardown_fleet"):
            self._teardown_fleet()
        # SIGTERM-then-SIGKILL the entire process group of every tracked
        # subprocess. Replaces the old per-attr proc.terminate() loop, which
        # only signalled the bash wrapper and left ros2 launch grandchildren
        # orphaned on the robot.
        self._kill_all_processes(reason="window closed")
        if hasattr(self, "_extension_loader"):
            self._extension_loader.teardown_all()
        if hasattr(self, "_teardown_gamepad"):
            self._teardown_gamepad()
        if hasattr(self, "_teardown_keyboard"):
            self._teardown_keyboard()
        srv = getattr(self, "_help_docs_server", None)
        if srv:
            srv.kill()
        event.accept()