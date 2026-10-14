# first_launch_wizard.py — First-launch setup wizard for FIC Studio
# 4-step QWizard: Welcome/Robot, Workspace, Test Launch, Finish

import os
import xml.etree.ElementTree as ET

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QWizard, QWizardPage, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QLineEdit, QRadioButton, QButtonGroup,
    QGroupBox, QGridLayout, QCheckBox, QFileDialog,
)

from fic_studio import theme


class FirstLaunchWizard(QWizard):
    """First-launch setup wizard for FIC Studio."""

    PAGE_WELCOME = 0
    PAGE_WORKSPACE = 1
    PAGE_TEST = 2
    PAGE_FINISH = 3

    def __init__(self, parent=None, robot_configs=None, settings=None):
        super().__init__(parent)
        self.robot_configs = robot_configs or {}
        self.settings = settings or {}
        self._test_passed = False

        self.setWindowTitle("FIC Studio \u2014 First Launch Setup")
        self.setMinimumSize(650, 500)
        self.setWizardStyle(QWizard.ModernStyle)

        theme.bind(self, lambda t: """
            QWizard {{ background-color: {bg}; color: {fg}; }}
            QWizardPage {{ background-color: {bg}; color: {fg}; }}
            QLabel {{ color: {fg}; }}
            QRadioButton {{
                color: {fg}; spacing: 8px; padding: 6px 0; font-size: 13px;
            }}
            QRadioButton::indicator {{ width: 16px; height: 16px; }}
            QLineEdit {{
                background-color: {bg_ov}; border: 1px solid {brd};
                border-radius: 4px; color: {fg}; padding: 6px 10px; font-size: 12px;
            }}
            QPushButton {{
                background-color: {bg_r}; border: 1px solid {brd};
                border-radius: 4px; color: {fg}; padding: 6px 16px; min-height: 28px;
            }}
            QPushButton:hover {{ background-color: {bg_hv}; }}
            QCheckBox {{ color: {sec}; spacing: 8px; padding: 4px 0; }}
            QGroupBox {{
                color: {acc}; border: 1px solid {brd_s}; border-radius: 4px;
                margin-top: 12px; padding-top: 16px; font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 12px; padding: 0 6px;
            }}
        """.format(
            bg=t['bg_base'], fg=t['text_primary'],
            bg_ov=t['bg_overlay'], bg_r=t['bg_raised'],
            bg_hv=t['bg_hover'], brd=t['border_medium'],
            brd_s=t['border_subtle'], sec=t['text_secondary'],
            acc=t['accent_blue'],
        ))

        self.addPage(self._create_welcome_page())
        self.addPage(self._create_workspace_page())
        self.addPage(self._create_test_page())
        self.addPage(self._create_finish_page())

    # ── Page 1: Welcome + Robot Selection ────────────────────────────────────

    def _create_welcome_page(self) -> QWizardPage:
        page = QWizardPage()
        page.setTitle("Welcome to FIC Studio")
        page.setSubTitle(
            "Fractal Impedance Control for Multi-Robot Systems\n"
            "Let's get your workspace set up.")

        layout = QVBoxLayout(page)
        layout.setSpacing(12)

        layout.addWidget(QLabel("Select your primary robot:"))

        self._robot_group = QButtonGroup()
        robot_list = list(self.robot_configs.keys())
        for i, robot_id in enumerate(robot_list):
            config = self.robot_configs[robot_id]
            num_joints = config.get('num_joints', '?')
            display = config.get('display_name', robot_id)
            radio = QRadioButton(f"{robot_id}  \u2014  {display} ({num_joints} DOF)")
            if i == 0:
                radio.setChecked(True)
            self._robot_group.addButton(radio, i)
            layout.addWidget(radio)

        layout.addSpacing(12)
        note = QLabel("You can add custom robots later from the Robot Config panel.")
        theme.bind(note, lambda t: "color: %s; font-size: 11px;" % t['text_disabled'])
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch()
        return page

    # ── Page 2: Workspace Verification ───────────────────────────────────────

    def _create_workspace_page(self) -> QWizardPage:
        page = QWizardPage()
        page.setTitle("Verify Workspace")
        page.setSubTitle(
            "FIC Studio needs your ROS2 workspace path to launch "
            "simulation and controller nodes.")

        layout = QVBoxLayout(page)
        layout.setSpacing(10)

        # Path input
        path_row = QHBoxLayout()
        path_row.addWidget(QLabel("Workspace:"))
        self._ws_path_edit = QLineEdit()
        self._ws_path_edit.setText(self._detect_workspace())
        path_row.addWidget(self._ws_path_edit, 1)
        browse_btn = QPushButton("Browse")
        browse_btn.clicked.connect(self._browse_workspace)
        path_row.addWidget(browse_btn)
        layout.addLayout(path_row)

        # Verify button
        verify_btn = QPushButton("Verify Workspace")
        theme.bind(verify_btn, lambda t: (
            "background-color: %s; border: 1px solid %s; color: %s;"
            % (t['bg_base'], t['border_medium'], t['accent_blue'])
        ))
        verify_btn.clicked.connect(self._run_verification)
        layout.addWidget(verify_btn)

        # Checks grid
        self._check_grid = QGridLayout()
        self._check_labels = {}
        check_names = [
            "ROS2 workspace", "fic_library", "config.xml",
            "robots_config.yaml", "colcon build", "setup.bash"
        ]
        for i, name in enumerate(check_names):
            icon = QLabel("\u25cb")
            icon.setFixedWidth(20)
            theme.bind(icon, lambda t: "color: %s; font-size: 14px;" % t['text_disabled'])
            self._check_grid.addWidget(icon, i, 0)

            label = QLabel(name)
            theme.bind(label, lambda t: "color: %s;" % t['text_secondary'])
            self._check_grid.addWidget(label, i, 1)

            status = QLabel("Not checked")
            theme.bind(status, lambda t: "color: %s; font-size: 11px;" % t['text_disabled'])
            self._check_grid.addWidget(status, i, 2)

            self._check_labels[name] = (icon, status)

        layout.addLayout(self._check_grid)
        layout.addStretch()
        return page

    def _detect_workspace(self) -> str:
        from fic_studio.config import WS_PATH as _FIC_WS
        candidates = [
            _FIC_WS,
            os.path.expanduser("~/fic_ws/fic_studio/ROS2_Nodes"),
        ]
        prefix = os.environ.get('COLCON_PREFIX_PATH', '')
        if prefix:
            candidates.insert(0, os.path.dirname(prefix))

        for path in candidates:
            if os.path.isdir(os.path.join(path, "src")):
                return path
        return candidates[0]

    def _browse_workspace(self):
        path = QFileDialog.getExistingDirectory(
            self, "Select ROS2 Workspace", self._ws_path_edit.text())
        if path:
            self._ws_path_edit.setText(path)

    def _run_verification(self):
        ws_path = self._ws_path_edit.text()
        fic_lib = os.path.abspath(os.path.join(ws_path, "..", "fic_library"))

        checks = {
            "ROS2 workspace": self._check_exists(
                os.path.join(ws_path, "src"), "src/ directory"),
            "fic_library": self._check_exists(fic_lib, "library directory"),
            "config.xml": self._check_config_xml(fic_lib),
            "robots_config.yaml": self._check_exists(
                os.path.join(fic_lib, "config", "robots_config.yaml"), "robot definitions"),
            "colcon build": self._check_exists(
                os.path.join(ws_path, "install"), "install/ directory"),
            "setup.bash": self._check_exists(
                os.path.join(ws_path, "install", "setup.bash"), "setup script"),
        }

        for name, (passed, detail) in checks.items():
            icon, status = self._check_labels[name]
            if passed:
                icon.setText("\u2713")
                theme.bind(
                    icon,
                    lambda t: "color: %s; font-size: 14px;" % t['accent_green'],
                )
                status.setText(detail)
                theme.bind(
                    status,
                    lambda t: "color: %s; font-size: 11px;" % t['accent_green'],
                )
            else:
                icon.setText("\u2717")
                theme.bind(
                    icon,
                    lambda t: "color: %s; font-size: 14px;" % t['accent_red'],
                )
                status.setText(detail)
                theme.bind(
                    status,
                    lambda t: "color: %s; font-size: 11px;" % t['accent_red'],
                )

    @staticmethod
    def _check_exists(path, label):
        if os.path.exists(path):
            return (True, f"{label} found")
        return (False, f"{label} not found")

    @staticmethod
    def _check_config_xml(fic_lib):
        path = os.path.join(fic_lib, "config", "config.xml")
        if os.path.exists(path):
            try:
                tree = ET.parse(path)
                n = len(tree.getroot().findall('.//robot'))
                return (True, f"{n} robots configured")
            except Exception:
                return (False, "exists but failed to parse")
        return (False, "not found")

    # ── Page 3: Test Launch ──────────────────────────────────────────────────

    def _create_test_page(self) -> QWizardPage:
        page = QWizardPage()
        page.setTitle("Test Your Setup")
        page.setSubTitle(
            "Launch a quick simulation to verify everything works. "
            "This is optional \u2014 you can skip it.")

        layout = QVBoxLayout(page)
        layout.setSpacing(12)

        self._test_robot_label = QLabel("Robot: --")
        theme.bind(
            self._test_robot_label,
            lambda t: (
                "font-size: 13px; font-weight: 600; color: %s;" % t['text_primary']
            ),
        )
        layout.addWidget(self._test_robot_label)

        self._test_launch_btn = QPushButton("LAUNCH TEST")
        theme.bind(
            self._test_launch_btn,
            lambda t: (
                "background-color: %s; border: 1px solid %s; "
                "color: %s; font-size: 14px; font-weight: 600; "
                "padding: 10px; min-height: 36px;"
                % (t['bg_base'], t['border_medium'], t['accent_blue'])
            ),
        )
        self._test_launch_btn.clicked.connect(self._on_launch_test)
        layout.addWidget(self._test_launch_btn)

        status_group = QGroupBox("Status")
        status_layout = QGridLayout(status_group)

        self._test_status = {}
        for i, (key, label) in enumerate([
            ("sim", "MuJoCo Simulation"),
            ("fic", "FIC Controller"),
            ("topic", "/trajectory_data topic"),
        ]):
            led = QLabel("\u25cb")
            led.setFixedWidth(20)
            theme.bind(led, lambda t: "color: %s; font-size: 14px;" % t['text_disabled'])
            status_layout.addWidget(led, i, 0)
            status_layout.addWidget(QLabel(label), i, 1)
            status_lbl = QLabel("Waiting...")
            theme.bind(
                status_lbl,
                lambda t: "color: %s; font-size: 11px;" % t['text_disabled'],
            )
            status_layout.addWidget(status_lbl, i, 2)
            self._test_status[key] = (led, status_lbl)

        layout.addWidget(status_group)

        self._skip_test = QCheckBox("Skip this step (I'll test manually later)")
        layout.addWidget(self._skip_test)

        layout.addStretch()
        return page

    def _on_launch_test(self):
        self._test_launch_btn.setEnabled(False)
        self._test_launch_btn.setText("Launching...")
        parent = self.parent()
        if parent and hasattr(parent, '_wizard_run_test'):
            parent._wizard_run_test(status_callback=self._update_test_status)
        else:
            self._update_test_status("sim", "fail", "No parent window available")

    def _update_test_status(self, key, state, detail):
        led, lbl = self._test_status[key]
        t = theme.current()
        _icon_token = {
            "ok":       ("\u2713", 'accent_green'),
            "fail":     ("\u2717", 'accent_red'),
            "progress": ("\u25cf", 'accent_orange'),
        }
        icon_text, tok = _icon_token.get(state, ("\u25cb", 'text_disabled'))
        color = t[tok]
        led.setText(icon_text)
        theme.bind(led, lambda t, _tok=tok: "color: %s; font-size: 14px;" % t[_tok])
        lbl.setText(detail)
        theme.bind(lbl, lambda t, _tok=tok: "color: %s; font-size: 11px;" % t[_tok])
        if state == "ok" and key == "topic":
            self._test_passed = True

    # ── Page 4: Finish ───────────────────────────────────────────────────────

    def _create_finish_page(self) -> QWizardPage:
        page = QWizardPage()
        page.setTitle("Setup Complete!")
        page.setSubTitle("FIC Studio is ready to use.")

        layout = QVBoxLayout(page)
        layout.setSpacing(12)

        self._finish_summary = QLabel("")
        theme.bind(
            self._finish_summary,
            lambda t: "font-size: 12px; color: %s;" % t['text_secondary'],
        )
        self._finish_summary.setWordWrap(True)
        layout.addWidget(self._finish_summary)

        layout.addSpacing(16)
        layout.addWidget(QLabel("What would you like to do first?"))

        self._action_group = QButtonGroup()
        actions = [
            "Open Dashboard (recommended)",
            "Launch Simulation + FIC and open Control Panel",
            "Open Tuning Panel",
            "Open Settings (configure further)",
        ]
        for i, label in enumerate(actions):
            radio = QRadioButton(label)
            if i == 0:
                radio.setChecked(True)
            self._action_group.addButton(radio, i)
            layout.addWidget(radio)

        layout.addStretch()
        return page

    # ── Page transitions ─────────────────────────────────────────────────────

    def initializePage(self, page_id):
        robot = self._selected_robot()
        if page_id == self.PAGE_TEST:
            config = self.robot_configs.get(robot, {})
            display = config.get('display_name', robot)
            self._test_robot_label.setText(f"Robot: {display} ({robot})")
        elif page_id == self.PAGE_FINISH:
            ws = self._ws_path_edit.text()
            test = "Passed" if self._test_passed else "Skipped"
            self._finish_summary.setText(
                f"Robot: {robot}\n"
                f"Workspace: {ws}\n"
                f"Test: {test}")

    def cleanupPage(self, page_id):
        if page_id == self.PAGE_TEST:
            parent = self.parent()
            if parent and hasattr(parent, '_wizard_cleanup_test'):
                parent._wizard_cleanup_test()

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _selected_robot(self) -> str:
        robot_list = list(self.robot_configs.keys())
        idx = self._robot_group.checkedId()
        if 0 <= idx < len(robot_list):
            return robot_list[idx]
        return robot_list[0] if robot_list else "Franka_FR3"

    def get_results(self) -> dict:
        action_map = {0: "dashboard", 1: "sim", 2: "tuning", 3: "settings"}
        action_idx = self._action_group.checkedId()
        return {
            'robot': self._selected_robot(),
            'workspace_path': self._ws_path_edit.text(),
            'test_passed': self._test_passed,
            'first_action': action_map.get(action_idx, "dashboard"),
        }
