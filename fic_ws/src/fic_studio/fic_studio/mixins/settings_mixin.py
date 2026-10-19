# mixins/settings_mixin.py — Settings page, theme toggle, log viewer, keyboard shortcuts

import copy
import logging
import os

from fic_studio import paths as _paths
import yaml
from datetime import datetime
from functools import partial

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QKeySequence, QTextCursor
from PyQt5.QtWidgets import (
    QTextBrowser, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit,
    QComboBox, QCheckBox, QGroupBox, QGridLayout, QScrollArea, QFrame,
    QSpinBox, QDoubleSpinBox, QTextEdit, QSplitter, QShortcut,
    QSizePolicy, QMessageBox, QApplication, QFileDialog, QListView,
)


# ═══════════════════════════════════════════════════════════════════════════════
#  THEMES
# ═══════════════════════════════════════════════════════════════════════════════
from fic_studio import theme as _theme
from fic_studio.theme import apply_theme as _engine_apply_theme
from fic_studio.theme.tokens import (
    PALETTE_LABELS, PALETTES,
    compose_theme_name, normalize_theme_name, split_theme_name,
)
from fic_studio.config import WS_PATH as _FIC_WS
from fic_studio.platform_utils import (
    IS_MACOS, colcon_build_command, default_mono_font,
    mono_font_choices,
)
# ═══════════════════════════════════════════════════════════════════════════════
#  DEFAULT SETTINGS
# ═══════════════════════════════════════════════════════════════════════════════

def _newton_available() -> bool:
    """Return True if the Newton physics backend is importable on this machine."""
    try:
        import importlib.util
        return importlib.util.find_spec('newton') is not None
    except Exception:
        return False


#: The two simulation backends: the CPU MuJoCo simulator (package
#: `mujoco_native`) and the optional GPU one (package `mujoco_newton`).
SIM_BACKEND_CPU = 'mujoco_native'
SIM_BACKEND_GPU = 'newton'


def normalize_sim_backend(value) -> str:
    """Map a saved `simulation.backend` value onto a backend that exists.

    'newton' stays 'newton'. Everything else — including 'mujoco', the C++
    simulator retired in v4.5.0 that older ~/.robot_selector_gui_settings.yaml
    files still name, and a missing or malformed value — is the CPU simulator.
    """
    return SIM_BACKEND_GPU if value == SIM_BACKEND_GPU else SIM_BACKEND_CPU


def _default_sim_backend() -> str:
    """Default backend: 'newton' if Newton was installed at setup (Linux with
    an NVIDIA GPU), else the CPU simulator."""
    if IS_MACOS:
        return SIM_BACKEND_CPU
    hint = os.path.expanduser('~/.fic_studio_install_hints.yaml')
    if os.path.exists(hint) and _newton_available():
        return SIM_BACKEND_GPU
    return SIM_BACKEND_CPU


DEFAULT_SETTINGS = {
    'general': {
        'default_robot': 'Franka_FR3',
        'station_id': 1,            # 1-4: a robot's robotN_* topics; 0: Master (the control seat)
        'wizard_completed': False,
        'auto_save': True,
        'auto_save_interval': 60,  # seconds
        'show_log_viewer': True,
        'log_viewer_height': 150,
    },
    # Where recordings go ('' = the default: see fic_studio.paths). Read at start-up.
    'paths': {
        'experiments': '',
    },
    'appearance': {
        'theme': 'warmth_dark',
        'font_size': 12,
        'font_family': default_mono_font(),
    },
    'network': {
        'ros_domain_id': 0,
        'partner_ip': '',
        'dds_mode': 'multicast',
    },
    'simulation': {
        'backend': _default_sim_backend(),  # 'mujoco_native' (CPU) or 'newton' (GPU)
    },
    'tolerances': {
        'joint_tolerance_deg': 3.0,
        'position_tolerance_mm': 5.0,
        'timeout_seconds': 30.0,
    },
    'shortcuts': {
        'estop': 'Space',
        'play_sequence': 'Ctrl+R',
        'stop_sequence': 'Escape',
        'save_settings': 'Ctrl+S',
        'toggle_log': 'Ctrl+L',
        'panel_dashboard': 'Ctrl+0',
        'panel_control': 'Ctrl+1',
        'panel_tuning': 'Ctrl+3',
        'panel_sequence': 'Ctrl+4',
        'panel_settings': 'Ctrl+5',
    },
    'extensions': {
        'enabled': [],
        'data': {},
    },
    'web_remote': {
        'enabled': True,            # serve the controls to browsers on the network
        'port': 8088,               # next free port is used if taken
    },
    'vr': {
        'relay_always_on': True,    # run the VR relay while the Studio is open, not only after VR > Start
        'allow_viewers': True,      # headsets may Watch the lab (every Studio's robots, live)
        'fleet_stations': True,     # a headset's robots are listed on Multi-Robot as VR stations
        'headset_ip': '',           # also announce to this address (when broadcasts do not reach the headset)
        'require_key': False,       # sign every relay <-> headset packet with the pairing key (fleet/vr_pairing.py)
    },
    'advanced': {
        'verbose_logging': False,
        'save_rosbag': False,
    },
}


# ═══════════════════════════════════════════════════════════════════════════════
#  LOG VIEWER WIDGET
# ═══════════════════════════════════════════════════════════════════════════════

class LogViewer(QWidget):
    """Collapsible log viewer widget."""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.max_lines = 500
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Header bar — always visible
        self.header = QWidget()
        self.header.setObjectName("LogViewerHeader")
        header_layout = QHBoxLayout(self.header)
        header_layout.setContentsMargins(8, 4, 8, 4)
        header_layout.setSpacing(8)

        self.collapse_btn = QPushButton("☰")
        self.collapse_btn.setObjectName("SidebarCollapse")
        self.collapse_btn.setCursor(Qt.PointingHandCursor)
        self.collapse_btn.setFixedSize(28, 28)
        header_layout.addWidget(self.collapse_btn)

        title_lbl = QLabel("LOGS")
        title_lbl.setObjectName("LogToggleBtn")
        header_layout.addWidget(title_lbl)

        header_layout.addStretch()

        # Filter dropdown
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["All", "INFO", "WARN", "ERROR"])
        self.filter_combo.setFixedWidth(90)
        self.filter_combo.currentTextChanged.connect(self._apply_filter)
        header_layout.addWidget(self.filter_combo)

        # Clear button
        clear_btn = QPushButton("Clear")
        clear_btn.setFixedWidth(90)
        clear_btn.clicked.connect(self.clear_logs)
        header_layout.addWidget(clear_btn)

        # Save button
        save_btn = QPushButton("Save")
        save_btn.setFixedWidth(90)
        save_btn.clicked.connect(self.save_logs)
        header_layout.addWidget(save_btn)

        layout.addWidget(self.header)

        # Log text area — this is what gets hidden/shown
        self.log_text = QTextEdit()
        self.log_text.setObjectName("LogViewer")
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(100)
        layout.addWidget(self.log_text)

        self._all_logs = []
    
    def add_log(self, level: str, message: str, source: str = ""):
        """Add a log entry."""
        timestamp = datetime.now().strftime("%H:%M:%S")

        # Color coding — sourced from active theme tokens
        t = _theme.current()
        _level_token = {
            'INFO': 'accent_blue',
            'WARN': 'accent_orange',
            'ERROR': 'accent_red',
            'DEBUG': 'text_secondary',
        }
        color = t.get(_level_token.get(level.upper(), 'text_secondary'), t['text_secondary'])

        # Format log entry
        source_str = f"[{source}] " if source else ""
        log_entry = {
            'timestamp': timestamp,
            'level': level.upper(),
            'source': source,
            'message': message,
            'formatted': (
                f'<span style="color:{t["text_disabled"]}">[{timestamp}]</span> '
                f'<span style="color:{color};font-weight: 600">{level.upper():5}</span> '
                f'<span style="color:{t["text_secondary"]}">{source_str}</span>'
                f'<span style="color:{t["text_primary"]}">{message}</span>'
            ),
        }
        
        self._log_seq = getattr(self, '_log_seq', 0) + 1
        log_entry['seq'] = self._log_seq
        self._all_logs.append(log_entry)
        
        # Trim if too many logs
        if len(self._all_logs) > self.max_lines:
            self._all_logs = self._all_logs[-self.max_lines:]
        
        # Apply filter and display
        self._apply_filter(self.filter_combo.currentText())
        
        # Auto-scroll to bottom
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_text.setTextCursor(cursor)
    
    def log(self, level: str, message: str, source: str = ""):
        """Alias for add_log (used by main window)."""
        self.add_log(level, message, source)
    
    def _apply_filter(self, filter_level: str):
        """Apply log level filter."""
        self.log_text.clear()
        
        for log in self._all_logs:
            if filter_level == "All" or log['level'] == filter_level:
                self.log_text.append(log['formatted'])
    
    def clear_logs(self):
        """Clear all logs."""
        self._all_logs = []
        self.log_text.clear()
    
    def save_logs(self):
        """Save logs to file."""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Logs", 
            f"robot_gui_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Text Files (*.txt)"
        )
        if filename:
            with open(filename, 'w') as f:
                for log in self._all_logs:
                    f.write(f"[{log['timestamp']}] {log['level']:5} {log['source']} {log['message']}\n")


# ═══════════════════════════════════════════════════════════════════════════════
#  SETTINGS MIXIN
# ═══════════════════════════════════════════════════════════════════════════════

class SettingsMixin:
    """Mixin for settings page, theme management, and keyboard shortcuts."""
    
    SETTINGS_FILE = _paths.settings_file()
    
    def _init_settings(self):
        """Initialize settings system."""
        self.settings = self._load_settings()
        self.current_theme = self.settings['appearance']['theme']
        self._setup_keyboard_shortcuts()
        self._setup_auto_save()
    
    def _load_settings(self) -> dict:
        """Load settings from file or use defaults."""
        if os.path.exists(self.SETTINGS_FILE):
            try:
                with open(self.SETTINGS_FILE, 'r') as f:
                    loaded = yaml.safe_load(f)
                    # Merge with defaults to handle new settings
                    merged = self._merge_settings(DEFAULT_SETTINGS, loaded)
                    merged['appearance']['theme'] = normalize_theme_name(
                        merged['appearance']['theme'])
                    # v4.5.0 retired the C++ 'mujoco' backend; a file saved
                    # before then maps onto the CPU simulator.
                    sim = merged.get('simulation')
                    if not isinstance(sim, dict):
                        sim = merged['simulation'] = {}
                    sim['backend'] = normalize_sim_backend(sim.get('backend'))
                    return merged
            except Exception as e:
                print(f"Could not load settings: {e}")
        return copy.deepcopy(DEFAULT_SETTINGS)
    
    def _merge_settings(self, defaults: dict, loaded: dict) -> dict:
        """Recursively merge loaded settings with defaults.

        Deep copy: sections the file does not mention must not be the default
        dicts themselves, or editing them would change DEFAULT_SETTINGS.
        """
        result = copy.deepcopy(defaults)
        for key, value in loaded.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._merge_settings(result[key], value)
            else:
                result[key] = value
        return result
    
    def _save_settings(self):
        """Save settings to file."""
        try:
            # Written whole, then moved over the old file: a crash or a full disk mid-write
            # leaves the previous settings, not half a file.
            tmp = self.SETTINGS_FILE + '.new'
            with open(tmp, 'w') as f:
                yaml.dump(self.settings, f, default_flow_style=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.SETTINGS_FILE)
            self.log_message("INFO", "Settings saved", "Settings")
        except Exception as e:
            self.log_message("ERROR", f"Could not save settings: {e}", "Settings")
    
    def _setup_auto_save(self):
        """Setup auto-save timer."""
        self.auto_save_timer = QTimer()
        self.auto_save_timer.timeout.connect(self._auto_save_settings)
        if self.settings['general']['auto_save']:
            interval = self.settings['general']['auto_save_interval'] * 1000
            self.auto_save_timer.start(interval)
    
    def _auto_save_settings(self):
        """Auto-save settings periodically."""
        if self.settings['general']['auto_save']:
            self._save_settings()
    
    def _setup_keyboard_shortcuts(self):
        """Setup keyboard shortcuts."""
        shortcuts = self.settings['shortcuts']
        
        # E-Stop
        self.shortcut_estop = QShortcut(QKeySequence(shortcuts['estop']), self)
        self.shortcut_estop.activated.connect(self._emergency_stop)
        
        # Play/Stop sequence
        self.shortcut_play = QShortcut(QKeySequence(shortcuts['play_sequence']), self)
        self.shortcut_play.activated.connect(self._shortcut_play_sequence)
        
        self.shortcut_stop = QShortcut(QKeySequence(shortcuts['stop_sequence']), self)
        self.shortcut_stop.activated.connect(self._shortcut_stop_sequence)

        # NOTE: save_settings (Ctrl+S), toggle_log (Ctrl+L) and panel_tuning
        # (Ctrl+3) are intentionally NOT registered here. The menu bar owns
        # those accelerators (File ▸ Save, View ▸ Show Log Panel, View ▸ Tuning
        # Panel) — binding them again as QShortcuts caused double-firing /
        # ambiguous-shortcut conflicts.

        # Panel navigation
        self.shortcut_p0 = QShortcut(QKeySequence(shortcuts['panel_dashboard']), self)
        self.shortcut_p0.activated.connect(self.show_dashboard)

        self.shortcut_p1 = QShortcut(QKeySequence(shortcuts['panel_control']), self)
        self.shortcut_p1.activated.connect(self.show_robot_control_panel)

        self.shortcut_p4 = QShortcut(QKeySequence(shortcuts['panel_sequence']), self)
        self.shortcut_p4.activated.connect(self.show_sequence_panel)
        
        self.shortcut_p5 = QShortcut(QKeySequence(shortcuts['panel_settings']), self)
        self.shortcut_p5.activated.connect(self.show_settings_panel)
    
    def _emergency_stop(self):
        """Emergency stop - nuke everything: processes, hardware, simulation, sequences, scripts."""
        self.log_message("ERROR", "E-STOP ACTIVATED! HALTING ALL SYSTEMS AND PROCESSES.", "Safety")
        
        # 1. Stop any running Sequence
        if hasattr(self, 'is_playing') and self.is_playing:
            self._stop_sequence()
            
        # 2. Stop any running Python Script
        if hasattr(self, 'script_running') and self.script_running:
            self._stop_script()
            
        # 3. Stop Teach Mode
        if hasattr(self, 'teach_mode_active') and self.teach_mode_active:
            self._stop_teach_mode()
            
        # 4. KILL ALL BACKGROUND PROCESSES (The true nuke)
        # Stop Hardware Controller
        if hasattr(self, 'controller_running') and self.controller_running:
            if hasattr(self, '_stop_controller'):
                self._stop_controller()
                
        # Stop Robot Driver
        if hasattr(self, 'driver_running') and self.driver_running:
            if hasattr(self, '_stop_driver'):
                self._stop_driver()
                
        # Stop FIC Controller
        if hasattr(self, 'fic_running') and self.fic_running:
            if hasattr(self, '_stop_fic_simulation'):
                self._stop_fic_simulation()
                
        # Stop Simulation
        if hasattr(self, 'simulation_running') and self.simulation_running:
            if hasattr(self, '_stop_simulation'):
                self._stop_simulation()
                
        # Stop Bag Recording
        if hasattr(self, 'bags_running') and self.bags_running:
            if hasattr(self, '_stop_bags'):
                self._stop_bags()

        # 5. Lock hardware in place via ROS topics (redundant safety if nodes survive briefly).
        # Broadcasts to every active robot so an E-STOP on a multi-robot
        # station locks them ALL, each at its own last-known joint pose.
        from std_msgs.msg import Float32MultiArray
        positions_by_robot = getattr(self, 'current_joint_positions_by_robot', {}) or {}
        for rid, pub in self.node.real_joint_publishers.items():
            pos = positions_by_robot.get(rid)
            if not pos and rid == self.node.station_robot_id:
                # Fall back to legacy single attribute for the first active robot.
                pos = getattr(self, 'current_joint_positions', None)
            if not pos:
                continue
            msg = Float32MultiArray()
            msg.data = [float(j) for j in pos]
            pub.publish(msg)

        # 6. Update global UI if available
        if hasattr(self, 'robot_status_label'):
            self.robot_status_label.setText("E-STOP ACTIVE - ALL PROCESSES KILLED")
            self.robot_status_label.setObjectName("EStopLabel")
            self.robot_status_label.style().unpolish(self.robot_status_label)
            self.robot_status_label.style().polish(self.robot_status_label)

    def _shortcut_play_sequence(self):
        """Play sequence via shortcut."""
        if hasattr(self, '_play_sequence'):
            self._play_sequence()
    
    def _shortcut_stop_sequence(self):
        """Stop sequence via shortcut."""
        if hasattr(self, '_stop_sequence'):
            self._stop_sequence()
    
    
    def log_message(self, level: str, message: str, source: str = ""):
        """Log a message to the log viewer."""
        if hasattr(self, 'log_viewer'):
            self.log_viewer.add_log(level, message, source)
    
    def apply_theme(self, theme_name: str):
        """Apply a theme by name (accepts legacy 'dark'/'light')."""
        normalized = _engine_apply_theme(self, theme_name)
        self.current_theme = normalized
        self.settings['appearance']['theme'] = normalized
    
    def _settings_card(self, title, restart=False):
        """Return (frame, body_layout) for a Warmth settings card.

        Creates a themed QFrame with an uppercase label row and an optional
        amber *Restart required* pill on the right.  All colours come from
        ``theme.bind`` — no inline hex literals.
        """
        frame = QFrame()
        frame.setObjectName('SettingsCard')
        _theme.bind(
            frame,
            lambda t: (
                'QFrame#SettingsCard {'
                ' background: %s;'
                ' border: 1px solid %s;'
                ' border-radius: 10px;'
                '}' % (t['bg_raised'], t['border_subtle'])
            ),
        )

        outer = QVBoxLayout(frame)
        outer.setContentsMargins(16, 12, 16, 16)
        outer.setSpacing(10)

        # ── header row ──────────────────────────────────────────────────
        header_row = QHBoxLayout()
        header_row.setSpacing(8)

        header_btn = QPushButton('▸  ' + title.upper())
        header_btn.setObjectName('SettingsCardTitle')
        header_btn.setCursor(Qt.PointingHandCursor)
        _theme.bind(
            header_btn,
            lambda t: (
                'QPushButton {'
                ' background: transparent; border: none;'
                ' text-align: left; padding: 0;'
                ' color: %s;'
                ' font-size: 11px;'
                ' font-weight: 700;'
                ' letter-spacing: 1px;'
                '}' % t['text_secondary']
            ),
        )
        header_row.addWidget(header_btn)
        header_row.addStretch()

        if restart:
            pill = QLabel('Restart required')
            pill.setObjectName('SettingsRestartPill')
            _theme.bind(
                pill,
                lambda t: (
                    'QLabel {'
                    ' background: transparent;'
                    ' color: %s;'
                    ' border: 1px solid %s;'
                    ' border-radius: 8px;'
                    ' font-size: 10px;'
                    ' font-weight: 500;'
                    ' padding: 2px 8px;'
                    '}' % (t['accent_orange'], t['accent_orange'])
                ),
            )
            header_row.addWidget(pill)

        outer.addLayout(header_row)

        # ── collapsible body returned to caller ──────────────────────────
        body_container = QWidget()
        body = QVBoxLayout(body_container)
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(8)
        outer.addWidget(body_container)
        # Collapsed on open. Nine expanded cards made Settings a long scroll
        # you had to hunt through; closed, the page is a list of headings you
        # can read at a glance and open the one you came for.
        body_container.setVisible(False)

        def _toggle(_=False, c=body_container, b=header_btn, ttl=title.upper()):
            # isHidden, not isVisible: a widget whose window is not shown yet
            # reports not-visible either way, so isVisible would make the
            # first click a no-op.
            vis = c.isHidden()
            c.setVisible(vis)
            b.setText(('▾  ' if vis else '▸  ') + ttl)
        header_btn.clicked.connect(_toggle)

        return frame, body

    def _build_settings_page(self) -> QScrollArea:
        """Build the settings page as a scrollable column of themed cards."""
        page = QWidget()
        root = QVBoxLayout(page)
        root.setContentsMargins(22, 20, 22, 20)
        root.setSpacing(12)
        root.setAlignment(Qt.AlignTop)

        # ── title row ────────────────────────────────────────────────────
        title_row = QHBoxLayout()
        title_row.addWidget(self._page_title('Settings'))
        title_row.addStretch()

        save_btn = self._btn('SAVE SETTINGS', 'BtnTeal')
        save_btn.clicked.connect(self._save_settings_from_ui)
        title_row.addWidget(save_btn)

        discard_btn = self._btn('DISCARD', 'BtnWarn')
        discard_btn.setToolTip('Discard unsaved changes and revert to last saved settings')
        discard_btn.clicked.connect(self._discard_settings)
        title_row.addWidget(discard_btn)

        help_btn = self._btn('? Help', 'BtnPrimary')
        help_btn.setToolTip('Open Help page')
        help_btn.clicked.connect(
            lambda: getattr(self, 'show_help_panel', lambda: None)()
        )
        title_row.addWidget(help_btn)

        root.addLayout(title_row)

        # ════════════════════════════════════════════════════════════════
        # CARD: Appearance
        # ════════════════════════════════════════════════════════════════
        card, body = self._settings_card('Appearance')

        # Theme — two independent choices rather than one button per theme.
        # Four themes as four buttons would be a wide row that grows every time
        # a palette is added; palette x brightness stays two short pairs and
        # says what the names mean.  ``BtnToggle`` is used because it is the
        # one button style the stylesheet gives a visible ``:checked`` state —
        # the old ``SegmentLeft``/``SegmentRight`` object names have no rules
        # anywhere in styles.py, so the selected theme looked exactly like the
        # unselected one.
        theme_row = QHBoxLayout()
        theme_row.addWidget(QLabel('Theme:'))

        _cur_theme = normalize_theme_name(self.settings['appearance']['theme'])
        _cur_palette, _cur_brightness = split_theme_name(_cur_theme)

        def _segment(text, checked, tooltip):
            """Build one button of a two-way choice."""
            button = QPushButton(text)
            button.setCheckable(True)
            button.setObjectName('BtnToggle')
            button.setCursor(Qt.PointingHandCursor)
            button.setChecked(checked)
            button.setToolTip(tooltip)
            return button

        # The palette half is built from PALETTES, not from a fixed pair, so
        # adding a palette does not mean rewiring this control again.
        _PALETTE_HINTS = {
            'warmth': 'Browns and ambers — the original FIC Studio look.',
            'cool': 'Slate and blues. Warning and stop colours stay orange and red.',
        }
        self._theme_palette_btns = {}
        for _p in PALETTES:
            _btn = _segment(PALETTE_LABELS.get(_p, _p.title()), _cur_palette == _p,
                            _PALETTE_HINTS.get(_p, ''))
            self._theme_palette_btns[_p] = _btn
        # Named handles the rest of the app and the tests already reach for.
        self._theme_warm_btn = self._theme_palette_btns['warmth']
        self._theme_cool_btn = self._theme_palette_btns['cool']

        self._theme_dark_btn = _segment(
            'Dark', _cur_brightness == 'dark', 'Light text on a dark surface.')
        self._theme_light_btn = _segment(
            'Light', _cur_brightness == 'light', 'Dark text on a pale surface.')

        def _pick(chosen, other):
            """Make ``chosen`` the selection of its pair, without applying it."""
            chosen.setChecked(True)
            other.setChecked(False)

        def _pick_palette(chosen):
            """Tick one palette and clear the others — any number of them."""
            for _btn in self._theme_palette_btns.values():
                _btn.setChecked(_btn is chosen)

        for _key, _btn in self._theme_palette_btns.items():
            _btn.clicked.connect(
                lambda _checked=False, b=_btn: _pick_palette(b))

        self._theme_dark_btn.clicked.connect(
            lambda: _pick(self._theme_dark_btn, self._theme_light_btn))
        self._theme_light_btn.clicked.connect(
            lambda: _pick(self._theme_light_btn, self._theme_dark_btn))

        for _btn in self._theme_palette_btns.values():
            theme_row.addWidget(_btn)
        theme_row.addSpacing(14)
        theme_row.addWidget(self._theme_dark_btn)
        theme_row.addWidget(self._theme_light_btn)
        theme_row.addStretch()
        body.addLayout(theme_row)

        # Font family / size
        font_row = QHBoxLayout()
        font_row.addWidget(QLabel('Font Family:'))
        self.settings_font_family = QComboBox()
        self.settings_font_family.addItems(mono_font_choices())
        self.settings_font_family.setCurrentText(self.settings['appearance']['font_family'])
        font_row.addWidget(self.settings_font_family)

        font_row.addWidget(QLabel('Font Size:'))
        self.settings_font_size = QSpinBox()
        self.settings_font_size.setRange(8, 18)
        self.settings_font_size.setValue(self.settings['appearance']['font_size'])
        self.settings_font_size.setSuffix(' px')
        font_row.addWidget(self.settings_font_size)
        font_row.addStretch()
        body.addLayout(font_row)

        root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Startup
        # ════════════════════════════════════════════════════════════════
        card, body = self._settings_card('Startup')

        # Default robot
        robot_row = QHBoxLayout()
        robot_row.addWidget(QLabel('Default Robot:'))
        self.settings_default_robot = QComboBox()
        robot_names = (
            list(self.robot_configs.keys())
            if hasattr(self, 'robot_configs')
            else ['Franka_FR3', 'KinovaGen3_7DOF']
        )
        self.settings_default_robot.addItems(robot_names)
        self.settings_default_robot.setCurrentText(self.settings['general']['default_robot'])
        robot_row.addWidget(self.settings_default_robot)
        robot_row.addStretch()
        body.addLayout(robot_row)

        # Auto-save
        autosave_row = QHBoxLayout()
        self.settings_auto_save = QCheckBox('Enable Auto-Save')
        self.settings_auto_save.setChecked(self.settings['general']['auto_save'])
        autosave_row.addWidget(self.settings_auto_save)
        autosave_row.addWidget(QLabel('Interval:'))
        self.settings_auto_save_interval = QSpinBox()
        self.settings_auto_save_interval.setRange(10, 600)
        self.settings_auto_save_interval.setValue(self.settings['general']['auto_save_interval'])
        self.settings_auto_save_interval.setSuffix(' sec')
        autosave_row.addWidget(self.settings_auto_save_interval)
        autosave_row.addStretch()
        body.addLayout(autosave_row)

        # Log viewer
        log_row = QHBoxLayout()
        self.settings_show_log = QCheckBox('Show Log Viewer')
        self.settings_show_log.setChecked(self.settings['general']['show_log_viewer'])
        log_row.addWidget(self.settings_show_log)
        log_row.addWidget(QLabel('Height:'))
        self.settings_log_height = QSpinBox()
        self.settings_log_height.setRange(50, 400)
        self.settings_log_height.setValue(self.settings['general']['log_viewer_height'])
        self.settings_log_height.setSuffix(' px')
        log_row.addWidget(self.settings_log_height)
        log_row.addStretch()
        body.addLayout(log_row)

        root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Network  (restart required)
        # ════════════════════════════════════════════════════════════════
        card, body = self._settings_card('Network', restart=True)

        net_grid = QGridLayout()
        net_grid.addWidget(QLabel('ROS Domain ID:'), 0, 0)
        self.settings_ros_domain = QSpinBox()
        self.settings_ros_domain.setRange(0, 232)
        self.settings_ros_domain.setValue(self.settings['network']['ros_domain_id'])
        self.settings_ros_domain.setToolTip('ROS_DOMAIN_ID (0-232) — restart to apply')
        net_grid.addWidget(self.settings_ros_domain, 0, 1)

        net_grid.addWidget(QLabel('Partner IP:'), 1, 0)
        self.settings_partner_ip = QLineEdit()
        self.settings_partner_ip.setPlaceholderText('e.g. 192.168.1.100')
        self.settings_partner_ip.setText(
            self.settings.get('network', {}).get('partner_ip', ''))
        net_grid.addWidget(self.settings_partner_ip, 1, 1)

        net_grid.addWidget(QLabel('DDS Discovery:'), 2, 0)
        self.settings_dds_mode = QComboBox()
        self.settings_dds_mode.addItems(['Multicast (Auto)', 'Unicast (Manual IP)'])
        if self.settings.get('network', {}).get('dds_mode', 'multicast') == 'unicast':
            self.settings_dds_mode.setCurrentIndex(1)
        net_grid.addWidget(self.settings_dds_mode, 2, 1)

        gen_dds_btn = QPushButton('Generate cyclonedds.xml')
        gen_dds_btn.setCursor(Qt.PointingHandCursor)
        gen_dds_btn.clicked.connect(self._generate_cyclone_config)
        net_grid.addWidget(gen_dds_btn, 3, 0, 1, 2)

        body.addLayout(net_grid)

        dds_hint = QLabel(
            'For cross-subnet or VPN setups enter the partner IP and click Generate.\n'
            'Then set CYCLONEDDS_URI=file://$HOME/cyclonedds.xml before launching.')
        dds_hint.setObjectName('HintLabel')
        dds_hint.setWordWrap(True)
        body.addWidget(dds_hint)

        root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Simulation
        # ════════════════════════════════════════════════════════════════
        card, body = self._settings_card('Simulation')

        sim_row = QHBoxLayout()
        sim_row.addWidget(QLabel('Backend:'))
        self.settings_sim_backend = QComboBox()
        self.settings_sim_backend.addItem('MuJoCo (CPU)', SIM_BACKEND_CPU)
        if IS_MACOS:
            # Newton needs CUDA, so on macOS the CPU simulator is the only one.
            self.settings_sim_backend.setToolTip(
                'MuJoCo on the CPU (package mujoco_native, official mujoco wheel).\n'
                'The only simulator on macOS / Apple Silicon.')
        else:
            newton_ok = _newton_available()
            if newton_ok:
                self.settings_sim_backend.addItem('Newton (NVIDIA Warp, GPU)', 'newton')
            else:
                self.settings_sim_backend.addItem(
                    'Newton (not installed — run install-deps.sh)', 'newton')
                self.settings_sim_backend.model().item(1).setEnabled(False)
            self.settings_sim_backend.setToolTip(
                'MuJoCo: CPU simulator (package mujoco_native, the default).\n'
                'Newton: GPU-accelerated via NVIDIA Warp (optional).')
        cur_backend = normalize_sim_backend(
            self.settings.get('simulation', {}).get('backend', _default_sim_backend()))
        idx = self.settings_sim_backend.findData(cur_backend)
        self.settings_sim_backend.setCurrentIndex(max(0, idx))
        sim_row.addWidget(self.settings_sim_backend)
        sim_row.addStretch()
        body.addLayout(sim_row)

        root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Playback Tolerances
        # ════════════════════════════════════════════════════════════════
        card, body = self._settings_card('Playback Tolerances')

        tol_grid = QGridLayout()

        tol_grid.addWidget(QLabel('Joint Tolerance:'), 0, 0)
        self.settings_joint_tol = QDoubleSpinBox()
        self.settings_joint_tol.setRange(0.5, 20.0)
        self.settings_joint_tol.setValue(self.settings['tolerances']['joint_tolerance_deg'])
        self.settings_joint_tol.setSuffix('°')
        self.settings_joint_tol.setToolTip('Joint position tolerance for sequence playback')
        tol_grid.addWidget(self.settings_joint_tol, 0, 1)

        tol_grid.addWidget(QLabel('Position Tolerance:'), 1, 0)
        self.settings_pos_tol = QDoubleSpinBox()
        self.settings_pos_tol.setRange(1.0, 50.0)
        self.settings_pos_tol.setValue(self.settings['tolerances']['position_tolerance_mm'])
        self.settings_pos_tol.setSuffix(' mm')
        self.settings_pos_tol.setToolTip('Cartesian position tolerance for sequence playback')
        tol_grid.addWidget(self.settings_pos_tol, 1, 1)

        tol_grid.addWidget(QLabel('Timeout:'), 2, 0)
        self.settings_timeout = QDoubleSpinBox()
        self.settings_timeout.setRange(5.0, 120.0)
        self.settings_timeout.setValue(self.settings['tolerances']['timeout_seconds'])
        self.settings_timeout.setSuffix(' sec')
        self.settings_timeout.setToolTip('Max wait time for each waypoint')
        tol_grid.addWidget(self.settings_timeout, 2, 1)

        body.addLayout(tol_grid)
        root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Shortcuts
        # ════════════════════════════════════════════════════════════════
        card, body = self._settings_card('Shortcuts')

        sc_grid = QGridLayout()
        sc_grid.setSpacing(6)

        self.shortcut_inputs = {}
        shortcuts = self.settings['shortcuts']
        sc_labels = {
            'estop': 'Emergency Stop',
            'play_sequence': 'Play Sequence',
            'stop_sequence': 'Stop Sequence',
            'save_settings': 'Save Settings',
            'toggle_log': 'Toggle Log Viewer',
            'panel_dashboard': 'Dashboard',
            'panel_control': 'Robot Control',
            'panel_tuning': 'Tuning Panel',
            'panel_sequence': 'Sequence Panel',
            'panel_settings': 'Settings Panel',
        }
        for i, (key, label) in enumerate(sc_labels.items()):
            sc_grid.addWidget(QLabel(f'{label}:'), i, 0)
            line_edit = QLineEdit()
            line_edit.setText(shortcuts.get(key, ''))
            line_edit.setPlaceholderText('Click and press key combo')
            sc_grid.addWidget(line_edit, i, 1)
            self.shortcut_inputs[key] = line_edit

        sc_inner = QWidget()
        sc_inner.setLayout(sc_grid)
        body.addWidget(sc_inner)

        sc_hint = QLabel('Tip: click in a field and press the desired key combo (e.g. Ctrl+R).')
        sc_hint.setObjectName('HintLabel')
        body.addWidget(sc_hint)

        root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Extensions  (restart required — injected panel)
        # ════════════════════════════════════════════════════════════════
        if hasattr(self, '_build_extensions_settings_tab'):
            card, body = self._settings_card('Extensions', restart=True)
            body.addWidget(self._build_extensions_settings_tab())
            root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Gamepad  (injected content)
        # ════════════════════════════════════════════════════════════════
        if hasattr(self, '_build_gamepad_settings_tab'):
            card, body = self._settings_card('Gamepad')
            body.addWidget(self._build_gamepad_settings_tab())
            root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Web Remote  (injected content)
        # ════════════════════════════════════════════════════════════════
        if hasattr(self, '_build_web_remote_settings_tab'):
            card, body = self._settings_card('Web Remote')
            body.addWidget(self._build_web_remote_settings_tab())
            root.addWidget(card)

        # ════════════════════════════════════════════════════════════════
        # CARD: Developer
        # ════════════════════════════════════════════════════════════════
        card, body = self._settings_card('Developer')

        debug_row = QHBoxLayout()
        self.verbose_logging = QCheckBox('Verbose ROS2 Logging')
        self.verbose_logging.setChecked(
            self.settings.get('advanced', {}).get('verbose_logging', False))
        debug_row.addWidget(self.verbose_logging)
        self.save_rosbag = QCheckBox('Auto-save rosbag on exit')
        self.save_rosbag.setChecked(
            self.settings.get('advanced', {}).get('save_rosbag', False))
        debug_row.addWidget(self.save_rosbag)
        debug_row.addStretch()
        body.addLayout(debug_row)

        # ── Build / rebuild the workspace ──────────────────────────────
        body.addWidget(QLabel('Build'))
        build_row = QHBoxLayout()
        rebuild_ws_btn = QPushButton('⟳  Rebuild Workspace')
        rebuild_ws_btn.setToolTip('colcon build --symlink-install  (Ctrl+B)')
        rebuild_ws_btn.clicked.connect(self._open_compile_terminal)
        build_row.addWidget(rebuild_ws_btn)
        rebuild_pkg_btn = QPushButton('⟳  Rebuild Package…')
        rebuild_pkg_btn.setToolTip(
            'colcon build --symlink-install --packages-select <pkg>')
        rebuild_pkg_btn.clicked.connect(self._open_compile_package_terminal)
        build_row.addWidget(rebuild_pkg_btn)
        clean_rebuild_btn = QPushButton('⟳  Clean Rebuild')
        clean_rebuild_btn.setToolTip(
            'Remove build/ install/ and rebuild from scratch')
        clean_rebuild_btn.clicked.connect(self._clean_and_rebuild)
        build_row.addWidget(clean_rebuild_btn)
        build_row.addStretch()
        body.addLayout(build_row)

        root.addWidget(card)

        # ── bottom spacer ─────────────────────────────────────────────
        root.addStretch()

        # ── wrap in scroll area ───────────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(page)
        return scroll

    # ── legacy alias kept so _save_settings_from_ui / _reset_settings still work ──
    def _get_theme_selection(self):
        """Return the theme key currently selected in the segmented control."""
        buttons = getattr(self, '_theme_palette_btns', {})
        palette = next((key for key, btn in buttons.items() if btn.isChecked()),
                       'warmth')
        light = getattr(self, '_theme_light_btn', None)
        brightness = 'light' if (light is not None
                                 and light.isChecked()) else 'dark'
        return compose_theme_name(palette, brightness)

    def _set_theme_selection(self, theme_name):
        """Tick the two buttons that make up ``theme_name``, if they exist."""
        palette, brightness = split_theme_name(theme_name)
        for key, button in getattr(self, '_theme_palette_btns', {}).items():
            button.setChecked(key == palette)
        for attr, wanted in (('_theme_dark_btn', brightness == 'dark'),
                             ('_theme_light_btn', brightness == 'light')):
            button = getattr(self, attr, None)
            if button is not None:
                button.setChecked(wanted)

    
    def _save_settings_from_ui(self):
        """Save settings from UI inputs to self.settings, then persist and apply."""
        # General  (station_id is authoritative from the toolbar, not a widget here)
        self.settings['general']['default_robot'] = self.settings_default_robot.currentText()
        self.settings['general']['auto_save'] = self.settings_auto_save.isChecked()
        self.settings['general']['auto_save_interval'] = self.settings_auto_save_interval.value()
        self.settings['general']['show_log_viewer'] = self.settings_show_log.isChecked()
        self.settings['general']['log_viewer_height'] = self.settings_log_height.value()

        # Appearance — theme read from the segmented button pair, not a QComboBox
        self.settings['appearance']['theme'] = self._get_theme_selection()
        self.settings['appearance']['font_size'] = self.settings_font_size.value()
        self.settings['appearance']['font_family'] = self.settings_font_family.currentText()
        
        # Network
        self.settings['network']['ros_domain_id'] = self.settings_ros_domain.value()
        self.settings['network']['partner_ip'] = self.settings_partner_ip.text().strip()
        self.settings['network']['dds_mode'] = (
            'unicast' if self.settings_dds_mode.currentIndex() == 1 else 'multicast')

        # Simulation
        self.settings.setdefault('simulation', {})['backend'] = (
            self.settings_sim_backend.currentData() or _default_sim_backend())

        # Tolerances
        self.settings['tolerances']['joint_tolerance_deg'] = self.settings_joint_tol.value()
        self.settings['tolerances']['position_tolerance_mm'] = self.settings_pos_tol.value()
        self.settings['tolerances']['timeout_seconds'] = self.settings_timeout.value()
        
        # Shortcuts
        for key, line_edit in self.shortcut_inputs.items():
            self.settings['shortcuts'][key] = line_edit.text()

        # Advanced / developer toggles
        self.settings.setdefault('advanced', {})
        self.settings['advanced']['verbose_logging'] = self.verbose_logging.isChecked()
        self.settings['advanced']['save_rosbag'] = self.save_rosbag.isChecked()

        # Save to file
        self._save_settings()

        # Apply settings (includes theme from segmented control)
        self._apply_settings()

        QMessageBox.information(self, 'Settings Saved', 'Settings have been saved successfully.')
    
    def _apply_settings(self):
        """Apply current settings to the GUI."""
        # Apply theme
        self.apply_theme(self.settings['appearance']['theme'])

        # Apply font family and size application-wide
        _app = QApplication.instance()
        if _app is not None:
            _app.setFont(QFont(
                self.settings['appearance']['font_family'],
                int(self.settings['appearance']['font_size']),
            ))
        
        # Apply default robot to dropdown
        if hasattr(self, 'robot_dropdown'):
            default_robot = self.settings['general']['default_robot']
            idx = self.robot_dropdown.findText(default_robot)
            if idx >= 0:
                self.robot_dropdown.setCurrentIndex(idx)
                self.log_message("INFO", f"Default robot set to: {default_robot}", "Settings")
        
        # The log viewer is its own page now, so it always shows its text and
        # fills the page: no visibility toggle and no fixed height to apply.
        if hasattr(self, 'log_viewer'):
            self.log_viewer.log_text.setVisible(True)
        
        # Apply tolerances to sequence mixin
        if hasattr(self, 'joint_tolerance_rad'):
            self.joint_tolerance_rad = self.settings['tolerances']['joint_tolerance_deg'] * 3.14159 / 180.0
        if hasattr(self, 'position_tolerance_m'):
            self.position_tolerance_m = self.settings['tolerances']['position_tolerance_mm'] / 1000.0
        if hasattr(self, 'timeout_seconds'):
            self.timeout_seconds = self.settings['tolerances']['timeout_seconds']
        
        # VR card: the background relay follows it (once the window is up)
        if getattr(self, '_vr_settings_ready', False) and hasattr(self, '_apply_vr_settings'):
            self._apply_vr_settings(from_settings=True)

        # Update auto-save timer
        if self.settings['general']['auto_save']:
            self.auto_save_timer.start(self.settings['general']['auto_save_interval'] * 1000)
        else:
            self.auto_save_timer.stop()
        
        # Apply verbose logging level
        _verbose = self.settings.get('advanced', {}).get('verbose_logging', False)
        _log_level = logging.DEBUG if _verbose else logging.INFO
        logging.getLogger().setLevel(_log_level)

        # Re-setup shortcuts
        self._setup_keyboard_shortcuts()
    

    def _discard_settings(self):
        """Revert all settings widgets and the live theme to the saved snapshot.

        The snapshot is taken in :meth:`show_settings_panel` each time the
        Settings page is opened, so Discard undoes any unsaved edits made
        during this visit without touching the YAML file.
        """
        snapshot = getattr(self, '_settings_snapshot', None)
        if snapshot is None:
            return

        # Restore the in-memory settings dict from the snapshot.
        self.settings = copy.deepcopy(snapshot)

        # Revert the live theme immediately.
        self.apply_theme(self.settings['appearance']['theme'])

        # Refresh every card widget to match the reverted settings.
        self._set_theme_selection(self.settings['appearance']['theme'])

        if hasattr(self, 'settings_font_family'):
            self.settings_font_family.setCurrentText(
                self.settings['appearance']['font_family'])
        if hasattr(self, 'settings_font_size'):
            self.settings_font_size.setValue(self.settings['appearance']['font_size'])

        if hasattr(self, 'settings_default_robot'):
            self.settings_default_robot.setCurrentText(
                self.settings['general']['default_robot'])
        if hasattr(self, 'settings_auto_save'):
            self.settings_auto_save.setChecked(self.settings['general']['auto_save'])
        if hasattr(self, 'settings_auto_save_interval'):
            self.settings_auto_save_interval.setValue(
                self.settings['general']['auto_save_interval'])
        if hasattr(self, 'settings_show_log'):
            self.settings_show_log.setChecked(self.settings['general']['show_log_viewer'])
        if hasattr(self, 'settings_log_height'):
            self.settings_log_height.setValue(self.settings['general']['log_viewer_height'])

        if hasattr(self, 'settings_ros_domain'):
            self.settings_ros_domain.setValue(self.settings['network']['ros_domain_id'])
        if hasattr(self, 'settings_partner_ip'):
            self.settings_partner_ip.setText(
                self.settings['network'].get('partner_ip', ''))
        if hasattr(self, 'settings_dds_mode'):
            self.settings_dds_mode.setCurrentIndex(
                1 if self.settings['network'].get('dds_mode') == 'unicast' else 0)
        if hasattr(self, 'settings_sim_backend'):
            idx = self.settings_sim_backend.findData(normalize_sim_backend(
                self.settings.get('simulation', {}).get('backend')))
            self.settings_sim_backend.setCurrentIndex(max(0, idx))

        if hasattr(self, 'settings_joint_tol'):
            self.settings_joint_tol.setValue(
                self.settings['tolerances']['joint_tolerance_deg'])
        if hasattr(self, 'settings_pos_tol'):
            self.settings_pos_tol.setValue(
                self.settings['tolerances']['position_tolerance_mm'])
        if hasattr(self, 'settings_timeout'):
            self.settings_timeout.setValue(self.settings['tolerances']['timeout_seconds'])

        if hasattr(self, 'shortcut_inputs'):
            for key, line_edit in self.shortcut_inputs.items():
                line_edit.setText(self.settings['shortcuts'].get(key, ''))

        if hasattr(self, 'verbose_logging'):
            self.verbose_logging.setChecked(
                self.settings.get('advanced', {}).get('verbose_logging', False))
        if hasattr(self, 'save_rosbag'):
            self.save_rosbag.setChecked(
                self.settings.get('advanced', {}).get('save_rosbag', False))

    def _generate_cyclone_config(self):
        """Generate CycloneDDS config file for unicast peer discovery."""
        partner_ip = self.settings_partner_ip.text().strip()
        if not partner_ip:
            QMessageBox.warning(self, "Missing IP",
                                "Please enter the partner machine's IP address.")
            return

        import socket
        try:
            own_ip = socket.gethostbyname(socket.gethostname())
        except Exception:
            own_ip = "127.0.0.1"

        config_xml = (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<CycloneDDS xmlns="https://cdds.io/config">\n'
            '  <Domain>\n'
            '    <General>\n'
            '      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>\n'
            '    </General>\n'
            '    <Discovery>\n'
            '      <ParticipantIndex>auto</ParticipantIndex>\n'
            '      <Peers>\n'
            f'        <Peer address="{own_ip}"/>\n'
            f'        <Peer address="{partner_ip}"/>\n'
            '      </Peers>\n'
            '    </Discovery>\n'
            '  </Domain>\n'
            '</CycloneDDS>\n'
        )

        config_path = os.path.expanduser("~/cyclonedds.xml")
        try:
            with open(config_path, 'w') as f:
                f.write(config_xml)
            self.log_message("INFO",
                             f"CycloneDDS config written to {config_path}", "Settings")
            QMessageBox.information(
                self, "Config Generated",
                f"CycloneDDS configuration saved to:\n{config_path}\n\n"
                f"Own IP: {own_ip}\nPartner IP: {partner_ip}\n\n"
                "To activate, set the environment variable before launching:\n"
                f"  export CYCLONEDDS_URI=file://{config_path}")
        except Exception as e:
            QMessageBox.critical(self, "Error",
                                 f"Failed to write config:\n{e}")

    def show_settings_panel(self):
        """Show the settings panel and snapshot current settings for Discard."""
        self._settings_snapshot = copy.deepcopy(self.settings)
        self._set_active_sidebar_btn(self._side_buttons[9])
        self._show_page('settings')

    # ═══════════════════════════════════════════════════════════════════════════════
    #  ADVANCED TAB METHODS
    # ═══════════════════════════════════════════════════════════════════════════════
    
    def _get_workspace_path(self) -> str:
        """Get the current workspace path."""
        if hasattr(self, 'ws_path_edit'):
            return self.ws_path_edit.text()
        return _FIC_WS
    
    def _open_compile_terminal(self):
        """Compile the full workspace in an in-app build dialog."""
        ws_path = self._get_workspace_path()
        if not os.path.exists(ws_path):
            QMessageBox.warning(self, "Path Not Found", f"Workspace not found:\n{ws_path}")
            return
        from fic_studio.widgets import BuildDialog
        dlg = BuildDialog(
            title="Compile Workspace",
            command=colcon_build_command(),
            work_dir=ws_path,
            parent=self,
        )
        self.log_message("INFO", "Started workspace build", "Build")
        dlg.exec_()

    def _open_compile_package_terminal(self):
        """Compile a specific package in an in-app build dialog."""
        ws_path = self._get_workspace_path()
        src_path = os.path.join(ws_path, "src")
        if not os.path.exists(src_path):
            QMessageBox.warning(self, "Path Not Found", f"Source directory not found:\n{src_path}")
            return
        found = sorted([
            d for d in os.listdir(src_path)
            if os.path.isdir(os.path.join(src_path, d)) and not d.startswith('.')
        ])
        packages = found
        if not packages:
            QMessageBox.warning(self, "No Packages", "No packages found in src directory.")
            return
        prompt = "Package to compile:"
        from PyQt5.QtWidgets import QInputDialog
        package, ok = QInputDialog.getItem(
            self, "Select Package", prompt, packages, 0, False)
        if not (ok and package):
            return
        from fic_studio.widgets import BuildDialog
        dlg = BuildDialog(
            title=f"Compile — {package}",
            command=colcon_build_command([package]),
            work_dir=ws_path,
            parent=self,
        )
        self.log_message("INFO", f"Started build for package: {package}", "Build")
        dlg.exec_()

    def _clean_and_rebuild(self):
        """Clean build directories and rebuild in an in-app build dialog."""
        ws_path = self._get_workspace_path()
        reply = QMessageBox.question(
            self, "Confirm Clean Build",
            f"This will delete:\n"
            f"  - {ws_path}/build/\n"
            f"  - {ws_path}/install/\n"
            f"  - {ws_path}/log/\n\n"
            f"And then run {colcon_build_command()}.\n\n"
            f"Continue?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        from fic_studio.widgets import BuildDialog
        dlg = BuildDialog(
            title="Clean Build",
            command=colcon_build_command(),
            work_dir=ws_path,
            parent=self,
            pre_command="rm -rf build install log",
        )
        self.log_message("INFO", "Started clean rebuild", "Build")
        dlg.exec_()
    
    def _browse_workspace(self):
        """Browse for workspace directory."""
        current = self._get_workspace_path()
        directory = QFileDialog.getExistingDirectory(
            self, "Select ROS2 Workspace", current
        )
        if directory:
            self.ws_path_edit.setText(directory)
            self.log_message("INFO", f"Workspace path set to: {directory}", "Settings")