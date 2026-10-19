# mixins/dashboard_mixin.py — Grouped numeric monitoring page (Dashboard home)
"""
Dashboard mixin: the Studio's monitoring page.

Modelled on the Kinova Kortex Web App's Monitoring page — a dense,
read-only numeric readout grouped into Base / Joints / End effector /
Safety, split across an Overview and a Detailed tab. Robot-driving
controls live on the global control bar, not here.

A channel with no live source renders ``--``; nothing is fabricated.
Only the visible tab is refreshed.
"""

import math
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QPushButton, QScrollArea, QTabWidget,
    QVBoxLayout, QWidget,
)

from fic_studio import theme
from fic_studio.components.dash_widgets import StatusRow
from fic_studio.components.telemetry_table import TelemetryTable
from fic_studio.math_helpers import _quat_to_euler_xyz
from fic_studio.pages import page_showing

#: main_stack index the dashboard page occupies.
DASHBOARD_PAGE_INDEX = 0

#: Force magnitude (N) above which the tool is considered in contact.
CONTACT_FORCE_N = 5.0

#: Caption column width, shared by every Base row and by the telemetry
#: tables' row labels, so captions and values line up down the page.
CAPTION_WIDTH = 118

JOINT_COLUMNS = ['commanded', 'position', 'error', 'velocity', 'torque']
JOINT_DETAIL_COLUMNS = JOINT_COLUMNS + [
    'τ max', '% τ', 'limit lo', 'limit hi', '% range',
]
EE_COLUMNS = ['x', 'y', 'z', 'ox', 'oy', 'oz']
EE_ROWS = ['commanded', 'pose', 'error', 'velocity']
EE_DETAIL_ROWS = EE_ROWS + ['quaternion']
DETAIL_COLUMN_WIDTH = 84
WRENCH_COLUMNS = ['x', 'y', 'z']
WRENCH_ROWS = ['force', 'torque']


def _lab(text, token=None, size=None, bold=False, mono=False):
    """Create a QLabel with token-bound styling that follows theme switches."""
    q = QLabel(text)

    def _css(T):
        css = 'background: transparent; border: none;'
        if size:
            css += ' font-size: %dpx;' % size
        if bold:
            css += ' font-weight: 700;'
        if mono:
            css += " font-family: 'JetBrains Mono', 'Consolas', monospace;"
        if token:
            css += ' color: %s;' % T[token]
        return css

    if token:
        q.setStyleSheet(_css(theme.current()))
        theme.bind(q, _css)
    else:
        q.setStyleSheet(_css({}))
    return q


def _card(title=None, accent_token=None):
    """
    Create a styled card frame with optional left-accent border.

    Returns (QFrame, QVBoxLayout). Also imported by topic_monitor_mixin,
    so the accent_token parameter is part of a shared contract.
    """
    f = QFrame()
    # Scoped by object name on purpose. QLabel derives from QFrame, so an
    # unscoped `QFrame { border: ... }` here cascades onto every label in
    # the card and draws a box around each one.
    f.setObjectName('DashCard')
    layout = QVBoxLayout(f)
    layout.setContentsMargins(15, 12, 15, 12)
    layout.setSpacing(7)

    def _style(T):
        if accent_token:
            return (
                'QFrame#DashCard { background: %s; border: 1px solid %s;'
                ' border-left: 3px solid %s; border-radius: 10px; }'
                % (T['bg_raised'], T['border_subtle'], T[accent_token])
            )
        return (
            'QFrame#DashCard { background: %s; border: 1px solid %s;'
            ' border-radius: 10px; }'
            % (T['bg_raised'], T['border_subtle'])
        )

    f.setStyleSheet(_style(theme.current()))
    theme.bind(f, _style)

    if title:
        hdr = _lab(title.upper(), token='text_disabled', size=10, bold=True)
        layout.addWidget(hdr)
    return f, layout


class DashboardMixin:
    """Mixin providing the grouped numeric monitoring page."""

    #: Skip periodic work while this page is hidden.
    _page_showing = page_showing

    # ──────────────────────────────────────────────────────────────────────
    #  Initialization
    # ──────────────────────────────────────────────────────────────────────

    def _init_dashboard(self):
        """Initialise dashboard state; called once from __init__."""
        self._dash_refresh_timer = QTimer()
        self._dash_refresh_timer.timeout.connect(self._refresh_dashboard_tiles)
        self._dash_refresh_timer.start(50)

    # ──────────────────────────────────────────────────────────────────────
    #  Page Builder
    # ──────────────────────────────────────────────────────────────────────

    def _build_dashboard_page(self):
        """Build the monitoring page and return it wrapped in a QScrollArea."""
        page = QWidget()
        root = QVBoxLayout(page)
        root.setContentsMargins(22, 18, 22, 18)
        root.setSpacing(12)

        root.addLayout(self._dash_build_header())

        self._dash_tabs = QTabWidget()
        self._dash_tabs.addTab(self._dash_build_overview(), 'Overview')
        self._dash_tabs.addTab(self._dash_build_detailed(), 'Detailed')

        # Seed the joint rows now so the page reads as a table of '--'
        # before any telemetry arrives, rather than an empty box.
        labels = [d['label'] for d in self._dash_display_labels()]
        self._dash_joint_table.set_rows(labels)
        self._dash_joint_detail_table.set_rows(labels)
        root.addWidget(self._dash_tabs)
        root.addStretch()

        # Hidden labels kept alive for hooks from other mixins.
        self._dash_autotune_status = QLabel()
        self._dash_experiment_status = QLabel()
        self._dash_teleop_status = QLabel()

        self._dash_last_ee_pos = None
        self._dash_last_ee_time = None

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(page)
        return scroll

    def _dash_build_header(self):
        """Build the title row: name, status pill, identity, health."""
        head = QHBoxLayout()
        head.setSpacing(10)
        head.addWidget(_lab('Dashboard', token='text_primary', size=22, bold=True))
        head.addSpacing(10)

        self._dash_status_pill = QLabel('● idle')
        self._dash_status_pill.setFixedHeight(24)
        self._dash_status_pill.setMaximumWidth(120)
        self._apply_pill_style(running=False)
        head.addWidget(self._dash_status_pill)
        head.addSpacing(8)

        self._dash_header_info = _lab('-- · -- · -- · --', token='text_disabled', size=12)
        head.addWidget(self._dash_header_info)
        head.addStretch()

        self._dash_modifiers_lbl = _lab('', token='accent_teal', size=11, bold=True)
        head.addWidget(self._dash_modifiers_lbl)
        head.addSpacing(10)
        self._dash_health_lbl = _lab('-- Hz · -- ms', token='text_disabled', size=12, mono=True)
        head.addWidget(self._dash_health_lbl)
        return head

    def _dash_build_overview(self):
        """Build the Overview tab: Base, Joints, End effector, Safety."""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 12, 0, 0)
        lay.setSpacing(12)

        lay.addWidget(self._dash_build_base_card())

        joints, jl = _card('JOINTS')
        self._dash_joint_table = TelemetryTable(
            columns=JOINT_COLUMNS, decimals=2, label_width=CAPTION_WIDTH)
        jl.addWidget(self._dash_joint_table)
        lay.addWidget(joints)

        ee, el = _card('END EFFECTOR')
        self._dash_ee_table = TelemetryTable(
            columns=EE_COLUMNS, decimals=3, label_width=CAPTION_WIDTH)
        self._dash_ee_table.set_rows(EE_ROWS)
        el.addWidget(self._dash_ee_table)

        wrench_head = QHBoxLayout()
        wrench_head.addWidget(_lab('tool wrench', token='text_disabled', size=10))
        wrench_head.addStretch()
        self._dash_force_mag = _lab('|F| -- N', token='text_primary', size=11, mono=True)
        self._dash_torque_mag = _lab('|τ| -- Nm', token='text_primary', size=11, mono=True)
        self._dash_contact_lbl = _lab('○ free', token='text_disabled', size=11, bold=True)
        for lbl in (self._dash_force_mag, self._dash_torque_mag, self._dash_contact_lbl):
            wrench_head.addWidget(lbl)
        el.addLayout(wrench_head)

        self._dash_wrench_table = TelemetryTable(
            columns=WRENCH_COLUMNS, decimals=2, label_width=CAPTION_WIDTH)
        self._dash_wrench_table.set_rows(WRENCH_ROWS)
        el.addWidget(self._dash_wrench_table)
        lay.addWidget(ee)

        lay.addWidget(self._dash_build_safety_card())
        lay.addStretch()
        return w

    def _dash_build_detailed(self):
        """Build the Detailed tab.

        Kinova's Detailed view adds motor currents, voltages and
        temperatures. FIC publishes none of those, so this shows what this
        stack genuinely knows instead: per-joint headroom against the
        configured limits, the full end-effector and wrench breakdown, the
        active robot's configuration, and which telemetry channels are
        actually carrying data.
        """
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 12, 0, 0)
        lay.setSpacing(12)

        joints, jl = _card('JOINTS  ·  limits and headroom')
        self._dash_joint_detail_table = TelemetryTable(
            columns=JOINT_DETAIL_COLUMNS, decimals=2,
            label_width=CAPTION_WIDTH, column_width=DETAIL_COLUMN_WIDTH)
        jl.addWidget(self._dash_joint_detail_table)
        lay.addWidget(joints)

        ee, el = _card('END EFFECTOR  ·  pose, twist and wrench')
        self._dash_ee_detail_table = TelemetryTable(
            columns=EE_COLUMNS, decimals=4, label_width=CAPTION_WIDTH,
            column_width=DETAIL_COLUMN_WIDTH)
        self._dash_ee_detail_table.set_rows(EE_DETAIL_ROWS)
        el.addWidget(self._dash_ee_detail_table)
        self._dash_wrench_detail_table = TelemetryTable(
            columns=['x', 'y', 'z', 'magnitude'], decimals=3,
            label_width=CAPTION_WIDTH, column_width=DETAIL_COLUMN_WIDTH)
        self._dash_wrench_detail_table.set_rows(['force', 'torque'])
        el.addWidget(self._dash_wrench_detail_table)
        lay.addWidget(ee)

        row = QHBoxLayout()
        row.setSpacing(12)

        robot, rl = _card('ROBOT')
        self._dash_robot_rows = self._dash_kv_rows(rl, [
            'model', 'joints', 'gripper', 'ee frame', 'elbow frame',
            'torque sign', 'gravity comp', 'wrench source', 'urdf',
        ])
        row.addWidget(robot, 1)

        chan, cl = _card('TELEMETRY CHANNELS')
        self._dash_channel_rows = self._dash_kv_rows(cl, [
            'joint states', 'joint velocity', 'joint effort',
            'end-effector pose', 'tool wrench', 'deadbands',
        ])
        row.addWidget(chan, 1)
        lay.addLayout(row)

        lay.addStretch()
        return w

    @staticmethod
    def _dash_kv_rows(layout, captions):
        """Add one caption/value row per entry; return {caption: value label}."""
        rows = {}
        for caption in captions:
            r = QHBoxLayout()
            r.setSpacing(14)
            cap = _lab(caption, token='text_disabled', size=11)
            cap.setFixedWidth(CAPTION_WIDTH)
            r.addWidget(cap)
            val = _lab('--', token='text_primary', size=11, mono=True)
            val.setWordWrap(False)
            r.addWidget(val, 1)
            layout.addLayout(r)
            rows[caption] = val
        return rows

    def _dash_build_base_card(self):
        """Build the Base card: process state, control mode, target, telemetry."""
        card, lay = _card('BASE')

        self._dash_session_rows = {}
        sessions = [
            ('simulator', 'SIM', self._start_simulation, self._stop_simulation),
            ('fic controller', 'FIC', self._start_fic_simulation, self._stop_fic_simulation),
            ('driver', 'DRV', self._start_driver, self._stop_driver),
        ]
        row = QHBoxLayout()
        row.setSpacing(22)
        _cap = _lab('operating mode', token='text_disabled', size=11)
        _cap.setFixedWidth(CAPTION_WIDTH)
        row.addWidget(_cap)
        for name, key, start_fn, stop_fn in sessions:
            cell = QHBoxLayout()
            cell.setSpacing(5)
            cell.setAlignment(Qt.AlignVCenter)
            cell.addWidget(_lab(name, token='text_secondary', size=11),
                           0, Qt.AlignVCenter)
            start_btn = QPushButton('▶')
            start_btn.setFixedSize(24, 20)
            start_btn.setCursor(Qt.PointingHandCursor)
            start_btn.clicked.connect(start_fn)
            stop_btn = QPushButton('■')
            stop_btn.setFixedSize(24, 20)
            stop_btn.setCursor(Qt.PointingHandCursor)
            stop_btn.clicked.connect(stop_fn)
            for b, tok in ((start_btn, 'accent_green'), (stop_btn, 'accent_red')):
                theme.bind(b, lambda T, tk=tok: (
                    'QPushButton { background: %s; color: %s; border: none;'
                    ' border-radius: 4px; font-size: 10px; }'
                    'QPushButton:hover { background: %s; }'
                    % (T['bg_overlay'], T[tk], T['border_subtle'])
                ))
            cell.addWidget(start_btn, 0, Qt.AlignVCenter)
            cell.addWidget(stop_btn, 0, Qt.AlignVCenter)
            status = _lab('stopped', token='text_disabled', size=10)
            status.setFixedWidth(52)
            cell.addWidget(status, 0, Qt.AlignVCenter)
            self._dash_session_rows[key] = status
            row.addLayout(cell)
        row.addStretch()
        lay.addLayout(row)

        self._dash_base_rows = {}
        for key, caption in (('control mode', 'control mode'),
                             ('target', 'target'),
                             ('profile', 'profile'),
                             ('telemetry', 'telemetry')):
            r = QHBoxLayout()
            r.setSpacing(22)
            cap = _lab(caption, token='text_disabled', size=11)
            cap.setFixedWidth(CAPTION_WIDTH)
            r.addWidget(cap)
            val = _lab('--', token='text_primary', size=12, mono=True)
            r.addWidget(val)
            r.addStretch()
            lay.addLayout(r)
            self._dash_base_rows[key] = val
        return card

    def _dash_build_safety_card(self):
        """Build the Safety card with its three limit indicators."""
        card, lay = _card('SAFETY')
        self._dash_safety_rows = {
            'joints': StatusRow('Joint limits', ok=True),
            'velocity': StatusRow('Velocity limits', ok=True),
            'torque': StatusRow('Force / torque', ok=True),
        }
        row = QHBoxLayout()
        row.setSpacing(20)
        for sr in self._dash_safety_rows.values():
            row.addWidget(sr)
        row.addStretch()
        lay.addLayout(row)
        return card

    # ──────────────────────────────────────────────────────────────────────
    #  Pill styling helper
    # ──────────────────────────────────────────────────────────────────────

    def _apply_pill_style(self, running=False):
        """Apply colour-token pill style to the header status pill label."""
        tok = 'accent_green' if running else 'text_disabled'
        t = theme.current()
        self._dash_status_pill.setStyleSheet(
            'background: %s; border-radius: 11px; padding: 4px 11px;'
            ' font-size: 11px; font-weight: 700; color: %s;'
            % (t['bg_overlay'], t[tok])
        )
        self._dash_status_pill.setText('● running' if running else '● idle')

    # ──────────────────────────────────────────────────────────────────────
    #  Periodic Refresh (~20 Hz / 50 ms timer)
    # ──────────────────────────────────────────────────────────────────────

    def _refresh_dashboard_tiles(self):
        """Refresh the visible tab from live telemetry.

        Exits immediately when the dashboard page is not showing, so the
        cost off-page is a single integer comparison per tick.
        """
        if not self._page_showing('dashboard'):
            return

        self._refresh_dash_header()
        self._refresh_dash_session()
        self._refresh_dash_safety()

        detailed = self._dash_tabs.currentIndex() == 1
        if detailed:
            self._refresh_dash_joints(self._dash_joint_detail_table, detailed=True)
            self._refresh_dash_ee_detail()
            self._refresh_dash_robot_card()
            self._refresh_dash_channels()
        else:
            self._refresh_dash_joints(self._dash_joint_table, detailed=False)
            self._refresh_dash_ee()
            self._refresh_dash_wrench()

    def _refresh_dash_header(self):
        """Update the header identity line, health readout and modifiers."""
        robot_name = self.robot_dropdown.currentText()
        rid = self.node.station_robot_id if hasattr(self, 'node') else 'robot1'
        station_n = rid.replace('robot', '') if rid.startswith('robot') else rid
        target = self.robot_target(rid) if hasattr(self, 'robot_target') else '--'
        mode_text = '--'
        if hasattr(self, '_rc_mode_dropdown'):
            mode_text = self._rc_mode_dropdown.currentText()
        elif hasattr(self, 'tb_mode_dropdown'):
            mode_text = self.tb_mode_dropdown.currentText()

        self._dash_header_info.setText(
            '%s · Robot %s · %s · %s' % (robot_name, station_n, target, mode_text))
        self._dash_base_rows['control mode'].setText(mode_text)
        self._dash_base_rows['target'].setText(str(target))
        profile = '--'
        if hasattr(self, '_rc_current_profile'):
            try:
                profile = self._rc_current_profile()
            except Exception:
                profile = '--'
        self._dash_base_rows['profile'].setText(str(profile))

        # One source of truth, shared with the control bar's indicator, so
        # the two cannot report different things about the same stream.
        rate, age_ms, state = 0.0, None, 'offline'
        if hasattr(self, 'telemetry_health'):
            rate, age_ms, state = self.telemetry_health(rid)
        if state == 'offline':
            health = '-- Hz · offline'
        else:
            health = '%s · %.0f Hz · %.0f ms' % (
                '● live' if state == 'live' else '● stale', rate, age_ms)
        self._dash_health_lbl.setText(health)
        self._dash_base_rows['telemetry'].setText(health)
        tok = {'live': 'accent_green', 'stale': 'accent_orange'}.get(
            state, 'text_disabled')
        t = theme.current()
        self._dash_health_lbl.setStyleSheet(
            'background: transparent; border: none; font-size: 12px;'
            " font-family: 'JetBrains Mono', 'Consolas', monospace;"
            ' color: %s;' % t[tok])

        mods = []
        for attr, name in (('_rc_postural_toggle', 'POST'),
                           ('_rc_elbow_toggle', 'ELB'),
                           ('_rc_hpmp_toggle', 'PMP'),
                           ('_rc_admittance_planner_toggle', 'ADM')):
            w = getattr(self, attr, None)
            if w is not None and w.isChecked():
                mods.append(name)
        self._dash_modifiers_lbl.setText('  '.join(mods))

        any_running = (getattr(self, 'simulation_running', False)
                       or getattr(self, 'fic_running', False)
                       or getattr(self, 'controller_running', False))
        self._apply_pill_style(running=any_running)

    def _dash_display_labels(self):
        """Display-joint list for the active robot's rows.

        Reuses the Robot-Control display-joint logic so a robot with
        view_joint_names (e.g. the CSPM parallel robot) shows ALL its joints —
        actuated + passive + platform — not just the actuated num_joints subset.
        Falls back to J1..Jn from num_joints when that mixin isn't available.
        """
        robot = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
        if hasattr(self, '_rc_display_joints'):
            try:
                disp = self._rc_display_joints(robot)
                if disp:
                    return disp
            except Exception:
                pass
        n = int(self.robots.get(robot, 7)) if hasattr(self, 'robots') else 7
        return [{'name': '%s_j%d' % (robot, i + 1), 'label': 'J%d' % (i + 1),
                 'actuated': True, 'cmd_index': i, 'lo': -180.0, 'hi': 180.0}
                for i in range(n)]

    def _refresh_dash_joints(self, table, detailed=False):
        """Fill one joint table from telemetry, in degrees."""
        display = self._dash_display_labels()
        table.set_rows([d['label'] for d in display])

        positions = self._dash_get('joint_positions_for')
        commanded = self._dash_get('joint_commanded_for')
        velocities = self._dash_get('joint_velocity_for')
        efforts = self._dash_get('joint_effort_for')

        cfg = {}
        if hasattr(self, 'robot_configs'):
            cfg = self.robot_configs.get(self.robot_dropdown.currentText(), {})
        tlim = cfg.get('joint_torque_limits', {})
        t_max_raw = tlim.get('max', None) if isinstance(tlim, dict) else None

        for d in display:
            i = d.get('cmd_index')
            pos = self._deg(self._at(positions, i))
            cmd = self._deg(self._at(commanded, i))
            vel = self._deg(self._at(velocities, i))
            eff = self._at(efforts, i)
            err = None if (pos is None or cmd is None) else cmd - pos

            values = [cmd, pos, err, vel, eff]
            if detailed:
                tmx = t_max_raw[i] if isinstance(t_max_raw, list) else t_max_raw
                pct_t = None
                if eff is not None and tmx:
                    pct_t = abs(eff) / abs(tmx) * 100.0
                lo, hi = d.get('lo', -180.0), d.get('hi', 180.0)
                pct_r = None
                if pos is not None and hi > lo:
                    pct_r = (pos - lo) / (hi - lo) * 100.0
                values += [tmx, pct_t, lo, hi, pct_r]
            table.set_values(d['label'], values)

    def _refresh_dash_ee(self):
        """Fill the end-effector table from the live pose."""
        pose = self.ee_pose_for(None) if hasattr(self, 'ee_pose_for') else None
        if pose is None:
            for name in EE_ROWS:
                self._dash_ee_table.set_values(name, [None] * len(EE_COLUMNS))
            return

        p = pose.position
        o = getattr(pose, 'orientation', None)
        if o is not None:
            ox, oy, oz = _quat_to_euler_xyz(o.x, o.y, o.z, o.w)
        else:
            ox = oy = oz = None
        self._dash_ee_table.set_values('pose', [p.x, p.y, p.z, ox, oy, oz])

        # No commanded-pose source is published yet; show '--' rather than
        # echoing the actual pose back as if it were a target.
        self._dash_ee_table.set_values('commanded', [None] * len(EE_COLUMNS))
        self._dash_ee_table.set_values('error', [None] * len(EE_COLUMNS))

        now = time.monotonic()
        cur = (p.x, p.y, p.z)
        vel = [None] * len(EE_COLUMNS)
        last, last_t = self._dash_last_ee_pos, self._dash_last_ee_time
        if last is not None and last_t is not None and now > last_t:
            dt = now - last_t
            vel[0] = (cur[0] - last[0]) / dt
            vel[1] = (cur[1] - last[1]) / dt
            vel[2] = (cur[2] - last[2]) / dt
        self._dash_ee_table.set_values('velocity', vel)
        self._dash_last_ee_pos = cur
        self._dash_last_ee_time = now

    def _refresh_dash_wrench(self):
        """Fill the tool-wrench rows and the contact indicator."""
        w = self.wrench_for(None) if hasattr(self, 'wrench_for') else None
        if w is None:
            self._dash_wrench_table.set_values('force', [None, None, None])
            self._dash_wrench_table.set_values('torque', [None, None, None])
            self._dash_force_mag.setText('|F| -- N')
            self._dash_torque_mag.setText('|τ| -- Nm')
            self._dash_contact_lbl.setText('○ free')
            return

        # wrench_for() returns the (Fx, Fy, Fz, Tx, Ty, Tz) tuple, not a
        # geometry_msgs/Wrench. Treating it as a message raised on the first
        # sample, and that uncaught exception tripped the crash hook, which
        # killed fic_control and the driver mid-run.
        fx, fy, fz, tx, ty, tz = w
        self._dash_wrench_table.set_values('force', [fx, fy, fz])
        self._dash_wrench_table.set_values('torque', [tx, ty, tz])
        fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
        tmag = math.sqrt(tx * tx + ty * ty + tz * tz)
        self._dash_force_mag.setText('|F| %.1f N' % fmag)
        self._dash_torque_mag.setText('|τ| %.2f Nm' % tmag)
        self._dash_contact_lbl.setText(
            '● contact' if fmag >= CONTACT_FORCE_N else '○ free')

    def _refresh_dash_session(self):
        """Reflect process running flags in the Base card."""
        state_map = {
            'SIM': getattr(self, 'simulation_running', False),
            'FIC': getattr(self, 'fic_running', False),
            'DRV': getattr(self, 'driver_running', False),
        }
        t = theme.current()
        for key, running in state_map.items():
            lbl = self._dash_session_rows.get(key)
            if lbl is None:
                continue
            lbl.setText('running' if running else 'stopped')
            tok = 'accent_green' if running else 'text_disabled'
            lbl.setStyleSheet(
                'color: %s; font-size: 10px; background: transparent;' % t[tok])

    def _refresh_dash_ee_detail(self):
        """Fill the Detailed tab's end-effector and wrench breakdown."""
        pose = self.ee_pose_for(None) if hasattr(self, 'ee_pose_for') else None
        blank = [None] * len(EE_COLUMNS)
        if pose is None:
            for name in EE_DETAIL_ROWS:
                self._dash_ee_detail_table.set_values(name, blank)
        else:
            p = pose.position
            o = getattr(pose, 'orientation', None)
            if o is not None:
                ox, oy, oz = _quat_to_euler_xyz(o.x, o.y, o.z, o.w)
                self._dash_ee_detail_table.set_values(
                    'quaternion', [o.x, o.y, o.z, o.w, None, None])
            else:
                ox = oy = oz = None
                self._dash_ee_detail_table.set_values('quaternion', blank)
            self._dash_ee_detail_table.set_values(
                'pose', [p.x, p.y, p.z, ox, oy, oz])
            self._dash_ee_detail_table.set_values('commanded', blank)
            self._dash_ee_detail_table.set_values('error', blank)
            self._dash_ee_detail_table.set_values('velocity', blank)

        w = self.wrench_for(None) if hasattr(self, 'wrench_for') else None
        if w is None:
            self._dash_wrench_detail_table.set_values('force', [None] * 4)
            self._dash_wrench_detail_table.set_values('torque', [None] * 4)
            return
        fx, fy, fz, tx, ty, tz = w
        fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
        tmag = math.sqrt(tx * tx + ty * ty + tz * tz)
        self._dash_wrench_detail_table.set_values('force', [fx, fy, fz, fmag])
        self._dash_wrench_detail_table.set_values('torque', [tx, ty, tz, tmag])

    def _dash_active_config(self):
        """Return the active robot's config dict."""
        if not hasattr(self, 'robot_configs'):
            return {}
        return self.robot_configs.get(self.robot_dropdown.currentText(), {})

    def _refresh_dash_robot_card(self):
        """Fill the Detailed tab's ROBOT card from robots_config.yaml."""
        cfg = self._dash_active_config()
        rows = self._dash_robot_rows
        name = cfg.get('display_name') or self.robot_dropdown.currentText() or '--'
        rows['model'].setText(str(name))
        rows['joints'].setText(str(cfg.get('num_joints', '--')))
        if cfg.get('has_gripper'):
            rng = cfg.get('gripper_range')
            rows['gripper'].setText(
                'yes  %s' % (rng if rng else '')) if rng else rows['gripper'].setText('yes')
        else:
            rows['gripper'].setText('none')
        rows['ee frame'].setText(str(cfg.get('ee_frame_name', '--')))
        rows['elbow frame'].setText(str(cfg.get('elbow_frame_name', '--')))
        rows['torque sign'].setText(str(cfg.get('torque_sign', '--')))
        rows['gravity comp'].setText(
            'on' if cfg.get('gravity_compensated') else 'off')
        src = []
        if cfg.get('wrench_from_api'):
            src.append('driver API')
        else:
            src.append('estimated')
        if cfg.get('wrench_gravity_compensated'):
            src.append('gravity-compensated')
        rows['wrench source'].setText(' · '.join(src))
        rows['urdf'].setText(str(cfg.get('urdf_path', '--')))

    def _refresh_dash_channels(self):
        """Report which telemetry channels are actually carrying data.

        Only joint state is timestamped, so that row shows a real rate and
        age; the rest report presence and sample width rather than a
        fabricated age.
        """
        rid = self.node.station_robot_id if hasattr(self, 'node') else 'robot1'
        rows = self._dash_channel_rows
        now = time.monotonic()

        age = None
        if hasattr(self, 'joint_update_time'):
            jt = self.joint_update_time(rid)
            if jt is not None:
                age = (now - jt) * 1000.0
        count = self.joint_update_count(rid) if hasattr(self, 'joint_update_count') else 0
        rows['joint states'].setText(
            'offline' if age is None else '%d samples · %.0f ms ago' % (count, age))

        for caption, accessor in (('joint velocity', 'joint_velocity_for'),
                                  ('joint effort', 'joint_effort_for')):
            vals = self._dash_get(accessor)
            rows[caption].setText('%d values' % len(vals) if vals else 'no data')

        pose = self.ee_pose_for(None) if hasattr(self, 'ee_pose_for') else None
        rows['end-effector pose'].setText('streaming' if pose is not None else 'no data')
        wrench = self.wrench_for(None) if hasattr(self, 'wrench_for') else None
        rows['tool wrench'].setText('streaming' if wrench is not None else 'no data')

        db = self._dash_active_config().get('admittance_deadbands', {})
        if db:
            rows['deadbands'].setText(
                'τ %s · F %s · M %s' % (db.get('joint_tau', '--'),
                                        db.get('cart_force', '--'),
                                        db.get('cart_torque', '--'))
                + (' · arm F %s' % db['cart_force_hardware'] if 'cart_force_hardware' in db else ''))
        else:
            rows['deadbands'].setText('--')

    # ── Small telemetry helpers ───────────────────────────────────────────

    def _dash_get(self, accessor):
        """Call a JointStateMixin list accessor, or return [] if absent."""
        fn = getattr(self, accessor, None)
        return list(fn(None)) if fn is not None else []

    @staticmethod
    def _at(seq, index):
        """Return seq[index], or None when out of range."""
        if index is None or not seq or index >= len(seq):
            return None
        return seq[index]

    @staticmethod
    def _deg(value):
        """Convert radians to degrees, passing None through."""
        return None if value is None else math.degrees(value)

    def _refresh_dash_safety(self):
        """Update the safety StatusRows (GUI-side heuristics)."""
        robot_name = self.robot_dropdown.currentText()
        cfg = {}
        if hasattr(self, 'robot_configs'):
            cfg = self.robot_configs.get(robot_name, {})
        jlim = cfg.get('joint_limits_deg', {})
        tlim = cfg.get('joint_torque_limits', {})

        positions = self._dash_get('joint_positions_for')
        efforts = self._dash_get('joint_effort_for')

        # Joint limit proximity (warn within 5% of range per joint).
        joints_ok = True
        if positions and isinstance(jlim, dict):
            lower_raw = jlim.get('lower', -180)
            upper_raw = jlim.get('upper', 180)
            for i, val in enumerate(positions):
                lo_deg = lower_raw[i] if isinstance(lower_raw, list) else lower_raw
                hi_deg = upper_raw[i] if isinstance(upper_raw, list) else upper_raw
                lo = math.radians(lo_deg)
                hi = math.radians(hi_deg)
                margin = (hi - lo) * 0.05
                if val <= lo + margin or val >= hi - margin:
                    joints_ok = False
                    break
        self._dash_safety_rows['joints'].set_ok(joints_ok)

        # Velocity limits: no velocity limit source yet — always OK.
        self._dash_safety_rows['velocity'].set_ok(True)

        # Effort / torque proximity (warn within 10% of limit).
        torque_ok = True
        if efforts and isinstance(tlim, dict):
            t_max_raw = tlim.get('max', None)
            t_min_raw = tlim.get('min', None)
            for i, eff in enumerate(efforts):
                tmx = t_max_raw[i] if isinstance(t_max_raw, list) else t_max_raw
                tmn = t_min_raw[i] if isinstance(t_min_raw, list) else t_min_raw
                lim = max(abs(tmx) if tmx is not None else 1.0,
                          abs(tmn) if tmn is not None else 1.0, 1.0)
                if abs(eff) >= lim * 0.90:
                    torque_ok = False
                    break
        self._dash_safety_rows['torque'].set_ok(torque_ok)

    # ──────────────────────────────────────────────────────────────────────────
    #  Toolbar Quick-Toggle helpers (kept for toolbar compatibility)
    # ──────────────────────────────────────────────────────────────────────────

    def _dash_toggle_sim(self):
        """One-click start/stop: MuJoCo simulation + FIC controller."""
        if self.simulation_running or self.fic_running:
            if self.locomotion_running:
                self._stop_locomotion()
                self.log_message('INFO', 'Dashboard: Stopping locomotion...', 'Dashboard')
            if self.fic_running:
                self._stop_fic_simulation()
                self.log_message('INFO', 'Dashboard: Stopping FIC controller...', 'Dashboard')
            if self.simulation_running:
                self._stop_simulation()
                self.log_message('INFO', 'Dashboard: Stopping simulation...', 'Dashboard')
            self._update_status()
        else:
            self._start_simulation()
            self.log_message('INFO', 'Dashboard: Starting simulation...', 'Dashboard')
            QTimer.singleShot(2000, self._dash_start_fic_after_sim)

    def _dash_start_fic_after_sim(self):
        """
        Auto-start FIC two seconds after simulation launch.

        Legged robots additionally need the gait controller: FIC only
        tracks joint targets, and for a quadruped those targets come
        from the locomotion node rather than the GUI's joint sliders.
        """
        if self.simulation_running and not self.fic_running:
            self._start_fic_simulation()
            self.log_message('INFO', 'Dashboard: Starting FIC controller...', 'Dashboard')
            if self._loco_current_is_legged() and not self.locomotion_running:
                self._start_locomotion()
                self.log_message(
                    'INFO', 'Dashboard: Starting locomotion controller...', 'Dashboard')
            self._update_status()

    def _dash_toggle_hw(self):
        """One-click start/stop: robot driver + FIC hardware controller."""
        if self.driver_running or self.controller_running:
            if self.controller_running:
                self._stop_controller()
                self.log_message('INFO', 'Dashboard: Stopping FIC controller...', 'Dashboard')
            if self.driver_running:
                self._stop_driver()
                self.log_message('INFO', 'Dashboard: Stopping driver...', 'Dashboard')
            self._update_status()
        else:
            self._start_driver()
            if not self.driver_running:
                return
            self.log_message('INFO', 'Dashboard: Starting driver...', 'Dashboard')
            QTimer.singleShot(2000, self._dash_start_ctrl_after_driver)

    def _dash_start_ctrl_after_driver(self):
        """Auto-start hardware controller two seconds after driver launch."""
        if self.driver_running and not self.controller_running:
            self._start_controller()
            self.log_message('INFO', 'Dashboard: Starting FIC controller...', 'Dashboard')
            self._update_status()

    # ──────────────────────────────────────────────────────────────────────────
    #  Getting-started card (kept to avoid breaking _dash_show_getting_started
    #  calls from the Help menu; renders nothing but keeps the attribute alive)
    # ──────────────────────────────────────────────────────────────────────────

    def _dash_show_getting_started(self):
        """Re-show the getting-started card (no-op in new hub layout)."""
        self.settings.setdefault('general', {})['hide_getting_started'] = False
        try:
            self._save_settings()
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    #  Activity Tracking Hooks (called by other mixins)
    # ──────────────────────────────────────────────────────────────────────────

    def _dash_on_autotune_completed(self, score):
        """Record the last auto-tune score (hook called by autotuning mixin)."""
        pass  # the refresh timer draws the telemetry

    def _dash_scan_recent_experiments(self):
        """Scan experiments directory on startup (non-critical, no-op in hub)."""
        pass
