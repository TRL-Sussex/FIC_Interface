#!/usr/bin/env python3
"""
FIC live telemetry plotter — tabbed dark dashboard.

The plotter for the controller (simulated or real robot). It subscribes to one station's telemetry and shows it in
a tabbed PyQt5 window with an embedded matplotlib canvas per tab:

    Joints        — per-joint current vs target (planner / admittance overlays)
    Error/Effort  — per-joint tracking error + control/desired/measured effort
    Cartesian     — EE position vs desired + per-axis error + orientation error
    Wrench        — tool force / torque (when a force_torque topic is present)

Parsing is mode-aware against the controller's *verified* trajectory_data layout
(stride 8 per joint):
    Joint modes (1-4)  data[i + k*N]:  k0 planner  k1 q_current  k2 error
                                       k3 q_target k4 effort     k5 q_adm
                                       k6 desired_eff  k7 measured_eff
    Cartesian (5-8)    per-joint data[i*8+0]=q_current data[i*8+1]=q_target,
                       block at base=8N: +3..5 EE current, +6..8 EE error,
                       +9..11 EE desired, +15 orientation error angle.

Features: pause/freeze, save PNG, export CSV, rolling time window, autoscale
toggle, plus matplotlib's zoom/pan toolbar per tab.
"""
from collections import deque
import csv
import sys
import time

from geometry_msgs.msg import Wrench
import matplotlib
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas, NavigationToolbar2QT as NavToolbar)
from matplotlib.figure import Figure
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QTabWidget, QVBoxLayout, QWidget)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, Int32

# ── warm-dark palette (matches FIC Control Studio) ───────────────────────────
BG, PANEL, GRID = '#1b1714', '#232020', '#3a3330'
TEXT, MUTED = '#c9b8a8', '#7a6f66'
JOINT_COLORS = ['#d77757', '#5aa9a3', '#7bbf6a', '#5a9fd4',
                '#c97fb0', '#d8a657', '#b0884f', '#8f8f8f',
                '#cf6b5a', '#6abf9a', '#9a86c4', '#d4a05a']
C_CUR, C_TGT, C_PLAN, C_ADM = '#d77757', '#5a9fd4', '#7bbf6a', '#c97fb0'
C_ERR, C_EFF, C_DEFF, C_MEFF = '#d8a657', '#5aa9a3', '#7bbf6a', '#c97fb0'
C_X, C_Y, C_Z, C_DES = '#d77757', '#7bbf6a', '#5a9fd4', '#7a6f66'

matplotlib.rcParams.update({
    'figure.facecolor': BG, 'axes.facecolor': PANEL, 'axes.edgecolor': GRID,
    'axes.labelcolor': TEXT, 'axes.titlecolor': TEXT, 'text.color': TEXT,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'grid.color': GRID,
    'axes.grid': True, 'grid.alpha': 0.4, 'font.size': 8, 'legend.fontsize': 7,
    'lines.linewidth': 1.3, 'figure.subplot.hspace': 0.45,
})

BUFFER_MAX = 12000          # rolling history cap (~2 min at 100 Hz)
J_SIGS = ('cur', 'tgt', 'plan', 'adm', 'err', 'eff', 'deff', 'meff')
EE_KEYS = ('cx', 'cy', 'cz', 'dx', 'dy', 'dz', 'ex', 'ey', 'ez', 'oang')
WR_KEYS = ('fx', 'fy', 'fz', 'tx', 'ty', 'tz')


def _g(data, i):
    """Safe indexed read; NaN when out of range (so lines just gap)."""
    return data[i] if 0 <= i < len(data) else float('nan')


# ═════════════════════════════════════════════════════════════════════════════
#  ROS backend — subscribes + buffers (single-threaded, spun from the Qt loop)
# ═════════════════════════════════════════════════════════════════════════════
class PlotterBackend(Node):
    """Subscribes to one station's telemetry and keeps rolling buffers."""

    def __init__(self, station_id, num_joints=7):
        """Wire subscriptions for ``robot{station_id}`` and init buffers."""
        super().__init__('fic_plotter_%d' % station_id)
        self.rid = 'robot%d' % station_id
        self.n = max(1, min(12, int(num_joints)))
        self.mode = 1
        self.t0 = time.monotonic()
        self.n_changed = True
        self._alloc_joint_buffers()
        self.wt = deque(maxlen=BUFFER_MAX)
        self.wr = {k: deque(maxlen=BUFFER_MAX) for k in WR_KEYS}
        self._last_traj = 0.0
        self._last_wrench = 0.0
        self.create_subscription(
            Float64MultiArray, self.rid + '_trajectory_data', self._on_traj, 20)
        self.create_subscription(
            Int32, self.rid + '_number_of_joints', self._on_n, 10)
        self.create_subscription(
            Int32, self.rid + '_control_mode', self._on_mode, 10)
        # Wrench: same topics the studio uses; BEST_EFFORT (sensor) QoS so the
        # sim/VR force_torque publishers actually match. Harmless if absent.
        self.create_subscription(
            Wrench, '/%s_simulation/force_torque' % self.rid, self._on_wrench,
            qos_profile_sensor_data)
        self.create_subscription(
            Wrench, '/vr/force_torque', self._on_wrench, qos_profile_sensor_data)

    def _alloc_joint_buffers(self):
        self.t = deque(maxlen=BUFFER_MAX)
        self.j = {s: [deque(maxlen=BUFFER_MAX) for _ in range(self.n)]
                  for s in J_SIGS}
        self.ee = {k: deque(maxlen=BUFFER_MAX) for k in EE_KEYS}

    def _on_n(self, msg):
        if msg.data and 1 <= msg.data <= 12 and msg.data != self.n:
            self.n = int(msg.data)
            self._alloc_joint_buffers()
            self.n_changed = True

    def _on_mode(self, msg):
        self.mode = int(msg.data)

    def _on_traj(self, msg):
        d, N = msg.data, self.n
        cart = self.mode >= 5
        self._last_traj = time.monotonic()
        self.t.append(self._last_traj - self.t0)
        for i in range(N):
            if cart:
                cur, tgt = _g(d, i * 8 + 0), _g(d, i * 8 + 1)
                plan = err = eff = adm = deff = meff = float('nan')
            else:
                cur, err = _g(d, i + 1 * N), _g(d, i + 2 * N)
                tgt, eff = _g(d, i + 3 * N), _g(d, i + 4 * N)
                plan, adm = _g(d, i + 0 * N), _g(d, i + 5 * N)
                deff, meff = _g(d, i + 6 * N), _g(d, i + 7 * N)
            self.j['cur'][i].append(cur)
            self.j['tgt'][i].append(tgt)
            self.j['plan'][i].append(plan)
            self.j['adm'][i].append(adm)
            self.j['err'][i].append(err)
            self.j['eff'][i].append(eff)
            self.j['deff'][i].append(deff)
            self.j['meff'][i].append(meff)
        base = 8 * N
        if cart and len(d) >= base + 16:
            self.ee['cx'].append(d[base + 3])
            self.ee['cy'].append(d[base + 4])
            self.ee['cz'].append(d[base + 5])
            self.ee['ex'].append(d[base + 6])
            self.ee['ey'].append(d[base + 7])
            self.ee['ez'].append(d[base + 8])
            self.ee['dx'].append(d[base + 9])
            self.ee['dy'].append(d[base + 10])
            self.ee['dz'].append(d[base + 11])
            self.ee['oang'].append(d[base + 15])
        else:
            for k in EE_KEYS:
                self.ee[k].append(float('nan'))

    def _on_wrench(self, msg):
        self._last_wrench = time.monotonic()
        self.wt.append(self._last_wrench - self.t0)
        self.wr['fx'].append(msg.force.x)
        self.wr['fy'].append(msg.force.y)
        self.wr['fz'].append(msg.force.z)
        self.wr['tx'].append(msg.torque.x)
        self.wr['ty'].append(msg.torque.y)
        self.wr['tz'].append(msg.torque.z)

    def fresh(self, stamp):
        """Return True if a stream produced data within the last second."""
        return (time.monotonic() - stamp) < 1.0


# ═════════════════════════════════════════════════════════════════════════════
#  Tab base + concrete tabs
# ═════════════════════════════════════════════════════════════════════════════
class _Tab(QWidget):
    """A canvas + nav toolbar; subclasses build axes/lines and refresh data."""

    def __init__(self):
        """Create the embedded figure, canvas and matplotlib nav toolbar."""
        super().__init__()
        self.fig = Figure(figsize=(8, 6), tight_layout=True)
        self.canvas = FigureCanvas(self.fig)
        nav = NavToolbar(self.canvas, self)
        nav.setStyleSheet('background:%s; color:%s;' % (PANEL, TEXT))
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.canvas, 1)
        lay.addWidget(nav)

    @staticmethod
    def _legend(ax):
        leg = ax.legend(loc='upper right', framealpha=0.25, ncol=2)
        if leg:
            leg.get_frame().set_facecolor(BG)
            for txt in leg.get_texts():
                txt.set_color(TEXT)

    def _apply_window(self, ax, t, window, autoscale):
        if t:
            now = t[-1]
            ax.set_xlim(max(0.0, now - window), max(window, now))
        if autoscale:
            ax.relim(visible_only=True)
            ax.autoscale_view(scalex=False, scaley=True)


class JointsTab(_Tab):
    """Per-joint current vs target (with planner / admittance overlays)."""

    def rebuild(self, n):
        """(Re)create one subplot per joint for an ``n``-joint robot."""
        self.fig.clear()
        self.axes, self.lines = [], []
        rows = (n + 1) // 2
        for i in range(n):
            ax = self.fig.add_subplot(rows, 2, i + 1)
            ax.set_title('Joint %d' % (i + 1), fontsize=8)
            c = JOINT_COLORS[i % len(JOINT_COLORS)]
            ln = {
                'cur': ax.plot([], [], color=c, label='current')[0],
                'tgt': ax.plot([], [], color=C_TGT, ls='--', label='target')[0],
                'plan': ax.plot([], [], color=C_PLAN, ls=':', label='planner')[0],
                'adm': ax.plot([], [], color=C_ADM, ls='-.', label='adm')[0],
            }
            self._legend(ax)
            self.axes.append(ax)
            self.lines.append(ln)

    def refresh(self, b, window, autoscale):
        """Push the latest joint buffers into the lines + rescale."""
        t = list(b.t)
        for i, ln in enumerate(self.lines):
            ln['cur'].set_data(t, list(b.j['cur'][i]))
            ln['tgt'].set_data(t, list(b.j['tgt'][i]))
            ln['plan'].set_data(t, list(b.j['plan'][i]))
            ln['adm'].set_data(t, list(b.j['adm'][i]))
            self._apply_window(self.axes[i], t, window, autoscale)
        self.canvas.draw_idle()


class ErrorEffortTab(_Tab):
    """Per-joint tracking error (left axis) + efforts (right axis)."""

    def rebuild(self, n):
        """(Re)create one twin-axis subplot per joint."""
        self.fig.clear()
        self.axes, self.twins, self.lines = [], [], []
        rows = (n + 1) // 2
        for i in range(n):
            ax = self.fig.add_subplot(rows, 2, i + 1)
            ax.set_title('Joint %d' % (i + 1), fontsize=8)
            tw = ax.twinx()
            tw.grid(False)
            tw.tick_params(colors=MUTED)
            ln = {
                'err': ax.plot([], [], color=C_ERR, label='error')[0],
                'eff': tw.plot([], [], color=C_EFF, ls='--', label='effort')[0],
                'deff': tw.plot([], [], color=C_DEFF, ls=':', label='des-eff')[0],
                'meff': tw.plot([], [], color=C_MEFF, ls='-.', label='meas-eff')[0],
            }
            self._legend(ax)
            self.axes.append(ax)
            self.twins.append(tw)
            self.lines.append(ln)

    def refresh(self, b, window, autoscale):
        """Push error/effort buffers; rescale both axes."""
        t = list(b.t)
        for i, ln in enumerate(self.lines):
            ln['err'].set_data(t, list(b.j['err'][i]))
            ln['eff'].set_data(t, list(b.j['eff'][i]))
            ln['deff'].set_data(t, list(b.j['deff'][i]))
            ln['meff'].set_data(t, list(b.j['meff'][i]))
            self._apply_window(self.axes[i], t, window, autoscale)
            if autoscale:
                self.twins[i].relim(visible_only=True)
                self.twins[i].autoscale_view(scalex=False, scaley=True)
            self.twins[i].set_xlim(self.axes[i].get_xlim())
        self.canvas.draw_idle()


class CartesianTab(_Tab):
    """EE position vs desired per axis + orientation error angle."""

    def rebuild(self, _n=0):
        """Create the fixed X / Y / Z / orientation subplots."""
        self.fig.clear()
        self.axes, self.lines = [], []
        specs = [('X', 'cx', 'dx', C_X), ('Y', 'cy', 'dy', C_Y),
                 ('Z', 'cz', 'dz', C_Z)]
        for r, (name, cur, des, col) in enumerate(specs):
            ax = self.fig.add_subplot(4, 1, r + 1)
            ax.set_title('EE %s (m)' % name, fontsize=8)
            self.lines.append({
                'cur': ax.plot([], [], color=col, label='current')[0],
                'des': ax.plot([], [], color=C_DES, ls='--', label='desired')[0],
                'key_cur': cur, 'key_des': des,
            })
            self._legend(ax)
            self.axes.append(ax)
        axo = self.fig.add_subplot(4, 1, 4)
        axo.set_title('Orientation error (rad)', fontsize=8)
        self.lines.append({'cur': axo.plot([], [], color=C_ADM)[0],
                           'des': None, 'key_cur': 'oang', 'key_des': None})
        self.axes.append(axo)

    def refresh(self, b, window, autoscale):
        """Push EE current/desired + orientation buffers."""
        t = list(b.t)
        for i, ln in enumerate(self.lines):
            ln['cur'].set_data(t, list(b.ee[ln['key_cur']]))
            if ln['des'] is not None:
                ln['des'].set_data(t, list(b.ee[ln['key_des']]))
            self._apply_window(self.axes[i], t, window, autoscale)
        self.canvas.draw_idle()


class WrenchTab(_Tab):
    """Tool force (top) and torque (bottom)."""

    def rebuild(self, _n=0):
        """Create the force and torque subplots."""
        self.fig.clear()
        self.ax_f = self.fig.add_subplot(2, 1, 1)
        self.ax_f.set_title('Force (N)', fontsize=8)
        self.ax_t = self.fig.add_subplot(2, 1, 2)
        self.ax_t.set_title('Torque (N·m)', fontsize=8)
        self.lines = {
            'fx': self.ax_f.plot([], [], color=C_X, label='Fx')[0],
            'fy': self.ax_f.plot([], [], color=C_Y, label='Fy')[0],
            'fz': self.ax_f.plot([], [], color=C_Z, label='Fz')[0],
            'tx': self.ax_t.plot([], [], color=C_X, label='Tx')[0],
            'ty': self.ax_t.plot([], [], color=C_Y, label='Ty')[0],
            'tz': self.ax_t.plot([], [], color=C_Z, label='Tz')[0],
        }
        self._legend(self.ax_f)
        self._legend(self.ax_t)
        self.axes = [self.ax_f, self.ax_t]

    def refresh(self, b, window, autoscale):
        """Push wrench buffers (own time base)."""
        t = list(b.wt)
        for k, ln in self.lines.items():
            ln.set_data(t, list(b.wr[k]))
        for ax in self.axes:
            self._apply_window(ax, t, window, autoscale)
        self.canvas.draw_idle()


# ═════════════════════════════════════════════════════════════════════════════
#  Main window
# ═════════════════════════════════════════════════════════════════════════════
class PlotterWindow(QMainWindow):
    """Tabbed dashboard with pause / save / CSV / time-window / autoscale."""

    def __init__(self, backend):
        """Build the toolbar, tabs and the render timer for ``backend``."""
        super().__init__()
        self.b = backend
        self.paused = False
        self._cur_n = 0
        self.setWindowTitle('FIC Plotter — %s' % backend.rid)
        self.resize(1100, 780)
        self.setStyleSheet(
            'QMainWindow,QWidget{background:%s;color:%s;}'
            'QPushButton{background:%s;border:1px solid %s;border-radius:4px;'
            'padding:5px 12px;color:%s;}QPushButton:checked{background:%s;'
            'color:#1b1714;}QComboBox{background:%s;border:1px solid %s;'
            'border-radius:4px;padding:3px 8px;}'
            % (BG, TEXT, PANEL, GRID, TEXT, C_CUR, PANEL, GRID))

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        bar = QHBoxLayout()
        self.pause_btn = QPushButton('‖  Pause')
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._on_pause)
        png_btn = QPushButton('Save PNG')
        png_btn.clicked.connect(self._save_png)
        csv_btn = QPushButton('⤓  Export CSV')
        csv_btn.clicked.connect(self._export_csv)
        bar.addWidget(self.pause_btn)
        bar.addWidget(png_btn)
        bar.addWidget(csv_btn)
        bar.addStretch()
        bar.addWidget(QLabel('Window:'))
        self.window_combo = QComboBox()
        for s in ('5 s', '10 s', '30 s', '60 s'):
            self.window_combo.addItem(s)
        self.window_combo.setCurrentIndex(1)
        bar.addWidget(self.window_combo)
        self.autoscale_cb = QCheckBox('Autoscale')
        self.autoscale_cb.setChecked(True)
        bar.addWidget(self.autoscale_cb)
        self.status = QLabel('waiting for telemetry…')
        self.status.setStyleSheet('color:%s;' % MUTED)
        bar.addWidget(self.status)
        root.addLayout(bar)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(
            'QTabBar::tab{background:%s;color:%s;padding:6px 14px;}'
            'QTabBar::tab:selected{background:%s;color:#1b1714;}'
            % (PANEL, TEXT, C_CUR))
        self.joints = JointsTab()
        self.erreff = ErrorEffortTab()
        self.cart = CartesianTab()
        self.wrench = WrenchTab()
        self.tabs.addTab(self.joints, 'Joints')
        self.tabs.addTab(self.erreff, 'Error / Effort')
        self.tabs.addTab(self.cart, 'Cartesian')
        self.tabs.addTab(self.wrench, 'Wrench')
        root.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self.cart.rebuild()
        self.wrench.rebuild()

        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self._render)
        self.render_timer.start(60)   # ~16 Hz redraw of the visible tab

    def _window_s(self):
        return [5, 10, 30, 60][self.window_combo.currentIndex()]

    def _on_pause(self, on):
        self.paused = on
        self.pause_btn.setText('▶  Resume' if on else '‖  Pause')

    def _render(self):
        if self.b.n_changed or self.b.n != self._cur_n:
            self.joints.rebuild(self.b.n)
            self.erreff.rebuild(self.b.n)
            self._cur_n = self.b.n
            self.b.n_changed = False
        live = self.b.fresh(self.b._last_traj)
        kind = 'Cartesian' if self.b.mode >= 5 else 'Joint'
        self.status.setText(
            '%s · mode %d (%s) · %d joints%s'
            % ('● live' if live else '○ no data', self.b.mode, kind, self.b.n,
               ' · PAUSED' if self.paused else ''))
        if self.paused:
            return
        win = self._window_s()
        auto = self.autoscale_cb.isChecked()
        tab = self.tabs.currentWidget()
        if hasattr(tab, 'refresh'):
            tab.refresh(self.b, win, auto)

    def _save_png(self):
        tab = self.tabs.currentWidget()
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save plot', 'fic_plot.png', 'PNG (*.png)')
        if path:
            tab.fig.savefig(path, facecolor=BG, dpi=140)
            self.status.setText('saved %s' % path)

    def _export_csv(self):
        path, _ = QFileDialog.getSaveFileName(
            self, 'Export CSV', 'fic_telemetry.csv', 'CSV (*.csv)')
        if not path:
            return
        t = list(self.b.t)
        cols = [('t', t)]
        for s in J_SIGS:
            for i in range(self.b.n):
                cols.append(('j%d_%s' % (i + 1, s), list(self.b.j[s][i])))
        for k in EE_KEYS:
            cols.append(('ee_%s' % k, list(self.b.ee[k])))
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([c[0] for c in cols])
            for r in range(len(t)):
                w.writerow([(c[1][r] if r < len(c[1]) else '') for c in cols])
        self.status.setText('exported %d rows → %s' % (len(t), path))


def main():
    """Launch the plotter window for the configured station."""
    rclpy.init(args=sys.argv)
    app = QApplication([])
    probe = rclpy.create_node('fic_plotter_probe')
    probe.declare_parameter('station_id', 1)
    probe.declare_parameter('num_joints', 7)
    station_id = probe.get_parameter('station_id').value
    num_joints = probe.get_parameter('num_joints').value
    probe.destroy_node()

    backend = PlotterBackend(station_id, num_joints)
    win = PlotterWindow(backend)
    # macOS: a GUI process launched from a script (not an .app bundle) is created
    # as a background/"accessory" app, so its window never comes on screen — it
    # looks like "no window appeared". Promote the process to a regular foreground
    # app (Dock icon + on-screen window) via AppKit, then bring it to the front.
    if sys.platform == 'darwin':
        try:
            from AppKit import NSApplication, NSApplicationActivationPolicyRegular
            _nsapp = NSApplication.sharedApplication()
            _nsapp.setActivationPolicy_(NSApplicationActivationPolicyRegular)
        except Exception:
            _nsapp = None

    win.show()
    win.raise_()
    win.activateWindow()
    if sys.platform == 'darwin':
        try:
            if _nsapp is not None:
                _nsapp.activateIgnoringOtherApps_(True)
        except Exception:
            pass

    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(backend, timeout_sec=0))
    spin.start(5)

    try:
        app.exec_()
    finally:
        backend.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
