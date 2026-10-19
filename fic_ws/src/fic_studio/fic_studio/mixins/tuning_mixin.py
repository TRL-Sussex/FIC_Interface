# mixins/tuning_mixin.py — Tuning panel UI with embedded draggable FIC force profile
# Supports all 8 control modes with separate parameter sets
# v2.0 — Draggable force-profile canvas (QPainter) replaces static matplotlib plot

import math
from collections import deque

import numpy as np

from fic_studio import theme
from fic_studio.components import BreakpointStrip, CollapsibleSection, MetricField
from fic_studio.pages import page_showing
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPainterPath, QCursor
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QGroupBox, QScrollArea, QFrame, QTextEdit,
    QMessageBox, QSizePolicy, QSpinBox, QCheckBox, QListView, QInputDialog,
)

# ─────────────────────────────────────────────────────────────────────────────
#  Control mode definitions
# ─────────────────────────────────────────────────────────────────────────────

CONTROL_MODES = {
    1: {"name": "Joint Impedance",              "type": "joint",     "has_planner": False, "has_admittance": False},
    2: {"name": "Joint Impedance with Planner", "type": "joint",     "has_planner": True,  "has_admittance": False},
    3: {"name": "Joint Admittance",             "type": "joint",     "has_planner": True,  "has_admittance": True},
    4: {"name": "Joint HAP-FIC",                "type": "joint",     "has_planner": True,  "has_admittance": True},
    5: {"name": "Cartesian Impedance",          "type": "cartesian", "has_planner": False, "has_admittance": False},
    6: {"name": "Cartesian Impedance with Planner","type": "cartesian", "has_planner": True,  "has_admittance": False},
    7: {"name": "Cartesian Admittance",         "type": "cartesian", "has_planner": True,  "has_admittance": True},
    8: {"name": "Cartesian HAP-FIC",            "type": "cartesian", "has_planner": True,  "has_admittance": True},
}

#: H-PMP's own tuning (xml_manager.HPMP_MODE): not a control mode, it is used instead of the mode's
#: own tuning whenever H-PMP is on in modes 5-8. Its joint layer is a joint admittance with a planner,
#: its Cartesian layer a Cartesian planner (no admittance).
HPMP_MODE = 100


# ═══════════════════════════════════════════════════════════════════════════════
#  FIC MATH  (mirrors FIC.cpp exactly)
# ═══════════════════════════════════════════════════════════════════════════════

def _tanh_interp(S, F_sat, F_0, xb, xb0, x):
    """Smooth tanh transition between two breakpoints (from getEffortValue in FIC.cpp)."""
    denom = (1.0 - S) * (xb - xb0)
    if abs(denom) < 1e-12:
        return F_sat
    arg = (2.0 * math.pi * (x - (S * (xb - xb0) + xb0))) / denom - math.pi
    return (F_sat - F_0) / 2.0 * (1.0 + math.tanh(arg)) + F_0


def fic_force_multi(x, k0, xbs, fs, srs):
    """
    Compute FIC nonlinear force profile for arbitrary breakpoints.
    Mirrors getEffort() from FIC.cpp with N breakpoints (symmetric positive half).

    xbs: [xb1, xb2, ...] — saturation boundary positions (positive side, ascending)
    fs:  [f1,  f2,  ...] — saturation force levels (ascending)
    srs: [sr1, sr2, ...]  — saturation ratios per segment (or single value broadcast)

    Zones (positive side) for each consecutive pair:
        [0,          sr*xb[0]]   — linear: F = k0 * x
        [sr*xb[i-1], xb[i-1]]   — tanh transition to fs[i-1]
        [xb[i-1],    sr*xb[i]]  — flat bridge at fs[i-1]
        [sr*xb[i],   xb[i]]     — tanh transition to fs[i]
        [xb[-1],     inf]        — saturated at fs[-1]
    """
    n = len(xbs)
    if n == 0:
        return k0 * x

    # Broadcast single sr to all segments
    if len(srs) == 1:
        srs = srs * n
    # Pad srs if shorter than xbs
    while len(srs) < n:
        srs = list(srs) + [srs[-1]]

    ax = abs(x)
    sg = 1.0 if x >= 0 else -1.0

    # Linear zone
    if ax <= srs[0] * xbs[0]:
        return k0 * x

    # First tanh transition: linear → fs[0]
    if ax <= xbs[0]:
        return sg * _tanh_interp(srs[0], fs[0], k0 * srs[0] * xbs[0], xbs[0], srs[0] * xbs[0], ax)

    # Subsequent breakpoints
    for i in range(1, n):
        sr_i = srs[i]
        # Bridge zone: flat at fs[i-1]
        if ax <= sr_i * xbs[i]:
            return sg * fs[i - 1]
        # Tanh transition: fs[i-1] → fs[i]
        if ax <= xbs[i]:
            return sg * _tanh_interp(sr_i, fs[i], fs[i - 1], xbs[i], sr_i * xbs[i], ax)

    # Beyond last breakpoint: saturated
    return sg * fs[-1]


def convergence_force(x, error_max, effort_max):
    """
    Convergence phase: K_conv = |effort_max| / |error_max/2|, centred pullback.
    Mirrors the convergence branch of FIC::control().
    """
    e_mid = error_max / 2.0
    k_conv = abs(effort_max) / max(abs(e_mid), 1e-5)
    return k_conv * (x - e_mid)


# ═══════════════════════════════════════════════════════════════════════════════
#  DRAGGABLE FORCE PROFILE CANVAS
# ═══════════════════════════════════════════════════════════════════════════════

class FICCanvas(QWidget):
    """
    Interactive QPainter canvas showing the FIC force profile with N draggable
    breakpoints. Supports arbitrary saturation counts (N_S >= 2).

    Handles per breakpoint i:
        xb_i / f_i  — saturation boundary position & force
        sr_i        — saturation ratio (via drag on sr·xb_i)

    Scroll wheel adjusts k0.
    Emits params_changed() when any parameter changes.
    """

    # Signal carries no args — caller reads properties directly
    params_changed = pyqtSignal()

    PAD_L, PAD_R, PAD_T, PAD_B = 56, 16, 18, 38
    TRAIL_LEN = 24                      # ~2.4 s of operating-point history at 100 ms
    HANDLE_R = 7
    SCROLL_STEP = 10.0

    # Colours are theme-derived on every paint — see _apply_theme_colors().

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self._apply_theme_colors()
        theme.signals.theme_changed.connect(self.update)

        # FIC parameters — lists for N breakpoints
        self.k0  = 600.0
        self.xbs = [0.05, 0.10]      # saturation boundary positions
        self.fs  = [5.0, 39.0]       # saturation force levels
        self.srs = [0.9]             # saturation ratios (broadcast if len==1)

        self._drag_handle = None
        self._hov_handle  = None
        self._op_point = None           # (displacement, force) or None
        self._op_trail = deque(maxlen=self.TRAIL_LEN)

    # ── public API ──────────────────────────────────────────────────────────

    def set_params(self, k0, xbs, fs, srs):
        """Set all parameters. xbs, fs, srs are lists."""
        self.k0  = float(k0)
        self.xbs = [float(v) for v in xbs]
        self.fs  = [float(v) for v in fs]
        self.srs = [float(v) for v in srs]
        self.update()

    def set_operating_point(self, x, y, valid):
        """Set the live operating point (displacement x, force y).

        When ``valid`` is False (telemetry absent) the marker and trail are
        cleared so nothing is drawn while the controller is idle.
        """
        if not valid:
            self._op_point = None
            self._op_trail.clear()
        else:
            self._op_point = (float(x), float(y))
            self._op_trail.append(self._op_point)
        self.update()

    def clear_operating_point(self):
        """Drop the operating point and its trail (e.g. on joint/axis change)."""
        self._op_point = None
        self._op_trail.clear()
        self.update()

    def _n(self):
        """Number of breakpoints."""
        return len(self.xbs)

    def _effective_srs(self):
        """Return srs broadcast to length N."""
        n = self._n()
        if len(self.srs) == 1:
            return self.srs * n
        out = list(self.srs)
        while len(out) < n:
            out.append(out[-1])
        return out[:n]

    # ── coordinate mapping ───────────────────────────────────────────────────

    def _x_range(self):
        return (self.xbs[-1] if self.xbs else 0.1) * 2.8

    def _y_range(self):
        return (self.fs[-1] if self.fs else 1.0) * 1.3

    def _to_px(self, vx, vy):
        W, H = self.width(), self.height()
        xl, xr = self.PAD_L, W - self.PAD_R
        yt, yb = self.PAD_T, H - self.PAD_B
        xr_v = self._x_range(); yr_v = self._y_range()
        px = xl + (vx + xr_v) / (2 * xr_v) * (xr - xl)
        py = yb - (vy + yr_v) / (2 * yr_v) * (yb - yt)
        return QPointF(px, py)

    def _from_px(self, px, py):
        W, H = self.width(), self.height()
        xl, xr = self.PAD_L, W - self.PAD_R
        yt, yb = self.PAD_T, H - self.PAD_B
        xr_v = self._x_range(); yr_v = self._y_range()
        vx = (px - xl) / (xr - xl) * 2 * xr_v - xr_v
        vy = (yb - py) / (yb - yt) * 2 * yr_v - yr_v
        return vx, vy

    # ── handles definition ───────────────────────────────────────────────────

    def _xb_color(self, i):
        return self._XB_COLORS[i % len(self._XB_COLORS)]

    def _handles(self):
        """Return list of (id_str, world_x, world_y, QColor) for each handle."""
        handles = []
        srs = self._effective_srs()
        for i in range(self._n()):
            col = self._xb_color(i)
            handles.append((f"xb{i}", self.xbs[i], self.fs[i], col))
            # sr handle: at sr*xb position, force = linear(sr*xb) for first, else fs[i-1]
            sr_x = srs[i] * self.xbs[i]
            if i == 0:
                sr_y = self.k0 * sr_x
            else:
                sr_y = self.fs[i - 1]
            handles.append((f"sr{i}", sr_x, sr_y, self.COL_H_SR))
        return handles

    def _hit_handle(self, px, py):
        for idx, (hid, wx, wy, col) in enumerate(self._handles()):
            for sx in (wx, -wx):
                sy = wy
                pt = self._to_px(sx, sy)
                if math.hypot(pt.x() - px, pt.y() - py) <= self.HANDLE_R + 4:
                    return idx
        return None

    # ── drag logic ───────────────────────────────────────────────────────────

    def _apply_drag(self, px, py):
        vx, vy = self._from_px(px, py)
        vx = abs(vx); vy = abs(vy)
        handle = self._handles()[self._drag_handle]
        hid = handle[0]
        n = self._n()
        srs = self._effective_srs()

        if hid.startswith("xb"):
            i = int(hid[2:])
            # Constrain: xbs[i-1] < xbs[i] < xbs[i+1]
            lo = self.xbs[i - 1] * 1.05 if i > 0 else 0.001
            hi = self.xbs[i + 1] * 0.95 if i < n - 1 else self._x_range() * 0.92
            self.xbs[i] = max(lo, min(vx, hi))
            # Force: fs[i-1] < fs[i] < fs[i+1]
            f_lo = self.fs[i - 1] * 1.02 if i > 0 else 0.1
            f_hi = self.fs[i + 1] * 0.98 if i < n - 1 else 500.0
            self.fs[i] = max(f_lo, min(vy, f_hi))
        elif hid.startswith("sr"):
            i = int(hid[2:])
            ref_xb = self.xbs[i]
            new_sr = max(0.4, min(vx / max(ref_xb, 1e-6), 0.95))
            # Update the specific sr or broadcast
            if len(self.srs) == 1:
                self.srs = [new_sr]
            else:
                while len(self.srs) <= i:
                    self.srs.append(self.srs[-1])
                self.srs[i] = new_sr

        self.params_changed.emit()
        self.update()

    # ── Qt events ────────────────────────────────────────────────────────────

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            idx = self._hit_handle(ev.x(), ev.y())
            if idx is not None:
                self._drag_handle = idx
                self.setCursor(QCursor(Qt.ClosedHandCursor))

    def mouseMoveEvent(self, ev):
        if self._drag_handle is not None:
            self._apply_drag(ev.x(), ev.y())
        else:
            idx = self._hit_handle(ev.x(), ev.y())
            self._hov_handle = idx
            self.setCursor(QCursor(Qt.OpenHandCursor if idx is not None else Qt.CrossCursor))
            self.update()

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._drag_handle = None
            self.setCursor(QCursor(Qt.CrossCursor))

    def wheelEvent(self, ev):
        delta = ev.angleDelta().y()
        step = self.SCROLL_STEP * (5 if ev.modifiers() & Qt.ShiftModifier else 1)
        self.k0 = max(1.0, min(self.k0 + (step if delta > 0 else -step), 5000.0))
        self.params_changed.emit()
        self.update()

    # ── painting ─────────────────────────────────────────────────────────────

    def _apply_theme_colors(self):
        """Rebuild the canvas colours from the active theme tokens."""
        from fic_studio import theme
        t = theme.current()
        self.COL_BG = QColor(t['bg_base'])
        self.COL_TICK = QColor(t['text_secondary'])
        self.COL_CURVE = QColor(t['accent_blue'])
        self.COL_LINEAR = QColor(t['accent_teal'])
        self.COL_CONV = QColor(t['accent_red'])
        self.COL_H_SR = QColor(t['accent_red'])
        grid = QColor(t['text_primary'])
        grid.setAlpha(18)
        self.COL_GRID = grid
        axis = QColor(t['text_primary'])
        axis.setAlpha(55)
        self.COL_AXIS = axis
        sat = QColor(t['accent_orange'])
        sat.setAlpha(160)
        self.COL_SAT_LINE = sat
        srl = QColor(t['accent_red'])
        srl.setAlpha(140)
        self.COL_SR_LINE = srl
        self.COL_OP = QColor(t['accent_blue'])
        self.COL_OP_OUTLINE = QColor(t['bg_base'])
        self.COL_OP_EDGE = QColor(t['accent_red'])
        self.COL_DEV_NEAR = QColor(t['text_disabled'])
        self.COL_DEV_FAR = QColor(t['accent_red'])
        self._XB_COLORS = [
            QColor(t['accent_blue']), QColor(t['accent_orange']),
            QColor(t['accent_green']), QColor(t['accent_orange']),
        ]

    def paintEvent(self, _ev):
        self._apply_theme_colors()
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()

        qp.fillRect(0, 0, W, H, self.COL_BG)

        xl, xr = self.PAD_L, W - self.PAD_R
        yt, yb = self.PAD_T, H - self.PAD_B
        xr_v = self._x_range()
        yr_v = self._y_range()
        n = self._n()
        srs = self._effective_srs()

        # ── grid ──
        qp.setPen(QPen(self.COL_GRID, 0.5))
        for i in range(5):
            t = i / 4
            gx = xl + t * (xr - xl)
            gy = yt + t * (yb - yt)
            qp.drawLine(int(gx), yt, int(gx), yb)
            qp.drawLine(xl, int(gy), xr, int(gy))

        # ── axes ──
        qp.setPen(QPen(self.COL_AXIS, 1.0))
        zero_px = self._to_px(0, 0)
        qp.drawLine(int(zero_px.x()), yt, int(zero_px.x()), yb)
        qp.drawLine(xl, int(zero_px.y()), xr, int(zero_px.y()))

        # ── axis ticks & labels ──
        font = QFont("Monospace", 8)
        qp.setFont(font)
        qp.setPen(QPen(self.COL_TICK, 1.0))
        for i in range(-4, 5):
            vx = i * xr_v / 4.0
            pt = self._to_px(vx, 0)
            if abs(vx) > 1e-4:
                qp.drawText(QRectF(pt.x() - 24, yb + 4, 48, 16),
                            Qt.AlignCenter, f"{vx:.3f}")
        for i in range(-4, 5):
            vy = i * yr_v / 4.0
            pt = self._to_px(0, vy)
            if abs(vy) > 0.01:
                qp.drawText(QRectF(2, pt.y() - 8, xl - 6, 16),
                            Qt.AlignRight | Qt.AlignVCenter, f"{vy:.1f}")

        qp.save()
        qp.translate(11, (yt + yb) / 2)
        qp.rotate(-90)
        qp.drawText(QRectF(-50, -8, 100, 16), Qt.AlignCenter, "effort")
        qp.restore()
        qp.drawText(QRectF(xl, yb + 20, xr - xl, 16), Qt.AlignCenter, "error  (rad / m)")

        # ── saturation boundary verticals ──
        for i in range(n):
            for vx, col in [
                (self.xbs[i],           self.COL_SAT_LINE),
                (srs[i] * self.xbs[i],  self.COL_SR_LINE),
            ]:
                pen = QPen(col, 1.0, Qt.DashLine)
                qp.setPen(pen)
                for sx in (vx, -vx):
                    pt = self._to_px(sx, 0)
                    qp.drawLine(int(pt.x()), yt, int(pt.x()), yb)

        # ── convergence phase (pink dashed) ──
        if n >= 1:
            e_max = self.xbs[-1] * 0.80
            ef_max = fic_force_multi(e_max, self.k0, self.xbs, self.fs, self.srs)
            conv_path = QPainterPath()
            N_pts = 300
            started = False
            for i in range(N_pts + 1):
                vx = -xr_v + i * 2 * xr_v / N_pts
                if abs(vx) > e_max:
                    started = False
                    continue
                vy = convergence_force(abs(vx), e_max, ef_max)
                pt = self._to_px(vx, vy)
                if not started:
                    conv_path.moveTo(pt)
                    started = True
                else:
                    conv_path.lineTo(pt)
            pen = QPen(self.COL_CONV, 2.0, Qt.DashLine)
            pen.setDashPattern([6, 4])
            qp.setPen(pen)
            qp.drawPath(conv_path)

        # ── linear stiffness reference (green dashed) ──
        f_peak = self.fs[-1] if self.fs else 1.0
        lin_path = QPainterPath()
        N_pts = 300
        for i in range(N_pts + 1):
            vx = -xr_v + i * 2 * xr_v / N_pts
            vy = min(self.k0 * abs(vx), f_peak)
            pt = self._to_px(vx, vy)
            if i == 0:
                lin_path.moveTo(pt)
            else:
                lin_path.lineTo(pt)
        pen = QPen(self.COL_LINEAR, 1.5, Qt.DashLine)
        pen.setDashPattern([8, 5])
        qp.setPen(pen)
        qp.drawPath(lin_path)

        # ── main FIC curve (blue solid) ──
        fic_path = QPainterPath()
        for i in range(N_pts + 1):
            vx = -xr_v + i * 2 * xr_v / N_pts
            vy = abs(fic_force_multi(vx, self.k0, self.xbs, self.fs, self.srs))
            pt = self._to_px(vx, vy)
            if i == 0:
                fic_path.moveTo(pt)
            else:
                fic_path.lineTo(pt)
        qp.setPen(QPen(self.COL_CURVE, 2.5))
        qp.drawPath(fic_path)

        # ── live operating point ──
        if self._op_point is not None:
            # trail: oldest faint -> newest bright
            tn = len(self._op_trail)
            qp.setPen(Qt.NoPen)
            for k, (tx, ty) in enumerate(self._op_trail):
                tc = QColor(self.COL_OP)
                tc.setAlpha(int(30 + 170 * (k + 1) / max(tn, 1)))
                qp.setBrush(QBrush(tc))
                qp.drawEllipse(self._to_px(tx, ty), 2.2, 2.2)

            ox, oy = self._op_point
            cx = max(-xr_v, min(ox, xr_v))
            cy = max(-yr_v, min(oy, yr_v))
            clamped = (cx != ox) or (cy != oy)
            op_px = self._to_px(cx, cy)

            # deviation connector to the designed curve at this displacement
            curve_y = abs(fic_force_multi(ox, self.k0, self.xbs, self.fs, self.srs))
            frac = min(1.0, abs(oy - curve_y) / max(yr_v, 1e-6))
            dev = QColor(
                int(self.COL_DEV_NEAR.red() + frac * (self.COL_DEV_FAR.red() - self.COL_DEV_NEAR.red())),
                int(self.COL_DEV_NEAR.green() + frac * (self.COL_DEV_FAR.green() - self.COL_DEV_NEAR.green())),
                int(self.COL_DEV_NEAR.blue() + frac * (self.COL_DEV_FAR.blue() - self.COL_DEV_NEAR.blue())))
            cv_px = self._to_px(cx, max(-yr_v, min(curve_y, yr_v)))
            qp.setPen(QPen(dev, 1.2, Qt.DotLine))
            qp.drawLine(op_px, cv_px)

            # marker dot (or red edge dot when clamped off-axis)
            if clamped:
                qp.setPen(QPen(self.COL_OP_EDGE, 1.0))
                qp.setBrush(QBrush(self.COL_OP_EDGE))
                qp.drawEllipse(op_px, 4.0, 4.0)
            else:
                ring = QColor(self.COL_OP)
                ring.setAlpha(70)
                qp.setPen(Qt.NoPen)
                qp.setBrush(QBrush(ring))
                qp.drawEllipse(op_px, 9.0, 9.0)
                qp.setPen(QPen(self.COL_OP_OUTLINE, 1.0))
                qp.setBrush(QBrush(self.COL_OP))
                qp.drawEllipse(op_px, 5.0, 5.0)

        # ── handles ──
        handles = self._handles()
        for idx, (hid, wx, wy, col) in enumerate(handles):
            active = (self._drag_handle == idx) or (self._hov_handle == idx)
            r = self.HANDLE_R + (2 if active else 0)
            fill = col if active else self.COL_BG

            for sx in (wx, -wx):
                sy = wy
                pt = self._to_px(sx, sy)
                qp.setPen(QPen(col, 2.0))
                qp.setBrush(QBrush(fill))
                qp.drawEllipse(pt, r, r)

            # label beside positive handle
            qp.setPen(QPen(self.COL_TICK, 1.0))
            qp.setFont(QFont("Monospace", 8))
            pt_pos = self._to_px(wx, wy)
            if hid.startswith("xb"):
                i = int(hid[2:])
                lbl = f"xb{i+1}\u00b7F{i+1}"
            else:
                i = int(hid[2:])
                lbl = f"S\u00b7xb{i+1}"
            qp.drawText(QRectF(pt_pos.x() + r + 3, pt_pos.y() - 8, 60, 16),
                        Qt.AlignLeft | Qt.AlignVCenter, lbl)

        # ── legend ──
        legend_items = [
            (self.COL_CURVE,    "FIC force curve",     Qt.SolidLine),
            (self.COL_LINEAR,   "Linear stiffness K\u2080", Qt.DashLine),
            (self.COL_CONV,     "Convergence phase",   Qt.DashLine),
            (self.COL_SAT_LINE, "x_b boundaries",      Qt.DashLine),
        ]
        lx = xl + 8
        ly = yt + 6
        for col, txt, style in legend_items:
            qp.setPen(QPen(col, 1.5, style))
            qp.drawLine(lx, ly + 5, lx + 18, ly + 5)
            qp.setPen(QPen(self.COL_TICK, 1.0))
            qp.setFont(QFont("Monospace", 8))
            qp.drawText(QRectF(lx + 22, ly - 2, 120, 14), Qt.AlignLeft | Qt.AlignVCenter, txt)
            ly += 16

        # ── tooltip: k0 + N_S ──
        qp.setPen(QPen(self.COL_TICK, 1.0))
        qp.setFont(QFont("Monospace", 8))
        qp.drawText(QRectF(xr - 160, yt + 4, 152, 14),
                    Qt.AlignRight, f"K\u2080={self.k0:.0f}  N_S={n*2}  scroll=\u00b1{self.SCROLL_STEP:.0f}")

        qp.end()


# ═══════════════════════════════════════════════════════════════════════════════
#  STATS BAR  (read-only metric cards below the canvas)
# ═══════════════════════════════════════════════════════════════════════════════

class _StatCard(QWidget):
    def __init__(self, label, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.setSpacing(2)
        self._lbl = QLabel(label)
        theme.bind(self._lbl,
                   lambda t: 'font-size:10px; color:%s;' % t['text_secondary'])
        self._val = QLabel('—')
        theme.bind(self._val,
                   lambda t: 'font-size:13px; font-weight:500; color:%s;' % t['text_primary'])
        lay.addWidget(self._lbl)
        lay.addWidget(self._val)
        theme.bind(self, lambda t: 'background:%s; border-radius:6px;' % t['bg_raised'])

    def set_value(self, v): self._val.setText(str(v))


class FICStatsBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 4, 0, 0)
        row.setSpacing(6)
        self._cards = {}
        for key, lbl in [("k0",  "Stiffness K\u2080"),
                          ("ns",  "N_S"),
                          ("xb",  "x_b (all)"),
                          ("fs",  "F (all)"),
                          ("sr",  "Ratio S"),
                          ("kc",  "Conv. K")]:
            c = _StatCard(lbl)
            self._cards[key] = c
            row.addWidget(c)
        theme.signals.theme_changed.connect(self.update)

    def update_stats_multi(self, k0, xbs, fs, srs):
        """Update stats from list-based parameters."""
        n = len(xbs)
        xb_last = xbs[-1] if xbs else 0.1
        e_max   = xb_last * 0.80
        ef_max  = fic_force_multi(e_max, k0, xbs, fs, srs)
        k_conv  = abs(ef_max) / max(abs(e_max / 2.0), 1e-5)
        sr_str  = ",".join(f"{s:.2f}" for s in srs)
        xb_str  = ",".join(f"{v:.4f}" for v in xbs)
        f_str   = ",".join(f"{v:.1f}" for v in fs)
        self._cards["k0"].set_value(f"{k0:.0f}")
        self._cards["ns"].set_value(f"{n * 2}")
        self._cards["xb"].set_value(xb_str)
        self._cards["fs"].set_value(f_str)
        self._cards["sr"].set_value(sr_str)
        self._cards["kc"].set_value(f"{k_conv:.1f}")


# ═══════════════════════════════════════════════════════════════════════════════
#  LIVE FEEDBACK STRIP
# ═══════════════════════════════════════════════════════════════════════════════

#: Maps Cartesian axis name to the index in the wrench tuple (Fx,Fy,Fz,Tx,Ty,Tz).
_AXIS_WRENCH_IDX = {'x': 0, 'y': 1, 'z': 2, 'q': 3}


class _TuningFeedbackStrip(QWidget):
    """Compact live-data bar showing tracking error, effort, and wrench.

    Placed under the FICCanvas banner on the Tuning page.  The owning mixin
    calls ``set_error`` / ``set_effort`` / ``set_wrench`` from a ~100 ms QTimer.
    Each metric is shown as a labelled ``_CVABar`` (a thin horizontal bar with
    a fill and a centre-zero tick).  When the value is ``None`` (controller
    idle, no subscription) the bar shows '--'.

    Uses ``theme.current()`` in paintEvent and connects ``theme.signals.theme_changed``
    so colours adapt automatically.
    """

    def __init__(self, parent=None):
        """Initialise with three bars: tracking error, effort, wrench."""
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 4)
        row.setSpacing(8)

        self._error_bar = _CVABar('Tracking error', 'rad / m')
        self._effort_bar = _CVABar('Effort', 'Nm / N')
        self._wrench_bar = _CVABar('Wrench', 'N / Nm')

        row.addWidget(self._error_bar, 1)
        row.addWidget(self._effort_bar, 1)
        row.addWidget(self._wrench_bar, 1)

        self.setFixedHeight(46)

    # ------------------------------------------------------------------
    # Public API (called from the mixin's timer tick)
    # ------------------------------------------------------------------

    def set_error(self, value, full_range):
        """Set the tracking-error bar.  ``value=None`` shows '--'."""
        self._error_bar.set_data(value, full_range)

    def set_effort(self, value, full_range):
        """Set the effort bar.  ``value=None`` shows '--'."""
        self._effort_bar.set_data(value, full_range)

    def set_wrench(self, value, full_range):
        """Set the wrench-component bar.  ``value=None`` shows '--'."""
        self._wrench_bar.set_data(value, full_range)


class _CVABar(QWidget):
    """Single labelled horizontal bar used inside ``_TuningFeedbackStrip``.

    Displays a fill from the centre (zero) to the current value fraction,
    coloured ``accent_green`` for positive, ``accent_red`` for negative.
    When ``value`` is ``None`` the bar shows '--' and is empty.

    Reads ``theme.current()`` in ``paintEvent``; connects
    ``theme.signals.theme_changed → self.update``.
    """

    def __init__(self, label, unit, parent=None):
        """Initialise the bar with a display label and unit string."""
        super().__init__(parent)
        self._label = label
        self._unit = unit
        self._value = None      # float | None
        self._full_range = 1.0  # ±full_range is the full bar extent

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(42)
        theme.signals.theme_changed.connect(self.update)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_data(self, value, full_range):
        """Update the bar value and range; triggers a repaint."""
        self._value = float(value) if value is not None else None
        self._full_range = max(float(full_range), 1e-9)
        self.update()

    # ------------------------------------------------------------------
    # Paint
    # ------------------------------------------------------------------

    def paintEvent(self, _ev):
        """Paint the bar background, fill, centre line, and text."""
        T = theme.current()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        BAR_H = 8
        bar_y = h - BAR_H - 4
        r = 4

        # Background panel
        p.setBrush(QColor(T['bg_raised']))
        p.setPen(QPen(QColor(T['border_subtle']), 1))
        p.drawRoundedRect(0, 0, w, h, 6, 6)

        # Label (top-left)
        p.setPen(QColor(T['text_secondary']))
        from PyQt5.QtGui import QFont as _QFont
        font = _QFont()
        font.setPointSize(8)
        p.setFont(font)
        p.drawText(4, 0, w - 8, bar_y, Qt.AlignLeft | Qt.AlignVCenter, self._label)

        # Bar track
        p.setBrush(QColor(T['bg_base']))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(4, bar_y, w - 8, BAR_H, r, r)

        if self._value is None:
            # Show '--' in centre
            p.setPen(QColor(T['text_disabled']))
            p.drawText(0, 0, w, h - BAR_H - 2, Qt.AlignCenter, '--')
        else:
            # Fill: centred at midpoint; positive goes right, negative left.
            mid_x = 4 + (w - 8) // 2
            frac = max(-1.0, min(1.0, self._value / self._full_range))
            fill_w = int(abs(frac) * (w - 8) / 2)
            if fill_w > 0:
                fill_color = T['accent_green'] if frac >= 0 else T['accent_red']
                p.setBrush(QColor(fill_color))
                p.setPen(Qt.NoPen)
                if frac >= 0:
                    p.drawRoundedRect(mid_x, bar_y, fill_w, BAR_H, r, r)
                else:
                    p.drawRoundedRect(mid_x - fill_w, bar_y, fill_w, BAR_H, r, r)

            # Value text
            txt = '%.3f %s' % (self._value, self._unit)
            p.setPen(QColor(T['text_primary']))
            font2 = _QFont('JetBrains Mono, Consolas, monospace')
            font2.setPointSize(9)
            font2.setBold(True)
            p.setFont(font2)
            p.drawText(4, 0, w - 8, bar_y, Qt.AlignRight | Qt.AlignVCenter, txt)

        # Centre line
        p.setPen(QPen(QColor(T['text_disabled']), 1))
        mid_x = 4 + (w - 8) // 2
        p.drawLine(mid_x, bar_y - 1, mid_x, bar_y + BAR_H + 1)

        p.end()


# ═══════════════════════════════════════════════════════════════════════════════
#  TUNING MIXIN
# ═══════════════════════════════════════════════════════════════════════════════

class TuningMixin:
    """Mixin for tuning panel UI with mode-specific parameter editing and live FIC canvas."""

    #: Skip periodic work while this page is hidden.
    _page_showing = page_showing

    # ── page builder ────────────────────────────────────────────────────────

    def _build_tuning_page(self) -> QScrollArea:
        """Build the Direction-2 Tuning panel layout and return a QScrollArea."""
        self.tuning_section = QWidget()
        layout = QVBoxLayout(self.tuning_section)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(10)
        layout.setAlignment(Qt.AlignTop)

        # ── Title + action buttons ──────────────────────────────────────────
        title_row = QHBoxLayout()
        title_row.addWidget(self._page_title('Tuning Panel'))
        title_row.addStretch()
        load_btn = self._btn('LOAD FROM XML', 'BtnPrimary')
        load_btn.clicked.connect(self._load_tuning_parameters)
        title_row.addWidget(load_btn)
        save_btn = self._btn('SAVE TO XML', 'BtnWarn')
        save_btn.clicked.connect(self._save_tuning_parameters)
        title_row.addWidget(save_btn)
        view_xml_btn = self._btn('VIEW XML')
        view_xml_btn.setToolTip('View and edit raw config.xml for the selected robot')
        view_xml_btn.clicked.connect(self._open_config_xml_viewer)
        title_row.addWidget(view_xml_btn)
        layout.addLayout(title_row)

        # ── Selector row: Mode | Joint-or-Axis | Config ─────────────────────
        sel_row = QHBoxLayout()
        sel_row.setSpacing(12)

        sel_row.addWidget(QLabel('Mode:'))
        self.control_mode_dropdown = QComboBox()
        self.control_mode_dropdown.setMinimumWidth(240)
        for num, info in CONTROL_MODES.items():
            self.control_mode_dropdown.addItem(f"Mode {num}: {info['name']}", num)
        self.control_mode_dropdown.addItem('H-PMP (modes 5-8 with H-PMP on)', HPMP_MODE)
        self.control_mode_dropdown.currentIndexChanged.connect(self._on_control_mode_changed)
        sel_row.addWidget(self.control_mode_dropdown)

        self.mode_type_label = QLabel('Type: Joint')
        sel_row.addWidget(self.mode_type_label)
        self.mode_info_label = QLabel('(NO damping)')
        sel_row.addWidget(self.mode_info_label)

        self.hpmp_layer_label = QLabel('Layer:')
        sel_row.addWidget(self.hpmp_layer_label)
        self.hpmp_layer_dropdown = QComboBox()
        self.hpmp_layer_dropdown.addItem('Joint layer', 'joint')
        self.hpmp_layer_dropdown.addItem('Cartesian layer', 'cartesian')
        self.hpmp_layer_dropdown.currentIndexChanged.connect(self._on_control_mode_changed)
        sel_row.addWidget(self.hpmp_layer_dropdown)
        self.hpmp_layer_label.setVisible(False)
        self.hpmp_layer_dropdown.setVisible(False)

        sel_row.addSpacing(16)
        self.joint_id_label = QLabel('Joint ID:')
        sel_row.addWidget(self.joint_id_label)
        self.joint_id_dropdown = QComboBox()
        _init_n = self.robots.get(self.robot_dropdown.currentText(), 7) if hasattr(self, 'robots') else 7
        self.joint_id_dropdown.addItems([str(i) for i in range(_init_n)])
        self.joint_id_dropdown.currentTextChanged.connect(self._on_joint_id_changed)
        sel_row.addWidget(self.joint_id_dropdown)

        self.axis_label = QLabel('Axis:')
        sel_row.addWidget(self.axis_label)
        self.axis_dropdown = QComboBox()
        # "postural" is the null-space task applied in Cartesian modes 5-6.
        # It's robot-scoped (not per-mode, not per-profile), so selecting it
        # here just reuses the same UI to edit the shared <postural> block.
        self.axis_dropdown.addItems(['x', 'y', 'z', 'q', 'elbow', 'postural'])
        self.axis_dropdown.currentTextChanged.connect(self._on_axis_changed)
        sel_row.addWidget(self.axis_dropdown)

        sel_row.addSpacing(16)
        sel_row.addWidget(QLabel('Config:'))
        self.profile_dropdown = QComboBox()
        self.profile_dropdown.setMinimumWidth(160)
        self.profile_dropdown.currentIndexChanged.connect(self._on_profile_changed)
        sel_row.addWidget(self.profile_dropdown)

        add_cfg_btn = self._btn('+ Config', 'BtnPrimary')
        add_cfg_btn.setToolTip('Create a new named tuning config for this mode by cloning the current one')
        add_cfg_btn.clicked.connect(self._on_add_profile)
        sel_row.addWidget(add_cfg_btn)

        del_cfg_btn = self._btn('- Config', 'BtnWarn')
        del_cfg_btn.setToolTip("Delete the selected config (the 'default' config cannot be the only one removed)")
        del_cfg_btn.clicked.connect(self._on_delete_profile)
        sel_row.addWidget(del_cfg_btn)

        sel_row.addStretch()
        layout.addLayout(sel_row)
        # Re-apply only the two mode-info label stylesheets on theme switch.
        # Connecting theme_changed to _on_control_mode_changed would also call
        # _load_tuning_parameters(), silently discarding any unsaved widget edits.
        theme.signals.theme_changed.connect(lambda _: self._refresh_mode_labels_theme())

        # ── Full-width FICCanvas + FICStatsBar ─────────────────────────────
        canvas_lbl = QLabel('Force profile — drag handles · scroll = K₀')
        theme.bind(canvas_lbl,
                   lambda t: 'font-size:11px; color:%s;' % t['text_secondary'])
        layout.addWidget(canvas_lbl)

        self.fic_canvas = FICCanvas()
        self.fic_canvas.setMinimumHeight(300)
        self.fic_canvas.params_changed.connect(self._canvas_to_fields)
        layout.addWidget(self.fic_canvas)

        self.fic_stats = FICStatsBar()
        layout.addWidget(self.fic_stats)

        # ── Live feedback strip ────────────────────────────────────
        self._feedback_strip = _TuningFeedbackStrip()
        layout.addWidget(self._feedback_strip)

        self._op_readout = QLabel('')
        theme.bind(self._op_readout,
                   lambda t: 'color: %s; font-family: monospace; font-size: 11px;'
                   % t['text_secondary'])
        layout.addWidget(self._op_readout)

        # Applied-stamp label — updated by _save_tuning_parameters on success.
        self._applied_label = QLabel('')
        theme.bind(
            self._applied_label,
            lambda t: 'font-size:10px; color:%s; padding:2px 0;' % t['text_secondary'],
        )
        layout.addWidget(self._applied_label)

        # ── CONTROLLER params ───────────────────────────────────────────────
        self.controller_group = QGroupBox('CONTROLLER')
        ctrl_lay = QVBoxLayout(self.controller_group)
        ctrl_lay.setContentsMargins(10, 8, 10, 8)
        ctrl_lay.setSpacing(6)

        # Row 1: Stiffness K0
        r1 = QHBoxLayout()
        r1.addWidget(QLabel('Stiffness K₀:'))
        self.tuning_stiffness = MetricField(
            'K₀', 600.0, 0.0, 5000.0,
            unit='Nm/rad', decimals=1,
        )
        self.tuning_stiffness.committed.connect(self._fields_to_canvas)
        self.tuning_stiffness.edited.connect(self._fields_to_canvas)
        r1.addWidget(self.tuning_stiffness, 1)
        ctrl_lay.addLayout(r1)

        # Row 2: Sat Ratio
        r2 = QHBoxLayout()
        r2.addWidget(QLabel('Sat. Ratio S:'))
        self.tuning_saturation_ratios = MetricField(
            'S', 0.9, 0.4, 0.95,
            unit='', decimals=3,
        )
        self.tuning_saturation_ratios.committed.connect(self._fields_to_canvas)
        self.tuning_saturation_ratios.edited.connect(self._fields_to_canvas)
        r2.addWidget(self.tuning_saturation_ratios, 1)
        r2.addSpacing(20)
        r2.addWidget(QLabel('N_S:'))
        self.tuning_n_sat = QSpinBox()
        self.tuning_n_sat.setRange(2, 8)
        self.tuning_n_sat.setSingleStep(2)
        self.tuning_n_sat.setValue(2)
        self.tuning_n_sat.setToolTip('Total saturation points (N_S). Symmetric: positive half = N_S/2 values.')
        self.tuning_n_sat.valueChanged.connect(self._on_n_sat_changed)
        r2.addWidget(self.tuning_n_sat)
        ctrl_lay.addLayout(r2)

        # Row 3: Error breakpoints
        r3 = QHBoxLayout()
        r3.addWidget(QLabel('Error Max (breakpoints):'))
        self.tuning_error_max = BreakpointStrip(
            1, 0.0001, 3.0, unit='rad', decimals=4,
        )
        self.tuning_error_max.committed.connect(self._fields_to_canvas)
        r3.addWidget(self.tuning_error_max, 1)
        ctrl_lay.addLayout(r3)

        # Row 4: Force breakpoints
        r4 = QHBoxLayout()
        r4.addWidget(QLabel('Force Max (breakpoints):'))
        self.tuning_force_max = BreakpointStrip(
            1, 0.1, 500.0, unit='Nm', decimals=2,
        )
        self.tuning_force_max.committed.connect(self._fields_to_canvas)
        r4.addWidget(self.tuning_force_max, 1)
        ctrl_lay.addLayout(r4)

        # Row 5: Velocity (joint+planner only — shown/hidden by _apply_axis_visibility)
        r5 = QHBoxLayout()
        self.tuning_velocity_label = QLabel('Velocity:')
        r5.addWidget(self.tuning_velocity_label)
        self.tuning_velocity = MetricField(
            'vel', 0.5, 0.0, 10.0,
            unit='rad/s', decimals=4,
        )
        r5.addWidget(self.tuning_velocity, 1)
        r5.addStretch()
        ctrl_lay.addLayout(r5)

        layout.addWidget(self.controller_group)

        # ── CollapsibleSection: PLANNER ─────────────────────────────────────
        self.planner_group = CollapsibleSection('PLANNER')
        plan_lay = self.planner_group.body_layout()

        # The planner's stiffness and frequency stay in config.xml but do nothing (FIC.cpp
        # Planner derives its stiffness and does not read the frequency), so they are not shown
        # and a save keeps what is stored. The widgets remain for the code that fills them.
        pg1 = QHBoxLayout()
        stiffness_p_label = QLabel('Stiffness P:')
        pg1.addWidget(stiffness_p_label)
        self.tuning_planner_stiffness = MetricField(
            'Kp', 100.0, 0.0, 5000.0, unit='Nm/rad', decimals=1,
        )
        pg1.addWidget(self.tuning_planner_stiffness, 1)
        stiffness_p_label.setVisible(False)
        self.tuning_planner_stiffness.setVisible(False)
        pg1.addWidget(QLabel('Vel Desired:'))
        self.tuning_vel_desired = MetricField(
            'Vd', 0.5, 0.0, 10.0, unit='rad/s', decimals=4,
        )
        pg1.addWidget(self.tuning_vel_desired, 1)
        plan_lay.addLayout(pg1)

        pg2 = QHBoxLayout()
        pg2.addWidget(QLabel('Acc Max:'))
        # A cap on the planner's acceleration (0: none). The shipped 5000 is never reached; the
        # range takes it whole (a 0-50 range clamped it to 50 on save).
        self.tuning_acc_max = MetricField(
            'Am', 5000.0, 0.0, 10000.0, unit='rad/s²', decimals=1,
        )
        pg2.addWidget(self.tuning_acc_max, 1)
        pg2.addSpacing(10)
        pg2.addWidget(QLabel('Damping Ratio:'))
        self.tuning_damping_ratio = MetricField(
            'Dr', 0.7, 0.0, 2.0, unit='', decimals=6,
        )
        pg2.addWidget(self.tuning_damping_ratio, 1)
        plan_lay.addLayout(pg2)

        pg3 = QHBoxLayout()
        frequency_label = QLabel('Frequency:')
        pg3.addWidget(frequency_label)
        self.tuning_frequency = MetricField(
            'Hz', 1.0, 0.01, 100.0, unit='Hz', decimals=1,
        )
        pg3.addWidget(self.tuning_frequency, 1)
        frequency_label.setVisible(False)
        self.tuning_frequency.setVisible(False)
        self.tuning_wrapped_label = QLabel('Wrapped:')
        pg3.addWidget(self.tuning_wrapped_label)
        self.tuning_wrapped = QComboBox()
        self.tuning_wrapped.addItems(['0', '1'])
        pg3.addWidget(self.tuning_wrapped)
        plan_lay.addLayout(pg3)

        pg4 = QHBoxLayout()
        self.tuning_pos_limits_label = QLabel('Pos Limits:')
        pg4.addWidget(self.tuning_pos_limits_label)
        self.tuning_pos_limits_lower = MetricField(
            'lo', -3.14, -6.28, 0.0, unit='rad', decimals=4,
        )
        pg4.addWidget(self.tuning_pos_limits_lower, 1)
        self.tuning_pos_limits_upper = MetricField(
            'hi', 3.14, 0.0, 6.28, unit='rad', decimals=4,
        )
        pg4.addWidget(self.tuning_pos_limits_upper, 1)
        plan_lay.addLayout(pg4)

        layout.addWidget(self.planner_group)

        # ── CollapsibleSection: ADMITTANCE ──────────────────────────────────
        self.admittance_group = CollapsibleSection('ADMITTANCE')
        adm_lay = self.admittance_group.body_layout()

        ag1 = QHBoxLayout()
        ag1.addWidget(QLabel('Inv-M:'))
        self.tuning_inv_m = MetricField(
            'M⁻¹', 1.0, 0.0, 100.0, unit='', decimals=6,
        )
        ag1.addWidget(self.tuning_inv_m, 1)
        ag1.addSpacing(10)
        ag1.addWidget(QLabel('Damping:'))
        self.tuning_damping = MetricField(
            'Kv', 1.0, 0.0, 500.0, unit='', decimals=1,
        )
        ag1.addWidget(self.tuning_damping, 1)
        adm_lay.addLayout(ag1)

        ag2 = QHBoxLayout()
        ag2.addWidget(QLabel('dFlow:'))
        self.tuning_dflow_0 = MetricField(
            '[0]', 0.0, -10.0, 10.0, unit='', decimals=4,
        )
        ag2.addWidget(self.tuning_dflow_0, 1)
        self.tuning_dflow_1 = MetricField(
            '[1]', 0.0, -10.0, 10.0, unit='', decimals=4,
        )
        ag2.addWidget(self.tuning_dflow_1, 1)
        adm_lay.addLayout(ag2)

        ag3 = QHBoxLayout()
        ag3.addWidget(QLabel('Flow:'))
        self.tuning_flow_0 = MetricField(
            '[0]', 0.0, -10.0, 10.0, unit='', decimals=4,
        )
        ag3.addWidget(self.tuning_flow_0, 1)
        self.tuning_flow_1 = MetricField(
            '[1]', 0.0, -10.0, 10.0, unit='', decimals=4,
        )
        ag3.addWidget(self.tuning_flow_1, 1)
        adm_lay.addLayout(ag3)

        ag4 = QHBoxLayout()
        ag4.addWidget(QLabel('Pose:'))
        self.tuning_pose_0 = MetricField(
            '[0]', 0.0, -10.0, 10.0, unit='', decimals=4,
        )
        ag4.addWidget(self.tuning_pose_0, 1)
        self.tuning_pose_1 = MetricField(
            '[1]', 0.0, -10.0, 10.0, unit='', decimals=4,
        )
        ag4.addWidget(self.tuning_pose_1, 1)
        adm_lay.addLayout(ag4)

        layout.addWidget(self.admittance_group)

        # ── CollapsibleSection: ADVANCED ────────────────────────────────────
        adv_section = CollapsibleSection('ADVANCED')
        adv_lay = adv_section.body_layout()

        av1 = QHBoxLayout()
        av1.addWidget(QLabel('Asymmetric:'))
        self.tuning_asymmetric = QCheckBox()
        self.tuning_asymmetric.setToolTip(
            'Asymmetric: different left/right saturation shapes. '
            'Requires full N_S values in error_max/force_max.'
        )
        av1.addWidget(self.tuning_asymmetric)
        av1.addSpacing(20)
        av1.addWidget(QLabel('N_negSat:'))
        self.tuning_n_neg_sat = QSpinBox()
        self.tuning_n_neg_sat.setRange(1, 4)
        self.tuning_n_neg_sat.setValue(1)
        self.tuning_n_neg_sat.setToolTip('Number of negative-side saturation points (N_negSat)')
        self.tuning_n_neg_sat.setEnabled(False)
        self.tuning_asymmetric.toggled.connect(self.tuning_n_neg_sat.setEnabled)
        av1.addWidget(self.tuning_n_neg_sat)
        av1.addStretch()
        adv_lay.addLayout(av1)

        layout.addWidget(adv_section)

        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.tuning_section)
        scroll.setFrameShape(QFrame.NoFrame)

        # Live feedback timer — started after the page is constructed.
        self._start_feedback_timer()

        self._on_control_mode_changed()
        return scroll

    # ── bidirectional sync ───────────────────────────────────────────────────

    def _clamp_stiffness(self, k0, em, fm):
        """Enforce max stiffness = force_max[0] / error_max[0]. Returns clamped k0."""
        if em and fm and em[0] > 1e-12:
            max_k = fm[0] / em[0]
            return min(k0, max_k)
        return k0

    def _fields_to_canvas(self):
        """Push MetricField/BreakpointStrip values → canvas whenever the user edits them."""
        try:
            k0 = self.tuning_stiffness.value()
            em = self.tuning_error_max.values()
            fm = self.tuning_force_max.values()
            sr = self.tuning_saturation_ratios.value()
            if len(em) >= 1 and len(fm) >= 1 and len(em) == len(fm):
                k0 = self._clamp_stiffness(k0, em, fm)
                self.tuning_stiffness.set_value(k0)
                self.fic_canvas.set_params(k0, em, fm, [sr])
                self.fic_stats.update_stats_multi(k0, em, fm, [sr])
        except (ValueError, IndexError, ZeroDivisionError):
            pass

    def _canvas_to_fields(self):
        """Push canvas drag results → MetricField/BreakpointStrip (no re-trigger)."""
        c = self.fic_canvas
        k0 = self._clamp_stiffness(c.k0, c.xbs, c.fs)
        c.k0 = k0
        self.tuning_stiffness.set_value(k0)
        # Resize strips if the canvas breakpoint count changed
        n_half = len(c.xbs)
        self.tuning_error_max.set_count(n_half)
        self.tuning_force_max.set_count(n_half)
        self.tuning_error_max.set_values(c.xbs)
        self.tuning_force_max.set_values(c.fs)
        sr = c.srs[0] if c.srs else 0.9
        self.tuning_saturation_ratios.set_value(sr)
        self.fic_stats.update_stats_multi(k0, c.xbs, c.fs, c.srs)

    def _sync_canvas_from_fields(self):
        """Called after loading params from XML — push all fields to canvas."""
        self._fields_to_canvas()

    def _on_n_sat_changed(self, n_s):
        """Handle N_S spinner change — resize BreakpointStrips and keep n_neg_sat in sync."""
        if not self.tuning_asymmetric.isChecked():
            self.tuning_n_neg_sat.blockSignals(True)
            self.tuning_n_neg_sat.setValue(n_s // 2)
            self.tuning_n_neg_sat.blockSignals(False)

        n_half = n_s // 2  # positive-side breakpoints for symmetric
        # Grow or shrink the breakpoint strips to match the new count.
        em = self.tuning_error_max.values()
        fm = self.tuning_force_max.values()
        # Pad if growing
        while len(em) < n_half:
            em.append((em[-1] if em else 0.1) * 2)
        while len(fm) < n_half:
            fm.append((fm[-1] if fm else 10.0) * 2)
        em = em[:n_half]
        fm = fm[:n_half]
        self.tuning_error_max.set_count(n_half)
        self.tuning_force_max.set_count(n_half)
        self.tuning_error_max.set_values(em)
        self.tuning_force_max.set_values(fm)
        self._fields_to_canvas()

    # ── mode change ──────────────────────────────────────────────────────────

    def _tuning_set(self):
        """(mode, is_joint, has_planner, has_admittance) of the set being edited. H-PMP's own tuning
        is a joint set on its joint layer and a set of axes on its Cartesian layer (no admittance)."""
        mode = self._get_current_mode()
        if mode == HPMP_MODE:
            joint = self.hpmp_layer_dropdown.currentData() == 'joint'
            return mode, joint, True, joint
        info = CONTROL_MODES.get(mode, CONTROL_MODES[1])
        return mode, info['type'] == 'joint', info['has_planner'], info['has_admittance']

    def _refresh_mode_labels_theme(self):
        """Re-apply stylesheets for mode_type_label and mode_info_label.

        Lightweight — reads theme.current() and the current mode combo state only.
        No XML load, no visibility changes. Called on theme switches so unsaved
        widget edits are never discarded by a full _on_control_mode_changed().
        """
        _, is_joint, has_planner, has_admittance = self._tuning_set()
        _tok = theme.current()

        self.mode_type_label.setStyleSheet(
            'color:%s; font-weight: 600;' % (
                _tok['accent_blue'] if is_joint else _tok['accent_teal']))

        if has_planner or has_admittance:
            self.mode_info_label.setStyleSheet(
                'color:%s; font-style:italic;' % _tok['accent_teal'])
        else:
            self.mode_info_label.setStyleSheet(
                'color:%s; font-style:italic;' % _tok['accent_orange'])

    def _on_control_mode_changed(self):
        mode, is_joint, has_planner, has_admittance = self._tuning_set()
        hpmp = mode == HPMP_MODE
        self.hpmp_layer_label.setVisible(hpmp)
        self.hpmp_layer_dropdown.setVisible(hpmp)

        # Units depend on the control space (joint vs Cartesian).
        u_stiff = 'Nm/rad' if is_joint else 'N/m'
        u_force = 'Nm' if is_joint else 'N'
        u_len = 'rad' if is_joint else 'm'
        u_vel = 'rad/s' if is_joint else 'm/s'
        for _attr, _u in (('tuning_stiffness', u_stiff),
                          ('tuning_error_max', u_len),
                          ('tuning_force_max', u_force),
                          ('tuning_velocity', u_vel),
                          ('tuning_planner_stiffness', u_stiff),
                          ('tuning_vel_desired', u_vel)):
            _w = getattr(self, _attr, None)
            if _w is not None and hasattr(_w, 'set_unit'):
                _w.set_unit(_u)

        self.mode_type_label.setText('Type: %s' % (
            ('H-PMP, %s layer' % ('joint' if is_joint else 'Cartesian')) if hpmp
            else 'Joint' if is_joint else 'Cartesian'))
        if hpmp:
            self.mode_info_label.setText("(used instead of the mode's own tuning whenever H-PMP is on)")
        elif has_planner:
            self.mode_info_label.setText('(HAS damping_ratio)')
        elif has_admittance:
            self.mode_info_label.setText('(HAS Kv damping)')
        else:
            self.mode_info_label.setText('(NO damping — use lower stiffness!)')

        # Apply theme-dependent colours to the two mode info labels.
        self._refresh_mode_labels_theme()

        self.joint_id_label.setVisible(is_joint)
        self.joint_id_dropdown.setVisible(is_joint)
        self.axis_label.setVisible(not is_joint)
        self.axis_dropdown.setVisible(not is_joint)

        # If we just switched from Cartesian to Joint, "postural"/"elbow" may
        # still be selected; they're invalid for joint modes, so snap back to x.
        if is_joint and self.axis_dropdown.currentText() in ("postural", "elbow"):
            self.axis_dropdown.blockSignals(True)
            self.axis_dropdown.setCurrentText("x")
            self.axis_dropdown.blockSignals(False)

        self._apply_axis_visibility(is_joint, has_planner, has_admittance)

        # Each mode has its own profile list — refresh before loading values.
        self._refresh_profile_dropdown()
        self._load_tuning_parameters()

    def _on_joint_id_changed(self, _text=None):
        """Clear the operating-point trail, then reload tuning params."""
        if hasattr(self, 'fic_canvas'):
            self.fic_canvas.clear_operating_point()
        self._load_tuning_parameters()

    def _on_axis_changed(self, _text=None):
        """Axis dropdown changed. Refresh UI visibility (postural hides a few
        groups) and reload values."""
        if hasattr(self, 'fic_canvas'):
            self.fic_canvas.clear_operating_point()
        _, is_joint, has_planner, has_admittance = self._tuning_set()
        self._apply_axis_visibility(is_joint, has_planner, has_admittance)
        self._load_tuning_parameters()

    def _apply_axis_visibility(self, is_joint: bool, has_planner: bool, has_admittance: bool):
        """Show/hide sections based on the current (mode, axis) combination.

        CollapsibleSections are shown/hidden (mode-aware) instead of greyed-out.
        Pulled out because both mode change and axis change call this.
        """
        axis_text = self.axis_dropdown.currentText() if not is_joint else ''
        is_postural = axis_text == 'postural'
        is_elbow = axis_text == 'elbow'
        is_nullspace = is_postural or is_elbow

        # Velocity row: joint + planner modes only
        show_vel = is_joint and has_planner
        self.tuning_velocity_label.setVisible(show_vel)
        self.tuning_velocity.setVisible(show_vel)

        # Wrapped / pos-limits rows: joint + planner only (inside Planner section)
        show_wrap = is_joint and has_planner
        self.tuning_wrapped_label.setVisible(show_wrap)
        self.tuning_wrapped.setVisible(show_wrap)
        self.tuning_pos_limits_label.setVisible(show_wrap)
        self.tuning_pos_limits_lower.setVisible(show_wrap)
        self.tuning_pos_limits_upper.setVisible(show_wrap)

        # Planner section: show when the mode has a planner and axis is not null-space.
        planner_visible = has_planner and not is_nullspace
        self.planner_group.setVisible(planner_visible)

        # Admittance section: show when the mode has admittance and axis is not null-space.
        admittance_visible = has_admittance and not is_nullspace
        self.admittance_group.setVisible(admittance_visible)

    def _get_current_mode(self) -> int:
        mode = self.control_mode_dropdown.currentData()
        return mode if mode is not None else 1

    def _get_current_profile(self) -> str:
        if not hasattr(self, "profile_dropdown"):
            return "default"
        # Display text may carry a "(fallback)" suffix — use itemData for the clean name.
        data = self.profile_dropdown.currentData()
        if data:
            return data
        name = self.profile_dropdown.currentText()
        return name if name else "default"

    def _refresh_profile_dropdown(self, select: str = None):
        """Populate the profile dropdown with the robot-scoped profile union.
        Profiles span every mode: editing 'Stiff' on mode 2 creates an override
        just for that mode, and other modes fall back to 'default' at runtime.
        If `select` is given, select it; otherwise keep the current selection
        when possible, falling back to 'default'."""
        if not hasattr(self, "profile_dropdown"):
            return
        try:
            from fic_studio.xml_manager import XMLManager
            mgr = XMLManager()
            robot = self.robot_dropdown.currentText()
            mode = self._get_current_mode()
            profiles = mgr.get_all_profiles(robot) or ["default"]
            with_override = set(mgr.get_available_profiles(robot, mode))
        except Exception:
            profiles = ["default"]
            with_override = {"default"}

        prev_data = self.profile_dropdown.currentData() if select is None else None
        prev = select if select is not None else (prev_data or self.profile_dropdown.currentText())
        self.profile_dropdown.blockSignals(True)
        self.profile_dropdown.clear()
        for p in profiles:
            label = p if p in with_override else f"{p}  (fallback → default)"
            self.profile_dropdown.addItem(label, p)
        idx = self.profile_dropdown.findData(prev) if prev else -1
        if idx < 0:
            idx = self.profile_dropdown.findData("default")
        self.profile_dropdown.setCurrentIndex(max(0, idx))
        self.profile_dropdown.blockSignals(False)

    def _on_profile_changed(self):
        """User picked a different profile — publish it so the controller reloads,
        then reload the tuning fields from XML."""
        self._publish_active_profile()
        self._load_tuning_parameters()

    def _publish_param_update(self, param_type, value=''):
        """Tell the controller its tuning file changed so it reloads config.xml.

        The FIC node has no other runtime reload trigger: it used to re-read
        config.xml on every incoming joint target, which was removed because it
        reparsed a 340 KB file at the target rate.  Without this signal a SAVE
        TO XML in Joint Impedance mode would not take effect until the operator
        toggled control modes or restarted the node.
        """
        try:
            node = getattr(self, "node", None)
            publish = getattr(node, "publish_param_update", None) if node else None
            if publish is None:
                return
            publish(None, str(param_type), str(value))
        except Exception:
            pass

    def _publish_active_profile(self):
        """Publish the currently selected profile to ros_node.active_profile_publisher_."""
        try:
            from std_msgs.msg import String
            node = getattr(self, "node", None)
            pub = getattr(node, "active_profile_publisher_", None) if node else None
            if pub is None:
                return
            msg = String()
            msg.data = self._get_current_profile() or "default"
            pub.publish(msg)
        except Exception:
            pass

    def _on_add_profile(self):
        """Prompt for a new profile name, clone the current one to it, select it."""
        try:
            from fic_studio.xml_manager import XMLManager
        except ImportError as e:
            QMessageBox.critical(self, "Error", f"Failed to import xml_manager: {e}")
            return
        robot = self.robot_dropdown.currentText()
        mode = self._get_current_mode()
        src = self._get_current_profile()

        name, ok = QInputDialog.getText(self, "Add Config",
            f"New config name for {robot}:\n"
            f"Creates an override for mode {mode} (cloned from '{src}').\n"
            f"Other modes fall back to 'default' until you edit them.")
        if not ok:
            return
        name = name.strip()
        if not name:
            QMessageBox.warning(self, "Add Config", "Name cannot be empty.")
            return

        try:
            mgr = XMLManager()
            mgr.clone_profile(robot, mode, src, name)
            mgr.save()
        except Exception as e:
            QMessageBox.critical(self, "Add Config", f"Failed to create config: {e}")
            return

        self._refresh_profile_dropdown(select=name)
        if hasattr(self, "_refresh_rc_profile_dropdown"):
            self._refresh_rc_profile_dropdown()
        self._publish_active_profile()
        self._load_tuning_parameters()

    def _on_delete_profile(self):
        try:
            from fic_studio.xml_manager import XMLManager
        except ImportError as e:
            QMessageBox.critical(self, "Error", f"Failed to import xml_manager: {e}")
            return
        robot = self.robot_dropdown.currentText()
        profile = self._get_current_profile()

        if profile == "default":
            QMessageBox.information(self, "Delete Config",
                "The 'default' config cannot be deleted.")
            return

        reply = QMessageBox.question(self, "Delete Config",
            f"Delete config '{profile}' for {robot}?\n"
            f"All per-mode overrides for this config will be removed.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return

        try:
            mgr = XMLManager()
            n = mgr.delete_profile_everywhere(robot, profile)
            if n == 0:
                QMessageBox.information(self, "Delete Config", "Config not found.")
                return
            mgr.save()
        except Exception as e:
            QMessageBox.critical(self, "Delete Config", f"Failed to delete: {e}")
            return

        self._refresh_profile_dropdown()
        if hasattr(self, "_refresh_rc_profile_dropdown"):
            self._refresh_rc_profile_dropdown()
        self._publish_active_profile()
        self._load_tuning_parameters()

    def _update_tuning_joint_count(self):
        """Update the Joint ID dropdown to match the currently selected robot's joint count."""
        if not hasattr(self, 'joint_id_dropdown'):
            return
        robot_name = self.robot_dropdown.currentText()
        num_joints = self.robots.get(robot_name, 7)
        
        # Block signals to avoid spurious load calls while rebuilding
        self.joint_id_dropdown.blockSignals(True)
        current = self.joint_id_dropdown.currentText()
        self.joint_id_dropdown.clear()
        self.joint_id_dropdown.addItems([str(i) for i in range(num_joints)])
        # Restore selection if still valid, otherwise go to 0
        idx = self.joint_id_dropdown.findText(current)
        self.joint_id_dropdown.setCurrentIndex(max(0, idx))
        self.joint_id_dropdown.blockSignals(False)
        
        # Robot changed — profiles are per (robot, mode), so refresh that list too.
        self._refresh_profile_dropdown()
        # Trigger a fresh load for the (possibly new) joint 0
        self._load_tuning_parameters()
    # ── live feedback timer ──────────────────────────────────────────────────

    def _start_feedback_timer(self):
        """Create and start the ~100 ms QTimer that refreshes the feedback strip."""
        self._feedback_timer = QTimer(self.tuning_section)
        self._feedback_timer.setInterval(100)
        self._feedback_timer.timeout.connect(self._refresh_feedback_strip)
        self._feedback_timer.start()

    def _set_op_readout(self, x, y, valid):
        """Update the operating-point numeric readout (no-op if absent)."""
        lbl = getattr(self, '_op_readout', None)
        if lbl is None:
            return
        if not valid:
            lbl.setText('Δx --  ·  F --  ·  dev --')
            return
        curve_y = abs(fic_force_multi(x, self.fic_canvas.k0, self.fic_canvas.xbs,
                                      self.fic_canvas.fs, self.fic_canvas.srs))
        lbl.setText('Δx %+.4f  ·  F %+.2f  ·  dev %+.2f' % (x, y, y - curve_y))

    def _refresh_feedback_strip(self):
        """Tick handler: push live telemetry data into the feedback strip.

        Runs every ~100 ms while the tuning page is in view (stack index 2).
        Reads telemetry from the JointStateMixin helpers via self; shows '--'
        when data is absent (controller idle / no subscription yet).
        """
        if not hasattr(self, '_feedback_strip'):
            return
        # Page-visible guard: only refresh when the tuning stack page is active.
        stack = getattr(self, 'stack', None)
        if not self._page_showing('tuning'):
            return

        is_joint = self._tuning_set()[1]

        # Resolve robot id.
        rid = None
        node = getattr(self, 'node', None)
        if node is not None:
            rid = getattr(node, 'station_robot_id', None)

        if is_joint:
            # Joint mode — show tracking error and effort for the selected joint.
            joint_idx = 0
            try:
                joint_idx = int(self.joint_id_dropdown.currentText())
            except (AttributeError, ValueError):
                pass

            cmd_list = []
            pos_list = []
            eff_list = []
            try:
                cmd_list = self.joint_commanded_for(rid)
                pos_list = self.joint_positions_for(rid)
                eff_list = self.joint_effort_for(rid)
            except AttributeError:
                # Telemetry helpers not available (no JointStateMixin in host).
                pass

            if cmd_list and pos_list and joint_idx < len(cmd_list) and joint_idx < len(pos_list):
                tracking_error = cmd_list[joint_idx] - pos_list[joint_idx]
                # Range for error bar: first breakpoint of error_max, minimum 0.01.
                em = self.tuning_error_max.values() if hasattr(self, 'tuning_error_max') else []
                err_range = em[0] if em else 0.1
                err_range = max(err_range, 1e-4)
                self._feedback_strip.set_error(tracking_error, err_range)
            else:
                self._feedback_strip.set_error(None, 1.0)

            if eff_list and joint_idx < len(eff_list):
                effort = eff_list[joint_idx]
                # Range for effort bar: the top of the curve, the last force level.
                fm = self.tuning_force_max.values() if hasattr(self, 'tuning_force_max') else []
                eff_range = max(fm[-1] if fm else 39.0, 1e-4)
                self._feedback_strip.set_effort(effort, eff_range)
            else:
                self._feedback_strip.set_effort(None, 1.0)

            ok = (cmd_list and pos_list and eff_list
                  and joint_idx < len(cmd_list) and joint_idx < len(pos_list)
                  and joint_idx < len(eff_list))
            if ok:
                self.fic_canvas.set_operating_point(
                    cmd_list[joint_idx] - pos_list[joint_idx],
                    eff_list[joint_idx], True)
            else:
                self.fic_canvas.set_operating_point(0.0, 0.0, False)
            self._set_op_readout(
                cmd_list[joint_idx] - pos_list[joint_idx] if ok else 0.0,
                eff_list[joint_idx] if ok else 0.0, ok)

            # No wrench bar for joint modes.
            self._feedback_strip.set_wrench(None, 1.0)

        else:
            # Cartesian mode — show wrench component for the selected axis.
            # Hide error/effort bars for Cartesian (not per-joint).
            self._feedback_strip.set_error(None, 1.0)
            self._feedback_strip.set_effort(None, 1.0)

            axis = ''
            try:
                axis = self.axis_dropdown.currentText()
            except AttributeError:
                pass

            # Map axis name → wrench tuple index: x→0, y→1, z→2, q→3.
            # elbow/postural have no meaningful wrench component → show '--'.
            _axis_wrench_idx = {'x': 0, 'y': 1, 'z': 2, 'q': 3}
            w_idx = _axis_wrench_idx.get(axis)

            wrench = None
            try:
                wrench = self.wrench_for(rid)
            except AttributeError:
                pass

            if wrench is not None and w_idx is not None and w_idx < len(wrench):
                component = wrench[w_idx]
                # Range: last element of force_max.
                fm = self.tuning_force_max.values() if hasattr(self, 'tuning_force_max') else []
                f_range = fm[-1] if fm else 39.0
                f_range = max(f_range, 1e-4)
                self._feedback_strip.set_wrench(component, f_range)
            else:
                self._feedback_strip.set_wrench(None, 1.0)

            _axis_err_idx = {'x': 0, 'y': 1, 'z': 2, 'q': 3}
            ai = _axis_err_idx.get(axis)
            cerr = None
            try:
                cerr = self.cartesian_error_for(rid)
            except AttributeError:
                pass
            if (ai is not None and wrench is not None and w_idx is not None
                    and w_idx < len(wrench) and cerr is not None and ai < len(cerr)):
                self.fic_canvas.set_operating_point(cerr[ai], wrench[w_idx], True)
                self._set_op_readout(cerr[ai], wrench[w_idx], True)
            else:
                self.fic_canvas.set_operating_point(0.0, 0.0, False)
                self._set_op_readout(0.0, 0.0, False)

    # ── load ─────────────────────────────────────────────────────────────────

    def _load_tuning_parameters(self):
        try:
            from fic_studio.xml_manager import XMLManager
        except ImportError as e:
            QMessageBox.critical(self, "Error", f"Failed to import xml_manager: {e}")
            return

        robot_name = self.robot_dropdown.currentText()
        mode, is_joint, _, _ = self._tuning_set()
        profile = self._get_current_profile()

        try:
            mgr = XMLManager()

            if is_joint:
                joint_id = int(self.joint_id_dropdown.currentText())
                p = mgr.load_joint_params(robot_name, joint_id, mode, profile)

                self.tuning_stiffness.set_value(p.stiffness)
                n_half = p.n_saturations // 2
                self.tuning_error_max.set_count(n_half)
                self.tuning_force_max.set_count(n_half)
                self.tuning_error_max.set_values(p.error_max)
                self.tuning_force_max.set_values(p.force_max)
                sr = p.saturation_ratios[0] if p.saturation_ratios else 0.9
                self.tuning_saturation_ratios.set_value(sr)
                self.tuning_n_sat.blockSignals(True)
                self.tuning_n_sat.setValue(p.n_saturations)
                self.tuning_n_sat.blockSignals(False)
                self.tuning_asymmetric.setChecked(p.asymmetric)
                self.tuning_n_neg_sat.setValue(p.n_neg_sat)
                self.tuning_velocity.set_value(p.velocity)

                self.tuning_planner_stiffness.set_value(p.planner_stiffness)
                self.tuning_vel_desired.set_value(p.planner_vel_desired)
                self.tuning_acc_max.set_value(p.planner_acc_max)
                self.tuning_damping_ratio.set_value(p.planner_damping_ratio)
                self.tuning_frequency.set_value(p.planner_frequency)
                self.tuning_wrapped.setCurrentText(str(p.planner_wrapped))
                self.tuning_pos_limits_lower.set_value(p.planner_pos_lower)
                self.tuning_pos_limits_upper.set_value(p.planner_pos_upper)

                self.tuning_inv_m.set_value(p.admittance_inv_m)
                self.tuning_damping.set_value(p.admittance_damping)
                _dflow = p.admittance_dflow
                self.tuning_dflow_0.set_value(_dflow[0] if len(_dflow) > 0 else 0.0)
                self.tuning_dflow_1.set_value(_dflow[1] if len(_dflow) > 1 else 0.0)
                _flow = p.admittance_flow
                self.tuning_flow_0.set_value(_flow[0] if len(_flow) > 0 else 0.0)
                self.tuning_flow_1.set_value(_flow[1] if len(_flow) > 1 else 0.0)
                _pose = p.admittance_pose
                self.tuning_pose_0.set_value(_pose[0] if len(_pose) > 0 else 0.0)
                self.tuning_pose_1.set_value(_pose[1] if len(_pose) > 1 else 0.0)
            else:
                axis = self.axis_dropdown.currentText()
                if axis == 'elbow':
                    p = mgr.load_elbow_params(robot_name)

                    self.tuning_stiffness.set_value(p.stiffness)
                    n_half = p.n_saturations // 2
                    self.tuning_error_max.set_count(n_half)
                    self.tuning_force_max.set_count(n_half)
                    self.tuning_error_max.set_values(p.error_max)
                    self.tuning_force_max.set_values(p.force_max)
                    sr = p.saturation_ratios[0] if p.saturation_ratios else 0.9
                    self.tuning_saturation_ratios.set_value(sr)
                    self.tuning_n_sat.blockSignals(True)
                    self.tuning_n_sat.setValue(p.n_saturations)
                    self.tuning_n_sat.blockSignals(False)
                    self.tuning_asymmetric.setChecked(p.asymmetric)
                    self.tuning_n_neg_sat.setValue(p.n_neg_sat)

                elif axis == 'postural':
                    p = mgr.load_postural_params(robot_name)

                    self.tuning_stiffness.set_value(p.stiffness)
                    n_half = p.n_saturations // 2
                    self.tuning_error_max.set_count(n_half)
                    self.tuning_force_max.set_count(n_half)
                    self.tuning_error_max.set_values(p.error_max)
                    self.tuning_force_max.set_values(p.force_max)
                    sr = p.saturation_ratios[0] if p.saturation_ratios else 0.9
                    self.tuning_saturation_ratios.set_value(sr)
                    self.tuning_n_sat.blockSignals(True)
                    self.tuning_n_sat.setValue(p.n_saturations)
                    self.tuning_n_sat.blockSignals(False)
                    self.tuning_asymmetric.setChecked(p.asymmetric)
                    self.tuning_n_neg_sat.setValue(p.n_neg_sat)

                    # Planner fields are persisted but runtime-inactive for postural.
                    self.tuning_planner_stiffness.set_value(p.planner_stiffness)
                    self.tuning_vel_desired.set_value(p.planner_vel_desired)
                    self.tuning_acc_max.set_value(p.planner_acc_max)
                    self.tuning_damping_ratio.set_value(p.planner_damping_ratio)
                    self.tuning_frequency.set_value(p.planner_frequency)
                else:
                    p = mgr.load_cartesian_axis(robot_name, mode, axis, profile)

                    self.tuning_stiffness.set_value(p.stiffness)
                    n_half = p.n_saturations // 2
                    self.tuning_error_max.set_count(n_half)
                    self.tuning_force_max.set_count(n_half)
                    self.tuning_error_max.set_values(p.error_max)
                    self.tuning_force_max.set_values(p.force_max)
                    sr = p.saturation_ratios[0] if p.saturation_ratios else 0.9
                    self.tuning_saturation_ratios.set_value(sr)
                    self.tuning_n_sat.blockSignals(True)
                    self.tuning_n_sat.setValue(p.n_saturations)
                    self.tuning_n_sat.blockSignals(False)
                    self.tuning_asymmetric.setChecked(p.asymmetric)
                    self.tuning_n_neg_sat.setValue(p.n_neg_sat)

                    self.tuning_planner_stiffness.set_value(p.planner_stiffness_p)
                    self.tuning_vel_desired.set_value(p.planner_vel_desired)
                    self.tuning_acc_max.set_value(p.planner_acc_max)
                    self.tuning_damping_ratio.set_value(p.planner_damping_ratio)
                    self.tuning_frequency.set_value(p.planner_frequency)

                    self.tuning_inv_m.set_value(p.admittance_inv_m)
                    self.tuning_damping.set_value(p.admittance_damping)
                    _dflow = p.admittance_dflow
                    self.tuning_dflow_0.set_value(_dflow[0] if len(_dflow) > 0 else 0.0)
                    self.tuning_dflow_1.set_value(_dflow[1] if len(_dflow) > 1 else 0.0)
                    _flow = p.admittance_flow
                    self.tuning_flow_0.set_value(_flow[0] if len(_flow) > 0 else 0.0)
                    self.tuning_flow_1.set_value(_flow[1] if len(_flow) > 1 else 0.0)
                    _pose = p.admittance_pose
                    self.tuning_pose_0.set_value(_pose[0] if len(_pose) > 0 else 0.0)
                    self.tuning_pose_1.set_value(_pose[1] if len(_pose) > 1 else 0.0)

        except Exception as e:
            QMessageBox.warning(self, 'Load Warning', 'Could not load parameters: %s' % e)

        self._sync_canvas_from_fields()

    # ── save helpers ─────────────────────────────────────────────────────────

    def _validate_before_save(self):
        """Check MetricField ranges and breakpoint monotonicity.

        Returns a non-empty error string when validation fails, or an empty
        string when everything is in order.  MetricField values are already
        clamped by the widget, so range violations can only happen when
        ``clamp=False`` is passed (currently no field uses that).  The
        breakpoint strips carry ascending-order requirements that the canvas
        drag enforces but manual field edits may violate.
        """
        em = self.tuning_error_max.values() if hasattr(self, 'tuning_error_max') else []
        fm = self.tuning_force_max.values() if hasattr(self, 'tuning_force_max') else []

        # Monotonicity: each breakpoint must be strictly greater than the previous.
        for i in range(1, len(em)):
            if em[i] <= em[i - 1]:
                return (
                    'error_max breakpoints must be strictly ascending: '
                    'index %d (%.4f) <= index %d (%.4f)' % (i, em[i], i - 1, em[i - 1])
                )
        for i in range(1, len(fm)):
            if fm[i] <= fm[i - 1]:
                return (
                    'force_max breakpoints must be strictly ascending: '
                    'index %d (%.2f) <= index %d (%.2f)' % (i, fm[i], i - 1, fm[i - 1])
                )

        # All MetricField widgets clamp to [min, max] — guard the stiffness field
        # specifically because the _clamp_stiffness helper imposes a tighter bound.
        if hasattr(self, 'tuning_stiffness') and em and fm:
            k0 = self.tuning_stiffness.value()
            if em[0] > 1e-12:
                max_k = fm[0] / em[0]
                if k0 > max_k + 1e-6:
                    return (
                        'Stiffness K₀ (%.1f) exceeds force_max[0]/error_max[0] '
                        '(%.1f)' % (k0, max_k)
                    )

        return ''

    def _set_applied_stamp(self, timestamp_str):
        """Update the applied label with the provided HH:MM:SS timestamp string.

        The caller supplies the timestamp string so that tests can inject a
        deterministic value without calling datetime.now().  At runtime,
        _save_tuning_parameters() passes QTime.currentTime().toString('hh:mm:ss').
        """
        if hasattr(self, '_applied_label'):
            self._applied_label.setText('Saved to config.xml at %s' % timestamp_str)

    # ── save ─────────────────────────────────────────────────────────────────

    def _save_tuning_parameters(self):
        try:
            from fic_studio.xml_manager import (
                XMLManager, JointParams, CartesianAxisParams, PosturalParams,
                ElbowParams, mode_name,
            )
        except ImportError as e:
            QMessageBox.critical(self, 'Error', 'Failed to import xml_manager: %s' % e)
            return

        # Validate before writing — rejects bad breakpoint order.
        err = self._validate_before_save()
        if err:
            QMessageBox.critical(self, 'Validation Error', err)
            return

        robot_name = self.robot_dropdown.currentText()
        mode, is_joint, _, _ = self._tuning_set()
        profile = self._get_current_profile()

        try:
            mgr = XMLManager()

            if is_joint:
                joint_id = int(self.joint_id_dropdown.currentText())
                p = JointParams()
                p.stiffness         = self.tuning_stiffness.value()
                # not on the page (no controller reads it): keep what is stored
                p.diff_effort_max   = mgr.load_joint_params(robot_name, joint_id, mode, profile).diff_effort_max
                p.error_max         = self.tuning_error_max.values()
                p.force_max         = self.tuning_force_max.values()
                p.saturation_ratios = [self.tuning_saturation_ratios.value()]
                p.n_saturations     = self.tuning_n_sat.value()
                p.asymmetric        = self.tuning_asymmetric.isChecked()
                p.n_neg_sat         = self.tuning_n_neg_sat.value()
                p.velocity          = self.tuning_velocity.value()
                # not on the page (the planner does not use them): keep what is stored
                stored = mgr.load_joint_params(robot_name, joint_id, mode, profile)
                p.planner_stiffness     = stored.planner_stiffness
                p.planner_vel_desired   = self.tuning_vel_desired.value()
                p.planner_acc_max       = self.tuning_acc_max.value()
                p.planner_damping_ratio = self.tuning_damping_ratio.value()
                p.planner_frequency     = stored.planner_frequency
                p.planner_wrapped       = int(self.tuning_wrapped.currentText())
                p.planner_pos_lower     = self.tuning_pos_limits_lower.value()
                p.planner_pos_upper     = self.tuning_pos_limits_upper.value()
                p.admittance_inv_m      = self.tuning_inv_m.value()
                p.admittance_damping    = self.tuning_damping.value()
                p.admittance_dflow      = [self.tuning_dflow_0.value(), self.tuning_dflow_1.value()]
                p.admittance_flow       = [self.tuning_flow_0.value(), self.tuning_flow_1.value()]
                p.admittance_pose       = [self.tuning_pose_0.value(), self.tuning_pose_1.value()]

                mgr.save_joint_params(robot_name, joint_id, mode, p, profile)
                mgr.save()
                self._publish_param_update('joint', joint_id)
                from PyQt5.QtCore import QTime
                self._set_applied_stamp(QTime.currentTime().toString('hh:mm:ss'))
                QMessageBox.information(
                    self, 'Saved',
                    'Joint %d saved for %s / %s.' % (joint_id, mode_name(mode), profile),
                )
            else:
                axis = self.axis_dropdown.currentText()
                if axis == 'elbow':
                    p = ElbowParams()
                    p.stiffness         = self.tuning_stiffness.value()
                    p.diff_effort_max   = mgr.load_elbow_params(robot_name).diff_effort_max
                    p.error_max         = self.tuning_error_max.values()
                    p.force_max         = self.tuning_force_max.values()
                    p.saturation_ratios = [self.tuning_saturation_ratios.value()]
                    p.n_saturations     = self.tuning_n_sat.value()
                    p.asymmetric        = self.tuning_asymmetric.isChecked()
                    p.n_neg_sat         = self.tuning_n_neg_sat.value()

                    mgr.save_elbow_params(robot_name, p)
                    mgr.save()
                    self._publish_param_update('elbow')
                    from PyQt5.QtCore import QTime
                    self._set_applied_stamp(QTime.currentTime().toString('hh:mm:ss'))
                    QMessageBox.information(
                        self, 'Saved',
                        'Elbow saved for %s (shared by all Cartesian modes).' % robot_name,
                    )
                elif axis == 'postural':
                    p = PosturalParams()
                    p.stiffness         = self.tuning_stiffness.value()
                    p.diff_effort_max   = mgr.load_postural_params(robot_name).diff_effort_max
                    p.error_max         = self.tuning_error_max.values()
                    p.force_max         = self.tuning_force_max.values()
                    p.saturation_ratios = [self.tuning_saturation_ratios.value()]
                    p.n_saturations     = self.tuning_n_sat.value()
                    p.asymmetric        = self.tuning_asymmetric.isChecked()
                    p.n_neg_sat         = self.tuning_n_neg_sat.value()
                    # Planner fields are persisted for round-tripping even
                    # though the postural path doesn't run the planner at runtime.
                    stored = mgr.load_postural_params(robot_name)
                    p.planner_stiffness     = stored.planner_stiffness
                    p.planner_vel_desired   = self.tuning_vel_desired.value()
                    p.planner_acc_max       = self.tuning_acc_max.value()
                    p.planner_damping_ratio = self.tuning_damping_ratio.value()
                    p.planner_frequency     = stored.planner_frequency

                    mgr.save_postural_params(robot_name, p)
                    mgr.save()
                    self._publish_param_update('postural')
                    from PyQt5.QtCore import QTime
                    self._set_applied_stamp(QTime.currentTime().toString('hh:mm:ss'))
                    QMessageBox.information(
                        self, 'Saved',
                        'Postural saved for %s (shared by all Cartesian modes).' % robot_name,
                    )
                else:
                    p = CartesianAxisParams()
                    p.stiffness         = self.tuning_stiffness.value()
                    p.diff_effort_max   = mgr.load_cartesian_axis(robot_name, mode, axis, profile).diff_effort_max
                    p.error_max         = self.tuning_error_max.values()
                    p.force_max         = self.tuning_force_max.values()
                    p.saturation_ratios = [self.tuning_saturation_ratios.value()]
                    p.n_saturations     = self.tuning_n_sat.value()
                    p.asymmetric        = self.tuning_asymmetric.isChecked()
                    p.n_neg_sat         = self.tuning_n_neg_sat.value()
                    stored = mgr.load_cartesian_axis(robot_name, mode, axis, profile)
                    p.planner_stiffness_p  = stored.planner_stiffness_p
                    p.planner_vel_desired  = self.tuning_vel_desired.value()
                    p.planner_acc_max      = self.tuning_acc_max.value()
                    p.planner_damping_ratio = self.tuning_damping_ratio.value()
                    p.planner_frequency    = stored.planner_frequency
                    p.admittance_inv_m      = self.tuning_inv_m.value()
                    p.admittance_damping    = self.tuning_damping.value()
                    p.admittance_dflow      = [self.tuning_dflow_0.value(), self.tuning_dflow_1.value()]
                    p.admittance_flow       = [self.tuning_flow_0.value(), self.tuning_flow_1.value()]
                    p.admittance_pose       = [self.tuning_pose_0.value(), self.tuning_pose_1.value()]

                    mgr.save_cartesian_axis(robot_name, mode, axis, p, profile)
                    mgr.save()
                    self._publish_param_update('cartesian', axis)
                    from PyQt5.QtCore import QTime
                    self._set_applied_stamp(QTime.currentTime().toString('hh:mm:ss'))
                    QMessageBox.information(
                        self, 'Saved',
                        'Cartesian %s saved for %s / %s.' % (axis, mode_name(mode), profile),
                    )

        except ValueError as e:
            QMessageBox.critical(self, 'Error', 'Invalid parameter value: %s' % e)
        except Exception as e:
            QMessageBox.critical(self, 'Error', 'Failed to save: %s' % e)

    # ── view raw XML ─────────────────────────────────────────────────────────

    def _open_config_xml_viewer(self):
        from fic_studio.mixins.config_viewer_dialog import ConfigViewerDialog
        from fic_studio.xml_manager import find_config_path

        robot_name = self.robot_dropdown.currentText()
        xml_path = find_config_path()

        dlg = ConfigViewerDialog(
            parent=self,
            file_path=xml_path,
            robot_name=robot_name,
            file_type="xml"
        )
        dlg.exec_()

    # ── stubs kept for compat ────────────────────────────────────────────────
