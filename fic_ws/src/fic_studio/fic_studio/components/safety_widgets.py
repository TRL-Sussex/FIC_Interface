"""
Safety-zone configuration + live-visualization widget (joint & Cartesian).

A reworked, mode-aware safety UI: an unmissable master enabled/disabled banner,
per-axis Cartesian workspace bounds and per-joint angle bounds, and a live
readout of the current position against each limit (green in-zone, red past it).

The widget is pure config + display — it owns no ROS. Hosts call ``set_live``
each refresh with telemetry, ``to_config``/``from_config`` to persist, and
``set_active_mode`` to emphasise the section that matters for the active mode.
"""
import math

from fic_studio import theme
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox, QLabel, QVBoxLayout,
    QWidget,
)


class _LimitRow:
    """One configurable axis/joint: min/max spinboxes + a live value label."""

    def __init__(self, grid, row, label, lo, hi, unit, decimals, rng):
        """Build the row's min/max spinboxes and live label into ``grid``."""
        self.unit = unit
        grid.addWidget(QLabel(label), row, 0)
        self.min = QDoubleSpinBox()
        self.min.setRange(-rng, rng)
        self.min.setDecimals(decimals)
        self.min.setValue(lo)
        grid.addWidget(self.min, row, 1)
        self.max = QDoubleSpinBox()
        self.max.setRange(-rng, rng)
        self.max.setDecimals(decimals)
        self.max.setValue(hi)
        grid.addWidget(self.max, row, 2)
        self.live = QLabel('--')
        self.live.setAlignment(Qt.AlignCenter)
        grid.addWidget(self.live, row, 3)

    def set_live(self, value):
        if value is None:
            theme.bind(self.live, lambda t: 'color: %s;' % t['text_disabled'])
            self.live.setText('--')
            return
        inzone = self.min.value() <= value <= self.max.value()
        tok = 'accent_green' if inzone else 'accent_red'
        theme.bind(self.live, lambda t, k=tok: 'color: %s; font-weight: 600;' % t[k])
        self.live.setText('%.3f %s' % (value, self.unit))


class SafetyZonePanel(QWidget):
    """Configure and live-visualize joint + Cartesian safety zones."""

    changed = pyqtSignal()

    def __init__(self, joint_count=7, joint_names=None, joint_limits_deg=None, parent=None):
        """Build the joint + Cartesian safety panel for ``joint_count`` joints."""
        super().__init__(parent)
        self.joint_count = joint_count
        self._joint_limits_deg = joint_limits_deg
        names = joint_names or ['Joint %d' % (i + 1) for i in range(joint_count)]
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        # ── master enabled banner (unmissable) ───────────────────────────────
        self._master = QCheckBox('Safety zones enabled')
        self._master.setChecked(False)
        self._master.toggled.connect(self._on_master)
        self._banner = QLabel()
        self._banner.setAlignment(Qt.AlignCenter)
        self._banner.setMinimumHeight(34)
        root.addWidget(self._banner)
        root.addWidget(self._master)

        # ── Cartesian workspace box ───────────────────────────────────────────
        self._cart_grp = QGroupBox('Cartesian workspace box  (metres)')
        cg = QGridLayout(self._cart_grp)
        for c, h in enumerate(('', 'Min', 'Max', 'Live')):
            cg.addWidget(QLabel(h), 0, c)
        self._cart_en = {}
        self._cart = {}
        defaults = {'x': (-0.8, 0.8), 'y': (-0.8, 0.8), 'z': (0.0, 1.2)}
        for r, ax in enumerate(('x', 'y', 'z'), start=1):
            lo, hi = defaults[ax]
            self._cart[ax] = _LimitRow(cg, r, ax.upper() + ':', lo, hi, 'm', 3, 2.0)
            en = QCheckBox('on')
            en.setChecked(False)
            en.toggled.connect(self.changed)
            cg.addWidget(en, r, 4)
            self._cart_en[ax] = en
            self._cart[ax].min.valueChanged.connect(self.changed)
            self._cart[ax].max.valueChanged.connect(self.changed)
        root.addWidget(self._cart_grp)

        # ── per-joint angle bounds (rebuilt per robot via set_joint_count) ────
        self._joint_grp = QGroupBox('Joint angle bounds  (degrees)')
        self._joint_grid = QGridLayout(self._joint_grp)
        self._joints = []
        self._joint_names = []
        self._build_joint_rows(joint_count, names, joint_limits_deg)
        root.addWidget(self._joint_grp)
        root.addStretch()

        theme.signals.theme_changed.connect(self._restyle_banner)
        self._restyle_banner()

    # ── joint rows (dynamic per robot) ───────────────────────────────────────
    def _build_joint_rows(self, joint_count, names, joint_limits_deg=None):
        """
        (Re)build the per-joint rows + enforce checkbox for ``joint_count``.

        Seed each row's default min/max from ``joint_limits_deg`` (the
        robot's mechanical ``{'lower': [...], 'upper': [...]}``, in degrees)
        at the matching index; fall back to the +/-170 deg arm default for
        any joint the robot's config leaves unspecified, e.g. a robot with
        no mechanical limits at all. This is only a starting point for the
        operator's own soft ``joint_safety_deg`` bounds -- it is not the same
        thing as the mechanical stops, and ``from_config`` still overwrites
        these defaults whenever the operator has saved explicit values.
        """
        grid = self._joint_grid
        while grid.count():
            item = grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)      # immediate visual removal
                w.deleteLater()        # free the C++ object
        self.joint_count = joint_count
        self._joint_names = list(names)
        self._joint_limits_deg = joint_limits_deg
        self._joints = []
        limits = joint_limits_deg or {}
        lo_mech = limits.get('lower') or []
        hi_mech = limits.get('upper') or []
        for c, h in enumerate(('', 'Min', 'Max', 'Live')):
            grid.addWidget(QLabel(h), 0, c)
        for r in range(joint_count):
            label = names[r] if r < len(names) else 'Joint %d' % (r + 1)
            if r < len(lo_mech) and r < len(hi_mech):
                lo, hi = lo_mech[r], hi_mech[r]
            else:
                lo, hi = -170.0, 170.0
            row = _LimitRow(grid, r + 1, label + ':', lo, hi, '°', 1, 360.0)
            row.min.valueChanged.connect(self.changed)
            row.max.valueChanged.connect(self.changed)
            self._joints.append(row)
        self._joint_en = QCheckBox('Enforce joint bounds')
        self._joint_en.setChecked(False)
        self._joint_en.toggled.connect(self.changed)
        grid.addWidget(self._joint_en, joint_count + 1, 0, 1, 4)

    def set_joint_count(self, n, joint_names=None, joint_limits_deg=None):
        """
        Rebuild the joint section for the current robot's ``n`` joints.

        Returns True if the rows were rebuilt, False if nothing changed. A
        change in joint count, joint names, or ``joint_limits_deg`` triggers a
        rebuild -- so switching between two robots that happen to share a
        joint count still reseeds each row's defaults from the newly-selected
        robot's own mechanical limits -- and resets the rows to those
        defaults, so callers that rebuild should re-apply ``from_config``.
        No-op otherwise, so it is cheap to call on every live refresh.
        """
        names = joint_names or ['Joint %d' % (i + 1) for i in range(n)]
        if (n == self.joint_count and names == self._joint_names
                and joint_limits_deg == self._joint_limits_deg):
            return False
        jen = self._joint_en.isChecked()
        self._build_joint_rows(n, names, joint_limits_deg)
        self._joint_en.blockSignals(True)      # restore state without a spurious publish
        self._joint_en.setChecked(jen)
        self._joint_en.blockSignals(False)
        return True

    # ── master + banner ──────────────────────────────────────────────────────
    def _on_master(self, _on):
        """Re-style the banner and notify when the master toggle changes."""
        self._restyle_banner()
        self.changed.emit()

    def _restyle_banner(self, *_):
        t = theme.current()
        on = self._master.isChecked()
        if on:
            self._banner.setText('●  SAFETY ZONES ACTIVE')
            col, bg = t['accent_green'], t['bg_raised']
        else:
            self._banner.setText('○  SAFETY ZONES OFF — robot motion is unrestricted')
            col, bg = t['accent_red'], t['bg_raised']
        self._banner.setStyleSheet(
            'QLabel { color: %s; background: %s; border: 1px solid %s;'
            ' border-radius: 6px; font-weight: 700; font-size: 13px; }'
            % (col, bg, col))

    def set_active_mode(self, is_joint):
        """Emphasise the section relevant to the active control mode."""
        self._joint_grp.setTitle(
            ('▸ ' if is_joint else '') + 'Joint angle bounds  (degrees)')
        self._cart_grp.setTitle(
            ('▸ ' if not is_joint else '') + 'Cartesian workspace box  (metres)')

    # ── live telemetry ───────────────────────────────────────────────────────
    def set_live(self, ee_pos=None, joint_pos_deg=None):
        """Update the live readouts. ee_pos=(x,y,z) m; joint_pos_deg=list deg."""
        if ee_pos is not None:
            for v, ax in zip(ee_pos, ('x', 'y', 'z')):
                self._cart[ax].set_live(v)
        if joint_pos_deg is not None:
            for i, row in enumerate(self._joints):
                row.set_live(joint_pos_deg[i] if i < len(joint_pos_deg) else None)

    # ── config round-trip (matches safety_mixin schema) ──────────────────────
    def to_config(self):
        """Return the safety_zones dict (degrees for joints, metres for Cartesian)."""
        master = self._master.isChecked()
        cart = {}
        for ax in ('x', 'y', 'z'):
            cart[ax] = {
                'min': self._cart[ax].min.value(),
                'max': self._cart[ax].max.value(),
                'enabled': master and self._cart_en[ax].isChecked(),
            }
        return {
            'cartesian': cart,
            'joint_safety_deg': {
                'lower': [r.min.value() for r in self._joints],
                'upper': [r.max.value() for r in self._joints],
                'enabled': master and self._joint_en.isChecked(),
            },
        }

    def from_config(self, cfg):
        """Load a safety_zones dict into the widgets."""
        cfg = cfg or {}
        cart = cfg.get('cartesian', {})
        any_en = False
        for ax in ('x', 'y', 'z'):
            a = cart.get(ax, {})
            if 'min' in a:
                self._cart[ax].min.setValue(a['min'])
            if 'max' in a:
                self._cart[ax].max.setValue(a['max'])
            en = bool(a.get('enabled', False))
            self._cart_en[ax].setChecked(en)
            any_en = any_en or en
        js = cfg.get('joint_safety_deg', {})
        lo, hi = js.get('lower', []), js.get('upper', [])
        for i, row in enumerate(self._joints):
            if i < len(lo):
                row.min.setValue(lo[i])
            if i < len(hi):
                row.max.setValue(hi[i])
        jen = bool(js.get('enabled', False))
        self._joint_en.setChecked(jen)
        self._master.setChecked(any_en or jen)
        self._restyle_banner()

    @staticmethod
    def joint_pos_deg_from_rad(positions_rad):
        """Convert a joint-position list (rad) to degrees."""
        return [math.degrees(p) for p in (positions_rad or [])]
