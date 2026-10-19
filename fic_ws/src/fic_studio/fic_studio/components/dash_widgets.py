"""
Painted dashboard widgets for the FIC Studio monitoring hub.

All widgets read ``theme.current()`` inside ``paintEvent`` and subscribe to
``theme.signals.theme_changed`` so they repaint automatically on every theme
switch.  No inline colour hex -- all colours come from the token dict.
"""
from fic_studio import theme
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QWidget


class TrajectoryPlot(QFrame):
    """
    Painted miniature line plot for End-Effector trajectory history.

    Accepts a list of normalised Y values in ``[0, 1]`` (oldest first).
    Background, grid and line colours come from the active theme.
    """

    def __init__(self, parent=None):
        """Initialise with an empty history buffer."""
        super().__init__(parent)
        self._pts = []
        self._max_pts = 120
        self.setMinimumHeight(120)
        theme.signals.theme_changed.connect(self.update)

    def set_points(self, pts):
        """Replace the entire history buffer and repaint."""
        self._pts = list(pts)[-self._max_pts:]
        self.update()

    def push(self, value):
        """Append a single normalised value to the rolling buffer."""
        self._pts.append(float(value))
        if len(self._pts) > self._max_pts:
            self._pts.pop(0)
        self.update()

    def paintEvent(self, event):
        """Paint the sparkline using the current theme tokens."""
        T = theme.current()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(T['bg_base']))
        p.setPen(QPen(QColor(T['border_subtle']), 1))
        p.drawRoundedRect(1, 1, w - 2, h - 2, 8, 8)
        p.setPen(QPen(QColor(T['border_subtle']), 1))
        for gy in range(1, 4):
            p.drawLine(8, int(h * gy / 4), w - 8, int(h * gy / 4))
        pts = self._pts
        n = len(pts)
        if n < 2:
            return
        p.setPen(QPen(QColor(T['accent_blue']), 2))
        prev = None
        for i, val in enumerate(pts):
            x = 8 + int((w - 16) * i / (n - 1))
            y = 10 + int((h - 20) * (1.0 - max(0.0, min(1.0, val))))
            if prev is not None:
                p.drawLine(prev[0], prev[1], x, y)
            prev = (x, y)


class LimitBar(QFrame):
    """
    Painted bar that turns ``accent_red`` when ``frac >= limit_frac``.

    Normal fill colour is ``accent_orange``; exceeding the limit flips it to
    ``accent_red``.  Set state via ``set_frac()`` or ``set_value()``.
    """

    def __init__(self, parent=None):
        """Initialise with zero value and a default limit of 0.85."""
        super().__init__(parent)
        self._frac = 0.0
        self._limit_frac = 0.85
        self.setFixedHeight(7)
        self.setMinimumWidth(80)
        theme.signals.theme_changed.connect(self.update)

    def set_value(self, value, frac, limit_frac=None):
        """Update the bar with a raw value, normalised frac, and optional new limit."""
        self._frac = max(0.0, min(1.0, float(frac)))
        if limit_frac is not None:
            self._limit_frac = float(limit_frac)
        self.update()

    def set_frac(self, frac, limit_frac=None):
        """Update the bar from a pre-computed normalised fraction."""
        self._frac = max(0.0, min(1.0, float(frac)))
        if limit_frac is not None:
            self._limit_frac = float(limit_frac)
        self.update()

    def paintEvent(self, event):
        """Paint the limit bar using the current theme tokens."""
        T = theme.current()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.setBrush(QColor(T['bg_base']))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(0, 0, w, h, 3, 3)
        fill_key = 'accent_red' if self._frac >= self._limit_frac else 'accent_orange'
        fill_w = int(w * self._frac)
        if fill_w > 0:
            p.setBrush(QColor(T[fill_key]))
            p.drawRoundedRect(0, 0, fill_w, h, 3, 3)


class StatusRow(QWidget):
    """
    A horizontal label + OK/WARN indicator row.

    The bullet and status text use ``accent_green`` (OK) or ``accent_orange``
    (WARN) from the active theme.  Drive state with ``set_ok(True/False)``.
    """

    def __init__(self, label, ok=True, parent=None):
        """Initialise with a descriptive label and initial ok state."""
        super().__init__(parent)
        self._ok = ok
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self._bullet = QLabel('●')
        self._bullet.setFixedWidth(16)
        layout.addWidget(self._bullet)

        self._label_w = QLabel(label)
        layout.addWidget(self._label_w)
        layout.addStretch()

        self._status_w = QLabel('OK')
        layout.addWidget(self._status_w)

        self._apply_style()
        theme.signals.theme_changed.connect(self._apply_style)

    @property
    def ok(self):
        """Return the current OK/WARN state."""
        return self._ok

    def set_ok(self, ok):
        """Set the OK/WARN state and refresh the indicator colours."""
        if bool(ok) == self._ok:
            return                      # refreshed 20 times a second: restyle only on a change
        self._ok = bool(ok)
        self._apply_style()

    def _apply_style(self, _theme_name=None):
        """Apply theme-token colours to the indicator labels."""
        T = theme.current()
        colour = T['accent_green'] if self._ok else T['accent_orange']
        text_col = T['text_secondary']
        self._bullet.setStyleSheet(
            'background: transparent; color: %s; font-size: 12px;' % colour
        )
        self._label_w.setStyleSheet(
            'background: transparent; color: %s; font-size: 12px;' % text_col
        )
        self._status_w.setStyleSheet(
            'background: transparent; color: %s; font-size: 11px;'
            ' font-weight: 700;' % colour
        )
        self._status_w.setText('OK' if self._ok else 'WARN')
