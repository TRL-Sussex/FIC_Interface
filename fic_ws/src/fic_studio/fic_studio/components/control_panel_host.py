"""ControlPanelHost — the control bar's button row and its slide-up panels.

Modelled on the Kinova Kortex Web App's robot control panel, which sits
along the bottom of every screen: clicking a button launches a small panel
up from the bottom, and clicking the same button again clears it.

The host owns the *mechanics* — registration, exclusivity, the animation,
enable/disable — and nothing about robot control. Callers supply the panel
bodies. Layout is panel body on top, button row beneath, so opening a panel
grows the bar and shortens the page rather than covering it: on a dense
telemetry page an overlay would hide the readings you are commanding
against.
"""

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QFrame, QHBoxLayout, QPushButton, QSizePolicy, QVBoxLayout, QWidget,
)

from fic_studio import theme

#: Floor for the open panel. There is no ceiling and no scroll area: the
#: panel is exactly as tall as its contents need, so nothing is ever
#: clipped or hidden behind a scrollbar.
MIN_PANEL_HEIGHT = 72

#: Qt's "no maximum" sentinel.
QWIDGETSIZE_MAX = (1 << 24) - 1


def _settled_height(widget):
    """Return ``widget``'s height hint after forcing its layout to settle.

    A widget whose layout was just rebuilt still reports the old hint until
    Qt re-activates it. Measuring at that moment sized the panel to the
    stale value and the control surface collapsed to a sliver.
    """
    lay = widget.layout()
    if lay is not None:
        lay.invalidate()
        lay.activate()
    widget.updateGeometry()
    return widget.sizeHint().height()


class ControlPanelHost(QWidget):
    """A button row with at most one slide-up panel open at a time."""

    panel_changed = pyqtSignal(object)

    def __init__(self, parent=None):
        """Build an empty host with no panels registered."""
        super().__init__(parent)
        self._panels = {}
        self._buttons = {}
        self._current = None

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self._panel_area = QFrame()
        self._panel_area.setObjectName('ControlPanelArea')
        theme.bind(self._panel_area, lambda T: (
            'QFrame#ControlPanelArea { background: %s;'
            ' border-top: 1px solid %s; }' % (T['bg_raised'], T['border_subtle'])
        ))
        self._panel_layout = QVBoxLayout(self._panel_area)
        self._panel_layout.setContentsMargins(12, 8, 12, 8)
        self._panel_layout.setSpacing(6)
        # Shared strip below the bodies: the control surface lives here so
        # it is on screen for every menu, not duplicated into each one.
        self._shared = None
        self._shared_keys = ()
        self._panel_area.setMaximumHeight(0)
        self._panel_area.setVisible(False)
        root.addWidget(self._panel_area)

        self._button_row = QHBoxLayout()
        self._button_row.setContentsMargins(8, 4, 8, 4)
        self._button_row.setSpacing(6)
        root.addLayout(self._button_row)

    # ── Registration ──────────────────────────────────────────────────────

    def _make_button(self, text, checkable):
        """Create and style one bar button."""
        btn = QPushButton(text)
        btn.setCheckable(checkable)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setObjectName('ControlBarButton')
        btn.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        self._button_row.addWidget(btn)
        return btn

    def add_panel(self, key, text, body):
        """Register a panel under ``key`` and return its bar button."""
        btn = self._make_button(text, checkable=True)
        btn.clicked.connect(lambda _checked=False, k=key: self.toggle(k))
        body.setParent(self._panel_area)
        body.setVisible(False)
        self._panel_layout.addWidget(body)
        self._panels[key] = body
        self._buttons[key] = btn
        return btn

    def add_action(self, key, text, callback):
        """Register a button that fires ``callback`` and opens no panel."""
        btn = self._make_button(text, checkable=False)
        btn.clicked.connect(lambda _checked=False: callback())
        self._buttons[key] = btn
        return btn

    def set_shared(self, widget, keys):
        """Show ``widget`` beneath the body whenever one of ``keys`` is open."""
        self._shared = widget
        self._shared_keys = tuple(keys)
        widget.setParent(self._panel_area)
        self._panel_layout.addWidget(widget)
        widget.setVisible(False)

    def add_stretch(self):
        """Push everything registered after this to the right."""
        self._button_row.addStretch(1)

    def add_leading_stretch(self):
        """Insert a stretch at the far left, so the buttons sit centred."""
        self._button_row.insertStretch(0, 1)

    def add_left_spacer(self, width):
        """Reserve ``width`` at the far left to balance the status readout."""
        spacer = QWidget()
        spacer.setFixedWidth(int(width))
        self._button_row.insertWidget(0, spacer)
        return spacer

    def add_widget(self, widget):
        """Place an arbitrary widget (a status readout) in the button row."""
        self._button_row.addWidget(widget)
        return widget

    def button(self, key):
        """Return the bar button registered under ``key``, or None."""
        return self._buttons.get(key)

    def set_enabled(self, key, enabled, reason=''):
        """Enable or disable one button, explaining why when disabling."""
        btn = self._buttons.get(key)
        if btn is None:
            return
        btn.setEnabled(bool(enabled))
        btn.setToolTip('' if enabled else reason)
        if not enabled and self._current == key:
            self.close_panel()

    # ── Panel state ───────────────────────────────────────────────────────

    def current(self):
        """Return the open panel's key, or None."""
        return self._current

    def toggle(self, key):
        """Open ``key``'s panel, or close it if it is already open."""
        if key not in self._panels:
            return
        btn = self._buttons.get(key)
        if btn is not None and not btn.isEnabled():
            return
        if self._current == key:
            self.close_panel()
        else:
            self.open_panel(key)

    def open_panel(self, key):
        """Show ``key``'s panel, hiding whichever was open."""
        if key not in self._panels:
            return
        for other, body in self._panels.items():
            body.setVisible(other == key)
        for other, btn in self._buttons.items():
            if btn.isCheckable():
                btn.setChecked(other == key)
        if self._shared is not None:
            self._shared.setVisible(key in self._shared_keys)
        self._current = key
        self._panel_area.setVisible(True)
        self._resize_to(self._height_for(key))
        self.panel_changed.emit(key)

    def close_panel(self):
        """Hide whatever panel is open."""
        if self._current is None:
            return
        for body in self._panels.values():
            body.setVisible(False)
        for btn in self._buttons.values():
            if btn.isCheckable():
                btn.setChecked(False)
        self._current = None
        self._resize_to(0)
        self.panel_changed.emit(None)

    def _height_for(self, key):
        """Height this panel needs, clamped to the allowed range."""
        body = self._panels.get(key)
        if body is None:
            return MIN_PANEL_HEIGHT
        wanted = _settled_height(body)
        if self._shared is not None and key in self._shared_keys:
            wanted += _settled_height(self._shared) + self._panel_layout.spacing()
        margins = self._panel_layout.contentsMargins()
        wanted += margins.top() + margins.bottom()
        return max(MIN_PANEL_HEIGHT, wanted)

    def _resize_to(self, height):
        """Show or hide the panel area, letting its layout set the height.

        Nothing pins a pixel height any more. Measuring the contents and
        applying the result raced the rebuild of the control surface: after
        a mode change the surface's layout had not been re-activated yet,
        so the panel was sized from a stale hint and the controls collapsed
        to a sliver. Qt already knows how tall the layout wants to be —
        letting it size the panel removes the race rather than timing
        around it.
        """
        if height <= 0:
            self._panel_area.setVisible(False)
            self.updateGeometry()
            return
        self._panel_area.setMinimumHeight(0)
        self._panel_area.setMaximumHeight(QWIDGETSIZE_MAX)
        self._panel_area.setVisible(True)
        self._panel_area.updateGeometry()
        self.updateGeometry()

    # ── Input ─────────────────────────────────────────────────────────────

    def keyPressEvent(self, event):
        """Dismiss the open panel on Esc."""
        if event.key() == Qt.Key_Escape and self._current is not None:
            self.close_panel()
            event.accept()
            return
        super().keyPressEvent(event)
