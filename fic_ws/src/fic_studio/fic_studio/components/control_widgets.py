"""
Smart-input control widgets for the Robot Control page.

Provides:
    - ``MetricField`` — editable, scrubbable numeric field with a painted fill
      bar, click-to-type inline editing, and drag-to-scrub interaction.
    - ``CommandedVsActual`` — ``MetricField`` with a ported ``DualTrack``
      actual overlay and a "moving" chip.
    - ``CollapsibleSection`` — collapsible disclosure header with a body widget.
    - ``BreakpointStrip`` — a horizontal row of ``MetricField``s for editing
      N-breakpoint parameter lists (error_max, force_max, etc.).

All widgets read ``theme.current()`` inside ``paintEvent`` and subscribe to
``theme.signals.theme_changed`` for automatic repaints on theme switches.
No inline colour hex — all colours come from theme tokens.
"""

from fic_studio import theme
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QColor, QCursor, QPainter, QPen
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


# ---------------------------------------------------------------------------
# _InlineEdit  (private helper)
# ---------------------------------------------------------------------------

class _InlineEdit(QLineEdit):
    """Floating QLineEdit overlaid on the MetricField value area."""

    commit_requested = pyqtSignal(str)
    cancel_requested = pyqtSignal()

    def __init__(self, text, parent=None):
        """Initialise with pre-filled text and connect focus-out handling."""
        super().__init__(text, parent)
        self._committing = False
        self.selectAll()

    def keyPressEvent(self, event):
        """
        Commit on Enter/Return, cancel on Escape, pass rest to super.

        The ``_committing`` guard is set before emitting so that the
        synchronous ``focusOutEvent`` Qt fires when the editor is hidden
        inside ``_commit_text`` does not emit a second commit.
        """
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self._committing = True
            self.commit_requested.emit(self.text())
        elif event.key() == Qt.Key_Escape:
            self._committing = True
            self.cancel_requested.emit()
        else:
            super().keyPressEvent(event)

    def focusOutEvent(self, event):
        """Commit current text when focus is lost, unless already committing."""
        super().focusOutEvent(event)
        if not self._committing:
            self.commit_requested.emit(self.text())


# ---------------------------------------------------------------------------
# MetricField
# ---------------------------------------------------------------------------

class MetricField(QWidget):
    """
    Editable, scrubbable numeric metric input with a painted fill bar.

    Emits ``committed(float)`` when the user finalises a value (Enter/blur/
    mouse-release) and ``edited(float)`` on each intermediate scrub tick.

    ``label`` is the short axis/joint label; ``value`` is the initial value;
    ``minimum``/``maximum`` define the range; ``unit`` is the suffix;
    ``decimals`` controls display precision; ``step`` sets the Up/Down key
    increment (default: span/100); ``clamp`` controls whether out-of-range
    commits are clamped (True) or flagged invalid (False).
    """

    committed = pyqtSignal(float)
    edited = pyqtSignal(float)

    # fine-scrub multiplier when Shift is held
    _FINE = 0.2

    def __init__(
        self,
        label,
        value,
        minimum,
        maximum,
        unit='',
        decimals=2,
        step=None,
        clamp=True,
        parent=None,
        compact=False,
    ):
        """Initialise MetricField with range, display, and interaction settings."""
        super().__init__(parent)
        self._label_text = label
        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._unit = unit
        self._decimals = decimals
        self._clamp = clamp
        self._compact = compact
        self._step = float(step) if step is not None else (self._maximum - self._minimum) / 100.0
        self._value = float(value)
        if self._clamp:
            self._value = max(self._minimum, min(self._maximum, self._value))
        self._actual = None       # float | None — the current actual value
        self._invalid = False     # True when clamp=False and value out of range

        # Inline editor (created on demand, overlaid on this widget)
        self._editor = None       # _InlineEdit | None

        # Drag state
        self._drag_start_x = None
        self._drag_start_val = None

        # Layout ------------------------------------------------------------------
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        # 46 px is the roomy default. Compact trims it for dense grids such
        # as the control bar's joint surface, where seven of these stacked
        # vertically overflowed the panel.
        self.setFixedHeight(28 if compact else 46)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setCursor(QCursor(Qt.SizeHorCursor))

        self._root = QHBoxLayout(self)
        self._root.setContentsMargins(0, 0, 0, 0)
        self._root.setSpacing(0)

        # Label pill
        self._label_w = QLabel(label)
        self._label_w.setFixedWidth(32)
        self._label_w.setAlignment(Qt.AlignCenter)
        self._root.addWidget(self._label_w)

        # Track + value area (painted by paintEvent)
        self._track_w = _MetricTrack(self)
        self._root.addWidget(self._track_w, 1)

        # Unit label
        self._unit_w = QLabel(unit)
        self._unit_w.setFixedWidth(max(20, len(unit) * 9 + 6))
        self._unit_w.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._root.addWidget(self._unit_w)

        # Theme wiring
        theme.signals.theme_changed.connect(self._apply_label_style)
        self._apply_label_style()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def value(self):
        """Return the current committed float value."""
        return self._value

    def set_value(self, v):
        """Set the value programmatically (does not emit committed)."""
        v = float(v)
        if self._clamp:
            v = max(self._minimum, min(self._maximum, v))
        self._value = v
        self._invalid = not (self._minimum <= v <= self._maximum)
        self._track_w.update()

    def set_actual(self, v):
        """Set the actual (live) value for the overlay; None to clear."""
        self._actual = float(v) if v is not None else None
        self._track_w.update()

    def set_unit(self, unit):
        """Update the unit suffix label shown beside the value."""
        self._unit = unit
        self._unit_w.setText(unit)
        self._unit_w.setFixedWidth(max(20, len(unit) * 9 + 6))
        self._track_w.update()

    def setEnabled(self, enabled):
        """Enable or disable the widget and repaint."""
        super().setEnabled(enabled)
        self._track_w.update()

    # ------------------------------------------------------------------
    # Scrub API (also called by tests)
    # ------------------------------------------------------------------

    def _scrub_to(self, frac):
        """Map a [0, 1] fraction to a value in [min, max] and emit edited."""
        frac = max(0.0, min(1.0, float(frac)))
        v = self._minimum + frac * (self._maximum - self._minimum)
        self._value = v
        self._invalid = False
        self._track_w.update()
        self.edited.emit(v)

    # ------------------------------------------------------------------
    # Step API (Up/Down keys)
    # ------------------------------------------------------------------

    def _step_value(self, direction):
        """Step value by direction (+1 or -1) × step, clamp if needed, emit edited."""
        v = self._value + direction * self._step
        if self._clamp:
            v = max(self._minimum, min(self._maximum, v))
        self._value = v
        self._invalid = not self._clamp and not (self._minimum <= v <= self._maximum)
        self._track_w.update()
        self.edited.emit(v)

    # ------------------------------------------------------------------
    # Commit from text
    # ------------------------------------------------------------------

    def _commit_text(self, text):
        """Parse text, optionally clamp, store, and emit committed."""
        self._close_editor()
        try:
            v = float(text)
        except ValueError:
            return
        if self._clamp:
            v = max(self._minimum, min(self._maximum, v))
            self._invalid = False
        else:
            self._invalid = not (self._minimum <= v <= self._maximum)
        self._value = v
        self._track_w.update()
        self.committed.emit(v)

    def _cancel_edit(self):
        """Cancel inline editing without changing the value."""
        self._close_editor()

    # ------------------------------------------------------------------
    # Inline editor lifecycle
    # ------------------------------------------------------------------

    def _open_editor(self):
        """Show the inline QLineEdit over the track area."""
        if self._editor is not None:
            return
        text = ('{:.%df}' % self._decimals).format(self._value)
        ed = _InlineEdit(text, self._track_w)
        ed.setGeometry(0, 0, self._track_w.width(), self._track_w.height())
        ed.commit_requested.connect(self._commit_text)
        ed.cancel_requested.connect(self._cancel_edit)
        ed.show()
        ed.setFocus()
        self._editor = ed
        self._track_w.update()

    def _close_editor(self):
        """Destroy the inline editor if present."""
        if self._editor is not None:
            self._editor.hide()
            self._editor.deleteLater()
            self._editor = None
        self._track_w.update()

    # ------------------------------------------------------------------
    # Mouse + keyboard events
    # ------------------------------------------------------------------

    def mousePressEvent(self, event):
        """Begin drag-scrub or open editor on single click."""
        if not self.isEnabled():
            return
        if event.button() == Qt.LeftButton:
            self._drag_start_x = event.globalX()
            self._drag_start_val = self._value

    def mouseMoveEvent(self, event):
        """Drag horizontally to scrub the value."""
        if not self.isEnabled():
            return
        if self._drag_start_x is None:
            return
        dx = event.globalX() - self._drag_start_x
        span = self._maximum - self._minimum
        if span == 0:
            return
        multiplier = self._FINE if (event.modifiers() & Qt.ShiftModifier) else 1.0
        delta = dx / max(1, self._track_w.width()) * span * multiplier
        v = self._drag_start_val + delta
        if self._clamp:
            v = max(self._minimum, min(self._maximum, v))
        self._value = v
        self._invalid = not self._clamp and not (self._minimum <= v <= self._maximum)
        self._track_w.update()
        self.edited.emit(v)

    def mouseReleaseEvent(self, event):
        """Commit on release if dragged, or open editor if no drag."""
        if not self.isEnabled():
            return
        if event.button() == Qt.LeftButton:
            start = (self._drag_start_x if self._drag_start_x is not None
                     else event.globalX())
            dx = abs(event.globalX() - start)
            if dx < 4:
                self._open_editor()
            else:
                self.committed.emit(self._value)
            self._drag_start_x = None
            self._drag_start_val = None

    def keyPressEvent(self, event):
        """Handle Up/Down arrow keys for stepping."""
        if event.key() == Qt.Key_Up:
            self._step_value(+1)
        elif event.key() == Qt.Key_Down:
            self._step_value(-1)
        else:
            super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # Theme helpers
    # ------------------------------------------------------------------

    def _apply_label_style(self, _name=None):
        """Apply theme token colours to the label and unit widgets."""
        T = theme.current()
        label_css = (
            'background: transparent;'
            ' color: %s;'
            ' font-size: 12px;'
            ' font-weight: 700;'
        ) % T['text_secondary']
        unit_css = (
            'background: transparent;'
            ' color: %s;'
            ' font-size: 11px;'
        ) % T['text_disabled']
        self._label_w.setStyleSheet(label_css)
        self._unit_w.setStyleSheet(unit_css)

    def _value_frac(self):
        """Return the current value as a [0, 1] fill fraction."""
        span = self._maximum - self._minimum
        if span == 0:
            return 0.5
        return max(0.0, min(1.0, (self._value - self._minimum) / span))


# ---------------------------------------------------------------------------
# _MetricTrack  (private painted sub-widget)
# ---------------------------------------------------------------------------

class _MetricTrack(QFrame):
    """
    Painted interior of a MetricField: fill bar + value label.

    Reads all colours from ``theme.current()`` in ``paintEvent``.
    """

    def __init__(self, field):
        """Initialise with a reference back to the owning MetricField."""
        super().__init__(field)
        self._field = field
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        theme.signals.theme_changed.connect(self.update)

    def paintEvent(self, event):
        """Paint the fill bar, value text, and invalid indicator."""
        T = theme.current()
        f = self._field
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = 6

        # Background
        bg = T['bg_overlay'] if f.isEnabled() else T['bg_base']
        brd = T['accent_blue'] if (f._editor is not None) else (
            T['accent_red'] if f._invalid else T['border_subtle']
        )
        p.setBrush(QColor(bg))
        p.setPen(QPen(QColor(brd), 1))
        p.drawRoundedRect(1, 1, w - 2, h - 2, r, r)

        # Fill bar
        frac = f._value_frac()
        fill_w = int((w - 4) * frac)
        if fill_w > 0 and f.isEnabled():
            p.setBrush(QColor(T['bg_hover']))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(2, 2, fill_w, h - 4, r - 1, r - 1)

        # Value text
        txt = ('{:.%df}' % f._decimals).format(f._value)
        txt_color = T['text_disabled'] if not f.isEnabled() else (
            T['accent_red'] if f._invalid else T['text_primary']
        )
        p.setPen(QColor(txt_color))
        from PyQt5.QtGui import QFont
        font = QFont('JetBrains Mono, Consolas, monospace')
        font.setPointSize(11)
        font.setBold(True)
        p.setFont(font)
        p.drawText(self.rect().adjusted(8, 0, -8, 0), Qt.AlignVCenter | Qt.AlignLeft, txt)

        p.end()


# ---------------------------------------------------------------------------
# CommandedVsActual
# ---------------------------------------------------------------------------

class CommandedVsActual(QWidget):
    """
    MetricField with a DualTrack actual-position overlay.

    Composes an inner ``MetricField`` (commanded) with a ``_CVADualTrack``
    bar showing the live actual position.  Forwards ``committed`` and
    ``edited`` signals.

    ``label`` is the short label (e.g. 'J1', 'X'); ``minimum``/``maximum``
    define the range; ``unit`` is the suffix; ``decimals`` controls precision.
    """

    committed = pyqtSignal(float)
    edited = pyqtSignal(float)

    def __init__(
        self,
        label,
        minimum,
        maximum,
        unit='',
        decimals=2,
        parent=None,
        compact=False,
    ):
        """Initialise with the label, range, and display settings."""
        super().__init__(parent)
        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._actual_val = None

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(3 if compact else 6)

        # Inner MetricField
        init_val = (self._minimum + self._maximum) / 2.0
        self._field = MetricField(
            label, init_val, minimum, maximum,
            unit=unit, decimals=decimals, clamp=True, compact=compact,
        )
        root.addWidget(self._field, 1)

        # DualTrack actual overlay
        self._track = _CVADualTrack(minimum, maximum)
        root.addWidget(self._track)

        # Forward signals
        self._field.committed.connect(self._on_field_committed)
        self._field.edited.connect(self._on_field_edited)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def commanded(self):
        """Return the current commanded value from the inner MetricField."""
        return self._field.value()

    def set_commanded(self, v):
        """Set the commanded value programmatically (updates the marker)."""
        self._field.set_value(v)
        self._track.set_commanded(self._field.value())

    def set_actual(self, v):
        """Update the actual-position overlay on the field and the track."""
        self._actual_val = float(v)
        self._field.set_actual(v)
        self._track.set_actual(v)
        self._track.set_commanded(self._field.value())

    # ------------------------------------------------------------------
    # Signal plumbing
    # ------------------------------------------------------------------

    def _on_field_committed(self, v):
        """Forward committed signal; update track commanded line."""
        self._track.set_commanded(v)
        self.committed.emit(v)

    def _on_field_edited(self, v):
        """Forward edited signal; update track commanded line."""
        self._track.set_commanded(v)
        self.edited.emit(v)


# ---------------------------------------------------------------------------
# _CVADualTrack  (private — CommandedVsActual's ported DualTrack)
# ---------------------------------------------------------------------------

class _CVADualTrack(QFrame):
    """
    Painted bar showing actual fill (accent_green) + commanded tick (accent_blue).

    Ported from ``dash_widgets.DualTrack``; reads ``theme.current()`` in
    ``paintEvent`` and subscribes to ``theme.signals.theme_changed``.
    """

    def __init__(self, range_min=0.0, range_max=1.0, parent=None):
        """Initialise with the value range and subscribe to theme changes."""
        super().__init__(parent)
        self._range_min = float(range_min)
        self._range_max = float(range_max)
        self._actual_frac = 0.5
        self._cmd_frac = 0.5
        self.setFixedHeight(10)
        self.setMinimumWidth(100)
        theme.signals.theme_changed.connect(self.update)

    def set_actual(self, value):
        """Set the actual value (in range units) and repaint."""
        self._actual_frac = self._to_frac(value)
        self.update()

    def set_commanded(self, value):
        """Set the commanded value (in range units) and repaint."""
        self._cmd_frac = self._to_frac(value)
        self.update()

    def _to_frac(self, value):
        """Map value in [range_min, range_max] to a [0, 1] fraction."""
        span = self._range_max - self._range_min
        if span == 0:
            return 0.5
        frac = (float(value) - self._range_min) / span
        return max(0.0, min(1.0, frac))

    def paintEvent(self, event):
        """Paint the dual-track bar using the current theme tokens."""
        T = theme.current()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.setBrush(QColor(T['bg_base']))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(0, 0, w, h, 5, 5)
        fill_w = int(w * self._actual_frac)
        if fill_w > 0:
            p.setBrush(QColor(T['accent_green']))
            p.drawRoundedRect(0, 0, fill_w, h, 5, 5)
        p.setPen(QPen(QColor(T['accent_blue']), 2))
        x = int(w * self._cmd_frac)
        p.drawLine(x, -1, x, h + 1)


# ---------------------------------------------------------------------------
# CollapsibleSection
# ---------------------------------------------------------------------------

class CollapsibleSection(QWidget):
    """
    Collapsible disclosure section with a toggle header button.

    ``title`` is the section label shown in the header button.  The body
    is hidden by default; clicking the header toggles it.  Use
    ``add_widget(w)`` or ``body_layout()`` to populate the interior::

        section = CollapsibleSection('Advanced')
        section.add_widget(some_widget)
        section.body_layout().addLayout(some_layout)
    """

    def __init__(self, title, parent=None):
        """Initialise the section in its collapsed (hidden body) state."""
        super().__init__(parent)
        self._expanded = False

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Header button
        self._header_btn = QPushButton('▸  ' + title)
        self._header_btn.setFlat(True)
        self._header_btn.clicked.connect(self._on_header_clicked)
        root.addWidget(self._header_btn)

        # Body (collapsed by default)
        self._body = QWidget()
        self._body_layout = QVBoxLayout(self._body)
        self._body_layout.setContentsMargins(8, 4, 8, 4)
        self._body_layout.setSpacing(6)
        self._body.setVisible(False)
        root.addWidget(self._body)

        # Theme wiring
        theme.signals.theme_changed.connect(self._apply_style)
        self._apply_style()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_widget(self, widget):
        """Add a widget to the body layout."""
        self._body_layout.addWidget(widget)

    def body_layout(self):
        """Return the QVBoxLayout of the body area for direct layout adds."""
        return self._body_layout

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _on_header_clicked(self):
        """Toggle expanded/collapsed state."""
        self._expanded = not self._expanded
        self._body.setVisible(self._expanded)
        arrow = '▾' if self._expanded else '▸'
        # Extract title from current text (remove old arrow prefix)
        current = self._header_btn.text()
        # Title is everything after the first two characters (arrow + space)
        parts = current.split('  ', 1)
        title = parts[1] if len(parts) > 1 else current
        self._header_btn.setText(arrow + '  ' + title)

    def _apply_style(self, _name=None):
        """Apply theme token colours to the header button and body frame."""
        T = theme.current()
        self._header_btn.setStyleSheet(
            'QPushButton {'
            ' background: %s;'
            ' border: 1px solid %s;'
            ' border-radius: 8px;'
            ' color: %s;'
            ' font-size: 12px;'
            ' font-weight: 700;'
            ' padding: 10px 14px;'
            ' text-align: left;'
            '}' % (T['bg_overlay'], T['border_subtle'], T['text_secondary'])
        )
        self._body.setStyleSheet(
            'QWidget {'
            ' background: %s;'
            ' border: 1px solid %s;'
            ' border-top: none;'
            ' border-bottom-left-radius: 8px;'
            ' border-bottom-right-radius: 8px;'
            '}' % (T['bg_raised'], T['border_subtle'])
        )


# ---------------------------------------------------------------------------
# BreakpointStrip
# ---------------------------------------------------------------------------

class BreakpointStrip(QWidget):
    """
    A horizontal row of N ``MetricField``s for editing breakpoint parameter lists.

    Used for ``error_max`` / ``force_max`` lists whose length tracks ``N_S/2``.
    ``set_count(n)`` rebuilds the row in-place (growing or shrinking); existing
    values are preserved up to the new count.  ``set_values`` / ``values``
    provide list-level access.  ``committed`` fires whenever any child commits.

    Parameters
    ----------
    n : int
        Initial number of fields in the strip.
    minimum : float
        Lower bound for each field.
    maximum : float
        Upper bound for each field.
    unit : str, optional
        Unit suffix shown beside each field (default ``''``).
    decimals : int, optional
        Display precision for each field (default ``2``).
    parent : QWidget, optional
        Parent widget.

    """

    committed = pyqtSignal()

    def __init__(
        self,
        n,
        minimum,
        maximum,
        unit='',
        decimals=2,
        parent=None,
    ):
        """Initialise the strip with ``n`` MetricField widgets."""
        super().__init__(parent)
        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._unit = unit
        self._decimals = decimals

        self._layout = QHBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(4)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self._fields = []
        self._build(n)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_count(self, n):
        """
        Rebuild the strip to exactly ``n`` fields.

        Existing values are preserved for the overlapping indices.
        New fields (when growing) are initialised to the last existing
        value, or to ``minimum`` when the strip was empty.  Calling
        ``set_count(n)`` when the strip already has ``n`` fields is a
        no-op (field references are unchanged).
        """
        current_n = len(self._fields)
        if n == current_n:
            return
        # Snapshot current values before rebuilding.
        old_values = [f.value() for f in self._fields]
        self._build(n, old_values)

    def values(self):
        """Return the current committed float values as a list of length ``n``."""
        return [f.value() for f in self._fields]

    def set_values(self, vals):
        """
        Set field values from a list.

        If ``vals`` is longer than the current count the extras are ignored.
        If ``vals`` is shorter, the last element of ``vals`` is repeated to
        fill remaining fields.  An empty ``vals`` sets all fields to
        ``minimum``.
        """
        pad = float(self._minimum)
        for i, field in enumerate(self._fields):
            if vals:
                v = vals[i] if i < len(vals) else vals[-1]
            else:
                v = pad
            field.set_value(float(v))

    def set_unit(self, unit):
        """Update the unit suffix on the strip and all current fields."""
        self._unit = unit
        for field in self._fields:
            field.set_unit(unit)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _build(self, n, old_values=None):
        """Tear down the current fields and rebuild with ``n`` new fields."""
        old_values = old_values or []

        # Remove all existing widgets from layout and delete them.
        for field in self._fields:
            self._layout.removeWidget(field)
            field.hide()
            field.deleteLater()
        self._fields = []

        last_val = float(self._minimum)
        for i in range(n):
            label = str(i + 1)
            if i < len(old_values):
                init_val = float(old_values[i])
                last_val = init_val
            else:
                init_val = last_val
            field = MetricField(
                label,
                init_val,
                self._minimum,
                self._maximum,
                unit=self._unit,
                decimals=self._decimals,
                clamp=True,
            )
            # Re-emit committed as a no-arg signal.
            field.committed.connect(self._on_child_committed)
            self._layout.addWidget(field)
            self._fields.append(field)

    def _on_child_committed(self, _value):
        """Re-emit the strip-level committed signal when any child commits."""
        self.committed.emit()
