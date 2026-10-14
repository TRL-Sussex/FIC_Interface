"""TelemetryTable — read-only numeric readout for the Dashboard.

Renders one labelled row per channel (a joint, or an end-effector
quantity) across a fixed set of columns, in the grouped numeric style the
Kinova Web App's Monitoring page uses.

Rows are addressed by *name*, not index, so a robot that reports more
than its actuated joints (the CSPM's passive and platform joints) tables
correctly. A channel with no live source renders ``--`` rather than a
fabricated zero.

``set_rows`` rebuilds only when the label list actually changes: the
Dashboard drives this at UI rate and must not churn widgets per frame.
"""

import math

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from fic_studio import theme

_MONO = "'JetBrains Mono', 'Consolas', monospace"

PLACEHOLDER = '--'

#: Width of one value column. Fixed rather than stretched: stretched
#: columns spread a handful of numbers across the whole window and the
#: reader has to track a long way from row label to value.
COLUMN_WIDTH = 104


def _cell_label(text, token, bold=False, width=COLUMN_WIDTH):
    """Build a monospaced, theme-bound value cell."""
    q = QLabel(text)
    q.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    q.setFixedWidth(width)
    # border:none is not decoration — QLabel derives from QFrame, so any
    # ancestor styling QFrame draws a box around every cell without it.
    theme.bind(q, lambda T, tk=token, b=bold: (
        'background: transparent; border: none; font-family: %s;'
        ' font-size: 12px; color: %s;%s'
        % (_MONO, T[tk], ' font-weight: 700;' if b else '')
    ))
    return q


class TelemetryTable(QWidget):
    """A fixed-column, name-addressed grid of read-only numeric cells."""

    def __init__(self, columns, decimals=2, label_width=110, parent=None,
                 column_width=COLUMN_WIDTH):
        """Build an empty table with the given column headings."""
        super().__init__(parent)
        self._columns = list(columns)
        self._decimals = int(decimals)
        self._label_width = label_width
        self._column_width = column_width
        self._rows = {}
        self._order = []

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(2)

        root.addLayout(self._build_header())

        self._rows_layout = QVBoxLayout()
        self._rows_layout.setContentsMargins(0, 0, 0, 0)
        self._rows_layout.setSpacing(0)
        root.addLayout(self._rows_layout)

    # ── Construction ──────────────────────────────────────────────────────

    def _build_header(self):
        """Return the heading row layout."""
        row = QHBoxLayout()
        row.setSpacing(9)
        spacer = QLabel('')
        spacer.setFixedWidth(self._label_width)
        theme.bind(spacer, lambda T: 'background: transparent; border: none;')
        row.addWidget(spacer)
        for name in self._columns:
            head = QLabel(name)
            head.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            head.setFixedWidth(self._column_width)
            theme.bind(head, lambda T: (
                'background: transparent; border: none; font-size: 10px;'
                ' color: %s;' % T['text_disabled']
            ))
            row.addWidget(head)
        row.addStretch(1)
        return row

    def header_labels(self):
        """Return the column headings, in order."""
        return list(self._columns)

    def set_rows(self, labels):
        """Show exactly ``labels``, rebuilding only if the set changed."""
        labels = list(labels)
        if labels == self._order:
            return
        while self._rows_layout.count():
            item = self._rows_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.setParent(None)
        self._rows = {}
        self._order = labels
        for label in labels:
            self._rows[label] = self._append_row(label)

    def _append_row(self, label):
        """Create one row and return its {'widget', 'cells'} record."""
        holder = QWidget()
        row = QHBoxLayout(holder)
        row.setContentsMargins(0, 2, 0, 2)
        row.setSpacing(9)

        name = QLabel(label)
        name.setFixedWidth(self._label_width)
        name.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        theme.bind(name, lambda T: (
            'background: transparent; border: none; font-size: 12px;'
            ' color: %s;' % T['text_secondary']
        ))
        row.addWidget(name)

        cells = {}
        for col in self._columns:
            cell = _cell_label(PLACEHOLDER, 'text_primary',
                               width=self._column_width)
            row.addWidget(cell)
            cells[col] = cell
        row.addStretch(1)

        self._rows_layout.addWidget(holder)
        return {'widget': holder, 'cells': cells}

    # ── Values ────────────────────────────────────────────────────────────

    def _fmt(self, value):
        """Format one value, or the placeholder if it is absent/non-finite."""
        if value is None:
            return PLACEHOLDER
        try:
            v = float(value)
        except (TypeError, ValueError):
            return PLACEHOLDER
        if not math.isfinite(v):
            return PLACEHOLDER
        return '%.*f' % (self._decimals, v)

    def set_values(self, label, values):
        """Write ``values`` across the row named ``label``, left to right.

        Unknown labels are ignored: telemetry can arrive for joints the
        active robot's table does not show.
        """
        record = self._rows.get(label)
        if record is None:
            return
        for col, value in zip(self._columns, values):
            record['cells'][col].setText(self._fmt(value))

    # ── Introspection ─────────────────────────────────────────────────────

    def row_count(self):
        """Return how many rows are currently shown."""
        return len(self._order)

    def row_widget(self, label):
        """Return the container widget for one row, or None."""
        record = self._rows.get(label)
        return None if record is None else record['widget']

    def cell_text(self, label, column):
        """Return the rendered text of one cell, or None if it is absent."""
        record = self._rows.get(label)
        if record is None:
            return None
        cell = record['cells'].get(column)
        return None if cell is None else cell.text()
