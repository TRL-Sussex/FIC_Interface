"""
JointTable widget — dynamic per-joint row editor for the Robot Config page.

Provides ``JointTable``, a ``QWidget`` wrapping a ``QVBoxLayout`` of joint
row widgets.  Each row contains:

  - A ``#N`` index label
  - A name ``QLineEdit`` (joint URDF name)
  - A ``MetricField`` for *lower* angle limit
  - A ``MetricField`` for *upper* angle limit
  - A ``MetricField`` for *torque_max* (one-sided magnitude)

``set_count(n)`` adds or removes rows while preserving existing values.
``rows()`` returns a ``list[dict]`` with keys
``{'name', 'lower', 'upper', 'torque_max'}``.
``set_rows(list[dict])`` sets the count and populates all fields.
The ``changed`` signal fires whenever any child field is edited.
"""

from fic_studio import theme
from fic_studio.components.control_widgets import MetricField
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


# Default numeric ranges for the three MetricField types.
_LOWER_MIN = -360.0
_LOWER_MAX = 360.0
_UPPER_MIN = -360.0
_UPPER_MAX = 360.0
_TAU_MIN = 0.0
_TAU_MAX = 999.0


class JointTable(QWidget):
    """
    Dynamic table of joint-definition rows for the Robot Config page.

    Each row holds a joint name ``QLineEdit`` and three ``MetricField``
    inputs (lower angle limit, upper angle limit, torque_max).

    Parameters
    ----------
    unit_deg : str, optional
        Unit suffix for angle fields (default ``'°'``).
    unit_tau : str, optional
        Unit suffix for torque field (default ``'Nm'``).
    parent : QWidget, optional
        Parent widget.

    """

    changed = pyqtSignal()

    def __init__(self, unit_deg='°', unit_tau='Nm', parent=None):
        """Initialise an empty JointTable with the given unit suffixes."""
        super().__init__(parent)
        self._unit_deg = unit_deg
        self._unit_tau = unit_tau

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(2)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        # List of row-descriptor dicts (one per row in display order).
        # Each dict: {'widget': QWidget, 'name_edit': QLineEdit,
        #             'lower_field': MetricField, 'upper_field': MetricField,
        #             'torque_field': MetricField}
        self._row_widgets = []

        # Header row
        self._header = self._build_header()
        self._layout.addWidget(self._header)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_count(self, n):
        """
        Resize the table to exactly ``n`` rows.

        When growing, new rows are appended with blank names and default
        numeric values.  Existing row values are preserved for the
        overlapping indices.  When shrinking, tail rows are removed.
        Calling ``set_count(n)`` when the table already has ``n`` rows is
        a no-op.
        """
        current = len(self._row_widgets)
        if n == current:
            return
        if n < current:
            # Remove tail rows.
            for i in range(current - 1, n - 1, -1):
                desc = self._row_widgets.pop(i)
                desc['widget'].hide()
                self._layout.removeWidget(desc['widget'])
                desc['widget'].deleteLater()
        else:
            # Append new rows.
            for i in range(current, n):
                self._append_row(i)

    def rows(self):
        """
        Return the current row data as a list of dicts.

        Each dict has keys: ``name`` (str), ``lower`` (float),
        ``upper`` (float), ``torque_max`` (float).
        """
        result = []
        for desc in self._row_widgets:
            result.append({
                'name': desc['name_edit'].text(),
                'lower': desc['lower_field'].value(),
                'upper': desc['upper_field'].value(),
                'torque_max': desc['torque_field'].value(),
            })
        return result

    def set_rows(self, data):
        """
        Populate the table from a list of dicts.

        Each dict should have keys matching ``rows()`` output:
        ``{'name', 'lower', 'upper', 'torque_max'}``.
        The row count is set to ``len(data)`` before populating.

        Parameters
        ----------
        data : list[dict]
            Row data.  Missing keys default to empty string / ``0.0``.

        """
        self.set_count(len(data))
        for i, item in enumerate(data):
            desc = self._row_widgets[i]
            desc['name_edit'].setText(str(item.get('name', '')))
            desc['lower_field'].set_value(float(item.get('lower', 0.0)))
            desc['upper_field'].set_value(float(item.get('upper', 0.0)))
            desc['torque_field'].set_value(float(item.get('torque_max', 0.0)))

    # ------------------------------------------------------------------
    # Internal — row construction
    # ------------------------------------------------------------------

    def _build_header(self):
        """Build and return the column header row widget."""
        w = QWidget()
        layout = QHBoxLayout(w)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(6)

        lbl_idx = QLabel('#')
        lbl_idx.setFixedWidth(26)
        lbl_idx.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_idx)

        lbl_name = QLabel('Joint Name')
        lbl_name.setFixedWidth(140)
        layout.addWidget(lbl_name)

        lbl_lower = QLabel('Lower (%s)' % self._unit_deg)
        layout.addWidget(lbl_lower, 1)

        lbl_upper = QLabel('Upper (%s)' % self._unit_deg)
        layout.addWidget(lbl_upper, 1)

        lbl_tau = QLabel('Torque Max (%s)' % self._unit_tau)
        layout.addWidget(lbl_tau, 1)

        for lbl in (lbl_idx, lbl_name, lbl_lower, lbl_upper, lbl_tau):
            theme.bind(
                lbl,
                lambda t: (
                    'color: %s; font-size: 11px; font-weight: 700;'
                    % t['text_secondary']
                ),
            )
        return w

    def _append_row(self, index):
        """Build one row widget for joint index and append to the layout."""
        row_w = QWidget()
        layout = QHBoxLayout(row_w)
        layout.setContentsMargins(4, 1, 4, 1)
        layout.setSpacing(6)

        # Index label
        idx_lbl = QLabel(str(index + 1))
        idx_lbl.setFixedWidth(26)
        idx_lbl.setAlignment(Qt.AlignCenter)
        theme.bind(
            idx_lbl,
            lambda t: (
                'color: %s; font-size: 11px; font-weight: 600;'
                % t['text_disabled']
            ),
        )
        layout.addWidget(idx_lbl)

        # Name QLineEdit
        name_edit = QLineEdit()
        name_edit.setFixedWidth(140)
        name_edit.setPlaceholderText('joint%d' % (index + 1))
        name_edit.textChanged.connect(self._on_changed)
        layout.addWidget(name_edit)

        # Lower angle limit
        lower_field = MetricField(
            '',
            0.0,
            _LOWER_MIN,
            _LOWER_MAX,
            unit=self._unit_deg,
            decimals=1,
            clamp=True,
        )
        lower_field.committed.connect(self._on_changed)
        lower_field.edited.connect(self._on_changed)
        layout.addWidget(lower_field, 1)

        # Upper angle limit
        upper_field = MetricField(
            '',
            0.0,
            _UPPER_MIN,
            _UPPER_MAX,
            unit=self._unit_deg,
            decimals=1,
            clamp=True,
        )
        upper_field.committed.connect(self._on_changed)
        upper_field.edited.connect(self._on_changed)
        layout.addWidget(upper_field, 1)

        # Torque max (one-sided, non-negative)
        torque_field = MetricField(
            '',
            0.0,
            _TAU_MIN,
            _TAU_MAX,
            unit=self._unit_tau,
            decimals=2,
            clamp=True,
        )
        torque_field.committed.connect(self._on_changed)
        torque_field.edited.connect(self._on_changed)
        layout.addWidget(torque_field, 1)

        self._layout.addWidget(row_w)

        desc = {
            'widget': row_w,
            'idx_label': idx_lbl,
            'name_edit': name_edit,
            'lower_field': lower_field,
            'upper_field': upper_field,
            'torque_field': torque_field,
        }
        self._row_widgets.append(desc)

    # ------------------------------------------------------------------
    # Signal plumbing
    # ------------------------------------------------------------------

    def _on_changed(self, *_args):
        """Emit ``changed`` when any child widget fires its change signal."""
        self.changed.emit()
