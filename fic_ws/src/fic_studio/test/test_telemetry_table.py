"""Tests for TelemetryTable: the Dashboard's read-only numeric readout.

TelemetryTable renders one labelled row per channel (a joint, or an
end-effector quantity) across a fixed set of columns. It replaces the
graphical DualTrack/LimitBar rows with the grouped numeric presentation
the Kinova Web App's Monitoring page uses.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from fic_studio.components.telemetry_table import TelemetryTable  # noqa: E402

_app = QApplication.instance() or QApplication([])

COLUMNS = ['commanded', 'position', 'error', 'velocity', 'torque']


def _table(decimals=2):
    return TelemetryTable(columns=COLUMNS, decimals=decimals)


class TestStructure:
    """Row and column construction."""

    def test_header_lists_every_column(self):
        """The header row carries one heading per configured column."""
        assert _table().header_labels() == COLUMNS

    def test_set_rows_creates_one_row_per_label(self):
        """Seven joint labels produce seven rows."""
        t = _table()
        t.set_rows(['j%d' % i for i in range(1, 8)])
        assert t.row_count() == 7

    def test_set_rows_shrinks_and_grows_with_joint_count(self):
        """Switching to a robot with fewer joints rebuilds to that count."""
        t = _table()
        t.set_rows(['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7'])
        t.set_rows(['j1', 'j2', 'j3'])
        assert t.row_count() == 3

    def test_repeated_set_rows_reuses_widgets(self):
        """Re-setting identical labels must not rebuild — this runs at 20 Hz."""
        t = _table()
        t.set_rows(['j1', 'j2'])
        first = t.row_widget('j1')
        t.set_rows(['j1', 'j2'])
        assert t.row_widget('j1') is first

    def test_arbitrary_labels_supported(self):
        """Rows are labelled by name, so CSPM's passive joints work too."""
        t = _table()
        t.set_rows(['platform_roll', 'passive_1'])
        assert t.row_count() == 2


class TestFormatting:
    """Value rendering."""

    def test_values_render_at_configured_decimals(self):
        """A 2-decimal table renders 12.041 as '12.04'."""
        t = _table(decimals=2)
        t.set_rows(['j1'])
        t.set_values('j1', [12.0, 12.041, 0.041, -0.31, 2.11])
        assert t.cell_text('j1', 'position') == '12.04'

    def test_decimals_are_configurable(self):
        """A 3-decimal table renders the same input as '12.041'."""
        t = _table(decimals=3)
        t.set_rows(['j1'])
        t.set_values('j1', [12.0, 12.041, 0.041, -0.31, 2.11])
        assert t.cell_text('j1', 'position') == '12.041'

    def test_none_renders_as_placeholder(self):
        """A channel with no live source shows '--', never a fabricated 0."""
        t = _table()
        t.set_rows(['j1'])
        t.set_values('j1', [None, None, None, None, None])
        assert t.cell_text('j1', 'commanded') == '--'

    def test_non_finite_renders_as_placeholder(self):
        """NaN and inf show '--' rather than 'nan' / 'inf'."""
        t = _table()
        t.set_rows(['j1'])
        t.set_values('j1', [float('nan'), float('inf'), 0.0, 0.0, 0.0])
        assert t.cell_text('j1', 'commanded') == '--'
        assert t.cell_text('j1', 'position') == '--'

    def test_rows_start_as_placeholders(self):
        """A freshly built row reads '--' before any telemetry arrives."""
        t = _table()
        t.set_rows(['j1'])
        assert t.cell_text('j1', 'torque') == '--'

    def test_short_value_list_leaves_remaining_columns_untouched(self):
        """Fewer values than columns fills what it can without raising."""
        t = _table()
        t.set_rows(['j1'])
        t.set_values('j1', [1.0, 2.0])
        assert t.cell_text('j1', 'commanded') == '1.00'
        assert t.cell_text('j1', 'error') == '--'

    def test_unknown_row_is_ignored(self):
        """Telemetry for a joint the table does not show must not raise."""
        t = _table()
        t.set_rows(['j1'])
        t.set_values('j9', [1.0, 2.0, 3.0, 4.0, 5.0])   # must not raise
        assert t.cell_text('j9', 'position') is None
