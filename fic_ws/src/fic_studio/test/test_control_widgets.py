"""Tests for control_widgets: MetricField, CommandedVsActual, CollapsibleSection."""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402
import pytest  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# MetricField tests
# ---------------------------------------------------------------------------

class TestMetricFieldClamp:
    """Tests for MetricField clamping behaviour on commit."""

    def test_clamp_true_clamps_to_max(self):
        """When clamp=True, setting 999 with max 180 clamps to 180."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -180.0, 180.0, unit='°', decimals=1, clamp=True)
        committed_values = []
        f.committed.connect(committed_values.append)
        f._commit_text('999')
        assert f.value() == pytest.approx(180.0)
        assert len(committed_values) == 1
        assert committed_values[0] == pytest.approx(180.0)

    def test_clamp_true_clamps_to_min(self):
        """When clamp=True, setting -999 with min -180 clamps to -180."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -180.0, 180.0, unit='°', decimals=1, clamp=True)
        committed_values = []
        f.committed.connect(committed_values.append)
        f._commit_text('-999')
        assert f.value() == pytest.approx(-180.0)
        assert committed_values[0] == pytest.approx(-180.0)

    def test_clamp_false_keeps_value_and_flags_invalid(self):
        """When clamp=False, out-of-range keeps the value but marks invalid."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -180.0, 180.0, unit='°', decimals=1, clamp=False)
        committed_values = []
        f.committed.connect(committed_values.append)
        f._commit_text('999')
        assert f.value() == pytest.approx(999.0)
        assert f._invalid is True
        assert len(committed_values) == 1
        assert committed_values[0] == pytest.approx(999.0)

    def test_clamp_false_in_range_not_invalid(self):
        """When clamp=False, in-range value is not flagged invalid."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -180.0, 180.0, unit='°', decimals=1, clamp=False)
        f._commit_text('90')
        assert f._invalid is False


class TestMetricFieldSetValue:
    """Tests for MetricField set_value / value round-trip."""

    def test_set_value_roundtrip(self):
        """set_value stores and value() returns the same float."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('X', 0.0, -1.0, 1.0, unit='m', decimals=3)
        f.set_value(0.412)
        assert f.value() == pytest.approx(0.412)

    def test_set_value_clamps_when_clamp_true(self):
        """set_value with clamp=True silently clamps the stored value."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('X', 0.0, -1.0, 1.0, unit='m', decimals=3, clamp=True)
        f.set_value(2.0)
        assert f.value() == pytest.approx(1.0)

    def test_set_value_does_not_emit_committed(self):
        """set_value does not emit the committed signal."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('X', 0.0, -1.0, 1.0, unit='m', decimals=3)
        emits = []
        f.committed.connect(emits.append)
        f.set_value(0.5)
        assert emits == []

    def test_initial_value(self):
        """Constructor value argument is stored correctly."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('A', 45.0, -180.0, 180.0, unit='°', decimals=1)
        assert f.value() == pytest.approx(45.0)


class TestMetricFieldScrub:
    """Tests for MetricField _scrub_to fraction mapping."""

    def test_scrub_to_zero_gives_min(self):
        """_scrub_to(0.0) maps to minimum."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        f._scrub_to(0.0)
        assert f.value() == pytest.approx(-90.0)

    def test_scrub_to_one_gives_max(self):
        """_scrub_to(1.0) maps to maximum."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        f._scrub_to(1.0)
        assert f.value() == pytest.approx(90.0)

    def test_scrub_to_half_gives_midpoint(self):
        """_scrub_to(0.5) maps to midpoint of the range."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        f._scrub_to(0.5)
        assert f.value() == pytest.approx(0.0, abs=1e-6)

    def test_scrub_emits_edited(self):
        """_scrub_to emits the edited signal."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        edited_vals = []
        f.edited.connect(edited_vals.append)
        f._scrub_to(0.75)
        assert len(edited_vals) == 1
        assert edited_vals[0] == pytest.approx(45.0)

    def test_scrub_clamped_to_valid_range(self):
        """_scrub_to clamps fractions outside [0, 1]."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, 0.0, 100.0, unit='%', decimals=0)
        f._scrub_to(1.5)
        assert f.value() == pytest.approx(100.0)
        f._scrub_to(-0.5)
        assert f.value() == pytest.approx(0.0)


class TestMetricFieldSetActual:
    """Tests for MetricField set_actual method."""

    def test_set_actual_stores_value(self):
        """set_actual stores the actual value for painting."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        f.set_actual(45.0)
        assert f._actual == pytest.approx(45.0)

    def test_set_actual_none_clears(self):
        """set_actual(None) resets to no actual value."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        f.set_actual(45.0)
        f.set_actual(None)
        assert f._actual is None


class TestMetricFieldSetEnabled:
    """Tests for MetricField setEnabled method."""

    def test_setenabled_false_disables(self):
        """setEnabled(False) disables the widget."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        f.setEnabled(False)
        assert not f.isEnabled()

    def test_setenabled_true_enables(self):
        """setEnabled(True) re-enables the widget."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 0.0, -90.0, 90.0, unit='°', decimals=1)
        f.setEnabled(False)
        f.setEnabled(True)
        assert f.isEnabled()


class TestMetricFieldStep:
    """Tests for MetricField Up/Down key step behaviour."""

    def test_step_up_increments(self):
        """step_up increments value by step."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 10.0, -90.0, 90.0, unit='°', decimals=1, step=5.0)
        f._step_value(+1)
        assert f.value() == pytest.approx(15.0)

    def test_step_down_decrements(self):
        """step_down decrements value by step."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 10.0, -90.0, 90.0, unit='°', decimals=1, step=5.0)
        f._step_value(-1)
        assert f.value() == pytest.approx(5.0)

    def test_step_emits_edited(self):
        """Stepping emits the edited signal."""
        from fic_studio.components.control_widgets import MetricField
        f = MetricField('J1', 10.0, -90.0, 90.0, unit='°', decimals=1, step=5.0)
        vals = []
        f.edited.connect(vals.append)
        f._step_value(+1)
        assert len(vals) == 1
        assert vals[0] == pytest.approx(15.0)


# ---------------------------------------------------------------------------
# CommandedVsActual tests
# ---------------------------------------------------------------------------

class TestCommandedVsActual:
    """Tests for CommandedVsActual composite widget."""

    def test_set_actual_no_error(self):
        """set_actual updates without raising any exception."""
        from fic_studio.components.control_widgets import CommandedVsActual
        w = CommandedVsActual('J1', -90.0, 90.0, unit='°', decimals=1)
        w.set_actual(45.0)   # should not raise

    def test_commanded_reflects_field_value(self):
        """commanded() returns the current MetricField value."""
        from fic_studio.components.control_widgets import CommandedVsActual
        w = CommandedVsActual('X', -1.0, 1.0, unit='m', decimals=3)
        w._field.set_value(0.412)
        assert w.commanded() == pytest.approx(0.412)

    def test_committed_signal_forwarded(self):
        """Committed signal is forwarded from the inner MetricField."""
        from fic_studio.components.control_widgets import CommandedVsActual
        w = CommandedVsActual('J1', -180.0, 180.0, unit='°', decimals=1)
        vals = []
        w.committed.connect(vals.append)
        w._field._commit_text('90')
        assert len(vals) == 1
        assert vals[0] == pytest.approx(90.0)

    def test_edited_signal_forwarded(self):
        """Edited signal is forwarded from the inner MetricField."""
        from fic_studio.components.control_widgets import CommandedVsActual
        w = CommandedVsActual('J1', -90.0, 90.0, unit='°', decimals=1)
        vals = []
        w.edited.connect(vals.append)
        w._field._scrub_to(0.5)
        assert len(vals) == 1

    def test_set_actual_updates_dual_track(self):
        """set_actual updates the dual-track actual fraction."""
        from fic_studio.components.control_widgets import CommandedVsActual
        w = CommandedVsActual('J1', 0.0, 100.0, unit='%', decimals=1)
        w.set_actual(75.0)
        expected_frac = 0.75
        assert w._track._actual_frac == pytest.approx(expected_frac)

    def test_initial_commanded(self):
        """Constructed CommandedVsActual has the initial value as commanded."""
        from fic_studio.components.control_widgets import CommandedVsActual
        w = CommandedVsActual('J1', -90.0, 90.0, unit='°', decimals=1)
        # default initial value is 0.0 (midpoint clamped)
        assert isinstance(w.commanded(), float)


# ---------------------------------------------------------------------------
# CollapsibleSection tests
# ---------------------------------------------------------------------------

class TestCollapsibleSection:
    """Tests for CollapsibleSection toggle widget."""

    def test_collapsed_by_default(self):
        """Content widget is hidden when initially created."""
        from fic_studio.components.control_widgets import CollapsibleSection
        s = CollapsibleSection('Advanced')
        assert s._expanded is False
        assert not s._body.isVisibleTo(s)

    def test_toggle_expands(self):
        """Clicking the header marks the body as visible (setVisible True)."""
        from fic_studio.components.control_widgets import CollapsibleSection
        s = CollapsibleSection('Advanced')
        s._on_header_clicked()
        # isVisible() is False when the top-level window is not shown; use
        # isVisibleTo to check visibility relative to the parent instead.
        assert s._expanded is True
        assert s._body.isVisibleTo(s)

    def test_toggle_collapses(self):
        """Clicking the header again hides the content widget."""
        from fic_studio.components.control_widgets import CollapsibleSection
        s = CollapsibleSection('Advanced')
        s._on_header_clicked()
        s._on_header_clicked()
        assert s._expanded is False
        assert not s._body.isVisibleTo(s)

    def test_add_widget(self):
        """add_widget places the child inside the body layout."""
        from fic_studio.components.control_widgets import CollapsibleSection
        from PyQt5.QtWidgets import QLabel
        s = CollapsibleSection('Settings')
        label = QLabel('hello')
        s.add_widget(label)
        found = False
        layout = s.body_layout()
        for i in range(layout.count()):
            if layout.itemAt(i).widget() is label:
                found = True
        assert found

    def test_body_layout_returns_layout(self):
        """body_layout() returns a QLayout instance."""
        from fic_studio.components.control_widgets import CollapsibleSection
        from PyQt5.QtWidgets import QLayout
        s = CollapsibleSection('Settings')
        assert isinstance(s.body_layout(), QLayout)

    def test_header_text_contains_title(self):
        """Header button text contains the section title."""
        from fic_studio.components.control_widgets import CollapsibleSection
        s = CollapsibleSection('Null-space')
        assert 'Null-space' in s._header_btn.text()


# ---------------------------------------------------------------------------
# Bug-fix regression tests (review findings)
# ---------------------------------------------------------------------------

class TestInlineEditDoubleCommit:
    """Regression tests for the _InlineEdit double-commit bug (Enter path)."""

    def test_enter_does_not_cause_double_commit(self):
        """
        Pressing Enter emits committed exactly once, not twice.

        Previously, keyPressEvent emitted commit_requested, which called
        _commit_text -> _close_editor -> editor.hide().  Qt then synchronously
        fired focusOutEvent (editor still visible when hidden), which emitted
        commit_requested a second time.  The _committing guard prevents this.
        """
        from fic_studio.components.control_widgets import MetricField
        from PyQt5.QtCore import Qt
        from PyQt5.QtGui import QKeyEvent
        from PyQt5.QtCore import QEvent
        f = MetricField('J1', 0.0, -180.0, 180.0, unit='°', decimals=1, clamp=True)
        committed_values = []
        f.committed.connect(committed_values.append)

        # Open the inline editor and simulate Enter.
        f._open_editor()
        editor = f._editor
        assert editor is not None, 'editor should be open'

        # Simulate the Enter key directly on the private _InlineEdit.
        press = QKeyEvent(QEvent.KeyPress, Qt.Key_Return, Qt.NoModifier, '\r')
        editor.keyPressEvent(press)

        # Exactly one committed signal must have fired.
        assert len(committed_values) == 1, (
            'committed should fire exactly once; got %d' % len(committed_values)
        )

    def test_escape_does_not_trigger_commit(self):
        """Pressing Escape cancels without emitting committed."""
        from fic_studio.components.control_widgets import MetricField
        from PyQt5.QtCore import Qt
        from PyQt5.QtGui import QKeyEvent
        from PyQt5.QtCore import QEvent
        f = MetricField('J1', 0.0, -180.0, 180.0, unit='°', decimals=1, clamp=True)
        committed_values = []
        f.committed.connect(committed_values.append)

        f._open_editor()
        editor = f._editor
        press = QKeyEvent(QEvent.KeyPress, Qt.Key_Escape, Qt.NoModifier, '')
        editor.keyPressEvent(press)

        assert committed_values == [], (
            'Escape must not emit committed; got %r' % committed_values
        )


# ---------------------------------------------------------------------------
# BreakpointStrip tests
# ---------------------------------------------------------------------------

class TestBreakpointStripConstruction:
    """Tests for BreakpointStrip construction."""

    def test_construction_creates_n_fields(self):
        """BreakpointStrip(3, ...) creates exactly 3 MetricField children."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(3, 0.0, 1.0)
        assert len(s._fields) == 3

    def test_construction_default_n_fields(self):
        """Strip with n=2 creates exactly 2 fields."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(2, 0.0, 10.0)
        assert len(s._fields) == 2

    def test_initial_values_are_defaults(self):
        """Freshly constructed strip returns n values, all in [min, max]."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(3, 0.0, 9.0)
        vals = s.values()
        assert len(vals) == 3
        for v in vals:
            assert 0.0 <= v <= 9.0


class TestBreakpointStripSetCount:
    """Tests for BreakpointStrip.set_count."""

    def test_set_count_grows_fields(self):
        """set_count(4) on a 2-field strip leaves exactly 4 fields."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(2, 0.0, 1.0)
        s.set_count(4)
        assert len(s._fields) == 4

    def test_set_count_shrinks_fields(self):
        """set_count(1) on a 3-field strip leaves exactly 1 field."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(3, 0.0, 1.0)
        s.set_count(1)
        assert len(s._fields) == 1

    def test_set_count_to_same_is_noop(self):
        """set_count(n) when already n fields keeps field references stable."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(2, 0.0, 1.0)
        original_ids = [id(f) for f in s._fields]
        s.set_count(2)
        new_ids = [id(f) for f in s._fields]
        assert original_ids == new_ids

    def test_set_count_values_length_matches(self):
        """After set_count(n), values() returns a list of length n."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(2, 0.0, 1.0)
        s.set_count(5)
        assert len(s.values()) == 5


class TestBreakpointStripValuesRoundTrip:
    """Tests for set_values / values round-trip."""

    def test_set_values_roundtrip(self):
        """set_values([0.1, 0.2, 0.3]) is returned exactly by values()."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(3, 0.0, 1.0)
        s.set_values([0.1, 0.2, 0.3])
        result = s.values()
        assert result == pytest.approx([0.1, 0.2, 0.3])

    def test_set_values_truncates_to_count(self):
        """set_values with more elements than count ignores extras."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(2, 0.0, 1.0)
        s.set_values([0.1, 0.2, 0.9])
        assert len(s.values()) == 2
        assert s.values() == pytest.approx([0.1, 0.2])

    def test_set_values_pads_with_last_when_short(self):
        """set_values with fewer elements than count pads with the last value."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(4, 0.0, 1.0)
        s.set_values([0.3, 0.6])
        result = s.values()
        assert len(result) == 4
        assert result[0] == pytest.approx(0.3)
        assert result[1] == pytest.approx(0.6)
        assert result[2] == pytest.approx(0.6)
        assert result[3] == pytest.approx(0.6)

    def test_set_values_empty_pads_with_minimum(self):
        """set_values([]) pads all fields with minimum."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(3, 0.5, 5.0)
        s.set_values([])
        result = s.values()
        for v in result:
            assert v == pytest.approx(0.5)

    def test_values_returns_floats(self):
        """values() always returns a list of floats."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(2, 0.0, 1.0)
        s.set_values([0.25, 0.75])
        for v in s.values():
            assert isinstance(v, float)


class TestBreakpointStripCommittedSignal:
    """Tests for BreakpointStrip.committed signal emission."""

    def test_committed_fires_when_child_commits(self):
        """Signal fires when any child MetricField commits."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(3, 0.0, 1.0)
        fired = []
        s.committed.connect(lambda: fired.append(True))
        # Simulate a commit on the first child field.
        s._fields[0]._commit_text('0.5')
        assert len(fired) == 1

    def test_committed_fires_for_each_child(self):
        """Signal fires independently for each child that commits."""
        from fic_studio.components.control_widgets import BreakpointStrip
        s = BreakpointStrip(3, 0.0, 1.0)
        fired = []
        s.committed.connect(lambda: fired.append(True))
        s._fields[0]._commit_text('0.1')
        s._fields[1]._commit_text('0.5')
        s._fields[2]._commit_text('0.9')
        assert len(fired) == 3


class TestBreakpointStripExported:
    """Tests that BreakpointStrip is exported from the components package."""

    def test_importable_from_components(self):
        """Class can be imported from fic_studio.components."""
        from fic_studio.components import BreakpointStrip
        assert BreakpointStrip is not None

    def test_in_all(self):
        """Name appears in components.__all__."""
        import fic_studio.components as comp
        assert 'BreakpointStrip' in comp.__all__
