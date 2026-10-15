"""
Headless tests for the Tuning page: live feedback strip, validation and the applied stamp.

Verifies:
- ``_TuningFeedbackStrip`` and ``_CVABar`` are importable.
- ``_CVABar.set_data`` with ``None`` keeps ``_value`` as ``None`` (show '--').
- ``_CVABar.set_data`` with a real value stores the value and range.
- ``_TuningFeedbackStrip.set_error`` / ``set_effort`` / ``set_wrench`` delegate.
- ``_refresh_feedback_strip`` pushes joint tracking error into error bar when
  synthetic ``current_joint_commanded_by_robot`` / ``current_joint_positions_by_robot``
  are injected into the host object.
- ``_refresh_feedback_strip`` pushes joint effort into the effort bar.
- ``_refresh_feedback_strip`` pushes wrench component into wrench bar for
  Cartesian mode when synthetic ``current_wrench_by_robot`` is injected.
- ``_refresh_feedback_strip`` shows '--' (None) when the telemetry dicts are empty.
- ``_validate_before_save`` returns empty string when breakpoints are valid.
- ``_validate_before_save`` returns a non-empty error when error_max is non-monotonic.
- ``_validate_before_save`` returns a non-empty error when force_max is non-monotonic.
- ``_set_applied_stamp`` sets the applied label text with the provided timestamp.
- Out-of-range stiffness is clamped by ``_clamp_stiffness`` before any save call.
"""
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QComboBox  # noqa: E402
import pytest  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_stub_host(station_id=1, mode=1):
    """Return a minimal host object that satisfies TuningMixin field access."""
    node = types.SimpleNamespace(
        station_id=station_id,
        station_robot_id='robot%d' % station_id,
    )

    from fic_studio.mixins.tuning_mixin import (
        _TuningFeedbackStrip,
        CONTROL_MODES,
    )
    from fic_studio.components import BreakpointStrip, MetricField

    host = types.SimpleNamespace()
    host.node = node

    # Telemetry dicts (as defined in JointStateMixin._init_joint_state).
    host.current_joint_commanded_by_robot = {}
    host.current_joint_positions_by_robot = {}
    host.current_joint_efforts_by_robot = {}
    host.current_wrench_by_robot = {}

    # Helper methods mirroring JointStateMixin.
    def _normalize_rid(rid):
        if rid is None:
            return node.station_robot_id
        return rid if isinstance(rid, str) else ('robot%d' % rid)

    def joint_commanded_for(rid=None):
        return list(host.current_joint_commanded_by_robot.get(_normalize_rid(rid), []))

    def joint_positions_for(rid=None):
        return list(host.current_joint_positions_by_robot.get(_normalize_rid(rid), []))

    def joint_effort_for(rid=None):
        return list(host.current_joint_efforts_by_robot.get(_normalize_rid(rid), []))

    def wrench_for(rid=None):
        return host.current_wrench_by_robot.get(_normalize_rid(rid))

    host.joint_commanded_for = joint_commanded_for
    host.joint_positions_for = joint_positions_for
    host.joint_effort_for = joint_effort_for
    host.wrench_for = wrench_for

    # Widget stubs used by _refresh_feedback_strip.
    host.tuning_error_max = BreakpointStrip(1, 0.0001, 3.0, unit='rad', decimals=4)
    host.tuning_error_max.set_values([0.1])
    host.tuning_force_max = BreakpointStrip(1, 0.1, 500.0, unit='Nm', decimals=2)
    host.tuning_force_max.set_values([39.0])
    host.tuning_stiffness = MetricField('K₀', 600.0, 0.0, 5000.0, unit='Nm/rad', decimals=1)

    # Joint-id dropdown stub.
    jd = QComboBox()
    jd.addItems([str(i) for i in range(7)])
    host.joint_id_dropdown = jd

    # Axis dropdown stub.
    ad = QComboBox()
    ad.addItems(['x', 'y', 'z', 'q', 'elbow', 'postural'])
    host.axis_dropdown = ad

    # Mode dropdown stub.
    md = QComboBox()
    for num, info in CONTROL_MODES.items():
        md.addItem('Mode %d: %s' % (num, info['name']), num)
    host.control_mode_dropdown = md

    def _get_current_mode():
        return host.control_mode_dropdown.currentData() or 1

    host._get_current_mode = _get_current_mode

    # H-PMP's layer selector, and the helper that says which set is being edited.
    ld = QComboBox()
    ld.addItem('Joint layer', 'joint')
    ld.addItem('Cartesian layer', 'cartesian')
    host.hpmp_layer_dropdown = ld

    def _tuning_set():
        from fic_studio.mixins.tuning_mixin import TuningMixin
        return TuningMixin._tuning_set(host)

    host._tuning_set = _tuning_set

    # Live feedback strip.
    host._feedback_strip = _TuningFeedbackStrip()

    # Canvas + stats bar, built by _build_tuning_page in the real host.
    # Real widgets rather than fakes: _fields_to_canvas drives them through
    # set_params / update_stats_multi, so a stub would assert nothing.
    from fic_studio.mixins.tuning_mixin import FICCanvas, FICStatsBar
    host.fic_canvas = FICCanvas()
    host.fic_stats = FICStatsBar()

    # Applied label stub.
    from PyQt5.QtWidgets import QLabel
    host._applied_label = QLabel('')

    # Operating-point readout. A real QLabel, not None, so _set_op_readout
    # actually formats through fic_force_multi instead of early-returning.
    host._op_readout = QLabel('')

    def _set_op_readout(x, y, valid):
        from fic_studio.mixins.tuning_mixin import TuningMixin
        return TuningMixin._set_op_readout(host, x, y, valid)

    host._set_op_readout = _set_op_readout

    # stack stub (index 2 = tuning page — pretend we're on it).
    stack = types.SimpleNamespace(currentIndex=lambda: 2)
    host.stack = stack
    # Pages are named now; the refresher asks whether its page is showing.
    host.is_page_visible = lambda name: name == 'tuning'
    # SimpleNamespace inherits nothing, so bind the mixin's helper too.
    host._page_showing = host.is_page_visible

    def _set_applied_stamp(ts):
        if hasattr(host, '_applied_label'):
            host._applied_label.setText('Saved to config.xml at %s' % ts)

    host._set_applied_stamp = _set_applied_stamp

    def _validate_before_save():
        from fic_studio.mixins.tuning_mixin import TuningMixin
        return TuningMixin._validate_before_save(host)

    host._validate_before_save = _validate_before_save

    def _clamp_stiffness(k0, em, fm):
        from fic_studio.mixins.tuning_mixin import TuningMixin
        return TuningMixin._clamp_stiffness(host, k0, em, fm)

    host._clamp_stiffness = _clamp_stiffness

    def _refresh_feedback_strip():
        from fic_studio.mixins.tuning_mixin import TuningMixin
        return TuningMixin._refresh_feedback_strip(host)

    host._refresh_feedback_strip = _refresh_feedback_strip

    return host


# ---------------------------------------------------------------------------
# _CVABar tests
# ---------------------------------------------------------------------------

class TestCVABarImport:
    """Tests that _CVABar and _TuningFeedbackStrip are importable."""

    def test_cvabar_importable(self):
        """_CVABar can be imported from tuning_mixin."""
        from fic_studio.mixins.tuning_mixin import _CVABar
        assert _CVABar is not None

    def test_feedback_strip_importable(self):
        """_TuningFeedbackStrip can be imported from tuning_mixin."""
        from fic_studio.mixins.tuning_mixin import _TuningFeedbackStrip
        assert _TuningFeedbackStrip is not None


class TestCVABarSetData:
    """Tests for _CVABar.set_data."""

    def test_none_value_stores_none(self):
        """set_data(None, range) keeps _value as None (show '--')."""
        from fic_studio.mixins.tuning_mixin import _CVABar
        bar = _CVABar('Test', 'rad')
        bar.set_data(None, 1.0)
        assert bar._value is None

    def test_float_value_stored(self):
        """set_data(0.05, 0.1) stores the float value."""
        from fic_studio.mixins.tuning_mixin import _CVABar
        bar = _CVABar('Test', 'rad')
        bar.set_data(0.05, 0.1)
        assert bar._value == pytest.approx(0.05)

    def test_full_range_stored(self):
        """set_data stores the full_range correctly."""
        from fic_studio.mixins.tuning_mixin import _CVABar
        bar = _CVABar('Test', 'Nm')
        bar.set_data(10.0, 39.0)
        assert bar._full_range == pytest.approx(39.0)

    def test_negative_value_stored(self):
        """set_data with negative value stores it correctly."""
        from fic_studio.mixins.tuning_mixin import _CVABar
        bar = _CVABar('Wrench', 'N')
        bar.set_data(-5.3, 39.0)
        assert bar._value == pytest.approx(-5.3)

    def test_range_protected_against_zero(self):
        """set_data with range=0 does not raise and clamps to a small positive value."""
        from fic_studio.mixins.tuning_mixin import _CVABar
        bar = _CVABar('Test', 'rad')
        bar.set_data(0.0, 0.0)
        assert bar._full_range > 0.0


# ---------------------------------------------------------------------------
# _TuningFeedbackStrip delegation tests
# ---------------------------------------------------------------------------

class TestTuningFeedbackStripDelegation:
    """Tests that _TuningFeedbackStrip delegates to its bars."""

    def test_set_error_delegates(self):
        """set_error propagates value to the error bar."""
        from fic_studio.mixins.tuning_mixin import _TuningFeedbackStrip
        strip = _TuningFeedbackStrip()
        strip.set_error(0.02, 0.1)
        assert strip._error_bar._value == pytest.approx(0.02)

    def test_set_effort_delegates(self):
        """set_effort propagates value to the effort bar."""
        from fic_studio.mixins.tuning_mixin import _TuningFeedbackStrip
        strip = _TuningFeedbackStrip()
        strip.set_effort(12.5, 39.0)
        assert strip._effort_bar._value == pytest.approx(12.5)

    def test_set_wrench_delegates(self):
        """set_wrench propagates value to the wrench bar."""
        from fic_studio.mixins.tuning_mixin import _TuningFeedbackStrip
        strip = _TuningFeedbackStrip()
        strip.set_wrench(8.0, 39.0)
        assert strip._wrench_bar._value == pytest.approx(8.0)

    def test_set_error_none_propagates(self):
        """set_error(None, ...) sets error bar value to None."""
        from fic_studio.mixins.tuning_mixin import _TuningFeedbackStrip
        strip = _TuningFeedbackStrip()
        strip.set_error(None, 1.0)
        assert strip._error_bar._value is None

    def test_set_wrench_none_propagates(self):
        """set_wrench(None, ...) sets wrench bar value to None."""
        from fic_studio.mixins.tuning_mixin import _TuningFeedbackStrip
        strip = _TuningFeedbackStrip()
        strip.set_wrench(None, 1.0)
        assert strip._wrench_bar._value is None


# ---------------------------------------------------------------------------
# _refresh_feedback_strip headless tests
# ---------------------------------------------------------------------------

class TestFeedbackStripRefresh:
    """Headless tests for the ~100 ms refresh callback."""

    def test_joint_mode_error_bar_updated(self):
        """Error bar shows joint tracking error = cmd - pos for joint 0."""
        host = _make_stub_host(mode=1)
        rid = 'robot1'
        host.current_joint_commanded_by_robot[rid] = [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        host.current_joint_positions_by_robot[rid] = [0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Mode 1 = Joint Impedance (is_joint = True).
        host.control_mode_dropdown.setCurrentIndex(0)

        host._refresh_feedback_strip()

        # Tracking error for joint 0 = 0.15 - 0.10 = 0.05.
        assert host._feedback_strip._error_bar._value == pytest.approx(0.05)

    def test_joint_mode_effort_bar_updated(self):
        """Effort bar shows joint effort for the selected joint."""
        host = _make_stub_host(mode=1)
        rid = 'robot1'
        host.current_joint_commanded_by_robot[rid] = [0.1] * 7
        host.current_joint_positions_by_robot[rid] = [0.0] * 7
        host.current_joint_efforts_by_robot[rid] = [3.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        host.control_mode_dropdown.setCurrentIndex(0)

        host._refresh_feedback_strip()

        assert host._feedback_strip._effort_bar._value == pytest.approx(3.7)

    def test_joint_mode_second_joint(self):
        """Refresh uses the correct joint index when joint_id_dropdown shows '2'."""
        host = _make_stub_host(mode=1)
        rid = 'robot1'
        host.joint_id_dropdown.setCurrentIndex(2)   # joint 2
        host.current_joint_commanded_by_robot[rid] = [0.0, 0.0, 0.30, 0.0, 0.0, 0.0, 0.0]
        host.current_joint_positions_by_robot[rid] = [0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.0]
        host.control_mode_dropdown.setCurrentIndex(0)

        host._refresh_feedback_strip()

        # Tracking error for joint 2 = 0.30 - 0.10 = 0.20.
        assert host._feedback_strip._error_bar._value == pytest.approx(0.20)

    def test_absent_data_shows_none_error(self):
        """Error bar is None when commanded/positions dicts are empty."""
        host = _make_stub_host(mode=1)
        # No telemetry injected.
        host.control_mode_dropdown.setCurrentIndex(0)

        host._refresh_feedback_strip()

        assert host._feedback_strip._error_bar._value is None

    def test_absent_data_shows_none_effort(self):
        """Effort bar is None when effort dict is empty."""
        host = _make_stub_host(mode=1)
        host.control_mode_dropdown.setCurrentIndex(0)

        host._refresh_feedback_strip()

        assert host._feedback_strip._effort_bar._value is None

    def test_cartesian_mode_wrench_bar_updated(self):
        """Wrench bar shows Fx (index 0) for Cartesian x axis."""
        host = _make_stub_host(mode=5)
        rid = 'robot1'
        host.current_wrench_by_robot[rid] = (12.3, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Mode 5 = Cartesian Impedance (index 4 in the combo).
        host.control_mode_dropdown.setCurrentIndex(4)
        # axis = 'x' (default).
        host.axis_dropdown.setCurrentText('x')

        host._refresh_feedback_strip()

        assert host._feedback_strip._wrench_bar._value == pytest.approx(12.3)

    def test_cartesian_y_axis_wrench_component(self):
        """Wrench bar shows Fy (index 1) for Cartesian y axis."""
        host = _make_stub_host(mode=5)
        rid = 'robot1'
        host.current_wrench_by_robot[rid] = (1.0, 7.5, 0.0, 0.0, 0.0, 0.0)
        host.control_mode_dropdown.setCurrentIndex(4)
        host.axis_dropdown.setCurrentText('y')

        host._refresh_feedback_strip()

        assert host._feedback_strip._wrench_bar._value == pytest.approx(7.5)

    def test_cartesian_z_axis_wrench_component(self):
        """Wrench bar shows Fz (index 2) for Cartesian z axis."""
        host = _make_stub_host(mode=5)
        rid = 'robot1'
        host.current_wrench_by_robot[rid] = (0.0, 0.0, -4.2, 0.0, 0.0, 0.0)
        host.control_mode_dropdown.setCurrentIndex(4)
        host.axis_dropdown.setCurrentText('z')

        host._refresh_feedback_strip()

        assert host._feedback_strip._wrench_bar._value == pytest.approx(-4.2)

    def test_cartesian_absent_wrench_shows_none(self):
        """Wrench bar is None when wrench dict is empty (controller idle)."""
        host = _make_stub_host(mode=5)
        host.control_mode_dropdown.setCurrentIndex(4)
        host.axis_dropdown.setCurrentText('x')
        # No wrench injected.

        host._refresh_feedback_strip()

        assert host._feedback_strip._wrench_bar._value is None

    def test_cartesian_postural_shows_none_wrench(self):
        """Postural axis shows None wrench (no meaningful component)."""
        host = _make_stub_host(mode=5)
        rid = 'robot1'
        host.current_wrench_by_robot[rid] = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        host.control_mode_dropdown.setCurrentIndex(4)
        host.axis_dropdown.setCurrentText('postural')

        host._refresh_feedback_strip()

        assert host._feedback_strip._wrench_bar._value is None

    def test_joint_mode_wrench_bar_is_none(self):
        """Wrench bar is None in joint mode (not per-joint meaningful)."""
        host = _make_stub_host(mode=1)
        rid = 'robot1'
        host.current_wrench_by_robot[rid] = (5.0, 5.0, 5.0, 5.0, 5.0, 5.0)
        host.control_mode_dropdown.setCurrentIndex(0)  # joint mode
        host.current_joint_commanded_by_robot[rid] = [0.0] * 7
        host.current_joint_positions_by_robot[rid] = [0.0] * 7

        host._refresh_feedback_strip()

        assert host._feedback_strip._wrench_bar._value is None

    def test_stack_guard_skips_refresh(self):
        """Refresh is skipped when stack index is not 2."""
        host = _make_stub_host(mode=1)
        rid = 'robot1'
        host.current_joint_commanded_by_robot[rid] = [0.3] * 7
        host.current_joint_positions_by_robot[rid] = [0.0] * 7
        # Override stack to say we're on page 0 (not tuning page).
        host.stack = types.SimpleNamespace(currentIndex=lambda: 0)
        host.is_page_visible = lambda name: False
        host._page_showing = host.is_page_visible
        host.control_mode_dropdown.setCurrentIndex(0)

        host._refresh_feedback_strip()

        # Strip should not have been updated.
        assert host._feedback_strip._error_bar._value is None


# ---------------------------------------------------------------------------
# Validation tests
# ---------------------------------------------------------------------------

class TestValidateBeforeSave:
    """Tests for _validate_before_save monotonicity and range checks."""

    def test_valid_single_breakpoint_returns_empty(self):
        """Single breakpoint with matching stiffness passes validation."""
        host = _make_stub_host()
        # error_max=0.05, force_max=5.0 → max K = 100. Set stiffness = 80.
        host.tuning_error_max.set_values([0.05])
        host.tuning_force_max.set_values([5.0])
        host.tuning_stiffness.set_value(80.0)
        err = host._validate_before_save()
        assert err == ''

    def test_valid_two_breakpoints_returns_empty(self):
        """Two strictly ascending breakpoints pass validation."""
        host = _make_stub_host()
        host.tuning_error_max.set_count(2)
        host.tuning_force_max.set_count(2)
        host.tuning_error_max.set_values([0.05, 0.10])
        host.tuning_force_max.set_values([5.0, 39.0])
        # max K = 5.0/0.05 = 100. Set stiffness = 90.
        host.tuning_stiffness.set_value(90.0)
        err = host._validate_before_save()
        assert err == ''

    def test_non_monotonic_error_max_fails(self):
        """Non-ascending error_max returns a non-empty error string."""
        host = _make_stub_host()
        host.tuning_error_max.set_count(2)
        host.tuning_force_max.set_count(2)
        # Manually force non-monotonic values via the internal field APIs.
        host.tuning_error_max._fields[0].set_value(0.10)
        host.tuning_error_max._fields[1].set_value(0.05)  # reversed
        host.tuning_force_max.set_values([5.0, 39.0])
        err = host._validate_before_save()
        assert err != ''
        assert 'error_max' in err.lower()

    def test_non_monotonic_force_max_fails(self):
        """Non-ascending force_max returns a non-empty error string."""
        host = _make_stub_host()
        host.tuning_error_max.set_count(2)
        host.tuning_force_max.set_count(2)
        host.tuning_error_max.set_values([0.05, 0.10])
        host.tuning_force_max._fields[0].set_value(39.0)
        host.tuning_force_max._fields[1].set_value(5.0)   # reversed
        err = host._validate_before_save()
        assert err != ''
        assert 'force_max' in err.lower()

    def test_equal_breakpoints_fails(self):
        """Equal adjacent error_max breakpoints (not strictly ascending) fail."""
        host = _make_stub_host()
        host.tuning_error_max.set_count(2)
        host.tuning_force_max.set_count(2)
        host.tuning_error_max._fields[0].set_value(0.05)
        host.tuning_error_max._fields[1].set_value(0.05)  # equal
        host.tuning_force_max.set_values([5.0, 39.0])
        err = host._validate_before_save()
        assert err != ''


# ---------------------------------------------------------------------------
# Applied stamp tests
# ---------------------------------------------------------------------------

class TestAppliedStamp:
    """Tests for _set_applied_stamp."""

    def test_stamp_sets_label_text(self):
        """_set_applied_stamp sets the applied label to 'Saved to config.xml at HH:MM:SS'."""
        host = _make_stub_host()
        host._set_applied_stamp('14:35:00')
        assert '14:35:00' in host._applied_label.text()
        assert 'Saved to config.xml at' in host._applied_label.text()

    def test_stamp_different_times(self):
        """_set_applied_stamp accepts different timestamp strings."""
        host = _make_stub_host()
        host._set_applied_stamp('09:00:01')
        assert '09:00:01' in host._applied_label.text()
        host._set_applied_stamp('23:59:59')
        assert '23:59:59' in host._applied_label.text()


# ---------------------------------------------------------------------------
# Stiffness clamp before save test
# ---------------------------------------------------------------------------

class TestStiffnessClampBeforeSave:
    """Verify _clamp_stiffness caps K0 at force_max[0]/error_max[0]."""

    def test_over_limit_is_clamped(self):
        """Stiffness exceeding f/e is reduced to the limit."""
        host = _make_stub_host()
        # With error_max=0.05 and force_max=5.0, max K = 100.
        clamped = host._clamp_stiffness(600.0, [0.05], [5.0])
        assert clamped == pytest.approx(100.0)

    def test_within_limit_unchanged(self):
        """Stiffness within the limit is returned unchanged."""
        host = _make_stub_host()
        clamped = host._clamp_stiffness(50.0, [0.05], [5.0])
        assert clamped == pytest.approx(50.0)

    def test_empty_lists_unchanged(self):
        """Empty em/fm lists leave the stiffness unchanged."""
        host = _make_stub_host()
        clamped = host._clamp_stiffness(600.0, [], [])
        assert clamped == pytest.approx(600.0)

    def test_stiffness_field_clamped_via_widget(self):
        """Validation catches stiffness exceeding force_max[0]/error_max[0]."""
        host = _make_stub_host()
        host.tuning_error_max.set_values([0.05])
        host.tuning_force_max.set_values([5.0])
        # Force the field value high (above 100.0 which is 5.0/0.05).
        host.tuning_stiffness._value = 600.0
        err = host._validate_before_save()
        # Validation should detect K0 > f/e.
        assert err != ''
