"""Tests for JointTable widget: row management, round-trip, signal, and clamp."""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402
import pytest  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------

class TestJointTableConstruction:
    """Tests for JointTable initial construction."""

    def test_default_zero_rows(self):
        """Widget with no arguments starts with zero rows."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        assert len(t.rows()) == 0

    def test_set_count_builds_rows(self):
        """set_count(7) produces exactly 7 rows."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(7)
        assert len(t.rows()) == 7

    def test_set_count_one(self):
        """set_count(1) produces exactly 1 row."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(1)
        assert len(t.rows()) == 1

    def test_row_dict_keys(self):
        """Each row dict has keys: name, lower, upper, torque_max."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(3)
        for row in t.rows():
            assert set(row.keys()) == {'name', 'lower', 'upper', 'torque_max'}

    def test_row_name_is_str(self):
        """Row name is a string."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(2)
        for row in t.rows():
            assert isinstance(row['name'], str)

    def test_row_numeric_fields_are_float(self):
        """lower, upper, torque_max values are floats."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(2)
        for row in t.rows():
            assert isinstance(row['lower'], float)
            assert isinstance(row['upper'], float)
            assert isinstance(row['torque_max'], float)


# ---------------------------------------------------------------------------
# set_rows / rows round-trip
# ---------------------------------------------------------------------------

class TestJointTableRoundTrip:
    """Tests for set_rows/rows value round-trip."""

    def test_set_rows_roundtrip_names(self):
        """set_rows then rows() returns the same joint names."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        data = [
            {'name': 'joint1', 'lower': -170.0, 'upper': 170.0, 'torque_max': 87.0},
            {'name': 'joint2', 'lower': -101.0, 'upper': 101.0, 'torque_max': 87.0},
            {'name': 'joint3', 'lower': -166.0, 'upper': 166.0, 'torque_max': 12.0},
        ]
        t.set_rows(data)
        result = t.rows()
        assert [r['name'] for r in result] == ['joint1', 'joint2', 'joint3']

    def test_set_rows_roundtrip_lower(self):
        """set_rows then rows() returns the correct lower limits."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        data = [
            {'name': 'A1', 'lower': -90.0, 'upper': 90.0, 'torque_max': 39.0},
            {'name': 'A2', 'lower': -128.0, 'upper': 128.0, 'torque_max': 39.0},
        ]
        t.set_rows(data)
        result = t.rows()
        assert result[0]['lower'] == pytest.approx(-90.0)
        assert result[1]['lower'] == pytest.approx(-128.0)

    def test_set_rows_roundtrip_upper(self):
        """set_rows then rows() returns the correct upper limits."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        data = [
            {'name': 'A1', 'lower': -90.0, 'upper': 90.0, 'torque_max': 39.0},
        ]
        t.set_rows(data)
        assert t.rows()[0]['upper'] == pytest.approx(90.0)

    def test_set_rows_roundtrip_torque_max(self):
        """set_rows then rows() returns the correct torque_max values."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        data = [
            {'name': 'J1', 'lower': -180.0, 'upper': 180.0, 'torque_max': 320.0},
            {'name': 'J2', 'lower': -120.0, 'upper': 120.0, 'torque_max': 40.0},
        ]
        t.set_rows(data)
        result = t.rows()
        assert result[0]['torque_max'] == pytest.approx(320.0)
        assert result[1]['torque_max'] == pytest.approx(40.0)

    def test_set_rows_sets_count(self):
        """set_rows(list of 5) leaves the table with exactly 5 rows."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        data = [
            {'name': 'j%d' % i, 'lower': -180.0, 'upper': 180.0, 'torque_max': 50.0}
            for i in range(5)
        ]
        t.set_rows(data)
        assert len(t.rows()) == 5


# ---------------------------------------------------------------------------
# Growing and shrinking preserves / drops values
# ---------------------------------------------------------------------------

class TestJointTableResize:
    """Tests for set_count growing/shrinking behaviour."""

    def test_grow_preserves_existing_names(self):
        """Growing from 3 to 5 rows preserves the first 3 names."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_rows([
            {'name': 'alpha', 'lower': -90.0, 'upper': 90.0, 'torque_max': 10.0},
            {'name': 'beta', 'lower': -90.0, 'upper': 90.0, 'torque_max': 10.0},
            {'name': 'gamma', 'lower': -90.0, 'upper': 90.0, 'torque_max': 10.0},
        ])
        t.set_count(5)
        result = t.rows()
        assert result[0]['name'] == 'alpha'
        assert result[1]['name'] == 'beta'
        assert result[2]['name'] == 'gamma'

    def test_grow_preserves_existing_limits(self):
        """Growing from 2 to 4 rows preserves lower/upper of original rows."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_rows([
            {'name': 'j1', 'lower': -170.0, 'upper': 170.0, 'torque_max': 87.0},
            {'name': 'j2', 'lower': -101.0, 'upper': 101.0, 'torque_max': 40.0},
        ])
        t.set_count(4)
        result = t.rows()
        assert result[0]['lower'] == pytest.approx(-170.0)
        assert result[0]['upper'] == pytest.approx(170.0)
        assert result[1]['lower'] == pytest.approx(-101.0)

    def test_shrink_drops_tail_rows(self):
        """Shrinking from 5 to 3 rows removes the last 2 rows."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        names = ['j%d' % i for i in range(5)]
        t.set_rows([
            {'name': n, 'lower': -90.0, 'upper': 90.0, 'torque_max': 10.0}
            for n in names
        ])
        t.set_count(3)
        result = t.rows()
        assert len(result) == 3
        assert [r['name'] for r in result] == ['j0', 'j1', 'j2']

    def test_shrink_to_zero(self):
        """Shrinking to 0 rows produces an empty list."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(4)
        t.set_count(0)
        assert t.rows() == []

    def test_set_count_same_is_noop_length(self):
        """set_count(n) when already n rows leaves the count unchanged."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(3)
        t.set_count(3)
        assert len(t.rows()) == 3


# ---------------------------------------------------------------------------
# changed signal
# ---------------------------------------------------------------------------

class TestJointTableChangedSignal:
    """Tests for JointTable.changed signal emission."""

    def test_changed_emits_on_name_edit(self):
        """Editing a joint name emits changed."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(2)
        fired = []
        t.changed.connect(lambda: fired.append(True))
        # Simulate text change on first row's name field.
        t._row_widgets[0]['name_edit'].setText('new_name')
        assert len(fired) >= 1

    def test_changed_emits_on_torque_commit(self):
        """Committing a torque_max value emits changed."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(2)
        fired = []
        t.changed.connect(lambda: fired.append(True))
        t._row_widgets[0]['torque_field']._commit_text('50.0')
        assert len(fired) >= 1

    def test_changed_emits_on_lower_commit(self):
        """Committing a lower limit value emits changed."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable()
        t.set_count(1)
        fired = []
        t.changed.connect(lambda: fired.append(True))
        t._row_widgets[0]['lower_field']._commit_text('-120.0')
        assert len(fired) >= 1


# ---------------------------------------------------------------------------
# MetricField clamp in torque rows
# ---------------------------------------------------------------------------

class TestJointTableTorqueClamp:
    """Tests for MetricField torque clamping inside JointTable rows."""

    def test_torque_max_clamped_to_range(self):
        """torque_max MetricField clamps an over-range value to its maximum."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable(unit_tau='Nm')
        t.set_count(1)
        field = t._row_widgets[0]['torque_field']
        field._commit_text('99999')
        # Value should be clamped to the field's configured max.
        assert field.value() <= field._maximum

    def test_torque_max_zero_clamps_to_min(self):
        """torque_max MetricField clamps a negative value to zero (min)."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable(unit_tau='Nm')
        t.set_count(1)
        field = t._row_widgets[0]['torque_field']
        field._commit_text('-999')
        assert field.value() >= field._minimum

    def test_lower_field_unit_matches(self):
        """The lower field has the unit_deg suffix."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable(unit_deg='deg')
        t.set_count(1)
        assert t._row_widgets[0]['lower_field']._unit == 'deg'

    def test_upper_field_unit_matches(self):
        """The upper field has the unit_deg suffix."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable(unit_deg='rad')
        t.set_count(1)
        assert t._row_widgets[0]['upper_field']._unit == 'rad'

    def test_torque_field_unit_matches(self):
        """The torque field has the unit_tau suffix."""
        from fic_studio.components.joint_table import JointTable
        t = JointTable(unit_tau='kNm')
        t.set_count(1)
        assert t._row_widgets[0]['torque_field']._unit == 'kNm'


# ---------------------------------------------------------------------------
# Export check
# ---------------------------------------------------------------------------

class TestJointTableExport:
    """Tests that JointTable is exported from fic_studio.components."""

    def test_importable_from_components(self):
        """Widget can be imported from fic_studio.components."""
        from fic_studio.components import JointTable
        assert JointTable is not None

    def test_in_all(self):
        """Name appears in components.__all__."""
        import fic_studio.components as comp
        assert 'JointTable' in comp.__all__
