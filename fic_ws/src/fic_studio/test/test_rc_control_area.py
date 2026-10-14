"""
Headless tests for the Robot Control page's adaptive control area and robot selector.

Verifies that ``_build_rc_control_area`` / ``_rebuild_rc_control_area``
produce the correct widget counts for joint and Cartesian kinds, and that
the robot selector initialises without error.
"""
import os
import types
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QComboBox, QFrame  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Minimal stubs — replicate just enough of the real objects so the mixin
# methods can execute without ROS, rclpy, or a live main window.
# ---------------------------------------------------------------------------

def _make_node(station_id=1):
    """Return a lightweight stub for FicStudioNode."""
    node = types.SimpleNamespace()
    node.station_id = station_id
    node.station_robot_id = 'robot%d' % station_id
    node._active_robots = [station_id]

    def add_robot(n):
        if n not in node._active_robots:
            node._active_robots.append(n)
        return 'robot%d' % n

    node.add_robot = add_robot
    return node


def _make_robot_configs(num_joints=7, lo=-170.0, hi=170.0):
    """Return a minimal robot_configs dict matching the mixin access pattern."""
    return {
        'Franka_FR3': {
            'num_joints': num_joints,
            'joint_limits_deg': {
                'lower': [lo] * num_joints,
                'upper': [hi] * num_joints,
            },
            'has_gripper': True,
            'default_ee_target': {'position': [0.38, 0.0, 0.432]},
        }
    }


def _make_robot_dropdown():
    """Return a QComboBox stub for self.robot_dropdown."""
    cb = QComboBox()
    cb.addItem('Franka_FR3')
    return cb


def _make_mixin_instance():
    """
    Construct a bare Python object with the attributes those methods read.

    Returns a _Host instance without building the full main window.
    """
    from fic_studio.mixins.robot_control_mixin import RobotControlMixin

    class _Host(RobotControlMixin):
        """Minimal host satisfying the mixin's attribute expectations."""

        def __init__(self):
            """Initialise with stub node, config, and minimal UI widgets."""
            # ── Node stub ──
            self.node = _make_node()

            # ── Robot config ──
            self.robot_configs = _make_robot_configs()
            self.robots = {'Franka_FR3': 7}

            # ── Global dropdown (used by _rebuild_rc_control_area) ──
            self.robot_dropdown = _make_robot_dropdown()

            # ── Mode state ──
            self._rc_selected_kind = 'joint'
            self._rc_selected_mode_idx = 0

            # ── Control-area state ──
            self._rc_active_rid = self.node.station_robot_id
            self._rc_joint_fields = []
            self._rc_pose_fields = {}

        # Stubs for methods called transitively from helpers
        def log_message(self, level, msg, source=''):
            """No-op log stub."""

        # JointStateMixin collaborators. The real mixin returns an empty
        # list / None until telemetry arrives, which is exactly the state
        # a freshly-built control area is assembled in.
        def joint_positions_for(self, robot_id):
            """No live joint telemetry in the stub."""
            return []

        def ee_pose_for(self, robot_id):
            """No live EE pose in the stub."""
            return None

    return _Host()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestBuildRcControlArea(unittest.TestCase):
    """_build_rc_control_area returns a QFrame with a populated layout."""

    def setUp(self):
        """Create a fresh host instance for each test."""
        self.host = _make_mixin_instance()

    def test_returns_qframe(self):
        """_build_rc_control_area returns a QFrame."""
        host = self.host
        frame = host._build_rc_control_area()
        self.assertIsInstance(frame, QFrame)

    def test_rc_area_layout_set(self):
        """After build, self._rc_area_layout is set."""
        host = self.host
        host._build_rc_control_area()
        self.assertIsNotNone(getattr(host, '_rc_area_layout', None))

    def test_joint_fields_populated_after_build(self):
        """Joint kind: _rc_joint_fields contains num_joints entries."""
        host = self.host
        host._rc_selected_kind = 'joint'
        host._build_rc_control_area()
        self.assertEqual(len(host._rc_joint_fields), 7)

    def test_joint_field_count_respects_config(self):
        """joint_fields count matches num_joints in robot_configs."""
        host = self.host
        host.robot_configs = _make_robot_configs(num_joints=6)
        host._rc_selected_kind = 'joint'
        host._build_rc_control_area()
        self.assertEqual(len(host._rc_joint_fields), 6)


class TestRebuildRcControlAreaJoint(unittest.TestCase):
    """_rebuild_rc_control_area (joint kind) produces correct field count."""

    def setUp(self):
        """Create host and build the control area in joint mode."""
        self.host = _make_mixin_instance()
        # Keep the frame alive so the layout is not garbage collected
        self._frame = self.host._build_rc_control_area()
        self.host._rc_selected_kind = 'joint'
        self.host._rebuild_rc_control_area()

    def test_joint_fields_count_default(self):
        """7-DOF config → 7 joint CommandedVsActual fields."""
        self.assertEqual(len(self.host._rc_joint_fields), 7)

    def test_joint_fields_are_commanded_vs_actual(self):
        """Each entry in _rc_joint_fields is a CommandedVsActual widget."""
        from fic_studio.components.control_widgets import CommandedVsActual
        for field in self.host._rc_joint_fields:
            self.assertIsInstance(field, CommandedVsActual)

    def test_pose_fields_empty_in_joint_mode(self):
        """_rc_pose_fields is empty when kind is joint."""
        self.assertEqual(self.host._rc_pose_fields, {})

    def test_joint_field_label_pattern(self):
        """Joint field labels follow J1..J7 pattern."""
        from fic_studio.components.control_widgets import CommandedVsActual
        for i, field in enumerate(self.host._rc_joint_fields):
            self.assertIsInstance(field, CommandedVsActual)
            # Label is in the inner MetricField's label widget
            expected = 'J%d' % (i + 1)
            self.assertEqual(field._field._label_text, expected)


class TestRebuildRcControlAreaCart(unittest.TestCase):
    """_rebuild_rc_control_area (cart kind) produces correct field count."""

    def setUp(self):
        """Create host and build the control area in cart mode."""
        self.host = _make_mixin_instance()
        # Keep the frame alive so the layout is not garbage collected
        self._frame = self.host._build_rc_control_area()
        self.host._rc_selected_kind = 'cart'
        self.host._rebuild_rc_control_area()

    def test_pose_fields_keys(self):
        """Cart kind: _rc_pose_fields has X, Y, Z + axis X/Y/Z + angle."""
        expected_keys = {'X', 'Y', 'Z', 'axis X', 'axis Y', 'axis Z', 'angle'}
        self.assertEqual(set(self.host._rc_pose_fields.keys()), expected_keys)

    def test_position_fields_are_commanded_vs_actual(self):
        """X/Y/Z fields are CommandedVsActual (carry actual overlay)."""
        from fic_studio.components.control_widgets import CommandedVsActual
        for k in ('X', 'Y', 'Z'):
            self.assertIsInstance(self.host._rc_pose_fields[k], CommandedVsActual)

    def test_orientation_fields_are_metric_field(self):
        """Axis X/Y/Z and angle are plain MetricField (command-only)."""
        from fic_studio.components.control_widgets import MetricField, CommandedVsActual
        for k in ('axis X', 'axis Y', 'axis Z', 'angle'):
            f = self.host._rc_pose_fields[k]
            self.assertIsInstance(f, MetricField)
            # Must NOT be the subclass CommandedVsActual
            self.assertNotIsInstance(f, CommandedVsActual)

    def test_joint_fields_empty_in_cart_mode(self):
        """_rc_joint_fields is empty when kind is cart."""
        self.assertEqual(self.host._rc_joint_fields, [])

    def test_total_pose_widget_count(self):
        """7 widgets total: 3 position + 4 orientation."""
        self.assertEqual(len(self.host._rc_pose_fields), 7)


class TestSwitchKindRebuild(unittest.TestCase):
    """Switching kind via _rebuild_rc_control_area clears previous fields."""

    def setUp(self):
        """Build area in joint mode then switch to cart."""
        self.host = _make_mixin_instance()
        # Keep the frame alive so the layout is not garbage collected
        self._frame = self.host._build_rc_control_area()

    def test_joint_to_cart_clears_joint_fields(self):
        """After switch to cart, _rc_joint_fields is empty."""
        self.host._rc_selected_kind = 'joint'
        self.host._rebuild_rc_control_area()
        self.assertGreater(len(self.host._rc_joint_fields), 0)

        self.host._rc_selected_kind = 'cart'
        self.host._rebuild_rc_control_area()
        self.assertEqual(self.host._rc_joint_fields, [])

    def test_cart_to_joint_clears_pose_fields(self):
        """After switch back to joint, _rc_pose_fields is empty."""
        self.host._rc_selected_kind = 'cart'
        self.host._rebuild_rc_control_area()
        self.assertGreater(len(self.host._rc_pose_fields), 0)

        self.host._rc_selected_kind = 'joint'
        self.host._rebuild_rc_control_area()
        self.assertEqual(self.host._rc_pose_fields, {})

    def test_multiple_rebuilds_no_error(self):
        """Calling _rebuild_rc_control_area multiple times does not raise."""
        for kind in ('joint', 'cart', 'joint', 'cart'):
            self.host._rc_selected_kind = kind
            self.host._rebuild_rc_control_area()   # must not raise


class TestRcCommandRid(unittest.TestCase):
    """_rc_command_rid returns _rc_active_rid."""

    def test_returns_station_robot_id_by_default(self):
        """Default _rc_active_rid equals station_robot_id."""
        host = _make_mixin_instance()
        self.assertEqual(host._rc_command_rid(), 'robot1')

    def test_returns_updated_rid_after_change(self):
        """After updating _rc_active_rid, _rc_command_rid reflects it."""
        host = _make_mixin_instance()
        host._rc_active_rid = 'robot2'
        self.assertEqual(host._rc_command_rid(), 'robot2')


class TestRcRobotSelector(unittest.TestCase):
    """_rc_rebuild_robot_selector populates the combo correctly."""

    def setUp(self):
        """Build a host with the selector widget."""
        from PyQt5.QtWidgets import QComboBox, QListView
        host = _make_mixin_instance()
        sel = QComboBox()
        sel.setView(QListView())
        host._rc_robot_selector = sel
        host._rc_rebuild_robot_selector()
        self.host = host
        self.sel = sel

    def test_selector_has_robot_entry(self):
        """Selector contains at least one 'Robot N' entry."""
        texts = [self.sel.itemText(i) for i in range(self.sel.count())]
        self.assertIn('Robot 1', texts)

    def test_selector_has_add_entry(self):
        """Selector always ends with the Add-robot entry."""
        last = self.sel.itemText(self.sel.count() - 1)
        self.assertEqual(last, '＋ Add robot…')

    def test_selector_count_matches_active_plus_one(self):
        """Selector count = len(active_robots) + 1 (for add entry)."""
        active_count = len(self.host.node._active_robots)
        self.assertEqual(self.sel.count(), active_count + 1)

    def test_on_robot_selected_updates_active_rid(self):
        """Selecting 'Robot 1' sets _rc_active_rid to 'robot1'."""
        # Add a second robot so the selector has > 1 real entry
        self.host.node.add_robot(2)
        self.host._rc_rebuild_robot_selector()
        # Simulate selecting 'Robot 2'
        self.host._rc_on_robot_selected('Robot 2')
        self.assertEqual(self.host._rc_active_rid, 'robot2')


class TestStubHandlers(unittest.TestCase):
    """Stub handlers exist and accept the expected arguments."""

    def setUp(self):
        """Create host instance."""
        self.host = _make_mixin_instance()

    def test_joint_field_stub_callable(self):
        """_rc_on_joint_field accepts (index, value) without error."""
        self.host._rc_on_joint_field(0, 45.0)   # must not raise

    def test_pose_field_stub_callable(self):
        """_rc_on_pose_field accepts (value,) without error."""
        self.host._rc_on_pose_field(0.5)   # must not raise

    def test_joint_field_stub_returns_none(self):
        """_rc_on_joint_field returns None."""
        result = self.host._rc_on_joint_field(2, -30.0)
        self.assertIsNone(result)

    def test_pose_field_stub_returns_none(self):
        """_rc_on_pose_field returns None."""
        result = self.host._rc_on_pose_field(0.3)
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()
