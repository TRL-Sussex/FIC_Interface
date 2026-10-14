"""
Headless tests for the Robot Control wiring: joint/pose publish, the live-publish gate and the refresh.

Verifies:
- Joint committed always publishes (spy on node method).
- Joint edited only publishes when _rc_live_publish is checked.
- Pose committed always publishes.
- Pose edited only publishes when _rc_live_publish is checked.
- Gripper handler routes to node.publish_gripper_width with the active rid.
- Joystick nudges the Cartesian MetricField values.
- Refresh tick pushes joint_positions_for (rad->deg) into set_actual.
- Refresh tick pushes ee_pose_for position into X/Y/Z set_actual.
"""
import math
import os
import types
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QCheckBox, QComboBox  # noqa: E402

_app = QApplication.instance() or QApplication([])


# ---------------------------------------------------------------------------
# Helpers — minimal stubs
# ---------------------------------------------------------------------------

def _make_node(station_id=1):
    """Return a lightweight stub for FicStudioNode with publish-call recording."""
    node = types.SimpleNamespace()
    node.station_id = station_id
    node.station_robot_id = 'robot%d' % station_id
    node._active_robots = [station_id]

    # Publish-call recording lists
    node.joint_angle_calls = []
    node.real_joint_calls = []
    node.position_calls = []
    node.gripper_calls = []

    def publish_joint_angles(positions, names=None, robot_id=None):
        node.joint_angle_calls.append({'positions': positions, 'names': names,
                                       'robot_id': robot_id})

    def publish_real_joint_angles(robot_id, positions):
        node.real_joint_calls.append({'robot_id': robot_id, 'positions': positions})

    def publish_position(pose, robot_id=None):
        node.position_calls.append({'pose': pose, 'robot_id': robot_id})

    def publish_gripper_width(robot_id, width_m):
        node.gripper_calls.append({'robot_id': robot_id, 'width_m': width_m})

    def add_robot(n):
        if n not in node._active_robots:
            node._active_robots.append(n)
        return 'robot%d' % n

    node.publish_joint_angles = publish_joint_angles
    node.publish_real_joint_angles = publish_real_joint_angles
    node.publish_position = publish_position
    node.publish_gripper_width = publish_gripper_width
    node.add_robot = add_robot
    return node


def _make_robot_configs(num_joints=7):
    """Return a minimal robot_configs dict."""
    return {
        'Franka_FR3': {
            'num_joints': num_joints,
            'joint_limits_deg': {
                'lower': [-170.0] * num_joints,
                'upper': [170.0] * num_joints,
            },
            'has_gripper': True,
            'default_ee_target': {'position': [0.38, 0.0, 0.432]},
        }
    }


def _make_mixin_instance(num_joints=7, target='simulation'):
    """Construct a bare host with the attributes those methods read."""
    from fic_studio.mixins.robot_control_mixin import RobotControlMixin

    class _Host(RobotControlMixin):
        """Minimal host satisfying the mixin's attribute expectations."""

        def __init__(self):
            """Initialise with stub node, config, and minimal UI widgets."""
            self.node = _make_node()
            self.robot_configs = _make_robot_configs(num_joints)
            self.robots = {'Franka_FR3': 7}

            cb = QComboBox()
            cb.addItem('Franka_FR3')
            self.robot_dropdown = cb

            self._rc_selected_kind = 'joint'
            self._rc_selected_mode_idx = 0
            self._rc_active_rid = self.node.station_robot_id
            self._rc_joint_fields = []
            self._rc_pose_fields = {}
            self.control_target = target

            # Live-publish checkbox (default unchecked)
            self._rc_live_publish = QCheckBox()
            self._rc_live_publish.setChecked(False)

            # Joystick auto-publish legacy (not used in new path)
            self.joy_autopublish = QCheckBox()
            self.joy_autopublish.setChecked(False)

            # Stub legacy pose inputs (referenced by _publish_rc_pose fallback)
            self.x_input = QComboBox()  # intentionally wrong type — not called
            self.y_input = QComboBox()
            self.z_input = QComboBox()

            # main_stack page-guard (page 1 = robot control)
            self.main_stack = types.SimpleNamespace()
            self.main_stack.currentIndex = lambda: 1
            self.is_page_visible = lambda name: name == 'multi_robot'

        def log_message(self, level, msg, source=''):
            """No-op log stub."""

        def send_pose_for(self, rid, pose=None):
            """Stub send_pose_for used by hardware path.

            Records the pose it was handed. It used to record None
            unconditionally, which is how the hardware path shipped for
            so long while publishing default_ee_target instead of the
            operator's input -- the stub could not tell the difference.
            """
            self.node.position_calls.append({'pose': pose, 'robot_id': rid,
                                             'via': 'send_pose_for'})

        def joint_positions_for(self, rid):
            """Return synthetic joint positions for refresh tests."""
            return getattr(self, '_fake_joint_positions', [])

        def ee_pose_for(self, rid):
            """Return synthetic EE pose for refresh tests."""
            return getattr(self, '_fake_ee_pose', None)

    host = _Host()
    host.control_target = target
    return host


def _build_joint_area(host):
    """Build the adaptive control area in joint mode and return the frame."""
    host._rc_selected_kind = 'joint'
    frame = host._build_rc_control_area()
    return frame


def _build_cart_area(host):
    """Build the adaptive control area in cart mode and return the frame."""
    host._rc_selected_kind = 'cart'
    frame = host._build_rc_control_area()
    return frame


# ---------------------------------------------------------------------------
# Tests: joint committed / edited
# ---------------------------------------------------------------------------

class TestJointPublishGating(unittest.TestCase):
    """_rc_on_joint_committed always publishes; edited gates on live-publish."""

    def setUp(self):
        """Build host in simulation target with joint fields."""
        self.host = _make_mixin_instance(target='simulation')
        self._frame = _build_joint_area(self.host)

    def test_committed_publishes_unconditionally(self):
        """_rc_on_joint_committed publishes even when live-publish is off."""
        self.host._rc_live_publish.setChecked(False)
        self.host._rc_on_joint_committed(0, 45.0)
        self.assertEqual(len(self.host.node.joint_angle_calls), 1)

    def test_edited_no_publish_when_live_off(self):
        """_rc_on_joint_edited does NOT publish when live-publish is unchecked."""
        self.host._rc_live_publish.setChecked(False)
        self.host._rc_on_joint_edited(0, 45.0)
        self.assertEqual(len(self.host.node.joint_angle_calls), 0)

    def test_edited_publishes_when_live_on(self):
        """_rc_on_joint_edited publishes when live-publish is checked."""
        self.host._rc_live_publish.setChecked(True)
        self.host._rc_on_joint_edited(0, 45.0)
        self.assertEqual(len(self.host.node.joint_angle_calls), 1)

    def test_sim_uses_publish_joint_angles(self):
        """Simulation target routes to publish_joint_angles, not real angles."""
        self.host._rc_on_joint_committed(0, 0.0)
        self.assertEqual(len(self.host.node.joint_angle_calls), 1)
        self.assertEqual(len(self.host.node.real_joint_calls), 0)

    def test_hardware_uses_publish_real_joint_angles(self):
        """Hardware target routes to publish_real_joint_angles."""
        self.host.control_target = 'hardware'
        self.host._rc_on_joint_committed(0, 0.0)
        self.assertEqual(len(self.host.node.real_joint_calls), 1)
        self.assertEqual(len(self.host.node.joint_angle_calls), 0)

    def test_joint_names_include_robot_type(self):
        """Joint names are '{robot_type}_j{k+1}' for sim/VR targets."""
        self.host._rc_on_joint_committed(0, 0.0)
        call = self.host.node.joint_angle_calls[-1]
        robot_type = self.host.robot_dropdown.currentText()
        expected_prefix = robot_type + '_j'
        for name in call['names']:
            self.assertTrue(name.startswith(expected_prefix),
                            msg='name %r missing robot_type prefix' % name)

    def test_robot_id_routed_via_command_rid(self):
        """The robot_id kwarg equals _rc_command_rid()."""
        self.host._rc_active_rid = 'robot2'
        self.host._rc_on_joint_committed(0, 0.0)
        call = self.host.node.joint_angle_calls[-1]
        self.assertEqual(call['robot_id'], 'robot2')

    def test_legacy_stub_callable(self):
        """_rc_on_joint_field (legacy alias) does not raise and publishes."""
        self.host._rc_on_joint_field(0, 45.0)
        self.assertGreaterEqual(len(self.host.node.joint_angle_calls), 1)

    def test_deg_to_rad_conversion(self):
        """Joint field values (deg) are converted to radians before publish."""
        # CommandedVsActual wraps an inner MetricField at ._field
        for field in self.host._rc_joint_fields:
            field._field.set_value(90.0)
        self.host._rc_publish_joints()
        call = self.host.node.joint_angle_calls[-1]
        for rad in call['positions']:
            self.assertAlmostEqual(rad, math.pi / 2, places=6)


# ---------------------------------------------------------------------------
# Tests: pose committed / edited
# ---------------------------------------------------------------------------

class TestPosePublishGating(unittest.TestCase):
    """_rc_on_pose_committed always publishes; edited gates on live-publish."""

    def setUp(self):
        """Build host in simulation target with cart fields."""
        self.host = _make_mixin_instance(target='simulation')
        self._frame = _build_cart_area(self.host)

    def test_committed_publishes_unconditionally(self):
        """_rc_on_pose_committed publishes even when live-publish is off."""
        self.host._rc_live_publish.setChecked(False)
        self.host._rc_on_pose_committed(0.5)
        self.assertEqual(len(self.host.node.position_calls), 1)

    def test_edited_no_publish_when_live_off(self):
        """_rc_on_pose_edited does NOT publish when live-publish is unchecked."""
        self.host._rc_live_publish.setChecked(False)
        self.host._rc_on_pose_edited(0.5)
        self.assertEqual(len(self.host.node.position_calls), 0)

    def test_edited_publishes_when_live_on(self):
        """_rc_on_pose_edited publishes when live-publish is checked."""
        self.host._rc_live_publish.setChecked(True)
        self.host._rc_on_pose_edited(0.5)
        self.assertEqual(len(self.host.node.position_calls), 1)

    def test_sim_uses_publish_position(self):
        """Simulation target routes to node.publish_position."""
        self.host._rc_on_pose_committed(0.0)
        call = self.host.node.position_calls[-1]
        self.assertNotIn('via', call)

    def test_hardware_uses_send_pose_for(self):
        """Hardware target routes to send_pose_for with active rid."""
        self.host.control_target = 'hardware'
        self.host._rc_active_rid = 'robot1'
        self.host._rc_on_pose_committed(0.0)
        # send_pose_for stub appends with 'via' key
        calls_with_via = [c for c in self.host.node.position_calls
                          if c.get('via') == 'send_pose_for']
        self.assertEqual(len(calls_with_via), 1)
        self.assertEqual(calls_with_via[-1]['robot_id'], 'robot1')

    def test_hardware_sends_this_panels_pose_not_the_cards(self):
        """The hardware path must hand over the pose it just built.

        Passing no pose makes send_pose_for re-derive one from the
        per-robot card widgets, which sit at default_ee_target -- so the
        arm received the config default no matter what was typed here.
        Measured on hardware as four consecutive sends all arriving
        byte-identical to the home pose (0.380, 0.000, 0.432).
        """
        self.host.control_target = 'hardware'
        self.host._rc_active_rid = 'robot1'
        self.host._rc_pose_fields['X'].set_commanded(0.111)
        self.host._rc_pose_fields['Y'].set_commanded(0.222)
        self.host._rc_pose_fields['Z'].set_commanded(0.333)
        self.host._rc_on_pose_committed(0.0)
        call = [c for c in self.host.node.position_calls
                if c.get('via') == 'send_pose_for'][-1]
        self.assertIsNotNone(
            call['pose'], 'hardware path handed over no pose, so the '
                          'per-robot card defaults would be published')
        self.assertAlmostEqual(call['pose'].position.x, 0.111, places=3)
        self.assertAlmostEqual(call['pose'].position.y, 0.222, places=3)
        self.assertAlmostEqual(call['pose'].position.z, 0.333, places=3)

    def test_robot_id_routed_via_command_rid(self):
        """The robot_id kwarg equals _rc_command_rid() for sim target."""
        self.host._rc_active_rid = 'robot3'
        self.host._rc_on_pose_committed(0.0)
        call = self.host.node.position_calls[-1]
        self.assertEqual(call['robot_id'], 'robot3')

    def test_legacy_stub_callable(self):
        """_rc_on_pose_field (legacy alias) does not raise and publishes."""
        self.host._rc_on_pose_field(0.5)
        self.assertGreaterEqual(len(self.host.node.position_calls), 1)

    def test_pose_fields_missing_no_crash(self):
        """_rc_publish_pose_from_fields returns silently when fields are empty."""
        self.host._rc_pose_fields = {}
        self.host._rc_publish_pose_from_fields()   # must not raise
        self.assertEqual(len(self.host.node.position_calls), 0)


# ---------------------------------------------------------------------------
# Tests: gripper
# ---------------------------------------------------------------------------

class TestGripperHandler(unittest.TestCase):
    """Gripper slider routes to node.publish_gripper_width with active rid."""

    def setUp(self):
        """Build host with a gripper slider attribute."""
        self.host = _make_mixin_instance()
        from PyQt5.QtWidgets import QLabel
        self.host._rc_gripper_val = QLabel('100 %')

    def test_gripper_width_published(self):
        """_on_rc_gripper_changed calls node.publish_gripper_width."""
        self.host._on_rc_gripper_changed(50)
        self.assertEqual(len(self.host.node.gripper_calls), 1)

    def test_gripper_width_calculation(self):
        """50% slider → 0.04 m (50/100 * 0.08)."""
        self.host._on_rc_gripper_changed(50)
        call = self.host.node.gripper_calls[-1]
        self.assertAlmostEqual(call['width_m'], 0.04, places=6)

    def test_gripper_uses_command_rid(self):
        """Gripper publishes to _rc_command_rid(), not hard-coded station id."""
        self.host._rc_active_rid = 'robot2'
        self.host._on_rc_gripper_changed(100)
        call = self.host.node.gripper_calls[-1]
        self.assertEqual(call['robot_id'], 'robot2')


# ---------------------------------------------------------------------------
# Tests: joystick nudges MetricFields
# ---------------------------------------------------------------------------

class TestJoystickNudge(unittest.TestCase):
    """Joystick handlers nudge the Cartesian MetricField values."""

    def setUp(self):
        """Build host in cart mode so pose fields are populated."""
        self.host = _make_mixin_instance(target='simulation')
        self._frame = _build_cart_area(self.host)

    def test_joy_xy_nudges_x_field(self):
        """XY joystick dx=1.0 increases the X MetricField value."""
        x_field = self.host._rc_pose_fields['X']
        # CommandedVsActual exposes commanded(); inner MetricField via ._field
        before = x_field.commanded()
        self.host._on_joy_xy_moved(1.0, 0.0)
        self.assertGreater(x_field.commanded(), before)

    def test_joy_xy_nudges_y_field(self):
        """XY joystick dy=1.0 increases the Y MetricField value."""
        y_field = self.host._rc_pose_fields['Y']
        before = y_field.commanded()
        self.host._on_joy_xy_moved(0.0, 1.0)
        self.assertGreater(y_field.commanded(), before)

    def test_joy_z_nudges_z_field(self):
        """Z joystick dy=1.0 increases the Z MetricField value."""
        z_field = self.host._rc_pose_fields['Z']
        before = z_field.commanded()
        self.host._on_joy_z_moved(0.0, 1.0)
        self.assertGreater(z_field.commanded(), before)

    def test_joy_no_publish_when_live_off(self):
        """Joystick move does NOT publish when live-publish is unchecked."""
        self.host._rc_live_publish.setChecked(False)
        self.host._on_joy_xy_moved(1.0, 0.0)
        self.assertEqual(len(self.host.node.position_calls), 0)

    def test_joy_publishes_when_live_on(self):
        """Joystick move publishes when live-publish is checked."""
        self.host._rc_live_publish.setChecked(True)
        self.host._on_joy_xy_moved(1.0, 0.0)
        self.assertEqual(len(self.host.node.position_calls), 1)


# ---------------------------------------------------------------------------
# Tests: ~10 Hz refresh
# ---------------------------------------------------------------------------

class TestRefreshTick(unittest.TestCase):
    """_rc_on_refresh_tick pushes actual state into field set_actual."""

    def setUp(self):
        """Build host and build control area in joint mode."""
        self.host = _make_mixin_instance(target='simulation')
        self._frame = _build_joint_area(self.host)

    def test_joint_refresh_updates_set_actual(self):
        """Refresh tick feeds joint_positions_for (rad->deg) into joint fields."""
        # Synthetic positions: 7 joints, all at pi/4 ≈ 45 deg
        self.host._fake_joint_positions = [math.pi / 4] * 7
        self.host._rc_on_refresh_tick()
        for field in self.host._rc_joint_fields:
            actual = field._actual_val
            self.assertIsNotNone(actual,
                                 msg='set_actual was not called on field')
            self.assertAlmostEqual(actual, 45.0, places=4)

    def test_refresh_no_crash_when_positions_empty(self):
        """Refresh tick with empty joint_positions_for does not raise."""
        self.host._fake_joint_positions = []
        self.host._rc_on_refresh_tick()   # must not raise

    def test_pose_refresh_updates_xyz_set_actual(self):
        """Refresh tick feeds ee_pose_for position into X/Y/Z fields."""
        # Switch to cart mode and rebuild
        self.host._rc_selected_kind = 'cart'
        self.host._rebuild_rc_control_area()

        from geometry_msgs.msg import Pose, Point, Quaternion
        fake_pose = Pose(
            position=Point(x=0.42, y=0.01, z=0.55),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.host._fake_ee_pose = fake_pose
        self.host._rc_on_refresh_tick()

        pf = self.host._rc_pose_fields
        self.assertAlmostEqual(pf['X']._actual_val, 0.42, places=6)
        self.assertAlmostEqual(pf['Y']._actual_val, 0.01, places=6)
        self.assertAlmostEqual(pf['Z']._actual_val, 0.55, places=6)

    def test_refresh_skips_when_wrong_page(self):
        """Refresh tick is a no-op when the main_stack is not on page 1."""
        self.host.main_stack.currentIndex = lambda: 0
        self.host.is_page_visible = lambda name: False
        self.host._fake_joint_positions = [math.pi / 4] * 7
        self.host._rc_on_refresh_tick()
        # Actual values should remain None (never set)
        for field in self.host._rc_joint_fields:
            self.assertIsNone(field._actual_val,
                              msg='set_actual was called despite wrong page')

    def test_refresh_no_crash_when_no_state_method(self):
        """Refresh tick is safe when joint_positions_for is absent."""
        del self.host.__class__.joint_positions_for
        self.host._rc_on_refresh_tick()   # must not raise


# ---------------------------------------------------------------------------
# Tests: _rc_on_send_command dispatch
# ---------------------------------------------------------------------------

class TestSendCommand(unittest.TestCase):
    """_rc_on_send_command dispatches to joint or pose publish."""

    def test_send_command_joint_mode(self):
        """Send command in joint mode publishes joint angles."""
        host = _make_mixin_instance(target='simulation')
        _build_joint_area(host)
        host._rc_selected_kind = 'joint'
        host._rc_on_send_command()
        self.assertEqual(len(host.node.joint_angle_calls), 1)

    def test_send_command_cart_mode(self):
        """Send command in cart mode publishes pose."""
        host = _make_mixin_instance(target='simulation')
        _build_cart_area(host)
        host._rc_selected_kind = 'cart'
        host._rc_on_send_command()
        self.assertEqual(len(host.node.position_calls), 1)


if __name__ == '__main__':
    unittest.main()
