"""A fake Studio host for extension tests: real Qt widgets, a ROS node that
records what it publishes. Used by the Teach Mode and Sequences web tests."""
import types

from PyQt5.QtWidgets import QComboBox, QLabel, QPushButton


class FakePub:
    """A ROS publisher that keeps what it was given."""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    """The GUI node's publishers and publish helpers, recording every call."""

    def __init__(self, rids=('robot1',)):
        self.station_robot_id = rids[0]
        self.pubs = {r: {'mode_select': FakePub(), 'position': FakePub()} for r in rids}
        self.real_joint_publishers = {r: FakePub() for r in rids}
        self.gripper_publishers = {r: FakePub() for r in rids}
        self.calls = []

    def publish_joint_angles(self, positions, names=None, robot_id=None):
        self.calls.append(('sim_joints', robot_id, list(positions)))

    def publish_real_joint_angles(self, robot_id, positions_rad):
        self.calls.append(('hw_joints', robot_id, list(positions_rad)))

    def publish_gripper_width(self, robot_id, width):
        self.calls.append(('gripper', robot_id, float(width)))

    def publish_position(self, pose, robot_id=None):
        self.calls.append(('sim_pose', robot_id, pose))


class FakeHost:
    """What an extension sees of the Studio."""

    def __init__(self, target='simulation', rids=('robot1',)):
        self.node = FakeNode(rids)
        self.target = target
        self.rids = list(rids)
        self.target_robot_id = self.rids[0]
        self.current_joint_positions = []
        self.current_ee_pose = None
        self.robots = {'KinovaGen3_7DOF': 7}
        self.logs = []
        self.extensions = {}
        self.main_window = types.SimpleNamespace(get_extension=lambda name: self.extensions.get(name))

    def log(self, level, message, source=''):
        self.logs.append((level, message))

    def robot_target(self, rid=None):
        return self.target

    def current_robot_name(self):
        return 'KinovaGen3_7DOF'

    def active_robot_rids(self):
        return list(self.rids)

    def set_target_robot(self, rid):
        self.target_robot_id = rid

    def make_page_title(self, text):
        return QLabel(text)

    def make_button(self, text, style=None):
        return QPushButton(text)

    def make_target_robot_combo(self):
        combo = QComboBox()
        combo.addItems(self.rids)
        return combo


def pose(x=0.4, y=0.0, z=0.5, q=(0.0, 0.0, 0.0, 1.0)):
    """A geometry_msgs-like pose."""
    ns = types.SimpleNamespace
    return ns(position=ns(x=x, y=y, z=z), orientation=ns(x=q[0], y=q[1], z=q[2], w=q[3]))
