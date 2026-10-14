"""The light state feed publishes a station's state as standard messages."""
from fic_studio.fleet import lite
from geometry_msgs.msg import Pose


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time(sec=1, nanosec=2)


class _Node:
    def __init__(self):
        self.pubs = {}

    def create_publisher(self, _type, topic, _qos):
        self.pubs[topic] = _Pub()
        return self.pubs[topic]

    def get_clock(self):
        return _Clock()

    def destroy_publisher(self, _pub):
        pass


def test_topics():
    assert lite.joints_topic('robot2') == '/robot2_lite/joint_states'
    assert lite.pose_topic('robot2') == '/robot2_lite/ee_pose'


def test_publishes_joints_and_pose():
    node = _Node()
    pose = Pose()
    pose.position.x = 0.5
    pub = lite.LitePublisher(node, 'robot2', lambda: [0.1, 0.2], lambda: [1.0, 2.0],
                             lambda: [], lambda: pose)
    pub.publish()
    js = node.pubs['/robot2_lite/joint_states'].sent[-1]
    assert list(js.position) == [0.1, 0.2] and list(js.velocity) == [1.0, 2.0]
    assert list(js.name) == ['joint_1', 'joint_2'] and list(js.effort) == []
    ps = node.pubs['/robot2_lite/ee_pose'].sent[-1]
    assert ps.pose.position.x == 0.5 and ps.header.frame_id == 'robot2_base'


def test_nothing_before_there_are_joints():
    node = _Node()
    lite.LitePublisher(node, 'robot1', lambda: [], lambda: [], lambda: [], lambda: None).publish()
    assert node.pubs['/robot1_lite/joint_states'].sent == []
