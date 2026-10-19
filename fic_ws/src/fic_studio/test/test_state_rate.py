"""Robot state reaches the Studio at display rate, not at the controller's 1 kHz.

With a simulation running, fic_control streams several state topics at 1 kHz.
The Studio's state thread took every message (~6000 Python callbacks a second):
on the MCU board that thread saturated and held the interpreter lock, and the
web UI, served from the Qt thread, became near unresponsive. The Studio only
ever shows the newest value, so it keeps the newest and reads at a steady pace.
"""
import os
import sys
import threading
import time
import types

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '78')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

from fic_studio.mixins import joint_state_mixin as jsm  # noqa: E402


def _host(monkeypatch, active_robots=(1,)):
    """A JointStateMixin host on a node that records its subscriptions."""
    comm = types.ModuleType('communication')
    comm_msg = types.ModuleType('communication.msg')
    comm_msg.ContactEstimate = object
    comm.msg = comm_msg
    monkeypatch.setitem(sys.modules, 'communication', comm)
    monkeypatch.setitem(sys.modules, 'communication.msg', comm_msg)
    node = types.SimpleNamespace(active_robots=list(active_robots), station_robot_id='robot1', subs=[])
    node.create_subscription = lambda t, topic, cb, qos: node.subs.append((topic, qos)) or object()
    node.destroy_subscription = lambda sub: None
    host = jsm.JointStateMixin()
    host.node = node
    host._init_joint_state()
    return host


def test_every_state_subscription_keeps_only_the_newest_message(monkeypatch):
    host = _host(monkeypatch)
    assert host.node.subs
    for topic, qos in host.node.subs:
        assert getattr(qos, 'depth', None) == 1, topic


def test_reliability_is_unchanged(monkeypatch):
    """Depth only: a reliable reader would never match fic_control's best-effort topics."""
    from rclpy.qos import ReliabilityPolicy
    qos = dict(_host(monkeypatch).node.subs)
    assert qos['robot1_q_current'].reliability == ReliabilityPolicy.RELIABLE
    assert qos['robot1_joint_torques'].reliability == ReliabilityPolicy.BEST_EFFORT


def test_a_1khz_stream_is_read_at_the_pace_and_the_newest_value_arrives():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Int32
    from fic_studio.ros_pump import pace

    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('state_rate_test_%d' % os.getpid(), context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    got = []
    node.create_subscription(Int32, 'state_rate_test', lambda m: got.append(m.data), jsm.latest_qos(10))
    pub = node.create_publisher(Int32, 'state_rate_test', 10)
    stop = threading.Event()
    reader = threading.Thread(target=pace, args=(executor, stop, 0.02))
    reader.start()
    try:
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        for k in range(1000):                       # one second at 1 kHz
            pub.publish(Int32(data=k))
            time.sleep(0.001)
        time.sleep(0.2)
    finally:
        stop.set()
        reader.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
    assert not reader.is_alive()
    assert got and got[-1] == 999
    assert len(got) < 150, len(got)              # ~50 reads a second, not 1000
