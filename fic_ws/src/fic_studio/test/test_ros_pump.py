"""The Studio's ROS pump: run what is ready, then give the Qt thread back.

It used to call spin_once(timeout_sec=0) in a loop for 8 ms of every 10 ms
whether or not anything had arrived: ~80 % of the Qt main thread spent idling,
which on the MCU board left the web UI's commands queued for seconds.
"""
import os
import time

import pytest

os.environ.setdefault('ROS_DOMAIN_ID', '78')            # not the lab's domain
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import Int32  # noqa: E402

from fic_studio.ros_pump import drain  # noqa: E402


@pytest.fixture
def ros():
    """A context, a node and a single-threaded executor, like the Studio's."""
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('ros_pump_test_%d' % os.getpid(), context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    yield node, executor
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown(context=context)


def test_nothing_ready_returns_at_once(ros):
    """Idle: no busy wait for the rest of the budget."""
    _node, executor = ros
    drain(executor, budget_s=0.008)                     # first call builds the wait set
    t0 = time.monotonic()
    ran = drain(executor, budget_s=0.008)
    assert ran == 0
    assert time.monotonic() - t0 < 0.004


def test_ready_callbacks_all_run(ros):
    """Every message that has arrived is handled in one call."""
    node, executor = ros
    got = []
    node.create_subscription(Int32, 'ros_pump_test', lambda m: got.append(m.data), 50)
    pub = node.create_publisher(Int32, 'ros_pump_test', 50)
    deadline = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        drain(executor, budget_s=0.01)
    for k in range(20):
        pub.publish(Int32(data=k))
    deadline = time.monotonic() + 5.0
    while len(got) < 20 and time.monotonic() < deadline:
        drain(executor, budget_s=0.05)
        time.sleep(0.005)
    assert got == list(range(20))


def test_the_budget_bounds_a_flood(ros):
    """A callback that keeps re-arming itself cannot hold the Qt thread past the budget."""
    node, executor = ros
    count = [0]
    node.create_timer(0.0, lambda: count.__setitem__(0, count[0] + 1))   # always ready
    t0 = time.monotonic()
    drain(executor, budget_s=0.01)
    assert time.monotonic() - t0 < 0.05
    assert count[0] >= 1


def test_a_callback_exception_is_raised(ros):
    """Errors are not swallowed (as with spin_once)."""
    node, executor = ros

    def boom():
        raise ValueError('callback failed')
    node.create_timer(0.0, boom)
    with pytest.raises(ValueError):
        drain(executor, budget_s=0.05)


def test_the_pump_slows_down_when_idle_and_speeds_up_when_busy():
    """Each idle pass rebuilds rclpy's wait set (~3 ms on the board): poll less while nothing arrives."""
    from fic_studio.ros_pump import BUSY_MS, IDLE_MS, next_interval_ms
    assert next_interval_ms(0) == IDLE_MS
    assert next_interval_ms(3) == BUSY_MS
    assert BUSY_MS < IDLE_MS <= 30                      # ROS callbacks still within a frame or two
