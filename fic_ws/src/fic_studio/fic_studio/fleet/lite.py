"""The light state feed: a robot's state at 50 Hz, for everyone off that robot's PC.

A controller or simulator publishes its state at 1 kHz on several topics. That is
right on the robot's own PC and too much across wifi for every Master, extension
and developer script that wants to watch a robot. Each Studio therefore republishes
its own station's state, ``RATE_HZ`` times a second, on two standard topics:

  /robotN_lite/joint_states   sensor_msgs/JointState   position, velocity, effort
  /robotN_lite/ee_pose        geometry_msgs/PoseStamped  end-effector pose (if known)

best-effort (sensor QoS). A Studio that drives a remote robot reads these instead
of the 1 kHz topics; an FIC API script can too (``Robot(..., state='lite')``).
"""

RATE_HZ = 50.0


def joints_topic(rid: str) -> str:
    """Return the light joint-state topic of ``rid`` ('robotN')."""
    return '/%s_lite/joint_states' % rid


def pose_topic(rid: str) -> str:
    """Return the light end-effector pose topic of ``rid``."""
    return '/%s_lite/ee_pose' % rid


class LitePublisher:
    """Publish one station's state from getters; call ``publish()`` at RATE_HZ."""

    def __init__(self, node, rid, positions, velocities, efforts, pose):
        """Create the two publishers. The getters return lists (radians) or a Pose / None."""
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        self._node = node
        self._JointState, self._PoseStamped = JointState, PoseStamped
        self._get = (positions, velocities, efforts, pose)
        self._names = []
        self._frame = rid + '_base'
        self._jpub = node.create_publisher(JointState, joints_topic(rid), qos_profile_sensor_data)
        self._ppub = node.create_publisher(PoseStamped, pose_topic(rid), qos_profile_sensor_data)

    def publish(self):
        """Publish the current state; nothing when there are no joint positions yet."""
        positions, velocities, efforts, pose = (g() for g in self._get)
        if not positions:
            return
        n = len(positions)
        if len(self._names) != n:
            self._names = ['joint_%d' % (i + 1) for i in range(n)]
        stamp = self._node.get_clock().now().to_msg()
        msg = self._JointState()
        msg.header.stamp = stamp
        msg.name = self._names
        msg.position = [float(v) for v in positions]
        msg.velocity = [float(v) for v in (velocities or [])][:n] if velocities else []
        msg.effort = [float(v) for v in (efforts or [])][:n] if efforts else []
        self._jpub.publish(msg)
        if pose is not None:
            ps = self._PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self._frame
            ps.pose = pose
            self._ppub.publish(ps)

    def close(self):
        """Destroy the publishers."""
        for pub in (self._jpub, self._ppub):
            try:
                self._node.destroy_publisher(pub)
            except Exception:
                pass


def subscribe(node, rid, on_joints, on_pose):
    """Subscribe to ``rid``'s light feed; return the subscriptions.

    ``on_joints(positions, velocities, efforts)`` and ``on_pose(pose)`` receive plain
    lists and a geometry_msgs/Pose.
    """
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    return [
        node.create_subscription(
            JointState, joints_topic(rid),
            lambda m: on_joints(list(m.position), list(m.velocity), list(m.effort)),
            qos_profile_sensor_data),
        node.create_subscription(
            PoseStamped, pose_topic(rid), lambda m: on_pose(m.pose), qos_profile_sensor_data),
    ]
