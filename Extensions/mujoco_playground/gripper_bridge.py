#!/usr/bin/env python3
"""Gripper bridge node — translates FIC's hardware gripper-width convention
to the MuJoCo simulator's actuator-command topic.

Subscribes to:
    {rid}_gripper_width    std_msgs/Float32       gripper aperture in metres

Publishes to:
    simulation/actuators_cmds  communication/ActuatorCmds  ctrl for "gripper" actuator

The translation is linear — width in [0, max_width] → actuator ctrl in [0, max_ctrl].
For Robotiq 85 (max width 0.085 m, ctrl range [0, 0.85]) this is a 10× scaling.

Usage:
    python3 gripper_bridge.py --robot-id robot1 [--max-width 0.085] [--max-ctrl 0.85]

The simulator's MJCF must have an actuator literally named "gripper" for this to
have any effect — if not, the bridge logs a one-time warning and the messages
are silently dropped by the simulator's actuator_cmd_callback.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from communication.msg import ActuatorCmds


class GripperBridge(Node):
    def __init__(self, robot_id: str, max_width: float, max_ctrl: float,
                 actuator_prefix: str = "", topic_prefix: str = ""):
        super().__init__("gripper_bridge")
        self._max_width = max_width
        self._max_ctrl = max_ctrl
        self._actuator_name = actuator_prefix + "gripper"
        self._latest = None

        cmd_topic = f"{topic_prefix}/actuators_cmds" if topic_prefix else f"{robot_id}_simulation/actuators_cmds"

        self._sub = self.create_subscription(
            Float32, f"{robot_id}_gripper_width", self._on_width, 10
        )
        self._pub = self.create_publisher(
            ActuatorCmds, cmd_topic, 10
        )
        self.get_logger().info(
            f"gripper_bridge: {robot_id}_gripper_width -> {cmd_topic} "
            f"(actuator '{self._actuator_name}', "
            f"width 0..{max_width:.3f} m mapped to ctrl 0..{max_ctrl:.3f})"
        )

    def _on_width(self, msg: Float32):
        w = max(0.0, min(self._max_width, float(msg.data)))
        ctrl = self._max_ctrl * (w / self._max_width) if self._max_width > 0 else 0.0
        out = ActuatorCmds()
        out.header.stamp = self.get_clock().now().to_msg()
        out.actuators_name = [self._actuator_name]
        out.torque = [float(ctrl)]   # simulator reads d->ctrl[id] = msg->torque[k]
        self._pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", default="robot1",
                        help="FIC rid string (e.g. robot1).")
    parser.add_argument("--max-width", type=float, default=0.085,
                        help="Maximum gripper aperture in metres (default 0.085 = Robotiq 85).")
    parser.add_argument("--max-ctrl", type=float, default=0.85,
                        help="Actuator ctrl value at max_width (default 0.85 — matches Robotiq 85 joint range).")
    parser.add_argument("--actuator-prefix", default="",
                        help="Name prefix for the gripper actuator in multi-robot scenes.")
    parser.add_argument("--topic-prefix", default="",
                        help="Simulation topic prefix (e.g. robot1_simulation).")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GripperBridge(args.robot_id, args.max_width, args.max_ctrl,
                         args.actuator_prefix, args.topic_prefix)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
