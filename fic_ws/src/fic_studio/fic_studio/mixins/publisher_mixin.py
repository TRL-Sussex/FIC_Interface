# mixins/publisher_mixin.py — ROS publishing methods

from PyQt5.QtWidgets import QMessageBox
from geometry_msgs.msg import Pose


def _resolve_rid(node, robot_id):
    """Accept None | int N | 'robotN' | 'robot_N' and return 'robotN' string
    (the topic-prefix form, no underscore)."""
    if robot_id is None or robot_id == "":
        return node.station_robot_id
    if isinstance(robot_id, int):
        return f'robot{robot_id}'
    s = str(robot_id)
    # Tolerate the widget-prefix form ('robot_1') as well.
    if s.startswith("robot_") and s[len("robot_"):].split("_", 1)[0].isdigit():
        n = s[len("robot_"):].split("_", 1)[0]
        return f'robot{n}'
    return s


def _widget_prefix(rid: str) -> str:
    """Convert topic rid 'robot1' → widget prefix 'robot_1'.
    Per-card UI widgets are named e.g. `robot_1_pos_x`, while topics use
    `robot1_*`. Use this when looking up Qt widgets by name."""
    if rid.startswith("robot") and not rid.startswith("robot_"):
        n = rid[len("robot"):]
        if n.isdigit():
            return f"robot_{n}"
    return rid


class PublisherMixin:
    """Mixin for all ROS publishing methods (modes, poses, joints, gripper).

    Two APIs coexist here:
      • Per-robot helpers (`send_mode_for`, `send_pose_for`, `stop_for`, …) —
        used by independent robot cards and broadcast-aware extensions.
      • Legacy `send_robot_N_*` aliases that read fixed widget names — kept
        so the existing UI keeps working until cards/extensions are migrated.
    """

    # ================================================================== Gripper publishers
    def _setup_gripper_publishers(self):
        """No-op: gripper publishers live in node.pubs[rid]['gripper_width']."""
        pass

    def publish_gripper_width(self, robot_id, width_meters: float):
        """Publish the desired gripper width in meters to the given robot."""
        from std_msgs.msg import Float32

        rid = _resolve_rid(self.node, robot_id)
        pubs = self.node.pubs.get(rid)
        if not pubs or "gripper_width" not in pubs:
            self.log_message("ERROR", f"No gripper publisher for {rid}", rid)
            return
        msg = Float32()
        msg.data = float(width_meters)
        pubs["gripper_width"].publish(msg)
        pct = (width_meters / 0.08) * 100
        self.log_message("INFO", f"Gripper: {width_meters:.4f}m ({pct:.0f}%)", rid)

    # ================================================================== Mode publishers (generic)
    def _publish_mode_string(self, publisher, mode_text: str, robot_id: str = ""):
        from std_msgs.msg import String
        msg = String()
        msg.data = mode_text
        publisher.publish(msg)
        self.node.get_logger().info(f"Published mode: {mode_text}")
        self.log_message("INFO", f"Mode: {mode_text}", robot_id or "Mode")

    def send_mode_for(self, robot_id, mode_text: str):
        """Publish a mode string to the given robot's mode_select topic."""
        rid = _resolve_rid(self.node, robot_id)
        pubs = self.node.pubs.get(rid)
        if not pubs:
            self.log_message("ERROR", f"No publishers for {rid}", rid)
            return
        self._publish_mode_string(pubs["mode_select"], mode_text, rid)

    def stop_for(self, robot_id):
        rid = _resolve_rid(self.node, robot_id)
        self.send_mode_for(rid, "Stop Controller")
        self.log_message("WARN", "STOP command sent", rid)

    def pause_for(self, robot_id):
        rid = _resolve_rid(self.node, robot_id)
        self.send_mode_for(rid, "Pause Controller")

    def broadcast_mode(self, mode_text: str):
        """Publish a mode to every active robot. Used by safety/global ops."""
        for n in self.node.active_robots:
            self.send_mode_for(n, mode_text)

    def broadcast_stop(self):
        for n in self.node.active_robots:
            self.stop_for(n)

    # ================================================================== Mode publishers (legacy per-card)
    def send_robot_1_mode(self):
        dd = getattr(self, 'robot_1_mode_dropdown', None)
        if dd:
            self._publish_mode_string(self.node.robot_1_mode_publisher, dd.currentText(), "robot1")

    def send_robot_2_mode(self):
        dd = getattr(self, 'robot_2_mode_dropdown', None)
        if dd:
            self._publish_mode_string(self.node.robot_2_mode_publisher, dd.currentText(), "robot2")

    def send_robot_3_mode(self):
        dd = getattr(self, 'robot_3_mode_dropdown', None)
        if dd:
            self._publish_mode_string(self.node.robot_3_mode_publisher, dd.currentText(), "robot3")

    def send_robot_4_mode(self):
        dd = getattr(self, 'robot_4_mode_dropdown', None)
        if dd:
            self._publish_mode_string(self.node.robot_4_mode_publisher, dd.currentText(), "robot4")

    # ================================================================== Pose publishers
    def send_pose_for(self, robot_id, pose=None):
        """Publish ``pose`` to ``robot_id``, or read this card's fields for one.

        ``pose`` exists because the caller is not always this card. The
        Robot Control page builds a pose from its OWN Cartesian fields
        (``x_input``/``ax_input``...) and used to hand off to this method
        on the hardware target, which then discarded it and re-derived a
        pose from the per-robot card widgets (``robot_1_pos_x``...). Those
        sit at ``default_ee_target``, so every hardware send published the
        config default whatever the operator typed -- measured on the arm
        as four consecutive sends arriving byte-identical to the home pose
        (0.380, 0.000, 0.432), while the page logged the typed values.

        Passing the pose through keeps the publisher guard and the logging
        below, which a direct ``publish_position`` call would skip.
        """
        rid = _resolve_rid(self.node, robot_id)
        if pose is None:
            pose = self.get_pose_from_inputs(_widget_prefix(rid))
        if pose is None:
            return
        pubs = self.node.pubs.get(rid)
        if not pubs:
            self.log_message("ERROR", f"No publishers for {rid}", rid)
            return
        pubs["position"].publish(pose)
        self._log_pose_publish(rid, pose)

    # Legacy per-card pose helpers kept for not-yet-migrated UI wiring.
    def send_robot_1_pose(self, pose=None): self.send_pose_for("robot1", pose)
    def send_robot_2_pose(self, pose=None): self.send_pose_for("robot2", pose)
    def send_robot_3_pose(self, pose=None): self.send_pose_for("robot3", pose)
    def send_robot_4_pose(self, pose=None): self.send_pose_for("robot4", pose)

    def _log_pose_publish(self, robot_id: str, pose: Pose):
        """Log a pose publish to the log viewer."""
        pos_str = f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
        self.node.get_logger().info(f"[{robot_id}] Published pose: {pos_str}")
        self.log_message("INFO", f"Pose: {pos_str}", robot_id)

    # ================================================================== Stop/Pause (legacy)
    def stop_robot_1(self): self.stop_for("robot1")
    def pause_robot_1(self): self.pause_for("robot1")
    def stop_robot_2(self): self.stop_for("robot2")
    def pause_robot_2(self): self.pause_for("robot2")
    def stop_robot_3(self): self.stop_for("robot3")
    def pause_robot_3(self): self.pause_for("robot3")
    def stop_robot_4(self): self.stop_for("robot4")
    def pause_robot_4(self): self.pause_for("robot4")

    def publish_mode(self, publisher, mode, robot_id: str = ""):
        from std_msgs.msg import String
        msg = String()
        msg.data = mode
        publisher.publish(msg)
        self.node.get_logger().info(f'Published mode command: {mode}')
        self.log_message("INFO", f"Mode: {mode}", robot_id or "Mode")

    # ================================================================== Pose builder
    def get_pose_from_inputs(self, robot_id: str):
        """
        Build a geometry_msgs/Pose from the per-robot Position + Axis–Angle fields.
        """
        from fic_studio.math_helpers import _axis_angle_to_quat

        # --- Position ---
        try:
            px = float(getattr(self, f'{robot_id}_pos_x').text().strip())
            py = float(getattr(self, f'{robot_id}_pos_y').text().strip())
            pz = float(getattr(self, f'{robot_id}_pos_z').text().strip())
        except Exception:
            QMessageBox.critical(self, "Input Error", f"[{robot_id}] Please enter valid X, Y, Z.")
            return None

        # --- Axis–Angle → Quaternion ---
        try:
            ax = float(getattr(self, f'{robot_id}_axis_x').text().strip())
            ay = float(getattr(self, f'{robot_id}_axis_y').text().strip())
            az = float(getattr(self, f'{robot_id}_axis_z').text().strip())
            ang = float(getattr(self, f'{robot_id}_angle').text().strip())
            units = getattr(self, f'{robot_id}_angle_units').currentText()
        except Exception:
            ax = ay = 0.0
            az = 1.0
            ang = 0.0
            units = "deg"

        qx, qy, qz, qw = _axis_angle_to_quat(ax, ay, az, ang, units)

        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    # ================================================================== Axis-angle conversion
    def convert_axis_angle_to_quat_fields(self, robot_id: str):
        """Convert axis-angle inputs to quaternion and populate fields."""
        from fic_studio.math_helpers import _axis_angle_to_quat

        try:
            ax = float(getattr(self, f'{robot_id}_axis_x').text())
            ay = float(getattr(self, f'{robot_id}_axis_y').text())
            az = float(getattr(self, f'{robot_id}_axis_z').text())
            ang = float(getattr(self, f'{robot_id}_angle').text())
            units = getattr(self, f'{robot_id}_angle_units').currentText()
        except ValueError:
            QMessageBox.warning(self, "Input Error", f"[{robot_id}] Please enter valid axis and angle values.")
            return

        qx, qy, qz, qw = _axis_angle_to_quat(ax, ay, az, ang, units)
        getattr(self, f'{robot_id}_orient_x').setText(f"{qx:.6f}")
        getattr(self, f'{robot_id}_orient_y').setText(f"{qy:.6f}")
        getattr(self, f'{robot_id}_orient_z').setText(f"{qz:.6f}")
        getattr(self, f'{robot_id}_orient_w').setText(f"{qw:.6f}")
        self.node.get_logger().info(
            f'[{robot_id}] Axis–Angle converted to quat: ({qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f})'
        )

    # ================================================================== Send pose to target
    def _send_pose_to_target_position(self, robot_id: str):
        """Send current pose inputs to the given robot's target position publisher."""
        rid = _resolve_rid(self.node, robot_id)
        pose = self.get_pose_from_inputs(_widget_prefix(rid))
        if pose is None:
            return
        pubs = self.node.pubs.get(rid)
        if not pubs:
            return
        pubs["position"].publish(pose)
        self.node.get_logger().info(f"[{rid}] Sent pose to target")
