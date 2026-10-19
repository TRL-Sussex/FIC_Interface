# ros_node.py
import socket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Pose, Twist, Wrench
from std_msgs.msg import Bool, String, Int32, Float32MultiArray, Float32
from sensor_msgs.msg import JointState


def _host_tag() -> str:
    """Short hostname tag for unique node naming (lowercase, no dots)."""
    return socket.gethostname().split('.')[0].lower().replace('-', '_')


# Per-robot publisher specs: (key, msg_type, topic_format, qos_tag)
# topic_format placeholders: {n} = numeric id, {rid} = "robotN"
# qos_tag: int → depth, "tl" → transient-local QoS for late joiners
_PER_ROBOT_PUB_SPECS = [
    ("control_mode",         Int32,             "robot{n}_control_mode",            10),
    ("position",             Pose,              "command_ee_position_change_{rid}", 10),
    ("mode_select",          String,            "{rid}_mode_select",                10),
    ("number_of_joints",     Int32,             "robot{n}_number_of_joints",        "tl"),
    ("param_update",         String,            "robot{n}_param_update",            10),
    ("target_joint_angles",  JointState,        "{rid}_target_joint_angles",        10),
    ("active_profile",       String,            "{rid}_active_profile",             "tl"),
    ("postural_enable",      Bool,              "{rid}_postural_enable",            "tl"),
    ("elbow_enable",         Bool,              "{rid}_elbow_enable",               "tl"),
    # H-PMP on/off; the nodes also accept the old {rid}_hpmp_enable.
    ("hpmp_enable",        Bool,              "{rid}_hpmp_enable",                "tl"),
    ("admittance_planner",   Bool,              "{rid}_admittance_planner_enable",  "tl"),
    # Desired wrench of the Cartesian admittance (modes 7/8, and H-PMP on them). Not
    # transient-local: a controller that (re)starts begins at zero, not at a stale wrench.
    ("desired_wrench",       Wrench,            "{rid}_desired_wrench",             10),
    ("desired_joint_angles", Float32MultiArray, "{rid}_desired_joint_angles",       10),
    ("gripper_width",        Float32,           "{rid}_gripper_width",              10),
    ("selected_robot",       String,            "{rid}_selected_robot",             10),
    ("gravity_comp",         Bool,              "{rid}_gravity_compensated",        "tl"),
    ("wrench_gravity_comp",  Bool,              "{rid}_wrench_gravity_compensated", "tl"),
    ("wrench_from_api",      Bool,              "{rid}_wrench_from_api",            "tl"),
    # Reflected contact from a teleoperation partner. Sensor QoS, not depth 10:
    # this is a stream where only the newest sample matters, and a reliable
    # queue stalls on a lost fragment over wifi. The controller deadbands,
    # saturates and fades it (robots_config.yaml `teleop_wrench_limits`).
    ("wrench_external",      Wrench,            "{rid}_wrench_external",            "sensor"),
    ("cmd_vel",              Twist,             "{rid}_cmd_vel",                     10),
    ("gait_command",         String,            "{rid}_gait_command",                10),
]


# ---------------------- ROS NODE ----------------------
#: The state thread reads robot state this far apart (50 Hz: above the web UI's
#: push rate and the dashboards' refresh).
STATE_PERIOD_S = 0.02


class FicStudioNode(Node):
    def __init__(self, station_id: int = 1, active_robots=None):
        # Unique node name per machine so two FIC Studios on the same
        # ROS domain don't collide.
        tag = _host_tag()
        node_name = f'fic_studio_{tag}'
        super().__init__(node_name)

        # QoS used by anything that needs late-joiner reliability.
        self._qos_tl = QoSProfile(depth=1)
        self._qos_tl.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Per-robot publisher table: pubs["robot1"]["mode_select"] = Publisher
        self.pubs: dict[str, dict] = {}
        self._active_robots: list[int] = []
        self._hpmp_sent: dict[str, bool] = {}   # rid -> the last H-PMP on/off sent

        # Bring up the requested robots. Default = [station_id] for back-compat.
        if active_robots is None:
            active_robots = [station_id]
        for n in active_robots:
            self.add_robot(n)

        self.get_logger().info(
            f'GUI Node initialized — active robots {self._active_robots}, host={tag}')

    # ── High-rate state subscriptions: their own node and thread ────────
    def start_state_node(self):
        """Create a companion node for robot-state subscriptions and spin it in a thread.

        A controller or simulator streams several state topics at 1 kHz. On
        the GUI node (dozens of publishers, each with QoS event handlers)
        rclpy rebuilds a large wait set for every few messages, tops out
        near 1.7 k messages/s and falls behind for good, and since that node
        is spun on the Qt thread the whole UI froze for ~29 s of every 30 s
        (measured on a robot PC running the simulator). The state node holds
        only those subscriptions, so its wait set is small, and its own
        thread keeps the UI responsive. Its callbacks only store data, so the
        thread reads them STATE_PERIOD_S apart and the subscriptions keep only
        the newest message (joint_state_mixin.latest_qos): at 1 kHz the board's
        state thread otherwise saturated and starved the Qt thread of the GIL.
        """
        import threading
        from rclpy.executors import SingleThreadedExecutor
        if getattr(self, 'state_node', None) is not None:
            return self.state_node
        self.state_node = Node(self.get_name() + '_state')
        self._state_executor = SingleThreadedExecutor()
        self._state_executor.add_node(self.state_node)
        self._state_stop = threading.Event()
        self._state_thread = threading.Thread(
            target=self._spin_state, name='fic-studio-state', daemon=True)
        self._state_thread.start()
        return self.state_node

    def _spin_state(self):
        from fic_studio.ros_pump import pace
        try:
            pace(self._state_executor, self._state_stop, STATE_PERIOD_S)
        except Exception:       # shutdown while spinning
            pass

    def stop_state_node(self):
        """Stop the state thread and destroy the state node."""
        node = getattr(self, 'state_node', None)
        if node is None:
            return
        self._state_stop.set()
        try:
            self._state_executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        self._state_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        self.state_node = None

    # ── Active-robot management ─────────────────────────────────────────
    @property
    def active_robots(self) -> list[int]:
        """Numeric ids of currently active robots, in insertion order."""
        return list(self._active_robots)

    @property
    def station_id(self) -> int:
        """Back-compat: numeric id of the first active robot (1 if none)."""
        return self._active_robots[0] if self._active_robots else 1

    @property
    def station_robot_id(self) -> str:
        """Back-compat: 'robotN' string of the first active robot."""
        return f'robot{self.station_id}'

    def add_robot(self, n: int) -> str:
        """Create publishers for robot N. Returns the rid string ('robotN').
        No-op if already active."""
        rid = f'robot{n}'
        if n in self._active_robots:
            return rid
        pubs = {}
        for key, msg_type, fmt, qos in _PER_ROBOT_PUB_SPECS:
            topic = fmt.format(n=n, rid=rid)
            if qos == "tl":
                q = self._qos_tl
            elif qos == "sensor":
                q = qos_profile_sensor_data
            else:
                q = qos
            pubs[key] = self.create_publisher(msg_type, topic, q)
        self.pubs[rid] = pubs
        # H-PMP is on by default in the Cartesian modes (latched: a controller started later
        # gets it); the toggle turns it off, sending the Cartesian torques directly.
        pubs["hpmp_enable"].publish(Bool(data=True))
        self._hpmp_sent[rid] = True
        self._active_robots.append(n)
        self._refresh_legacy_aliases()
        self.get_logger().info(f'Added robot {rid}')
        return rid

    def remove_robot(self, n: int) -> None:
        """Tear down publishers for robot N. No-op if not active."""
        rid = f'robot{n}'
        if n not in self._active_robots:
            return
        for pub in self.pubs.get(rid, {}).values():
            try:
                self.destroy_publisher(pub)
            except Exception:
                pass
        self.pubs.pop(rid, None)
        self._active_robots.remove(n)
        self._refresh_legacy_aliases()
        self.get_logger().info(f'Removed robot {rid}')

    def _refresh_legacy_aliases(self) -> None:
        """Keep the legacy single-robot attribute names pointing at the first
        active robot so callers that haven't been migrated to the per-robot
        API still publish to *something* sensible. Migrate callers to the
        new pubs[rid][key] form to address a specific robot."""
        if not self._active_robots:
            return
        first_rid = self.station_robot_id
        first = self.pubs[first_rid]
        self.mode_publisher_ = first["control_mode"]
        self.position_publisher_ = first["position"]
        self.mode_select_publisher = first["mode_select"]
        self.joints_publisher = first["number_of_joints"]
        self.param_update_publisher = first["param_update"]
        self.joint_angles_pub = first["target_joint_angles"]
        self.active_profile_publisher_ = first["active_profile"]
        self.postural_enable_publisher_ = first["postural_enable"]
        self.elbow_enable_publisher_ = first["elbow_enable"]

        # Per-rid keyed dicts (used by publisher_mixin gripper / joint paths).
        self.real_joint_publishers = {
            rid: p["desired_joint_angles"] for rid, p in self.pubs.items()
        }
        self.gripper_publishers = {
            rid: p["gripper_width"] for rid, p in self.pubs.items()
        }

        # robot_N_mode_publisher / robot_N_pose_publisher aliases used by
        # publisher_mixin's per-robot send/stop/pause helpers. If a robot N
        # is active, alias to its own pubs; otherwise alias to first so
        # not-yet-migrated callers don't AttributeError.
        for i in range(1, 5):
            rid_i = f'robot{i}'
            target = self.pubs.get(rid_i, first)
            setattr(self, f'robot_{i}_mode_publisher', target["mode_select"])
            setattr(self, f'robot_{i}_pose_publisher', target["position"])

    # ── Publishing helpers (per-robot aware) ────────────────────────────
    def _resolve_rid(self, robot_id) -> str:
        """Accept 'robotN' string, int N, or None (→ first active) and return rid."""
        if robot_id is None:
            return self.station_robot_id
        if isinstance(robot_id, int):
            return f'robot{robot_id}'
        return str(robot_id)

    def publish_gripper_width(self, robot_id, width_meters: float):
        """Publish gripper width to the given robot. robot_id may be 'robotN' or N."""
        rid = self._resolve_rid(robot_id)
        pubs = self.pubs.get(rid)
        if not pubs or "gripper_width" not in pubs:
            self.get_logger().error(
                f"No gripper publisher for {rid} (active: {list(self.pubs.keys())})")
            return
        msg = Float32()
        msg.data = float(width_meters)
        pubs["gripper_width"].publish(msg)
        self.get_logger().info(
            f"Published {rid} gripper width: {width_meters:.4f}m")

    def publish_joint_angles(self, positions, names=None, robot_id=None):
        """Publish JointState to the given robot's target_joint_angles topic."""
        rid = self._resolve_rid(robot_id)
        pubs = self.pubs.get(rid)
        if not pubs or "target_joint_angles" not in pubs:
            self.get_logger().error(f"No joint publisher for {rid}")
            return
        msg = JointState()
        msg.name = names if names else [
            f'joint_{i+1}' for i in range(len(positions))]
        msg.position = positions
        pubs["target_joint_angles"].publish(msg)

    def publish_real_joint_angles(self, robot_id, positions_rad):
        rid = self._resolve_rid(robot_id)
        pubs = self.pubs.get(rid)
        if not pubs or "desired_joint_angles" not in pubs:
            self.get_logger().error(f"No real-joint publisher for {rid}")
            return
        msg = Float32MultiArray()
        msg.data = positions_rad
        pubs["desired_joint_angles"].publish(msg)

    def publish_wrench_external(self, robot_id, wrench6):
        """Reflect a teleoperation partner's contact onto this arm.

        ``wrench6`` is [fx, fy, fz, tx, ty, tz]. Publishing zeros is the way to
        release the arm, and is what the session layer sends the moment a
        follower goes quiet. The controller clamps, deadbands and fades this,
        so nothing here needs to be trusted for safety.
        """
        rid = self._resolve_rid(robot_id)
        pubs = self.pubs.get(rid)
        if not pubs or "wrench_external" not in pubs:
            self.get_logger().error(f"No wrench_external publisher for {rid}")
            return
        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = (float(v) for v in wrench6[:3])
        msg.torque.x, msg.torque.y, msg.torque.z = (float(v) for v in wrench6[3:6])
        pubs["wrench_external"].publish(msg)

    def publish_position(self, position: Pose, robot_id=None):
        rid = self._resolve_rid(robot_id)
        self.pubs[rid]["position"].publish(position)

    _MODE_NAMES = [
        "Joint Impedance",
        "Joint Impedance With Planner",
        "Joint Admittance",
        "Joint Admittance Haptic",
        "Cartesian Impedance",
        "Cartesian Planner",
        "Cartesian Admittance",
        "Cartesian Admittance Haptic",
    ]

    def publish_mode(self, mode: int, robot_id=None):
        rid = self._resolve_rid(robot_id)
        if 5 <= mode <= 8 and rid in self.pubs:
            # The controllers start with H-PMP off and miss the H-PMP messages sent before they
            # started, so every Cartesian mode carries the last one sent to this robot (on, as the
            # toggle starts): a controller started later then runs as one that ran all along.
            self.pubs[rid]["hpmp_enable"].publish(Bool(data=self._hpmp_sent.get(rid, True)))
        if 1 <= mode <= len(self._MODE_NAMES):
            self.pubs[rid]["mode_select"].publish(
                String(data=self._MODE_NAMES[mode - 1]))
        self.pubs[rid]["control_mode"].publish(Int32(data=mode))

    def publish_postural_enable(self, enabled: bool, robot_id=None):
        rid = self._resolve_rid(robot_id)
        self.pubs[rid]["postural_enable"].publish(Bool(data=bool(enabled)))

    def publish_elbow_enable(self, enabled: bool, robot_id=None):
        rid = self._resolve_rid(robot_id)
        self.pubs[rid]["elbow_enable"].publish(Bool(data=bool(enabled)))

    def publish_hpmp_enable(self, enabled: bool, robot_id=None):
        rid = self._resolve_rid(robot_id)
        self._hpmp_sent[rid] = bool(enabled)
        self.pubs[rid]["hpmp_enable"].publish(Bool(data=bool(enabled)))

    def publish_admittance_planner(self, enabled: bool, robot_id=None):
        rid = self._resolve_rid(robot_id)
        self.pubs[rid]["admittance_planner"].publish(Bool(data=bool(enabled)))

    def publish_desired_wrench(self, wrench, robot_id=None):
        """Publish the admittance's desired wrench [Fx, Fy, Fz, Mx, My, Mz] (N, N m)."""
        values = [float(v) for v in wrench]
        if len(values) != 6:
            raise ValueError('a desired wrench has six values, got %d' % len(values))
        msg = Wrench()
        msg.force.x, msg.force.y, msg.force.z = values[:3]
        msg.torque.x, msg.torque.y, msg.torque.z = values[3:]
        rid = self._resolve_rid(robot_id)
        self.pubs[rid]["desired_wrench"].publish(msg)

    def publish_joints(self, num_joints: int, robot_id=None):
        rid = self._resolve_rid(robot_id)
        pubs = self.pubs.get(rid)
        if pubs and "number_of_joints" in pubs:
            pubs["number_of_joints"].publish(Int32(data=num_joints))

    def publish_param_update(self, robot_id, param_type, updated_param):
        rid = self._resolve_rid(robot_id)
        msg = String()
        msg.data = f"{rid}:{param_type}:{updated_param}"
        self.pubs[rid]["param_update"].publish(msg)

    def publish_robot(self, robot_name: str, robot_id=None):
        rid = self._resolve_rid(robot_id)
        pubs = self.pubs.get(rid)
        if pubs and "selected_robot" in pubs:
            pubs["selected_robot"].publish(String(data=robot_name))


