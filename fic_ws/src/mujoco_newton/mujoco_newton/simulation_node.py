"""GPU-accelerated MuJoCo simulation backed by NVIDIA Newton.

Mirrors the topic interface of the CPU simulator (`mujoco_native`) so the FIC controller
can swap backends without code changes.

Published topics (under `simulation/`):
    imu_data        sensor_msgs/Imu
    joint_states    sensor_msgs/JointState
    odom            nav_msgs/Odometry
    gravity_torques std_msgs/Float64MultiArray
    force_torque    geometry_msgs/Wrench   (zeros if no F/T sensor in MJCF)

Subscribed topics:
    actuators_cmds  communication/msg/ActuatorCmds

Usage:
    ros2 launch mujoco_newton simulation_launch.py robot_name:=Franka_FR3
"""

from __future__ import annotations

import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench
from communication.msg import ActuatorCmds


def _sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos



def gravity_from_qpos(mj_model, scratch, qpos, nv) -> list:
    """g(q): ``qfrc_bias`` with zero velocity, on a scratch ``MjData``.

    ``qfrc_bias`` is the Coriolis/centrifugal/gravity bias; at qvel = 0 only
    gravity is left, and it depends on qpos alone, so the actuator command
    (``ctrl``) cannot leak into it. ``scratch`` is never the stepped data.
    """
    import mujoco
    n_q = min(len(qpos), len(scratch.qpos))
    scratch.qpos[:n_q] = qpos[:n_q]
    scratch.qvel[:] = 0.0
    scratch.ctrl[:] = 0.0
    mujoco.mj_forward(mj_model, scratch)
    return [float(x) for x in scratch.qfrc_bias[:nv]]

class NewtonSimulation(Node):
    def __init__(self) -> None:
        super().__init__('simulation_mujoco_newton')

        self.declare_parameter('simulation/model_file', '')
        self.declare_parameter('headless', False)
        self.declare_parameter('sim_dt', 0.002)         # 500 Hz physics step
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('topic_prefix', 'simulation')
        # Start pose of a fixed-base arm: comma-separated angles (rad) for its
        # first hinge joints, in model order; empty = all joints at 0.
        self.declare_parameter('initial_joint_positions', '')

        model_file = self.get_parameter('simulation/model_file').value
        if not model_file or not os.path.exists(model_file):
            raise RuntimeError(f"MJCF model file not found: {model_file!r}")

        self._headless = bool(self.get_parameter('headless').value)
        self._dt = float(self.get_parameter('sim_dt').value)
        self._publish_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self._device = str(self.get_parameter('device').value)

        # Newton imports are deferred until ROS init succeeds so a missing
        # newton install fails with a clean message instead of a stack trace
        # at module load time.
        import newton  # noqa: F401
        import warp as wp
        self._wp = wp
        self._newton = newton

        wp.init()
        self.get_logger().info(
            f"Newton init OK; device={self._device}; loading MJCF: {model_file}")

        self._build_model(model_file)
        self._spawn_q = None              # held until the first command (see run_sim_loop)
        self._apply_initial_joint_positions()
        self._build_pubsubs()
        self._build_viewer()

        self._cmd_lock = threading.Lock()
        self._latest_cmd: ActuatorCmds | None = None
        self._last_cmd_stamp = 0.0
        self._stop = threading.Event()
        # When a viewer is attached, the sim+render loop must run on the main
        # thread (pyglet/OpenGL constraint). main() handles thread layout.

    # ------------------------------------------------------------------
    #  Setup
    # ------------------------------------------------------------------
    def _build_model(self, model_file: str) -> None:
        nw = self._newton
        builder = nw.ModelBuilder(up_axis=nw.Axis.Z)
        # Pass the absolute path so Newton can resolve <compiler meshdir>
        # and <mesh file=...> references relative to the MJCF location.
        builder.add_mjcf(
            os.path.abspath(model_file),
            collapse_fixed_joints=False,
            ignore_inertial_definitions=False,
            parse_mujoco_options=True,
        )

        self._model = builder.finalize(device=self._device)
        self._state_in = self._model.state()
        self._state_out = self._model.state()
        self._control = self._model.control()

        # Newton's MJCF importer prefixes joint names with the body path
        # ("pendulum/worldbody/base_link/Actuator1"). FIC controllers and the
        # CPU simulator (mujoco_native) use bare joint names, so strip the prefix.
        labels = list(getattr(self._model, 'joint_label', []) or [])
        self._joint_names = [n.rsplit('/', 1)[-1] for n in labels if n]
        self._body_label = list(getattr(self._model, 'body_label', []) or [])

        # Per-joint DoF starting indices. Fixed joints have 0 DoFs so the
        # position of a joint in joint_label is NOT the same as its DoF index.
        # Newton stores joint_qd_start[i] = first DoF in state.joint_qd /
        # control.joint_act for joint i.
        qd_start_attr = getattr(self._model, 'joint_qd_start', None)
        q_start_attr = getattr(self._model, 'joint_q_start', None)
        self._joint_qd_start = (
            qd_start_attr.numpy() if hasattr(qd_start_attr, 'numpy')
            else np.asarray(qd_start_attr) if qd_start_attr is not None
            else np.arange(len(self._joint_names) + 1))
        self._joint_q_start = (
            q_start_attr.numpy() if hasattr(q_start_attr, 'numpy')
            else np.asarray(q_start_attr) if q_start_attr is not None
            else np.arange(len(self._joint_names) + 1))
        # Indices of joints that actually contribute DoFs (skip fixed joints).
        self._active_joint_indices = [
            i for i in range(len(self._joint_names))
            if i + 1 < len(self._joint_qd_start)
            and self._joint_qd_start[i + 1] > self._joint_qd_start[i]
        ]

        # Build an actuator-name → DoF-index map by parsing the MJCF directly.
        # Newton 1.1.0's MJCF importer ignores <actuator> blocks, so the FIC
        # controller's actuators_cmds (which addresses motors by name like
        # "torq_j1") would never match raw joint labels. Parse <motor joint=...
        # name=...> entries and resolve through joint_label → DoF index.
        self._actuator_to_dof = self._build_actuator_map(model_file)
        self.get_logger().info(
            f"Actuator map: {len(self._actuator_to_dof)} entries "
            f"(e.g. {list(self._actuator_to_dof.items())[:3]})")

        # Solver: SolverMuJoCo runs MuJoCo's solver on Warp (GPU). We don't
        # override integrator/cone here — the MJCF's <option integrator=.../>
        # tag drives that, and mujoco_warp only supports a subset
        # (euler, implicitfast); forcing values causes NotImplementedError on
        # MJCFs that declare e.g. integrator="Euler".
        self._solver = nw.solvers.SolverMuJoCo(
            self._model,
            use_mujoco_cpu=False,
        )

        self._contacts = self._model.collision_pipeline.create_contacts() \
            if hasattr(self._model, 'collision_pipeline') else None

        self.get_logger().info(
            f"Model finalized: {len(self._joint_names)} named joints, "
            f"{self._model.joint_dof_count} DoF, {self._model.body_count} bodies.")

    def _apply_initial_joint_positions(self) -> None:
        """Start a fixed-base arm at `initial_joint_positions` (e.g. its home).

        The angles go to the first one-DoF joints in model order, and the pose is
        held until the first actuator command so the arm does not sag away from
        it before the controller connects. Empty (the default) keeps all zeros;
        a model with a free joint is left alone.
        """
        raw = str(self.get_parameter('initial_joint_positions').value or '').strip()
        if not raw:
            return
        try:
            values = [float(v) for v in raw.replace(' ', '').split(',') if v]
        except ValueError:
            self.get_logger().warn(
                f"initial_joint_positions {raw!r} is not a list of numbers; ignored.")
            return
        one_dof = [i for i in range(len(self._joint_names))
                   if i + 1 < len(self._joint_qd_start)
                   and self._joint_qd_start[i + 1] - self._joint_qd_start[i] == 1
                   and self._joint_q_start[i + 1] - self._joint_q_start[i] == 1]
        multi = any(i + 1 < len(self._joint_qd_start)
                    and self._joint_qd_start[i + 1] - self._joint_qd_start[i] > 1
                    for i in range(len(self._joint_names)))
        if multi or len(values) > len(one_dof):
            self.get_logger().warn(
                "initial_joint_positions applies to fixed-base arms with at least "
                f"{len(values)} one-DoF joints; ignored.")
            return
        q = self._state_in.joint_q.numpy().copy()
        for j, value in zip(one_dof, values):
            q[int(self._joint_q_start[j])] = value
        for state in (self._state_in, self._state_out):
            state.joint_q.assign(q)
            state.joint_qd.zero_()
        self._spawn_q = q
        self.get_logger().info(
            f"Spawned at initial_joint_positions ({len(values)} joints); "
            f"holding this pose until the first actuator command arrives.")

    def _build_actuator_map(self, mjcf_path: str) -> dict:
        """Map MJCF actuator name → DoF index in self._control.joint_act.

        Parses <motor>/<position>/<velocity> entries inside <actuator> blocks.
        For each entry, the `joint=` attribute names the joint it drives;
        we resolve that joint to its DoF index by looking up the bare joint
        name in self._joint_names. Returns {} if parsing fails (caller falls
        back to positional ordering).
        """
        import xml.etree.ElementTree as ET

        actuator_to_joint: dict[str, str] = {}

        def _walk(path: str, seen: set):
            if path in seen:
                return
            seen.add(path)
            try:
                root = ET.parse(path).getroot()
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not parse MJCF {path} for actuator map: {exc}")
                return
            base = os.path.dirname(os.path.abspath(path))
            # Recurse into <include file=...>.
            for inc in root.iter('include'):
                inc_file = inc.get('file')
                if inc_file:
                    inc_abs = inc_file if os.path.isabs(inc_file) \
                        else os.path.join(base, inc_file)
                    _walk(inc_abs, seen)
            # Collect motor/position/velocity entries.
            for actuator in root.iter('actuator'):
                for child in actuator:
                    if child.tag in ('motor', 'position', 'velocity', 'general'):
                        name = child.get('name')
                        joint = child.get('joint')
                        if name and joint:
                            actuator_to_joint[name] = joint

        _walk(os.path.abspath(mjcf_path), set())

        # Resolve joint names to DoF indices using joint_qd_start so fixed
        # joints (which take a slot in joint_label but contribute 0 DoFs)
        # don't shift the mapping.
        result: dict[str, int] = {}
        for actu_name, joint_name in actuator_to_joint.items():
            try:
                joint_idx = self._joint_names.index(joint_name)
            except ValueError:
                continue
            if joint_idx + 1 >= len(self._joint_qd_start):
                continue
            dof_start = int(self._joint_qd_start[joint_idx])
            dof_end = int(self._joint_qd_start[joint_idx + 1])
            if dof_end <= dof_start:
                continue  # fixed joint, can't actuate it
            result[actu_name] = dof_start
        return result

    def _build_viewer(self) -> None:
        if self._headless:
            self._viewer = None
            return
        try:
            wp = self._wp
            self._viewer = self._newton.viewer.ViewerGL()
            self._viewer.set_model(self._model)
            # Position the camera so the robot at the origin is in frame on
            # first launch. Newton's set_camera takes pitch/yaw in DEGREES
            # (despite no docstring saying so — the source clamps pitch to
            # ±89.0 and computes front via np.deg2rad). For a Z-up world,
            # camera at (2,-2,1.5) looking at (0,0,0.5) →
            #   yaw   = atan2(0-(-2), 0-2)  = 135°  (CCW from +X)
            #   pitch = asin((0.5-1.5)/dist) ≈ -20° (looking slightly down)
            self._viewer.set_camera(
                pos=wp.vec3(2.0, -2.0, 1.5),
                pitch=-20.0,
                yaw=135.0,
            )
            self.get_logger().info("Newton ViewerGL initialised.")
        except Exception as exc:  # pragma: no cover — display-dependent
            self.get_logger().warn(f"Falling back to headless mode: {exc}")
            self._viewer = None

    def _build_pubsubs(self) -> None:
        qos = _sensor_qos()
        prefix = self.get_parameter('topic_prefix').value + '/'
        self._pub_imu = self.create_publisher(Imu, prefix + 'imu_data', qos)
        self._pub_joint = self.create_publisher(JointState, prefix + 'joint_states', qos)
        self._pub_odom = self.create_publisher(Odometry, prefix + 'odom', qos)
        self._pub_grav = self.create_publisher(
            Float64MultiArray, prefix + 'gravity_torques', qos)
        self._pub_ft = self.create_publisher(Wrench, prefix + 'force_torque', qos)

        self._sub_cmd = self.create_subscription(
            ActuatorCmds, prefix + 'actuators_cmds', self._on_cmd, qos)

    # ------------------------------------------------------------------
    #  Simulation loop  (must run on the main thread when viewer is active —
    #  pyglet/OpenGL won't render off-thread, leading to a black window.)
    # ------------------------------------------------------------------
    def run_sim_loop(self) -> None:
        wp = self._wp
        next_publish = time.monotonic()

        while not self._stop.is_set() and rclpy.ok():
            t_start = time.monotonic()

            # Bail out if the user closed the viewer window.
            if self._viewer is not None and not self._viewer.is_running():
                self.get_logger().info("Viewer window closed; shutting down.")
                self._stop.set()
                break

            self._apply_latest_cmd()
            if self._spawn_q is not None:
                with self._cmd_lock:
                    commanded = self._latest_cmd is not None
                if commanded:
                    self._spawn_q = None   # the controller has spoken: release for good
                    self.get_logger().info(
                        "First actuator command received; releasing spawn-pose hold.")
                else:
                    self._state_in.joint_q.assign(self._spawn_q)
                    self._state_in.joint_qd.zero_()

            # Step physics (the SolverMuJoCo writes into state_out).
            with wp.ScopedDevice(self._device):
                self._solver.step(
                    self._state_in,
                    self._state_out,
                    self._control,
                    self._contacts,
                    self._dt,
                )
            # Swap states for the next step.
            self._state_in, self._state_out = self._state_out, self._state_in

            now = time.monotonic()
            if now >= next_publish:
                try:
                    self._publish_state()
                except Exception as exc:
                    # Publisher context goes invalid during shutdown; stop the
                    # loop instead of spamming tracebacks.
                    if rclpy.ok():
                        self.get_logger().warn(f"publish_state failed: {exc}")
                    self._stop.set()
                    break
                next_publish = now + self._publish_period

            if self._viewer is not None:
                try:
                    self._viewer.begin_frame(time.time())
                    self._viewer.log_state(self._state_in)
                    self._viewer.end_frame()
                except Exception as exc:  # pragma: no cover
                    self.get_logger().warn(f"Viewer error: {exc}; disabling viewer.")
                    self._viewer = None

            # Real-time pacing: sleep if we ran ahead of dt.
            elapsed = time.monotonic() - t_start
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    # ------------------------------------------------------------------
    #  Command handling
    # ------------------------------------------------------------------
    def _on_cmd(self, msg: ActuatorCmds) -> None:
        with self._cmd_lock:
            self._latest_cmd = msg
            self._last_cmd_stamp = time.monotonic()

    def _apply_latest_cmd(self) -> None:
        with self._cmd_lock:
            cmd = self._latest_cmd
            stale = (time.monotonic() - self._last_cmd_stamp) > 0.2

        if cmd is None:
            return

        # Drop torques older than 200 ms (matches the C++ node's drop_old_message).
        torques = np.zeros(self._model.joint_dof_count, dtype=np.float32)
        n_applied = 0
        if not stale:
            for i, name in enumerate(cmd.actuators_name):
                if i >= len(cmd.torque):
                    break
                # Prefer the actuator-name → DoF map built from the MJCF's
                # <actuator> block. FIC publishes actuator names like
                # "torq_j1" that don't match Newton's joint labels directly.
                dof_idx = self._actuator_to_dof.get(name)
                if dof_idx is None:
                    # Fallback: some upstream callers send joint names instead
                    # of actuator names. Resolve the same way.
                    try:
                        j_idx = self._joint_names.index(name)
                    except ValueError:
                        continue
                    if j_idx + 1 >= len(self._joint_qd_start):
                        continue
                    dof_idx = int(self._joint_qd_start[j_idx])
                    if int(self._joint_qd_start[j_idx + 1]) <= dof_idx:
                        continue
                if 0 <= dof_idx < len(torques):
                    torques[dof_idx] = float(cmd.torque[i])
                n_applied += 1

        # Diagnostic: log the first few cmds so users can confirm the FIC
        # controller is wired up correctly. Suppressed after 5 logs to avoid
        # spamming at 1 kHz.
        if not stale and not getattr(self, '_cmd_log_done', False):
            self._cmd_log_count = getattr(self, '_cmd_log_count', 0) + 1
            if self._cmd_log_count <= 5:
                self.get_logger().info(
                    f"actuator cmd #{self._cmd_log_count}: "
                    f"applied {n_applied}/{len(cmd.actuators_name)} torques "
                    f"(names={list(cmd.actuators_name)[:3]}...)")
            if self._cmd_log_count == 5:
                self._cmd_log_done = True
                self.get_logger().info(
                    "(actuator cmd logging suppressed after 5 messages)")

        # Push torques to Warp control buffer. SolverMuJoCo reads from
        # control.joint_f (it's wired into mjData->qfrc_applied via
        # apply_mjc_qfrc_kernel — see solver_mujoco.py:3191). joint_act is for
        # actuator-target control (POSITION/VELOCITY mode), not for direct
        # generalised forces.
        if self._control.joint_f is None:
            wp = self._wp
            self._control.joint_f = wp.zeros(
                self._model.joint_dof_count, dtype=wp.float32,
                device=self._device)
        self._control.joint_f.assign(torques)

    # ------------------------------------------------------------------
    #  Publishing
    # ------------------------------------------------------------------
    def _publish_state(self) -> None:
        # Pull joint q/qd from device once per publish.
        jq = self._state_in.joint_q.numpy()
        jqd = self._state_in.joint_qd.numpy()

        stamp = self.get_clock().now().to_msg()
        frame_id = self._body_label[0] if self._body_label else 'world'

        # Joint states — emit one entry per joint that contributes ≥1 DoF.
        # Fixed joints (0 DoF) are skipped, matching the C++ backend's
        # mjJNT_HINGE-only filter.
        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = frame_id
        names: list[str] = []
        positions: list[float] = []
        velocities: list[float] = []
        for j_idx in self._active_joint_indices:
            q_start = int(self._joint_q_start[j_idx])
            qd_start = int(self._joint_qd_start[j_idx])
            if q_start >= len(jq) or qd_start >= len(jqd):
                continue
            names.append(self._joint_names[j_idx])
            positions.append(float(jq[q_start]))
            velocities.append(float(jqd[qd_start]))
        js.name = names
        js.position = positions
        js.velocity = velocities
        js.effort = [0.0] * len(names)
        self._pub_joint.publish(js)

        # Odometry from body 0 if it has a free joint (qpos[0:7]).
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frame_id
        if jq.size >= 7 and self._has_floating_base():
            odom.pose.pose.position.x = float(jq[0])
            odom.pose.pose.position.y = float(jq[1])
            odom.pose.pose.position.z = float(jq[2])
            odom.pose.pose.orientation.w = float(jq[3])
            odom.pose.pose.orientation.x = float(jq[4])
            odom.pose.pose.orientation.y = float(jq[5])
            odom.pose.pose.orientation.z = float(jq[6])
            if jqd.size >= 6:
                odom.twist.twist.linear.x = float(jqd[0])
                odom.twist.twist.linear.y = float(jqd[1])
                odom.twist.twist.linear.z = float(jqd[2])
                odom.twist.twist.angular.x = float(jqd[3])
                odom.twist.twist.angular.y = float(jqd[4])
                odom.twist.twist.angular.z = float(jqd[5])
        self._pub_odom.publish(odom)

        # IMU is left zeroed unless the MJCF declares accelerometer/gyro
        # sensors and Newton's MuJoCo solver populates sensordata. The C++
        # path mirrors this — clients that don't need IMU just ignore it.
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = frame_id
        self._pub_imu.publish(imu)

        # Gravity torques: g(q) from the CPU MuJoCo model that SolverMuJoCo
        # builds internally. FIC adds these to its output to cancel gravity, so
        # without real values the arm falls under gravity even with FIC running.
        grav = Float64MultiArray()
        grav.data = self._compute_gravity_torques()
        self._pub_grav.publish(grav)

        # Force/torque: zeroed (subscribers gate on magnitude).
        self._pub_ft.publish(Wrench())

    def _compute_gravity_torques(self) -> list:
        """Per-DoF gravity torque g(q) at Newton's current joint positions.

        Read from ``qfrc_bias`` at zero velocity on a scratch ``MjData`` built
        from the CPU ``mjModel`` Newton's SolverMuJoCo keeps (see
        :func:`gravity_from_qpos`). It used to come from ``mj_inverse`` on the
        solver's own ``mj_data`` with qvel/qacc zeroed, the recipe that made the
        C++ simulator publish its own torque command as "gravity":
        ``mj_forward`` recomputes ``qacc`` from ``data.ctrl`` before
        ``mj_inverse`` runs, so inverse dynamics returned the command, and the
        FIC controller adds this vector to its output (positive feedback). It
        also zeroed the solver's velocities in place.

        Falls back to a zero vector on any error so subscribers (FIC, autotuner)
        never hang.
        """
        nv = int(self._model.joint_dof_count)
        try:
            mj_model = getattr(self._solver, 'mj_model', None)
            if mj_model is None:
                return [0.0] * nv
            if getattr(self, '_gdata', None) is None:
                import mujoco
                self._gdata = mujoco.MjData(mj_model)
            return gravity_from_qpos(mj_model, self._gdata,
                                     self._state_in.joint_q.numpy(), nv)
        except Exception as exc:
            if not getattr(self, '_grav_warned', False):
                self.get_logger().warn(
                    f"Gravity-torque computation failed; publishing zeros. "
                    f"FIC may not be able to hold position. Reason: {exc}")
                self._grav_warned = True
            return [0.0] * nv

    def _has_floating_base(self) -> bool:
        # 7 qpos coords for a free joint (xyz + quaternion).
        return self._model.joint_coord_count >= 7 and self._model.joint_dof_count >= 6

    # ------------------------------------------------------------------
    #  Lifecycle
    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        self._stop.set()
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node: NewtonSimulation | None = None
    spin_thread: threading.Thread | None = None
    try:
        node = NewtonSimulation()

        # rclpy.spin runs in a daemon thread so the main thread is free for
        # the viewer + sim_loop. pyglet/OpenGL contexts must live on the main
        # thread or the window stays black.
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True,
            name='mujoco_newton-rclpy-spin')
        spin_thread.start()

        node.run_sim_loop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None and spin_thread.is_alive():
            spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
