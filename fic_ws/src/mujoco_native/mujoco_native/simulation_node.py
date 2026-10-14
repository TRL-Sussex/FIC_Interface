"""The FIC Control Studio CPU simulator, built on the official `mujoco` wheel.

Since v4.5.0 this is the only CPU MuJoCo simulator in the repository (the C++
`mujoco` package, which linked a vendored x86-64 libmujoco, was retired; its
features were ported here). `mujoco_newton` remains a separate, optional GPU
backend with the same topic contract.

Published topics (under `topic_prefix`, default rates in brackets):
    joint_states    sensor_msgs/JointState      [1 kHz, `publish_rate_hz`]
    gravity_torques std_msgs/Float64MultiArray  [1 kHz, with joint_states]
    contact_torques std_msgs/Float64MultiArray  [1 kHz, with joint_states] the
                    joint torques of the constraint forces (contacts, joint
                    limits) plus qfrc_applied: what the environment applies to
                    each joint, as a joint-torque sensor would see it without
                    the controller's own command
    imu_data        sensor_msgs/Imu             [500 Hz] from the MJCF's
                    accelerometer / gyro / framequat sensors (zeros if none)
    force_torque    geometry_msgs/Wrench        [200 Hz] from `force_sensor` /
                    `torque_sensor`, or any sensor whose name ends in those
                    (multi-robot scenes prefix them, e.g. `r1_force_sensor`)
    odom            nav_msgs/Odometry           [50 Hz] (floating base only)
  plus the experiment hooks (wall_force, applied_wrench_truth, body_poses).

Subscribed topics:
    actuators_cmds  communication/msg/ActuatorCmds  (several publishers may
                    share it — the FIC controller and the gripper bridge, or
                    one controller per robot in a multi-robot scene; commands
                    are merged per actuator name)
    wall_cmd, apply_wrench                          (experiment hooks)

Service:
    sim_reset       communication/srv/SimulationReset

Control law (per commanded actuator): if the MJCF defines `<name>_pos` and
`<name>_vel` sensors a PD+feedforward law, otherwise the feedforward torque,
written to `data.ctrl` and clamped to [-100, 100]. A torque/motor actuator
whose command is older than 0.2 s is zeroed (a dead controller leaves the arm
limp, not pushing); an actuator with a position-type bias (e.g. the Robotiq
`gripper`) keeps its last setpoint, since the gripper bridge publishes only
when the width changes.

Physics steps at the MJCF `timestep` (override with `sim_dt`), paced to wall
clock and re-synchronised after a stall of more than 0.1 s, like MuJoCo's
`simulate`.

Rendering uses `mujoco.viewer.launch_passive` (Space pauses/resumes, Backspace
resets to the spawn state). On macOS the passive viewer must own the process
main thread AND the process must be launched via `mjpython` (the launch file
handles this). Pass `headless:=true` to disable the viewer (works under plain
`python`), e.g. over SSH or in CI.

Usage:
    ros2 launch mujoco_native simulation_launch.py robot_name:=Franka_FR3
"""

from __future__ import annotations

import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Wrench, WrenchStamped
from communication.msg import ActuatorCmds
from communication.srv import SimulationReset

import mujoco

from mujoco_native.experiment_hooks import (
    body_positions, set_wall, site_wrench, step_push,
    wall_contact_force)

# Links whose world positions are published on `body_poses` (flattened xpos).
_POSE_BODIES = ['Shoulder_Link', 'HalfArm1_Link', 'HalfArm2_Link', 'ForeArm_Link',
                'SphericalWrist1_Link', 'SphericalWrist2_Link', 'Bracelet_Link',
                'ee_link']


# Per-actuator command clamp (the retired C++ node's apply_ctrl used the same
# +/-100). Every shipped MJCF's ctrlrange is inside it.
_CTRL_CLAMP = 100.0
# Torque commands older than this are treated as stale and zeroed (the Newton
# backend does the same).
_CMD_STALE_S = 0.2
# Wall-clock drift beyond which the pacing loop re-synchronises instead of
# trying to catch up (MuJoCo simulate's syncMisalign).
_SYNC_MISALIGN_S = 0.1
# Extra physics steps one loop iteration may take to catch up with wall clock.
_MAX_CATCH_UP = 4
# Viewer refresh period (the physics loop runs much faster).
_RENDER_PERIOD_S = 1.0 / 60.0
# GLFW key codes delivered to the passive viewer's key_callback.
_KEY_SPACE = 32
_KEY_BACKSPACE = 259


def _cmd_qos() -> QoSProfile:
    """Actuator-command QoS: best effort, depth 20.

    Several publishers can share one actuators_cmds topic (each FIC controller
    of a multi-robot scene at 1 kHz, plus the gripper bridge), so a depth of 1
    would drop one publisher's command whenever two arrive between spins.
    """
    qos = QoSProfile(depth=20)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


def load_model(path: str) -> mujoco.MjModel:
    """Load an MJCF/URDF (`.xml`, `.urdf`) or a compiled binary model (`.mjb`)."""
    path = os.path.abspath(path)
    if path.lower().endswith('.mjb'):
        return mujoco.MjModel.from_binary_path(path)
    return mujoco.MjModel.from_xml_path(path)


def model_name(model: mujoco.MjModel) -> str:
    """The MJCF `<mujoco model=...>` name (the first entry of `names`)."""
    return bytes(model.names).split(b'\0', 1)[0].decode(errors='replace')


def find_ft_sensors(model: mujoco.MjModel):
    """(force, torque) sensor names, or None.

    Prefers the unprefixed `force_sensor`/`torque_sensor`; otherwise the first
    sensors whose names end in those (multi-robot scenes prefix every element,
    e.g. `r1_force_sensor`).
    """
    names = [model.sensor(i).name for i in range(model.nsensor)]
    force = 'force_sensor' if 'force_sensor' in names else next(
        (n for n in names if n.endswith('force_sensor')), None)
    torque = 'torque_sensor' if 'torque_sensor' in names else next(
        (n for n in names if n.endswith('torque_sensor')), None)
    return (force, torque) if force and torque else None


def find_imu_sensors(model: mujoco.MjModel) -> dict:
    """Sensor addresses for the IMU message: {'acc'|'gyro'|'quat': adr}.

    Like the retired C++ node, the last sensor of each type wins.
    """
    kinds = {mujoco.mjtSensor.mjSENS_ACCELEROMETER: 'acc',
             mujoco.mjtSensor.mjSENS_GYRO: 'gyro',
             mujoco.mjtSensor.mjSENS_FRAMEQUAT: 'quat'}
    out = {}
    for i in range(model.nsensor):
        kind = kinds.get(int(model.sensor_type[i]))
        if kind:
            out[kind] = int(model.sensor_adr[i])
    return out


def _sensor_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.history = HistoryPolicy.KEEP_LAST
    return qos


def _quat_to_euler(w: float, x: float, y: float, z: float):
    """(w,x,y,z) unit quaternion -> (roll, pitch, yaw) in radians (ZYX/XYZ).

    Used to surface a MuJoCo ball joint's orientation (e.g. a parallel
    manipulator's platform) as three scalar JointState entries, since a
    JointState carries one value per name and cannot hold a 4-vector.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = float(np.arctan2(sinr_cosp, cosr_cosp))
    pitch = float(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = float(np.arctan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


class NativeMujocoSimulation(Node):
    def __init__(self) -> None:
        super().__init__('simulation_mujoco_native')

        self.declare_parameter('simulation/model_file', '')
        self.declare_parameter('headless', False)
        # 0 = step at the MJCF's own <option timestep> (what the C++ node did).
        self.declare_parameter('sim_dt', 0.0)
        # joint_states + gravity_torques; capped by the physics rate.
        self.declare_parameter('publish_rate_hz', 1000.0)
        self.declare_parameter('imu_rate_hz', 500.0)
        self.declare_parameter('ft_rate_hz', 200.0)
        self.declare_parameter('odom_rate_hz', 50.0)
        self.declare_parameter('body_poses_rate_hz', 200.0)
        self.declare_parameter('topic_prefix', 'simulation')
        # Start pose of a fixed-base arm: comma-separated angles (rad) for its
        # first hinge joints, in model order; empty = MuJoCo's qpos0.
        self.declare_parameter('initial_joint_positions', '')

        model_file = self.get_parameter('simulation/model_file').value
        if not model_file or not os.path.exists(model_file):
            raise RuntimeError(f"MJCF model file not found: {model_file!r}")

        self._headless = bool(self.get_parameter('headless').value)

        def _period(param):
            hz = float(self.get_parameter(param).value)
            return 1.0 / hz if hz > 0 else float('inf')
        self._periods = {
            'state': _period('publish_rate_hz'),
            'imu': _period('imu_rate_hz'),
            'ft': _period('ft_rate_hz'),
            'odom': _period('odom_rate_hz'),
            'poses': _period('body_poses_rate_hz'),
        }

        self.get_logger().info(f"Loading model: {model_file}")
        self._model = load_model(model_file)
        self._data = mujoco.MjData(self._model)
        # Scratch data for g(q), so computing it never disturbs the stepped
        # state (sensordata, contacts, qacc_warmstart) — see
        # _compute_gravity_torques.
        self._gdata = mujoco.MjData(self._model)
        self._apply_home_keyframe()
        # Advance sim_dt of simulated time per mj_step; by default that is the
        # MJCF's own timestep (every shipped robot authors 1 or 2 ms).
        self._mjcf_dt = float(self._model.opt.timestep)
        requested_dt = float(self.get_parameter('sim_dt').value)
        self._dt = requested_dt if requested_dt > 0 else self._mjcf_dt
        self._model.opt.timestep = self._dt

        # Hinge joints only (matches the C++ backend's mjJNT_HINGE filter) —
        # used for joint_states. Precompute (index, name, qposadr, dofadr).
        self._hinge_joints = []
        for i in range(self._model.njnt):
            if self._model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
                name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, i)
                self._hinge_joints.append((
                    i,
                    name if name else f"joint{i}",
                    int(self._model.jnt_qposadr[i]),
                    int(self._model.jnt_dofadr[i]),
                ))

        self._js_hinge_names = [h[1] for h in self._hinge_joints]
        self._js_qadr = np.array([h[2] for h in self._hinge_joints], dtype=int)
        self._js_dadr = np.array([h[3] for h in self._hinge_joints], dtype=int)

        # Ball joints (e.g. a parallel manipulator's platform) — surfaced as
        # three euler pseudo-joints (<name>_roll/_pitch/_yaw) appended to
        # joint_states so their orientation is viewable. The C++/FIC consumers
        # name-match against their configured joint_names, so the extra entries
        # are ignored by controllers and only picked up by data views.
        self._ball_joints = []
        for i in range(self._model.njnt):
            if self._model.jnt_type[i] == mujoco.mjtJoint.mjJNT_BALL:
                name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, i)
                self._ball_joints.append((
                    i,
                    name if name else f"joint{i}",
                    int(self._model.jnt_qposadr[i]),
                    int(self._model.jnt_dofadr[i]),
                ))

        # frame_id — the model name (`<mujoco model=...>`), as the C++ node
        # published; sim_reset requests must carry the same name.
        self._model_name = model_name(self._model)
        self._frame_id = self._model_name or 'world'
        self._floating = any(
            self._model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE
            for i in range(self._model.njnt))
        # Actuators that keep their last setpoint when their command goes
        # stale: any with a bias term (position/velocity servos such as the
        # Robotiq gripper). Plain torque motors are zeroed instead.
        self._hold_setpoint = [
            int(self._model.actuator_biastype[i]) != int(mujoco.mjtBias.mjBIAS_NONE)
            for i in range(self._model.nu)]

        # Caches for actuator/sensor id lookups keyed by commanded name so we
        # don't call mj_name2id every control step.
        self._act_id_cache: dict[str, int] = {}
        self._sensor_id_cache: dict[str, int] = {}

        self._cmd_lock = threading.Lock()
        self._latest_cmd: ActuatorCmds | None = None
        self._last_cmd_stamp = 0.0
        # Per-actuator merged commands: name -> (torque, kp, pos, kd, vel, t).
        self._cmd_table: dict[str, tuple] = {}
        self._stop = threading.Event()
        self._viewer = None
        self._paused = False
        self._viewer_reset = False
        # sim_reset requests are executed by the sim loop (it owns the data).
        self._reset_lock = threading.Lock()
        self._reset_req = None

        # Experiment hooks (wall / pushes / EE wrench / link poses). Requests
        # arrive on the rclpy thread and are consumed by the sim loop.
        self._hook_lock = threading.Lock()
        self._wall_req = None
        self._push = None
        self._active_push_body = None
        body_names = [self._model.body(i).name for i in range(self._model.nbody)]
        self._has_wall = ('wall' in body_names and 'wall_geom' in
                          [self._model.geom(i).name for i in range(self._model.ngeom)])
        self._ft_sensors = find_ft_sensors(self._model)
        self._has_ft = self._ft_sensors is not None
        self._imu_adr = find_imu_sensors(self._model)
        self._pose_bodies = [b for b in _POSE_BODIES if b in body_names]
        self._ft_cache = None
        self._imu_cache = None

        self._build_pubsubs()

        self.get_logger().info(
            f"MuJoCo model loaded: nq={self._model.nq}, nv={self._model.nv}, "
            f"nu={self._model.nu}, {len(self._hinge_joints)} hinge joints, "
            f"{len(self._ball_joints)} ball joints "
            f"(MJCF timestep {self._mjcf_dt} -> stepping at {self._dt}; "
            f"F/T {'/'.join(self._ft_sensors) if self._has_ft else 'none'}; "
            f"IMU sensors {sorted(self._imu_adr) or 'none'}).")

    # ------------------------------------------------------------------
    #  Setup
    # ------------------------------------------------------------------
    def _apply_home_keyframe(self) -> None:
        """Start a floating-base model in its MJCF `home` keyframe (key 0).

        A fixed-base arm is bolted to the world, so MuJoCo's default `qpos0`
        (every joint at 0 rad) is a perfectly valid state for it to sit in
        until the controller connects. A *floating-base* model has no such
        support: at `qpos0` Spot hangs in mid-air with all twelve legs
        straight — a pose that is already outside its own knee range — and
        free-falls onto its joint stops in the seconds before the FIC node is
        up, penetrating the limit constraints by several degrees. The
        controller then inherits a robot lying on its stops.

        The reset is therefore gated on the model having a free joint rather
        than merely having a keyframe. Franka_FR3 and IIWA_14 both ship a
        `home` keyframe, and applying it would silently change their startup
        pose (e.g. Franka from all-zeros to `0 0 0 -1.57 0 1.57 -0.785`);
        KinovaGen3_7DOF, Pendulum and CSPM define no keyframe at all. Gating
        on the floating base makes this a strict no-op for every one of them,
        while giving every legged/floating model the designed spawn pose.

        `data.ctrl` is also loaded from the keyframe by
        ``mj_resetDataKeyframe``; ``_apply_latest_cmd`` zeroes it before the
        first ``mj_step`` (no command has arrived yet), so the keyframe's
        ``ctrl`` never reaches the actuators.
        """
        self._hold_pose = False
        self._first_cmd_seen = False
        self._spawn_qpos = self._data.qpos.copy()
        has_free_joint = any(
            self._model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE
            for i in range(self._model.njnt))
        if not has_free_joint:
            self._apply_initial_joint_positions()
            return
        if self._model.nkey < 1:
            self.get_logger().warn(
                "Floating-base model defines no keyframe; spawning at qpos0. "
                "It may free-fall before the controller connects.")
            return
        mujoco.mj_resetDataKeyframe(self._model, self._data, 0)
        mujoco.mj_forward(self._model, self._data)
        self._spawn_qpos = self._data.qpos.copy()
        self._hold_pose = True
        key_name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_KEY, 0)
        self.get_logger().info(
            f"Floating-base model: reset to keyframe 0 "
            f"({key_name or 'unnamed'}), base height "
            f"{float(self._data.qpos[2]):.3f} m; holding this pose until the "
            f"first actuator command arrives.")

    def _apply_initial_joint_positions(self) -> None:
        """Start a fixed-base arm at `initial_joint_positions` (e.g. its home).

        The angles go to the first hinge joints in model order (the order of
        joint_states), and the pose is held, as a floating base's keyframe, until
        the first actuator command, so the arm does not sag away from it before
        the controller connects. Empty (the default) keeps qpos0.
        """
        raw = str(self.get_parameter('initial_joint_positions').value or '').strip()
        if not raw:
            return
        try:
            values = [float(v) for v in raw.replace(' ', '').split(',') if v]
        except ValueError:
            self.get_logger().warn(
                f"initial_joint_positions {raw!r} is not a list of numbers; "
                f"spawning at qpos0.")
            return
        hinges = [i for i in range(self._model.njnt)
                  if self._model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
        if len(values) > len(hinges):
            self.get_logger().warn(
                f"initial_joint_positions has {len(values)} values for "
                f"{len(hinges)} hinge joints; spawning at qpos0.")
            return
        for jid, q in zip(hinges, values):
            self._data.qpos[self._model.jnt_qposadr[jid]] = q
        mujoco.mj_forward(self._model, self._data)
        self._spawn_qpos = self._data.qpos.copy()
        self._hold_pose = True
        self.get_logger().info(
            f"Spawned at initial_joint_positions ({len(values)} joints); "
            f"holding this pose until the first actuator command arrives.")

    def _build_pubsubs(self) -> None:
        qos = _sensor_qos()
        prefix = self.get_parameter('topic_prefix').value + '/'
        self._pub_imu = self.create_publisher(Imu, prefix + 'imu_data', qos)
        self._pub_joint = self.create_publisher(JointState, prefix + 'joint_states', qos)
        self._pub_odom = self.create_publisher(Odometry, prefix + 'odom', qos)
        self._pub_grav = self.create_publisher(
            Float64MultiArray, prefix + 'gravity_torques', qos)
        self._pub_contact = self.create_publisher(
            Float64MultiArray, prefix + 'contact_torques', qos)
        self._pub_ft = self.create_publisher(Wrench, prefix + 'force_torque', qos)

        self._sub_cmd = self.create_subscription(
            ActuatorCmds, prefix + 'actuators_cmds', self._on_cmd, _cmd_qos())
        self._srv_reset = self.create_service(
            SimulationReset, prefix + 'sim_reset', self._on_reset)

        # Experiment hooks.
        self._pub_wall_force = self.create_publisher(
            WrenchStamped, prefix + 'wall_force', qos)
        self._pub_push_truth = self.create_publisher(
            Float64MultiArray, prefix + 'applied_wrench_truth', qos)
        self._pub_body_poses = self.create_publisher(
            Float64MultiArray, prefix + 'body_poses', qos)
        self._sub_wall = self.create_subscription(
            Float64MultiArray, prefix + 'wall_cmd', self._on_wall_cmd, qos)
        self._sub_push = self.create_subscription(
            String, prefix + 'apply_wrench', self._on_apply_wrench, qos)

    def _on_wall_cmd(self, msg) -> None:
        if len(msg.data) != 8:
            self.get_logger().warn(f"wall_cmd needs 8 values, got {len(msg.data)}")
            return
        with self._hook_lock:
            self._wall_req = tuple(float(v) for v in msg.data)

    def _on_apply_wrench(self, msg) -> None:
        try:
            r = json.loads(msg.data)
            name = str(r['body'])
            self._model.body(name)  # raises if unknown
            push = dict(body=name, force=[float(v) for v in r['force']],
                        point=[float(v) for v in r['point']],
                        ramp_s=float(r['ramp_s']), hold_s=float(r['hold_s']),
                        t0=None)  # stamped by the sim loop on first sight
            if len(push['force']) != 3 or len(push['point']) != 3:
                raise ValueError("force and point must have 3 entries")
        except Exception as exc:
            self.get_logger().warn(f"apply_wrench rejected: {exc}")
            return
        with self._hook_lock:
            self._push = push
        self.get_logger().info(f"push on {push['body']} queued")

    def _build_viewer(self) -> None:
        """Open the passive viewer. Must be called on the main thread.

        Returns silently in headless mode; falls back to headless on any error
        (e.g. no display) so the simulation still runs and publishes.
        """
        if self._headless:
            self._viewer = None
            return
        try:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(
                self._model, self._data, key_callback=self._on_key)
            self.get_logger().info(
                "MuJoCo passive viewer initialised "
                "(Space: pause/resume, Backspace: reset).")
        except Exception as exc:  # pragma: no cover — display/interpreter dependent
            self.get_logger().warn(
                f"Falling back to headless mode (viewer unavailable): {exc}. "
                f"On macOS the viewer requires launching via 'mjpython'.")
            self._viewer = None

    def _on_key(self, keycode: int) -> None:
        """Passive-viewer keys (viewer thread): Space pauses, Backspace resets."""
        if keycode == _KEY_SPACE:
            self._paused = not self._paused
            self.get_logger().info(
                "Simulation paused." if self._paused else "Simulation resumed.")
        elif keycode == _KEY_BACKSPACE:
            self._viewer_reset = True

    # ------------------------------------------------------------------
    #  Simulation loop  (runs on the main thread; the viewer's GL/Cocoa
    #  context must live there — see main()).
    # ------------------------------------------------------------------
    def run_sim_loop(self) -> None:
        self._build_viewer()
        m, d = self._model, self._data
        # Wall clock <-> sim time anchor for real-time pacing.
        sync_wall = time.monotonic()
        sync_sim = d.time
        next_due = dict.fromkeys(self._periods, sync_wall)
        next_render = sync_wall

        while not self._stop.is_set() and rclpy.ok():
            # Bail out if the user closed the viewer window.
            if self._viewer is not None and not self._viewer.is_running():
                self.get_logger().info("Viewer window closed; shutting down.")
                self._stop.set()
                break

            self._service_resets()

            if self._paused:
                # Keep the viewer's perturbations/edits visible, like simulate.
                mujoco.mj_forward(m, d)
            else:
                # One step, plus catch-up steps (at most _MAX_CATCH_UP) when the
                # loop fell behind wall clock, so a slow iteration (viewer
                # sync, a GC pause) costs publish rate rather than sim speed.
                lag = (time.monotonic() - sync_wall) - (d.time - sync_sim)
                n_steps = 1 + min(max(int(lag / self._dt), 0), _MAX_CATCH_UP)
                ok = True
                for _ in range(n_steps):
                    self._apply_latest_cmd()
                    self._apply_hooks_pre_step()
                    mujoco.mj_step(m, d)
                    self._check_diverged()
                    # Read contacts/sensors right after the step, before
                    # anything else touches the data.
                    try:
                        self._read_hooks_post_step()
                    except Exception as exc:
                        if rclpy.ok():
                            self.get_logger().warn(f"read_hooks failed: {exc}")
                        ok = False
                        break
                    self._hold_spawn_pose_until_first_cmd()
                    if self._paused:
                        break
                if not ok:
                    self._stop.set()
                    break

            now = time.monotonic()
            try:
                due = [k for k, t in next_due.items() if now >= t]
                if due:
                    self._publish(due)
                    for k in due:
                        # Advance on the stream's own grid; skip missed slots.
                        nxt = next_due[k] + self._periods[k]
                        next_due[k] = nxt if nxt > now else now + self._periods[k]
            except Exception as exc:
                # Publisher context goes invalid during shutdown; stop the
                # loop instead of spamming tracebacks.
                if rclpy.ok():
                    self.get_logger().warn(f"publish failed: {exc}")
                self._stop.set()
                break

            # Render at ~60 Hz, not every physics step: a sync per 1 ms step
            # costs more than the step itself and drags the loop below real
            # time (simulate likewise renders on its own refresh clock).
            if self._viewer is not None and now >= next_render:
                next_render = now + _RENDER_PERIOD_S
                try:
                    self._viewer.sync()
                except Exception as exc:  # pragma: no cover
                    self.get_logger().warn(f"Viewer error: {exc}; disabling viewer.")
                    self._viewer = None

            # Real-time pacing against an absolute anchor, so per-step sleep
            # overshoot does not accumulate into a slow simulation. After a
            # stall (or a pause, or a reset that rewinds d.time) re-anchor
            # instead of fast-forwarding to catch up.
            now = time.monotonic()
            ahead = (d.time - sync_sim) - (now - sync_wall)
            if self._paused or abs(ahead) > _SYNC_MISALIGN_S:
                sync_wall, sync_sim = now, d.time
                if self._paused:
                    time.sleep(self._dt)
            elif ahead > 0:
                time.sleep(ahead)

    def _check_diverged(self) -> None:
        """Pause on divergence when the MJCF disables auto-reset (as simulate).

        With auto-reset enabled (the default) MuJoCo resets a diverged state
        itself and warns; with it disabled, stepping on would only publish
        NaNs, so stop stepping and say why.
        """
        m, d = self._model, self._data
        if not (m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_AUTORESET):
            return
        for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
                  mujoco.mjtWarning.mjWARN_BADQPOS):
            if d.warning[int(w)].number > 0:
                self._paused = True
                self.get_logger().error(
                    f"Simulation diverged ({w.name}); paused. Reset with the "
                    f"sim_reset service or Backspace in the viewer.")
                return

    def _apply_hooks_pre_step(self) -> None:
        m, d = self._model, self._data
        with self._hook_lock:
            wall_req, self._wall_req = self._wall_req, None
            push = self._push
        queued = push
        fresh = push is not None and push.get('t0') is None
        if wall_req is not None and self._has_wall:
            en, x, y, z, qw, qx, qy, qz = wall_req
            set_wall(m, d, 'wall', 'wall_geom', bool(en), (x, y, z), (qw, qx, qy, qz))
            self.get_logger().info(
                f"wall {'ENABLED' if en else 'RELEASED'} at t_sim={d.time:.4f}")
        push, self._active_push_body, truth = step_push(
            m, d, push, self._active_push_body)
        if fresh and push is not None:
            self.get_logger().info(
                f"push started on {push['body']} at t_sim={push['t0']:.4f}")
        if push is None:
            with self._hook_lock:
                # drop the finished request, unless a newer one arrived meanwhile
                if self._push is not None and self._push is queued:
                    self._push = None
        if truth is not None:
            self._pub_push_truth.publish(Float64MultiArray(data=truth))

    def _read_hooks_post_step(self) -> None:
        m, d = self._model, self._data
        if self._has_wall:
            # Force ON the wall is negative z for a probe pressing down;
            # publish the load-cell convention (compression positive).
            f = -wall_contact_force(m, d, 'wall_geom')
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = map(float, f)
            self._pub_wall_force.publish(msg)
        if self._has_ft:
            self._ft_cache = site_wrench(m, d, *self._ft_sensors)
        if self._imu_adr:
            self._imu_cache = {k: d.sensordata[a:a + (4 if k == 'quat' else 3)].copy()
                               for k, a in self._imu_adr.items()}

    def _hold_spawn_pose_until_first_cmd(self) -> None:
        """Keep a floating-base model in its spawn pose until a controller talks.

        The keyframe reset gives Spot the right pose at t=0, but nothing holds
        it there: with `data.ctrl` at zero (no command has arrived) a 50 kg
        quadruped folds onto its joint stops within 0.5 s, and the controller
        node needs several seconds to load its model and configuration before
        it can send anything. Whatever the controller is then handed, it is not
        the robot's designed pose — it is a heap on the floor, and the first
        torque it computes is a full-scale snap out of it.

        So while no `actuators_cmds` message has ever been received, the spawn
        state is simply re-applied after each step. The simulation is still
        running and still publishing (the controller sees the pose it will
        actually be asked to hold, with zero velocity); it just is not allowed
        to fall over before anyone is driving it.

        Scope is deliberately narrow:
          * floating base only, so every fixed-base arm is untouched — a
            Franka at `qpos0` is supported by its own base and its small
            uncontrolled gravity sag is part of the behaviour existing runs
            were measured against;
          * first command only. Once a controller has spoken, the hold is
            released permanently and never re-arms, so a controller that later
            dies or goes stale still leaves the robot free to fall, exactly as
            before (`_apply_latest_cmd` zeroes `ctrl` on staleness).
        """
        if self._first_cmd_seen or not self._hold_pose:
            return
        with self._cmd_lock:
            if self._latest_cmd is not None:
                self._first_cmd_seen = True
        if self._first_cmd_seen:
            self.get_logger().info(
                "First actuator command received; releasing spawn-pose hold.")
            return
        self._data.qpos[:] = self._spawn_qpos
        self._data.qvel[:] = 0.0
        self._data.qacc[:] = 0.0
        mujoco.mj_forward(self._model, self._data)

    # ------------------------------------------------------------------
    #  Command handling
    # ------------------------------------------------------------------
    def _on_cmd(self, msg: ActuatorCmds) -> None:
        """Merge a command into the per-actuator table.

        Publishers sharing the topic (the FIC controller, the gripper bridge,
        one controller per robot of a multi-robot scene) each name only their
        own actuators, so every message updates just the entries it names.
        """
        now = time.monotonic()

        def at(seq, k):
            return float(seq[k]) if k < len(seq) else 0.0
        with self._cmd_lock:
            self._latest_cmd = msg
            self._last_cmd_stamp = now
            for k, name in enumerate(msg.actuators_name):
                self._cmd_table[name] = (at(msg.torque, k), at(msg.kp, k), at(msg.pos, k),
                                         at(msg.kd, k), at(msg.vel, k), now)

    def _actuator_id(self, name: str) -> int:
        aid = self._act_id_cache.get(name)
        if aid is None:
            aid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            self._act_id_cache[name] = aid
        return aid

    def _sensor_id(self, name: str) -> int:
        sid = self._sensor_id_cache.get(name)
        if sid is None:
            sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            self._sensor_id_cache[name] = sid
        return sid

    def _apply_latest_cmd(self) -> None:
        with self._cmd_lock:
            table = list(self._cmd_table.items())

        # No command yet -> hold zero control so nothing moves on its own
        # (this also discards any ctrl a keyframe reset loaded).
        if not table:
            self._data.ctrl[:] = 0.0
            return

        now = time.monotonic()
        names = [name for name, _ in table]
        n_applied = 0
        for name, (torque, kp, pos, kd, vel, stamp) in table:
            aid = self._actuator_id(name)
            if aid == -1:
                continue
            if now - stamp > _CMD_STALE_S and not self._hold_setpoint[aid]:
                # A torque the controller stopped refreshing is dropped, so a
                # dead controller leaves the arm limp rather than pushing.
                self._data.ctrl[aid] = 0.0
                continue

            # PD+feedforward only if the MJCF exposes per-actuator position
            # AND velocity sensors, else pure feedforward torque. For the
            # Kinova/Franka <motor> actuators these sensors are absent.
            pos_sid = self._sensor_id(name + "_pos")
            vel_sid = self._sensor_id(name + "_vel")
            if pos_sid == -1 or vel_sid == -1:
                ctrl = torque
            else:
                pos_meas = float(self._data.sensordata[self._model.sensor_adr[pos_sid]])
                vel_meas = float(self._data.sensordata[self._model.sensor_adr[vel_sid]])
                ctrl = kp * (pos - pos_meas) + kd * (vel - vel_meas) + torque

            self._data.ctrl[aid] = min(max(-_CTRL_CLAMP, ctrl), _CTRL_CLAMP)
            n_applied += 1

        # Diagnostic: log the first few commands so the user can confirm the FIC
        # controller is wired up correctly, then suppress to avoid 1 kHz spam.
        if not getattr(self, '_cmd_log_done', False):
            self._cmd_log_count = getattr(self, '_cmd_log_count', 0) + 1
            if self._cmd_log_count <= 5:
                self.get_logger().info(
                    f"actuator cmd #{self._cmd_log_count}: "
                    f"applied {n_applied}/{len(names)} (names={names[:3]}...)")
            if self._cmd_log_count == 5:
                self._cmd_log_done = True
                self.get_logger().info(
                    "(actuator cmd logging suppressed after 5 messages)")

    # ------------------------------------------------------------------
    #  Publishing
    # ------------------------------------------------------------------
    def _publish(self, streams) -> None:
        """Publish the streams that are due (keys of self._periods)."""
        stamp = self.get_clock().now().to_msg()
        d = self._data
        if 'state' in streams:
            self._publish_joint_states(stamp)
            # Gravity torques — FIC adds these to cancel gravity; without them
            # the arm sags even with FIC running. Sliced to hinge DOF so the
            # vector always lines up with joint_states.
            grav = Float64MultiArray()
            grav.data = self._hinge_gravity_torques(
                self._compute_gravity_torques(), self._hinge_joints)
            self._pub_grav.publish(grav)
            # Joint torques of the environment on the arm (contact and limit
            # constraints, qfrc_applied): the controller estimates the external
            # wrench from these in simulation, where the actuator effort of
            # joint_states is its own command.
            ext = Float64MultiArray()
            ext.data = self._hinge_gravity_torques(
                (d.qfrc_constraint + d.qfrc_applied).tolist(), self._hinge_joints)
            self._pub_contact.publish(ext)

        if 'odom' in streams:
            self._publish_odom(stamp)

        if 'imu' in streams:
            # From the MJCF's accelerometer / gyro / framequat sensors (read
            # right after the step); fields without a sensor stay at the
            # message defaults.
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self._frame_id
            c = self._imu_cache
            if c:
                if 'acc' in c:
                    a = c['acc']
                    imu.linear_acceleration.x, imu.linear_acceleration.y, \
                        imu.linear_acceleration.z = float(a[0]), float(a[1]), float(a[2])
                if 'gyro' in c:
                    g = c['gyro']
                    imu.angular_velocity.x, imu.angular_velocity.y, \
                        imu.angular_velocity.z = float(g[0]), float(g[1]), float(g[2])
                if 'quat' in c:
                    q = c['quat']
                    imu.orientation.w, imu.orientation.x, imu.orientation.y, \
                        imu.orientation.z = (float(q[0]), float(q[1]),
                                             float(q[2]), float(q[3]))
            self._pub_imu.publish(imu)

        if 'ft' in streams:
            # Zeros unless the MJCF has the F/T sensors; subscribers gate on
            # magnitude so zeros are safe.
            ft = Wrench()
            if self._has_ft and self._ft_cache is not None:
                w = self._ft_cache
                ft.force.x, ft.force.y, ft.force.z = map(float, w[:3])
                ft.torque.x, ft.torque.y, ft.torque.z = map(float, w[3:])
            self._pub_ft.publish(ft)

        if 'poses' in streams and self._pose_bodies:
            self._pub_body_poses.publish(Float64MultiArray(
                data=body_positions(self._model,
                                    d, self._pose_bodies).ravel().tolist()))

    def _publish_joint_states(self, stamp) -> None:
        """One entry per hinge joint, then roll/pitch/yaw per ball joint."""
        d = self._data
        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = self._frame_id
        names = list(self._js_hinge_names)
        positions = d.qpos[self._js_qadr].tolist()
        velocities = d.qvel[self._js_dadr].tolist()
        efforts = d.qfrc_actuator[self._js_dadr].tolist()
        # Ball joints -> roll/pitch/yaw pseudo-joints (view-only orientation).
        for _, name, qposadr, dofadr in self._ball_joints:
            w = float(d.qpos[qposadr]); x = float(d.qpos[qposadr + 1])
            y = float(d.qpos[qposadr + 2]); z = float(d.qpos[qposadr + 3])
            euler = _quat_to_euler(w, x, y, z)
            for k, suffix in enumerate(('roll', 'pitch', 'yaw')):
                names.append(f"{name}_{suffix}")
                positions.append(euler[k])
                velocities.append(float(d.qvel[dofadr + k]))
                efforts.append(0.0)
        js.name = names
        js.position = positions
        js.velocity = velocities
        js.effort = efforts
        self._pub_joint.publish(js)

    def _publish_odom(self, stamp) -> None:
        """Base pose/twist — only meaningful for a floating base (qpos[0:7]).

        A fixed-base model publishes a zero odometry message (the C++ node
        published the first seven joint angles as a "pose" instead).
        """
        d = self._data
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_id
        if self._floating:
            odom.pose.pose.position.x = float(d.qpos[0])
            odom.pose.pose.position.y = float(d.qpos[1])
            odom.pose.pose.position.z = float(d.qpos[2])
            odom.pose.pose.orientation.w = float(d.qpos[3])
            odom.pose.pose.orientation.x = float(d.qpos[4])
            odom.pose.pose.orientation.y = float(d.qpos[5])
            odom.pose.pose.orientation.z = float(d.qpos[6])
            odom.twist.twist.linear.x = float(d.qvel[0])
            odom.twist.twist.linear.y = float(d.qvel[1])
            odom.twist.twist.linear.z = float(d.qvel[2])
            odom.twist.twist.angular.x = float(d.qvel[3])
            odom.twist.twist.angular.y = float(d.qvel[4])
            odom.twist.twist.angular.z = float(d.qvel[5])
        self._pub_odom.publish(odom)

    def _compute_gravity_torques(self) -> list:
        """Per-DoF gravity torque g(q). Returns zeros on any error so
        subscribers (FIC, autotuner) never hang.

        This is read from ``qfrc_bias`` evaluated at zero velocity, which is
        exactly g(q): ``qfrc_bias`` is the Coriolis/centrifugal/gravity bias,
        and at qvel=0 only gravity survives. It depends on `qpos` alone.

        It used to be read from ``mj_inverse``'s ``qfrc_inverse`` with qvel and
        qacc zeroed first, which looks equivalent but is not: ``mj_forward``
        *recomputes* ``qacc`` from the current ``data.ctrl`` before
        ``mj_inverse`` ever runs, so the zeroing of qacc was undone and inverse
        dynamics dutifully returned the force that had produced that
        acceleration — the actuator command itself. Measured on Spot at the
        home keyframe, the published "gravity" matched `data.ctrl` to 3e-14 Nm.
        Since the FIC controller *adds* this vector to its own output in the
        joint-impedance modes, that closed a unity-gain positive feedback loop
        through the controller: the commanded torque re-entered as its own
        feedforward and saturated every actuator within ~0.1 s from a standing
        start with zero tracking error. On IIWA_14 the same bug produced
        ±6000 Nm "gravity"; on Pendulum a verbatim echo of the command.

        `qfrc_bias` also makes the passive/frictionloss disable mask
        unnecessary — spring, damper and friction forces live in
        ``qfrc_passive``, which is not part of the bias term.

        The forward pass runs on a separate scratch ``MjData`` holding the
        current ``qpos`` with zero velocity, so it cannot disturb the stepped
        state (sensor readings, contacts, ``qfrc_actuator`` in joint_states,
        the solver warm start) the way zeroing ``qvel`` in place did.
        """
        nv = int(self._model.nv)
        try:
            m, g = self._model, self._gdata
            g.qpos[:] = self._data.qpos
            g.qvel[:] = 0.0
            g.act[:] = self._data.act
            g.mocap_pos[:] = self._data.mocap_pos
            g.mocap_quat[:] = self._data.mocap_quat
            mujoco.mj_forward(m, g)
            return g.qfrc_bias[:nv].tolist()
        except Exception as exc:
            if not getattr(self, '_grav_warned', False):
                self.get_logger().warn(
                    f"Gravity-torque computation failed; publishing zeros. "
                    f"FIC may not be able to hold position. Reason: {exc}")
                self._grav_warned = True
            return [0.0] * nv

    # ------------------------------------------------------------------
    #  sim_reset service
    # ------------------------------------------------------------------
    def _on_reset(self, request, response):
        """Reset the state to a requested base pose and named joint positions.

        Runs on the rclpy thread; the sim loop owns the data, so the request is
        handed over and this waits (up to 2 s) for the loop to apply it.
        `header.frame_id` must equal the model name, as on the C++ node.
        """
        if request.header.frame_id != self._model_name:
            self.get_logger().error(
                f"Reset request is not for {self._model_name!r} "
                f"(got {request.header.frame_id!r})")
            response.is_success = False
            return response
        done = threading.Event()
        with self._reset_lock:
            self._reset_req = (request, done)
        response.is_success = done.wait(timeout=2.0)
        return response

    def _service_resets(self) -> None:
        """Apply a pending sim_reset request or viewer Backspace (sim loop)."""
        with self._reset_lock:
            req, self._reset_req = self._reset_req, None
        if req is None and not self._viewer_reset:
            return
        m, d = self._model, self._data
        if req is None:
            self._viewer_reset = False
            d.qpos[:] = self._spawn_qpos
            d.qvel[:] = 0.0
            d.time = 0.0
            mujoco.mj_forward(m, d)
            self._clear_commands()
            self.get_logger().info("Viewer reset to the spawn state.")
            return
        request, done = req
        mujoco.mj_resetData(m, d)
        if self._floating:
            # Base pose goes to the (first) free joint. The C++ node wrote it
            # to qpos[0:7] unconditionally, i.e. over a fixed-base arm's joint
            # angles; a fixed base ignores it here.
            jid = next(i for i in range(m.njnt)
                       if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE)
            a = int(m.jnt_qposadr[jid])
            p, o = request.base_pose.position, request.base_pose.orientation
            d.qpos[a:a + 7] = [p.x, p.y, p.z, o.w, o.x, o.y, o.z]
        js = request.joint_state
        for name, pos in zip(js.name, js.position):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                d.qpos[m.jnt_qposadr[jid]] = pos
            else:
                self.get_logger().warn(f"Request joint {name} does not exist")
        mujoco.mj_forward(m, d)
        self._spawn_qpos = d.qpos.copy()
        self._clear_commands()
        self._paused = False
        self.get_logger().info(
            f"Reset robot state; total mass {float(m.body_subtreemass[0]):.3f} kg")
        done.set()

    def _clear_commands(self) -> None:
        with self._cmd_lock:
            self._cmd_table.clear()
        self._data.ctrl[:] = 0.0

    @staticmethod
    def _hinge_gravity_torques(full_tau: list, hinge_joints: list) -> list:
        """Reduce an nv-long gravity vector to one entry per hinge joint.

        g(q) has a torque for every DOF, which for a floating base
        includes the six unactuated trunk DOF. joint_states only carries the
        hinges, and the FIC controller copies the first `num_joints` entries of
        whatever it receives — so publishing the raw vector would hand it trunk
        forces as leg torques. Indexing by each hinge's own dofadr is robust to
        the free joint appearing anywhere in the model.
        """
        return [full_tau[dofadr] if dofadr < len(full_tau) else 0.0
                for _, _, _, dofadr in hinge_joints]

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


def _spin(node) -> None:
    """rclpy.spin, quiet on Ctrl-C/SIGTERM (the context shuts down under it)."""
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node: NativeMujocoSimulation | None = None
    spin_thread: threading.Thread | None = None
    try:
        node = NativeMujocoSimulation()

        # rclpy.spin runs in a daemon thread so the main thread is free for the
        # viewer + sim loop. The MuJoCo passive viewer's GL/Cocoa context must
        # live on the main thread (and, on macOS, under mjpython).
        spin_thread = threading.Thread(
            target=_spin, args=(node,), daemon=True,
            name='mujoco_native-rclpy-spin')
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
