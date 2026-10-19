# mixins/joint_state_mixin.py — Always-loaded shared joint-state subscribers
#
# Owns per-robot `current_joint_positions_by_robot` / `current_ee_pose_by_robot`
# dicts, plus legacy `current_joint_positions` / `current_ee_pose` attributes
# that mirror the first active robot for back-compat.
#
# Subscribes per active robot to: q_current, ee_current_pose, trajectory_data.
# Plus two global fallbacks (joint_states, simulation_joint_states) that route
# into the first active robot's slot since those topics carry no rid prefix.

import math
import time

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Wrench
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data


def latest_qos(base):
    """``base`` (a depth, i.e. reliable, or a QoS profile) keeping only the newest message.

    The Studio shows the newest robot state; its state thread reads it at a
    steady pace (ros_node.STATE_PERIOD_S), so older samples are dropped in DDS
    instead of each costing a Python callback. Reliability is unchanged: a
    reliable reader never matches a best-effort publisher.
    """
    import copy
    from rclpy.qos import QoSProfile
    if isinstance(base, int):
        return QoSProfile(depth=1)
    qos = copy.copy(base)
    qos.depth = 1
    return qos


def parse_trajectory_joint(data, n, mode):
    """Return (commanded, effort) lists for n joints from trajectory_data.

    ``effort`` is the FIC controller's COMMANDED joint control effort
    (block 4), NOT a measured torque sensor reading — it is what the
    controller commanded, not what the arm felt. For the measured torque
    use ``joint_torque_for()``, which reads ``{rid}_joint_torques``.

    Joint modes (1-4) use stride i + k*n; Cartesian modes (5-8) use i*8.
    """
    cmd, eff = [], []
    if not data or n <= 0:
        return cmd, eff
    cartesian = mode >= 5
    for i in range(n):
        if cartesian:
            base = i * 8
            cmd.append(data[base + 1] if base + 1 < len(data) else 0.0)
            eff.append(0.0)
        else:
            ci, ei = i + 3 * n, i + 4 * n
            cmd.append(data[ci] if ci < len(data) else 0.0)
            eff.append(data[ei] if ei < len(data) else 0.0)
    return cmd, eff


def parse_trajectory_positions(data, n, mode):
    """Return the n current joint positions (rad) in trajectory_data, or None.

    The layout follows the controller (fic_library fic_control.h): joint modes
    (1-4) write blocks of n, current position in block 1 (``i + n``; block 0 is
    the planner target); Cartesian modes (5+) write 8 values per joint, current
    position first (``i * 8``).
    """
    if not data or n <= 0:
        return None
    if mode >= 5:
        if len(data) < n * 8:
            return None
        return [data[i * 8] for i in range(n)]
    if len(data) < 2 * n:
        return None
    return list(data[n:2 * n])


def parse_cartesian_error(data, n):
    """Return (ex, ey, ez, e_angle) from the trajectory_data Cartesian block.

    The Cartesian block starts at index ``n * 8``, after the 8 values per joint
    the task-space modes write (fic_control.h); within it the position error
    x/y/z sit at offsets 6,7,8 (metres) and the orientation-angle error at
    offset 15 (radians). Returns ``None`` when the block is absent (the joint
    modes write 8 blocks of n and nothing after them).
    """
    base = n * 8
    if n <= 0 or len(data) < base + 16:
        return None
    return (data[base + 6], data[base + 7], data[base + 8], data[base + 15])


class JointStateMixin:
    """Built-in mixin that owns the shared joint/pose state. Extensions read
    via `host.current_joint_positions` (legacy/first-robot) or the per-robot
    `host.joint_positions_for(rid)` / `host.ee_pose_for(rid)`."""

    def _init_joint_state(self):
        # Legacy single-robot mirrors (= first active robot's state).
        self.current_joint_positions = []
        self.current_ee_pose = None
        # Per-robot state.
        self.current_joint_positions_by_robot: dict[str, list] = {}
        self.current_ee_pose_by_robot: dict[str, Pose] = {}
        # Dashboard telemetry: commanded positions, velocities, efforts, wrench.
        self.current_joint_commanded_by_robot: dict[str, list] = {}
        self.current_joint_velocities_by_robot: dict[str, list] = {}
        self.current_joint_efforts_by_robot: dict[str, list] = {}
        self.current_wrench_by_robot: dict[str, tuple] = {}
        self.current_cartesian_error_by_robot: dict[str, tuple] = {}
        # Contact-localization telemetry (fic_contact node): measured
        # torque, the estimated contacted link, and the controller's own
        # accumulated admittance offset.
        self.current_joint_torque_by_robot: dict[str, list] = {}
        self.current_contact_estimate_by_robot: dict[str, dict] = {}
        self.current_admittance_offset_by_robot: dict[str, list] = {}
        # Measured gripper state from fic_control's {rid}_gripper_state:
        # rid -> (width_m, motor_current, monotonic receive time).
        self.current_gripper_state_by_robot: dict[str, tuple] = {}
        # Raw sim joint_states keyed BY NAME (for robots whose control page
        # shows more joints than the controller drives, e.g. the CSPM parallel
        # robot: 3 actuated + 6 passive hinges + platform orientation). Unlike
        # q_current (controller output, num_joints entries) this preserves every
        # joint the sim publishes so passive/view-only joints show live data.
        self.current_sim_joint_by_name_by_robot: dict[str, dict] = {}
        self.current_joint_update_count_by_robot: dict = {}
        self.current_joint_update_time_by_robot: dict = {}
        # Track which robots have received a q_current message so that
        # trajectory_data doesn't overwrite with potentially stale values.
        self._q_current_received: set = set()
        # Simulated robots publish no q_current: their joints come from the
        # simulator's joint_states by name (rid -> monotonic time of the last),
        # not from trajectory_data, whose layout depends on the mode the
        # controller is in (H-PMP writes the joint layout in a Cartesian mode).
        self._sim_q_time: dict = {}
        self._sim_order_cache: dict = {}
        # Subscription bookkeeping: subs_by_robot[rid] = {key: subscription}
        self._js_subs_by_robot: dict[str, dict] = {}
        self._setup_joint_state_subscribers()

    def _setup_joint_state_subscribers(self):
        # Per-active-robot subscriptions.
        for n in self.node.active_robots:
            self._add_robot_subscriptions(n)

        # Global fallbacks (no rid prefix on these topics — they update the
        # first active robot's slot).
        sub_node = self._js_sub_node()
        if not hasattr(self.node, 'js_joint_states_sub'):
            self.node.js_joint_states_sub = sub_node.create_subscription(
                JointState, 'joint_states',
                self._on_js_joint_states_global, latest_qos(10))

        if not hasattr(self.node, 'js_sim_joint_sub'):
            self.node.js_sim_joint_sub = sub_node.create_subscription(
                JointState, 'simulation_joint_states',
                self._on_js_joint_states_global, latest_qos(10))

    def _js_sub_node(self):
        """The node robot-state subscriptions live on: the state node if running.

        ``FicStudioNode.start_state_node`` gives them their own node and
        thread (see there); without it (unit tests, older entry points) they
        stay on the GUI node. Their callbacks only store data, so either works.
        """
        return getattr(self.node, 'state_node', None) or self.node

    def _add_robot_subscriptions(self, n: int):
        """Create q_current / ee_pose / trajectory subscriptions for robot N."""
        rid = f'robot{n}'
        if rid in self._js_subs_by_robot:
            return
        # Deferred import: `communication` is a workspace-local package,
        # only on sys.path once `install/setup.bash` is sourced (unlike
        # rclpy/std_msgs/etc., which ship inside the conda env itself).
        # Importing it here rather than at module scope means merely
        # importing this mixin — as every fic_studio unit test does —
        # never requires a sourced workspace.
        from communication.msg import ContactEstimate
        sub_node = self._js_sub_node()
        subs = {}
        subs['q_current'] = sub_node.create_subscription(
            Float64MultiArray, f'{rid}_q_current',
            lambda msg, r=rid: self._on_q_current(r, msg), latest_qos(10))
        subs['ee_pose'] = sub_node.create_subscription(
            Pose, f'{rid}_ee_current_pose',
            lambda msg, r=rid: self._on_ee_pose(r, msg), latest_qos(10))
        subs['trajectory'] = sub_node.create_subscription(
            Float64MultiArray, f'{rid}_trajectory_data',
            lambda msg, r=rid: self._on_trajectory_data(r, msg), latest_qos(10))
        subs['joint_velocities'] = sub_node.create_subscription(
            Float64MultiArray, f'{rid}_joint_velocities',
            lambda msg, r=rid: self._on_joint_velocities(r, msg), latest_qos(10))
        subs['sim_joint_states'] = sub_node.create_subscription(
            JointState, '/robot%d_simulation/joint_states' % n,
            lambda msg, r=rid: self._on_sim_joint_states(r, msg),
            latest_qos(qos_profile_sensor_data))
        subs['force_torque_sim'] = sub_node.create_subscription(
            Wrench, '/robot%d_simulation/force_torque' % n,
            lambda msg, r=rid: self._on_wrench(r, msg),
            latest_qos(qos_profile_sensor_data))
        subs['force_torque_hw'] = sub_node.create_subscription(
            Wrench, f'{rid}_force_torque',
            lambda msg, r=rid: self._on_wrench(r, msg),
            latest_qos(qos_profile_sensor_data))
        subs['force_torque_vr'] = sub_node.create_subscription(
            Wrench, '/vr/force_torque',
            lambda msg, r=rid: self._on_wrench(r, msg),
            latest_qos(qos_profile_sensor_data))
        subs['joint_torques'] = sub_node.create_subscription(
            Float64MultiArray, f'{rid}_joint_torques',
            lambda msg, r=rid: self._on_joint_torques(r, msg),
            latest_qos(qos_profile_sensor_data))
        subs['contact_estimate'] = sub_node.create_subscription(
            ContactEstimate, f'{rid}_contact_estimate',
            lambda msg, r=rid: self._on_contact_estimate(r, msg), latest_qos(10))
        subs['admittance_offset'] = sub_node.create_subscription(
            Float64MultiArray, f'{rid}_admittance_offset',
            lambda msg, r=rid: self._on_admittance_offset(r, msg),
            latest_qos(qos_profile_sensor_data))
        # Best-effort publisher (SensorDataQoS) -- a reliable subscription
        # would never match it.
        subs['gripper_state'] = sub_node.create_subscription(
            Float64MultiArray, f'{rid}_gripper_state',
            lambda msg, r=rid: self._on_gripper_state(r, msg),
            latest_qos(qos_profile_sensor_data))
        self._js_subs_by_robot[rid] = subs

    def _remove_robot_subscriptions(self, n: int):
        rid = f'robot{n}'
        subs = self._js_subs_by_robot.pop(rid, None)
        if not subs:
            return
        sub_node = self._js_sub_node()
        for sub in subs.values():
            try:
                sub_node.destroy_subscription(sub)
            except Exception:
                pass
        self.current_joint_positions_by_robot.pop(rid, None)
        self.current_ee_pose_by_robot.pop(rid, None)
        self.current_gripper_state_by_robot.pop(rid, None)

    # ── Per-robot read helpers ──────────────────────────────────────────
    def joint_positions_for(self, robot_id) -> list:
        rid = self._normalize_rid(robot_id)
        return list(self.current_joint_positions_by_robot.get(rid, []))

    def ee_pose_for(self, robot_id):
        rid = self._normalize_rid(robot_id)
        return self.current_ee_pose_by_robot.get(rid)

    def sim_joint_by_name(self, robot_id) -> dict:
        """Raw sim joint state keyed by joint name: {name: (pos, vel, eff)}.

        Populated from the sim's <rid>_simulation/joint_states and includes
        EVERY joint the sim publishes (all hinges + any ball-joint orientation
        pseudo-joints), so view-only/passive joints can show live data even
        though the controller only tracks the actuated subset.
        """
        rid = self._normalize_rid(robot_id)
        return self.current_sim_joint_by_name_by_robot.get(rid, {})

    def _normalize_rid(self, robot_id) -> str:
        """Normalise robot_id to a string key like 'robot1'.

        None falls back to station_robot_id so callers may pass None to mean
        'the currently active robot' (matches joint_commanded_for / wrench_for).
        """
        if robot_id is None:
            return self.node.station_robot_id
        if isinstance(robot_id, int):
            return f'robot{robot_id}'
        s = str(robot_id)
        if s.startswith("robot_") and s[len("robot_"):].isdigit():
            return f'robot{s[len("robot_"):]}'
        return s

    # ── Callbacks ───────────────────────────────────────────────────────
    #: Telemetry older than this many milliseconds counts as stale.
    STALE_MS = 500.0

    #: Shortest window used to measure the message rate. Callers tick at
    #: different intervals (Dashboard 20 Hz, control bar 5 Hz); without a
    #: floor, whichever asked second measured a near-zero elapsed time and
    #: reported 0 Hz, so the two indicators disagreed.
    RATE_WINDOW_S = 0.25

    def telemetry_health(self, rid=None):
        """Return ``(rate_hz, age_ms, state)`` for a robot's joint stream.

        ``state`` is 'live', 'stale' or 'offline'. Single source of truth so
        the Dashboard header and the control bar's indicator cannot report
        different things about the same stream. The rate is re-sampled only
        once per ``RATE_WINDOW_S``; callers in between get the last figure.
        """
        now = time.monotonic()
        count = self.joint_update_count(rid)

        if not hasattr(self, '_health_sample'):
            self._health_sample = {}
        sample = self._health_sample.get(rid)
        if sample is None:
            self._health_sample[rid] = {'count': count, 'time': now, 'rate': 0.0}
            rate = 0.0
        else:
            elapsed = now - sample['time']
            if elapsed >= self.RATE_WINDOW_S:
                sample['rate'] = (count - sample['count']) / elapsed
                sample['count'] = count
                sample['time'] = now
            rate = sample['rate']

        stamp = self.joint_update_time(rid)
        if stamp is None:
            return 0.0, None, 'offline'
        age_ms = (now - stamp) * 1000.0
        return rate, age_ms, ('live' if age_ms <= self.STALE_MS else 'stale')

    def joint_update_count(self, rid=None):
        """Total joint-state messages received for the robot."""
        d = getattr(self, 'current_joint_update_count_by_robot', {})
        return d.get(rid or self.node.station_robot_id, 0)

    def joint_update_time(self, rid=None):
        """Monotonic time of the last joint-state update, or None."""
        d = getattr(self, 'current_joint_update_time_by_robot', {})
        return d.get(rid or self.node.station_robot_id)

    def _on_q_current(self, rid: str, msg):
        self.current_joint_positions_by_robot[rid] = list(msg.data)
        self._q_current_received.add(rid)
        self.current_joint_update_count_by_robot[rid] = (
            self.current_joint_update_count_by_robot.get(rid, 0) + 1)
        self.current_joint_update_time_by_robot[rid] = time.monotonic()
        if rid == self.node.station_robot_id:
            self.current_joint_positions = list(msg.data)

    def _on_sim_joint_states(self, rid: str, msg):
        """Store raw sim joint_states keyed by name (pos, vel, eff per joint)."""
        n = len(msg.name)
        pos = msg.position if len(msg.position) == n else [0.0] * n
        vel = msg.velocity if len(msg.velocity) == n else [0.0] * n
        eff = msg.effort if len(msg.effort) == n else [0.0] * n
        self.current_sim_joint_by_name_by_robot[rid] = {
            nm: (float(pos[i]), float(vel[i]), float(eff[i]))
            for i, nm in enumerate(msg.name)
        }
        # The robot's actuated joints, in controller order, as the robot's
        # position (what a real arm's q_current gives).
        if rid in self._q_current_received:
            return
        order = self._sim_joint_order(rid, tuple(msg.name))
        if order is None:
            return
        q = [float(pos[k]) for k in order]
        self.current_joint_positions_by_robot[rid] = q
        now = time.monotonic()
        self._sim_q_time[rid] = now
        self.current_joint_update_count_by_robot[rid] = (
            self.current_joint_update_count_by_robot.get(rid, 0) + 1)
        self.current_joint_update_time_by_robot[rid] = now
        if rid == self.node.station_robot_id:
            self.current_joint_positions = q

    def _sim_joint_order(self, rid, names):
        """Where each actuated joint of the robot is in the simulator's joint_states, or None.

        The robot is the one whose configured ``joint_names`` are all found, the
        toolbar's first; a multi-robot scene prefixes the simulator's names
        (``r2_Actuator1``), so a name also matches after a prefix ending in '_'."""
        key = (rid, names)
        if key in self._sim_order_cache:
            return self._sim_order_cache[key]
        configs = getattr(self, 'robot_configs', {}) or {}
        first = None
        try:
            first = self.robot_dropdown.currentText()
        except Exception:
            pass
        order = None
        for robot in ([first] if first in configs else []) + [r for r in configs if r != first]:
            wanted = list((configs.get(robot) or {}).get('joint_names') or [])
            if not wanted:
                continue
            found = []
            for nm in wanted:
                hits = [k for k, s in enumerate(names) if s == nm] or [
                    k for k, s in enumerate(names) if s.endswith(nm) and s[:-len(nm)].endswith('_')]
                if len(hits) != 1:
                    break
                found.append(hits[0])
            if len(found) == len(wanted):
                order = found
                break
        self._sim_order_cache[key] = order
        return order

    def _on_ee_pose(self, rid: str, msg):
        self.current_ee_pose_by_robot[rid] = msg
        if rid == self.node.station_robot_id:
            self.current_ee_pose = msg

    def _on_trajectory_data(self, rid: str, msg):
        # Only use trajectory_data as a position source when q_current is not
        # available (e.g. simulation-only nodes). If q_current is already
        # feeding this robot, skip position update to avoid overwriting with
        # stale/zero data. Dashboard commanded/effort is always parsed.
        if not msg.data:
            return
        # Cached: this may run on the state thread, where widgets must not be
        # read. _on_robot_changed clears it; the next message fills it again.
        num_joints = getattr(self, '_js_num_joints', None)
        if not num_joints:
            num_joints = self.robots.get(
                self.robot_dropdown.currentText(), 7
            ) if hasattr(self, 'robots') and hasattr(self, 'robot_dropdown') else 7
            self._js_num_joints = num_joints
        raw = list(msg.data)
        # Store commanded position and effort for the dashboard (mode-aware).
        # The active mode picks the layout. _current_control_mode is never set;
        # the selected mode (control bar, web remote, teleop) is the real one.
        mode = getattr(self, '_current_control_mode', None) \
            or int(getattr(self, '_rc_selected_mode_idx', 0)) + 1
        cmd, eff = parse_trajectory_joint(raw, num_joints, mode)
        self.current_joint_commanded_by_robot[rid] = cmd
        if eff and any(eff):
            self.current_joint_efforts_by_robot[rid] = eff
        cart_err = parse_cartesian_error(raw, num_joints)
        if cart_err is not None:
            self.current_cartesian_error_by_robot[rid] = cart_err
        # Position update: only when neither q_current nor the simulator's
        # joint_states provide it.
        if rid in self._q_current_received:
            return
        if time.monotonic() - self._sim_q_time.get(rid, -1e9) < 0.5:
            return
        data = parse_trajectory_positions(raw, num_joints, mode)
        if data is None:
            return
        self.current_joint_positions_by_robot[rid] = data
        if rid == self.node.station_robot_id:
            self.current_joint_positions = data

    def _on_js_joint_states_global(self, msg):
        """Global /joint_states or /simulation_joint_states — no rid prefix.
        Routes to the first active robot's slot (best we can do without a
        prefix)."""
        if not msg.position:
            return
        rid = self.node.station_robot_id
        data = list(msg.position)
        self.current_joint_positions_by_robot[rid] = data
        self.current_joint_positions = data
        if msg.velocity:
            self.current_joint_velocities_by_robot[rid] = list(msg.velocity)
        if msg.effort:
            self.current_joint_efforts_by_robot[rid] = list(msg.effort)

    def _on_joint_velocities(self, rid: str, msg):
        """Store joint velocities from the hardware node."""
        if msg.data:
            self.current_joint_velocities_by_robot[rid] = list(msg.data)

    def _on_wrench(self, rid: str, msg):
        """Store the latest tool wrench for the given robot."""
        f, t = msg.force, msg.torque
        self.current_wrench_by_robot[rid] = (f.x, f.y, f.z, t.x, t.y, t.z)

    def _on_joint_torques(self, rid: str, msg):
        """Store the latest MEASURED joint torque for the given robot.

        From ``{rid}_joint_torques`` — the arm's real torque sensor
        reading, unlike joint_effort_for()'s commanded control effort.
        """
        if msg.data:
            self.current_joint_torque_by_robot[rid] = list(msg.data)

    def _on_contact_estimate(self, rid: str, msg):
        """Store the latest contact estimate for the given robot.

        From ``{rid}_contact_estimate``. Stored unconditionally (even when
        link_index is -1) because -1 and 0 are both meaningful readings —
        see contact_estimate_for()'s docstring for the full state table.
        """
        self.current_contact_estimate_by_robot[rid] = {
            'link_index': int(msg.link_index),
            'link_name': msg.link_name,
            'confidence': float(msg.confidence),
            'calibrated': bool(msg.calibrated),
        }

    def _on_admittance_offset(self, rid: str, msg):
        """Store the latest accumulated human push offset for the robot.

        From ``{rid}_admittance_offset``, published only while an
        admittance mode (3 or 4) is active — absent, not zero, otherwise.
        """
        if msg.data:
            self.current_admittance_offset_by_robot[rid] = list(msg.data)

    def _on_gripper_state(self, rid: str, msg):
        """Store the latest MEASURED gripper state for the given robot.

        From ``{rid}_gripper_state`` (fic_control), data = [width_m,
        motor_current]. A short or non-finite message is dropped rather
        than clobbering the last good reading.
        """
        data = msg.data
        if len(data) < 2:
            return
        width, current = float(data[0]), float(data[1])
        if not (math.isfinite(width) and math.isfinite(current)):
            return
        self.current_gripper_state_by_robot[rid] = (
            width, current, time.monotonic())

    # ── Dashboard telemetry read helpers ────────────────────────────────
    def joint_commanded_for(self, rid=None):
        """Return a copy of the latest commanded joint positions for rid."""
        return list(self.current_joint_commanded_by_robot.get(
            rid or self.node.station_robot_id, []))

    def joint_velocity_for(self, rid=None):
        """Return a copy of the latest joint velocities for rid."""
        return list(self.current_joint_velocities_by_robot.get(
            rid or self.node.station_robot_id, []))

    def joint_effort_for(self, rid=None):
        """Return a copy of the latest COMMANDED joint control effort, Nm.

        From trajectory_data block 4 — what the FIC controller commanded,
        not what the arm measured. For the measured torque sensor reading
        use joint_torque_for(), which reads {rid}_joint_torques.
        """
        return list(self.current_joint_efforts_by_robot.get(
            rid or self.node.station_robot_id, []))

    def wrench_for(self, rid=None):
        """Return (Fx, Fy, Fz, Tx, Ty, Tz) tuple for rid, or None."""
        return self.current_wrench_by_robot.get(rid or self.node.station_robot_id)

    def cartesian_error_for(self, rid=None):
        """Return (ex, ey, ez, e_angle) for rid, or None."""
        return self.current_cartesian_error_by_robot.get(
            rid or self.node.station_robot_id)

    def joint_torque_for(self, rid=None):
        """Return a copy of the latest MEASURED joint torque for rid, Nm.

        From {rid}_joint_torques — the arm's real torque sensor reading.
        Empty ([]) before the first message. Do not confuse this with
        joint_effort_for(), which returns the controller's COMMANDED
        effort, not a measurement.
        """
        return list(self.current_joint_torque_by_robot.get(
            rid or self.node.station_robot_id, []))

    def contact_estimate_for(self, rid=None):
        """Return the latest contact estimate dict for rid, or None.

        Keys: ``link_index``, ``link_name``, ``confidence`` (0..1) and
        ``calibrated`` (bool). ``link_index`` has three regimes callers
        must not collapse: ``-1`` means not calibrated / gated off (no
        statement is possible), ``0`` means a genuine "nothing is
        touching the arm" measurement, and ``1..n`` names the contacted
        link. Returning ``None`` (no {rid}_contact_estimate message has
        ever arrived) is a fourth, distinct state — absence is not zero.
        """
        return self.current_contact_estimate_by_robot.get(
            rid or self.node.station_robot_id)

    def admittance_offset_for(self, rid=None):
        """Return a copy of the accumulated human push offset for rid, rad.

        From {rid}_admittance_offset — the FIC controller's own
        integrated joint-admittance offset. Published only while an
        admittance mode (3 or 4) is active; ``[]`` in every other mode or
        before the first message.
        """
        return list(self.current_admittance_offset_by_robot.get(
            rid or self.node.station_robot_id, []))

    def gripper_state_for(self, rid=None):
        """Return ``(width_m, current, age_s)`` for rid's gripper, or None.

        From {rid}_gripper_state, which fic_control publishes at ~50 Hz
        only once the driver writes a measured gripper state. ``width_m``
        is the measured finger opening in metres (0 = closed, ~0.085 =
        fully open on a Robotiq 2F-85), ``current`` the gripper motor
        current as the driver reports it (Kinova: mA) and ``age_s`` the
        seconds since that sample arrived -- a growing age means the feed
        stopped. None means no message has ever arrived: simulation, no
        gripper, or a driver that predates the feed.
        """
        state = self.current_gripper_state_by_robot.get(
            self._normalize_rid(rid))
        if state is None:
            return None
        width, current, stamp = state
        return width, current, max(0.0, time.monotonic() - stamp)
