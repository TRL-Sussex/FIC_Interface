"""
Locomotion controller: robot state in, twelve joint targets out.

Deliberately free of ROS so the whole gait can be exercised in a unit test or
driven straight into MuJoCo without a running graph. The node in
`locomotion_node.py` is a thin shell around this class.

The output contract is twelve joint position targets in `JOINT_NAMES` order,
which the FIC controller tracks in joint-impedance mode. Nothing here computes
a torque — compliance and tracking are entirely FIC's job.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from fic_locomotion import leg_kinematics as lk
from fic_locomotion import posture, raibert, swing_trajectory, trot_scheduler
from fic_locomotion.gait_state_machine import GaitState, GaitStateMachine
from fic_locomotion.trot_scheduler import GaitParams


@dataclass
class RobotState:
    """Everything the controller needs to know about the robot right now."""

    joint_positions: Sequence[float]  # 12, in JOINT_NAMES order
    height: float  # trunk height above the ground plane, metres
    roll: float
    pitch: float
    velocity: Tuple[float, float]  # trunk-frame vx, vy
    yaw_rate: float
    valid: bool = True


@dataclass
class VelocityCommand:
    """What the operator is asking the robot to do."""

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    @property
    def speed(self) -> float:
        """
        Scalar summary of demand, used only to decide whether to gait.

        The yaw term is weighted by a nominal body radius so that a pure turn
        registers as motion on the same scale as a translation.
        """
        return math.hypot(self.vx, self.vy) + abs(self.yaw_rate) * 0.3


@dataclass
class ControllerConfig:
    """Tunable gait, posture and command limits."""

    gait: GaitParams = field(default_factory=GaitParams)
    gains: posture.PostureGains = field(default_factory=posture.PostureGains)
    stand_height: float = 0.46
    max_vx: float = 0.6
    max_vy: float = 0.4
    max_yaw_rate: float = 1.0
    raibert_gain: float = 0.12


def _validate_legs(legs: Sequence[lk.LegGeometry]) -> None:
    """
    Raise ValueError if `legs` cannot drive the fixed 12-entry joint tables.

    `JOINT_LIMITS`/`JOINT_NAMES` assume exactly three joints per leg, and
    `trot_scheduler.LEG_PHASE_OFFSET` must carry an entry for every leg name.
    Either mismatch would otherwise truncate silently through the `zip` calls
    in `clamp_to_limits`/`_rate_limit`, or raise a confusing `KeyError` deep
    inside `trot_scheduler.leg_phase`.
    """
    joints_per_leg = 3
    expected = len(lk.JOINT_LIMITS) // joints_per_leg
    if len(legs) != expected:
        raise ValueError(
            'legs must have %d entries (%d joints each) to match '
            'JOINT_LIMITS/JOINT_NAMES, got %d'
            % (expected, joints_per_leg, len(legs)))
    unknown = [leg.name for leg in legs
               if leg.name not in trot_scheduler.LEG_PHASE_OFFSET]
    if unknown:
        raise ValueError(
            'leg names %r have no entry in trot_scheduler.LEG_PHASE_OFFSET'
            % unknown)


def _feedback_is_finite(state: RobotState) -> bool:
    """
    Return whether every float `update` reads from `state` is finite.

    A NaN comparison is silently False in Python, so an unguarded NaN in
    attitude or velocity would slip past the tilt-fault check in
    `GaitStateMachine.update`, and `clamp_to_limits` would then collapse it
    onto the joint's upper limit instead of tripping FAULT.
    """
    scalars = (state.height, state.roll, state.pitch,
               state.velocity[0], state.velocity[1], state.yaw_rate)
    return (all(math.isfinite(v) for v in scalars)
            and all(math.isfinite(v) for v in state.joint_positions))


class LocomotionController:
    """Turn a velocity command into twelve joint position targets."""

    # Ceiling on how fast a target may move, rad/s. Generous enough not to bind
    # during a trot, tight enough that a fault or mode change cannot command a
    # step discontinuity into a compliant controller.
    MAX_JOINT_RATE = 8.0

    def __init__(self, config: Optional[ControllerConfig] = None,
                 legs: Sequence[lk.LegGeometry] = lk.SPOT_LEGS):
        _validate_legs(legs)
        self.config = config or ControllerConfig()
        self.legs = legs
        self.machine = GaitStateMachine()
        self.phase = 0.0
        self._targets: List[float] = self._pose(lk.SIT_Q)
        self._blend_from: List[float] = list(self._targets)
        self._capture_blend = False
        # Replaced by the first measured joint state on the first `update`
        # call, so the rate limiter bounds motion from where the robot
        # actually is rather than from an assumed SIT_Q starting pose.
        self._seeded = False

    @property
    def state(self) -> GaitState:
        """Return the current gait state."""
        return self.machine.state

    def request(self, command: str) -> None:
        """Apply an operator command: 'sit', 'stand' or 'walk'."""
        before = self.machine.state
        self.machine.request(command)
        if self.machine.state is not before and self.machine.state in (
                GaitState.STANDING_UP, GaitState.SITTING_DOWN):
            # Anchor the blend on the next tick, when measured joint angles are
            # available. Starting from the last *command* instead would step the
            # target by however far the tracker is lagging, and a step into a
            # compliant controller is exactly the torque spike to avoid.
            self._capture_blend = True

    def update(self, dt: float, state: RobotState,
               command: VelocityCommand) -> List[float]:
        """Advance one control tick and return twelve joint position targets."""
        if not self._seeded and all(math.isfinite(q) for q in state.joint_positions):
            self._targets = list(state.joint_positions)
            self._seeded = True

        cmd = self._clamp_command(command)
        tilt = math.hypot(state.roll, state.pitch)
        feedback_ok = state.valid and _feedback_is_finite(state)
        gait = self.machine.update(dt, cmd.speed, tilt, feedback_ok)

        if self._capture_blend:
            self._blend_from = list(state.joint_positions)
            self._capture_blend = False

        if gait in (GaitState.SIT, GaitState.FAULT):
            targets = self._pose(lk.SIT_Q)
        elif gait is GaitState.STANDING_UP:
            targets = self._blend(self._blend_from, self._pose(lk.STAND_Q))
        elif gait is GaitState.SITTING_DOWN:
            targets = self._blend(self._blend_from, self._pose(lk.SIT_Q))
        else:
            targets = self._foot_targets(dt, state, cmd,
                                         walking=gait is GaitState.TROT)

        targets = lk.clamp_to_limits(self._rate_limit(targets, dt))
        self._targets = targets
        return targets

    # -- internals -------------------------------------------------------
    def _clamp_command(self, command: VelocityCommand) -> VelocityCommand:
        cfg = self.config
        return VelocityCommand(
            max(-cfg.max_vx, min(cfg.max_vx, command.vx)),
            max(-cfg.max_vy, min(cfg.max_vy, command.vy)),
            max(-cfg.max_yaw_rate, min(cfg.max_yaw_rate, command.yaw_rate)))

    def _pose(self, q: Sequence[float]) -> List[float]:
        """Repeat one per-leg joint triple across all four legs."""
        return [angle for _ in self.legs for angle in q]

    def _blend(self, start: Sequence[float],
               end: Sequence[float]) -> List[float]:
        ratio = self.machine.blend
        return [a + (b - a) * ratio for a, b in zip(start, end)]

    def _rate_limit(self, targets: Sequence[float], dt: float) -> List[float]:
        step = self.MAX_JOINT_RATE * dt
        return [max(prev - step, min(prev + step, target))
                for prev, target in zip(self._targets, targets)]

    def _foot_targets(self, dt: float, state: RobotState,
                      cmd: VelocityCommand, walking: bool) -> List[float]:
        cfg = self.config
        gait = cfg.gait
        if walking:
            self.phase = trot_scheduler.advance(self.phase, dt, gait.cycle_time)
        height_error = state.height - cfg.stand_height
        stance_time = trot_scheduler.stance_duration(gait)

        targets: List[float] = []
        for leg in self.legs:
            nominal = lk.nominal_stance(leg)
            ground = nominal[2] + posture.stance_offset(
                nominal, height_error, state.roll, state.pitch, cfg.gains)

            if not walking:
                foot = (nominal[0], nominal[1], ground)
            else:
                touchdown = raibert.touchdown_target(
                    nominal, (cmd.vx, cmd.vy), state.velocity, cmd.yaw_rate,
                    stance_time, cfg.raibert_gain)
                phase = trot_scheduler.leg_phase(
                    self.phase, leg.name, gait.duty_factor)
                if phase.stance:
                    foot = self._stance_foot(
                        touchdown, cmd, stance_time, phase.progress, ground)
                else:
                    # Swing starts exactly where stance ended, so lift-off is
                    # continuous without caching any per-leg history.
                    lift = self._stance_foot(
                        touchdown, cmd, stance_time, 1.0, ground)
                    land = (touchdown[0], touchdown[1], ground)
                    foot = swing_trajectory.swing_foot(
                        lift, land, phase.progress, gait.step_height)
            targets.extend(lk.inverse_kinematics(leg, foot))
        return targets

    @staticmethod
    def _stance_foot(touchdown: lk.Vec3, cmd: VelocityCommand,
                     stance_time: float, progress: float,
                     ground: float) -> lk.Vec3:
        """
        Return the trunk-frame position of a planted foot `progress` through stance.

        A planted foot is fixed in the world, so in the trunk frame it slides
        backwards at exactly the commanded velocity. That sliding is the term
        that actually propels the robot.
        """
        travel = progress * stance_time
        x = touchdown[0] - cmd.vx * travel
        y = touchdown[1] - cmd.vy * travel
        theta = -cmd.yaw_rate * travel
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        return (x * cos_t - y * sin_t, x * sin_t + y * cos_t, ground)
