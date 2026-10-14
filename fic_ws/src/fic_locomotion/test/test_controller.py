"""Locomotion controller: the whole gait, driven without ROS or physics."""
import math

import pytest

from fic_locomotion import leg_kinematics as lk
from fic_locomotion import controller as controller_module
from fic_locomotion.controller import (
    ControllerConfig, LocomotionController, RobotState, VelocityCommand)
from fic_locomotion.gait_state_machine import GaitState
from fic_locomotion.trot_scheduler import GaitParams

DT = 0.005
# Written out rather than recomputed from LocomotionController.MAX_JOINT_RATE:
# a bound derived from the same constant the limiter uses is satisfied by any
# value of that constant, so raising it to 1e6 left the old assertions green.
# 8.0 rad/s * 0.005 s. If the ceiling is deliberately retuned, this is meant to
# fail and be updated by hand.
MAX_STEP = 0.04


def _state(targets=None, height=0.46, roll=0.0, pitch=0.0,
           velocity=(0.0, 0.0), yaw_rate=0.0, valid=True):
    """Build a state where the robot perfectly tracks whatever was last commanded."""
    return RobotState(
        joint_positions=list(targets or lk.SIT_Q * 4),
        height=height, roll=roll, pitch=pitch,
        velocity=velocity, yaw_rate=yaw_rate, valid=valid)


def _run(controller, seconds, command=None, **kwargs):
    command = command or VelocityCommand()
    targets = list(lk.SIT_Q * 4)
    for _ in range(int(seconds / DT)):
        targets = controller.update(DT, _state(targets, **kwargs), command)
    return targets


def test_wrong_leg_count_is_rejected():
    with pytest.raises(ValueError):
        LocomotionController(legs=lk.SPOT_LEGS[:2])


def test_unknown_leg_name_is_rejected():
    bogus = lk.LegGeometry('bogus', (0.0, 0.0, 0.0), 1.0)
    with pytest.raises(ValueError):
        LocomotionController(legs=lk.SPOT_LEGS[:3] + (bogus,))


def test_targets_seed_from_the_first_measured_state():
    # `_targets` used to start from SIT_Q regardless of the robot's actual
    # pose, so the very first rate-limited step was measured against a
    # fictitious starting point rather than where the robot really is.
    controller = LocomotionController()
    measured = [q + 0.05 for q in lk.STAND_Q * 4]
    targets = controller.update(DT, _state(measured), VelocityCommand())
    for old, new in zip(measured, targets):
        assert abs(new - old) <= MAX_STEP + 1e-9


def test_starts_and_stays_sitting():
    controller = LocomotionController()
    targets = _run(controller, 0.5)
    assert controller.state is GaitState.SIT
    assert targets == pytest.approx(list(lk.SIT_Q * 4), abs=1e-6)


def test_stand_reaches_the_home_pose():
    controller = LocomotionController()
    controller.request('stand')
    targets = _run(controller, 2.5)
    assert controller.state is GaitState.STAND
    assert targets == pytest.approx(list(lk.STAND_Q * 4), abs=1e-3)


def test_walking_produces_periodic_joint_motion_at_the_gait_frequency():
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    command = VelocityCommand(vx=0.3)
    knee = []
    for _ in range(int(1.0 / DT)):
        targets = controller.update(DT, _state(height=0.46, velocity=(0.3, 0.0)),
                                    command)
        knee.append(targets[2])
    assert controller.state is GaitState.TROT
    assert max(knee) - min(knee) > 0.05  # the leg is actually stepping
    # A 0.30 s cycle inside a 1 s window means three or four lift-offs.
    crossings = sum(1 for a, b in zip(knee, knee[1:])
                    if (a < sum(knee) / len(knee)) != (b < sum(knee) / len(knee)))
    assert 3 <= crossings <= 8


def test_diagonal_legs_step_out_of_phase():
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    command = VelocityCommand(vx=0.3)
    fl, fr = [], []
    for _ in range(int(0.30 / DT)):
        targets = controller.update(DT, _state(velocity=(0.3, 0.0)), command)
        fl.append(targets[2])
        fr.append(targets[5])
    # If the diagonals were in phase these two traces would be identical.
    assert max(abs(a - b) for a, b in zip(fl, fr)) > 0.05


def test_trunk_riding_high_shortens_the_stance_legs():
    # Asserted with a SIGN, deliberately. `!=` passes for a sign-flipped
    # posture height gain -- i.e. for a controller that responds to a high
    # trunk by pushing it higher still -- and for NaN, and for any random
    # perturbation. Only the direction says the feedback is stabilising.
    controller = LocomotionController()
    controller.request('stand')
    level = _run(controller, 2.5)
    high = _run(controller, 0.5, height=0.52)
    # Drawing the feet up toward a high trunk closes the knee: measured
    # -1.940 against a level -1.800.
    assert high[2] < level[2] - 0.05

    controller = LocomotionController()
    controller.request('stand')
    level = _run(controller, 2.5)
    low = _run(controller, 0.5, height=0.40)
    # ...and a low trunk extends it, measured -1.651.
    assert low[2] > level[2] + 0.05


def test_a_fall_faults_and_tucks_the_legs():
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    targets = _run(controller, 2.0, roll=0.9)
    assert controller.state is GaitState.FAULT
    assert targets == pytest.approx(list(lk.SIT_Q * 4), abs=1e-2)


def test_invalid_feedback_faults():
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    _run(controller, 0.1, valid=False)
    assert controller.state is GaitState.FAULT


def test_non_finite_velocity_faults_and_tucks():
    # A NaN comparison is silently False in Python, so an unguarded NaN in
    # velocity would slip past every threshold check and, through
    # raibert.touchdown_target and clamp_to_limits, drive every joint toward
    # its upper limit instead of tripping FAULT.
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    command = VelocityCommand(vx=0.3)
    _run(controller, 1.0, command, velocity=(0.3, 0.0))
    assert controller.state is GaitState.TROT
    targets = _run(controller, 2.0, command, velocity=(float('nan'), 0.0))
    assert controller.state is GaitState.FAULT
    assert targets == pytest.approx(list(lk.SIT_Q * 4), abs=1e-2)


def test_non_finite_attitude_faults_and_tucks():
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    targets = _run(controller, 2.0, roll=float('nan'))
    assert controller.state is GaitState.FAULT
    assert targets == pytest.approx(list(lk.SIT_Q * 4), abs=1e-2)


def test_commands_are_clamped_to_the_configured_caps():
    # The old joint-limit test carried a "far over the caps" comment but never
    # asserted the caps, so deleting the max_vx clamp left the suite green.
    controller = LocomotionController()
    clamped = controller._clamp_command(VelocityCommand(5.0, 5.0, 5.0))
    cfg = controller.config
    assert clamped.vx == pytest.approx(cfg.max_vx)
    assert clamped.vy == pytest.approx(cfg.max_vy)
    assert clamped.yaw_rate == pytest.approx(cfg.max_yaw_rate)
    mirrored = controller._clamp_command(VelocityCommand(-5.0, -5.0, -5.0))
    assert mirrored.vx == pytest.approx(-cfg.max_vx)
    assert mirrored.vy == pytest.approx(-cfg.max_vy)
    assert mirrored.yaw_rate == pytest.approx(-cfg.max_yaw_rate)


def test_targets_never_leave_the_joint_limits(monkeypatch):
    # This used to run the shipped gait, which never approaches a joint limit,
    # so making `clamp_to_limits` a pure passthrough left it green: it asserted
    # an invariant nothing was threatening. Drive it instead with a mis-tune
    # that is *inside* the node's accepted parameter ranges -- a 0.5 m lift on
    # a 2 s cycle -- which does command the knee past its stop, and require the
    # clamp to have actually engaged.
    engaged = []
    real_clamp = lk.clamp_to_limits

    def spy(targets):
        clamped = real_clamp(targets)
        if any(abs(a - b) > 1e-12 for a, b in zip(targets, clamped)):
            engaged.append(True)
        return clamped

    monkeypatch.setattr(controller_module.lk, 'clamp_to_limits', spy)
    config = ControllerConfig(
        gait=GaitParams(cycle_time=2.0, duty_factor=0.65, step_height=0.5))
    controller = LocomotionController(config)
    controller.request('stand')
    _run(controller, 2.5)
    command = VelocityCommand(vx=0.3)
    for _ in range(int(8.0 / DT)):
        targets = controller.update(DT, _state(velocity=(0.3, 0.0)), command)
        for value, (lo, hi) in zip(targets, lk.JOINT_LIMITS):
            assert lo - 1e-9 <= value <= hi + 1e-9
    assert engaged, 'the clamp never engaged, so this proves nothing'


def test_output_respects_the_joint_rate_limit():
    # Two-sided on purpose. The upper bound alone was satisfiable by any
    # MAX_JOINT_RATE because it was recomputed from that same constant; the
    # lower bound requires the limiter to actually be the thing shaping the
    # output, which it is on roughly half the ticks of a trot.
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    command = VelocityCommand(vx=0.3)
    previous = None
    largest = 0.0
    for _ in range(int(2.0 / DT)):
        targets = controller.update(DT, _state(velocity=(0.3, 0.0)), command)
        if previous is not None:
            for old, new in zip(previous, targets):
                assert abs(new - old) <= MAX_STEP + 1e-9
            largest = max(largest,
                          max(abs(n - o) for o, n in zip(previous, targets)))
        previous = targets
    assert largest == pytest.approx(MAX_STEP, abs=1e-6), \
        'the limiter never saturated, so the upper bound proves nothing'


def test_the_default_raibert_gain_feeds_velocity_error_back():
    # raibert_gain is the only feedback stabilising the kinematic trot, and
    # setting it to 0.0 passed the entire suite: test_raibert.py always passes
    # a gain explicitly, and the walk test's +/-30 % bands are far too wide to
    # notice. With the feedback live, a robot moving slower than commanded must
    # place its feet differently from one tracking perfectly; with the gain at
    # zero the two are bit-identical, because touchdown then depends only on
    # the command.
    def targets_for(measured_velocity):
        controller = LocomotionController()
        controller.request('stand')
        _run(controller, 2.5)
        command = VelocityCommand(vx=0.3)
        history = []
        for _ in range(int(0.6 / DT)):
            history.append(controller.update(
                DT, _state(velocity=measured_velocity), command))
        return history

    tracking = targets_for((0.3, 0.0))
    lagging = targets_for((0.0, 0.0))
    divergence = max(max(abs(a - b) for a, b in zip(one, other))
                     for one, other in zip(tracking, lagging))
    # Measured 0.105 rad at the shipped gain of 0.12; exactly 0.0 at gain 0.
    assert divergence > 0.03, 'velocity error is not reaching foot placement'


def test_returns_twelve_targets_in_joint_name_order():
    controller = LocomotionController()
    targets = controller.update(DT, _state(), VelocityCommand())
    assert len(targets) == len(lk.JOINT_NAMES) == 12
    assert all(math.isfinite(value) for value in targets)


def test_transition_anchors_the_blend_on_measured_joints():
    # Anchoring on the last command instead would aim the whole stand-up
    # trajectory from a pose the robot is not actually in. The rate limiter
    # stops the output stepping either way; this is about aiming it correctly.
    controller = LocomotionController()
    controller.request('stand')
    lagging = [q - 0.25 for q in lk.SIT_Q * 4]
    controller.update(DT, _state(lagging), VelocityCommand())
    assert controller._blend_from == pytest.approx(lagging)


def test_standing_still_converges_even_from_a_lagging_start():
    controller = LocomotionController()
    controller.request('stand')
    targets = _run(controller, 3.0)
    assert targets == pytest.approx(list(lk.STAND_Q * 4), abs=1e-3)


def test_combined_forward_and_yaw_command_produces_valid_gait():
    # A review of raibert.touchdown_target noted that yaw is applied to an
    # already-translated foot target, and that forward-plus-yaw together --
    # Spot's normal turning-while-walking mode -- was untested. Drive the
    # controller with both at once and check the gait stays well-behaved.
    controller = LocomotionController()
    controller.request('stand')
    _run(controller, 2.5)
    command = VelocityCommand(vx=0.3, yaw_rate=0.5)
    knee = []
    for _ in range(int(1.0 / DT)):
        targets = controller.update(DT, _state(velocity=(0.3, 0.0), yaw_rate=0.5),
                                    command)
        assert all(math.isfinite(value) for value in targets)
        for value, (lo, hi) in zip(targets, lk.JOINT_LIMITS):
            assert lo - 1e-9 <= value <= hi + 1e-9
        knee.append(targets[2])
    assert controller.state is GaitState.TROT
    assert max(knee) - min(knee) > 0.05  # still stepping, not locked up
