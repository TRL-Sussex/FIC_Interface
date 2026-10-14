"""
The turn command's sign, pinned end to end through the planner.

A yaw sign error is invisible to every other test here: `touchdown_target`
rotates by `+0.5 * stance_time * yaw_rate` and `_stance_foot` rotates by
`-yaw_rate * travel`, and each of those signs is individually correct, so a
test of either one in isolation passes whichever way the robot actually turns.
What matters is the *trunk* rotation the pair implies, and that is what is
asserted here.

The physics-level counterpart lives in `test_walk_integration.py`, which
asserts the signed yaw MuJoCo actually produces. This file is the cheap,
deterministic guard: it needs no simulator and runs in milliseconds.
"""
import math

import pytest

from fic_locomotion import leg_kinematics as lk
from fic_locomotion import raibert, trot_scheduler
from fic_locomotion.controller import LocomotionController, VelocityCommand

GAIT = trot_scheduler.GaitParams()
STANCE = trot_scheduler.stance_duration(GAIT)


def _implied_trunk_yaw_rate(yaw_rate, leg, gain=0.12):
    """
    Yaw rate of the trunk implied by how the planner sweeps a planted foot.

    A foot that is fixed in the world, seen from a trunk yawing at `omega`,
    moves in the trunk frame as `dp/dt = -omega x p`, whose z component gives
    `omega = -(p x dp/dt)_z / |p|^2`. Feeding the planner's own stance
    trajectory through that identity recovers the trunk rotation the gait is
    actually asking for, sign included.
    """
    nominal = lk.nominal_stance(leg)
    cmd = VelocityCommand(yaw_rate=yaw_rate)
    touchdown = raibert.touchdown_target(
        nominal, (0.0, 0.0), (0.0, 0.0), yaw_rate, STANCE, gain)
    early = LocomotionController._stance_foot(
        touchdown, cmd, STANCE, 0.4, nominal[2])
    late = LocomotionController._stance_foot(
        touchdown, cmd, STANCE, 0.6, nominal[2])
    dt = 0.2 * STANCE
    vx = (late[0] - early[0]) / dt
    vy = (late[1] - early[1]) / dt
    px = 0.5 * (early[0] + late[0])
    py = 0.5 * (early[1] + late[1])
    return -(px * vy - py * vx) / (px * px + py * py)


@pytest.mark.parametrize('leg', lk.SPOT_LEGS, ids=[leg.name for leg in lk.SPOT_LEGS])
def test_positive_yaw_command_turns_the_trunk_left(leg):
    """A positive `angular.z` must sweep the stance feet so the trunk turns +z."""
    implied = _implied_trunk_yaw_rate(0.5, leg)
    assert implied > 0.0, (
        'leg %s sweeps its stance foot the wrong way: a +0.5 rad/s command '
        'implies %.3f rad/s of trunk yaw' % (leg.name, implied))
    assert implied == pytest.approx(0.5, rel=0.05)


@pytest.mark.parametrize('leg', lk.SPOT_LEGS, ids=[leg.name for leg in lk.SPOT_LEGS])
def test_negative_yaw_command_turns_the_trunk_right(leg):
    implied = _implied_trunk_yaw_rate(-0.5, leg)
    assert implied < 0.0, (
        'leg %s sweeps its stance foot the wrong way: a -0.5 rad/s command '
        'implies %.3f rad/s of trunk yaw' % (leg.name, implied))
    assert implied == pytest.approx(-0.5, rel=0.05)


def test_the_stance_sweep_is_centred_on_the_neutral_point():
    """
    Touch-down leads the neutral point by the same angle lift-off trails it.

    This is what makes the two opposite-signed rotations a matched pair rather
    than a double negative: half the sweep ahead, half behind. A sign flip in
    either one alone breaks the symmetry as well as the direction.
    """
    leg = lk.SPOT_LEGS[0]
    nominal = lk.nominal_stance(leg)
    cmd = VelocityCommand(yaw_rate=0.5)
    touchdown = raibert.touchdown_target(
        nominal, (0.0, 0.0), (0.0, 0.0), 0.5, STANCE, 0.12)
    plant = LocomotionController._stance_foot(
        touchdown, cmd, STANCE, 0.0, nominal[2])
    lift = LocomotionController._stance_foot(
        touchdown, cmd, STANCE, 1.0, nominal[2])
    neutral = math.atan2(nominal[1], nominal[0])
    ahead = math.atan2(plant[1], plant[0]) - neutral
    behind = neutral - math.atan2(lift[1], lift[0])
    assert ahead > 0.0
    assert ahead == pytest.approx(behind, rel=1e-6)
    assert ahead == pytest.approx(0.5 * STANCE * 0.5, rel=1e-6)


def test_a_yaw_command_alone_does_not_translate_the_stance_pattern():
    """The four stance feet must rotate about the trunk, not drift with it."""
    cmd = VelocityCommand(yaw_rate=0.5)
    for progress in (0.0, 0.5, 1.0):
        radii = []
        for leg in lk.SPOT_LEGS:
            nominal = lk.nominal_stance(leg)
            touchdown = raibert.touchdown_target(
                nominal, (0.0, 0.0), (0.0, 0.0), 0.5, STANCE, 0.12)
            foot = LocomotionController._stance_foot(
                touchdown, cmd, STANCE, progress, nominal[2])
            radii.append((math.hypot(foot[0], foot[1]),
                          math.hypot(nominal[0], nominal[1])))
        for got, want in radii:
            assert got == pytest.approx(want, rel=1e-9)
