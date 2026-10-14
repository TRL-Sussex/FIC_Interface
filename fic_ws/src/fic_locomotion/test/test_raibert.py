"""Raibert foot placement: neutral stance, velocity lead, error correction."""
import math

import pytest

from fic_locomotion.raibert import touchdown_target

NOMINAL = (0.3, 0.17, -0.43)


def test_standing_still_lands_on_the_nominal_point():
    out = touchdown_target(NOMINAL, (0.0, 0.0), (0.0, 0.0), 0.0, 0.24)
    assert out == pytest.approx(NOMINAL)


def test_forward_command_leads_by_half_a_stance_period():
    # The classic heuristic: land half a stance-length ahead so the foot ends
    # up under the trunk at mid-stance.
    out = touchdown_target(NOMINAL, (0.4, 0.0), (0.4, 0.0), 0.0, 0.24)
    assert out[0] == pytest.approx(NOMINAL[0] + 0.5 * 0.24 * 0.4)


def test_velocity_error_pulls_the_foot_to_correct_drift():
    # Moving faster than commanded: plant the foot further ahead to brake.
    fast = touchdown_target(NOMINAL, (0.0, 0.0), (0.3, 0.0), 0.0, 0.24, gain=0.1)
    assert fast[0] > NOMINAL[0]
    slow = touchdown_target(NOMINAL, (0.0, 0.0), (-0.3, 0.0), 0.0, 0.24, gain=0.1)
    assert slow[0] < NOMINAL[0]


def test_lateral_command_shifts_sideways():
    out = touchdown_target(NOMINAL, (0.0, 0.2), (0.0, 0.2), 0.0, 0.24)
    assert out[1] == pytest.approx(NOMINAL[1] + 0.5 * 0.24 * 0.2)


def test_yaw_rate_rotates_the_target_about_the_trunk_centre():
    out = touchdown_target(NOMINAL, (0.0, 0.0), (0.0, 0.0), 1.0, 0.24)
    theta = 0.5 * 0.24 * 1.0
    expected_x = NOMINAL[0] * math.cos(theta) - NOMINAL[1] * math.sin(theta)
    assert out[0] == pytest.approx(expected_x)
    assert math.hypot(out[0], out[1]) == pytest.approx(
        math.hypot(NOMINAL[0], NOMINAL[1]))


def test_height_is_never_changed():
    out = touchdown_target(NOMINAL, (0.5, 0.3), (0.1, 0.1), 0.8, 0.24)
    assert out[2] == pytest.approx(NOMINAL[2])
