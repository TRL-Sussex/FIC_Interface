"""Posture correction: trunk height and attitude driven through stance feet."""
import pytest

from fic_locomotion.posture import PostureGains, stance_offset

GAINS = PostureGains()
FRONT_LEFT = (0.3, 0.17, -0.43)
HIND_RIGHT = (-0.3, -0.17, -0.43)


def test_level_and_on_height_needs_no_correction():
    assert stance_offset(FRONT_LEFT, 0.0, 0.0, 0.0, GAINS) == pytest.approx(0.0)


def test_trunk_riding_high_draws_the_feet_up_toward_it():
    # Positive height error means measured above desired: shorten the legs by
    # raising the foot target in the trunk frame.
    assert stance_offset(FRONT_LEFT, 0.05, 0.0, 0.0, GAINS) > 0.0
    assert stance_offset(FRONT_LEFT, -0.05, 0.0, 0.0, GAINS) < 0.0


def test_pitch_correction_is_antisymmetric_front_to_back():
    front = stance_offset(FRONT_LEFT, 0.0, 0.0, 0.1, GAINS)
    hind = stance_offset(HIND_RIGHT, 0.0, 0.0, 0.1, GAINS)
    assert front == pytest.approx(-hind, abs=1e-9)
    assert front != pytest.approx(0.0)


def test_roll_correction_is_antisymmetric_left_to_right():
    left = stance_offset(FRONT_LEFT, 0.0, 0.1, 0.0, GAINS)
    right = stance_offset((0.3, -0.17, -0.43), 0.0, 0.1, 0.0, GAINS)
    assert left == pytest.approx(-right, abs=1e-9)
    assert left != pytest.approx(0.0)


def test_correction_holds_the_foot_at_constant_world_height():
    # For small angles a foot at (x, y) sits at world height z + roll*y -
    # pitch*x, so the correction must cancel exactly that term.
    gains = PostureGains(height=1.0, roll=1.0, pitch=1.0)
    roll, pitch = 0.05, 0.03
    dz = stance_offset(FRONT_LEFT, 0.0, roll, pitch, gains)
    assert dz == pytest.approx(pitch * FRONT_LEFT[0] - roll * FRONT_LEFT[1])


def test_offset_is_clamped_to_the_configured_limit():
    huge = stance_offset(FRONT_LEFT, 10.0, 0.0, 0.0, GAINS)
    assert huge == pytest.approx(GAINS.max_offset)
    assert stance_offset(FRONT_LEFT, -10.0, 0.0, 0.0, GAINS) == \
        pytest.approx(-GAINS.max_offset)
