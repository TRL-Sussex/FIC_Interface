"""Swing trajectory: clean lift-off and touch-down, apex clearance."""
import pytest

from fic_locomotion.swing_trajectory import swing_foot

START = (0.0, 0.0, -0.4)
END = (0.1, 0.0, -0.4)


def test_endpoints_are_exact():
    assert swing_foot(START, END, 0.0, 0.08) == pytest.approx(START)
    assert swing_foot(START, END, 1.0, 0.08) == pytest.approx(END)


def test_apex_clears_the_ground_by_the_step_height():
    mid = swing_foot(START, END, 0.5, 0.08)
    assert mid[2] == pytest.approx(-0.4 + 0.08)
    assert mid[0] == pytest.approx(0.05)


def test_progress_is_clamped():
    assert swing_foot(START, END, -1.0, 0.08) == pytest.approx(START)
    assert swing_foot(START, END, 2.0, 0.08) == pytest.approx(END)


def test_horizontal_velocity_vanishes_at_both_ends():
    # A linear ramp would scuff the ground on touch-down and kick the trunk on
    # lift-off; the cycloid blend removes both.
    eps = 1e-4
    lift = swing_foot(START, END, eps, 0.08)[0] - swing_foot(START, END, 0.0, 0.08)[0]
    land = swing_foot(START, END, 1.0, 0.08)[0] - swing_foot(START, END, 1 - eps, 0.08)[0]
    mid = (swing_foot(START, END, 0.5 + eps, 0.08)[0]
           - swing_foot(START, END, 0.5, 0.08)[0])
    assert abs(lift) < abs(mid) / 100.0
    assert abs(land) < abs(mid) / 100.0


def test_vertical_endpoints_respect_differing_heights():
    # Stepping onto a different height must still start and end exactly.
    high = (0.1, 0.0, -0.3)
    assert swing_foot(START, high, 0.0, 0.08) == pytest.approx(START)
    assert swing_foot(START, high, 1.0, 0.08) == pytest.approx(high)
