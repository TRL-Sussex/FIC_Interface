"""Trot scheduling: diagonal pairing, phase wrapping, stance/swing split."""
import pytest

from fic_locomotion import trot_scheduler as ts


def test_diagonal_pairs_move_together():
    # A trot is defined by FL+HR swinging while FR+HL support, and vice versa.
    assert ts.LEG_PHASE_OFFSET['fl'] == ts.LEG_PHASE_OFFSET['hr']
    assert ts.LEG_PHASE_OFFSET['fr'] == ts.LEG_PHASE_OFFSET['hl']
    assert abs(ts.LEG_PHASE_OFFSET['fl'] - ts.LEG_PHASE_OFFSET['fr']) == 0.5


def test_advance_wraps_at_one():
    assert ts.advance(0.9, 0.05, 0.25) == pytest.approx(0.1)
    assert 0.0 <= ts.advance(0.99, 10.0, 0.36) < 1.0


def test_stance_then_swing_within_a_cycle():
    params = ts.GaitParams()
    early = ts.leg_phase(0.0, 'fl', params.duty_factor)
    late = ts.leg_phase(0.9, 'fl', params.duty_factor)
    assert early.stance is True
    assert early.progress == pytest.approx(0.0)
    assert late.stance is False


def test_progress_spans_zero_to_one_in_each_sub_phase():
    duty = 0.6
    assert ts.leg_phase(0.0, 'fl', duty).progress == pytest.approx(0.0)
    assert ts.leg_phase(0.599, 'fl', duty).progress == pytest.approx(0.9983, abs=1e-3)
    assert ts.leg_phase(0.6, 'fl', duty).progress == pytest.approx(0.0)
    assert ts.leg_phase(0.999, 'fl', duty).progress == pytest.approx(0.9975, abs=1e-3)


def test_at_least_two_legs_support_at_all_times():
    # With duty >= 0.5 the diagonals overlap, so the robot is never airborne.
    params = ts.GaitParams()
    for step in range(100):
        phase = step / 100.0
        supporting = sum(
            1 for leg in ('fl', 'fr', 'hl', 'hr')
            if ts.leg_phase(phase, leg, params.duty_factor).stance)
        assert supporting >= 2


def test_durations_split_the_cycle():
    params = ts.GaitParams(cycle_time=0.4, duty_factor=0.6)
    assert ts.stance_duration(params) == pytest.approx(0.24)
    assert ts.swing_duration(params) == pytest.approx(0.16)
