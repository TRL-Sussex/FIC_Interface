"""Leg kinematics: FK/IK agreement and consistency with the Spot MJCF."""
import math

import pytest

from fic_locomotion import leg_kinematics as lk


def test_four_legs_in_mjcf_order():
    assert [leg.name for leg in lk.SPOT_LEGS] == ['fl', 'fr', 'hl', 'hr']
    assert lk.JOINT_NAMES[:3] == ('fl_hx', 'fl_hy', 'fl_kn')
    assert len(lk.JOINT_NAMES) == 12
    assert len(lk.JOINT_LIMITS) == 12


def test_forward_kinematics_matches_mjcf_home_pose():
    # The MJCF `home` keyframe puts every leg at (0, 1.04, -1.8) with the trunk
    # at z=0.46; the foot centre must then sit 0.4275 m below the hip.
    # Values cross-checked against the compiled MJCF: loading spot.xml in
    # MuJoCo, resetting to the `home` keyframe and reading the FL foot geom
    # gives exactly these coordinates.
    foot = lk.forward_kinematics(lk.SPOT_LEGS[0], lk.STAND_Q)
    assert foot[0] == pytest.approx(0.26635822, abs=1e-7)
    assert foot[1] == pytest.approx(0.1658, abs=1e-7)
    assert foot[2] == pytest.approx(-0.42745791, abs=1e-7)


def test_ik_inverts_the_home_pose_for_every_leg():
    for leg in lk.SPOT_LEGS:
        q = lk.inverse_kinematics(leg, lk.nominal_stance(leg))
        assert q == pytest.approx(lk.STAND_Q, abs=1e-9)


def test_ik_round_trips_forward_kinematics():
    for leg in lk.SPOT_LEGS:
        for q1 in (-0.3, 0.0, 0.4):
            for q2 in (0.5, 1.04, 1.6):
                for q3 in (-2.2, -1.8, -1.0):
                    foot = lk.forward_kinematics(leg, (q1, q2, q3))
                    assert lk.inverse_kinematics(leg, foot) == \
                        pytest.approx((q1, q2, q3), abs=1e-9)


def test_right_legs_mirror_left_legs():
    left = lk.nominal_stance(lk.SPOT_LEGS[0])
    right = lk.nominal_stance(lk.SPOT_LEGS[1])
    assert left[0] == pytest.approx(right[0])
    assert left[1] == pytest.approx(-right[1])
    assert left[2] == pytest.approx(right[2])


def test_unreachable_target_is_projected_not_raised():
    # A gait controller must always emit a command; clamping beats an
    # exception at 200 Hz.
    q = lk.inverse_kinematics(lk.SPOT_LEGS[0], (5.0, 5.0, -5.0))
    assert all(math.isfinite(a) for a in q)


def test_ik_output_is_not_limit_checked_callers_must_clamp():
    # A target only 1 mm below the hip is well inside |thigh - shank|, so the
    # analytic solve below still succeeds, but the knee bend it needs is far
    # outside JOINT_LIMITS. inverse_kinematics() does not enforce the limits
    # itself -- that is the caller's job, via clamp_to_limits().
    leg = lk.SPOT_LEGS[0]
    close_foot = (leg.hip_offset[0],
                  leg.hip_offset[1] + leg.side * leg.y_offset,
                  leg.hip_offset[2] - 0.001)
    q1, q2, q3 = lk.inverse_kinematics(leg, close_foot)

    hy_lo, hy_hi = lk.JOINT_LIMITS[1]
    kn_lo, kn_hi = lk.JOINT_LIMITS[2]
    assert not (hy_lo <= q2 <= hy_hi) or not (kn_lo <= q3 <= kn_hi)

    # clamp_to_limits() zips against the first three JOINT_LIMITS entries,
    # which are exactly this (front-left) leg's own limits.
    clamped_q1, clamped_q2, clamped_q3 = lk.clamp_to_limits((q1, q2, q3))
    assert hy_lo <= clamped_q2 <= hy_hi
    assert kn_lo <= clamped_q3 <= kn_hi


def test_ik_solves_only_the_below_hip_branch():
    # This documents a known limitation, not a bug to fix: inverse_kinematics
    # always returns the below-hip knee-bend branch. Feeding back a forward
    # kinematics pose whose foot lands at (or near) hip height exercises the
    # above-hip branch, which does not round-trip to the original angles --
    # even though the returned angles do land on the very same foot position.
    leg = lk.SPOT_LEGS[0]
    q_in = (-0.785398, -0.898845, -2.7929)
    foot = lk.forward_kinematics(leg, q_in)
    assert foot[2] == pytest.approx(0.0, abs=5e-3)  # foot is near hip height

    q_back = lk.inverse_kinematics(leg, foot)
    assert q_back != pytest.approx(q_in, abs=1e-6)

    # The mismatched angles are nonetheless a valid solution for that foot.
    assert lk.forward_kinematics(leg, q_back) == pytest.approx(foot, abs=1e-9)
