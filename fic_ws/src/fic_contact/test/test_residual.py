"""Tests for the Pinocchio arm model and the torque residual."""

import os

import numpy as np
import pinocchio as pin
import pytest

from fic_contact.residual import LINK_NAMES, ArmModel, ResidualEstimator, derive_link_names

_URDF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'KinovaGen3_7DOF', 'URDF', 'KinovaGen3_7DOF.urdf')
_PENDULUM_URDF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'Pendulum', 'URDF', 'pendulum.urdf')
_FR3_URDF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'Franka_FR3', 'URDF', 'Franka_FR3.urdf')
_JOINTS = ['Actuator%d' % i for i in range(1, 8)]
_BENT = np.array([0.0, 0.5, 0.0, 1.5, 0.0, 1.0, 0.0])


@pytest.fixture(scope='module')
def model():
    """An ArmModel built from the shipped Kinova URDF."""
    return ArmModel(_URDF, _JOINTS)


def test_model_builds_despite_malformed_ee_link_inertial(model):
    """ee_link has a bad ixx; Pinocchio warns on stderr but still builds."""
    assert model.n == 7


def test_gravity_is_not_degenerate(model):
    """A bent pose must produce a substantial gravity torque vector."""
    g = model.gravity(_BENT)
    assert g.shape == (7,)
    assert np.linalg.norm(g) > 5.0


def test_gravity_is_near_zero_at_neutral(model):
    """Straight up, gravity produces almost no joint torque."""
    assert np.linalg.norm(model.gravity(np.zeros(7))) < 0.1


def test_link_jacobian_is_structurally_triangular(model):
    """Link k must have nonzero Jacobian columns exactly for joints 1..k."""
    jacs = model.link_jacobians(_BENT)
    assert sorted(jacs) == list(range(1, 8))
    for k in range(1, 8):
        nonzero = np.nonzero(np.abs(jacs[k]).sum(axis=0) > 1e-9)[0]
        assert list(nonzero) == list(range(k)), f'link {k} broke the pattern'


def test_link_poses_returns_all_links_with_orthonormal_rotations(model):
    """Every link's rotation must be a valid orthonormal frame."""
    poses = model.link_poses(_BENT)
    assert sorted(poses) == list(range(1, 8))
    for pos, rot in poses.values():
        assert pos.shape == (3,)
        assert rot.shape == (3, 3)
        assert np.allclose(rot.T @ rot, np.eye(3), atol=1e-8)
        assert np.isclose(np.linalg.det(rot), 1.0, atol=1e-8)


def test_link_poses_position_matches_the_jacobian_derivative(model):
    """d(position)/dq from link_poses must match link_jacobians' linear rows.

    Both are separate Pinocchio calls (link_jacobians uses
    computeJointJacobians, link_poses uses forwardKinematics), so this
    is a real cross-check that the two agree on the same base frame.
    """
    q0 = _BENT
    eps = 1e-6
    jacs = model.link_jacobians(q0)
    for k in (1, 4, 7):
        pos0, _ = model.link_poses(q0)[k]
        jac_lin = jacs[k][:3]
        for j in range(7):
            dq = np.zeros(7)
            dq[j] = eps
            pos1, _ = model.link_poses(q0 + dq)[k]
            numeric = (pos1 - pos0) / eps
            assert np.allclose(numeric, jac_lin[:, j], atol=1e-3)


def test_link_names_are_proximal_to_distal(model):
    """The link order must match the kinematic chain."""
    assert LINK_NAMES[0] == 'Shoulder_Link'
    assert LINK_NAMES[-1] == 'Bracelet_Link'
    assert len(LINK_NAMES) == 7


def test_unknown_joint_name_is_rejected(model):
    """A joint not in the URDF must fail loudly, not silently mis-map."""
    with pytest.raises(ValueError, match='not in URDF'):
        ArmModel(_URDF, ['Actuator1', 'Nope'])


def test_derive_link_names_reproduces_kinova_list():
    """derive_link_names must exactly reproduce the Kinova link names."""
    raw = pin.buildModelFromUrdf(_URDF)
    assert derive_link_names(raw, _JOINTS) == list(LINK_NAMES)


def test_derive_link_names_raises_when_no_body_frame_exists():
    """A joint with no BODY frame must fail loudly, not return None.

    ``hinge`` is a real joint on the Pendulum URDF and does derive a link;
    the bogus second name has no joint at all, so it has no BODY frame and
    must raise instead of silently landing a None in the returned list.
    """
    raw = pin.buildModelFromUrdf(_PENDULUM_URDF)
    with pytest.raises(ValueError, match='explicitly'):
        derive_link_names(raw, ['hinge', 'not-a-joint'])


def test_arm_model_exposes_derived_link_names(model):
    """ArmModel.link_names must be the derived tuple, not a private detail."""
    assert model.link_names == LINK_NAMES


def test_link_names_override_is_honoured():
    """An explicit link_names argument is stored verbatim, not re-derived.

    Reversing the real Kinova names makes this discriminating: if the
    override were ignored in favour of derivation, ``custom.link_names``
    would come back in forward order instead.
    """
    reversed_names = tuple(reversed(LINK_NAMES))
    custom = ArmModel(_URDF, _JOINTS, link_names=reversed_names)
    assert custom.link_names == reversed_names


def test_gravity_is_degenerate_is_false_for_kinova(model):
    """The real Kinova model must not be flagged as gravity-degenerate."""
    assert model.gravity_is_degenerate() is False


def test_gravity_is_degenerate_detects_a_massless_urdf():
    """Franka_FR3's URDF has zero <inertial> blocks, so g(q) is ~zero."""
    fr3 = ArmModel(_FR3_URDF, ['fr3_joint%d' % i for i in range(1, 8)])
    assert fr3.gravity_is_degenerate() is True


def _estimator(model, **kw):
    """A ResidualEstimator calibrated at the bent pose with zero noise."""
    est = ResidualEstimator(model, **kw)
    est.start_calibration()
    for _ in range(50):
        est.accumulate(_BENT, model.gravity(_BENT))
    est.finish_calibration()
    return est


def test_residual_is_zero_after_calibration(model):
    """With no contact and no noise, the residual must vanish."""
    est = _estimator(model)
    assert np.allclose(est.residual(_BENT, model.gravity(_BENT)), 0.0, atol=1e-9)


def test_residual_recovers_an_applied_torque(model):
    """An extra joint torque appears in the residual unchanged."""
    est = _estimator(model)
    extra = np.array([0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0])
    got = est.residual(_BENT, model.gravity(_BENT) + extra)
    assert np.allclose(got, extra, atol=1e-9)


def test_torque_sign_defaults_to_one(model):
    """torque_sign must default to 1.0 — fic_hardware already applied it."""
    assert ResidualEstimator(model).torque_sign == 1.0


def test_torque_sign_negates_correctly(model):
    """torque_sign=-1.0 must actually flip the measured torque's sign.

    Calibrating with negated gravity and then querying with a negated
    applied torque only cancels out if ``torque_sign`` is really multiplied
    in; a stub that ignores the parameter (e.g. hardcoding ``1.0 *``) would
    recover ``-extra`` instead of ``extra`` below.
    """
    est = ResidualEstimator(model, torque_sign=-1.0)
    est.start_calibration()
    for _ in range(50):
        est.accumulate(_BENT, -model.gravity(_BENT))
    est.finish_calibration()
    extra = np.array([0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0])
    got = est.residual(_BENT, -(model.gravity(_BENT) + extra))
    assert np.allclose(got, extra, atol=1e-9)


def test_gravity_compensated_skips_the_gravity_term(model):
    """When the driver already compensates gravity, do not subtract it.

    Calibrating and querying at DIFFERENT poses makes this discriminating:
    if gravity were wrongly subtracted (or wrongly not-subtracted), g at the
    calibration pose would not match g at the query pose and the residual
    below would pick up that mismatch instead of vanishing.
    """
    calib_q = _BENT
    query_q = _BENT + 0.3
    est = ResidualEstimator(model, gravity_compensated=True)
    est.start_calibration()
    for _ in range(10):
        est.accumulate(calib_q, np.zeros(7))
    est.finish_calibration()
    assert np.allclose(est.residual(query_q, np.zeros(7)), 0.0, atol=1e-9)


def test_thresholds_scale_with_measured_noise(model):
    """Thresholds are sigma_k standard deviations of the calibration window."""
    rng = np.random.default_rng(0)
    est = ResidualEstimator(model, sigma_k=6.0)
    est.start_calibration()
    for _ in range(4000):
        est.accumulate(_BENT, model.gravity(_BENT) + rng.normal(0, 0.2, 7))
    cal = est.finish_calibration()
    assert np.allclose(cal.sigma, 0.2, rtol=0.15)
    assert np.allclose(cal.thresholds, 6.0 * cal.sigma)


def test_thresholds_have_a_floor_for_a_noiseless_calibration(model):
    """A zero-noise calibration must not yield a zero detection threshold."""
    est = _estimator(model)
    assert np.all(est.calibration.thresholds > 0.0)


def test_uncalibrated_estimator_reports_so(model):
    """Before calibration the estimator must not claim to be usable."""
    assert ResidualEstimator(model).calibrated is False


def test_calibration_goes_stale_when_the_arm_moves(model):
    """A bias measured at one pose is not valid at another."""
    est = _estimator(model, pose_tolerance=0.05)
    assert est.pose_drift(_BENT) == pytest.approx(0.0, abs=1e-9)
    assert est.is_stale(_BENT) is False
    assert est.is_stale(_BENT + 0.5) is True


def test_is_stale_pose_tolerance_override(model):
    """A per-call pose_tolerance overrides the estimator's own default."""
    est = _estimator(model, pose_tolerance=0.05)
    drifted = _BENT + 0.3
    assert est.is_stale(drifted) is True
    assert est.is_stale(drifted, pose_tolerance=0.5) is False


def test_pose_tolerance_property_matches_constructor(model):
    """The pose_tolerance property must expose the constructor's value."""
    est = ResidualEstimator(model, pose_tolerance=0.2)
    assert est.pose_tolerance == pytest.approx(0.2)


_DQ = np.array([0.0, 0.3, 0.0, 0.5, 0.0, 0.2, 0.0])


def test_nonlinear_at_zero_velocity_equals_gravity(model):
    """nonLinearEffects(q, 0) must equal computeGeneralizedGravity(q)."""
    got = model.nonlinear(_BENT, np.zeros(7))
    assert np.allclose(got, model.gravity(_BENT), atol=1e-9)


def test_nonlinear_shape_is_one_torque_per_joint(model):
    """nonlinear() must return one torque per actuated joint, ROS order."""
    tau = model.nonlinear(_BENT, _DQ)
    assert tau.shape == (7,)


def test_residual_with_dq_none_matches_the_old_gravity_only_path(model):
    """residual(q, tau, dq=None) must be bit-identical to the old call."""
    est = _estimator(model)
    old_style = est.residual(_BENT, model.gravity(_BENT))
    explicit_none = est.residual(_BENT, model.gravity(_BENT), dq=None)
    assert np.array_equal(old_style, explicit_none)


def test_residual_with_all_zero_dq_matches_the_old_gravity_only_path(model):
    """An explicit all-zero dq must not perturb the gravity-only residual."""
    est = _estimator(model)
    old_style = est.residual(_BENT, model.gravity(_BENT))
    zeroed = est.residual(_BENT, model.gravity(_BENT), dq=np.zeros(7))
    assert np.array_equal(old_style, zeroed)


def test_accumulate_with_dq_none_matches_the_old_gravity_only_path(model):
    """accumulate(q, tau, dq=None) must be bit-identical to the old call."""
    est_old = ResidualEstimator(model)
    est_old.start_calibration()
    for _ in range(10):
        est_old.accumulate(_BENT, model.gravity(_BENT))
    cal_old = est_old.finish_calibration()

    est_new = ResidualEstimator(model)
    est_new.start_calibration()
    for _ in range(10):
        est_new.accumulate(_BENT, model.gravity(_BENT), dq=None)
    cal_new = est_new.finish_calibration()

    assert np.array_equal(cal_old.bias, cal_new.bias)


def test_nonzero_dq_changes_the_residual(model):
    """A non-zero dq must route through the Coriolis-aware nonlinear term."""
    est = _estimator(model)
    at_rest = est.residual(_BENT, model.gravity(_BENT))
    moving = est.residual(_BENT, model.gravity(_BENT), dq=_DQ)
    assert not np.allclose(at_rest, moving)


def test_moving_residual_recovers_an_applied_torque_over_the_coriolis_term(model):
    """With dq, the residual is measured torque minus C(q,dq)*dq + g(q).

    Calibrating with the moving-mode model term and then querying the same
    way must recover an extra applied torque unchanged — the Coriolis term
    must be modelled, not merely nonzero.
    """
    est = ResidualEstimator(model)
    est.start_calibration()
    for _ in range(20):
        est.accumulate(_BENT, model.nonlinear(_BENT, _DQ), dq=_DQ)
    est.finish_calibration()

    extra = np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0])
    got = est.residual(_BENT, model.nonlinear(_BENT, _DQ) + extra, dq=_DQ)
    assert np.allclose(got, extra, atol=1e-9)


def test_pose_drift_is_measured_as_an_angle_not_a_number():
    """A joint crossing +-pi has not moved 2*pi, and must not read that way.

    Measured on hardware: q_current and q_target differed by 6.2805 rad --
    exactly 2*pi -- because a wrist joint sat on the branch cut. Compared
    linearly against a 0.05 rad tolerance, that latches ``is_stale``
    permanently true and the estimator goes silent for as long as the
    joint stays on the far side. Pushing the arm one way carries the
    joint across the cut and kills detection; pushing it the other way
    does not, which is exactly the direction-dependent behaviour reported
    from the robot.
    """
    import numpy as np
    from fic_contact.residual import angular_difference

    # Physically identical poses, represented either side of the cut.
    a = np.array([0.0, 0.0, np.pi - 1e-6, 0.0, 0.0, 0.0, 0.0])
    b = np.array([0.0, 0.0, -np.pi + 1e-6, 0.0, 0.0, 0.0, 0.0])
    assert np.abs(angular_difference(a, b)).max() < 1e-4

    # A genuine small motion still reads as a small motion.
    c = a.copy()
    c[1] += 0.03
    assert np.abs(angular_difference(a, c)).max() == pytest.approx(0.03,
                                                                  abs=1e-9)

    # A genuine large motion is not wrapped away into nothing.
    d = a.copy()
    d[1] += 2.0
    assert np.abs(angular_difference(a, d)).max() == pytest.approx(2.0,
                                                                  abs=1e-9)


def _window(est, model, sigma, rng, n=100, offset=None):
    """Run one calibration window at _BENT with noise ``sigma`` Nm."""
    est.start_calibration()
    extra = np.zeros(7) if offset is None else np.asarray(offset, dtype=float)
    for _ in range(n):
        est.accumulate(_BENT, model.gravity(_BENT) + extra
                       + rng.normal(0, sigma, 7))
    return est.finish_calibration()


def test_an_aborted_window_keeps_the_previous_calibration(model):
    """A disturbed window must not replace a clean bias."""
    est = _estimator(model)
    before = est.calibration
    est.start_calibration()
    est.accumulate(_BENT, model.gravity(_BENT) + 20.0)
    assert est.calibrating is True
    est.abort_calibration()
    assert est.calibrating is False
    assert est.calibration is before


def test_a_window_that_caught_a_push_is_discarded(model):
    """The hardware case: a push starting inside a window.

    Measured: one such window set thresholds of [13.1, 8.2, 6.3, 2.2, ...]
    Nm for ~28 s because the noise is RMS-pooled. With outlier rejection
    the window is thrown away and the clean bias and thresholds stay.
    """
    rng = np.random.default_rng(1)
    est = ResidualEstimator(model, sigma_pool=8, outlier_ratio=5.0)
    for _ in range(3):
        _window(est, model, 0.04, rng)
    clean = est.calibration
    est.start_calibration()
    for i in range(100):
        ramp = np.array([20.0, 8.0, 18.0, 1.5, 3.0, 0.3, 0.1]) * (i / 100.0)
        est.accumulate(_BENT, model.gravity(_BENT) + ramp
                       + rng.normal(0, 0.04, 7))
    assert est.finish_calibration() is None
    assert est.calibration is clean
    assert 'joint' in est.last_discard


def test_ordinary_noise_variation_is_not_discarded(model):
    """Per-second noise varies ~3x on a still arm; that must still pass."""
    rng = np.random.default_rng(2)
    est = ResidualEstimator(model, sigma_pool=8, outlier_ratio=5.0)
    for sigma in (0.03, 0.05, 0.02, 0.09, 0.04, 0.06):
        assert _window(est, model, sigma, rng) is not None
    assert est.last_discard is None


def test_outlier_rejection_is_off_by_default(model):
    """Other callers (evaluate, the analysis scripts) always get a Calibration."""
    rng = np.random.default_rng(3)
    est = ResidualEstimator(model, sigma_pool=8)
    _window(est, model, 0.04, rng)
    assert _window(est, model, 5.0, rng) is not None


def test_the_first_window_has_nothing_to_be_judged_against(model):
    rng = np.random.default_rng(4)
    est = ResidualEstimator(model, sigma_pool=8, outlier_ratio=5.0)
    assert _window(est, model, 3.0, rng) is not None


def test_a_genuine_rise_in_noise_is_believed_after_a_few_windows(model):
    """Discarding for ever would leave a bias that can never refresh."""
    from fic_contact.residual import OUTLIER_MAX_DISCARDS
    rng = np.random.default_rng(5)
    est = ResidualEstimator(model, sigma_pool=8, outlier_ratio=5.0)
    for _ in range(3):
        _window(est, model, 0.02, rng)
    results = [_window(est, model, 0.5, rng)
               for _ in range(OUTLIER_MAX_DISCARDS + 1)]
    assert results[:OUTLIER_MAX_DISCARDS] == [None] * OUTLIER_MAX_DISCARDS
    assert results[-1] is not None
    # ...and the new level is now the yardstick, not the old quiet one.
    assert _window(est, model, 0.5, rng) is not None


def test_gravity_does_not_depend_on_the_kinova_base_joint(model):
    """Its first axis is vertical: turning it changes no joint's load."""
    assert model.gravity_free_joints() == [0]


def test_turning_the_base_does_not_make_the_bias_stale(model):
    """What blocked the estimate after every Following turn on hardware."""
    est = ResidualEstimator(model, pose_tolerance=0.05,
                            ignore_joints=model.gravity_free_joints())
    est.start_calibration()
    for _ in range(20):
        est.accumulate(_BENT, model.gravity(_BENT))
    est.finish_calibration()
    turned = _BENT.copy()
    turned[0] += 1.2                     # a 69-degree turn of the base
    assert est.is_stale(turned) is False
    # ...and the bias really is still right there.
    assert np.allclose(est.residual(turned, model.gravity(turned)), 0.0,
                       atol=1e-9)
    bent = _BENT.copy()
    bent[1] += 0.1                       # the shoulder is another matter
    assert est.is_stale(bent) is True


def test_staleness_counts_every_joint_unless_told(model):
    est = _estimator(model, pose_tolerance=0.05)
    turned = _BENT.copy()
    turned[0] += 1.2
    assert est.is_stale(turned) is True
