"""Tests for the contact-link hypothesis estimators."""

import os

import numpy as np
import pytest

from fic_contact.geometry import load_candidates
from fic_contact.hypotheses import (
    LinkDebouncer, derate_confidence_for_motion, derate_confidence_for_pose_drift,
    fit_contact, solve_push_force, support_link)
from fic_contact.residual import LINK_NAMES, ArmModel

_TH = np.full(7, 1.0)

_URDF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'KinovaGen3_7DOF', 'URDF', 'KinovaGen3_7DOF.urdf')
_JOINTS = ['Actuator%d' % i for i in range(1, 8)]
_BENT = np.array([0.0, 0.5, 0.0, 1.5, 0.0, 1.0, 0.0])

_MJCF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'KinovaGen3_7DOF', 'xml', 'scene.xml')


def test_no_contact_returns_zero():
    """A residual below every threshold is no contact."""
    link, conf = support_link(np.full(7, 0.1), _TH)
    assert link == 0
    assert conf == 0.0


def test_contact_on_link_four():
    """The most distal joint above threshold names the contacted link."""
    tau = np.array([5.0, 4.0, 3.0, 2.0, 0.0, 0.0, 0.0])
    link, conf = support_link(tau, _TH)
    assert link == 4
    assert conf > 0.0


def test_contact_on_the_last_link():
    """All seven joints loaded means contact at or beyond link 7."""
    link, _ = support_link(np.full(7, 5.0), _TH)
    assert link == 7


def test_contact_on_the_first_link():
    """Only joint 1 loaded means contact on the shoulder."""
    tau = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    link, _ = support_link(tau, _TH)
    assert link == 1


def test_sign_of_the_residual_is_irrelevant():
    """A pull and a push on the same link give the same link index."""
    tau = np.array([5.0, 4.0, 3.0, 2.0, 0.0, 0.0, 0.0])
    assert support_link(tau, _TH)[0] == support_link(-tau, _TH)[0]


def test_confidence_rises_with_margin():
    """A residual far above threshold is more confident than one just over."""
    weak = np.array([1.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    strong = np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert support_link(strong, _TH)[1] > support_link(weak, _TH)[1]


def test_confidence_falls_when_a_distal_joint_is_near_threshold():
    """An almost-firing distal joint means the link index is uncertain."""
    clean = np.array([5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0])
    murky = np.array([5.0, 5.0, 5.0, 5.0, 0.99, 0.0, 0.0])
    assert support_link(murky, _TH)[1] < support_link(clean, _TH)[1]


def test_per_joint_thresholds_are_honoured():
    """Each joint is compared against its own threshold, not a global one."""
    thresholds = np.array([10.0, 10.0, 10.0, 10.0, 0.5, 0.5, 0.5])
    tau = np.array([5.0, 5.0, 5.0, 5.0, 1.0, 0.0, 0.0])
    assert support_link(tau, thresholds)[0] == 5


def test_mismatched_lengths_are_rejected():
    """A threshold vector of the wrong length is a programming error."""
    with pytest.raises(ValueError):
        support_link(np.zeros(7), np.zeros(6))


def test_non_finite_tau_ext_returns_no_statement():
    """A NaN in the residual must not silently read as a legitimate link."""
    tau = np.array([5.0, 4.0, 3.0, 2.0, np.nan, 0.0, 0.0])
    assert support_link(tau, _TH) == (-1, 0.0)


def test_infinite_tau_ext_returns_no_statement():
    """An infinite residual is corrupt data, not an extremely confident hit."""
    tau = np.array([5.0, 4.0, 3.0, 2.0, np.inf, 0.0, 0.0])
    assert support_link(tau, _TH) == (-1, 0.0)


def test_non_finite_threshold_returns_no_statement():
    """A NaN threshold is a calibration fault, not a free pass to fire."""
    thresholds = np.array([1.0, 1.0, 1.0, 1.0, np.nan, 1.0, 1.0])
    tau = np.array([5.0, 4.0, 3.0, 2.0, 0.0, 0.0, 0.0])
    assert support_link(tau, thresholds) == (-1, 0.0)


def test_confidence_is_near_zero_at_the_threshold_boundary():
    """A residual barely over its threshold must carry almost no confidence."""
    tau = np.array([1.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    _, conf = support_link(tau, _TH)
    assert conf < 0.01


def _synth(model, link, force, offset=None):
    """Joint torque produced by a point force on ``link`` at ``offset``."""
    jac = model.link_jacobians(_BENT)[link]
    lin, ang = jac[:3], jac[3:]
    # Velocity of a point rigidly attached at r: v_p = v_o + w x r,
    # so J_p = J_lin - skew(r) @ J_ang.
    r = np.zeros(3) if offset is None else np.asarray(offset, dtype=float)
    skew = np.array([[0.0, -r[2], r[1]],
                     [r[2], 0.0, -r[0]],
                     [-r[1], r[0], 0.0]])
    jac_point = lin - skew @ ang
    return jac_point.T @ np.asarray(force, dtype=float)


def test_recovers_every_link_from_a_synthetic_push():
    """A ~20 N push on each link in turn must be classified correctly.

    The push direction and offset are chosen off-axis: at ``_BENT``, joints
    2, 4 and 6 all share a world-frame rotation axis (every odd "twist"
    joint sits at 0 rad, so the intervening "bend" joints never turn away
    from it). A push along that shared axis is structurally invisible to
    all three joints at once, so the direction here has components on
    every axis to keep each contact link identifiable.
    """
    model = ArmModel(_URDF, _JOINTS)
    thresholds = np.full(7, 0.05)
    offset = np.array([0.08, -0.08, 0.07])
    for link in range(1, 8):
        tau = _synth(model, link, np.array([-14.0, -13.0, 6.0]), offset)
        assert support_link(tau, thresholds)[0] == link


def test_survives_realistic_measurement_noise():
    """Classification must hold at the hardware noise floor for most trials."""
    model = ArmModel(_URDF, _JOINTS)
    rng = np.random.default_rng(1)
    thresholds = np.full(7, 1.2)
    offset = np.array([0.08, -0.08, 0.07])
    hits = 0
    trials = 200
    for i in range(trials):
        link = 1 + i % 7
        tau = _synth(model, link, np.array([-21.0, -20.0, 9.5]), offset)
        tau = tau + rng.normal(0.0, 0.2, 7)
        if support_link(tau, thresholds)[0] == link:
            hits += 1
    assert hits / trials > 0.7


def test_v1_fails_when_the_force_line_passes_near_a_joint_axis():
    """The documented v1 failure mode, asserted so v2 can be shown to fix it.

    A force whose line of action nearly intersects a joint axis produces
    almost no torque about that joint, so the support pattern reports a
    link that is too proximal.
    """
    model = ArmModel(_URDF, _JOINTS)
    jac = model.link_jacobians(_BENT)[5]
    # Choose a force in the null space of joint 5's torque row, so joint 5
    # reads ~0 despite the contact being on link 5. jac[:3] is 3x7 (linear
    # rows by joint columns); column 4 is the direction that generates
    # joint 5's torque, so a force orthogonal to it leaves tau[4] at zero.
    row = jac[:3][:, 4]
    basis = np.linalg.svd(row.reshape(1, 3))[2][1:]
    force = 25.0 * basis[0]
    tau = jac[:3].T @ force
    assert abs(tau[4]) < 0.05 * np.max(np.abs(tau))
    assert support_link(tau, np.full(7, 0.5))[0] < 5


def test_a_push_parallel_to_a_joint_axis_is_invisible():
    """A push along a joint's rotation axis is a real sensing limit, not a bug.

    A force whose line of action passes through a joint axis exerts no
    moment about it, and no estimator can recover what the sensors never
    saw. A 20 N +Y push on Shoulder_Link, applied at the point where the
    link's own frame origin sits on joint 1's rotation axis, produces zero
    torque on every joint — so ``support_link`` must report no contact
    rather than guess a link from a signal that was never there.
    """
    model = ArmModel(_URDF, _JOINTS)
    tau = _synth(model, 1, np.array([0.0, 20.0, 0.0]))
    assert np.allclose(tau, 0.0, atol=1e-6)
    assert support_link(tau, np.full(7, 0.05))[0] == 0


def test_derate_confidence_for_motion_is_a_no_op_at_rest():
    """Zero velocity must not touch the confidence at all."""
    assert derate_confidence_for_motion(0.8, 0.0, 0.02) == pytest.approx(0.8)


def test_derate_confidence_for_motion_falls_as_speed_rises():
    """Confidence at a real contact must be lower while moving than at rest."""
    at_rest = derate_confidence_for_motion(0.8, 0.0, 0.02)
    slow = derate_confidence_for_motion(0.8, 0.02, 0.02)
    fast = derate_confidence_for_motion(0.8, 0.5, 0.02)
    assert at_rest > slow > fast > 0.0


def test_derate_confidence_for_motion_halves_at_the_threshold():
    """At max|dq| == dq_threshold the penalty must be exactly one half."""
    assert derate_confidence_for_motion(1.0, 0.02, 0.02) == pytest.approx(0.5)


def test_derate_confidence_for_pose_drift_is_a_no_op_within_tolerance():
    """A calibration still within its own tolerance is not penalised."""
    got = derate_confidence_for_pose_drift(0.8, 0.03, 0.05, 0.5)
    assert got == pytest.approx(0.8)


def test_derate_confidence_for_pose_drift_falls_between_the_tolerances():
    """Drift beyond the strict tolerance costs confidence, up to half."""
    fresh = derate_confidence_for_pose_drift(0.8, 0.05, 0.05, 0.5)
    mid = derate_confidence_for_pose_drift(0.8, 0.275, 0.05, 0.5)
    stalest_allowed = derate_confidence_for_pose_drift(0.8, 0.5, 0.05, 0.5)
    assert fresh > mid > stalest_allowed
    assert stalest_allowed == pytest.approx(0.4)


def test_derate_confidence_for_pose_drift_disabled_when_tolerances_equal():
    """Equal tolerances must disable the penalty rather than divide by zero."""
    got = derate_confidence_for_pose_drift(0.8, 10.0, 0.05, 0.05)
    assert got == pytest.approx(0.8)


def test_unconstrained_solution_is_kept_when_it_is_already_a_push():
    """If the least-squares force already points inward, keep it."""
    jac = np.zeros((3, 7))
    jac[0, 0] = jac[1, 1] = jac[2, 2] = 1.0
    normal = np.array([0.0, 0.0, 1.0])
    tau = jac.T @ np.array([1.0, 2.0, -3.0])
    force, residual = solve_push_force(jac, tau, normal)
    assert np.allclose(force, [1.0, 2.0, -3.0], atol=1e-6)
    assert residual < 1e-6


def test_pulling_solution_is_projected_onto_the_surface_plane():
    """A force that would pull is replaced by the best inward-or-tangent fit."""
    jac = np.zeros((3, 7))
    jac[0, 0] = jac[1, 1] = jac[2, 2] = 1.0
    normal = np.array([0.0, 0.0, 1.0])
    tau = jac.T @ np.array([1.0, 2.0, +5.0])
    force, _ = solve_push_force(jac, tau, normal)
    assert force @ normal <= 1e-9


def test_v2_recovers_the_link_of_a_push_on_an_actual_candidate():
    """A push-consistent synthetic contact on a real candidate is recovered.

    An earlier version of this test synthesised tau from the
    link-5 BODY-ORIGIN Jacobian (``jacs[5][:3].T[:, 4]``), a point that
    ``fit_contact`` never searches -- it only sweeps the decimated
    surface candidates from ``load_candidates``. A link-6 or link-7
    candidate can fit that body-origin torque as well or better, so that
    version is not a reliable test of the fit. Drawing the synthetic
    point from an actual candidate, with a force chosen to push into the
    surface (``F.n < 0``, honouring the constraint ``fit_contact`` itself
    enforces), is both satisfiable and a more honest check that the fit
    recovers a real, push-consistent contact.
    """
    model = ArmModel(_URDF, _JOINTS)
    cands = load_candidates(_MJCF, LINK_NAMES, n_per_link=120)
    jacs = model.link_jacobians(_BENT)

    link = 5
    cand = cands[link]
    point, normal = cand.points[7], cand.normals[7]
    lin, ang = jacs[link][:3], jacs[link][3:]
    skew = np.array([[0.0, -point[2], point[1]],
                     [point[2], 0.0, -point[0]],
                     [-point[1], point[0], 0.0]])
    jac_point = lin - skew @ ang

    # Push inward along the negative normal, scaled up so the residual
    # this produces clears support_link's threshold comfortably.
    force = 25.0 * (-normal)
    assert force @ normal < 0.0
    tau = jac_point.T @ force

    best = fit_contact(tau, jacs, cands)
    assert best is not None
    assert best.link == link


def test_returns_none_when_there_is_no_contact():
    """A zero residual must not be fitted to a phantom contact."""
    model = ArmModel(_URDF, _JOINTS)
    cands = load_candidates(_MJCF, LINK_NAMES, n_per_link=60)
    jacs = model.link_jacobians(_BENT)
    assert fit_contact(np.zeros(7), jacs, cands, f_max=200.0) is None


def test_single_noisy_frame_is_rejected():
    """One stray detection must not produce a reported contact."""
    deb = LinkDebouncer(enter_frames=5, exit_frames=15)
    assert deb.update(4) == 0
    for _ in range(20):
        assert deb.update(0) == 0


def test_sustained_contact_is_reported():
    """Five consecutive agreeing frames commit to the link."""
    deb = LinkDebouncer(enter_frames=5, exit_frames=15)
    out = [deb.update(4) for _ in range(6)]
    assert out[:4] == [0, 0, 0, 0]
    assert out[4] == 4


def test_release_requires_more_frames_than_entry():
    """Contact is sticky: a brief dropout must not clear the highlight."""
    deb = LinkDebouncer(enter_frames=5, exit_frames=15)
    for _ in range(5):
        deb.update(4)
    for _ in range(14):
        assert deb.update(0) == 4
    assert deb.update(0) == 0


def test_switching_links_restarts_the_challenger_count():
    """A challenger must earn its frames, and an interruption resets them.

    Displacing an established link now costs ``switch_frames`` rather
    than ``enter_frames`` -- holding a contact still is the common case
    and the estimate wandering to a neighbour is the common failure -- so
    this pins the counting behaviour at whatever that cost is, not the
    cost itself.
    """
    deb = LinkDebouncer(enter_frames=5, exit_frames=15, switch_frames=5)
    for _ in range(5):
        deb.update(4)
    for _ in range(4):
        assert deb.update(2) == 4
    # A third link interrupting restarts the challenger's count.
    assert deb.update(6) == 4
    for _ in range(4):
        assert deb.update(2) == 4
    assert deb.update(2) == 2


def test_fit_prefers_the_proximal_link_when_residuals_tie():
    """Adjacent distal links fit a contact almost as well as the true one.

    A contact on link k is explained exactly by link k, and very nearly
    by link k+1 -- measured on the Kinova at one pose, 0.0000 Nm against
    0.0628 Nm. Selecting on minimum residual alone therefore lets noise
    hand the answer to the more distal link, which is exactly the +1
    bias the MuJoCo sweep exposed (28.5% of detected cases). Where two
    hypotheses are statistically indistinguishable, the more proximal one
    is the parsimonious answer: it explains the data with strictly fewer
    joints in support.
    """
    import numpy as np
    from fic_contact.hypotheses import Hypothesis, choose_parsimonious

    best = [
        Hypothesis(3, np.zeros(3), np.zeros(3), 0.0000),
        Hypothesis(4, np.zeros(3), np.zeros(3), 0.0628),
    ]
    assert choose_parsimonious(best, tol_nm=0.5).link == 3


def test_fit_keeps_a_clearly_better_distal_link():
    """Parsimony must not override a decisive residual difference."""
    import numpy as np
    from fic_contact.hypotheses import Hypothesis, choose_parsimonious

    best = [
        Hypothesis(2, np.zeros(3), np.zeros(3), 4.0),
        Hypothesis(5, np.zeros(3), np.zeros(3), 0.05),
    ]
    assert choose_parsimonious(best, tol_nm=0.5).link == 5


def test_parsimony_handles_an_empty_set():
    """No hypotheses means no answer, not a crash."""
    from fic_contact.hypotheses import choose_parsimonious
    assert choose_parsimonious([], tol_nm=0.5) is None


def test_debouncer_holds_a_latched_link_against_brief_competition():
    """Once committed to a link, a short burst elsewhere must not move it.

    Observed on hardware: the highlight jumps between links while a hand
    rests in one place, because switching link costs the same evidence as
    detecting one in the first place. Holding a contact still is the
    common case; changing which link is held is rare and should cost
    much more.
    """
    from fic_contact.hypotheses import LinkDebouncer

    d = LinkDebouncer(enter_frames=3, switch_frames=25, exit_frames=10)
    for _ in range(3):
        d.update(4)
    assert d.update(4) == 4

    # A competing link for well past enter_frames must not take over.
    for _ in range(10):
        assert d.update(6) == 4


def test_debouncer_still_switches_on_sustained_evidence():
    """A genuine move to another link must eventually be honoured."""
    from fic_contact.hypotheses import LinkDebouncer

    d = LinkDebouncer(enter_frames=3, switch_frames=8, exit_frames=10)
    for _ in range(4):
        d.update(4)
    for _ in range(7):
        assert d.update(6) == 4
    assert d.update(6) == 6


def test_debouncer_entry_from_idle_stays_responsive():
    """Sticking must not make first detection sluggish."""
    from fic_contact.hypotheses import LinkDebouncer

    d = LinkDebouncer(enter_frames=3, switch_frames=25, exit_frames=10)
    assert d.update(5) == 0
    assert d.update(5) == 0
    assert d.update(5) == 5


def test_debouncer_release_needs_sustained_absence():
    """Letting go must clear, but only after the contact really stops."""
    from fic_contact.hypotheses import LinkDebouncer

    d = LinkDebouncer(enter_frames=2, switch_frames=20, exit_frames=6)
    for _ in range(3):
        d.update(4)
    for _ in range(5):
        assert d.update(0) == 4
    assert d.update(0) == 0


def test_debouncer_dropout_during_contact_does_not_release():
    """A single quiet tick mid-contact must not blank the highlight."""
    from fic_contact.hypotheses import LinkDebouncer

    d = LinkDebouncer(enter_frames=2, switch_frames=20, exit_frames=8)
    for _ in range(3):
        d.update(4)
    assert d.update(0) == 4
    assert d.update(4) == 4
    assert d.update(0) == 4
