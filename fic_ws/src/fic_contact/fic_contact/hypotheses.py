"""Estimate which link a contact acted on, from a joint torque residual.

A contact on link ``k`` can only load joints proximal to it: the contact
Jacobian has zero columns for every joint distal to ``k``. So ``tau_ext``
is structurally zero on joints ``k+1..n``, and the contacted link is the
most distal joint carrying significant torque.
"""

import numpy as np


def support_link(tau_ext, thresholds):
    """Return ``(link_index, confidence)`` from the residual's support.

    ``link_index`` has three regimes: ``-1`` means no statement is possible
    because the input is corrupt (a non-finite ``tau_ext`` or ``thresholds``,
    e.g. from a dropped filter frame); ``0`` means no joint exceeded its
    threshold, i.e. no contact; ``1..n`` is the 1-based index of the most
    distal joint carrying significant torque, i.e. the contacted link.
    Corrupt data must never be indistinguishable from a genuine "no
    contact", so it gets its own code rather than silently reading as 0.

    Confidence combines how far the deciding joint sits above its own
    threshold with how quiet the joints distal to it are — a distal joint
    close to firing means the boundary could be one link further out.
    Confidence is always 0.0 when ``link_index`` is 0 or -1.
    """
    tau_ext = np.asarray(tau_ext, dtype=float)
    thresholds = np.asarray(thresholds, dtype=float)
    if tau_ext.shape != thresholds.shape:
        raise ValueError(
            'tau_ext %s and thresholds %s must have the same shape'
            % (tau_ext.shape, thresholds.shape))
    if not (np.isfinite(tau_ext).all() and np.isfinite(thresholds).all()):
        return -1, 0.0

    ratio = np.abs(tau_ext) / np.maximum(thresholds, 1e-12)
    over = np.nonzero(ratio > 1.0)[0]
    if over.size == 0:
        return 0, 0.0

    k = int(over[-1]) + 1
    margin = ratio[k - 1] - 1.0
    # Squash an unbounded margin into [0, 1).
    strength = margin / (1.0 + margin)
    # Penalise by the loudest distal joint: at ratio 1 it would have decided
    # the answer itself, so the link boundary is maximally uncertain.
    distal = float(np.max(ratio[k:])) if k < ratio.size else 0.0
    confidence = float(np.clip(strength, 0.0, 1.0)
                       * (1.0 - np.clip(distal, 0.0, 1.0)))
    return k, confidence


def derate_confidence_for_motion(confidence, max_abs_dq, dq_threshold):
    """Multiplicatively derate confidence for the current joint speed.

    ``derated = confidence / (1 + max_abs_dq / dq_threshold)``

    At rest (``max_abs_dq`` at or below ``dq_threshold``) this is close to
    ``confidence`` unchanged; it falls monotonically, and without a jump,
    as the arm moves faster, reaching half at ``max_abs_dq ==
    dq_threshold`` and continuing to shrink beyond that.

    This exists because the residual a moving-mode estimate is built on
    models ``C(q,dq)*dq + g(q)`` but never the inertial term
    ``M(q)*ddq`` — no measured joint acceleration exists anywhere in this
    project to feed it. A moving-mode estimate is therefore always
    missing a term a stationary one is not, and this derating makes that
    gap visible in the published confidence instead of hiding it.
    """
    max_abs_dq = max(float(max_abs_dq), 0.0)
    return confidence / (1.0 + max_abs_dq / float(dq_threshold))


class Hypothesis:
    """One candidate contact: a link, a surface point and a force.

    Only the force's LINE OF ACTION is determined by the torque residual:
    with ``n`` torque sensors, every point along that line produces the
    same ``tau_ext``, so ``point`` is one estimate on that line, picked
    because it also lies on the candidate surface — not a uniquely
    determined contact location. See ``fit_contact``.
    """

    def __init__(self, link, point, force, residual):
        """Store the fitted contact hypothesis."""
        self.link = link
        self.point = point
        self.force = force
        self.residual = residual


def solve_push_force(jac_point, tau_ext, normal):
    """Least-squares contact force constrained to push into the surface.

    Solves ``min ||tau_ext - J^T f||`` subject to ``f . n <= 0`` — the
    force must point into the surface (or lie tangent to it), never pull.

    With a single inequality the optimum is either the unconstrained
    solution (when it already satisfies the constraint) or lies on the
    boundary ``f . n = 0``. On the boundary, parametrising ``f = B y``
    with ``B`` an orthonormal basis of the plane reduces it to another
    unconstrained least squares. No QP solver is needed.
    """
    mat = np.asarray(jac_point, dtype=float).T
    tau_ext = np.asarray(tau_ext, dtype=float)
    normal = np.asarray(normal, dtype=float)

    force, *_ = np.linalg.lstsq(mat, tau_ext, rcond=None)
    if force @ normal <= 0.0:
        return force, float(np.linalg.norm(tau_ext - mat @ force))

    basis = np.linalg.svd(normal.reshape(1, 3))[2][1:]
    coeffs, *_ = np.linalg.lstsq(mat @ basis.T, tau_ext, rcond=None)
    force = basis.T @ coeffs
    return force, float(np.linalg.norm(tau_ext - mat @ force))


def _point_jacobians(jac, points):
    """Shift a body-origin Jacobian to every candidate point.

    ``v_p = v_o + w x r`` gives ``J_p = J_lin - skew(r) @ J_ang``.
    """
    lin, ang = jac[:3], jac[3:]
    out = np.empty((points.shape[0], 3, jac.shape[1]))
    for i, r in enumerate(points):
        skew = np.array([[0.0, -r[2], r[1]],
                         [r[2], 0.0, -r[0]],
                         [-r[1], r[0], 0.0]])
        out[i] = lin - skew @ ang
    return out


#: Nm. Default OFF. The tolerance below which two hypotheses count as
#: explaining the data equally well must scale with the actual sensor
#: noise, not be a constant: on noiseless synthetic data an adjacent link
#: fits within any fixed constant of the true one, so a constant default
#: would turn the distal bias into a proximal one. Callers that know
#: their noise floor -- contact_node, which holds the calibration
#: thresholds -- pass a scaled value; everyone else gets exact
#: minimum-residual selection unchanged.
PARSIMONY_TOL_NM = 0.0


def choose_parsimonious(hypotheses, tol_nm=PARSIMONY_TOL_NM):
    """Pick the most proximal hypothesis among the statistically tied.

    A contact on link ``k`` is explained exactly by link ``k`` and very
    nearly by link ``k+1``, because link ``k+1``'s Jacobian spans link
    ``k``'s columns plus one more. Choosing on minimum residual alone
    therefore lets sensor noise award the answer to a link one or more
    joints too far out -- measured at 28.5% of detected cases on the
    MuJoCo sweep, with a tail out to +6.

    Among hypotheses within ``tol_nm`` of the best residual, the most
    proximal one explains the data using strictly fewer joints in
    support, so it is the answer that does not assume more than the
    measurement can show. A decisively better distal fit still wins.

    Returns None for an empty set.
    """
    best = None
    for hyp in hypotheses:
        if best is None or hyp.residual < best.residual:
            best = hyp
    if best is None:
        return None
    limit = best.residual + float(tol_nm)
    tied = [h for h in hypotheses if h.residual <= limit]
    return min(tied, key=lambda h: h.link)


def fit_contact(tau_ext, link_jacobians, candidates, f_max=200.0,
                min_norm=1e-6, parsimony_tol_nm=PARSIMONY_TOL_NM):
    """Best push-consistent contact hypothesis, or None if there is none.

    Sweeps every link's decimated surface candidates (see
    ``geometry.load_candidates``), solving the push-constrained force fit
    at each one, and keeps the lowest-residual hypothesis whose force
    magnitude is within ``f_max`` newtons. Returns None outright when
    ``tau_ext`` itself is indistinguishable from zero — a quiet residual
    must not be fitted to a phantom contact.

    IMPORTANT LIMIT: with ``n`` torque sensors, a contact's point is only
    determined up to the force's line of action — every point on that
    line produces identical joint torques. This search only ever sees
    the decimated candidate points, so the winner is the best-fitting
    candidate along (or near) that line, not a uniquely located contact.
    Treat ``Hypothesis.point`` as an estimate on the line of action, not
    a certainty.
    """
    tau_ext = np.asarray(tau_ext, dtype=float)
    if np.linalg.norm(tau_ext) < min_norm:
        return None

    # Best hypothesis PER LINK, not one global best: the parsimony
    # choice below needs to compare links against each other, which it
    # cannot do if a single winner has already been picked.
    per_link = {}
    for link, jac in link_jacobians.items():
        cand = candidates.get(link)
        if cand is None:
            continue
        jac_points = _point_jacobians(jac, cand.points)
        for i in range(cand.points.shape[0]):
            force, residual = solve_push_force(
                jac_points[i], tau_ext, cand.normals[i])
            if np.linalg.norm(force) > f_max:
                continue
            current = per_link.get(link)
            if current is None or residual < current.residual:
                per_link[link] = Hypothesis(
                    link, cand.points[i], force, residual)
    return choose_parsimonious(list(per_link.values()),
                               tol_nm=parsimony_tol_nm)


def derate_confidence_for_pose_drift(
        confidence, drift, pose_tolerance, relaxed_tolerance):
    """Multiplicatively derate confidence for a pose-drifted calibration.

    No penalty while ``drift`` is within the calibration's own
    ``pose_tolerance`` — the bias is still fresh. Between that and
    ``relaxed_tolerance`` — the window where a stationary-mode gate would
    already refuse as stale but a moving mode deliberately lets through,
    because the arm leaves its calibration pose almost immediately once
    it starts moving — confidence falls linearly to half, so a stale
    bias never reports the same confidence a fresh one would. The
    tradeoff: the noise floor under a stale bias is higher than the
    calibration measured, so small contacts near threshold can be
    missed. ``relaxed_tolerance <= pose_tolerance`` disables the penalty.
    """
    if drift <= pose_tolerance:
        return confidence
    span = float(relaxed_tolerance) - float(pose_tolerance)
    if span <= 0.0:
        return confidence
    frac = min(max((drift - pose_tolerance) / span, 0.0), 1.0)
    return confidence * (1.0 - 0.5 * frac)


class LinkDebouncer:
    """Temporal hysteresis on the reported contact link.

    Three separate evidence costs, because the three transitions are not
    equally likely. Detecting a contact from idle should be quick; giving
    it up should be slow; and *moving* an established contact to a
    different link should be slowest of all, because a hand resting in
    one place is the common case and the estimate wandering between
    neighbouring links is the common failure.

    Observed on hardware with a single threshold for entry and switching:
    the highlight jumped between links while a hand stayed still, because
    changing link cost the same five frames as finding one.
    """

    def __init__(self, enter_frames=5, exit_frames=15, switch_frames=40):
        """Configure the entry, exit and switch frame counts."""
        self._enter = int(enter_frames)
        self._exit = int(exit_frames)
        self._switch = max(int(switch_frames), int(enter_frames))
        self._reported = 0
        self._pending = 0
        self._count = 0

    @property
    def reported(self):
        """Return the link currently being held."""
        return self._reported

    @property
    def pending_count(self):
        """Return frames the challenger has held, for diagnostics."""
        return self._count

    def reset(self):
        """Drop the latch, e.g. on recalibration."""
        self._reported = 0
        self._pending = 0
        self._count = 0

    def _needed(self, link):
        """Frames of evidence required for ``link`` to take over."""
        if link == 0:
            return self._exit
        if self._reported == 0:
            return self._enter
        # Displacing an established link is the expensive transition.
        return self._switch

    def update(self, link):
        """Feed one raw link index; return the debounced one."""
        if link == self._reported:
            self._pending = link
            self._count = 0
            return self._reported

        if link != self._pending:
            self._pending = link
            self._count = 0
        self._count += 1

        if self._count >= self._needed(link):
            self._reported = link
            self._pending = link
            self._count = 0
        return self._reported

        if link != self._pending:
            self._pending = link
            self._count = 0
        self._count += 1

        needed = self._exit if link == 0 else self._enter
        if self._count >= needed:
            self._reported = link
            self._count = 0
        return self._reported
