"""Measure contact-localization accuracy against the MuJoCo rig.

Sweeps contacts over links, poses, force directions and noise levels,
scoring what the estimator reports against what was actually applied.
Torques come from `simrig.ContactSim` -- a different engine from the
Pinocchio model being scored -- so the numbers reflect real model error
rather than a self-consistency check.

The calibration policy is a first-class variable, because it is the one
that exposed the gravity bug on hardware:

* ``per-pose``  -- re-calibrate the bias at each test pose, contact-free.
  The optimistic case, and roughly what an auto-calibrating arm achieves.
* ``reference`` -- calibrate once at a reference pose and test everywhere
  else. What a real session does between manual calibrations, and the
  case a pose-dependent model error destroys.

Run it:

    python3 -m fic_contact.evaluate --robot KinovaGen3_7DOF
    python3 -m fic_contact.evaluate --no-tool-payload   # the old behaviour
"""

import argparse
import json

import numpy as np

from fic_contact.deflection import (
    KINOVA_STIFFNESS_NM_PER_RAD, deflection_sigma, fuse_inverse_variance,
    tau_ext_from_deflection)
from fic_contact.geometry import load_candidates
from fic_contact.hypotheses import fit_contact, support_link
from fic_contact.residual import (
    ArmModel, ResidualEstimator, tool_payload_from_mjcf)
from fic_contact.simrig import ContactSim

#: Poses spanning the workspace. Fixed rather than random so a published
#: table is regenerable without carrying a seed for the pose set too.
POSES = (
    (0.0, 0.60, 0.0, -1.20, 0.0, 0.90, 0.0),
    (0.4, 0.30, 0.5, -1.80, 0.0, 1.20, 0.3),
    (-0.5, 0.90, -0.4, -0.80, 0.3, 0.60, -0.2),
    (0.0, -0.40, 0.0, -2.20, 0.0, 1.40, 0.0),
    (0.8, 0.50, 1.0, -1.50, -0.5, 0.80, 0.5),
)

#: Unit force directions, chosen to span sign as well as axis: the
#: hardware failure was directional, so a sweep that only pushed one way
#: would have scored it as healthy.
DIRECTIONS = (
    (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
    (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
    (0.577, 0.577, 0.577), (-0.577, -0.577, -0.577),
)


def build(mjcf, urdf, joints, tool_payload=True, sigma_k=6.0,
          n_per_link=60):
    """Return ``(sim, model, est, candidates)`` as contact_node wires them.

    The ArmModel and the candidate set are returned alongside the
    estimator rather than reached for through it: ResidualEstimator keeps
    its model private, and a scoring harness should not be the thing that
    breaks that boundary.
    """
    sim = ContactSim(mjcf, joints)
    payload = None
    if tool_payload:
        mass, com = tool_payload_from_mjcf(mjcf, joints)
        if mass > 0.0:
            payload = (mass, com)
    model = ArmModel(urdf, joints, tool_payload=payload)
    est = ResidualEstimator(model, torque_sign=1.0,
                            gravity_compensated=False, sigma_k=sigma_k)
    candidates = load_candidates(mjcf, model.link_names,
                                 n_per_link=n_per_link)
    return sim, model, est, candidates


def calibrate_at(sim, est, pose, samples=200, noise_nm=0.0, rng=None):
    """Measure the bias at ``pose`` with no contact applied."""
    est.start_calibration()
    for _ in range(samples):
        tau = sim.measured_torque(pose, noise_nm=noise_nm, rng=rng)
        est.accumulate(np.asarray(pose, dtype=float), tau)
    return est.finish_calibration()


def _deflection_channel(sim, model, pose, tau_contact, stiffness,
                        encoder_sigma_rad, rng):
    """Return this pose's deflection-channel estimate of the contact.

    Derived from static equilibrium rather than assumed. The controller
    applies ``K (q_target - q_current) + g_model``; balancing that
    against true gravity and the contact gives

        K (q_current - q_target) = tau_contact - (g_true - g_model)

    so this channel carries the gravity model error with the OPPOSITE
    sign to the torque residual, which sees ``tau_contact + (g_true -
    g_model)``. That is the interesting part: averaging the two cancels
    the model error rather than merely averaging its noise.
    """
    gravity_error = sim.gravity(pose) - model.gravity(pose)
    deflection = (tau_contact - gravity_error) / stiffness
    if encoder_sigma_rad:
        deflection = deflection + rng.normal(
            0.0, encoder_sigma_rad, size=deflection.shape)
    q_target = np.zeros_like(deflection)
    return tau_ext_from_deflection(q_target, deflection, stiffness)


def run(mjcf, urdf, joints, tool_payload=True, calibration='per-pose',
        newtons=25.0, noise_nm=0.2, seed=0, sites=2, fit_points=True,
        parsimony_k=0.0, fuse_deflection=False,
        encoder_sigma_rad=1e-4, fusion='inverse-variance'):
    """Sweep contacts and score the estimate against ground truth.

    Returns a dict of aggregate metrics plus the per-case rows, so a
    caller can re-aggregate (per link, per direction) without re-running
    the sweep.
    """
    rng = np.random.default_rng(seed)
    sim, model, est, candidates = build(
        mjcf, urdf, joints, tool_payload=tool_payload)

    if calibration == 'reference':
        calibrate_at(sim, est, POSES[0], noise_nm=noise_nm, rng=rng)

    rows = []
    for pose in POSES:
        pose = np.asarray(pose, dtype=float)
        if calibration == 'per-pose':
            calibrate_at(sim, est, pose, noise_nm=noise_nm, rng=rng)
        thresholds = est.calibration.thresholds
        # Scale the parsimony tolerance by the measured noise floor the
        # thresholds already encode, so the rule tightens to nothing on
        # clean data and only bites where noise really can swap links.
        tol_nm = float(parsimony_k) * float(np.mean(thresholds))
        stiffness = np.asarray(KINOVA_STIFFNESS_NM_PER_RAD, dtype=float)
        sigma_defl = deflection_sigma(stiffness, encoder_sigma_rad)
        # The deflection channel needs its own bias, measured the same
        # way and at the same pose as the torque channel's: contact-free,
        # so what it captures is -gravity_error there.
        if fuse_deflection:
            cal_pose = POSES[0] if calibration == 'reference' else pose
            defl_bias = _deflection_channel(
                sim, model, np.asarray(cal_pose, dtype=float),
                np.zeros(sim.n), stiffness, 0.0, rng)

        # A contact-free tick at this pose: anything reported here is a
        # false positive, which matters more than a missed detection for
        # a behaviour that moves the arm at whoever it thinks touched it.
        quiet = est.residual(pose, sim.measured_torque(
            pose, noise_nm=noise_nm, rng=rng))
        quiet_link, _ = support_link(quiet, thresholds)

        for link in range(1, sim.n + 1):
            for site in range(sites):
                point = sim.surface_point(pose, link, index=site)
                for direction in DIRECTIONS:
                    force = np.asarray(direction, dtype=float) * newtons
                    tau = sim.measured_torque(
                        pose, link=link, point=point, force=force,
                        noise_nm=noise_nm, rng=rng)
                    tau_ext = est.residual(pose, tau)
                    if fuse_deflection:
                        tau_c = sim.contact_torque(pose, link, point, force)
                        tau_defl = _deflection_channel(
                            sim, model, pose, tau_c, stiffness,
                            encoder_sigma_rad, rng)
                        if tau_defl is not None and defl_bias is not None:
                            tau_defl = tau_defl - defl_bias
                            if fusion == 'equal':
                                # Equal weights are the only blend that
                                # cancels the shared gravity error, since
                                # the two channels carry it with opposite
                                # signs. It costs the SNR advantage of
                                # the quieter channel, so the two aims
                                # are in direct tension.
                                fused = 0.5 * (tau_ext + tau_defl)
                            else:
                                fused, _ = fuse_inverse_variance(
                                    [tau_ext, tau_defl],
                                    [noise_nm or 1e-6, sigma_defl])
                            if fused is not None:
                                tau_ext = fused
                    got, confidence = support_link(tau_ext, thresholds)
                    row = {
                        'link_true': link, 'link_v1': int(got),
                        'link_est': int(got),
                        'confidence': float(confidence),
                        'direction': list(direction),
                        'quiet_link': int(quiet_link),
                    }
                    if fit_points and got > 0:
                        row.update(_score_fit(
                            model, candidates, pose, point, force, tau_ext,
                            tol_nm=tol_nm))
                        # What contact_node actually publishes: _fit_v2
                        # replaces the support-pattern link with the
                        # fitted one whenever a push-consistent fit
                        # exists, precisely because the fit is not bound
                        # by the most-distal-over-threshold rule and can
                        # name a link the threshold test missed.
                        if row.get('fit'):
                            row['link_est'] = int(row['fit_link'])
                    rows.append(row)

    return _aggregate(rows)


def _score_fit(model, candidates, pose, point, force, tau_ext, tol_nm=0.0):
    """Score the v2 point/force fit against the applied contact.

    ``point_err_m`` is bounded below by the line-of-action limit: joint
    torques fix the force's line, not a unique point on it, so a fit that
    lands anywhere along that line is as correct as the sensor set
    permits. Read it as a localisation ceiling, not a tracking error.
    """
    jac = model.link_jacobians(pose)
    hyp = fit_contact(tau_ext, jac, candidates, parsimony_tol_nm=tol_nm)
    if hyp is None:
        return {'fit': False}
    position, rotation = model.link_poses(pose)[hyp.link]
    got_point = rotation @ hyp.point + position
    err = float(np.linalg.norm(np.asarray(got_point) - np.asarray(point)))
    unit_true = np.asarray(force) / np.linalg.norm(force)
    got_force = np.asarray(hyp.force)
    norm = float(np.linalg.norm(got_force))
    cos = float(unit_true @ (got_force / norm)) if norm > 0 else 0.0
    return {'fit': True, 'fit_link': int(hyp.link),
            'point_err_m': err, 'force_cos': cos}


def _aggregate(rows):
    """Reduce per-case rows to the numbers a table would report."""
    total = len(rows)
    correct = sum(1 for r in rows if r['link_est'] == r['link_true'])
    missed = sum(1 for r in rows if r['link_est'] == 0)
    nostate = sum(1 for r in rows if r['link_est'] == -1)
    wrong = total - correct - missed - nostate
    false_pos = sum(1 for r in rows if r['quiet_link'] > 0)

    per_dir = {}
    for r in rows:
        key = tuple(r['direction'])
        hit, seen = per_dir.get(key, (0, 0))
        per_dir[key] = (hit + (r['link_est'] == r['link_true']), seen + 1)
    per_link = {}
    for r in rows:
        key = r['link_true']
        hit, seen = per_link.get(key, (0, 0))
        per_link[key] = (hit + (r['link_est'] == r['link_true']), seen + 1)

    # Signed link error tells you WHICH way the method is wrong. A
    # contact near joint k's own axis has almost no lever arm about it,
    # so joint k stays under threshold and support_link's most-distal
    # rule names k-1: a systematic proximal bias, not random confusion.
    offsets = {}
    for r in rows:
        if r['link_est'] > 0:
            d = r['link_est'] - r['link_true']
            offsets[d] = offsets.get(d, 0) + 1

    v1_correct = sum(1 for r in rows
                     if r.get('link_v1') == r['link_true'])
    v1_offsets = {}
    for r in rows:
        if r.get('link_v1', 0) > 0:
            d = r['link_v1'] - r['link_true']
            v1_offsets[d] = v1_offsets.get(d, 0) + 1

    out = {
        'link_accuracy_v1': v1_correct / total if total else 0.0,
        'link_offset_histogram_v1': {
            str(k): v for k, v in sorted(v1_offsets.items())},
        'link_offset_histogram': {
            str(k): v for k, v in sorted(offsets.items())},
        'cases': total,
        'link_accuracy': correct / total if total else 0.0,
        'missed_rate': missed / total if total else 0.0,
        'no_statement_rate': nostate / total if total else 0.0,
        'wrong_link_rate': wrong / total if total else 0.0,
        'false_positive_ticks': false_pos,
        'accuracy_by_direction': {
            ','.join('%+.2f' % v for v in k): h / n
            for k, (h, n) in sorted(per_dir.items())},
        'accuracy_by_link': {
            str(k): h / n for k, (h, n) in sorted(per_link.items())},
        'rows': rows,
    }
    fits = [r for r in rows if r.get('fit')]
    if fits:
        out['fit_rate'] = len(fits) / max(
            1, sum(1 for r in rows if r['link_est'] > 0))
        out['point_err_median_m'] = float(np.median(
            [r['point_err_m'] for r in fits]))
        out['force_cos_median'] = float(np.median(
            [r['force_cos'] for r in fits]))
    return out


def _directional_spread(metrics):
    """Return max-min accuracy across force directions.

    The hardware symptom was "detected one way, not the other", so this
    single number is the one that would have caught it: a healthy
    estimator scores the same in every direction.
    """
    vals = list(metrics['accuracy_by_direction'].values())
    return (max(vals) - min(vals)) if vals else 0.0


def main(argv=None):
    """Run a sweep and print the table."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mjcf', required=True)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--joints', required=True,
                    help='comma-separated actuated joint names')
    ap.add_argument('--calibration', default='per-pose',
                    choices=('per-pose', 'reference'))
    ap.add_argument('--no-tool-payload', action='store_true')
    ap.add_argument('--newtons', type=float, default=25.0)
    ap.add_argument('--noise-nm', type=float, default=0.2)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--parsimony-k', type=float, default=0.0,
                    help='parsimony tolerance as a multiple of the mean '
                         'detection threshold; 0 disables it')
    ap.add_argument('--json', help='write full metrics here')
    args = ap.parse_args(argv)

    metrics = run(args.mjcf, args.urdf, args.joints.split(','),
                  tool_payload=not args.no_tool_payload,
                  calibration=args.calibration,
                  newtons=args.newtons, noise_nm=args.noise_nm,
                  seed=args.seed, parsimony_k=args.parsimony_k)

    print('cases              %d' % metrics['cases'])
    print('link accuracy v1   %.1f%%  (support pattern alone)'
          % (100 * metrics['link_accuracy_v1']))
    print('link accuracy      %.1f%%  (as published, v2-refined)'
          % (100 * metrics['link_accuracy']))
    print('missed (link 0)    %.1f%%' % (100 * metrics['missed_rate']))
    print('wrong link         %.1f%%' % (100 * metrics['wrong_link_rate']))
    print('directional spread %.1f%%  (0 = symmetric)'
          % (100 * _directional_spread(metrics)))
    hist = metrics['link_offset_histogram']
    seen = sum(hist.values()) or 1
    print('link offset (est - true), over detected cases:')
    for key in sorted(hist, key=int):
        print('   %+3s  %5.1f%%' % (key, 100 * hist[key] / seen))
    print('by direction:')
    for key, value in metrics['accuracy_by_direction'].items():
        print('   %-22s %.1f%%' % (key, 100 * value))
    if args.json:
        payload = dict(metrics)
        payload.pop('rows', None)
        with open(args.json, 'w') as handle:
            json.dump(payload, handle, indent=2)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())


# ══════════════════════════════════════════════════════════════════════════
#  The following experiment
# ══════════════════════════════════════════════════════════════════════════

def bearing_error(mjcf, urdf, joints, tool_payload=True, parsimony_k=0.25,
                  calibration='per-pose', newtons=25.0, noise_nm=0.2,
                  seed=0, sites=2):
    """Angular error of the recovered push bearing, in degrees.

    This is the quantity the `following` extension actually consumes. It
    aims and steps along ``-f_hat``, so what matters is not whether the
    link index is right but whether the recovered force direction points
    back at the person.

    The two are not the same question, and the difference is favourable:
    a fit that names the wrong link can still recover very nearly the
    right direction, because an adjacent link's candidate lies close to
    the true line of action. Link accuracy therefore understates how well
    `following` can work.

    Progress is geometric: a step along a bearing within 90 degrees of the
    true direction reduces the distance to the person, so
    ``progress_rate`` -- the fraction of accepted contacts with error
    below 90 degrees -- is the convergence condition for the behaviour.
    """
    rng = np.random.default_rng(seed)
    sim, model, est, candidates = build(
        mjcf, urdf, joints, tool_payload=tool_payload)
    if calibration == 'reference':
        calibrate_at(sim, est, POSES[0], noise_nm=noise_nm, rng=rng)

    errors, confidences = [], []
    for pose in POSES:
        pose = np.asarray(pose, dtype=float)
        if calibration == 'per-pose':
            calibrate_at(sim, est, pose, noise_nm=noise_nm, rng=rng)
        thresholds = est.calibration.thresholds
        tol_nm = float(parsimony_k) * float(np.mean(thresholds))

        for link in range(1, sim.n + 1):
            for site in range(sites):
                point = sim.surface_point(pose, link, index=site)
                for direction in DIRECTIONS:
                    truth = np.asarray(direction, dtype=float)
                    truth = truth / np.linalg.norm(truth)
                    force = truth * newtons
                    tau = sim.measured_torque(
                        pose, link=link, point=point, force=force,
                        noise_nm=noise_nm, rng=rng)
                    tau_ext = est.residual(pose, tau)
                    got, confidence = support_link(tau_ext, thresholds)
                    if got <= 0:
                        continue
                    hyp = fit_contact(tau_ext, model.link_jacobians(pose),
                                      candidates, parsimony_tol_nm=tol_nm)
                    if hyp is None:
                        continue
                    fitted = np.asarray(hyp.force, dtype=float)
                    norm = float(np.linalg.norm(fitted))
                    if norm <= 0.0:
                        continue
                    # The behaviour uses -f_hat for both, so comparing the
                    # forces directly gives the same angle as comparing
                    # the bearings derived from them.
                    cos = float(np.clip(truth @ (fitted / norm), -1.0, 1.0))
                    errors.append(float(np.degrees(np.arccos(cos))))
                    confidences.append(float(confidence))

    errors = np.asarray(errors, dtype=float)
    confidences = np.asarray(confidences, dtype=float)
    if not errors.size:
        return {'samples': 0}
    # Does the accept gate's confidence floor actually screen out the
    # bearings that point the wrong way? If it does, the gate is not just
    # conservative -- it is selecting for direction quality, which is the
    # property `following` depends on.
    gated = {}
    for floor in (0.0, 0.25, 0.5, 0.75):
        keep = errors[confidences >= floor]
        gated['%.2f' % floor] = {
            'kept_frac': float(keep.size / errors.size),
            'median_deg': float(np.median(keep)) if keep.size else None,
            'progress_rate': (float(np.mean(keep < 90.0))
                              if keep.size else None),
        }
    return {
        'samples': int(errors.size),
        'median_deg': float(np.median(errors)),
        'p90_deg': float(np.percentile(errors, 90)),
        'within_30_deg': float(np.mean(errors < 30.0)),
        'within_60_deg': float(np.mean(errors < 60.0)),
        'progress_rate': float(np.mean(errors < 90.0)),
        'by_confidence_floor': gated,
        'errors_deg': errors.tolist(),
        'confidences': confidences.tolist(),
    }
