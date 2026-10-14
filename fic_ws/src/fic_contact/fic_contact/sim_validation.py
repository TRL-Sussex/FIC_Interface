"""Accuracy-versus-injected-noise sweep for support_link (v1) and fit_contact (v2).

No hardware time is spent on this: it synthesises the joint-torque residual a
real push would produce and scores both estimators against it. Method, for
each noise level ``sigma`` and each link:

1. Draw a random arm pose ``q`` within a representative joint range.
2. Draw a random contact point ON THE LINK'S FULL MESH SURFACE -- every
   stored vertex, not the decimated set ``load_candidates`` hands to
   ``fit_contact``. Pushing exactly at a candidate point is a best case that
   never happens on a real robot: a real hand lands anywhere on the surface,
   and v2's own search only ever sees the decimated candidates, so an
   off-candidate push is the honest test of what it can recover. See
   ``_load_full_mesh_points``.
3. Choose a push direction with ``F . n < 0`` (into the surface) at a FIXED
   representative magnitude (``--force-n``, default 20 N -- the same order
   of magnitude used throughout this package's own tests). The magnitude is
   deliberately NOT scaled up per link: Shoulder_Link and HalfArm1_Link are
   known, from a direct force-needed measurement, to require far more force
   than a hand can supply (~69 N and ~24 N respectively, against ~3-4 N for
   the distal links) to clear the detection threshold at all. Using one
   fixed, human-plausible magnitude for every link is what lets this sweep
   report that ceiling honestly instead of hiding it behind a per-link force
   that was tuned to make every link detectable.
4. Synthesise ``tau = J_p(q)^T F`` with the point-Jacobian shift
   ``J_p = J_lin - skew(r) @ J_ang``, add ``N(0, sigma)`` noise per joint,
   and score both estimators against the true link.

v1 (``support_link``) is scored against a fixed per-joint threshold of
1.2 Nm -- the same figure this package's own tests
(``test_survives_realistic_measurement_noise``) already use as the
representative calibrated threshold, so this sweep's sigma=0 column is
directly comparable to prior measurements rather than introducing a new,
un-cross-checked constant.

The same set of (pose, point, direction) scenarios is reused across every
noise level for a given link -- only the injected noise differs -- so the
curve reflects noise sensitivity alone, not a change in trial difficulty
from one sigma to the next, and the sigma=0 row is the exact noise-free
figure this sweep is meant to reproduce.

Run (from ``fic_ws/src/fic_contact``, matching how this package's own tests
are invoked -- see the module-level path comment below):

    python3 -m fic_contact.sim_validation --trials 200 --out /tmp/accuracy_vs_noise.csv

``--trials`` controls trials per (link, sigma level); the default (60) is
kept modest because ``fit_contact`` is the cost driver, at ~32 ms/call with
the default 200-candidate/link density (measured; see
``contact_node.py``'s ``n_per_link`` comment) -- a full 11-level sweep over
7 links costs roughly ``11 * 7 * trials * 0.032`` s for v2 alone. --trials
200 (the plan's example) is close to 9 minutes; the default 60 is close to
2.5 minutes and is what reproduced the sigma=0 baseline used to write this
module. Both are printed at the end of a real run.
"""

import argparse
import csv
import os
import time

import numpy as np

import matplotlib
matplotlib.use('Agg')  # must precede importing pyplot -- Qt crashes in this project

import matplotlib.pyplot as plt  # noqa: E402

from fic_contact.geometry import load_candidates  # noqa: E402
from fic_contact.hypotheses import fit_contact, support_link  # noqa: E402
from fic_contact.residual import ArmModel  # noqa: E402

#: This file lives at fic_ws/src/fic_contact/fic_contact/sim_validation.py --
#: the same depth under the package root as test/test_hypotheses.py -- so
#: the identical '..', '..' relative walk to fic_ws/src/menagerie applies.
#: Matches the existing _URDF/_MJCF convention in test/test_hypotheses.py.
_HERE = os.path.dirname(__file__)
_URDF = os.path.join(
    _HERE, '..', '..', 'menagerie', 'model', 'KinovaGen3_7DOF', 'URDF',
    'KinovaGen3_7DOF.urdf')
_MJCF = os.path.join(
    _HERE, '..', '..', 'menagerie', 'model', 'KinovaGen3_7DOF', 'xml', 'scene.xml')
_JOINTS = ['Actuator%d' % i for i in range(1, 8)]

#: Sampling range per joint, rad. Joints 1/3/5/7 ("twist" joints) carry no
#: <limit lower/upper> in the URDF (see KinovaGen3_7DOF.urdf), so a
#: representative operating range is used instead of the (absent) model
#: bound; joints 2/4/6 use their real URDF limits.
_JOINT_RANGES = (
    (-3.0, 3.0), (-2.24, 2.24), (-3.0, 3.0), (-2.57, 2.57),
    (-3.0, 3.0), (-2.09, 2.09), (-3.0, 3.0),
)

#: v1's fixed per-joint detection threshold; see the module docstring.
DETECT_THRESHOLD_NM = 1.2
#: Fixed push-force magnitude, N; see the module docstring, step 3.
PUSH_FORCE_N = 20.0
#: v2's own candidate density per link -- matches geometry.load_candidates's
#: default and the density the sigma=0 baseline this module reproduces was
#: measured at.
N_CANDIDATES = 200
#: Default noise sweep: 0.0 to 2.0 Nm in steps of 0.2, so 1.2 Nm -- the
#: plan's own working detection-threshold figure -- lands exactly on a row.
SIGMA_MAX_NM = 2.0
SIGMA_STEP_NM = 0.2


def _random_pose(rng):
    """Return one uniformly random joint configuration within _JOINT_RANGES."""
    return np.array([rng.uniform(lo, hi) for lo, hi in _JOINT_RANGES])


def _push_direction(rng, normal):
    """Return a random unit direction with ``direction . normal <= 0``.

    Sampling uniformly on the sphere and reflecting the outward half back
    in gives a direction uniform over the inward hemisphere -- any push
    consistent with ``fit_contact``'s own ``F . n <= 0`` constraint, not
    just the pure normal.
    """
    direction = rng.normal(size=3)
    direction /= np.linalg.norm(direction)
    if direction @ normal > 0.0:
        direction = -direction
    return direction


def _point_jacobian(jac, point):
    """Shift a body-origin 6xn Jacobian to a point via v_p = v_o + w x r."""
    lin, ang = jac[:3], jac[3:]
    skew = np.array([[0.0, -point[2], point[1]],
                     [point[2], 0.0, -point[0]],
                     [-point[1], point[0], 0.0]])
    return lin - skew @ ang


def _load_full_mesh_points(mjcf_path, link_names):
    """Return ``{link: Candidates}`` sampled from EVERY mesh vertex, undecimated.

    ``load_candidates`` decimates to ``n_per_link`` via a uniform stride
    once a geom has more vertices than the budget (see
    ``geometry._sample_geom``). Passing an ``n_per_link`` far larger than
    any geom's vertex count disables that stride entirely, so every stored
    vertex survives -- the link's full surface, not v2's own decimated
    search set. This must never be the set ``fit_contact`` searches; see
    the module docstring for why that distinction is the whole point.
    """
    return load_candidates(mjcf_path, link_names, n_per_link=1_000_000)


def _sample_scenario(model, full_mesh, link_idx, rng, force_n):
    """Draw one noise-free (tau, link_jacobians) trial for ``link_idx``.

    Returns ``(tau_clean, jacs)``: the clean residual a ``force_n`` newton
    push at a random full-mesh point on ``link_idx``, in a random pose,
    would produce, and that pose's full per-link Jacobian dict (needed by
    ``fit_contact``, which searches every link, not just the true one).
    """
    q = _random_pose(rng)
    jacs = model.link_jacobians(q)
    cand = full_mesh[link_idx]
    idx = int(rng.integers(cand.points.shape[0]))
    point, normal = cand.points[idx], cand.normals[idx]
    direction = _push_direction(rng, normal)
    jac_point = _point_jacobian(jacs[link_idx], point)
    tau_clean = jac_point.T @ (force_n * direction)
    return tau_clean, jacs


def sweep(trials, n_candidates=N_CANDIDATES, seed=0, sigma_levels=None,
          force_n=PUSH_FORCE_N, threshold_nm=DETECT_THRESHOLD_NM):
    """Run the full accuracy-vs-noise sweep and return a list of CSV rows.

    Each row is ``{sigma_nm, estimator, link, trials, correct}``, ordered
    by ascending sigma and then by link (proximal to distal). The same
    ``trials`` scenarios are reused for every sigma level at a given link
    (see the module docstring), so ``correct`` at ``sigma_nm == 0.0`` is
    the exact noise-free classification rate for that link.
    """
    if sigma_levels is None:
        sigma_levels = np.round(
            np.arange(0.0, SIGMA_MAX_NM + 1e-9, SIGMA_STEP_NM), 6)

    model = ArmModel(_URDF, _JOINTS)
    full_mesh = _load_full_mesh_points(_MJCF, model.link_names)
    candidates = load_candidates(_MJCF, model.link_names, n_per_link=n_candidates)
    rng = np.random.default_rng(seed)
    thresholds = np.full(model.n, threshold_nm)

    scenarios = {
        link_idx: [_sample_scenario(model, full_mesh, link_idx, rng, force_n)
                   for _ in range(trials)]
        for link_idx in range(1, model.n + 1)
    }

    rows = []
    for sigma in sigma_levels:
        sigma = float(sigma)
        for link_idx, link_name in enumerate(model.link_names, start=1):
            v1_hits = 0
            v2_hits = 0
            for tau_clean, jacs in scenarios[link_idx]:
                tau = tau_clean if sigma == 0.0 else (
                    tau_clean + rng.normal(0.0, sigma, tau_clean.shape[0]))

                v1_link, _ = support_link(tau, thresholds)
                if v1_link == link_idx:
                    v1_hits += 1

                hyp = fit_contact(tau, jacs, candidates)
                if hyp is not None and hyp.link == link_idx:
                    v2_hits += 1

            rows.append({'sigma_nm': round(sigma, 4), 'estimator': 'v1',
                         'link': link_name, 'trials': trials, 'correct': v1_hits})
            rows.append({'sigma_nm': round(sigma, 4), 'estimator': 'v2',
                         'link': link_name, 'trials': trials, 'correct': v2_hits})
    return rows


def write_csv(rows, path):
    """Write the sweep rows as accuracy_vs_noise.csv."""
    fieldnames = ['sigma_nm', 'estimator', 'link', 'trials', 'correct']
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _print_summary(rows, link_names, elapsed):
    """Print a link-by-sigma accuracy table, in percent, to stdout."""
    sigmas = sorted({row['sigma_nm'] for row in rows})
    by_key = {(r['sigma_nm'], r['estimator'], r['link']): r for r in rows}

    print('\nLinks:', ', '.join(
        'L%d=%s' % (i, n) for i, n in enumerate(link_names, start=1)))
    header = 'sigma_nm  est  ' + '  '.join('L%d' % i for i in range(1, 8))
    print(header)
    print('-' * len(header))
    for sigma in sigmas:
        for estimator in ('v1', 'v2'):
            cells = []
            for name in link_names:
                r = by_key[(sigma, estimator, name)]
                cells.append('%5.1f%%' % (100.0 * r['correct'] / r['trials']))
            print('%8.2f  %3s  %s' % (sigma, estimator, ' '.join(cells)))
    print('\nSweep runtime: %.1f s' % elapsed)


def _plot_summary(rows, link_names, fig_path):
    """Save a small accuracy-vs-sigma figure (Agg backend) next to the CSV."""
    sigmas = sorted({row['sigma_nm'] for row in rows})
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
    for ax, estimator, title in zip(
            axes, ('v1', 'v2'), ('v1: support_link', 'v2: fit_contact')):
        for name in link_names:
            ys = [100.0 * r['correct'] / r['trials'] for r in rows
                  if r['estimator'] == estimator and r['link'] == name]
            ax.plot(sigmas, ys, marker='o', markersize=3, label=name)
        ax.set_title(title)
        ax.set_xlabel('Injected torque noise sigma (N*m)')
        ax.set_ylim(-5, 105)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('Accuracy (%)')
    axes[1].legend(loc='center left', bbox_to_anchor=(1.0, 0.5), fontsize=8)
    fig.suptitle('Accuracy vs injected noise, off-candidate mesh contacts')
    fig.tight_layout()
    fig.savefig(fig_path, dpi=150, bbox_inches='tight')
    plt.close(fig)


_DESCRIPTION = (
    'Accuracy-vs-noise sweep for support_link (v1) and fit_contact (v2), '
    'scored off-candidate on the full mesh.')


def _parse_args(argv=None):
    """Parse CLI arguments for the sweep."""
    parser = argparse.ArgumentParser(description=_DESCRIPTION)
    parser.add_argument(
        '--trials', type=int, default=60,
        help='trials per (link, sigma level) (default: 60)')
    parser.add_argument(
        '--out', default='accuracy_vs_noise.csv',
        help='CSV output path (default: accuracy_vs_noise.csv)')
    parser.add_argument(
        '--n-candidates', type=int, default=N_CANDIDATES,
        help="v2's candidate density per link (default: 200)")
    parser.add_argument(
        '--force-n', type=float, default=PUSH_FORCE_N,
        help='fixed push force magnitude, N (default: 20.0)')
    parser.add_argument(
        '--threshold-nm', type=float, default=DETECT_THRESHOLD_NM,
        help="v1's fixed per-joint threshold, Nm (default: 1.2)")
    parser.add_argument(
        '--sigma-max', type=float, default=SIGMA_MAX_NM,
        help='max injected noise sigma, Nm (default: 2.0)')
    parser.add_argument(
        '--sigma-step', type=float, default=SIGMA_STEP_NM,
        help='sigma sweep step, Nm (default: 0.2)')
    parser.add_argument('--seed', type=int, default=0, help='RNG seed (default: 0)')
    return parser.parse_args(argv)


def main(argv=None):
    """Run the sweep, write the CSV and figure, and print a summary."""
    args = _parse_args(argv)
    sigma_levels = np.round(
        np.arange(0.0, args.sigma_max + 1e-9, args.sigma_step), 6)

    start = time.time()
    rows = sweep(
        args.trials, n_candidates=args.n_candidates, seed=args.seed,
        sigma_levels=sigma_levels, force_n=args.force_n,
        threshold_nm=args.threshold_nm)
    elapsed = time.time() - start

    write_csv(rows, args.out)
    link_names = list(dict.fromkeys(r['link'] for r in rows))
    fig_path = os.path.splitext(args.out)[0] + '.png'
    _plot_summary(rows, link_names, fig_path)

    print('Wrote %s (%d rows) and %s' % (args.out, len(rows), fig_path))
    _print_summary(rows, link_names, elapsed)


if __name__ == '__main__':
    main()
