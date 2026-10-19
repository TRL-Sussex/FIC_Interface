"""Which joint is yielding when the arm is under admittance control.

STATUS: ``fic_contact.contact_node`` no longer calls
``parse_trajectory_blocks`` / ``admitting_joints`` below — it now
subscribes directly to ``{rid}_admittance_offset``, the controller's own
accumulated per-joint human-push offset (``fic::ficControl::humanOffset``,
published by ``fic_control``/``fic_control`` only in modes 3/4), and
feeds that straight into ``admitting_link``. That topic did not exist
when this module was written, and everything below explains why the
``{rid}_trajectory_data``-derived quantity it computes is a flawed stand-in
for it (planner lag, not yield). The two functions are kept, and still
correct, for anyone parsing a raw ``trajectory_data`` recording offline;
``admitting_link`` and ``combine_link_estimates`` are the parts still on
the live path.

``{rid}_trajectory_data`` (see ``fic_api/fic_api/trajectory.py`` for the
canonical parser and the authoritative block map) carries, for the flat
joint-mode layout (control modes 1-4), 8 blocks of ``N`` floats: block 0
planner target, 1 ``q_current``, 2 error, 3 ``q_target``, 4 control
effort, 5 ``q_target_adm``, 6 desired effort, 7 measured effort. This
module reads blocks 3 and 5 to build a second contact observable, next
to the joint-torque residual already used for the impedance modes (see
``residual.py`` / ``hypotheses.py``): the admittance offset,
``q_target_adm - q_target``. A contact on link ``k`` can only load
joints ``1..k`` (the same triangular argument ``hypotheses.py``'s module
docstring makes for the torque residual), so ``admitting_link`` below
*is* ``hypotheses.support_link`` with the residual replaced by the
offset, sharing its exact algorithm and return contract.

WHAT BLOCK 3 ACTUALLY HOLDS IN THE ADMITTANCE MODES — READ THIS FIRST.
The feature request that led to this module assumed block 3 was "the
hub's original commanded angle" in every mode, offset by admittance in
block 5. That is true in the plain impedance modes (1, 2): the hub's own
``q_target`` is passed straight into ``handleJointImpedanceControl``,
which writes it into block 3 verbatim (``fic_control.h:433-435``). It is
NOT true in the admittance modes (3, 4). ``handleJointAdmitanceControl``
(``fic_control.h:470-567``) never publishes the hub's original target —
its by-reference ``q_target`` parameter is only ever read (line 521,
``wrapToPi(q_target)``), never written to ``traj_msg``. Step 8 of that
function instead feeds the ADMITTANCE-ADJUSTED target back through the
impedance/planner plumbing (``handleJointImpedanceControlWithPlanner``,
``fic_control.h:558-560``), and *that* plumbing is what fills block 3.
So, in modes 3/4:

    block 5 (q_target_adm) = the raw admittance-adjusted target
    block 3 (q_target)     = plan(block 5), or clamp(block 5) if the
                              robot config disables the planner — i.e.
                              a filtered COPY of block 5, not the
                              pre-push command.

Consequently ``q_target_adm - q_target`` is not "how far this joint has
drifted from where the hub told it to go" — that original value never
reaches this message. It is the trajectory planner's own tracking lag on
the admittance target. The Kinova config's planner (``Planner`` in
``FIC.cpp``) is a fast rate-limited filter, so this lag sits near zero
whenever the admittance target is holding still and grows while it is
actively ramping. Read that as "is this joint's target *currently*
moving because someone is pushing it" — a fair, live answer to "which
joint is admitting" — not as a permanent record of an old push.
``admitting_joints`` below is written and documented against this
corrected reading.

THE 2.0 Nm DEADBAND GATES BOTH OBSERVABLES, NOT JUST THE TORQUE ONE.
``handleJointAdmitanceControl`` zeroes its ``ext_tau`` before it ever
reaches the admittance integrator whenever ``|ext_tau| <
admittance_deadbands.joint_tau`` (2.0 Nm on the Kinova, see
``robots_config.yaml``) — every control tick, unconditionally. A push
that never clears 2.0 Nm on any tick produces exactly zero admittance
offset; it is not "too small for one sample but visible once
integrated" — it never reaches the integrator at all, by the same
construction that leaves the torque residual under its own noise floor.
What the offset genuinely adds over one fresh residual sample is
memory: once ``ext_tau`` clears 2.0 Nm on even a single tick, mode 1
(Push-and-Stay) keeps that nudge permanently — it is a running integral
(the ``offest`` accumulator in ``fic_control.h``) — where an
instantaneous residual reading forgets it the very next tick. So the
offset is better described as "does not forget a force it once saw"
than "sees forces the residual cannot".

BLOCKS 6/7 ARE DELIBERATELY NOT USED HERE FOR THE TORQUE OBSERVABLE.
``fic_control.cpp`` calls the admittance controller with
``current_effort_ = -joint_torques_[i]`` (note the minus —
``fic_control.cpp:1315``), so block 7 is ``-tau_meas``, not
``tau_meas``. Block 6 is ``DesiredEffort = gravity_torques_[i] = g(q)``
(the Kinova is not gravity-compensated). So ``block6 - block7 == g(q) -
(-tau_meas) == g(q) + tau_meas`` — a SUM, not the ``g(q) - tau_meas``
difference this module's originating spec assumed. A quantity that does
not cancel to ~0 at rest is not a usable zero-mean residual, and
reusing it here would likely bias or invert every downstream link
decision. ``fic_contact.contact_node`` therefore computes the torque
observable the same way it already does for the impedance modes — via
``ResidualEstimator.residual()`` against the independent
``{rid}_joint_torques`` / ``{rid}_q_current`` topics — in every
admittance mode too, rather than reconstructing it from these blocks.

CARTESIAN ADMITTANCE (MODES 7, 8) HAS NO PER-JOINT OFFSET BLOCK AT ALL.
``cartesianimpedancecontrol`` (called at the end of the Cartesian
admittance chain) only ever writes ``traj_msg[i*8 + 0]`` and
``traj_msg[i*8 + 1]`` per joint (``fic_control.h``, around line 709) —
blocks 3 and 5 in the flat 8*N sense are simply left at 0.
``fic_api/fic_api/trajectory.py`` already encodes this: its
``is_cartesian`` branch hardcodes ``q_target`` and ``q_target_adm`` to
all zero. ``parse_trajectory_blocks`` below is only valid for the flat
joint-mode layout (modes 1-4); callers must not use it for modes 7/8,
and ``fic_contact.contact_node`` does not — it falls back to the torque
residual alone there, a single-observable and therefore weaker estimate.
"""

import numpy as np

from fic_contact.hypotheses import support_link

#: Number of stacked N-joint blocks in the flat joint-mode layout of
#: `{rid}_trajectory_data` (control modes 1-4 only). See the module
#: docstring and `fic_api/fic_api/trajectory.py` for the block map.
_N_BLOCKS = 8

#: Block order, matching `fic_api/fic_api/trajectory.py`'s field names
#: exactly so the two parsers never disagree about what a block means.
_BLOCK_NAMES = (
    'planner_target', 'q_current', 'error', 'q_target',
    'control_effort', 'q_target_adm', 'desired_effort', 'measured_effort',
)

#: Disagreement penalty applied by `combine_link_estimates` on top of
#: taking the lower of the two confidences, so a disagreement always
#: reads as strictly less trustworthy than either observable's own
#: agreement case — never merely "as confident as the weaker one alone".
_DISAGREEMENT_PENALTY = 0.5


def parse_trajectory_blocks(data, n_joints):
    """Split a flat joint-mode `{rid}_trajectory_data` array into blocks.

    ``data`` must have exactly ``8 * n_joints`` entries — the flat
    layout used by joint modes 1-4 only (see the module docstring for
    why Cartesian modes 7/8 cannot use this parser). Raises
    ``ValueError`` naming the expected and actual lengths otherwise, so
    a caller that points this at a Cartesian-mode message gets a clear
    failure instead of a silently wrong parse.

    Returns a dict keyed by the same names
    ``fic_api/fic_api/trajectory.py`` uses (``q_target``,
    ``q_target_adm``, ``desired_effort``, ``measured_effort``, ...),
    each an ``n_joints``-length ``np.ndarray``.
    """
    n = int(n_joints)
    arr = np.asarray(data, dtype=float)
    expected = _N_BLOCKS * n
    if arr.shape != (expected,):
        raise ValueError(
            'trajectory_data must have %d entries (8 blocks of %d '
            'joints) for the flat joint-mode layout, got %d'
            % (expected, n, arr.size))
    return {name: arr[k * n:(k + 1) * n]
            for k, name in enumerate(_BLOCK_NAMES)}


def admitting_joints(q_target, q_target_adm, threshold_rad):
    """Return each joint's signed admittance offset, in radians.

    ``q_target_adm - q_target`` — see the module docstring's "WHAT
    BLOCK 3 ACTUALLY HOLDS" section for exactly what that difference
    means in the admittance modes (a live tracking-lag signal, not a
    persistent record of an old push). Any entry smaller in magnitude
    than ``threshold_rad`` is floored to exactly 0.0: the raw
    difference carries a small amount of planner-settling residue even
    absent any push, and a caller feeding this into ``admitting_link``
    with the same threshold should not have to re-derive that floor.
    Non-finite entries pass through unchanged, so ``admitting_link``
    can still report "no statement" on them.
    """
    q_target = np.asarray(q_target, dtype=float)
    q_target_adm = np.asarray(q_target_adm, dtype=float)
    if q_target.shape != q_target_adm.shape:
        raise ValueError(
            'q_target %s and q_target_adm %s must have the same shape'
            % (q_target.shape, q_target_adm.shape))
    offset = q_target_adm - q_target
    return np.where(np.abs(offset) < float(threshold_rad), 0.0, offset)


def admitting_link(offsets, threshold_rad):
    """Return ``(link_index, confidence)`` from the admittance offsets.

    Same contract as ``hypotheses.support_link``, because this *is*
    ``support_link`` with the joint-torque residual replaced by the
    admittance offset and a single scalar threshold broadcast to every
    joint: ``-1`` no statement (a non-finite offset), ``0`` no joint is
    admitting, ``1..n`` the most distal admitting joint. The same
    triangular argument (a contact on link k loads only joints 1..k)
    applies just as well to "whose target is being pushed" as it does
    to "whose torque is elevated". Delegating to ``support_link`` also
    guarantees the two confidences share one shape and can be compared
    or combined directly — see ``combine_link_estimates``.
    """
    offsets = np.asarray(offsets, dtype=float)
    thresholds = np.full(offsets.shape, float(threshold_rad))
    return support_link(offsets, thresholds)


def combine_link_estimates(link_a, confidence_a, link_b, confidence_b):
    """Combine two independent ``(link_index, confidence)`` verdicts.

    Both inputs use the ``support_link`` / ``admitting_link`` contract
    (``-1`` / ``0`` / ``1..n``). When one side has no statement (``-1``
    — e.g. the offset side in Cartesian modes 7/8, which has no offset
    block to draw on, or a corrupt sample), the other side is returned
    unchanged: a missing observable is not a disagreement. When both
    sides have a statement and agree — including both saying "0,
    nobody is admitting" — that cross-check is a strong result, so the
    returned confidence is the *larger* of the two: agreement must
    never look weaker than either observable alone. When they disagree,
    this does not silently trust whichever one happens to be passed
    first: it reports the link from whichever side is more confident,
    but the confidence itself drops to half the *smaller* of the two —
    always below both single-observable confidences, and always below
    what an agreement would have reported.
    """
    if link_a == -1 and link_b == -1:
        return -1, 0.0
    if link_a == -1:
        return link_b, confidence_b
    if link_b == -1:
        return link_a, confidence_a
    if link_a == link_b:
        return link_a, max(confidence_a, confidence_b)
    link = link_a if confidence_a >= confidence_b else link_b
    return link, _DISAGREEMENT_PENALTY * min(confidence_a, confidence_b)
