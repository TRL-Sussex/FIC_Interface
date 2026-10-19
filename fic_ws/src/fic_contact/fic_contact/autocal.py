"""Decide when to re-measure the contact bias without an operator.

The bias `ResidualEstimator` subtracts is measured at one pose and drifts
as the arm leaves it, and it is lost outright when the node restarts --
which now happens on every hardware restart, since `contact_launch.py`
rides along with the arm. Re-measuring by hand each time is the thing
this removes.

The trigger is a proxy, and its one load-bearing assumption is stated
here because getting it wrong hides real contacts: **in a stiff mode, an
arm sitting on its commanded pose is not being touched.** A push
deflects a stiff joint off target, so "on target" implies untouched. In
an admittance mode that inverts -- the arm yields to the push and the
command follows the arm -- so the proxy would read "untouched" while
someone leans on it, and the calibration would absorb the push into the
bias and hide that contact permanently. Hence STIFF_MODES.

This module imports no ROS and no numpy: it is a predicate over numbers,
tested directly (see test/test_autocal.py).
"""

import math

#: Modes where the arm holds against contact rather than yielding to it,
#: so pose deviation is a usable proxy for being touched. The complement
#: of `gate._ADMITTANCE_MODES` (3, 4, 7, 8) over the eight control modes:
#: 1 Joint Impedance, 2 Joint Impedance + Planner, 5 Cartesian Impedance,
#: 6 Cartesian Planner.
STIFF_MODES = frozenset({1, 2, 5, 6})


def calibration_disturbed(q, q_start, moving, started_moving, tolerance):
    """Return True when an open calibration window has been disturbed.

    The trigger checks the arm is settled and untouched only when it OPENS
    a window; nothing checked it again while the window averaged. A push
    that starts inside the window is therefore absorbed: its mean into
    the bias, its variation into the noise. Measured on hardware, a
    window opened at rest caught the start of a push 0.5 s in and came
    out with thresholds of [13.1, 8.2, 6.3, 2.2, ...] Nm, where a clean
    window gives about [0.25, 0.5, 0.2, 0.2, ...]. Windows open every
    ~3.5 s and last 1 s, so about one push in three lands in one.

    So a window that opened at rest is disturbed the moment the arm moves
    or any joint leaves the pose it opened at by more than ``tolerance``
    (rad). A window that opened while moving -- a deliberate calibration
    in a planner mode -- is never judged this way: motion is its normal.
    """
    if started_moving:
        return False
    if moving:
        return True
    if q is None or q_start is None or len(q) != len(q_start):
        return False
    for now, then in zip(q, q_start):
        change = (float(now) - float(then) + math.pi) % (2.0 * math.pi) - math.pi
        if abs(change) > float(tolerance):
            return True
    return False


def choose_reference_pose(q_target, q_desired, expected=None,
                          active=None):
    """Return the pose auto-calibration should compare against, or None.

    Prefers ``active`` -- the target the live control mode is genuinely
    following, which fic_control publishes because only it knows whether
    that is the joint target or the IK solution. In Cartesian modes the
    controller tracks ``q_target_ik_``, so comparing against ``q_target``
    leaves auto-calibration permanently unsatisfied: measured on hardware
    as zero firings and a node stuck on 'calibration stale'. Then
    ``q_target`` -- republished every control cycle -- over
    ``{rid}_desired_joint_angles``, which is only sent when a new command
    is issued and is therefore silent on a running arm. That silence is
    why auto-calibration fired zero times across an entire hardware
    session: the cached desired pose stayed None and the trigger could
    never evaluate.

    A candidate of the wrong length is skipped rather than used, since a
    short vector would otherwise be compared element-wise against a full
    pose and silently pass.
    """
    for candidate in (active, q_target, q_desired):
        if candidate is None:
            continue
        if expected is not None and len(candidate) != expected:
            continue
        return candidate
    return None


class AutoCalibrator:
    """Track the settled-and-untouched condition and its dwell."""

    def __init__(self, tolerance, dwell_ticks, stiff_modes=STIFF_MODES,
                 max_age_ticks=0, veto_max_age_ticks=0):
        """Configure the tolerance (rad), dwell and bias age (ticks).

        ``max_age_ticks`` refreshes a bias that is still valid by pose but
        has simply grown old. Measured on hardware, a bias can drift to
        five times the detection thresholds while the arm has not moved
        at all -- thermal and sensor-bias drift do not present as pose
        change, so staleness-by-pose never notices them. Zero restores the
        previous behaviour, refreshing only when the bias is missing or
        the arm has left the calibration pose.
        """
        self._tolerance = float(tolerance)
        self._dwell_ticks = max(1, int(dwell_ticks))
        self._stiff_modes = frozenset(stiff_modes)
        self._max_age = max(0, int(max_age_ticks))
        self._veto_max = max(0, int(veto_max_age_ticks))
        self._age = 0
        self._held = 0
        self._offset_ref = None

    @property
    def offset_reference(self):
        """The steady-state offset the tolerance is measured from, or None."""
        return self._offset_ref

    def set_offset_reference(self, offset):
        """Record the untouched ``q - q_desired`` offset, per joint.

        An impedance-controlled arm never reaches its commanded pose: it
        settles where the restoring torque ``K*(q_target - q_current)``
        balances the gravity-model and friction torque, leaving a
        CONSTANT offset. Measured on this Kinova at 0.0342 rad on joint 6
        and 0.0183 rad on joint 2, against a 0.02 rad tolerance -- so the
        absolute test ``|q - q_desired| <= tolerance`` could essentially
        never pass, auto-calibration never fired, and the bias went stale
        and reported phantom contact on a completely still arm.

        Comparing the CHANGE in that offset instead keeps the tolerance
        tight where it matters. The steady-state part cancels; a push,
        which moves the offset away from this baseline by its own
        deflection, is still rejected at the original tolerance.

        Pass None to drop the baseline back to the bootstrap state.
        """
        self._offset_ref = (None if offset is None
                            else [float(v) for v in offset])

    @property
    def held_ticks(self):
        """Consecutive ticks the condition has held, for diagnostics."""
        return self._held

    def reset(self):
        """Forget the dwell and restart the bias clock."""
        self._held = 0
        self._age = 0

    def _eligible(self, mode, q, q_desired, moving,
                  calibrated, stale, calibrating, touched=False):
        """Return True when this tick satisfies every precondition."""
        if calibrating:
            return False
        if mode not in self._stiff_modes:
            return False
        if moving:
            return False
        # A direct observation beats the pose proxy. ``touched`` is the
        # existing bias's own residual saying a joint is loaded, which is
        # what this whole trigger is trying not to absorb -- a push held
        # perfectly still defeats every pose-based test, but it cannot
        # hide from the torque it is applying.
        #
        # Trusted only while the bias is fresh. A STALE bias manufactures
        # phantom contact (that is the failure this trigger exists to
        # prevent), so vetoing on it would deadlock: never recalibrate,
        # so never fresh, so never stop reporting contact. When the bias
        # is stale it is invalid anyway and must be replaced regardless.
        # ...but the veto MUST expire. Observed on hardware: a bad bias
        # (left by a transient during a stack restart) made the residual
        # read 15-25x over threshold on a completely untouched arm, the
        # veto then refused every recalibration, and the node sat stuck
        # for 75 s -- 99.7% false positives -- because nothing could ever
        # replace the bias that was generating the contact. Staleness
        # does not catch this: it is pose-based, and the arm had not
        # moved. Bounding the veto by bias age turns an unbounded
        # deadlock into a worst case of veto_max_age_ticks, at the cost
        # of absorbing a push held longer than that.
        if touched and calibrated and not stale:
            if not self._veto_max or self._age < self._veto_max:
                return False
        # A valid bias at this pose usually needs no replacing -- but one
        # that has simply aged does, because drift that is not pose drift
        # is invisible to the staleness test.
        if calibrated and not stale:
            if not self._max_age or self._age < self._max_age:
                return False
        if q is None or q_desired is None:
            return False
        if len(q) != len(q_desired) or not len(q):
            return False
        # Angular difference: a joint on the +-pi branch cut differs from
        # its target by 2*pi while sitting exactly on it.
        import math
        offsets = [(float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi
                   for a, b in zip(q, q_desired)]

        if self._offset_ref is None or len(self._offset_ref) != len(offsets):
            # No baseline yet: fall back to the absolute test, which is
            # exactly what this class did before baselining existed. It
            # only ever refuses MORE than the relative test, so it is
            # safe as a fallback -- never accepting a push it should have
            # rejected. Deviation from the commanded pose is the contact
            # signal itself, so blanket-accepting here (even with no bias
            # yet) would calibrate while someone is leaning on the arm.
            # contact_node seeds a baseline as soon as it has a still arm
            # and a reference pose, so the live node leaves this path
            # almost immediately.
            for measured in offsets:
                if abs(measured) > self._tolerance:
                    return False
            return True

        for measured, baseline in zip(offsets, self._offset_ref):
            change = (measured - baseline + math.pi) % (2.0 * math.pi) - math.pi
            if abs(change) > self._tolerance:
                return False
        return True

    def update(self, mode, q, q_desired, moving,
               calibrated, stale, calibrating, touched=False):
        """Advance one tick; True exactly when a calibration should start.

        Returns True on the tick the dwell completes and not again until
        the condition has lapsed and rebuilt, so the caller starts one
        calibration per settle rather than one per tick.
        """
        self._age += 1
        if not self._eligible(mode, q, q_desired, moving,
                              calibrated, stale, calibrating, touched):
            self._held = 0
            return False
        self._held += 1
        if self._held >= self._dwell_ticks:
            self._held = 0
            return True
        return False
