"""Tests for the contact bias auto-recalibration trigger.

The bias is measured at one pose and goes stale as the arm moves, and it
is lost entirely whenever the node restarts -- which now happens on every
hardware restart, since contact_launch.py rides along with the arm. This
trigger re-measures it automatically whenever the arm is demonstrably
untouched, so the operator does not have to.
"""

from fic_contact.autocal import AutoCalibrator


def _settled(**over):
    """Return trigger inputs describing a settled, untouched, stale arm."""
    state = {'mode': 1, 'q': [0.0] * 7, 'q_desired': [0.0] * 7,
             'moving': False, 'calibrated': False, 'stale': False,
             'calibrating': False}
    state.update(over)
    return state


def test_fires_when_settled_at_target_and_uncalibrated():
    """The whole point: no bias, arm parked on target, so measure one."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled()) is True


def test_requires_a_dwell_before_firing():
    """An arm passing through its target must not trigger a calibration."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=3)
    assert auto.update(**_settled()) is False
    assert auto.update(**_settled()) is False
    assert auto.update(**_settled()) is True


def test_dwell_resets_when_the_condition_breaks():
    """Partial dwell must not carry across an interruption."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=3)
    auto.update(**_settled())
    auto.update(**_settled(moving=True))
    assert auto.update(**_settled()) is False
    assert auto.update(**_settled()) is False
    assert auto.update(**_settled()) is True


def test_never_fires_in_an_admittance_mode():
    """Admittance modes yield to contact, so "on target" proves nothing.

    In modes 3, 4, 7 and 8 the arm follows the push and the desired
    pose follows the arm, so the position proxy reads "untouched" while
    someone is leaning on it -- calibrating there would absorb the push
    into the bias and hide that contact for good.
    """
    for mode in (3, 4, 7, 8):
        auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
        assert auto.update(**_settled(mode=mode)) is False, mode


def test_fires_in_every_stiff_mode():
    """Impedance and planner modes deflect under contact, so they qualify."""
    for mode in (1, 2, 5, 6):
        auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
        assert auto.update(**_settled(mode=mode)) is True, mode


def test_refuses_when_the_arm_is_off_target():
    """Deviation from the commanded pose is the contact signal itself."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    q = [0.0] * 7
    q[3] = 0.5
    assert auto.update(**_settled(q=q)) is False


def test_tolerance_is_per_joint_worst_case():
    """A single joint outside tolerance blocks, however small the rest."""
    auto = AutoCalibrator(tolerance=0.1, dwell_ticks=1)
    q = [0.0] * 7
    q[6] = 0.15
    assert auto.update(**_settled(q=q)) is False
    q[6] = 0.05
    assert auto.update(**_settled(q=q)) is True


def test_refuses_while_the_arm_is_moving():
    """A bias measured mid-motion carries the inertial term it omits."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(moving=True)) is False


def test_refuses_while_a_calibration_is_already_running():
    """Re-arming mid-window would restart it forever."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(calibrating=True)) is False


def test_skips_when_the_existing_bias_is_still_valid():
    """Do not churn: a fresh bias at this pose needs no replacing."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(
        **_settled(calibrated=True, stale=False)) is False


def test_fires_when_calibrated_but_stale():
    """This is what keeps it fresh as the arm is moved around."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(calibrated=True, stale=True)) is True


def test_refuses_without_a_desired_pose():
    """Cartesian modes may never publish joint targets -- fail closed."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(q_desired=None)) is False
    assert auto.update(**_settled(q=None)) is False


def test_refuses_on_a_desired_pose_of_the_wrong_length():
    """A mismatched vector must not compare element-wise by luck."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(q_desired=[0.0] * 3)) is False


def test_refuses_before_any_mode_is_known():
    """`_mode` is None until the operator selects one; not a stiff mode."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(mode=None)) is False


def test_desired_angles_subscription_type_matches_the_publisher():
    """contact_node must subscribe to desired_joint_angles as Float32.

    FIC Studio and fic_hardware both use Float32MultiArray on that
    topic. A Float64 subscription simply never matches the publisher --
    no error, no warning, `_q_desired` stays None and auto-calibration
    silently never fires. Cheap source check, because the failure mode
    is invisible at runtime.
    """
    import os
    here = os.path.dirname(__file__)
    node = open(os.path.join(
        here, '..', 'fic_contact', 'contact_node.py')).read()
    line = next(ln for ln in node.splitlines()
                if 'desired_joint_angles' in ln and '%s_' in ln)
    idx = node.splitlines().index(line)
    decl = '\n'.join(node.splitlines()[idx - 1:idx + 1])
    assert 'Float32MultiArray' in decl, decl


def test_target_pose_is_preferred_over_the_desired_command():
    """Auto-calibration must run off a pose that is actually streamed.

    `{rid}_desired_joint_angles` is published only when a new command is
    issued, so on hardware it is silent and the cached value stays None --
    which is why auto-calibration never fired in the first robot session
    (0 firings across a whole run). `{rid}_q_target` is published every
    cycle by fic_hardware and is the target the controller is genuinely
    tracking, so it is the correct signal.
    """
    from fic_contact.autocal import choose_reference_pose

    q_target = [0.1] * 7
    q_desired = [0.9] * 7
    assert choose_reference_pose(q_target, q_desired) == q_target
    assert choose_reference_pose(None, q_desired) == q_desired
    assert choose_reference_pose(q_target, None) == q_target
    assert choose_reference_pose(None, None) is None


def test_reference_pose_rejects_a_wrong_length_target():
    """A short or malformed target must fall through, not be used."""
    from fic_contact.autocal import choose_reference_pose
    short, full = [0.1, 0.2], [0.9] * 7
    assert choose_reference_pose(short, full, expected=7) == full
    assert choose_reference_pose(short, None, expected=7) is None


def test_refreshes_a_valid_bias_once_it_is_old_enough():
    """A bias that is fresh by pose can still be stale in fact.

    Measured on hardware: with pose drift correctly small, `is_stale`
    read False while the resting residual had grown to 0.53 Nm against
    0.10 Nm thresholds -- 100% false positives with nothing touching the
    arm. The drift was not from moving; thermal and sensor-bias drift do
    not announce themselves as pose change. So a settled, untouched arm
    re-measures periodically rather than only when it has moved.
    """
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1,
                          max_age_ticks=5)
    fresh = _settled(calibrated=True, stale=False)
    # Young and valid: leave it alone.
    for _ in range(4):
        assert auto.update(**fresh) is False
    # Old enough now, and the arm is settled, so refresh.
    assert auto.update(**fresh) is True


def test_age_refresh_still_respects_every_other_precondition():
    """Age must not become a way around the safety conditions."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1, max_age_ticks=1)
    # Old, but the arm is being pushed off target -- must not calibrate.
    q = [0.0] * 7
    q[3] = 0.5
    for _ in range(10):
        state = _settled(calibrated=True, stale=False, q=q)
        assert auto.update(**state) is False
    # Old, but in an admittance mode -- must not calibrate.
    for _ in range(10):
        state = _settled(calibrated=True, stale=False, mode=3)
        assert auto.update(**state) is False


def test_age_refresh_can_be_switched_off():
    """max_age_ticks=0 restores the previous behaviour exactly."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1, max_age_ticks=0)
    for _ in range(50):
        state = _settled(calibrated=True, stale=False)
        assert auto.update(**state) is False


def test_active_target_is_preferred_over_the_joint_target():
    """Cartesian modes track a different target, so prefer the active one.

    Measured on hardware in Cartesian Impedance: auto-calibration fired
    zero times and the node sat blocked on 'calibration stale', because
    it compared q_current against q_target while the controller was
    tracking q_target_ik_. fic_hardware now publishes whichever target
    the live mode actually follows, and that is the one to use.
    """
    from fic_contact.autocal import choose_reference_pose

    active = [0.1] * 7
    joint = [0.5] * 7
    desired = [0.9] * 7
    assert choose_reference_pose(joint, desired, active=active) == active
    assert choose_reference_pose(joint, desired, active=None) == joint
    assert choose_reference_pose(None, desired, active=None) == desired
    # A malformed active target falls through rather than being trusted.
    assert choose_reference_pose(joint, desired, active=[0.1],
                                 expected=7) == joint


def test_refuses_while_the_arm_is_loaded():
    """A push held perfectly still defeats every pose test; torque does not.

    Once the transient is over the arm is stationary and its offset has
    stopped changing, so the pose proxy reads 'untouched' while somebody
    is leaning on it. The residual against the current bias is what
    catches that, and it must veto.
    """
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(calibrated=True, stale=False,
                                  touched=True)) is False


def test_a_touched_but_stale_bias_is_still_replaced():
    """The veto must not deadlock the trigger it is protecting.

    A stale bias manufactures phantom contact -- that is the exact
    failure auto-calibration exists to prevent. If ``touched`` vetoed on
    a stale bias too, the node would never recalibrate, so the bias would
    never become fresh, so the phantom would never clear.
    """
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(calibrated=True, stale=True,
                                  touched=True)) is True


def test_touch_veto_is_inactive_before_any_bias_exists():
    """With no bias there is no residual to judge contact by."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(calibrated=False, touched=True)) is True


def test_touch_veto_defaults_off_for_existing_callers():
    """Omitting ``touched`` keeps the pre-veto behaviour unchanged."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1)
    assert auto.update(**_settled(calibrated=True, stale=True)) is True


def test_the_touch_veto_expires_so_a_bad_bias_cannot_self_sustain():
    """The veto must not be able to protect the bias that causes it.

    Observed on hardware: a transient left a bad bias, the residual then
    read 15-25x over threshold on a completely untouched arm, and the
    veto refused every recalibration for 75 s -- 99.7% false positives.
    Staleness cannot break it (it is pose-based and the arm had not
    moved), so the veto itself has to expire.
    """
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1, max_age_ticks=2,
                          veto_max_age_ticks=5)
    state = _settled(calibrated=True, stale=False, touched=True)
    for _ in range(4):
        assert auto.update(**state) is False
    # Age has now passed the ceiling, so the bias is replaced regardless.
    assert auto.update(**state) is True


def test_the_touch_veto_still_holds_inside_the_ceiling():
    """A push shorter than the ceiling is not absorbed."""
    auto = AutoCalibrator(tolerance=0.02, dwell_ticks=1, max_age_ticks=2,
                          veto_max_age_ticks=50)
    state = _settled(calibrated=True, stale=False, touched=True)
    for _ in range(40):
        assert auto.update(**state) is False


# ── calibration_disturbed: an open window must notice a push ─────────────

from fic_contact.autocal import calibration_disturbed  # noqa: E402

_Q0 = [0.0, 0.25, -3.139, -2.263, -0.017, 1.032, 1.568]


def test_a_still_untouched_window_is_not_disturbed():
    assert not calibration_disturbed(_Q0, _Q0, moving=False,
                                     started_moving=False, tolerance=0.02)


def test_motion_during_a_window_opened_at_rest_disturbs_it():
    """A push arriving mid-window moves the arm before it deflects far."""
    assert calibration_disturbed(_Q0, _Q0, moving=True,
                                 started_moving=False, tolerance=0.02)


def test_a_slow_deflection_past_tolerance_disturbs_it():
    q = list(_Q0)
    q[1] += 0.03
    assert calibration_disturbed(q, _Q0, moving=False,
                                 started_moving=False, tolerance=0.02)


def test_the_branch_cut_is_not_a_deflection():
    """Joint 3 sits at -179.9 deg here; crossing +-pi is not 2 pi of motion."""
    q = list(_Q0)
    q[2] = 3.14159 - 0.001
    q0 = list(_Q0)
    q0[2] = -3.14159 + 0.001
    assert not calibration_disturbed(q, q0, moving=False,
                                     started_moving=False, tolerance=0.02)


def test_a_window_opened_while_moving_is_never_judged():
    """A deliberate calibration in a planner mode moves by design."""
    q = list(_Q0)
    q[1] += 0.5
    assert not calibration_disturbed(q, _Q0, moving=True,
                                     started_moving=True, tolerance=0.02)


def test_missing_poses_do_not_disturb():
    assert not calibration_disturbed(None, _Q0, moving=False,
                                     started_moving=False, tolerance=0.02)
    assert not calibration_disturbed(_Q0, None, moving=False,
                                     started_moving=False, tolerance=0.02)
