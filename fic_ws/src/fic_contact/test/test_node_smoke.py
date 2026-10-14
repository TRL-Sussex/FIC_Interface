"""Tests for the contact node's gating logic, without a ROS graph."""

from fic_contact.gate import ContactGate


def test_refuses_without_feedback():
    """No joint data means no statement."""
    gate = ContactGate(allowed_mode=1)
    gate.set_mode(1)
    assert gate.blocked_reason() == 'no joint feedback'


def test_refuses_in_admittance_mode():
    """Mode 3 lets the arm yield, so the residual is closed-loop."""
    gate = ContactGate(allowed_mode=1)
    gate.set_feedback(True)
    gate.set_calibrated(True)
    gate.set_mode(3)
    assert 'mode' in gate.blocked_reason()


def test_refuses_in_every_mode_but_one():
    """Only mode 1 is permitted — not 0, not 2, not the Cartesian modes."""
    for mode in (0, 2, 3, 4, 5, 6, 7, 8):
        gate = ContactGate(allowed_mode=1)
        gate.set_feedback(True)
        gate.set_calibrated(True)
        gate.set_mode(mode)
        assert gate.blocked_reason() is not None, f'mode {mode} was allowed'


def test_refuses_when_uncalibrated():
    """Without a bias the residual is dominated by model error."""
    gate = ContactGate(allowed_mode=1)
    gate.set_feedback(True)
    gate.set_mode(1)
    gate.set_calibrated(False)
    assert gate.blocked_reason() == 'not calibrated'


def test_refuses_when_calibration_is_stale():
    """A bias measured at another pose must not be used silently."""
    gate = ContactGate(allowed_mode=1)
    gate.set_feedback(True)
    gate.set_mode(1)
    gate.set_calibrated(True)
    gate.set_stale(True)
    assert gate.blocked_reason() == 'calibration stale'


def test_passes_when_everything_is_satisfied():
    """The happy path reports no blocker."""
    gate = ContactGate(allowed_mode=1)
    gate.set_feedback(True)
    gate.set_mode(1)
    gate.set_calibrated(True)
    assert gate.blocked_reason() is None


def test_default_allows_all_four_impedance_modes():
    """The default gate accepts modes 1, 2, 5 and 6.

    Joint/Cartesian Impedance, either stationary or moving under a planner.
    """
    for mode in (1, 2, 5, 6):
        gate = ContactGate()
        gate.set_feedback(True)
        gate.set_calibrated(True)
        gate.set_mode(mode)
        assert gate.blocked_reason() is None, 'mode %d was refused' % mode


def test_default_refuses_all_four_admittance_modes():
    """The default gate refuses modes 3, 4, 7 and 8, each naming the mode."""
    for mode in (3, 4, 7, 8):
        gate = ContactGate()
        gate.set_feedback(True)
        gate.set_calibrated(True)
        gate.set_mode(mode)
        reason = gate.blocked_reason()
        assert reason is not None, 'mode %d was allowed' % mode
        assert str(mode) in reason, 'reason %r does not name mode %d' % (reason, mode)


def test_admittance_refusal_reason_explains_the_arm_yields():
    """The refusal reason for an admittance mode explains, not just rejects."""
    gate = ContactGate()
    gate.set_feedback(True)
    gate.set_calibrated(True)
    gate.set_mode(4)
    reason = gate.blocked_reason()
    assert 'yield' in reason
    assert '4' in reason


def test_allowed_mode_accepts_a_single_int_for_backward_compatibility():
    """Passing one int still restricts the gate to that mode alone."""
    gate = ContactGate(allowed_mode=5)
    gate.set_feedback(True)
    gate.set_calibrated(True)
    gate.set_mode(5)
    assert gate.blocked_reason() is None
    gate.set_mode(1)
    assert gate.blocked_reason() is not None


def test_no_mode_yet_is_distinct_from_a_disallowed_mode():
    """Never having received a mode must not read as a rejected mode.

    `{rid}_control_mode` is VOLATILE and published only on change, so a
    node started alongside the hardware stack -- as contact_launch.py now
    is -- holds `_mode = None` until the operator next touches the mode
    selector. That is an ordinary startup state with an obvious operator
    fix, not a rejected mode, and the reason string is the only place it
    is ever explained.
    """
    gate = ContactGate(allowed_mode=1)
    gate.set_feedback(True)
    gate.set_calibrated(True)
    reason = gate.blocked_reason()
    assert reason is not None
    assert 'None' not in reason
    assert 'no control mode' in reason.lower()


def test_a_default_mode_removes_the_no_mode_block():
    """The arm boots in mode 1, so the gate must not wait to be told.

    `{rid}_control_mode` is VOLATILE and published only on change, so a
    node started alongside the arm can sit blocked indefinitely waiting
    for a message that will not come until the operator touches the mode
    selector. Since this robot always powers up in Joint Impedance, the
    node assumes that until told otherwise.
    """
    gate = ContactGate(allowed_mode=(1, 2, 5, 6))
    gate.set_feedback(True)
    gate.set_calibrated(True)
    gate.set_mode(1)
    assert gate.blocked_reason() is None


def test_an_explicit_mode_still_overrides_the_default():
    """A real mode message must still be able to close the gate."""
    gate = ContactGate(allowed_mode=(1, 2, 5, 6))
    gate.set_feedback(True)
    gate.set_calibrated(True)
    gate.set_mode(1)
    gate.set_mode(3)
    assert gate.blocked_reason() is not None
