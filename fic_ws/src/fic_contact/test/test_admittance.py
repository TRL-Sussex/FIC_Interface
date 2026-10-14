"""Tests for the admittance-mode contact observable and its gating."""

import numpy as np
import pytest

from fic_contact.admittance import (
    admitting_joints, admitting_link, combine_link_estimates,
    parse_trajectory_blocks)
from fic_contact.gate import ALL_MODES, ContactGate

_BLOCK_NAMES = (
    'planner_target', 'q_current', 'error', 'q_target',
    'control_effort', 'q_target_adm', 'desired_effort', 'measured_effort',
)


def test_parse_trajectory_blocks_round_trips():
    """Each of the 8 blocks comes back at the position it was written."""
    n = 7
    data = np.arange(8 * n, dtype=float)
    blocks = parse_trajectory_blocks(data, n)
    for k, name in enumerate(_BLOCK_NAMES):
        np.testing.assert_array_equal(blocks[name], data[k * n:(k + 1) * n])


def test_parse_trajectory_blocks_rejects_wrong_length():
    """A message that is not 8*N long is a programming/wiring error."""
    with pytest.raises(ValueError):
        parse_trajectory_blocks(np.zeros(8 * 7 - 1), 7)


def test_offset_pattern_yields_link_three():
    """A big offset on joints 1-3 and none beyond names link 3."""
    offsets = np.array([0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0])
    link, confidence = admitting_link(offsets, 0.01)
    assert link == 3
    assert confidence > 0.0


def test_admitting_joints_feeds_admitting_link_the_same_way():
    """The two-function pipeline reproduces the same link as a raw array."""
    q_target = np.zeros(7)
    q_target_adm = np.array([0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0])
    offsets = admitting_joints(q_target, q_target_adm, 0.01)
    link, _ = admitting_link(offsets, 0.01)
    assert link == 3


def test_all_zero_offsets_mean_none_admitting_not_no_statement():
    """Zero everywhere is a real answer (0), not a refusal (-1)."""
    link, confidence = admitting_link(np.zeros(7), 0.01)
    assert link == 0
    assert confidence == 0.0


def test_non_finite_offsets_yield_no_statement():
    """A NaN or inf offset must never be read as 'no contact'."""
    offsets = np.array([0.5, np.nan, 0.5, 0.0, 0.0, 0.0, 0.0])
    assert admitting_link(offsets, 0.01) == (-1, 0.0)
    offsets = np.array([0.5, np.inf, 0.5, 0.0, 0.0, 0.0, 0.0])
    assert admitting_link(offsets, 0.01) == (-1, 0.0)


def test_deadband_floors_small_offsets_to_exactly_zero():
    """An offset smaller than threshold_rad must not leak through as noise."""
    q_target = np.zeros(3)
    q_target_adm = np.array([0.001, 0.0, 0.0])
    offsets = admitting_joints(q_target, q_target_adm, 0.01)
    assert offsets[0] == 0.0


def test_admitting_joints_rejects_mismatched_shapes():
    """q_target and q_target_adm of different lengths is a wiring error."""
    with pytest.raises(ValueError):
        admitting_joints(np.zeros(7), np.zeros(6), 0.01)


# --- New offset path: `{rid}_admittance_offset` fed straight to
# `admitting_link`, the way `fic_contact.contact_node` now does it. This
# is the controller's own accumulated per-joint push offset
# (`fic::ficControl::humanOffset`), NOT derived from `q_target` /
# `q_target_adm` — there is no subtraction step any more, so these tests
# exercise `admitting_link` on a raw accumulator array directly rather
# than going through `admitting_joints` first.

def test_raw_accumulated_offset_feeds_admitting_link_without_subtraction():
    """A humanOffset()-shaped array needs no q_target/q_target_adm pair.

    Simulates the `{rid}_admittance_offset` message payload: joints 0-3
    have been pushed, 4-6 have not. No `admitting_joints` call — this is
    exactly what `contact_node._estimate` passes to `admitting_link`.
    """
    accumulated_offset = np.array([0.4, 0.3, 0.2, 0.15, 0.0, 0.0, 0.0])
    link, confidence = admitting_link(
        accumulated_offset, threshold_rad=0.01)
    assert link == 4
    assert confidence > 0.0


def test_raw_accumulated_offset_below_threshold_reads_as_no_contact():
    """Sub-threshold accumulator noise must not read as a push.

    `admitting_link` applies the threshold itself (via
    `hypotheses.support_link`'s per-joint ratio test), so a raw offset
    array does not need the `admitting_joints` pre-flooring step to get
    the same "too small to count" behaviour.
    """
    accumulated_offset = np.array([0.001, -0.002, 0.0, 0.0, 0.0])
    link, confidence = admitting_link(accumulated_offset, threshold_rad=0.01)
    assert link == 0
    assert confidence == 0.0


def test_raw_accumulated_offset_combines_with_torque_residual():
    """The new offset path still cross-checks against the residual.

    Mirrors `contact_node._estimate`'s call into `combine_link_estimates`
    once both observables have a verdict: agreement on link 5 between the
    `{rid}_admittance_offset`-derived estimate and the torque-residual
    estimate should read as at least as confident as either alone.
    """
    accumulated_offset = np.array([0.3, 0.25, 0.2, 0.15, 0.1, 0.0, 0.0])
    link_a, confidence_a = admitting_link(accumulated_offset, threshold_rad=0.01)
    assert link_a == 5

    link_b, confidence_b = 5, 0.6
    link, confidence = combine_link_estimates(
        link_a, confidence_a, link_b, confidence_b)
    assert link == 5
    assert confidence == max(confidence_a, confidence_b)


def test_agreement_is_more_confident_than_disagreement():
    """Two observables landing on the same link should read as stronger."""
    agree_link, agree_conf = combine_link_estimates(3, 0.8, 3, 0.7)
    disagree_link, disagree_conf = combine_link_estimates(3, 0.8, 5, 0.7)
    assert agree_link == 3
    assert agree_conf > disagree_conf


def test_agreement_confidence_is_never_below_either_observable_alone():
    """Agreeing must not look weaker than either single observable did."""
    link, confidence = combine_link_estimates(4, 0.3, 4, 0.9)
    assert link == 4
    assert confidence >= 0.9


def test_combine_returns_the_other_side_when_one_has_no_statement():
    """A missing observable (-1) must not be treated as a disagreement."""
    assert combine_link_estimates(-1, 0.0, 4, 0.6) == (4, 0.6)
    assert combine_link_estimates(4, 0.6, -1, 0.0) == (4, 0.6)


def test_combine_reports_no_statement_when_both_sides_have_none():
    """Two corrupt/absent observables must still refuse, not guess."""
    assert combine_link_estimates(-1, 0.0, -1, 0.0) == (-1, 0.0)


def test_combine_agrees_on_no_contact_too():
    """Both observables saying 'nobody is admitting' is itself an agreement."""
    link, _ = combine_link_estimates(0, 0.0, 0, 0.0)
    assert link == 0


def test_gate_permits_all_admittance_modes_with_all_modes_allowed():
    """Modes 3, 4, 7 and 8 are no longer refused once the gate allows them."""
    for mode in (3, 4, 7, 8):
        gate = ContactGate(allowed_mode=ALL_MODES)
        gate.set_feedback(True)
        gate.set_calibrated(True)
        gate.set_mode(mode)
        assert gate.blocked_reason() is None, 'mode %d was refused' % mode


def test_gate_still_permits_the_impedance_modes_too():
    """The same expanded gate keeps allowing the original impedance modes."""
    for mode in (1, 2, 5, 6):
        gate = ContactGate(allowed_mode=ALL_MODES)
        gate.set_feedback(True)
        gate.set_calibrated(True)
        gate.set_mode(mode)
        assert gate.blocked_reason() is None, 'mode %d was refused' % mode


def test_bare_gate_default_still_refuses_admittance():
    """The default gate (used nowhere in this feature) is unchanged."""
    for mode in (3, 4, 7, 8):
        gate = ContactGate()
        gate.set_feedback(True)
        gate.set_calibrated(True)
        gate.set_mode(mode)
        assert gate.blocked_reason() is not None, 'mode %d was allowed' % mode
