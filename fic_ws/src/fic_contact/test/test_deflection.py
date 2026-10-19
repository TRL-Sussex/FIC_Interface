"""Tests for the deflection channel: external torque from the FIC error.

A second, independent observable of the same contact. The residual
channel reads the torque sensors and subtracts a gravity model; this one
reads the position encoders and multiplies by the controller stiffness.
Same physical quantity, different sensor.

What it is NOT: immune to gravity model error. fic_control computes its
gravity feedforward from the same URDF (`solver_->gravity(q_current_)`),
so an under-modelled tool makes the arm sag and that sag appears as
deflection. Both channels see `contact + gravity error`; what differs is
the sensor and therefore the noise.

Where it wins is low force. Encoder noise of ~1e-4 rad against a
stiffness of 200 Nm/rad is ~0.02 Nm of torque-equivalent noise, against
~0.2 Nm on the torque sensors -- and force, not resolution, is what
currently limits detection (10.4% accuracy at 5 N).
"""

import numpy as np
import pytest

from fic_contact.deflection import (
    fuse_inverse_variance, tau_ext_from_deflection)

_K = np.array([200.0, 260.0, 200.0, 200.0, 50.0, 50.0, 50.0])


def test_recovers_the_torque_that_caused_the_deflection():
    """The sign convention, pinned against the controller's own.

    fic_control calls `impedance(i, ..., q_target_[i], q_current_[i])`,
    so the controller error is `q_target - q_current` and its restoring
    effort is `K * error`. At quasi-static equilibrium that effort
    balances the external torque, so
    `tau_ext = K * (q_current - q_target)`. An inverted sign here would
    make every fused estimate fight the residual channel instead of
    reinforcing it.
    """
    tau_true = np.array([3.0, -5.0, 2.0, 0.0, 1.0, -0.5, 0.25])
    q_target = np.zeros(7)
    q_current = q_target + tau_true / _K       # equilibrium deflection
    got = tau_ext_from_deflection(q_target, q_current, _K)
    assert np.allclose(got, tau_true)


def test_zero_deflection_is_zero_torque():
    """An arm sitting on its target reports no external torque."""
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    assert np.allclose(tau_ext_from_deflection(q, q, _K), np.zeros(7))


def test_rejects_mismatched_or_nonfinite_input():
    """Bad input yields None, never a plausible-looking torque."""
    q = np.zeros(7)
    assert tau_ext_from_deflection(q, np.zeros(3), _K) is None
    assert tau_ext_from_deflection(q, q, np.zeros(3)) is None
    bad = q.copy()
    bad[2] = np.nan
    assert tau_ext_from_deflection(q, bad, _K) is None
    assert tau_ext_from_deflection(None, q, _K) is None


def test_rejects_nonpositive_stiffness():
    """A zero or negative stiffness cannot map deflection to torque."""
    q = np.zeros(7)
    k = _K.copy()
    k[3] = 0.0
    assert tau_ext_from_deflection(q, q, k) is None


def test_fusion_lies_between_its_inputs():
    """An inverse-variance blend cannot leave the interval it blends."""
    a = np.array([2.0, 0.0])
    b = np.array([4.0, 0.0])
    fused, _ = fuse_inverse_variance([a, b], [1.0, 1.0])
    assert np.allclose(fused, [3.0, 0.0])


def test_fusion_weights_the_quieter_channel_more():
    """The point of fusing: trust the channel with less noise."""
    a = np.array([2.0])
    b = np.array([4.0])
    fused, _ = fuse_inverse_variance([a, b], [0.1, 1.0])
    assert fused[0] < 2.2, fused


def test_fusion_reduces_the_uncertainty():
    """Two independent measurements beat either alone."""
    _, sigma = fuse_inverse_variance(
        [np.zeros(3), np.zeros(3)], [0.2, 0.2])
    assert sigma == pytest.approx(0.2 / np.sqrt(2.0))


def test_fusion_ignores_a_missing_channel():
    """A channel with no data must not drag the estimate toward zero."""
    a = np.array([2.0, 3.0])
    fused, sigma = fuse_inverse_variance([a, None], [0.2, 0.02])
    assert np.allclose(fused, a)
    assert sigma == pytest.approx(0.2)


def test_fusion_of_nothing_is_nothing():
    """No channels means no estimate."""
    assert fuse_inverse_variance([None, None], [0.2, 0.2]) == (None, None)
