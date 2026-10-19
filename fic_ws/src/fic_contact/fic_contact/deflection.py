"""External joint torque from the FIC controller's tracking error.

A second observable of the same contact, built from different hardware.
`residual.ResidualEstimator` reads the joint torque sensors and subtracts
a modelled gravity term. This module reads the position encoders and
multiplies the controller's tracking error by its stiffness. Same
physical quantity, different sensor.

**Where the two channels are independent, and where they are not.**
Not independent in model error: `fic_control` computes its gravity
feedforward as `solver_->gravity(q_current_)`, from the same URDF the
estimator inverts, so an under-modelled tool makes the arm sag and that
sag shows up as tracking error. Both channels therefore observe
``contact + gravity_model_error``, and a bias calibration is still
required on each.

Independent in *sensor noise*, which is what fusing them buys. And the
asymmetry favours this channel where it matters most: encoder noise of
order 1e-4 rad against a stiffness of 200 Nm/rad is ~0.02 Nm of
torque-equivalent noise, an order of magnitude below the ~0.2 Nm on the
torque sensors. Since detection is force-limited rather than
resolution-limited -- 10.4% link accuracy at 5 N against 58.9% at 40 N --
the quieter channel is exactly what a light touch needs.

**The regime this is valid in.** The FIC is hysteretic: `FIC.cpp`'s
`control()` takes the divergence branch (``effort = getEffort(error)``,
a function of error alone) only while ``|error|`` is growing, and a
path-dependent release branch while it shrinks. An operator pressing
into the arm *is* the divergence branch, which is why this works at all;
during release the mapping is not invertible from error alone. Callers
should use this while a push is developing, not while it recovers.

`getEffort` is linear, ``K0 * error``, only up to the first saturation
breakpoint. Beyond that the profile bends over toward a constant
saturation effort, so a large deflection under-reports. This module
models the linear range and reports its own validity.

No ROS, no Qt. numpy only.
"""

import numpy as np

#: Nm/rad, joints 1-7, read from ``fic_library/config/config.xml``. The
#: wrist joints are far softer than the shoulder, so the same push
#: produces a much larger deflection out there -- which is also why the
#: torque-equivalent noise of this channel is lowest at the wrist.
KINOVA_STIFFNESS_NM_PER_RAD = (200.0, 260.0, 200.0, 200.0, 50.0, 50.0, 50.0)

#: rad. Beyond this the FIC's linear range gives way to its saturation
#: segments and ``K0 * error`` over-reports the restoring effort. Used
#: only to flag validity; the caller decides what to do about it.
LINEAR_RANGE_RAD = 0.30


def tau_ext_from_deflection(q_target, q_current, stiffness):
    """Return the external joint torque implied by the tracking error.

    ``fic_control`` calls ``impedance(i, ..., q_target_[i],
    q_current_[i])``, so the controller error is
    ``q_target - q_current`` and the restoring effort it applies is
    ``K * error``. At quasi-static equilibrium that effort balances the
    external torque, giving

        tau_ext = K * (q_current - q_target)

    Returns None -- never a plausible-looking torque -- for mismatched
    lengths, non-finite entries, or a non-positive stiffness.
    """
    if q_target is None or q_current is None or stiffness is None:
        return None
    target = np.asarray(q_target, dtype=float).reshape(-1)
    current = np.asarray(q_current, dtype=float).reshape(-1)
    gain = np.asarray(stiffness, dtype=float).reshape(-1)
    if target.shape != current.shape or target.shape != gain.shape:
        return None
    if not target.size:
        return None
    for vector in (target, current, gain):
        if not np.isfinite(vector).all():
            return None
    if np.any(gain <= 0.0):
        return None
    return gain * (current - target)


def deflection_is_linear(q_target, q_current,
                         linear_range_rad=LINEAR_RANGE_RAD):
    """Return True while every joint sits inside the FIC's linear range.

    Outside it, ``K0 * error`` over-reports because the real profile is
    bending toward a saturation plateau, so a caller fusing the two
    channels should down-weight or drop this one.
    """
    if q_target is None or q_current is None:
        return False
    target = np.asarray(q_target, dtype=float).reshape(-1)
    current = np.asarray(q_current, dtype=float).reshape(-1)
    if target.shape != current.shape or not target.size:
        return False
    if not (np.isfinite(target).all() and np.isfinite(current).all()):
        return False
    return bool(np.all(np.abs(current - target) <= float(linear_range_rad)))


def deflection_sigma(stiffness, encoder_sigma_rad):
    """Return this channel's torque-equivalent noise, per joint, Nm.

    Encoder noise enters multiplied by the stiffness, so a stiff joint is
    a noisy torque estimate and a soft one is a quiet one -- the reverse
    of the intuition that a stiff joint measures force better.
    """
    gain = np.asarray(stiffness, dtype=float).reshape(-1)
    return gain * float(encoder_sigma_rad)


def fuse_inverse_variance(estimates, sigmas):
    """Combine independent estimates of the same vector, weighting by 1/var.

    Returns ``(fused, sigma)``, or ``(None, None)`` when every channel is
    missing. A missing channel is skipped rather than treated as a
    measurement of zero, which would drag the fused estimate toward zero
    and suppress real contacts.

    ``sigmas`` may be scalars or per-joint arrays. The independence this
    assumes holds for the *sensor* noise, which is what it is being
    applied to; it does not hold for the shared gravity model error, so
    fusing does not reduce that and both channels still need their bias
    calibrated.
    """
    weighted = None
    weight_sum = None
    for estimate, sigma in zip(estimates, sigmas):
        if estimate is None:
            continue
        value = np.asarray(estimate, dtype=float).reshape(-1)
        if not np.isfinite(value).all():
            continue
        var = np.asarray(sigma, dtype=float)
        var = np.broadcast_to(var, value.shape).astype(float) ** 2
        if np.any(var <= 0.0) or not np.isfinite(var).all():
            continue
        weight = 1.0 / var
        contribution = weight * value
        weighted = (contribution if weighted is None
                    else weighted + contribution)
        weight_sum = weight if weight_sum is None else weight_sum + weight
    if weighted is None:
        return None, None
    fused = weighted / weight_sum
    sigma = np.sqrt(1.0 / weight_sum)
    # A scalar-sigma fusion of equal-length inputs gives a uniform sigma;
    # collapse it so callers can compare against a scalar threshold.
    if np.allclose(sigma, sigma.flat[0]):
        return fused, float(sigma.flat[0])
    return fused, sigma
