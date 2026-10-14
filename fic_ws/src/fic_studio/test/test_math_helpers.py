"""Quaternion → Euler conversion used by the Dashboard's end-effector block.

The Dashboard shows orientation as theta-x / theta-y / theta-z beside the
native quaternion, matching the convention the Kinova Web App's Monitoring
page reports: an x-y-z *extrinsic* triple, i.e. R = Rz(tz) @ Ry(ty) @ Rx(tx).
"""
import math

from fic_studio.math_helpers import _axis_angle_to_quat, _quat_to_euler_xyz


def test_identity_quaternion_is_zero_rotation():
    """The identity quaternion maps to no rotation about any axis."""
    assert _quat_to_euler_xyz(0.0, 0.0, 0.0, 1.0) == (0.0, 0.0, 0.0)


def test_ninety_degrees_about_x():
    """A quarter turn about x reads on theta-x alone."""
    q = _axis_angle_to_quat(1.0, 0.0, 0.0, 90.0, "deg")
    tx, ty, tz = _quat_to_euler_xyz(*q)
    assert math.isclose(tx, 90.0, abs_tol=1e-6)
    assert math.isclose(ty, 0.0, abs_tol=1e-6)
    assert math.isclose(tz, 0.0, abs_tol=1e-6)


def test_ninety_degrees_about_z():
    """A quarter turn about z reads on theta-z alone."""
    q = _axis_angle_to_quat(0.0, 0.0, 1.0, 90.0, "deg")
    tx, ty, tz = _quat_to_euler_xyz(*q)
    assert math.isclose(tx, 0.0, abs_tol=1e-6)
    assert math.isclose(ty, 0.0, abs_tol=1e-6)
    assert math.isclose(tz, 90.0, abs_tol=1e-6)


def test_radians_units():
    """units='rad' returns radians rather than degrees."""
    q = _axis_angle_to_quat(0.0, 1.0, 0.0, 30.0, "deg")
    _, ty, _ = _quat_to_euler_xyz(*q, units="rad")
    assert math.isclose(ty, math.radians(30.0), abs_tol=1e-9)


def test_gimbal_lock_pitch_does_not_raise():
    """Pitch at +90 degrees must clamp, not raise a math domain error."""
    q = _axis_angle_to_quat(0.0, 1.0, 0.0, 90.0, "deg")
    tx, ty, tz = _quat_to_euler_xyz(*q)
    assert math.isclose(ty, 90.0, abs_tol=1e-6)
    assert all(math.isfinite(v) for v in (tx, ty, tz))


def test_denormalised_quaternion_is_normalised_first():
    """A non-unit quaternion gives the same angles as its normalised form."""
    assert _quat_to_euler_xyz(0.0, 0.0, 0.0, 4.0) == (0.0, 0.0, 0.0)


def test_zero_quaternion_falls_back_to_identity():
    """An all-zero quaternion degrades to identity rather than dividing by zero."""
    assert _quat_to_euler_xyz(0.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)
