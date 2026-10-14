# math_helpers.py
import math

def _axis_angle_to_quat(ax: float, ay: float, az: float, angle: float, units: str):
    """Convert axis-angle to a normalized quaternion (x,y,z,w)."""
    theta = math.radians(angle) if units == "deg" else angle
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12 or abs(theta) < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    ux, uy, uz = ax / n, ay / n, az / n
    half = 0.5 * theta
    s = math.sin(half)
    qx, qy, qz, qw = ux * s, uy * s, uz * s, math.cos(half)
    qn = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if qn < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / qn, qy / qn, qz / qn, qw / qn

def _normalize_quat(qx: float, qy: float, qz: float, qw: float):
    qn = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if qn < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / qn, qy / qn, qz / qn, qw / qn


def _quat_to_axis_angle(qx: float, qy: float, qz: float, qw: float, units: str = "deg"):
    """Convert (x,y,z,w) quaternion to (ax, ay, az, angle). Inverse of _axis_angle_to_quat."""
    qx, qy, qz, qw = _normalize_quat(qx, qy, qz, qw)
    qw_clamped = max(-1.0, min(1.0, qw))
    theta = 2.0 * math.acos(qw_clamped)
    s = math.sqrt(max(0.0, 1.0 - qw_clamped * qw_clamped))
    if s < 1e-6:
        ax, ay, az = 0.0, 0.0, 1.0
    else:
        ax, ay, az = qx / s, qy / s, qz / s
    angle = math.degrees(theta) if units == "deg" else theta
    return ax, ay, az, angle


def _quat_to_euler_xyz(qx: float, qy: float, qz: float, qw: float, units: str = "deg"):
    """Convert (x,y,z,w) quaternion to (theta_x, theta_y, theta_z).

    Uses the x-y-z *extrinsic* convention the Kinova Web App reports
    orientation in, i.e. R = Rz(theta_z) @ Ry(theta_y) @ Rx(theta_x).
    """
    qx, qy, qz, qw = _normalize_quat(qx, qy, qz, qw)
    tx = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    # Clamp before asin: at +/-90 degrees of pitch the argument can land just
    # outside [-1, 1] through float error and raise a math domain error.
    ty = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    tz = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    if units == "deg":
        return math.degrees(tx), math.degrees(ty), math.degrees(tz)
    return tx, ty, tz
