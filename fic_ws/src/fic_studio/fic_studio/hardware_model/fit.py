"""Fit the robots_config.yaml `hardware_model` block from a recording (pure numpy, no ROS).

The joint torque the controller measures, in the simulator's sign (the torque the joint applies:
about +g(q) at rest; fic_hardware's joint_torques negated), less the arm model's inverse dynamics
M(q) qdd + C(q, qd) qd + g(q), leaves what the model does not have. On a swept joint that is

    r = friction * sign(qd) + damping * qd + armature * qdd + offset

(offset: the sensor's and the gravity model's bias). The simulator gives its joints these
friction, damping and armature values (MuJoCo frictionloss, damping, armature), so its measured
torque, the actuator's effort, carries the same terms. Noise: the standard deviation of the
measured torque at rest, after a slow trend is removed.
"""
import numpy as np

MIN_SPEED = 0.02        # rad/s: samples slower than this are left out of the friction fit


def savgol(x, dt, window_s=0.08, deriv=0):
    from scipy.signal import savgol_filter
    w = max(5, int(round(window_s / dt)) | 1)
    return savgol_filter(x, w, 3, deriv=deriv, delta=dt, axis=0)


def fit_joint(qd, qdd, r, min_speed=MIN_SPEED):
    """Least squares of r = c sign(qd) + b qd + a qdd + d over the samples moving faster than
    min_speed. Returns dict(friction=c, damping=b, armature=a, offset=d, r2, n)."""
    qd, qdd, r = (np.asarray(v, float) for v in (qd, qdd, r))
    m = np.abs(qd) > min_speed
    if m.sum() < 50:
        return None
    A = np.c_[np.sign(qd[m]), qd[m], qdd[m], np.ones(m.sum())]
    coef, *_ = np.linalg.lstsq(A, r[m], rcond=None)
    pred = A @ coef
    var = np.var(r[m])
    r2 = 1.0 - np.var(r[m] - pred) / var if var > 0 else 0.0
    c, b, a, d = (float(x) for x in coef)
    return {'friction': c, 'damping': b, 'armature': a, 'offset': d, 'r2': float(r2),
            'n': int(m.sum())}


def consistent(whole, each, rel=0.3, abs_friction=0.2, abs_damping=2.0, min_r2=0.9):
    """'' when the fits of each recording agree with the joint's fit over all of them (friction
    within max(abs_friction, rel), damping within max(abs_damping, rel)) and it explains the
    residual (R^2 >= min_r2); else why not."""
    if whole is None:
        return 'too little motion'
    if whole['r2'] < min_r2:
        return 'poor fit (R^2 %.2f)' % whole['r2']
    for f in each:
        if f is None:
            return 'a recording with too little motion'
        for key, a in (('friction', abs_friction), ('damping', abs_damping)):
            if abs(f[key] - whole[key]) > max(a, rel * abs(whole[key])):
                return 'the speeds disagree (%s %.2f vs %.2f)' % (key, f[key], whole[key])
    return ''


def rest_noise(tau, dt, trend_s=0.5):
    """Per-joint standard deviation of a rest recording (n x J) about its slow trend."""
    tau = np.asarray(tau, float)
    w = max(5, int(round(trend_s / dt)) | 1)
    if tau.shape[0] <= w:
        return np.std(tau, axis=0)
    from scipy.signal import savgol_filter
    trend = savgol_filter(tau, w, 1, axis=0)
    return np.std(tau - trend, axis=0)


def block(fits, noise, filter_hz, source, n, armature=False):
    """The hardware_model block from per-joint fits (None where a joint could not be fitted: 0
    friction and damping). Negative fits (noise on a nearly frictionless joint) are clipped to 0.
    The armature fit also takes up any difference between the URDF's link inertias and the MuJoCo
    model's (on the simulator it returned 0.06-0.34 kg m^2 for 0.01), so it goes into the block
    only with armature=True; otherwise the MuJoCo model keeps its own."""
    fr = [round(max(0.0, f['friction']), 3) if f else 0.0 for f in fits]
    dm = [round(max(0.0, f['damping']), 3) if f else 0.0 for f in fits]
    ar = [round(max(0.0, f['armature']), 4) if f else 0.0 for f in fits]
    out = {'enabled': True, 'source': source, 'joint_friction': fr, 'joint_damping': dm,
           'torque_noise': [round(float(x), 4) for x in noise][:n],
           'torque_filter_hz': float(filter_hz), 'command_delay_ms': 0.0}
    if armature and any(f is not None for f in fits):
        out['armature'] = ar
    return out
