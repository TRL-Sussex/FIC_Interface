"""Measure how a robot's joints behave, for the simulator's hardware_model (robots_config.yaml).

The arm holds still for a few seconds (torque-sensor noise), then moves one joint at a time
through a slow sine about the pose it starts in, in Joint Impedance, twice as fast the second
time. Its measured joint torque less the arm model's inverse dynamics gives each joint's friction,
damping and rotor inertia (fic_studio/hardware_model/fit.py). The block is printed and saved
in the run folder; --write puts it into robots_config.yaml and the Studio's robot list.

Run it on the robot's PC with the controller running (FIC Studio, Hardware target), the arm in
free space with room for each joint to swing --amplitude-deg either way:

    python3 -m fic_studio.hardware_model --env hardware --robot KinovaGen3_7DOF --write

with the workspace sourced (from another computer: over ssh to the robot's PC). --env sim does
the same on the simulator, a check that the fit returns the values of the MuJoCo model.
"""
import argparse
import json
import os
import sys
import threading
import time

import numpy as np

from . import config_write, fit
from .model import ArmModel, git_sha, repo_root, robot_cfg

HOLD_S = 4.0
PERIODS_S = (8.0, 4.0)
CYCLES = 2


class Io:
    """Mode, joint targets and feedback (angles and measured torque, in the simulator's sign) of
    station `station` on the simulator or the robot, for any joint count."""

    def __init__(self, node, env, station, joint_names):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32, String
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        def _reliable(depth):
            return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        MODE_NAMES = {1: 'Joint Impedance'}
        self.env, self.names, self.n = env, list(joint_names), len(joint_names)
        r = 'robot%d' % station
        self._lock = threading.Lock()
        self.samples = []                      # (t, q[n], tau[n])
        self._q = self._tau = None
        self.recording = False
        self._MODE_NAMES, self._Int32, self._String = MODE_NAMES, Int32, String
        self._JointState, self._F32 = JointState, Float32MultiArray
        if env == 'sim':
            self._mode = node.create_publisher(Int32, r + '_control_mode',
                                               qos_profile_sensor_data)
            self._joint = node.create_publisher(JointState, r + '_target_joint_angles',
                                                _reliable(10))
            node.create_subscription(JointState, '/%s_simulation/joint_states' % r, self._on_js,
                                     qos_profile_sensor_data)
        else:
            self._mode = node.create_publisher(String, r + '_mode_select', _reliable(10))
            self._joint = node.create_publisher(Float32MultiArray, r + '_desired_joint_angles',
                                                _reliable(10))
            node.create_subscription(Float64MultiArray, r + '_q_current_raw', self._on_q,
                                     qos_profile_sensor_data)
            node.create_subscription(Float64MultiArray, r + '_joint_torques', self._on_tau,
                                     qos_profile_sensor_data)

    def _store(self, stamp=None):
        # The simulator's message stamp when it has one (its samples are not always delivered at
        # an even pace), else the arrival time; analyse() resamples onto an even 1 ms grid.
        if self.recording and self._q is not None and self._tau is not None:
            self.samples.append((stamp or time.monotonic(), self._q.copy(), self._tau.copy()))

    def _on_js(self, m):
        idx = {nm: i for i, nm in enumerate(m.name)}
        if not all(nm in idx for nm in self.names) or len(m.effort) < len(m.name):
            return
        with self._lock:
            self._q = np.array([m.position[idx[nm]] for nm in self.names])
            self._tau = np.array([m.effort[idx[nm]] for nm in self.names])     # +g at rest
            st = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            self._store(st if st > 0 else None)

    def _on_q(self, m):
        with self._lock:
            self._q = np.array(m.data[:self.n], float)
            self._store()

    def _on_tau(self, m):
        with self._lock:
            self._tau = -np.array(m.data[:self.n], float)                     # -joint_torques
            self._store()

    def q(self):
        with self._lock:
            return None if self._q is None else self._q.copy()

    def mode(self, mode):
        msg = (self._Int32(data=mode) if self.env == 'sim'
               else self._String(data=self._MODE_NAMES[mode]))
        for _ in range(1 if self.env == 'hardware' else 10):
            self._mode.publish(msg)
            time.sleep(0.05)

    def target(self, q):
        if self.env == 'sim':
            self._joint.publish(self._JointState(position=[float(x) for x in q]))
        else:
            self._joint.publish(self._F32(data=[float(x) for x in q]))

    def take(self):
        with self._lock:
            s, self.samples = self.samples, []
        return s


def sweep(io, q0, j, amp, period, cycles, rate=100.0):
    """Joint j through amp*sin about q0[j] (others held), smoothly started and ended."""
    t0 = time.monotonic()
    dur = period * cycles
    while True:
        t = time.monotonic() - t0
        if t > dur:
            break
        q = q0.copy()
        q[j] += amp * np.sin(2 * np.pi * t / period)
        io.target(q)
        time.sleep(1.0 / rate)
    io.target(q0)


def _even(seg, dt=0.001):
    """(t, Q, T) of a recording resampled onto an even grid."""
    t = np.array([s[0] for s in seg])
    Q = np.unwrap(np.array([s[1] for s in seg]), axis=0)
    T = np.array([s[2] for s in seg])
    keep = np.r_[True, np.diff(t) > 0]
    t, Q, T = t[keep], Q[keep], T[keep]
    tg = np.arange(t[0], t[-1], dt)

    def interp(X):
        return np.column_stack([np.interp(tg, t, X[:, k]) for k in range(X.shape[1])])
    return tg, interp(Q), interp(T)


def _residual(seg, am, j):
    """(qd, qdd, r) of joint j in one recording: r = measured torque - inverse dynamics."""
    t, Q, T = _even(seg)
    dt = 0.001
    Qs, Qd = fit.savgol(Q, dt), fit.savgol(Q, dt, deriv=1)
    Qdd = fit.savgol(Q, dt, window_s=0.15, deriv=2)
    ks = range(0, len(t), 5)
    return (np.array([Qd[k, j] for k in ks]), np.array([Qdd[k, j] for k in ks]),
            np.array([T[k, j] - am.rnea(Qs[k], Qd[k], Qdd[k])[j] for k in ks]))


def analyse(segments, rest, am, n, filter_hz, source, armature=False):
    """Per joint: one fit over its recordings, kept only when each recording's own fit agrees
    with it (fit.consistent); a joint that fails is reported and left out (None)."""
    fits, notes = [], {}
    for j in range(n):
        parts = [_residual(seg, am, j) for seg in segments.get(j, []) if len(seg) >= 200]
        if not parts:
            fits.append(None)
            continue
        whole = fit.fit_joint(*(np.concatenate(x) for x in zip(*parts)))
        each = [fit.fit_joint(*p) for p in parts]
        why = fit.consistent(whole, each)
        if why:
            notes[j] = why
            fits.append(None)
        else:
            fits.append(whole)
    if len(rest) > 50:
        t, _, T = _even(rest)
        noise = fit.rest_noise(T, 0.001)
    else:
        noise = np.zeros(n)
    return fits, noise, fit.block(fits, noise, filter_hz, source, n, armature), notes


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--env', choices=('sim', 'hardware'), required=True)
    ap.add_argument('--robot', default='KinovaGen3_7DOF',
                    help='robot name in robots_config.yaml')
    ap.add_argument('--station', type=int, default=1)
    ap.add_argument('--joints', default='',
                    help='joints to measure, 1-based, e.g. 1,2,4 (default all)')
    ap.add_argument('--amplitude-deg', type=float, default=10.0,
                    help='swing either way (default 10)')
    ap.add_argument('--out', default=os.path.expanduser('~/.fic_studio/hardware_model'),
                    help='run folders go here (default ~/.fic_studio/hardware_model)')
    ap.add_argument('--write', action='store_true', help='write the block into the robot files')
    ap.add_argument('--fit-armature', action='store_true',
                    help='also write the rotor-inertia fit (it takes up URDF/MuJoCo inertia '
                         'differences too)')
    ap.add_argument('--yes', action='store_true',
                    help='skip the hardware confirmation (not advised)')
    a = ap.parse_args(argv)

    repo = repo_root()
    cfg = robot_cfg(repo, a.robot)
    am = ArmModel(repo, cfg)
    names = cfg['joint_names']
    n = len(names)
    joints = [int(x) - 1 for x in a.joints.split(',') if x.strip()] or list(range(n))
    amp = np.radians(a.amplitude_deg)
    if a.env == 'hardware':
        filter_hz = 200.0                       # fic_control.h kTorqueCutoffHz
    else:
        filter_hz = float((cfg.get('hardware_model') or {}).get('torque_filter_hz') or 10.0)
    peak = amp * 2 * np.pi / min(PERIODS_S)
    total = HOLD_S + len(joints) * (PERIODS_S[0] + 1 + sum(p * CYCLES + 2 for p in PERIODS_S))
    print('%s on %s: joints %s, %.0f deg either way, periods %s s (peak %.2f rad/s), %.0f s in all'
          % (a.robot, a.env, [j + 1 for j in joints], a.amplitude_deg, PERIODS_S, peak, total),
          flush=True)
    if a.env == 'hardware' and not a.yes:
        print('The arm switches to Joint Impedance and swings one joint at a time %.0f deg either '
              'way about where it is now. Clear its reach, keep the e-stop in hand.'
              % a.amplitude_deg)
        if input('Ready? Type y to start: ').strip().lower() != 'y':
            print('Not started.')
            return 1

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('hardware_model_measure')
    io = Io(node, a.env, a.station, names)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t_end = time.monotonic() + 5.0
    while io.q() is None and time.monotonic() < t_end:
        time.sleep(0.05)
    if io.q() is None:
        print('No joint feedback: is the controller running on station %d?' % a.station)
        return 2
    q0 = io.q()
    lim = cfg.get('joint_limits_deg') or {}
    for j in joints:
        lo, hi = ((np.radians(lim['lower'][j]), np.radians(lim['upper'][j]))
                  if lim.get('lower') else (-1e9, 1e9))
        if not (lo + 0.05 < q0[j] - amp and q0[j] + amp < hi - 0.05):
            print('Joint %d is too close to its limit to swing %.0f deg; move it or lower '
                  '--amplitude-deg.' % (j + 1, a.amplitude_deg))
            return 2
    segments, rest = {}, []
    try:
        io.mode(1)
        io.target(q0)
        time.sleep(2.0)
        io.take()
        io.recording = True
        time.sleep(HOLD_S)
        rest = io.take()
        for j in joints:
            sweep(io, q0, j, amp, PERIODS_S[0], 1)        # warm-up, not recorded
            time.sleep(1.0)
            for period in PERIODS_S:
                print('joint %d, period %.0f s' % (j + 1, period), flush=True)
                io.take()
                sweep(io, q0, j, amp, period, CYCLES)
                segments.setdefault(j, []).append(io.take())
                time.sleep(2.0)
                io.take()
    finally:
        io.recording = False
        io.target(q0)
        time.sleep(1.0)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    source = 'measured on %s (%s), %s' % (a.env, a.robot, time.strftime('%Y-%m-%d'))
    fits, noise, blk, notes = analyse(segments, rest, am, n, filter_hz, source, a.fit_armature)
    old = cfg.get('hardware_model') or {}
    good = [j for j in joints if fits[j] is not None]
    # joints not measured, or not used: as they were
    for key in ('joint_friction', 'joint_damping', 'armature'):
        if key in blk and len(old.get(key) or []) == n:
            blk[key] = [blk[key][j] if j in good else old[key][j] for j in range(n)]
    folder = os.path.join(os.path.expanduser(a.out), '%s-%s-%s' % (a.robot, a.env, stamp))
    os.makedirs(folder, exist_ok=True)
    np.savez_compressed(os.path.join(folder, 'recording.npz'),
                        rest=np.array([np.r_[s[0], s[1], s[2]] for s in rest]),
                        **{'joint%d_%d' % (j + 1, k):
                           np.array([np.r_[s[0], s[1], s[2]] for s in seg])
                           for j, segs in segments.items() for k, seg in enumerate(segs)})
    with open(os.path.join(folder, 'hardware_model.json'), 'w') as f:
        json.dump({'robot': a.robot, 'env': a.env, 'commit': git_sha(repo), 'fits': fits,
                   'noise': [float(x) for x in noise], 'block': blk,
                   'refused': {str(j + 1): w for j, w in notes.items()}}, f, indent=1)
    print('\njoint  friction N m  damping N m s/rad  armature kg m^2  fit R^2')
    for j, ft in enumerate(fits):
        if ft:
            row = '%9.3f  %15.3f  %15.4f  %7.2f' % (ft['friction'], ft['damping'],
                                                    ft['armature'], ft['r2'])
        else:
            row = ('NOT USED: ' + notes[j]) if j in notes else 'not measured'
        print('  %d    %s' % (j + 1, row))
    if notes:
        print('Joints marked NOT USED keep their previous values; measure them again '
              '(--joints %s).' % ','.join(str(j + 1) for j in notes))
    print('\n' + config_write.render(blk), end='')
    print('saved in', folder)
    missing = [j for j in range(n)
               if fits[j] is None and len(old.get('joint_friction') or []) != n]
    if a.write and missing:
        print('Not written: joint%s %s %s no value (not measured or not used) and no previous '
              'one to keep.' % ('' if len(missing) == 1 else 's',
                                ', '.join(str(j + 1) for j in missing),
                                'has' if len(missing) == 1 else 'have'))
    elif a.write:
        for p in config_write.write(repo, a.robot, blk):
            print('written to', p)
        print('Save the robot in the Studio or restart the simulator to use it.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
