"""The follower's maths: leader motion in, safe joint targets out.

Relative, not absolute: at the start the follower stays exactly where it is,
and from then on it copies how far the leader has moved (times ``scale``).
Two arms never have to be lined up first, and starting never jumps.

Each step, per joint:
  1. the leader's motion since the start, accumulated from wrapped increments
     so a continuous joint crossing +-pi does not look like a 2*pi jump
  2. desired = follower start + scale * motion
  3. clamped to the follower's joint limits
  4. smoothed by a first-order filter of time constant ``tau``: a property of
     the arm, not of how often this is called, so the stream rate can change
     without changing how the follower behaves
  5. capped at ``max_speed`` rad/s from the last target sent
  6. not sent at all while every joint's target is within ``deadband`` of
     where it should be (so a still leader costs no traffic, and the
     follower always settles within ``deadband`` of the true target)
"""
import math

#: The tick ``alpha`` is quoted against. ``alpha`` is the fraction of the
#: remaining error a single tick removes, so it only means something once you
#: say how long a tick is; the Studio's teleop timer has been 10 ms.
REF_DT_S = 0.010

#: Clamp on the measured ``dt``. The caller derives it from wall-clock
#: differences of a GUI timer, so a stalled tick can arrive with dt an order of
#: magnitude too large. Unclamped that granted a proportionally larger slew cap
#: for exactly the tick that followed a stall, which is the worst moment for it.
MIN_DT_S, MAX_DT_S = 1e-4, 0.050


def _tau_from_alpha(alpha, ref_dt=REF_DT_S):
    """Return the time constant an ``alpha`` per ``ref_dt`` tick corresponds to.

    ``alpha = 1`` means take the whole step every tick, i.e. no filter at all;
    that is tau = 0, and it stays no-filter at every rate.
    """
    if alpha >= 1.0:
        return 0.0
    return -float(ref_dt) / math.log(1.0 - float(alpha))


def _wrap(a):
    """Return ``a`` wrapped to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class JointFollower:
    """Turn a stream of leader joint positions into follower targets (radians)."""

    def __init__(self, n_joints, lower=None, upper=None, scale=1.0, alpha=0.4,
                 deadband=0.002, max_speed=0.6, tau=None, ref_dt=REF_DT_S):
        """Configure for ``n_joints`` joints.

        ``lower`` / ``upper``: the follower's joint limits in radians (None:
        unlimited). ``max_speed``: rad/s any joint target may move.
        """
        if n_joints < 1:
            raise ValueError('n_joints must be at least 1')
        if not 0.0 < alpha <= 1.0:
            raise ValueError('alpha must be in (0, 1]')
        self.n = n_joints
        self.lower = list(lower) if lower else [-math.inf] * n_joints
        self.upper = list(upper) if upper else [math.inf] * n_joints
        if len(self.lower) != n_joints or len(self.upper) != n_joints:
            raise ValueError('limits must have one value per joint')
        self.scale = float(scale)
        self.alpha = float(alpha)
        # Smoothing is held as a time constant and the per-tick fraction is
        # derived from the real dt, so that calling this 10x more often filters
        # the same amount rather than 10x less. Quoting it as ``alpha`` keeps
        # every existing caller working: at a ``ref_dt`` tick the derived
        # fraction is the alpha that was passed in, exactly.
        self.tau = _tau_from_alpha(alpha, ref_dt) if tau is None else float(tau)
        if self.tau < 0.0:
            raise ValueError('tau must not be negative')
        self.deadband = float(deadband)
        self.max_speed = float(max_speed)
        self.started = False

    def start(self, leader_q, follower_q):
        """Anchor both arms where they are now. The follower's first target is itself."""
        leader_q, follower_q = self._check(leader_q), self._check(follower_q)
        self._leader_prev = list(leader_q)
        self._motion = [0.0] * self.n
        self._follower_home = list(follower_q)
        self._filtered = list(follower_q)
        self.last_target = list(follower_q)
        self.started = True

    def step(self, leader_q, dt):
        """Advance by ``dt`` seconds with the leader at ``leader_q``.

        Return the new follower target, or None when it would not move
        (within the deadband). Call ``start`` first.
        """
        if not self.started:
            raise RuntimeError('call start() first')
        leader_q = self._check(leader_q)
        for i in range(self.n):
            self._motion[i] += _wrap(leader_q[i] - self._leader_prev[i])
        self._leader_prev = list(leader_q)

        dt_eff = min(max(float(dt), MIN_DT_S), MAX_DT_S)
        cap = self.max_speed * dt_eff
        # exp(), not a constant: dt is measured, not assumed.
        a = 1.0 if self.tau <= 0.0 else 1.0 - math.exp(-dt_eff / self.tau)
        target, remaining = [], 0.0
        for i in range(self.n):
            desired = self._follower_home[i] + self.scale * self._motion[i]
            desired = min(max(desired, self.lower[i]), self.upper[i])
            self._filtered[i] += a * (desired - self._filtered[i])
            last = self.last_target[i]
            remaining = max(remaining, abs(desired - last))
            target.append(last + min(max(self._filtered[i] - last, -cap), cap))

        if remaining <= self.deadband:
            return None
        self.last_target = target
        return list(target)

    def error(self, follower_q):
        """Return the largest |target - actual| over the joints (rad), or None."""
        if not self.started or not follower_q or len(follower_q) < self.n:
            return None
        return max(abs(_wrap(t - q)) for t, q in zip(self.last_target, follower_q))

    def _check(self, q):
        if q is None or len(q) < self.n:
            raise ValueError('expected %d joint positions, got %s'
                             % (self.n, 'none' if q is None else len(q)))
        values = [float(v) for v in q[:self.n]]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('joint positions must be finite')
        return values
