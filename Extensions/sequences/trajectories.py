"""Trajectory templates: a line, a circle or a figure-eight traced from where the end effector is.

A trajectory is a stream of Cartesian targets, sent every 1/60 s, that the controller follows as
it goes (the way a tracking experiment streams a lemniscate). A sequence is a list of
waypoints the robot reaches one after the other. ``samples`` gives the stream; ``waypoints``
gives the same path as a sequence, so the two can be compared.

Every path starts and ends at the start point at rest: the speed along the path ramps up over
the first second and down over the last (so it never exceeds the steady peak), and the
orientation is held. Pure functions: no ROS, no Qt.
"""
import math

KINDS = {'line': 'Line (back and forth)', 'circle': 'Circle', 'eight': 'Figure-eight'}
PLANES = {'xy': (0, 1), 'xz': (0, 2), 'yz': (1, 2)}      # (first axis, second axis) of the base frame
RATE_HZ = 60.0
RAMP_S = 1.0
#: Limits: size (m), period (s), cycles, and peak speed (m/s) in simulation and on the real robot.
SIZE = (0.01, 0.15)
PERIOD = (2.0, 60.0)
CYCLES = (1, 10)
PEAK_SPEED = {'simulation': 0.30, 'hardware': 0.05}


def offset(kind, phase, size):
    """(a, b) offset in the plane at ``phase`` (cycles, 0..1 per cycle) for a path of ``size`` (m):
    the line's half-length, the circle's radius, the figure-eight's half-width. (0, 0) at phase 0."""
    w = 2.0 * math.pi * phase
    if kind == 'line':
        return size * math.sin(w), 0.0
    if kind == 'circle':
        return size * (math.cos(w) - 1.0), size * math.sin(w)
    if kind == 'eight':
        return size * math.sin(w), size * math.sin(w) * math.cos(w)
    raise ValueError('The trajectory is a line, a circle or a figure-eight')


def peak_speed(kind, size, period):
    """Largest speed along the path (m/s), without the ramp."""
    w = 2.0 * math.pi / period
    return {'line': size * w, 'circle': size * w, 'eight': size * w * math.sqrt(2.0)}[kind] if kind in KINDS else 0.0


def check(kind, size, period, cycles, plane, target='simulation'):
    """The arguments, validated and converted; ValueError says what is wrong."""
    if kind not in KINDS:
        raise ValueError('The trajectory is a line, a circle or a figure-eight')
    if plane not in PLANES:
        raise ValueError('The plane is xy, xz or yz')
    try:
        size, period, cycles = float(size), float(period), int(cycles)
    except (TypeError, ValueError):
        raise ValueError('Size, period and cycles are numbers')
    if not SIZE[0] <= size <= SIZE[1]:
        raise ValueError('Size %g to %g m' % SIZE)
    if not PERIOD[0] <= period <= PERIOD[1]:
        raise ValueError('Period %g to %g s' % PERIOD)
    if not CYCLES[0] <= cycles <= CYCLES[1]:
        raise ValueError('Cycles %d to %d' % CYCLES)
    limit = PEAK_SPEED['hardware' if target == 'hardware' else 'simulation']
    v = peak_speed(kind, size, period)
    if v > limit + 1e-9:
        raise ValueError('Too fast: %.0f mm/s at the fastest point (at most %.0f mm/s %s): make it '
                         'smaller or slower' % (v * 1e3, limit * 1e3,
                                                'on the real robot' if target == 'hardware' else 'in simulation'))
    return kind, size, period, cycles, plane


def _ramp_integral(u):
    """Integral of the smoothstep 3u^2 - 2u^3 from 0 to u (1/2 at u = 1)."""
    u = min(max(u, 0.0), 1.0)
    return u ** 3 - 0.5 * u ** 4


def duration(period, cycles):
    """Seconds from start to stop: the cycles plus the ramps (each ramp costs half its length)."""
    return period * cycles + RAMP_S


def phase(t, period, cycles):
    """Cycles done at time ``t``: the rate ramps smoothly from 0 to 1/period over RAMP_S, and back
    to 0 over the last RAMP_S, so the path ends at exactly ``cycles``."""
    w, T, D = 1.0 / period, RAMP_S, duration(period, cycles)
    if t <= 0.0:
        return 0.0
    if t >= D:
        return float(cycles)
    if t < T:
        return w * T * _ramp_integral(t / T)
    if t > D - T:
        return cycles - w * T * _ramp_integral((D - t) / T)
    return w * (T / 2.0 + (t - T))


def point(start, kind, size, plane, t, period, cycles=1):
    """The target position at time ``t`` (s) from ``start`` [x, y, z]."""
    a, b = offset(kind, phase(t, period, cycles), size)
    i, j = PLANES[plane]
    p = list(start)
    p[i] += a
    p[j] += b
    return p


def samples(start, kind, size, period, cycles, plane, rate=RATE_HZ):
    """[(t, [x, y, z])] at ``rate`` from 0 to the end; the last sample is the start point."""
    n = int(math.ceil(duration(period, cycles) * rate))
    return [(k / rate, point(start, kind, size, plane, k / rate, period, cycles)) for k in range(n + 1)]


def waypoints(start, axis_angle, kind, size, plane, per_cycle=16, hold=0.0):
    """One cycle as Cartesian sequence waypoints (full size, no ramp), ending at the start."""
    out = []
    i, j = PLANES[plane]
    for k in range(1, per_cycle + 1):
        a, b = offset(kind, k / per_cycle, size)
        p = list(start)
        p[i] += a
        p[j] += b
        out.append({'type': 'cartesian', 'position': [round(v, 5) for v in p],
                    'axis_angle': list(axis_angle), 'hold_time': hold})
    return out
