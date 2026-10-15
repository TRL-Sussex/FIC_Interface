"""Trajectory templates: paths start and end where the arm is, stay within their size, ramp up
smoothly, and refuse sizes and speeds beyond the limits (lower on the real robot)."""
import math

import pytest

from Extensions.sequences import trajectories as tr

START = [0.45, 0.0, 0.43]


@pytest.mark.parametrize('kind', sorted(tr.KINDS))
@pytest.mark.parametrize('plane', sorted(tr.PLANES))
def test_paths_start_and_end_at_the_start_and_stay_within_their_size(kind, plane):
    pts = tr.samples(START, kind, 0.1, 8.0, 2, plane)
    assert pts[0][1] == pytest.approx(START) and pts[-1][1] == pytest.approx(START, abs=1e-9)
    assert pts[-1][0] == pytest.approx(16.0 + tr.RAMP_S, abs=1 / tr.RATE_HZ)
    i, j = tr.PLANES[plane]
    k = 3 - i - j
    for _, p in pts:
        assert p[k] == pytest.approx(START[k])                 # stays in its plane
        assert math.dist(p, START) <= 0.2 + 1e-9               # a circle's far side is 2 x radius away
    steps = [math.dist(a[1], b[1]) for a, b in zip(pts, pts[1:])]
    assert max(steps) * tr.RATE_HZ <= tr.peak_speed(kind, 0.1, 8.0) * 1.01   # never faster than the steady peak
    assert steps[0] * tr.RATE_HZ < 0.01 and steps[-1] * tr.RATE_HZ < 0.01         # starts and stops at rest


def test_the_phase_reaches_the_cycles_exactly_and_never_goes_back():
    ph = [tr.phase(k / 60.0, 8.0, 3) for k in range(int(tr.duration(8.0, 3) * 60) + 2)]
    assert ph[0] == 0.0 and ph[-1] == pytest.approx(3.0)
    assert all(b >= a for a, b in zip(ph, ph[1:]))


def test_limits_and_the_real_robot_is_slower():
    assert tr.check('circle', 0.1, 8.0, 2, 'xy')
    with pytest.raises(ValueError, match='Too fast'):
        tr.check('circle', 0.1, 8.0, 2, 'xy', target='hardware')      # 79 mm/s > 50 mm/s
    assert tr.check('circle', 0.05, 8.0, 1, 'xy', target='hardware')
    for bad in (('cone', 0.1, 8, 1, 'xy'), ('line', 0.5, 8, 1, 'xy'), ('line', 0.1, 1, 1, 'xy'),
                ('line', 0.1, 8, 0, 'xy'), ('line', 0.1, 8, 1, 'xw')):
        with pytest.raises(ValueError):
            tr.check(*bad)


def test_one_cycle_as_waypoints_ends_at_the_start():
    wps = tr.waypoints(START, [1, 0, 0, 180], 'eight', 0.08, 'xy', per_cycle=12)
    assert len(wps) == 12 and all(w['type'] == 'cartesian' for w in wps)
    assert wps[-1]['position'] == pytest.approx(START, abs=1e-5)
    assert max(abs(w['position'][0] - START[0]) for w in wps) == pytest.approx(0.08, abs=1e-3)
