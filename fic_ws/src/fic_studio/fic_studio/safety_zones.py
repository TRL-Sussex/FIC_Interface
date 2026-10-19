"""Whole-arm safety zones: shapes the arm must stay inside (keep-in) or out of (keep-out).

The zones and the arm's size live in a robot's ``safety_zones`` next to the box workspace:

    zones:                     # each one
      - name: Bench
        shape: box             # box | sphere | cylinder | prism (an outline drawn on the floor, raised)
        keep: out              # out: the arm stays out of it; in: inside it (with several, inside any one)
        enabled: true
        centre: [0.5, 0.0, 0.2]           # box, sphere; cylinder: [x, y]
        size: [0.4, 0.6, 0.4]             # box, full sizes in m
        yaw_deg: 0                        # box, turn about z
        radius: 0.1                       # sphere, cylinder
        z_min: 0.0                        # cylinder, prism
        z_max: 0.8
        points: [[x, y], ...]             # prism corners, in order, at least three
    body:
      radius: 0.05             # m, each link of the arm
      tool_radius: 0.03        # m, the tool point
      from_joint: 2            # the first joint checked (1-based): the first link turns about its own axis

Every robot's controller checks the zones along the arm, from that joint to the tool, and holds the arm out
of them (fic_library/include/fic/safety_zones.h); it reports the arm's worst clearance on
robotN_safety/zone_state, which the Studio's monitor and "Stop on violation" read. This module checks what a
user or a file gives and encodes it for robotN_safety/zones.
"""
import math

SHAPES = ('box', 'sphere', 'cylinder', 'prism')
SHAPE_CODE = {'box': 0, 'sphere': 1, 'cylinder': 2, 'prism': 3}
MAX_ZONES = 16
MAX_POINTS = 32
MAX_ABS_M = 5.0
MAX_NAME = 40
#: A zone within this of the arm is shown as near (amber) on the Safety page.
WARN_BAND_M = 0.05
DEFAULT_BODY = {'radius': 0.05, 'tool_radius': 0.03, 'from_joint': 2}


def _num(v, what, lo=-MAX_ABS_M, hi=MAX_ABS_M):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError('%s must be a number' % what)
    if not lo <= v <= hi:
        raise ValueError('%s must be %g to %g' % (what, lo, hi))
    return float(v)


def _vec(v, n, what, lo=-MAX_ABS_M, hi=MAX_ABS_M):
    if not isinstance(v, (list, tuple)) or len(v) != n:
        raise ValueError('%s must be %d numbers' % (what, n))
    return [_num(x, what, lo, hi) for x in v]


def _crosses(a, b, c, d):
    """True if segments ab and cd cross (touching ends do not count)."""
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return o1 * o2 < 0 and o3 * o4 < 0


def check_zone(z, i=0):
    """A clean copy of one zone, or ValueError saying what is wrong."""
    if not isinstance(z, dict):
        raise ValueError('Zone %d must be an object' % (i + 1))
    shape = z.get('shape')
    if shape not in SHAPES:
        raise ValueError('Zone %d: shape must be one of %s' % (i + 1, ', '.join(SHAPES)))
    name = str(z.get('name') or '%s %d' % (shape.capitalize(), i + 1)).strip()[:MAX_NAME]
    keep = z.get('keep', 'out')
    if keep not in ('in', 'out'):
        raise ValueError('Zone %d: keep must be "in" or "out"' % (i + 1))
    enabled = z.get('enabled', True)
    if not isinstance(enabled, bool):
        raise ValueError('Zone %d: enabled must be true or false' % (i + 1))
    out = {'name': name, 'shape': shape, 'keep': keep, 'enabled': enabled}
    label = 'Zone %d (%s)' % (i + 1, name)
    if shape == 'box':
        out['centre'] = _vec(z.get('centre'), 3, label + ' centre')
        out['size'] = _vec(z.get('size'), 3, label + ' size', 0.01, 2 * MAX_ABS_M)
        out['yaw_deg'] = _num(z.get('yaw_deg', 0.0), label + ' turn', -360.0, 360.0)
    elif shape == 'sphere':
        out['centre'] = _vec(z.get('centre'), 3, label + ' centre')
        out['radius'] = _num(z.get('radius'), label + ' radius', 0.01, MAX_ABS_M)
    else:
        if shape == 'cylinder':
            out['centre'] = _vec(z.get('centre'), 2, label + ' centre')
            out['radius'] = _num(z.get('radius'), label + ' radius', 0.01, MAX_ABS_M)
        else:
            pts = z.get('points')
            if not isinstance(pts, (list, tuple)) or not 3 <= len(pts) <= MAX_POINTS:
                raise ValueError('%s needs 3 to %d corners' % (label, MAX_POINTS))
            pts = [_vec(p, 2, label + ' corner') for p in pts]
            m = len(pts)
            for a in range(m):
                for b in range(a + 2, m):
                    if a == 0 and b == m - 1:
                        continue
                    if _crosses(pts[a], pts[(a + 1) % m], pts[b], pts[(b + 1) % m]):
                        raise ValueError('%s: its outline crosses itself' % label)
            area = 0.5 * sum(pts[k - 1][0] * pts[k][1] - pts[k][0] * pts[k - 1][1] for k in range(len(pts)))
            if abs(area) < 1e-4:
                raise ValueError('%s: its outline encloses no area' % label)
            out['points'] = pts
        out['z_min'] = _num(z.get('z_min'), label + ' bottom')
        out['z_max'] = _num(z.get('z_max'), label + ' top')
        if out['z_max'] - out['z_min'] < 0.01:
            raise ValueError('%s: the top must be at least 1 cm above the bottom' % label)
    return out


def check_zones(zones):
    if not isinstance(zones, list):
        raise ValueError('zones must be a list')
    if len(zones) > MAX_ZONES:
        raise ValueError('At most %d zones' % MAX_ZONES)
    return [check_zone(z, i) for i, z in enumerate(zones)]


def check_body(body, n_joints=None):
    if not isinstance(body, dict):
        raise ValueError('body must be an object')
    out = dict(DEFAULT_BODY)
    for key in ('radius', 'tool_radius'):
        if key in body:
            out[key] = _num(body[key], 'Arm ' + key.replace('_', ' '), 0.0, 0.3)
    if 'from_joint' in body:
        j = body['from_joint']
        if isinstance(j, bool) or not isinstance(j, int) or j < 1 or (n_joints and j > n_joints):
            raise ValueError('The first joint checked must be 1 to %s' % (n_joints or 'the joint count'))
        out['from_joint'] = j
    return out


def encode(zones, body):
    """robotN_safety/zones: [1, radius, tool_radius, from_joint (0-based), n, zone...] as safety_zones.h reads."""
    body = dict(DEFAULT_BODY, **(body or {}))
    zones = zones or []
    out = [1.0, float(body['radius']), float(body['tool_radius']), float(max(int(body['from_joint']) - 1, 0)),
           float(len(zones))]
    for z in zones:
        s = z['shape']
        if s == 'box':
            a = list(z['centre']) + [v / 2.0 for v in z['size']] + [math.radians(z.get('yaw_deg', 0.0))]
        elif s == 'sphere':
            a = list(z['centre']) + [z['radius']]
        elif s == 'cylinder':
            a = list(z['centre']) + [z['radius'], z['z_min'], z['z_max']]
        else:
            a = [z['z_min'], z['z_max']] + [c for p in z['points'] for c in p]
        out += [float(SHAPE_CODE[s]), 1.0 if z.get('keep') == 'in' else 0.0,
                1.0 if z.get('enabled', True) else 0.0, float(len(a))] + [float(v) for v in a]
    return out


def zone_state(msg_data, zones, stamp, now, stale_s=1.0):
    """A robot's robotN_safety/zone_state as the Safety page shows it: None if none recent or no zone is on.

    {'clearance': m (negative: past a boundary), 'zone': index, 'name', 'point': [x, y, z],
     'past': segments past a zone, 'state': 'ok' | 'warn' | 'violation'}"""
    if msg_data is None or stamp is None or now - stamp > stale_s or len(msg_data) < 7:
        return None
    if not any(z.get('enabled', True) for z in zones or []):
        return None
    c, zi = float(msg_data[0]), int(msg_data[1])
    if not math.isfinite(c):
        return None
    name = zones[zi]['name'] if 0 <= zi < len(zones or []) else ''
    return {'clearance': c, 'zone': zi, 'name': name, 'past': int(msg_data[3]),
            'point': [float(v) for v in msg_data[4:7]],
            'state': 'violation' if c < 0 else 'warn' if c < WARN_BAND_M else 'ok'}
