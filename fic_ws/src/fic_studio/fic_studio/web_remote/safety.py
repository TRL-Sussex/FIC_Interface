"""Safety zones and limits for the web UI: read the current state, edit it.

The state is the Studio's own (``safety_limits``, ``safety_enabled``,
``safety_enforce``); an edit goes through the same two steps as the desktop
Safety page (store, then ``_publish_safety_limits`` to every robot's
controller). What is added here is checking, because a web form can send
anything: every number must be finite and sane, a lower bound must be under
its upper bound, joint safety limits must lie inside the joint's mechanical
range, and nothing is applied without an explicit ``confirm: true`` (the page
shows the change and asks again).
"""
import copy
import math

from fic_studio.web_remote.common import Refused, _num, _who, need_confirm

AXES = ('x', 'y', 'z')
#: Cartesian zone bounds are workspace coordinates in metres.
MAX_CART_ABS_M = 5.0
MAX_JOINT_VEL = 10.0          # rad/s
MAX_CART_VEL = 5.0            # m/s

DEFAULT_LIMITS = {
    'cartesian': {'x': {'min': -1.0, 'max': 1.0, 'enabled': True},
                  'y': {'min': -1.0, 'max': 1.0, 'enabled': True},
                  'z': {'min': 0.0, 'max': 1.5, 'enabled': True}},
    'joint_safety_deg': {'lower': [], 'upper': [], 'enabled': True},
    'velocity': {'max_joint_vel': 1.0, 'max_cart_vel': 0.5, 'enabled': True},
}


def axis_state(value, lim, threshold):
    """Say where ``value`` is against a zone, as the Studio's monitor does.

    Returns 'off' (zone disabled or no value), 'ok', 'warn' (inside the outer
    ``1 - threshold`` fraction of the range) or 'violation'.
    """
    if value is None or not lim.get('enabled', False):
        return 'off'
    lo, hi = lim.get('min', -math.inf), lim.get('max', math.inf)
    span = hi - lo
    if value < lo or value > hi:
        return 'violation'
    if value < lo + span * (1 - threshold) or value > hi - span * (1 - threshold):
        return 'warn'
    return 'ok'


def _merged(w):
    """The Studio's limits with every key present."""
    limits = copy.deepcopy(DEFAULT_LIMITS)
    have = getattr(w, 'safety_limits', None) or {}
    for key, section in have.items():
        if isinstance(section, dict) and isinstance(limits.get(key), dict):
            for k, v in section.items():
                if isinstance(v, dict) and isinstance(limits[key].get(k), dict):
                    limits[key][k].update(v)
                else:
                    limits[key][k] = v
    return limits


def _mechanical(w):
    """Mechanical joint limits (deg) from the robot's config, or (None, None)."""
    try:
        robot = w.robot_dropdown.currentText()
        cfg = getattr(w, 'robot_configs', {}).get(robot, {})
        mech = cfg.get('joint_limits_deg') or {}
        return list(mech.get('lower') or []), list(mech.get('upper') or [])
    except Exception:
        return [], []


def _joint_count(w, mech_lo):
    count = getattr(w, '_safety_joint_count', None)
    if callable(count):
        try:
            return int(count())
        except Exception:
            pass
    return len(mech_lo) or 7


def state(w, ctx=None):
    """The Safety page's data: limits, live values and where each stands."""
    limits = _merged(w)
    thr = float(getattr(w, 'warning_threshold', 0.9))
    pose = None
    try:
        pose = w.ee_pose_for(ctx['rid']) if ctx else getattr(w, 'current_ee_pose', None)
    except Exception:
        pose = None
    ee = None
    if pose is not None and hasattr(pose, 'position'):
        ee = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z}
    cart = {}
    for axis in AXES:
        lim = limits['cartesian'][axis]
        value = _num(ee[axis]) if ee else None
        cart[axis] = {'min': lim['min'], 'max': lim['max'], 'enabled': bool(lim['enabled']),
                      'value': value, 'state': axis_state(value, lim, thr)}
    mech_lo, mech_hi = _mechanical(w)
    n = _joint_count(w, mech_lo)
    joint = limits['joint_safety_deg']
    actual = []
    if ctx is not None:
        try:
            actual = [None if v is None else math.degrees(v)
                      for v in w.joint_positions_for(ctx['rid'])]
        except Exception:
            actual = []
    rows = []
    for i in range(n):
        lo = joint['lower'][i] if i < len(joint['lower']) else None
        hi = joint['upper'][i] if i < len(joint['upper']) else None
        value = _num(actual[i]) if i < len(actual) else None
        if lo is not None and hi is not None:
            st = axis_state(value, {'min': lo, 'max': hi, 'enabled': joint.get('enabled', True)}, thr)
        else:
            st = 'off'
        rows.append({'label': 'Joint %d' % (i + 1), 'lower': lo, 'upper': hi, 'value': value,
                     'mech_lower': _num(mech_lo[i]) if i < len(mech_lo) else None,
                     'mech_upper': _num(mech_hi[i]) if i < len(mech_hi) else None, 'state': st})
    states = [c['state'] for c in cart.values()] + [r['state'] for r in rows]
    enabled = bool(getattr(w, 'safety_enabled', True))
    overall = ('off' if not enabled else 'violation' if 'violation' in states
               else 'warn' if 'warn' in states else 'ok')
    return {
        'enabled': enabled,
        'enforce': bool(getattr(w, 'safety_enforce', True)),
        'warning_threshold': thr,
        'overall': overall,
        'cartesian': cart,
        'joints': {'enabled': bool(joint.get('enabled', True)), 'rows': rows},
        'velocity': dict(limits['velocity']),
        'bounds': {'cart_abs': MAX_CART_ABS_M, 'joint_vel': MAX_JOINT_VEL, 'cart_vel': MAX_CART_VEL},
    }


# ── Editing ──────────────────────────────────────────────────────────────

def _flag(value, what):
    if not isinstance(value, bool):
        raise Refused('%s must be true or false' % what, 'bad_args')
    return value


def _finite(value, what):
    number = _num(value)
    if number is None or isinstance(value, bool):
        raise Refused('%s must be a number' % what, 'bad_args')
    return number


def _check_cartesian(edit, current):
    if not isinstance(edit, dict):
        raise Refused('cartesian must be an object', 'bad_args')
    out = copy.deepcopy(current)
    for axis, vals in edit.items():
        if axis not in AXES or not isinstance(vals, dict):
            raise Refused('No such Cartesian axis: %s' % axis, 'bad_args')
        zone = out[axis]
        for key, value in vals.items():
            if key in ('min', 'max'):
                zone[key] = _finite(value, '%s %s' % (axis.upper(), key))
                if abs(zone[key]) > MAX_CART_ABS_M:
                    raise Refused('%s %s is beyond +/-%.0f m' % (axis.upper(), key, MAX_CART_ABS_M),
                                  'bad_args')
            elif key == 'enabled':
                zone[key] = _flag(value, '%s enabled' % axis.upper())
            else:
                raise Refused('Unknown field %s' % key, 'bad_args')
        if zone['min'] >= zone['max']:
            raise Refused('%s: min must be below max' % axis.upper(), 'bad_args')
    return out


def _check_joints(edit, current, n, mech_lo, mech_hi):
    if not isinstance(edit, dict):
        raise Refused('joints must be an object', 'bad_args')
    out = copy.deepcopy(current)
    for key in edit:
        if key not in ('lower', 'upper', 'enabled'):
            raise Refused('Unknown field %s' % key, 'bad_args')
    if 'enabled' in edit:
        out['enabled'] = _flag(edit['enabled'], 'joint safety enabled')
    if 'lower' in edit or 'upper' in edit:
        lower = edit.get('lower', out['lower'])
        upper = edit.get('upper', out['upper'])
        if (not isinstance(lower, list) or not isinstance(upper, list)
                or len(lower) != n or len(upper) != n):
            raise Refused('Give %d lower and %d upper joint limits' % (n, n), 'bad_args')
        lo = [_finite(v, 'Joint %d lower' % (i + 1)) for i, v in enumerate(lower)]
        hi = [_finite(v, 'Joint %d upper' % (i + 1)) for i, v in enumerate(upper)]
        for i in range(n):
            if lo[i] >= hi[i]:
                raise Refused('Joint %d: lower must be below upper' % (i + 1), 'bad_args')
            if i < len(mech_lo) and i < len(mech_hi) and (
                    lo[i] < mech_lo[i] - 1e-6 or hi[i] > mech_hi[i] + 1e-6):
                raise Refused('Joint %d: outside the mechanical range %.1f..%.1f deg'
                              % (i + 1, mech_lo[i], mech_hi[i]), 'bad_args')
        out['lower'], out['upper'] = lo, hi
    return out


def _check_velocity(edit, current):
    if not isinstance(edit, dict):
        raise Refused('velocity must be an object', 'bad_args')
    out = copy.deepcopy(current)
    for key, value in edit.items():
        if key == 'max_joint_vel':
            out[key] = _finite(value, 'Maximum joint velocity')
            if not 0 < out[key] <= MAX_JOINT_VEL:
                raise Refused('Maximum joint velocity must be above 0 and at most %g rad/s'
                              % MAX_JOINT_VEL, 'bad_args')
        elif key == 'max_cart_vel':
            out[key] = _finite(value, 'Maximum Cartesian velocity')
            if not 0 < out[key] <= MAX_CART_VEL:
                raise Refused('Maximum Cartesian velocity must be above 0 and at most %g m/s'
                              % MAX_CART_VEL, 'bad_args')
        elif key == 'enabled':
            out[key] = _flag(value, 'velocity limits enabled')
        else:
            raise Refused('Unknown field %s' % key, 'bad_args')
    return out


def _describe_change(old, new):
    """A short 'what changed' line for the log."""
    parts = []
    for axis in AXES:
        a, b = old['cartesian'][axis], new['cartesian'][axis]
        if a != b:
            parts.append('%s %.3f..%.3f%s -> %.3f..%.3f%s' % (
                axis.upper(), a['min'], a['max'], '' if a['enabled'] else ' (off)',
                b['min'], b['max'], '' if b['enabled'] else ' (off)'))
    if old['joint_safety_deg'] != new['joint_safety_deg']:
        parts.append('joint limits changed')
    if old['velocity'] != new['velocity']:
        v = new['velocity']
        parts.append('velocity %.2f rad/s, %.2f m/s%s' % (
            v['max_joint_vel'], v['max_cart_vel'], '' if v['enabled'] else ' (off)'))
    return '; '.join(parts) or 'no change'


def set_limits(w, args, client):
    """Apply an edit to the zones and limits. Needs ``confirm: true``."""
    need_confirm(args, 'Changing the safety limits needs confirmation')
    wanted = [k for k in ('cartesian', 'joints', 'velocity', 'monitor', 'enforce') if k in args]
    if not wanted:
        raise Refused('Nothing to change', 'bad_args')
    old = _merged(w)
    new = copy.deepcopy(old)
    mech_lo, mech_hi = _mechanical(w)
    if 'cartesian' in args:
        new['cartesian'] = _check_cartesian(args['cartesian'], old['cartesian'])
    if 'joints' in args:
        new['joint_safety_deg'] = _check_joints(
            args['joints'], old['joint_safety_deg'], _joint_count(w, mech_lo), mech_lo, mech_hi)
    if 'velocity' in args:
        new['velocity'] = _check_velocity(args['velocity'], old['velocity'])
    monitor = _flag(args['monitor'], 'monitor') if 'monitor' in args else None
    enforce = _flag(args['enforce'], 'enforce') if 'enforce' in args else None
    # All checks passed: apply, then publish, as the desktop page does.
    w.safety_limits = new
    if monitor is not None:
        w.safety_enabled = monitor
    if enforce is not None:
        w.safety_enforce = enforce
    w._publish_safety_limits()
    panel = getattr(w, '_safety_panel', None)
    if panel is not None:
        try:
            panel.from_config(new)
        except Exception:
            pass
    note = _describe_change(old, new)
    if monitor is not None:
        note += '; monitor %s' % ('on' if monitor else 'OFF')
    if enforce is not None:
        note += '; stop on violation %s' % ('on' if enforce else 'OFF')
    w.log_message('WARN', 'Safety limits edited from the web UI (%s): %s' % (_who(client), note),
                  'Safety')
    ctx = {'rid': w._rc_command_rid() if hasattr(w, '_rc_command_rid') else w.node.station_robot_id}
    return {'ok': True, 'safety': state(w, ctx), 'changed': note}
