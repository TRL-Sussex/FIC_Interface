"""Controller tuning for the web UI: config.xml, through the Studio's xml_manager.

Everything is read and written with ``fic_studio.xml_manager`` (the same
loaders and savers the desktop Tuning page uses), so the XML on disk is
produced by one code path. The page asks for the parameter set it is editing
(robot, mode, config profile, joint or axis) and gets the values together
with a description of the fields, so the browser draws the form without
knowing what a stiffness is. A save validates the numbers, writes the XML,
and tells the controller to reload (``_publish_param_update``).

A live trial (``trial``) lets the robot run edits as they are made without
touching the saved config: see ``TRIAL``.
"""
import dataclasses
import math

from fic_studio.web_remote.common import Refused, _num, _who, need_confirm, scoped_to_selected

CART_AXES = ('x', 'y', 'z', 'q')
SCOPED_AXES = ('elbow', 'postural')       # robot-wide: no mode, no profile

# (key, label, unit, type, group, min, max). Not diff_effort_max: config.xml still carries it, but no
# controller reads it, so the page does not offer it (a save keeps the stored value).
_CONTROLLER = [
    ('stiffness', 'Stiffness K0', '', 'number', 'controller', 0.0, 1e6),
    ('error_max', 'Error breakpoints', 'rad or m', 'list', 'controller', 0.0, 1e3),
    ('force_max', 'Force levels', 'N or N·m', 'list', 'controller', 0.0, 1e5),
    ('saturation_ratio', 'Saturation ratio', '', 'number', 'controller', 0.01, 0.999),
    ('n_saturations', 'Saturation points N_S', '', 'int', 'controller', 2, 8),
    ('asymmetric', 'Asymmetric', '', 'bool', 'controller', None, None),
    ('n_neg_sat', 'Negative-side points', '', 'int', 'controller', 1, 4),
]
_VELOCITY = ('velocity', 'Velocity limit', 'rad/s', 'number', 'controller', 0.0, 20.0)
# Not the planner's stiffness or frequency either: the planner derives its stiffness and does not
# read the frequency (FIC.cpp Planner). Acceleration max is a cap (0: none). The robot-wide
# frequency cap and speed margin are on the Robot Config page (robots_config.yaml fic_law).
_PLANNER_JOINT = [
    ('planner_vel_desired', 'Desired velocity', '', 'number', 'planner', 0.0, 50.0),
    ('planner_acc_max', 'Acceleration max', '', 'number', 'planner', 0.0, 1e6),
    ('planner_damping_ratio', 'Damping ratio', '', 'number', 'planner', 0.0, 10.0),
    ('planner_wrapped', 'Wrapped joint', '', 'choice', 'planner', 0, 1),
    ('planner_pos_lower', 'Position lower', 'rad', 'number', 'planner', -100.0, 100.0),
    ('planner_pos_upper', 'Position upper', 'rad', 'number', 'planner', -100.0, 100.0),
]
_PLANNER_AXIS = [
    ('planner_vel_desired', 'Desired velocity', '', 'number', 'planner', 0.0, 50.0),
    ('planner_acc_max', 'Acceleration max', '', 'number', 'planner', 0.0, 1e6),
    ('planner_damping_ratio', 'Damping ratio', '', 'number', 'planner', 0.0, 10.0),
]
_PLANNER_POSTURAL = list(_PLANNER_AXIS)
_ADMITTANCE = [
    ('admittance_inv_m', 'Inverse mass', '', 'number', 'admittance', 0.0, 1e6),
    ('admittance_damping', 'Damping', '', 'number', 'admittance', 0.0, 1e5),
    ('admittance_dflow', 'dFlow limits', '', 'list', 'admittance', -1e4, 1e4),
    ('admittance_flow', 'Flow limits', '', 'list', 'admittance', -1e4, 1e4),
    ('admittance_pose', 'Pose limits', '', 'list', 'admittance', -1e4, 1e4),
]
_KEYS = ('key', 'label', 'unit', 'type', 'group', 'min', 'max')


def _specs(kind, mode):
    """The fields of one parameter set. H-PMP (mode 100) has a joint layer (controller, planner,
    admittance) and a Cartesian layer (controller, planner: no admittance)."""
    from fic_studio import xml_manager as xm
    fields = list(_CONTROLLER)
    if kind == 'joint':
        if xm.mode_uses_planner(mode):
            fields.append(_VELOCITY)
            fields += _PLANNER_JOINT
        if xm.mode_uses_admittance(mode):
            fields += _ADMITTANCE
    elif kind == 'axis':
        if xm.mode_uses_planner(mode):
            fields += _PLANNER_AXIS
        if xm.mode_uses_admittance(mode) and not xm.is_hpmp_mode(mode):
            fields += _ADMITTANCE
    elif kind == 'postural':
        fields.append(_VELOCITY)
        fields += _PLANNER_POSTURAL
    return [dict(zip(_KEYS, f)) for f in fields]


def _xml(w):
    from fic_studio.xml_manager import XMLManager
    path = getattr(w, '_web_config_path', None)
    try:
        return XMLManager(path) if path else XMLManager()
    except FileNotFoundError as exc:
        raise Refused(str(exc), 'unavailable')


def _robot(w):
    return w.robot_dropdown.currentText()


def _joint_count(w, robot):
    robots = getattr(w, 'robots', None)
    if isinstance(robots, dict) and robot in robots:
        return int(robots[robot])
    return int((getattr(w, 'robot_configs', {}).get(robot, {}) or {}).get('num_joints', 7))


def _target(args):
    """Validate which parameter set the request means.

    H-PMP (mode 100) is a joint set with layer 'joint' and a Cartesian axis otherwise."""
    from fic_studio import xml_manager as xm
    try:
        mode = int(args.get('mode', 1))
    except (TypeError, ValueError):
        raise Refused('mode must be 1 to 8, or %d for H-PMP' % xm.HPMP_MODE, 'bad_args')
    if mode not in xm.CONTROL_MODES and not xm.is_hpmp_mode(mode):
        raise Refused('mode must be 1 to 8, or %d for H-PMP' % xm.HPMP_MODE, 'bad_args')
    profile = str(args.get('profile') or 'default')
    if profile != profile.strip() or not profile or len(profile) > 48:
        raise Refused('Bad config name', 'bad_args')
    axis = args.get('axis')
    if axis in SCOPED_AXES:
        return {'mode': mode, 'profile': profile, 'kind': axis if axis == 'postural' else 'elbow',
                'axis': axis}
    if xm.is_joint_mode(mode) or (xm.is_hpmp_mode(mode) and args.get('layer') == 'joint'):
        try:
            joint = int(args.get('joint', 0))
        except (TypeError, ValueError):
            raise Refused('joint must be a whole number', 'bad_args')
        return {'mode': mode, 'profile': profile, 'kind': 'joint', 'joint': joint}
    axis = str(axis or 'x')
    if axis not in CART_AXES:
        raise Refused('axis must be one of x, y, z, q, elbow, postural', 'bad_args')
    return {'mode': mode, 'profile': profile, 'kind': 'axis', 'axis': axis}


def _to_dict(params, kind):
    d = dataclasses.asdict(params)
    sr = d.pop('saturation_ratios', None) or [0.9]
    d['saturation_ratio'] = sr[0]
    return d


def _load(mgr, w, t):
    robot = _robot(w)
    if t['kind'] == 'joint':
        n = _joint_count(w, robot)
        if not 0 <= t['joint'] < n:
            raise Refused('Joint %d does not exist (0 to %d)' % (t['joint'], n - 1), 'bad_args')
        return mgr.load_joint_params(robot, t['joint'], t['mode'], t['profile'])
    if t['kind'] == 'axis':
        return mgr.load_cartesian_axis(robot, t['mode'], t['axis'], t['profile'])
    if t['kind'] == 'postural':
        return mgr.load_postural_params(robot)
    return mgr.load_elbow_params(robot)


# ── Plot ─────────────────────────────────────────────────────────────────

def curve(params, points=96, full_points=241, view=None):
    """The force-versus-error profile for these values (the FIC law, from the Studio).

    ``x`` / ``y`` is the positive half. ``sym`` is the whole curve over the
    classic plot's range (signed error, magnitude of the effort: a V/W), with the linear-stiffness reference and
    the convergence phase beside it, the saturation lines and the stats the
    classic page shows under its plot. ``view`` (error range, effort range)
    replaces that range: the page holds its axes still while a handle is dragged.
    """
    from fic_studio.mixins.tuning_mixin import convergence_force, fic_force_multi
    em = [float(v) for v in params['error_max']]
    fm = [float(v) for v in params['force_max']]
    if not em or len(em) != len(fm):
        return None
    k0, sr = float(params['stiffness']), float(params['saturation_ratio'])
    srs = [sr]
    xmax = em[-1] * 1.5
    xs = [xmax * i / (points - 1) for i in range(points)]
    ys = [fic_force_multi(x, k0, em, fm, srs) for x in xs]
    # the classic canvas: +-2.8 x the last breakpoint, +-1.3 x the last force
    xr, yr = view or (em[-1] * 2.8, (fm[-1] or 1.0) * 1.3)
    fx = [-xr + 2 * xr * i / (full_points - 1) for i in range(full_points)]
    e_max = em[-1] * 0.80
    ef_max = fic_force_multi(e_max, k0, em, fm, srs)
    conv = [(x, convergence_force(abs(x), e_max, ef_max)) for x in fx if abs(x) <= e_max]
    k_conv = abs(ef_max) / max(abs(e_max / 2.0), 1e-5)
    return {
        'x': xs, 'y': ys, 'breakpoints': [{'x': a, 'y': b} for a, b in zip(em, fm)],
        'sym': {'x': fx, 'y': [abs(fic_force_multi(x, k0, em, fm, srs)) for x in fx],   # |effort| against signed error, as the classic canvas
                'linear': [min(k0 * abs(x), fm[-1]) for x in fx],
                'convergence': {'x': [c[0] for c in conv], 'y': [c[1] for c in conv]},
                'xr': xr, 'yr': yr,
                'lines': [{'x': a, 'sat': True} for a in em] + [{'x': sr * a, 'sat': False} for a in em]},
        'stats': {'k0': k0, 'n_s': 2 * len(em), 'xb': em, 'f': fm, 'sr': [sr] * len(em), 'k_conv': k_conv},
    }


# ── Validation ───────────────────────────────────────────────────────────

def _listed(value, key, spec):
    if isinstance(value, str):
        value = [v for v in value.replace(';', ',').split(',') if v.strip()]
    if not isinstance(value, list) or not value:
        raise Refused('%s needs a list of numbers' % spec['label'], 'bad_args')
    out = []
    for v in value:
        n = _num(v)
        if n is None or isinstance(v, bool):
            raise Refused('%s: "%s" is not a number' % (spec['label'], v), 'bad_args')
        out.append(n)
    return out


def clean(values, specs):
    """Check a dict of edited values against the field specs; return typed values."""
    out = {}
    by_key = {s['key']: s for s in specs}
    for key, raw in values.items():
        spec = by_key.get(key)
        if spec is None:
            raise Refused('Unknown parameter: %s' % key, 'bad_args')
        kind = spec['type']
        if kind == 'bool':
            if not isinstance(raw, bool):
                raise Refused('%s must be true or false' % spec['label'], 'bad_args')
            out[key] = raw
        elif kind == 'list':
            out[key] = _listed(raw, key, spec)
        else:
            n = _num(raw)
            if n is None or isinstance(raw, bool):
                raise Refused('%s must be a number' % spec['label'], 'bad_args')
            if kind in ('int', 'choice'):
                if n != int(n):
                    raise Refused('%s must be a whole number' % spec['label'], 'bad_args')
                n = int(n)
            out[key] = n
        if kind in ('number', 'int', 'choice'):
            if not spec['min'] <= out[key] <= spec['max']:
                raise Refused('%s must be between %g and %g' % (spec['label'], spec['min'], spec['max']),
                              'bad_args')
        elif kind == 'list':
            lo, hi = spec['min'], spec['max']
            if any(not lo <= v <= hi for v in out[key]):
                raise Refused('%s must be between %g and %g' % (spec['label'], lo, hi), 'bad_args')
    return out


def check_profile_shape(p):
    """The same rules the desktop page applies before it writes."""
    em, fm = p['error_max'], p['force_max']
    if len(em) != len(fm):
        raise Refused('error_max and force_max need the same number of values', 'bad_args')
    for name, seq in (('error_max', em), ('force_max', fm)):
        for i in range(1, len(seq)):
            if seq[i] <= seq[i - 1]:
                raise Refused('%s must be strictly ascending (index %d: %g <= %g)'
                              % (name, i, seq[i], seq[i - 1]), 'bad_args')
    if em and em[0] > 1e-12 and p['stiffness'] > fm[0] / em[0] + 1e-6:
        raise Refused('Stiffness %.1f exceeds force_max[0] / error_max[0] = %.1f'
                      % (p['stiffness'], fm[0] / em[0]), 'bad_args')
    if 'planner_pos_lower' in p and p['planner_pos_lower'] >= p['planner_pos_upper']:
        raise Refused('Planner position lower must be below upper', 'bad_args')


def _apply(base, edits, specs, kind):
    """The dataclass with ``edits`` applied (and N_S kept in step with the list)."""
    merged = _to_dict(base, kind)
    merged.update(edits)
    check_profile_shape(merged)
    if not merged['asymmetric']:
        merged['n_saturations'] = 2 * len(merged['error_max'])
        merged['n_neg_sat'] = len(merged['error_max'])
    fields = dict(merged)
    fields['saturation_ratios'] = [fields.pop('saturation_ratio')]
    names = {f.name for f in dataclasses.fields(base)}
    return type(base)(**{k: v for k, v in fields.items() if k in names}), merged


# ── Commands ─────────────────────────────────────────────────────────────

def info(w, args, client):
    """Robot, joint count, configs, modes: what the page's selectors offer."""
    from fic_studio import xml_manager as xm
    robot = _robot(w)
    mgr = _xml(w)
    profiles = mgr.get_all_profiles(robot) or ['default']
    modes = [{'id': i, 'name': m['name'], 'type': m['type'], 'planner': m['has_planner'],
              'admittance': m['has_admittance']} for i, m in sorted(xm.CONTROL_MODES.items())]
    modes.append({'id': xm.HPMP_MODE, 'name': xm.HPMP_INFO['name'], 'type': 'hpmp', 'planner': True,
                  'admittance': True})       # not a control mode: H-PMP's own tuning, used in modes 5-8
    return {'ok': True, 'robot': robot, 'joints': _joint_count(w, robot), 'profiles': profiles,
            'active': _active(w), 'modes': modes, 'axes': list(CART_AXES) + list(SCOPED_AXES),
            'config_path': str(mgr.config_path), 'trial': _trial(w, mgr, robot)}


def _active(w):
    """The config the controller runs: the Studio's own selector."""
    combo = getattr(w, '_rc_profile_dropdown', None)
    return (combo.currentData() or combo.currentText()) if combo is not None else None


def get(w, args, client):
    """The values and field list for one parameter set, with the force profile."""
    from fic_studio import xml_manager as xm
    t = _target(args)
    mgr = _xml(w)
    params = _load(mgr, w, t)
    values = _to_dict(params, t['kind'])
    specs = _specs(t['kind'], t['mode'])
    fallback = hpmp_layers = False
    if t['kind'] in ('joint', 'axis'):
        have = mgr.get_available_profiles(_robot(w), t['mode'])
        fallback = t['profile'] not in have
        # H-PMP with no block of its own on this robot runs on mode 4's joints and mode 6's axes
        hpmp_layers = xm.is_hpmp_mode(t['mode']) and not {t['profile'], 'default'} & set(have)
    return {'ok': True, 'target': t, 'values': values, 'fields': specs,
            'fallback': fallback and not hpmp_layers, 'hpmp_layers': hpmp_layers, 'plot': curve(values)}


def plot(w, args, client):
    """The force profile for edited, unsaved values (read-only)."""
    t = _target(args)
    specs = _specs(t['kind'], t['mode'])
    mgr = _xml(w)
    base = _to_dict(_load(mgr, w, t), t['kind'])
    base.update(clean(args.get('values') or {}, specs))
    return {'ok': True, 'plot': curve(base, view=_view(args.get('view')))}


def _view(value):
    """The axes the page is holding while a handle is dragged: [error range, effort range], or None."""
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    xr, yr = _num(value[0]), _num(value[1])
    if xr is None or yr is None or not (0 < xr <= 1e3 and 0 < yr <= 1e6):
        return None
    return xr, yr


def save(w, args, client):
    """Write edited values to config.xml and tell the controller to reload."""
    need_confirm(args, 'Saving to config.xml needs confirmation')
    t = _target(args)
    what, merged = _write(w, t, args.get('values'))
    w.log_message('INFO', 'Tuning saved from the web UI (%s): %s on %s' % (_who(client), what, _robot(w)),
                  'Tuning')
    return {'ok': True, 'saved': what, 'values': merged, 'plot': curve(merged)}


def _write(w, t, values):
    """Check ``values`` against the stored set, write it to config.xml and tell the controller."""
    from fic_studio import xml_manager as xm
    if not isinstance(values, dict) or not values:
        raise Refused('No values to save', 'bad_args')
    specs = _specs(t['kind'], t['mode'])
    edits = clean(values, specs)
    mgr = _xml(w)
    robot = _robot(w)
    base = _load(mgr, w, t)
    params, merged = _apply(base, edits, specs, t['kind'])
    if t['kind'] == 'joint':
        mgr.save_joint_params(robot, t['joint'], t['mode'], params, t['profile'])
        note = ('joint', t['joint'])
        what = 'joint %d, %s / %s' % (t['joint'], xm.mode_name(t['mode']), t['profile'])
    elif t['kind'] == 'axis':
        mgr.save_cartesian_axis(robot, t['mode'], t['axis'], params, t['profile'])
        note = ('cartesian', t['axis'])
        what = '%s axis, %s / %s' % (t['axis'], xm.mode_name(t['mode']), t['profile'])
    elif t['kind'] == 'postural':
        mgr.save_postural_params(robot, params)
        note = ('postural', '')
        what = 'postural (all Cartesian modes)'
    else:
        mgr.save_elbow_params(robot, params)
        note = ('elbow', '')
        what = 'elbow (all Cartesian modes)'
    mgr.save()
    with scoped_to_selected(w):
        w._publish_param_update(*note)
    return what, merged


def profile(w, args, client):
    """Add a config (cloned from another) or delete one."""
    from fic_studio import xml_manager as xm
    op = args.get('op')
    if op not in ('add', 'delete'):
        raise Refused('No such config command', 'bad_args')
    need_confirm(args, 'Changing configs needs confirmation')
    name = str(args.get('name') or '')
    if not name or name != name.strip() or len(name) > 48 or any(c in name for c in '<>"&\'/\\'):
        raise Refused('Give the config a plain name (letters, digits, spaces, - _ .)', 'bad_args')
    if name.endswith(TRIAL):
        raise Refused('Names ending in %s belong to live trials: keep or discard the trial instead' % TRIAL,
                      'refused')
    mgr = _xml(w)
    robot = _robot(w)
    if op == 'add':
        mode = int(_num(args.get('mode'), 1))
        if mode not in xm.CONTROL_MODES and not xm.is_hpmp_mode(mode):
            raise Refused('mode must be 1 to 8, or %d for H-PMP' % xm.HPMP_MODE, 'bad_args')
        src = str(args.get('from') or 'default')
        try:
            mgr.clone_profile(robot, mode, src, name)
        except ValueError as exc:
            raise Refused(str(exc), 'refused')
        mgr.save()
        w.log_message('INFO', 'Config "%s" added for %s (from "%s", mode %d) — web UI (%s)'
                      % (name, robot, src, mode, _who(client)), 'Tuning')
    else:
        if name == 'default':
            raise Refused("The 'default' config cannot be deleted", 'refused')
        removed = mgr.delete_profile_everywhere(robot, name)
        if not removed:
            raise Refused('No such config', 'bad_args')
        mgr.save()
        w.log_message('WARN', 'Config "%s" deleted for %s (%d blocks) — web UI (%s)'
                      % (name, robot, removed, _who(client)), 'Tuning')
    refresh = getattr(w, '_refresh_rc_profile_dropdown', None)
    if refresh is not None:
        refresh()
    return {'ok': True, 'profiles': mgr.get_all_profiles(robot) or ['default']}


# ── Live trial ───────────────────────────────────────────────────────────

#: A live trial runs on a scratch copy of a config, '<config>~trial'. The robot
#: runs the copy, and each edit is written to it and reloaded as it is made, so
#: the saved config is not touched: KEEP copies the trial back over it and makes
#: it the active config, DISCARD puts the robot back on the config it ran before.
#: Either way the copy is deleted. The copy is an ordinary config in config.xml,
#: so a trial left behind by a crash is found again and can still be kept or
#: discarded. Elbow and postural are robot-wide, not per config, so they are not
#: part of a trial.
TRIAL = '~trial'


def _trial(w, mgr, robot):
    """The trial on this robot (name, the config it copies, the config DISCARD returns to), or None."""
    names = [p for p in mgr.get_all_profiles(robot) if p.endswith(TRIAL) and len(p) > len(TRIAL)]
    if not names:
        return None
    name = names[0]
    source = name[:-len(TRIAL)]
    mem = getattr(w, '_web_tuning_trial', None) or {}
    back = mem.get('back') if (mem.get('robot'), mem.get('name')) == (robot, name) else None
    if back not in mgr.get_all_profiles(robot):
        back = source if source in mgr.get_all_profiles(robot) else 'default'
    return {'name': name, 'source': source, 'back': back}


def _activate(w, name):
    """Make ``name`` the config the controller runs; the Studio's own selector publishes it."""
    refresh = getattr(w, '_refresh_rc_profile_dropdown', None)
    if refresh is not None:
        refresh()
    combo = getattr(w, '_rc_profile_dropdown', None)
    index = combo.findData(name) if combo is not None else -1
    if index < 0:
        raise Refused('The Studio has no config called %s' % name, 'unavailable')
    if index != combo.currentIndex():
        with scoped_to_selected(w):
            combo.setCurrentIndex(index)


def _profiles_reply(w, mgr, robot, **extra):
    refresh = getattr(w, '_refresh_rc_profile_dropdown', None)
    if refresh is not None:
        refresh()
    return dict({'ok': True, 'profiles': mgr.get_all_profiles(robot) or ['default'], 'active': _active(w),
                 'trial': _trial(w, mgr, robot)}, **extra)


def trial(w, args, client):
    """Start a live trial of a config, apply edits to it, or keep or discard it (see TRIAL)."""
    op = args.get('op')
    if op not in ('start', 'apply', 'keep', 'discard'):
        raise Refused('No such trial command', 'bad_args')
    mgr = _xml(w)
    robot = _robot(w)
    running = _trial(w, mgr, robot)
    if op == 'start':
        need_confirm(args, 'Starting a live trial needs confirmation')
        if running:
            raise Refused('A live trial of "%s" is already running' % running['source'], 'refused')
        source = str(args.get('profile') or 'default')
        if source not in (mgr.get_all_profiles(robot) or ['default']):
            raise Refused('No such config', 'bad_args')
        name = source + TRIAL
        if len(name) > 48:
            raise Refused('That config name is too long to trial', 'bad_args')
        back = _active(w) or source
        mgr.copy_profile_blocks(robot, source, name)
        mgr.save()
        w._web_tuning_trial = {'robot': robot, 'name': name, 'back': back}
        _activate(w, name)
        w.log_message('INFO', 'Live tuning trial of "%s" started on %s — web UI (%s)' % (source, robot, _who(client)),
                      'Tuning')
        return _profiles_reply(w, mgr, robot)
    if running is None:
        raise Refused('No live trial is running', 'refused')
    if op == 'apply':
        t = _target(dict(args, profile=running['name']))
        if t['kind'] not in ('joint', 'axis'):
            raise Refused('Elbow and postural are robot-wide, not part of a trial: save them directly', 'bad_args')
        what, merged = _write(w, t, args.get('values'))
        return {'ok': True, 'applied': what, 'values': merged, 'plot': curve(merged)}
    need_confirm(args, 'Ending a live trial needs confirmation')
    if op == 'keep':
        mgr.copy_profile_blocks(robot, running['name'], running['source'])
        mgr.save()
        _activate(w, running['source'])       # the robot carries on with the same values, now the saved ones
        what = 'kept: written to "%s"' % running['source']
    else:
        _activate(w, running['back'])         # the robot goes back first, so it never runs a missing config
        what = 'discarded: back on "%s"' % running['back']
    mgr.delete_profile_everywhere(robot, running['name'])
    mgr.save()
    w._web_tuning_trial = None
    w.log_message('INFO' if op == 'keep' else 'WARN', 'Live tuning trial of "%s" %s on %s — web UI (%s)'
                  % (running['source'], what, robot, _who(client)), 'Tuning')
    return _profiles_reply(w, mgr, robot, ended=what)


# ── Raw XML (the classic VIEW XML) ───────────────────────────────────────

def xml_section(w, args, client):
    """Read, or replace with an explicit confirm, this robot's block of config.xml."""
    import xml.etree.ElementTree as ET
    from fic_studio.xml_manager import write_tree_atomic
    mgr = _xml(w)
    path = str(mgr.config_path)
    robot = _robot(w)
    tree = ET.parse(path)
    root = tree.getroot()
    found = next((e for e in root.iter('robot') if e.get('name') == robot), None)
    if args.get('op') != 'save':
        if found is None:
            raise Refused("Robot '%s' is not in config.xml" % robot, 'unavailable')
        ET.indent(found, space='  ')
        return {'ok': True, 'robot': robot, 'path': path, 'text': ET.tostring(found, encoding='unicode')}
    need_confirm(args, 'Replacing a robot block in config.xml needs confirmation')
    text = args.get('text')
    if not isinstance(text, str) or len(text) > 2_000_000:
        raise Refused('Send the robot block as text', 'bad_args')
    try:
        edited = ET.fromstring(text)
    except ET.ParseError as exc:
        raise Refused('That is not valid XML: %s' % exc, 'bad_args')
    if edited.tag != 'robot' or edited.get('name') != robot:
        raise Refused('The block must be <robot name="%s">' % robot, 'bad_args')
    for parent in root.iter():
        for i, child in enumerate(list(parent)):
            if child is found:
                parent.remove(child)
                parent.insert(i, edited)
                ET.indent(root, space='  ')
                write_tree_atomic(tree, path, encoding='unicode')
                w.log_message('WARN', 'config.xml block for %s replaced from the web UI (%s)' % (robot, _who(client)),
                              'Tuning')
                return {'ok': True}
    raise Refused("Robot '%s' is not in config.xml" % robot, 'unavailable')
