"""The Settings page, for the web UI.

The classic page is a column of cards built from ``DEFAULT_SETTINGS``; this
module describes the same cards as data (one field spec per setting, with its
range and a hint), reads their values from ``w.settings`` and writes edits
back through the Studio's own ``_save_settings`` and ``_apply_settings``. The
browser draws a form from the description and knows no setting by name.

Commands: ``settings_get``, ``settings_set``, ``settings_reset``,
``settings_extensions``, ``settings_gamepad``, ``settings_web``.
"""
import copy
import os

from fic_studio import features
from fic_studio.web_remote.common import Refused, _num, _who, need_confirm

#: Keys that only take effect after a restart (the classic page marks them).
RESTART_KEYS = ('network.ros_domain_id', 'network.partner_ip', 'network.dds_mode', 'paths.experiments')

#: Keys that apply the moment they are saved: the web UI restyles itself in
#: place (see ``appearance_state``) and the classic window re-themes in
#: ``_apply_settings``. Nothing here ever needs a restart.
LIVE_KEYS = ('appearance.palette', 'appearance.brightness', 'appearance.font_family', 'appearance.font_size')


def appearance_state(w):
    """What the browser needs to restyle itself: brightness, palette, font and an
    accent for each brightness. ``rev`` changes whenever any of it does."""
    from fic_studio.theme.tokens import THEMES, normalize_theme_name, split_theme_name
    a = (getattr(w, 'settings', None) or {}).get('appearance') or {}
    palette, brightness = split_theme_name(normalize_theme_name(a.get('theme')))
    accent = {b: THEMES['%s_%s' % (palette, b)]['accent_blue'] for b in ('light', 'dark')}
    out = {'palette': palette, 'brightness': brightness, 'font_family': a.get('font_family') or '',
           'font_size': int(a.get('font_size') or 12), 'accent': accent}
    out['rev'] = '|'.join(str(out[k]) for k in ('palette', 'brightness', 'font_family', 'font_size')) + '|' + accent['light'] + accent['dark']
    return out


def _emit_appearance(w, keys):
    """Tell every open browser (desktop window, phones) right now, not on the next poll."""
    if not any(k in LIVE_KEYS for k in keys):
        return
    data = {'changed': sorted(k for k in keys if k in LIVE_KEYS), 'appearance': appearance_state(w), 'restart': False}
    for name in ('_web_server', '_web_ui_server'):
        server = getattr(w, name, None)
        if server is not None and hasattr(server, 'emit'):
            server.emit('settings', data)
    for server in getattr(w, '_web_event_sinks', None) or ():
        server.emit('settings', data)


def _field(key, label, kind, **extra):
    spec = {'key': key, 'label': label, 'type': kind}
    spec.update({k: v for k, v in extra.items() if v is not None})
    return spec


def _robots(w):
    names = list(getattr(w, 'robot_configs', {}) or {}) or ['Franka_FR3']
    return [{'value': n, 'label': (w.robot_configs.get(n) or {}).get('display_name') or n}
            if hasattr(w, 'robot_configs') else {'value': n, 'label': n} for n in names]


def _backends(w):
    from fic_studio.mixins.settings_mixin import _newton_available
    from fic_studio.platform_utils import IS_MACOS
    items = [{'value': 'mujoco_native', 'label': 'MuJoCo (CPU)'}]
    if not IS_MACOS:
        items.append({'value': 'newton', 'label': 'Newton (NVIDIA Warp, GPU)'
                      + ('' if _newton_available() else ' - not installed'),
                      'disabled': not _newton_available()})
    return items


def _default_experiments():
    from fic_studio import paths
    from fic_studio.config import FIC_INTERFACE_DIR
    old = os.environ.pop(paths.EXPERIMENTS_ENV, None)
    try:
        return paths.experiments_dir(FIC_INTERFACE_DIR / 'experiments', settings_path=os.devnull)
    finally:
        if old is not None:
            os.environ[paths.EXPERIMENTS_ENV] = old


def sections(w):
    """The cards, in the classic page's order, each a list of field specs."""
    from fic_studio.platform_utils import mono_font_choices
    from fic_studio.theme.tokens import PALETTE_LABELS, PALETTES
    out = [
        ('Appearance', 'Applies the moment you save, in the web UI and the classic window. No restart.', False, [
            _field('appearance.palette', 'Palette', 'choice',
                   choices=[{'value': p, 'label': PALETTE_LABELS.get(p, p.title())} for p in PALETTES]),
            _field('appearance.brightness', 'Brightness', 'choice',
                   choices=[{'value': 'dark', 'label': 'Dark'}, {'value': 'light', 'label': 'Light'}]),
            _field('appearance.font_family', 'Font family', 'choice',
                   choices=[{'value': f, 'label': f} for f in mono_font_choices()]),
            _field('appearance.font_size', 'Font size', 'int', min=8, max=18, unit='px'),
        ]),
        ('Startup', '', False, [
            _field('general.default_robot', 'Default robot', 'choice', choices=_robots(w)),
            _field('general.auto_save', 'Auto-save settings', 'bool'),
            _field('general.auto_save_interval', 'Auto-save interval', 'int', min=10, max=600, unit='s'),
            _field('general.show_log_viewer', 'Show log viewer', 'bool'),
            _field('general.log_viewer_height', 'Log viewer height', 'int', min=50, max=400, unit='px'),
        ]),
        ('Network', 'Needs a restart: these are read once, when the Studio starts.', True, [
            _field('network.ros_domain_id', 'ROS domain ID', 'int', min=0, max=232,
                   hint='ROS_DOMAIN_ID (0 to 232)'),
            _field('network.partner_ip', 'Partner IP', 'text', placeholder='e.g. 192.168.1.100'),
            _field('network.dds_mode', 'DDS discovery', 'choice',
                   choices=[{'value': 'multicast', 'label': 'Multicast (auto)'},
                            {'value': 'unicast', 'label': 'Unicast (manual IP)'}]),
        ]),
        ('Data', 'Where experiments and bags are recorded. Needs a restart.', True, [
            _field('paths.experiments', 'Experiments folder', 'text',
                   placeholder='blank: ' + _default_experiments(),
                   hint='Absolute path. Robot configs and tuning stay with the lab robot library'),
        ]),
        ('Simulation', '', False, [
            _field('simulation.backend', 'Backend', 'choice', choices=_backends(w)),
        ]),
    ]
    if features.headset():
        out.append(
        ('VR', 'The MuJoCo VR headset app. Applies the moment you save.', False, [
            _field('vr.relay_always_on', 'Relay always on', 'bool',
                   hint='Headsets find this Studio whenever it is open, not only after VR > Start'),
            _field('vr.allow_viewers', 'Let headsets watch the lab', 'bool',
                   hint="Watch the lab: every Studio's robots, live, in the headset"),
            _field('vr.fleet_stations', 'Headset robots on Multi-Robot', 'bool',
                   hint="A headset's robots are listed as VR stations, to lead or follow"),
            _field('vr.headset_ip', 'Headset address', 'text', placeholder='blank: find it on the WiFi',
                   hint='Fixed IP, for networks that do not pass broadcasts (e.g. a 6 GHz band)'),
            _field('vr.require_key', 'Require a pairing key', 'bool',
                   hint='Sign every packet: only a headset paired over USB can drive the robots'),
        ]))
    out += [
        ('Playback tolerances', '', False, [
            _field('tolerances.joint_tolerance_deg', 'Joint tolerance', 'float', min=0.5, max=20.0, step=0.5, unit='deg',
                   hint='Joint position tolerance for sequence playback'),
            _field('tolerances.position_tolerance_mm', 'Position tolerance', 'float', min=1.0, max=50.0, step=0.5, unit='mm',
                   hint='Cartesian position tolerance for sequence playback'),
            _field('tolerances.timeout_seconds', 'Timeout', 'float', min=5.0, max=120.0, step=1.0, unit='s',
                   hint='Longest wait for each waypoint'),
        ]),
        ('Shortcuts', 'Key combinations for the classic window (for example Ctrl+R).', False, [
            _field('shortcuts.' + k, k.replace('_', ' ').capitalize(), 'text')
            for k in sorted((w.settings.get('shortcuts') or {}))
        ]),
        ('Developer', '', False, [
            _field('advanced.verbose_logging', 'Verbose ROS 2 logging', 'bool'),
            _field('advanced.save_rosbag', 'Auto-save rosbag on exit', 'bool'),
        ]),
    ]
    return out


def _get_path(settings, key):
    node = settings
    for part in key.split('.'):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node


def _set_path(settings, key, value):
    node = settings
    parts = key.split('.')
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def _values(w, specs):
    from fic_studio.theme.tokens import normalize_theme_name, split_theme_name
    out = {}
    palette, brightness = split_theme_name(normalize_theme_name(w.settings['appearance']['theme']))
    for spec in specs:
        key = spec['key']
        if key == 'appearance.palette':
            out[key] = palette
        elif key == 'appearance.brightness':
            out[key] = brightness
        else:
            out[key] = _get_path(w.settings, key)
    return out


def _extensions(w):
    enabled = set((w.settings.get('extensions') or {}).get('enabled', []))
    return [{'id': m.id, 'name': ((getattr(m, 'sidebar_icon', '') + '  ' + m.name).strip()), 'description': m.description or '',
             'enabled': m.id in enabled} for m in getattr(w, '_extensions_manifests', []) or []]


def _gamepad(w):
    if not hasattr(w, '_gp_settings'):
        return {'available': False}
    from fic_studio.mixins import gamepad_mixin as gp
    active = list(getattr(w.node, 'active_robots', [1])) or [1]
    return {
        'available': True,
        'deadzone': float(getattr(w, '_gp_deadzone', 0.12)),
        'invert': {ax: bool((getattr(w, '_gp_invert', {}) or {}).get(ax, False)) for ax in ('LX', 'LY', 'RX', 'RY')},
        'actions': list(gp._ACTION_IDS),
        'buttons': [{'name': n, 'action': (getattr(w, '_gp_button_map', {}) or {}).get(n, 'none')}
                    for n in gp._DEFAULT_BUTTON_MAP],
        'pads': [{'path': p.path, 'name': p.name, 'robot': p.target_robot_n, 'space': p.space,
                  'speed': p.speed_level} for p in (getattr(w, '_gp_pads', {}) or {}).values()],
        'robots': [{'n': n, 'label': 'Robot %d' % n} for n in active],
        'fixed': ['LT/RT: variable gripper (close/open)', 'D-pad up/down: speed level',
                  'D-pad left/right: cycle robot'],
    }


def _web(w):
    server = getattr(w, '_web_server', None)
    cfg = w._web_settings() if hasattr(w, '_web_settings') else {'enabled': False, 'port': 8088}
    holder = None
    clients = 0
    if server is not None:
        clients = len(server.clients())
        holder = w._web_lease.holder()
    return {
        'enabled': bool(cfg.get('enabled')), 'port': int(cfg.get('port', 8088)),
        'serving': server is not None, 'error': getattr(w, '_web_error', '') or '',
        'urls': list(getattr(w, '_web_urls', []) or []), 'clients': clients,
        'holder': ({'name': holder.get('name') or 'browser', 'addr': holder.get('addr')} if holder else None),
    }


def get(w, args, client):
    specs = []
    cards = []
    for title, hint, restart, fields in sections(w):
        specs.extend(fields)
        cards.append({'title': title, 'hint': hint, 'restart': restart, 'fields': fields})
    return {'ok': True, 'cards': cards, 'values': _values(w, specs), 'extensions': _extensions(w),
            'gamepad': _gamepad(w), 'web': _web(w), 'file': getattr(w, 'SETTINGS_FILE', '')}


def _validate(spec, raw):
    kind = spec['type']
    if kind == 'bool':
        if not isinstance(raw, bool):
            raise Refused('%s must be on or off' % spec['label'], 'bad_args')
        return raw
    if kind in ('int', 'float'):
        n = _num(raw)
        if n is None or isinstance(raw, bool):
            raise Refused('%s must be a number' % spec['label'], 'bad_args')
        if kind == 'int':
            if n != int(n):
                raise Refused('%s must be a whole number' % spec['label'], 'bad_args')
            n = int(n)
        if not spec['min'] <= n <= spec['max']:
            raise Refused('%s must be between %g and %g' % (spec['label'], spec['min'], spec['max']), 'bad_args')
        return n
    if kind == 'choice':
        ok = [c['value'] for c in spec['choices'] if not c.get('disabled')]
        if raw not in ok:
            raise Refused('%s: pick one of the listed choices' % spec['label'], 'bad_args')
        return raw
    if not isinstance(raw, str) or len(raw) > 200:
        raise Refused('%s must be text' % spec['label'], 'bad_args')
    if spec['key'] == 'paths.experiments' and raw.strip():
        p = os.path.expanduser(raw.strip())
        if not os.path.isabs(p):
            raise Refused('Experiments folder must be an absolute path, like /data/fic or ~/fic_data', 'bad_args')
        from fic_studio import paths
        if not paths._writable_dir(p):
            raise Refused('The Studio cannot write to %s' % p, 'bad_args')
    if spec['key'] == 'vr.headset_ip' and raw.strip():
        import ipaddress
        try:
            ipaddress.IPv4Address(raw.strip())
        except ValueError:
            raise Refused('Headset address must be an IPv4 address, like 192.168.0.234', 'bad_args')
    return raw.strip()


def set_values(w, args, client):
    """Save edited settings, as the classic page's SAVE SETTINGS does."""
    from fic_studio.theme.tokens import compose_theme_name, normalize_theme_name, split_theme_name
    values = args.get('values')
    if not isinstance(values, dict) or not values:
        raise Refused('No values to save', 'bad_args')
    by_key = {f['key']: f for _t, _h, _r, fields in sections(w) for f in fields}
    clean = {}
    for key, raw in values.items():
        spec = by_key.get(key)
        if spec is None:
            raise Refused('Unknown setting: %s' % key, 'bad_args')
        clean[key] = _validate(spec, raw)
    if any(k in RESTART_KEYS for k in clean):
        need_confirm(args, 'Network settings need a restart and need confirmation')
    palette, brightness = split_theme_name(normalize_theme_name(w.settings['appearance']['theme']))
    for key, value in clean.items():
        if key == 'appearance.palette':
            palette = value
        elif key == 'appearance.brightness':
            brightness = value
        elif key == 'network.dds_mode':
            w.settings['network']['dds_mode'] = value
        else:
            _set_path(w.settings, key, value)
    if 'appearance.palette' in clean or 'appearance.brightness' in clean:
        w.settings['appearance']['theme'] = compose_theme_name(palette, brightness)
    w._save_settings()
    w._apply_settings()
    w.log_message('INFO', 'Settings saved from the web UI (%s): %s' % (_who(client), ', '.join(sorted(clean))), 'Settings')
    _emit_appearance(w, clean)
    return {'ok': True, 'saved': sorted(clean), 'restart': any(k in RESTART_KEYS for k in clean)}


def reset(w, args, client):
    need_confirm(args, 'Resetting every setting needs confirmation')
    from fic_studio.mixins.settings_mixin import DEFAULT_SETTINGS
    w.settings = copy.deepcopy(DEFAULT_SETTINGS)
    w._save_settings()
    w._apply_settings()
    w.log_message('WARN', 'Settings reset to defaults from the web UI (%s)' % _who(client), 'Settings')
    _emit_appearance(w, LIVE_KEYS)
    return {'ok': True}


def extensions(w, args, client):
    """Turn extensions on or off (takes effect after a restart, as in the classic page)."""
    ids = args.get('enabled')
    known = {m.id for m in getattr(w, '_extensions_manifests', []) or []}
    if not isinstance(ids, list) or any(not isinstance(i, str) or i not in known for i in ids):
        raise Refused('Choose from the extensions that exist', 'bad_args')
    need_confirm(args, 'Changing extensions needs confirmation')
    w._save_extensions_enabled(sorted(set(ids)))
    w.log_message('INFO', 'Extensions set from the web UI (%s): %s' % (_who(client), ', '.join(sorted(ids)) or 'none'),
                  'Settings')
    return {'ok': True, 'restart': True}


def gamepad(w, args, client):
    if not hasattr(w, '_gp_settings'):
        raise Refused('The gamepad is not available on this Studio', 'unavailable')
    from fic_studio.mixins import gamepad_mixin as gp
    op = args.get('op')
    if op == 'state':
        return {'ok': True, 'gamepad': _gamepad(w)}
    if op == 'deadzone':
        v = _num(args.get('value'))
        if v is None or not 0.0 <= v <= 0.5:
            raise Refused('Deadzone must be between 0 and 0.5', 'bad_args')
        w._gp_on_deadzone(v)
    elif op == 'invert':
        axis = args.get('axis')
        if axis not in ('LX', 'LY', 'RX', 'RY'):
            raise Refused('No such axis', 'bad_args')
        w._gp_invert[axis] = bool(args.get('value'))
        w._gp_settings()['invert_axes'] = w._gp_invert
        w._gp_save_settings()
    elif op == 'map':
        button, action = args.get('button'), args.get('action')
        if button not in gp._DEFAULT_BUTTON_MAP or action not in gp._ACTION_IDS:
            raise Refused('No such button or action', 'bad_args')
        w._gp_on_mapping_changed(button, action)
    elif op == 'pad':
        pad = (w._gp_pads or {}).get(args.get('path'))
        n = args.get('robot')
        if pad is None or n not in list(getattr(w.node, 'active_robots', [1])):
            raise Refused('No such controller or robot', 'bad_args')
        if n != pad.target_robot_n:
            pad.target_robot_n = n
            w._gp_reset_pad_state(pad)
    else:
        raise Refused('No such gamepad command', 'bad_args')
    return {'ok': True, 'gamepad': _gamepad(w)}


def web(w, args, client):
    op = args.get('op')
    if op == 'state':
        return {'ok': True, 'web': _web(w)}
    if op == 'enable':
        on = bool(args.get('value'))
        w._web_save_setting('enabled', on)
        (w._web_start if on else w._web_stop)()
    elif op == 'port':
        port = args.get('value')
        if not isinstance(port, int) or isinstance(port, bool) or not 1024 <= port <= 65535:
            raise Refused('Port must be between 1024 and 65535', 'bad_args')
        w._web_save_setting('port', port)
        if getattr(w, '_web_server', None) is not None:
            w._web_stop()
            w._web_start()
    elif op == 'revoke':
        w._web_revoke_control()
    else:
        raise Refused('No such web remote command', 'bad_args')
    return {'ok': True, 'web': _web(w)}


def _vr(w):
    from fic_studio.fleet import vr_pairing
    s = w._vr_settings() if hasattr(w, '_vr_settings') else {}
    key = vr_pairing.read_key()
    return {'require_key': bool(s.get('require_key')), 'fingerprint': vr_pairing.fingerprint(key),
            'key_file': vr_pairing.key_path()}


def vr(w, args, client):
    """Pair the headset over USB, or make a new pairing key."""
    if not features.headset():
        raise Refused('This build of the Studio has no VR headset support', 'unavailable')
    from fic_studio.fleet import vr_pairing
    op = args.get('op')
    if op == 'state':
        return {'ok': True, 'vr': _vr(w)}
    if op == 'pair':
        need_confirm(args, 'Pairing restarts the headset app and needs confirmation')
        key = vr_pairing.relay_key(w._vr_settings())
        ok, message = vr_pairing.pair(key)
        w.log_message('INFO' if ok else 'WARN', 'VR: ' + message, 'VR')
        if not ok:
            raise Refused(message, 'failed')
        return {'ok': True, 'message': message, 'vr': _vr(w)}
    if op == 'new_key':
        need_confirm(args, 'A new key unpairs every headset and needs confirmation')
        vr_pairing.new_key()
        if hasattr(w, '_stop_vr_background'):
            w._stop_vr_background()
            w._apply_vr_settings(from_settings=True)
        w.log_message('WARN', 'VR: new pairing key %s: pair each headset again'
                      % vr_pairing.fingerprint(vr_pairing.read_key()), 'VR')
        return {'ok': True, 'vr': _vr(w)}
    raise Refused('No such VR command', 'bad_args')


HANDLERS = {
    'settings_get': get,
    'settings_set': set_values,
    'settings_reset': reset,
    'settings_extensions': extensions,
    'settings_gamepad': gamepad,
    'settings_web': web,
    'settings_vr': vr,
}
