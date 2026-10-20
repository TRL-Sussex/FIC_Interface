"""The Studio's window chrome, for the web UI: menus, toolbar, status bar, drawer.

Everything here is a thin read of, or call into, the very handlers the
classic Qt window wires its menus, toolbar buttons, status-bar LEDs and
control-bar panels to. The browser draws; this module says what is true and
runs what was asked.

Snapshot sections: ``statusbar``, ``toolbar``, ``stations``, ``dashboard``,
``live`` (the tuning page's live telemetry), ``drawer`` (control-bar panels).
Commands: ``toolbar``, ``station``, ``menu``, ``robot_pause``, ``robot_stop``,
``live_publish``, ``reset_defaults``, ``preset``, ``contact_truth``,
``export``, ``import_snapshot``, ``recent_experiments``, ``build``.
"""
import math
import os
import subprocess
import threading
import time
from collections import deque

from fic_studio.web_remote.common import Refused, _num, _who, need_confirm

#: Where a build's output waits for the page to read it.
BUILD_LINES = 400


# ── small readers ────────────────────────────────────────────────────────

def _robot(w):
    return w.robot_dropdown.currentText()


def _cfg(w, robot=None):
    return getattr(w, 'robot_configs', {}).get(robot or _robot(w), {}) or {}


def _led(w, name, label_attr=None, default_text=''):
    """One status-bar light: its colour word and the text beside it."""
    led = getattr(w, name, None)
    status = getattr(led, '_status', 'idle') if led is not None else 'idle'
    text = default_text
    label = getattr(w, label_attr, None) if label_attr else None
    if label is not None and hasattr(label, 'text'):
        text = label.text()
    return {'status': status, 'text': text}


def _checked(w, attr, default=False):
    box = getattr(w, attr, None)
    return bool(box.isChecked()) if box is not None and hasattr(box, 'isChecked') else default


def _joint_count(w, robot):
    robots = getattr(w, 'robots', None)
    if isinstance(robots, dict) and robot in robots:
        return int(robots[robot])
    return int(_cfg(w, robot).get('num_joints', 7))


# ── Status bar ───────────────────────────────────────────────────────────

def statusbar_state(w, ctx):
    """ROS2 / FIC / SIM / BAGS / INNER / TELEOP / GAMEPAD / WEB, and the robot line."""
    robot = ctx['robot']
    label = getattr(w, 'robot_status_label', None)
    line = label.text() if label is not None and hasattr(label, 'text') else \
        'Robot: %s (%d DOF)' % (_cfg(w, robot).get('display_name') or robot, _joint_count(w, robot))
    web = _led(w, '_web_led', '_web_status_label', 'WEB')
    return {
        'ros': _led(w, 'ros_led', None, 'ROS2'),
        'fic': _led(w, 'fic_led'),
        'sim': _led(w, 'sim_led'),
        'bags': _led(w, 'bags_led'),
        'inner': _led(w, 'il_led', 'il_label'),
        'teleop': _led(w, 'teleop_led', 'teleop_status_bar_label'),
        'gamepad': _led(w, 'gamepad_led', 'gamepad_state_label', 'GAMEPAD'),
        'web': web,
        'robot': line,
    }


# ── Toolbar ──────────────────────────────────────────────────────────────

def _stations(w):
    from fic_studio.fleet.stations import all_stations, station_label
    return [{'n': n, 'label': station_label(n)} for n in all_stations()]


def toolbar_state(w, ctx):
    """What the classic toolbar's EXP / BAG / PLT / VIEW / Teleop / Robot pickers show."""
    from fic_studio.fleet.stations import picker_value
    ext = w.get_extension('experiments') if hasattr(w, 'get_extension') else None
    return {
        'exp': {'visible': ext is not None, 'running': bool(ext is not None and getattr(ext, 'state_running', False))},
        'bags': bool(getattr(w, 'bags_running', False)),
        'plotter': bool(getattr(w, 'plotter_running', False)),
        'viewer': bool(getattr(w, 'contact_twin_running', False)),
        'keyboard': bool(getattr(w, '_kbd_on', False)),
        'keyboard_drive': w.keyboard_state() if hasattr(w, 'keyboard_state') else None,
        'stations': _stations(w),
        'station': picker_value(w.node.station_id),
    }


def stations_state(w, ctx):
    """Robots 1..4 this Studio can command, for the Multi-robot page's selector."""
    from fic_studio.fleet.stations import station_label
    active = list(getattr(w.node, '_active_robots', None) or getattr(w.node, 'active_robots', []) or [])
    current = getattr(w, '_rc_active_rid', ctx['rid'])
    return {
        'items': [{'n': n, 'rid': 'robot%d' % n, 'label': station_label(n)} for n in active],
        'active': current,
        'next': (max(active) + 1) if active else 2,
    }


# ── Dashboard (the classic Overview and Detailed tabs) ───────────────────

CONTACT_FORCE_N = 5.0
EE_COLUMNS = ('x', 'y', 'z', 'ox', 'oy', 'oz')


def _at(seq, index):
    if index is None or not seq or index >= len(seq):
        return None
    return seq[index]


def _deg(v):
    v = _num(v)
    return None if v is None else math.degrees(v)


def _get(w, accessor, rid):
    fn = getattr(w, accessor, None)
    try:
        return list(fn(rid)) if fn is not None else []
    except Exception:
        return []


def _rows_for(w, robot, cfg):
    fn = getattr(w, '_rc_display_joints', None)
    if fn is not None:
        try:
            rows = fn(robot, cfg)
            if rows:
                return rows
        except Exception:
            pass
    n = _joint_count(w, robot)
    return [{'label': 'J%d' % (i + 1), 'cmd_index': i, 'lo': -180.0, 'hi': 180.0} for i in range(n)]


def _dash_joints(w, ctx):
    rid, robot, cfg = ctx['rid'], ctx['robot'], ctx['cfg']
    pos, cmd = _get(w, 'joint_positions_for', rid), _get(w, 'joint_commanded_for', rid)
    vel, eff = _get(w, 'joint_velocity_for', rid), _get(w, 'joint_effort_for', rid)
    tlim = cfg.get('joint_torque_limits') or {}
    tmax_raw = tlim.get('max') if isinstance(tlim, dict) else None
    out = []
    for row in _rows_for(w, robot, cfg):
        i = row.get('cmd_index')
        p, c, v, e = _deg(_at(pos, i)), _deg(_at(cmd, i)), _deg(_at(vel, i)), _num(_at(eff, i))
        err = None if p is None or c is None else c - p
        tmx = _num(tmax_raw[i]) if isinstance(tmax_raw, list) and i is not None and i < len(tmax_raw) else _num(tmax_raw)
        lo, hi = _num(row.get('lo'), -180.0), _num(row.get('hi'), 180.0)
        out.append({
            'label': row['label'], 'commanded': c, 'position': p, 'error': err, 'velocity': v, 'torque': e,
            'tau_max': tmx, 'pct_tau': (abs(e) / abs(tmx) * 100.0) if e is not None and tmx else None,
            'lo': lo, 'hi': hi, 'pct_range': ((p - lo) / (hi - lo) * 100.0) if p is not None and hi > lo else None,
        })
    return out


def _dash_ee(w, ctx):
    """The four end-effector rows, as the classic table shows them.

    Velocity is the change since the previous snapshot, as the classic page
    computes it between its own refreshes.
    """
    from fic_studio.math_helpers import _quat_to_euler_xyz
    blank = [None] * 6
    pose = w.ee_pose_for(ctx['rid']) if hasattr(w, 'ee_pose_for') else None
    out = {'commanded': blank, 'pose': blank, 'error': blank, 'velocity': blank, 'quaternion': blank}
    if pose is None:
        return out
    p, o = pose.position, getattr(pose, 'orientation', None)
    if o is not None:
        ox, oy, oz = _quat_to_euler_xyz(o.x, o.y, o.z, o.w)
        out['quaternion'] = [o.x, o.y, o.z, o.w, None, None]
    else:
        ox = oy = oz = None
    out['pose'] = [p.x, p.y, p.z, ox, oy, oz]
    now, cur = time.monotonic(), (p.x, p.y, p.z)
    last = getattr(w, '_web_ee_last', None)
    if last is not None and now - last[1] > 1e-3:
        dt = now - last[1]
        out['velocity'] = [(cur[i] - last[0][i]) / dt for i in range(3)] + [None, None, None]
    if last is None or now - last[1] > 0.05:                  # smooth over ~50 ms, not per snapshot
        w._web_ee_last = (cur, now)
    return out


#: Joint data older than this is not evidence that a limit is respected.
SAFETY_STALE_MS = 1000.0


def _limit_at(raw, i, default):
    if isinstance(raw, list):
        return raw[i] if i < len(raw) and raw[i] is not None else default
    return default if raw is None else raw


def _dash_safety(w, ctx, joints):
    """The Dashboard's three safety lights: 'ok', 'warn' (within 5 % of a joint limit, at 90 % of a
    torque or velocity limit), 'violation' (past a joint limit) or 'unknown' (no fresh data, or no
    limit to compare with). Never 'ok' without data."""
    cfg, rid = ctx['cfg'], ctx['rid']
    age = None
    if hasattr(w, 'joint_update_time'):
        t = w.joint_update_time(rid)
        age = None if t is None else (time.monotonic() - t) * 1000.0
    pos = _get(w, 'joint_positions_for', rid)
    if not pos or (age is not None and age > SAFETY_STALE_MS) or (age is None and hasattr(w, 'joint_update_time')):
        return {'joints': 'unknown', 'velocity': 'unknown', 'torque': 'unknown'}

    jlim = cfg.get('joint_limits_deg') or {}
    joints_state = 'unknown'
    if isinstance(jlim, dict) and jlim:
        joints_state = 'ok'
        lo_raw, hi_raw = jlim.get('lower', -180), jlim.get('upper', 180)
        for i, val in enumerate(pos):
            lo, hi = math.radians(_limit_at(lo_raw, i, -180)), math.radians(_limit_at(hi_raw, i, 180))
            if val < lo or val > hi:
                joints_state = 'violation'
                break
            margin = (hi - lo) * 0.05
            if val <= lo + margin or val >= hi - margin:
                joints_state = 'warn'

    vel = _get(w, 'joint_velocity_for', rid)
    vmax = None
    try:
        from fic_studio.web_remote.safety import _merged
        v = _merged(w)['velocity']
        vmax = _num(v.get('max_joint_vel')) if v.get('enabled') else None
    except Exception:
        vmax = None
    velocity_state = 'unknown'
    if vel and vmax:
        velocity_state = 'warn' if any(abs(x) >= vmax * 0.90 for x in vel) else 'ok'

    tlim = cfg.get('joint_torque_limits') or {}
    eff = _get(w, 'joint_effort_for', rid)
    torque_state = 'unknown'
    if eff and isinstance(tlim, dict) and tlim:
        torque_state = 'ok'
        tmx_raw, tmn_raw = tlim.get('max'), tlim.get('min')
        for i, e in enumerate(eff):
            tmx, tmn = _limit_at(tmx_raw, i, None), _limit_at(tmn_raw, i, None)
            lim = max(abs(tmx) if tmx is not None else 1.0, abs(tmn) if tmn is not None else 1.0, 1.0)
            if abs(e) >= lim * 0.90:
                torque_state = 'warn'
                break
    return {'joints': joints_state, 'velocity': velocity_state, 'torque': torque_state}


def dashboard_state(w, ctx):
    """Everything the classic Dashboard's Overview and Detailed tabs read."""
    cfg, rid = ctx['cfg'], ctx['rid']
    wrench = w.wrench_for(rid) if hasattr(w, 'wrench_for') else None
    force = torque = None
    fmag = tmag = None
    if wrench is not None:
        fx, fy, fz, tx, ty, tz = wrench
        force, torque = [fx, fy, fz], [tx, ty, tz]
        fmag, tmag = math.sqrt(fx * fx + fy * fy + fz * fz), math.sqrt(tx * tx + ty * ty + tz * tz)
    joints = _dash_joints(w, ctx)
    db = cfg.get('admittance_deadbands') or {}
    gripper_range = cfg.get('gripper_range')
    count = w.joint_update_count(rid) if hasattr(w, 'joint_update_count') else 0
    age = None
    if hasattr(w, 'joint_update_time'):
        t = w.joint_update_time(rid)
        age = None if t is None else (time.monotonic() - t) * 1000.0
    vel, eff = _get(w, 'joint_velocity_for', rid), _get(w, 'joint_effort_for', rid)
    pose = w.ee_pose_for(rid) if hasattr(w, 'ee_pose_for') else None
    src = ['driver API' if cfg.get('wrench_from_api') else 'estimated']
    if cfg.get('wrench_gravity_compensated'):
        src.append('gravity-compensated')
    return {
        'modifiers': [name for attr, name in (('_rc_postural_toggle', 'POST'), ('_rc_elbow_toggle', 'ELB'),
                                              ('_rc_hpmp_toggle', 'PMP'), ('_rc_admittance_planner_toggle', 'ADM'))
                      if _checked(w, attr)],
        'joints': joints,
        'ee': _dash_ee(w, ctx),
        'ee_columns': list(EE_COLUMNS),
        'wrench': {'force': force, 'torque': torque, 'force_mag': fmag, 'torque_mag': tmag,
                   'contact': bool(fmag is not None and fmag >= CONTACT_FORCE_N)},
        'safety': _dash_safety(w, ctx, joints),
        'robot': {
            'model': cfg.get('display_name') or _robot(w), 'joints': cfg.get('num_joints'),
            'gripper': ('yes %s' % gripper_range if gripper_range else 'yes') if cfg.get('has_gripper') else 'none',
            'ee frame': cfg.get('ee_frame_name'), 'elbow frame': cfg.get('elbow_frame_name'),
            'torque sign': cfg.get('torque_sign'), 'gravity comp': 'on' if cfg.get('gravity_compensated') else 'off',
            'wrench source': ' · '.join(src), 'urdf': cfg.get('urdf_path'),
        },
        'channels': {
            'joint states': 'offline' if age is None else '%d samples · %.0f ms ago' % (count, age),
            'joint velocity': '%d values' % len(vel) if vel else 'no data',
            'joint effort': '%d values' % len(eff) if eff else 'no data',
            'end-effector pose': 'streaming' if pose is not None else 'no data',
            'tool wrench': 'streaming' if wrench is not None else 'no data',
            'deadbands': ('τ %s · F %s · M %s' % (db.get('joint_tau', '--'), db.get('cart_force', '--'),
                                                  db.get('cart_torque', '--'))
                          + (' · arm F %s' % db['cart_force_hardware'] if 'cart_force_hardware' in db else ''))
                         if db else '--',
        },
    }


# ── Live tuning telemetry ────────────────────────────────────────────────

def live_state(w, ctx):
    """What the Tuning page's feedback strip and operating point read.

    Joint modes: tracking error (commanded - actual, rad) and the commanded
    effort, per joint. Cartesian modes: the Cartesian error and the tool
    wrench. The page picks the selected joint or axis, as the classic page does.
    """
    rid = ctx['rid']
    cmd, pos = _get(w, 'joint_commanded_for', rid), _get(w, 'joint_positions_for', rid)
    eff = _get(w, 'joint_effort_for', rid)
    n = min(len(cmd), len(pos))
    wrench = w.wrench_for(rid) if hasattr(w, 'wrench_for') else None
    cerr = w.cartesian_error_for(rid) if hasattr(w, 'cartesian_error_for') else None
    return {
        'joint_error': [_num(cmd[i] - pos[i]) for i in range(n)],
        'joint_effort': [_num(v) for v in eff],
        'wrench': [_num(v) for v in wrench] if wrench is not None else None,
        'cart_error': [_num(v) for v in cerr] if cerr is not None else None,
    }


# ── Control-bar drawer: panels the classic bar slides up ─────────────────

def drawer_state(w, ctx):
    """Preset poses, live publish, contact ground truth, and what the hardware buttons need."""
    presets = []
    for p in getattr(w, 'presets', []) or []:
        presets.append({'label': '(%.1f, %.1f, %.2f)' % (p.position.x, p.position.y, p.position.z)})
    gt = getattr(w, '_cb_gt_combo', None)
    target = getattr(w, 'control_target', 'simulation')
    return {
        'presets': presets if target == 'simulation' else [],
        'live_publish': _checked(w, '_rc_live_publish'),
        'hardware': target == 'hardware',
        'ground_truth': {
            'current': gt.currentText() if gt is not None and hasattr(gt, 'currentText') else 'none',
            'items': [gt.itemText(i) for i in range(gt.count())] if gt is not None and hasattr(gt, 'count') else ['none'],
        },
    }


# ── Commands: toolbar ────────────────────────────────────────────────────

def _toolbar(w, args, client):
    op = args.get('op')
    value = args.get('value')
    if op == 'keyboard':
        w.set_keyboard_control(bool(value))
        return {'ok': True}
    table = {'bags': ('bags_running', '_toggle_bags', 'ROS bag recording'),
             'plotter': ('plotter_running', '_toggle_plotter', 'the plotter'),
             'viewer': ('contact_twin_running', '_cb_on_contact_twin', 'the contact viewer'),
             'exp': ('', '_quick_start_experiment', 'the quick experiment')}
    if op not in table:
        raise Refused('No such toolbar command', 'bad_args')
    need_confirm(args, 'Starting or stopping needs confirmation')
    attr, handler, label = table[op]
    if op == 'exp' and (not hasattr(w, 'get_extension') or w.get_extension('experiments') is None):
        raise Refused('Enable the Experiments extension in Settings first', 'unavailable')
    w.log_message('INFO', 'Web UI (%s): toggle %s' % (_who(client), label), 'Web')
    getattr(w, handler)()
    update = getattr(w, '_update_status', None)
    if update is not None:
        update()
    return {'ok': True}


def _keyboard_keys(w, args, client):
    """The keys the page holds down while Keyboard Control is on (see keyboard_mixin).

    ``keys`` is the whole list held now; ``tap`` is Tab or a joint number just
    pressed. Letting go (an empty list) is never refused.
    """
    if not hasattr(w, 'keyboard_web_keys'):
        raise Refused('Keyboard Control is not available', 'unavailable')
    keys, tap = args.get('keys') or [], args.get('tap')
    if keys or tap:
        from fic_studio.web_remote.actions import _estop_gate
        try:
            _estop_gate(w)
        except Refused:
            w.keyboard_web_keys([])
            raise
    reply = w.keyboard_web_keys(keys, tap=tap)
    if not reply.get('ok'):
        raise Refused('Keyboard Control is off: turn it on in View', 'keyboard_off')
    return reply


def _station(w, args, client):
    """Pick the robot station the Multi-robot page controls, add one, or re-point this Studio."""
    from fic_studio.fleet.stations import valid_station
    op = args.get('op')
    if op == 'set':
        _local(client)
        # The toolbar's "Robot 1 / Robot 2 / Master": re-points the whole Studio and restarts it.
        need_confirm(args, 'Switching station restarts the Studio and needs confirmation')
        n = args.get('n')
        if not isinstance(n, int) or isinstance(n, bool) or not valid_station(n):
            raise Refused('No such station', 'bad_args')
        if w._tb_session_running():
            raise Refused('Stop the running session before changing station', 'session_running')
        w.settings['general']['station_id'] = n
        w._save_settings()
        w.log_message('WARN', 'Station set to %d from the web UI (%s); restarting' % (n, _who(client)), 'Web')
        restart = getattr(w, '_web_restart', None)
        if restart is not None:
            restart()
        return {'ok': True, 'restarting': restart is not None}
    n = args.get('n')
    if not isinstance(n, int) or isinstance(n, bool) or not 1 <= n <= 99:
        raise Refused('Give a station number from 1 to 99', 'bad_args')
    if op == 'add':
        active = list(getattr(w.node, '_active_robots', []) or [])
        if n in active:
            raise Refused('Station %d is already active' % n, 'bad_args')
        _remember_mode(w)
        w.node.add_robot(n)
        # The robot card never made this robot's safety publishers, so a Safety edit
        # reached robot1 only; make them (remove_robot already drops them) and send
        # the limits now in force.
        if hasattr(w, '_add_robot_safety_publishers'):
            w._add_robot_safety_publishers(n)
            w._publish_safety_limits()
        w._rc_active_rid = 'robot%d' % n
        w._rc_rebuild_robot_selector()
        w._rebuild_rc_control_area()
        w.log_message('INFO', 'Added robot%d (web UI, %s)' % (n, _who(client)), 'Control')
    elif op == 'remove':
        need_confirm(args, 'Removing a robot needs confirmation')
        active = list(getattr(w.node, '_active_robots', []) or [])
        if n not in active:
            raise Refused('Station %d is not active' % n, 'bad_args')
        if n == w.node.station_id or len(active) <= 1:
            raise Refused('This Studio\'s own station cannot be removed', 'refused')
        station = n
        if hasattr(w, '_remove_robot_subscriptions'):
            w._remove_robot_subscriptions(station)
        if hasattr(w, '_remove_robot_safety_publishers'):
            w._remove_robot_safety_publishers(station)
        w.node.remove_robot(station)
        if getattr(w, '_rc_active_rid', '') == 'robot%d' % n:
            w._rc_active_rid = w.node.station_robot_id
        w._rc_rebuild_robot_selector()
        w._rebuild_rc_control_area()
        w.log_message('WARN', 'Removed robot%d (web UI, %s)' % (n, _who(client)), 'Control')
    elif op == 'select':
        if n not in list(getattr(w.node, '_active_robots', []) or []):
            raise Refused('Station %d is not active' % n, 'bad_args')
        _remember_mode(w)
        w._rc_active_rid = 'robot%d' % n
        w._rc_rebuild_robot_selector()
        w._rebuild_rc_control_area()
        _show_robot_mode(w)
        w.log_message('INFO', 'Active robot -> robot%d' % n, 'Control')
    else:
        raise Refused('No such station command', 'bad_args')
    return {'ok': True}


def _remember_mode(w):
    """Note the mode the robot being left was in, so its card and bar can show it later."""
    modes = getattr(w, '_web_modes', None)
    if modes is None:
        modes = w._web_modes = {}
    rid = getattr(w, '_rc_active_rid', '')
    modes[rid] = int(getattr(w, '_rc_selected_mode_idx', 0))


def _show_robot_mode(w):
    """Point the bar's mode readout at the mode the selected robot was last given.

    The bar has one mode state for the Studio. After the robot selector moves,
    show the mode this web session last sent to that robot (nothing is
    published: the readout and the joint / Cartesian surface follow).
    """
    index = (getattr(w, '_web_modes', None) or {}).get(getattr(w, '_rc_active_rid', ''))
    if index is None or index == getattr(w, '_rc_selected_mode_idx', None):
        return
    from fic_studio.mixins.robot_control_mixin import CONTROL_MODE_NAMES
    kind = 'cart' if 'Cart' in CONTROL_MODE_NAMES[index] else 'joint'
    w._rc_selected_mode_idx, w._rc_selected_kind = index, kind
    for hook in ('_cb_sync_mode_buttons', '_cb_place_surface', '_cb_refresh_status'):
        fn = getattr(w, hook, None)
        if callable(fn):
            fn()
    w._update_rc_section_visibility(kind)
    w._rebuild_rc_control_area()


def _station_rid(w, args):
    """The station a pause or stop is for: the one asked for, else the one being driven."""
    n = args.get('n')
    if n is None:
        return w._rc_command_rid()
    if not isinstance(n, int) or isinstance(n, bool) or n not in list(getattr(w.node, '_active_robots', []) or []):
        raise Refused('That station is not active', 'bad_args')
    return 'robot%d' % n


def _robot_pause(w, args, client):
    w.pause_for(_station_rid(w, args))
    return {'ok': True}


def _robot_resume(w, args, client):
    from fic_studio.web_remote.actions import _estop_gate
    _estop_gate(w)
    if not w.resume_for(_station_rid(w, args)):
        raise Refused('That robot is not paused', 'not_paused')
    return {'ok': True}


def _robot_stop(w, args, client):
    w.stop_for(_station_rid(w, args))
    return {'ok': True}


def _live_publish(w, args, client):
    box = getattr(w, '_rc_live_publish', None)
    if box is None:
        raise Refused('Live publish is not available', 'unavailable')
    box.setChecked(bool(args.get('value')))
    return {'ok': True}


def _reset_defaults(w, args, client):
    w._reset_rc_defaults()
    return {'ok': True}


def _preset(w, args, client):
    idx = args.get('index')
    presets = getattr(w, 'presets', []) or []
    if not isinstance(idx, int) or isinstance(idx, bool) or not 0 <= idx < len(presets):
        raise Refused('No such preset', 'bad_args')
    if getattr(w, 'control_target', 'simulation') != 'simulation':
        raise Refused('Preset poses are for simulation only', 'unavailable')
    w.node.publish_position(presets[idx])
    return {'ok': True}


def _contact_truth(w, args, client):
    link = str(args.get('link') or '')
    combo = getattr(w, '_cb_gt_combo', None)
    if combo is None:
        raise Refused('The contact panel is not available', 'unavailable')
    items = [combo.itemText(i) for i in range(combo.count())]
    if link not in items:
        raise Refused('No such link', 'bad_args')
    combo.setCurrentText(link)              # its handler sets the node's parameter
    return {'ok': True}


# ── Commands: menus ──────────────────────────────────────────────────────

#: Menu id -> (handler name, needs confirm, what to tell the log).
MENU_CALLS = {
    'file.new_script': ('_new_script', False),
    'robot.home': ('_rc_on_home', False),
    'sim.toggle_simulation': ('_toggle_simulation', True),
    'sim.toggle_fic': ('_toggle_fic_simulation', True),
    'sim.launch_all': ('_dash_toggle_sim', True),
    'sim.toggle_controller': ('_toggle_controller', True),
    'sim.toggle_driver': ('_toggle_driver', True),
    'tools.open_experiments_folder': ('_open_experiments_folder', False),
}


def _stop_all(w, client):
    for attr, stop in (('simulation_running', '_stop_simulation'), ('fic_running', '_stop_fic_simulation'),
                       ('controller_running', '_stop_controller'), ('driver_running', '_stop_driver'),
                       ('bags_running', '_stop_bags'), ('plotter_running', '_stop_plotter')):
        if getattr(w, attr, False) and hasattr(w, stop):
            getattr(w, stop)()
    w.log_message('INFO', 'All processes stopped (web UI, %s)' % _who(client), 'System')


def _robot_info(w):
    robot = _robot(w)
    cfg = _cfg(w, robot)
    return {'rows': [
        ['Robot', robot], ['Joints', _joint_count(w, robot)], ['Model file', cfg.get('model_xml', 'N/A')],
        ['URDF', cfg.get('urdf_path', 'N/A')], ['Driver', cfg.get('driver_command', 'N/A')],
        ['Home (deg)', cfg.get('q_home_deg', 'N/A')]]}


#: Menu entries that act on the Studio machine itself (its files, its windows, its life).
LOCAL_MENU = ('file.exit', 'tools.open_experiments_folder', 'help.docs', 'file.new_script')


def _local(client):
    if not (client or {}).get('local'):
        raise Refused('That is only available in the Studio window itself', 'local_only')


def _menu(w, args, client):
    """One menu entry that has a Studio handler behind it."""
    item = args.get('id')
    if item in LOCAL_MENU:
        _local(client)
    if item == 'robot.info':
        return {'ok': True, 'info': _robot_info(w)}
    if item == 'sim.stop_all':
        need_confirm(args, 'Stopping every process needs confirmation')
        _stop_all(w, client)
        return {'ok': True}
    if item == 'file.exit':
        need_confirm(args, 'Closing the Studio needs confirmation')
        w.log_message('WARN', 'Studio closed from the web UI (%s)' % _who(client), 'System')
        closer = getattr(w, '_web_close', None)
        if closer is not None:
            closer()
        return {'ok': True}
    if item == 'help.docs':
        opened = w.serve_docs(str(args.get('page') or 'home')) if hasattr(w, 'serve_docs') else False
        return {'ok': True, 'opened': bool(opened)}
    if item == 'help.wizard':
        if not (client or {}).get('local') or not hasattr(w, '_run_first_launch_wizard'):
            raise Refused('The setup wizard is a window on the Studio machine; open it there '
                          '(Help, Re-run Setup Wizard)', 'unavailable')
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(0, w._run_first_launch_wizard)     # a window of its own; this reply goes first
        return {'ok': True}
    if item not in MENU_CALLS:
        raise Refused('No such menu entry', 'bad_args')
    handler, hardware = MENU_CALLS[item]
    if hardware:
        need_confirm(args, 'Starting or stopping a process needs confirmation')
    if not hasattr(w, handler):
        raise Refused('This Studio cannot do that', 'unavailable')
    w.log_message('INFO', 'Web UI (%s): %s' % (_who(client), item), 'Web')
    getattr(w, handler)()
    update = getattr(w, '_update_status', None)
    if update is not None and hardware:
        update()
    return {'ok': True}


# ── Commands: files ──────────────────────────────────────────────────────

def _config_path(w):
    path = getattr(w, '_web_config_path', None)
    if path:
        return path
    from fic_studio.config import XML_FILE_PATH
    return XML_FILE_PATH


def _export(w, args, client):
    """A file the Studio would save with a dialog, handed to the browser to download."""
    what = args.get('what')
    if what == 'snapshot':
        with open(_config_path(w), 'r', encoding='utf-8') as f:
            return {'ok': True, 'filename': 'config_snapshot.xml', 'mime': 'application/xml', 'text': f.read()}
    if what == 'params':
        # the classic File, Export, FIC Parameters as YAML dump; its loop raises on a Cartesian
        # mode, so those are written as their axes here instead
        import yaml
        from fic_studio import xml_manager as xm
        from fic_studio.xml_manager import XMLManager
        mgr = XMLManager(_config_path(w))
        robot = _robot(w)
        data = {'robot': robot, 'modes': {}}
        for mode in mgr.get_available_modes(robot):
            if xm.is_joint_mode(mode):
                joints = {}
                for j in range(_joint_count(w, robot)):
                    jp = mgr.load_joint_params(robot, j, mode)
                    joints[j] = {k: v for k, v in jp.__dict__.items() if v is not None}
                data['modes'][mode] = {'joints': joints}
            else:
                axes = {}
                for axis in ('x', 'y', 'z', 'q'):
                    ap = mgr.load_cartesian_axis(robot, mode, axis)
                    axes[axis] = {k: v for k, v in ap.__dict__.items() if v is not None}
                data['modes'][mode] = {'axes': axes}
        return {'ok': True, 'filename': 'fic_params.yaml', 'mime': 'text/yaml',
                'text': yaml.dump(data, default_flow_style=False, sort_keys=False)}
    if what == 'robots':
        import yaml
        return {'ok': True, 'filename': 'robots_export.yaml', 'mime': 'text/yaml',
                'text': yaml.safe_dump({'version': '1.0', 'robots': getattr(w, 'robot_configs', {})},
                                       sort_keys=False)}
    raise Refused('Nothing to export by that name', 'bad_args')


def _import_snapshot(w, args, client):
    """File, Open, Config Snapshot: the XML arrives as text, is checked, then replaces config.xml."""
    need_confirm(args, 'Replacing config.xml needs confirmation')
    text = args.get('text')
    if not isinstance(text, str) or not text.strip() or len(text) > 4_000_000:
        raise Refused('Send the XML file as text (up to 4 MB)', 'bad_args')
    import xml.etree.ElementTree as ET
    try:
        ET.fromstring(text)
    except ET.ParseError as exc:
        raise Refused('That is not valid XML: %s' % exc, 'bad_args')
    path = _config_path(w)
    backup = path + '.web-backup'
    try:
        with open(path, 'rb') as src, open(backup, 'wb') as dst:
            dst.write(src.read())
    except OSError:
        pass
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    w.log_message('WARN', 'Config snapshot loaded from the web UI (%s); the old file is kept as %s'
                  % (_who(client), os.path.basename(backup)), 'Config')
    return {'ok': True}


def _recent_experiments(w, args, client):
    from fic_studio.config import EXPERIMENTS_ROOT
    root = getattr(w, '_web_experiments_root', None) or EXPERIMENTS_ROOT
    found = []
    if os.path.isdir(root):
        import yaml
        for entry in os.listdir(root):
            meta = os.path.join(root, entry, 'experiment.yaml')
            if os.path.isfile(meta):
                try:
                    with open(meta) as f:
                        name = (yaml.safe_load(f) or {}).get('name', entry)
                except Exception:
                    name = entry
                found.append((os.path.getmtime(meta), entry, str(name)))
    found.sort(reverse=True)
    if args.get('open'):
        names = {e: n for _, e, n in found}
        if args['open'] not in names:
            raise Refused('No such experiment', 'bad_args')
        w._open_experiment_dir(os.path.join(root, args['open']))
        return {'ok': True}
    return {'ok': True, 'items': [{'id': e, 'name': n} for _, e, n in found[:10]]}


# ── Commands: build (Tools menu, Settings, Developer) ────────────────────

def _build_command(w, op, package):
    from fic_studio.platform_utils import colcon_build_command
    ws = w._get_workspace_path()
    if op == 'workspace':
        return ws, colcon_build_command(), None
    if op == 'package':
        src = os.path.join(ws, 'src')
        names = sorted(d for d in os.listdir(src) if os.path.isdir(os.path.join(src, d)) and not d.startswith('.'))
        if package not in names:
            raise Refused('No such package', 'bad_args')
        return ws, colcon_build_command([package]), None
    return ws, colcon_build_command(), 'rm -rf build install log'


def _build(w, args, client):
    """Compile the workspace, one package, or clean and rebuild; output is read back by polling."""
    op = args.get('op')
    state = getattr(w, '_web_build', None)
    if op == 'status':
        return {'ok': True, **_build_view(state)}
    if op == 'packages':
        src = os.path.join(w._get_workspace_path(), 'src')
        names = sorted(d for d in os.listdir(src) if os.path.isdir(os.path.join(src, d)) and not d.startswith('.')) \
            if os.path.isdir(src) else []
        return {'ok': True, 'packages': names}
    if op not in ('workspace', 'package', 'clean'):
        raise Refused('No such build command', 'bad_args')
    need_confirm(args, 'A build needs confirmation' if op != 'clean' else
                 'A clean rebuild deletes build, install and log, and needs confirmation')
    cwd, command, pre = _build_command(w, op, args.get('package'))
    return {'ok': True, **start_build(w, op, cwd, (pre + ' && ' if pre else '') + command, client)}


def start_build(w, label, cwd, full, client):
    """Run one build command in the background; its output is read back by polling ``build`` status."""
    state = getattr(w, '_web_build', None)
    if state and state['running']:
        raise Refused('A build is already running', 'busy')
    if not os.path.isdir(cwd):
        raise Refused('Directory not found: %s' % cwd, 'unavailable')
    state = {'running': True, 'op': label, 'lines': deque(maxlen=BUILD_LINES), 'code': None, 'started': time.time()}
    w._web_build = state

    def run():
        try:
            proc = subprocess.Popen(['bash', '-lc', full], cwd=cwd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                state['lines'].append(line.rstrip())
            state['code'] = proc.wait()
        except Exception as exc:
            state['lines'].append('could not start: %s' % exc)
            state['code'] = -1
        state['running'] = False
    threading.Thread(target=run, daemon=True).start()
    w.log_message('INFO', 'Build started from the web UI (%s): %s' % (_who(client), label), 'Build')
    return _build_view(state)


def _build_view(state):
    if not state:
        return {'running': False, 'op': None, 'lines': [], 'code': None}
    return {'running': state['running'], 'op': state['op'], 'lines': list(state['lines'])[-120:], 'code': state['code']}


HANDLERS = {
    'toolbar': _toolbar,
    'keyboard_keys': _keyboard_keys,
    'station': _station,
    'menu': _menu,
    'robot_pause': _robot_pause,
    'robot_resume': _robot_resume,
    'robot_stop': _robot_stop,
    'live_publish': _live_publish,
    'reset_defaults': _reset_defaults,
    'preset': _preset,
    'contact_truth': _contact_truth,
    'export': _export,
    'import_snapshot': _import_snapshot,
    'recent_experiments': _recent_experiments,
    'build': _build,
}
# ── Dashboard live view: the end effector against its target ─────────────

def _fk_target(w, ctx):
    """Where the commanded joints put the end effector, from the 3D scene's own bodies.

    Read-only; the page does no kinematics. None when the robot has no scene,
    no named end-effector frame, or no commanded joints yet.
    """
    provider = getattr(w, '_web_scene_provider', None)
    frame = ctx['cfg'].get('ee_frame_name')
    cmd = _get(w, 'joint_commanded_for', ctx['rid'])
    if provider is None or not frame or not cmd:
        return None
    try:
        from fic_studio.web_remote import scene as scene_mod
        sc = provider(None)
        names = [b['name'] for b in sc['bodies']]
        if frame not in names:
            return None
        values = {}
        for k, jn in enumerate(sc.get('drive_joints') or ()):
            if k < len(cmd) and _num(cmd[k]) is not None:
                values[jn] = cmd[k]
        pos = scene_mod.pose_bodies(sc, values)[names.index(frame)][0]
        return [float(pos[0]), float(pos[1]), float(pos[2])]
    except Exception:
        return None


def trace_state(w, ctx):
    """Measured and commanded end-effector position, for the Dashboard's live view.

    ``target`` is the commanded point in the robot's base frame: the Cartesian
    target in a Cartesian mode (or whatever the Studio's ``ee_target_for``
    returns), otherwise the forward kinematics of the commanded joints. The
    page keeps the history; this is one sample. ``force`` is the tool force
    (N), drawn from the end effector as an arrow.
    """
    rid = ctx['rid']
    pose = w.ee_pose_for(rid) if hasattr(w, 'ee_pose_for') else None
    ee = None if pose is None else [_num(pose.position.x), _num(pose.position.y), _num(pose.position.z)]
    target, source = None, None
    fn = getattr(w, 'ee_target_for', None)
    if fn is not None:
        try:
            got = fn(rid)
            if got is not None:
                target, source = [_num(v) for v in list(got)[:3]], 'streamed target'
        except Exception:
            pass
    fields = getattr(w, '_rc_pose_fields', {}) or {}
    if target is None and getattr(w, '_rc_selected_kind', 'joint') == 'cart' and 'X' in fields:
        try:
            target, source = [_num(fields[k].commanded()) for k in ('X', 'Y', 'Z')], 'cartesian target'
        except Exception:
            pass
    if target is None:
        target = _fk_target(w, ctx)
        source = 'commanded joints' if target is not None else None
    if target is not None and None in target:
        target = source = None
    if ee is not None and None in ee:
        ee = None
    err = None if ee is None or target is None else math.dist(ee, target)
    wrench = w.wrench_for(rid) if hasattr(w, 'wrench_for') else None
    return {
        'ee': ee, 'target': target, 'source': source, 'error': err, 'frame': 'base',
        'force': None if wrench is None else [_num(v) for v in list(wrench)[:3]],
        'contact_n': CONTACT_FORCE_N,
    }


SECTIONS = (
    ('statusbar', statusbar_state),
    ('toolbar', toolbar_state),
    ('stations', stations_state),
    ('dashboard', dashboard_state),
    ('trace', trace_state),
    ('live', live_state),
    ('drawer', drawer_state),
)
