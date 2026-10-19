"""What the web remote can see and do. Everything here runs on the Qt thread.

Commands go through the very handlers the Studio's own control bar and
toolbar call (``_on_rc_mode_selected``, ``_rc_publish_joints``,
``_tb_toggle_start``, ``_emergency_stop``...), and joint / pose targets are
written into the desktop's own fields before publishing. So a phone and the
desktop can never disagree about what a button does, and each shows what the
other commanded.

What is added on top is only what a remote needs: a check that the phone is
looking at the robot the Studio is driving, a short hold-off after an E-stop,
and confirmation for the two things that act without a visible target (Home
during a sequence, and starting or stopping a session).
"""
import math
import time

from fic_studio import audit, notices, theme
from fic_studio.web_remote import chrome as _chrome
from fic_studio.web_remote import help as _help
from fic_studio.web_remote import robot_config as _robot_config
from fic_studio.web_remote import safety as _safety
from fic_studio.web_remote import settings_page as _settings
from fic_studio.web_remote import topics as _topics
from fic_studio.web_remote import tuning as _tuning
from fic_studio.web_remote.common import (  # noqa: F401
    Refused, _arg_num, _num, _who, need_confirm, scoped_to_selected,
)
from fic_studio.math_helpers import _quat_to_axis_angle, _quat_to_euler_xyz
from fic_studio.mixins.control_bar_mixin import (
    MODE_MENUS, MODE_ROW_LABELS, NO_CARTESIAN, NO_SEQUENCES, VR_NO_GRIPPER,
)
from fic_studio.mixins.robot_control_mixin import (
    CONTROL_MODE_DISPLAY, CONTROL_MODE_TABS, mode_display_name,
)

#: Motion commands are refused for this long after an E-stop, so a jog
#: packet already in flight cannot re-command the arm as it stops.
ESTOP_HOLDOFF_S = 2.0

#: Log lines sent to the browser.
LOG_TAIL = 25

#: Web option name -> the Studio checkbox that owns it.
OPTION_TOGGLES = {
    'admittance_planner': ('_rc_admittance_planner_toggle', 'joint'),
    'postural': ('_rc_postural_toggle', 'cart'),
    'elbow': ('_rc_elbow_toggle', 'cart'),
    'hpmp': ('_rc_hpmp_toggle', 'cart'),
}

TARGETS = ('simulation', 'hardware', 'vr')

#: Width the Studio's gripper slider maps 100 % to (robot_control_mixin).
GRIPPER_MAX_WIDTH_M = 0.08


# ── Small helpers ────────────────────────────────────────────────────────

def _robot(w):
    return w.robot_dropdown.currentText()


def _cfg(w, robot):
    return getattr(w, 'robot_configs', {}).get(robot, {})


def _rid(w):
    if hasattr(w, '_rc_command_rid'):
        return w._rc_command_rid()
    return w.node.station_robot_id


def _sequences_ext(w):
    getter = getattr(w, '_cb_sequences', None)
    return getter() if getter is not None else None


def availability(w):
    """Which parts of the control surface may be used right now, and why not."""
    cfg = _cfg(w, _robot(w))
    legged = cfg.get('robot_category') == 'legged'
    vr = getattr(w, 'control_target', 'simulation') == 'vr'
    return {
        'command': True,
        'joint': True,
        'cart': not legged,
        'gripper': not vr and bool(cfg.get('has_gripper', True)),
        'home': True,
        'sequences': _sequences_ext(w) is not None,
        'reason': '',
        'cart_reason': NO_CARTESIAN if legged else '',
        'gripper_reason': VR_NO_GRIPPER if vr else '',
        'sequences_reason': NO_SEQUENCES,
    }


def _estop_gate(w):
    """Refuse anything that moves a robot or starts a process while an E-stop is latched
    (until Reset), and for a moment after one fired."""
    latch = getattr(w, 'estop_latch_state', None)
    if latch is not None and latch().get('latched'):
        raise Refused('E-STOP is latched. Check the robot, then press Reset on the red banner', 'estop_latched')
    if time.monotonic() - getattr(w, '_web_estop_time', float('-inf')) < ESTOP_HOLDOFF_S:
        raise Refused('E-stop just fired — wait a moment', 'estop_holdoff')


def _require_command(w, args, kind=None):
    """Refuse motion after an E-stop, or on the wrong robot."""
    _estop_gate(w)
    expected = args.get('robot')
    if expected is not None and expected != _robot(w):
        raise Refused('The Studio switched robot to %s — reloading' % _robot(w),
                      'robot_changed')
    avail = availability(w)
    if kind == 'cart' and not avail['cart']:
        raise Refused(avail['cart_reason'], 'no_cartesian')
    if kind is not None and getattr(w, '_rc_selected_kind', 'joint') != kind:
        raise Refused('Select a %s mode first' % ('Joint' if kind == 'joint' else 'Cartesian'),
                      'wrong_kind')


# ── Commands ─────────────────────────────────────────────────────────────

def _estop(w, args, client):
    # Stamped first so anything queued behind the E-stop is refused.
    w._web_estop_time = time.monotonic()
    w.log_message('ERROR', 'E-STOP from web remote (%s)' % _who(client), 'Web')
    w._estop_source = _who(client)
    if hasattr(w, '_loco_velocity'):
        w._loco_velocity = (0.0, 0.0, 0.0)        # a held walk command ends with the stop
        w._loco_hold_until = None
    w._emergency_stop()


def _estop_reset(w, args, client):
    """Clear a latched E-stop. Starts nothing; asks first (the page says what to check)."""
    need_confirm(args, 'Resetting the E-STOP needs confirmation')
    if not w._estop_latch_reset(_who(client)):
        return {'ok': True, 'already': True}


def _notices_list(w, args, client):
    return {'ok': True, 'items': notices.history(w)}


def _notices_clear(w, args, client):
    return {'ok': True, 'cleared': notices.clear(w, _who(client))}


def _set_mode(w, args, client):
    index = int(_arg_num(args, 'index'))
    if not 0 <= index < len(CONTROL_MODE_TABS):
        raise Refused('No such mode', 'bad_args')
    name, kind = CONTROL_MODE_TABS[index]
    _require_command(w, args)
    if kind == 'cart' and not availability(w)['cart']:
        raise Refused(availability(w)['cart_reason'], 'no_cartesian')
    with scoped_to_selected(w):
        w._on_rc_mode_selected(index, name)
    modes = getattr(w, '_web_modes', None)
    if modes is None:
        modes = w._web_modes = {}
    modes[getattr(w, '_rc_active_rid', '')] = index      # what each robot was last told
    paused = getattr(w, '_paused_rids', None)
    if paused is not None:
        paused.discard(getattr(w, '_rc_active_rid', ''))   # a new mode ends a pause


def _set_option(w, args, client):
    name = args.get('name')
    if name not in OPTION_TOGGLES:
        raise Refused('No such option', 'bad_args')
    _require_command(w, args)
    box = getattr(w, OPTION_TOGGLES[name][0], None)
    if box is None:
        raise Refused('That option is not available', 'unavailable')
    with scoped_to_selected(w):
        box.setChecked(bool(args.get('value')))   # its toggled handler publishes


def _desired_wrench(w, args, client):
    """Send the Cartesian admittance's desired wrench [Fx, Fy, Fz, Mx, My, Mz]."""
    _require_command(w, args)
    raw = args.get('values')
    if not isinstance(raw, list) or len(raw) != 6:
        raise Refused('The desired wrench is six numbers', 'bad_args')
    values = [_num(v) for v in raw]
    if any(v is None for v in values):
        raise Refused('The desired wrench is six numbers', 'bad_args')
    with scoped_to_selected(w):
        w._publish_desired_wrench(values)
    return {'ok': True, 'values': values}


def _set_profile(w, args, client):
    _require_command(w, args)
    combo = getattr(w, '_rc_profile_dropdown', None)
    index = combo.findData(str(args.get('name'))) if combo is not None else -1
    if index < 0:
        raise Refused('No such config', 'bad_args')
    if index != combo.currentIndex():
        with scoped_to_selected(w):
            combo.setCurrentIndex(index)          # mirrors the menus, publishes
        w.log_message('INFO', 'Active config → %s (web remote)' % args.get('name'), 'Control')


def _send_joints(w, args, client):
    _require_command(w, args, kind='joint')
    fields = list(getattr(w, '_rc_joint_fields', []) or [])
    meta = getattr(w, '_rc_joint_meta', None) or []
    degrees = args.get('degrees')
    if not isinstance(degrees, list) or len(degrees) != len(fields) or not fields:
        raise Refused('The joint list changed — reloading', 'joints_changed')
    for i, (field, value) in enumerate(zip(fields, degrees)):
        if value is None or (i < len(meta) and not meta[i].get('actuated', True)):
            continue
        value = _num(value)
        if value is None:
            raise Refused('Joint %d is not a number' % (i + 1), 'bad_args')
        field.set_commanded(value)                # clamps to the joint's limits
    w._rc_publish_joints()
    return {'ok': True, 'commanded': [f.commanded() for f in fields]}


def _send_pose(w, args, client):
    _require_command(w, args, kind='cart')
    fields = getattr(w, '_rc_pose_fields', {}) or {}
    if not all(k in fields for k in ('X', 'Y', 'Z', 'axis X', 'axis Y', 'axis Z', 'angle')):
        raise Refused('The Cartesian controls are not ready', 'unavailable')
    values = {k: _arg_num(args, k) for k in ('x', 'y', 'z', 'ax', 'ay', 'az', 'angle')}
    if math.sqrt(values['ax'] ** 2 + values['ay'] ** 2 + values['az'] ** 2) < 1e-9 \
            and abs(values['angle']) > 1e-9:
        raise Refused('The rotation axis cannot be zero', 'bad_args')
    fields['X'].set_commanded(values['x'])
    fields['Y'].set_commanded(values['y'])
    fields['Z'].set_commanded(values['z'])
    fields['axis X'].set_value(values['ax'])
    fields['axis Y'].set_value(values['ay'])
    fields['axis Z'].set_value(values['az'])
    fields['angle'].set_value(values['angle'])
    w._rc_publish_pose_from_fields()


def _gripper(w, args, client):
    _require_command(w, args)
    avail = availability(w)
    if not avail['gripper']:
        raise Refused(avail['gripper_reason'] or 'This robot has no gripper', 'unavailable')
    percent = int(round(min(100.0, max(0.0, _arg_num(args, 'percent')))))
    slider = w._rc_gripper_slider
    if slider.value() == percent:
        w._on_rc_gripper_changed(percent)         # re-send an unchanged width
    else:
        slider.setValue(percent)                  # valueChanged publishes


def _home(w, args, client):
    _require_command(w, args)
    if getattr(w, 'control_target', 'simulation') == 'hardware' and not args.get('confirm'):
        raise Refused('Home moves the real arm. Confirm first', 'confirm')
    ext = _sequences_ext(w)
    if ext is not None and getattr(ext, 'is_playing', False) and not args.get('confirm'):
        raise Refused('A sequence is playing. Home anyway?', 'confirm')
    with scoped_to_selected(w):
        w._rc_on_home()


def _set_robot(w, args, client):
    """Pick the robot the Studio drives (the toolbar's robot selector)."""
    name = str(args.get('name') or '')
    combo = w.robot_dropdown
    index = combo.findText(name) if hasattr(combo, 'findText') else -1
    if index < 0:
        raise Refused('No such robot', 'bad_args')
    if w._tb_session_running():
        raise Refused('Stop the running session before changing robot', 'session_running')
    if index != combo.currentIndex():
        combo.setCurrentIndex(index)              # the toolbar's own handlers follow


def _set_target(w, args, client):
    target = args.get('target')
    if target not in TARGETS:
        raise Refused('No such target', 'bad_args')
    if w._tb_session_running():
        raise Refused('Stop the running session before changing target', 'session_running')
    if target != getattr(w, 'control_target', 'simulation'):
        audit.record('target', _who(client), target)
        live = getattr(w, '_rc_live_publish', None)
        if target == 'hardware' and live is not None and live.isChecked():
            live.setChecked(False)              # a slider must not stream to a real arm unasked
            w.log_message('INFO', 'Live publish turned off for hardware', 'Web')
    w._tb_set_target(target)


def _session(w, args, client):
    op = args.get('op')
    if op not in ('start', 'stop'):
        raise Refused('No such session command', 'bad_args')
    if not args.get('confirm'):
        raise Refused('Starting or stopping needs confirmation', 'confirm')
    running = w._tb_session_running()
    if (op == 'start') == running:
        return {'ok': True, 'already': True}
    target = getattr(w, 'control_target', 'simulation')
    if op == 'start':
        _estop_gate(w)
        robot = _robot(w)
        # _start_driver would otherwise pop a modal dialog on the desktop,
        # where nobody holding a phone can dismiss it.
        # An inner-loop MCU that can drive the robot is its driver: a robot
        # with only an ``mcu:`` adapter needs no vendor driver command.
        if target == 'hardware' and not w._resolve_driver_command(robot) and \
                _inner_loop_state(w, {}).get('path') != 'mcu':
            raise Refused('No driver command and no inner-loop MCU for %s' % robot, 'no_driver')
    w.log_message('WARN' if target == 'hardware' else 'INFO',
                  'Web remote (%s): %s %s session' % (_who(client), op, target), 'Web')
    audit.record('session_' + op, _who(client), target, target=target)
    w._tb_toggle_start()


def _teleop(w, args, client):
    """Answer a teleoperation request, or end a pairing, from the browser.

    On hardware the Studio asks its operator in a modal dialog
    (``session.py`` ``confirm_request``), and a dialog on a robot PC's desktop
    is only reachable over NoMachine, which on 2026-09-30 was itself what kept
    powering robot1 off. A request that can only be answered there is a request
    that often cannot be answered at all: it expires after ANSWER_S, 30 s.

    This does not bypass the decision, it relocates it. ``accept()`` is the
    same call the dialog makes, so the different-robot check, the
    already-following check and ``begin_following()`` putting this arm into
    Mode 1 with the planner off all still happen, here, on this machine.
    """
    teleop = getattr(w, '_teleop', None)
    if teleop is None:
        raise Refused('This Studio has no teleoperation', 'unavailable')
    op = args.get('op')
    if op not in ('accept', 'reject', 'stop', 'start'):
        raise Refused('No such teleop command', 'bad_args')
    if not args.get('confirm'):
        raise Refused('Answering a pairing needs confirmation', 'confirm')
    if op == 'start':
        return _teleop_start(w, teleop, args, client)
    if op == 'accept':
        _estop_gate(w)

    me = w.node.station_id
    sid = args.get('sid')
    s = teleop.sessions.get(sid) if sid else None
    if op in ('accept', 'reject'):
        # The phone shows what was pending a moment ago; by the time a thumb
        # lands it may have expired, so fall back to whatever is pending now
        # rather than refusing on a stale sid.
        if s is None or s.state != 'pending' or s.follower != me:
            waiting = [x for x in teleop.sessions.values()
                       if x.state == 'pending' and x.follower == me]
            s = waiting[-1] if waiting else None
        if s is None:
            raise Refused('Nothing is waiting for an answer here', 'nothing_pending')

    target = getattr(w, 'control_target', 'simulation')
    level = 'WARN' if target == 'hardware' else 'INFO'
    if op == 'accept':
        w.log_message(level, 'Web remote (%s): following station %d (session %s)'
                      % (_who(client), s.leader, s.sid), 'Teleop')
        if not teleop.accept(s.sid):
            raise Refused('This station cannot follow right now', 'refused')
        return {'ok': True, 'sid': s.sid}
    if op == 'reject':
        w.log_message(level, 'Web remote (%s): refused station %d'
                      % (_who(client), s.leader), 'Teleop')
        teleop.reject(s.sid, 'refused from the web remote')
        return {'ok': True, 'sid': s.sid}
    w.log_message(level, 'Web remote (%s): ending teleoperation' % _who(client), 'Teleop')
    if s is not None:
        teleop.stop(s.sid, 'stopped from the web remote')
    else:
        teleop.stop_all('stopped from the web remote')
    return {'ok': True}


def _kortex(w, args, client):
    """Clear faults / Home through the arm's Kortex API (one-switch layout only).

    The Kortex supervisor refuses both while the MCU or a driver has the arm,
    and reports the outcome in the state's kortex.last. E-stop is not here:
    the web E-STOP already reaches the arm through w._emergency_stop().
    """
    op = args.get('op')
    if op not in ('clear_faults', 'home'):
        raise Refused('No such arm command', 'bad_args')
    if op == 'home' and not args.get('confirm'):
        raise Refused('Homing moves the arm: it needs confirmation', 'confirm')
    _require_command(w, args)
    command = getattr(w, 'kortex_command', None)
    if command is None:
        raise Refused('This Studio has no Kortex support', 'unavailable')
    sent, why = command(op)
    if not sent:
        raise Refused(why, 'unavailable')
    w.log_message('WARN' if op == 'home' else 'INFO',
                  'Web remote (%s): Kortex %s' % (_who(client), op.replace('_', ' ')), 'Web')


def _sequence(w, args, client):
    """Quick Actions on the Sequences extension: play / loop / step / stop a sequence, make a new
    one, set the playback speed, run a trajectory template or save one as a sequence."""
    ext = _sequences_ext(w)
    if ext is None:
        raise Refused(NO_SEQUENCES, 'unavailable')
    op = args.get('op')
    try:
        if op == 'stop':
            ext.stop()
            return None
        if op == 'new':
            return {'ok': True, 'result': {'name': ext.new_sequence(args.get('name'))}}
        if op == 'speed':
            ext.set_speed(args.get('value'))
            return None
        if op == 'save_trajectory':
            return {'ok': True, 'result': {'name': ext.save_trajectory(
                args.get('name'), args.get('kind'), args.get('size'), args.get('plane'))}}
        if op == 'trajectory':
            _require_command(w, args)
            if ext.target_is_hardware() and args.get('hardware_ok') is not True:
                raise Refused('The trajectory goes to the REAL robot: confirm to run it', 'confirm_hw')
            return {'ok': True, 'result': {'duration': ext.run_trajectory(
                args.get('kind'), args.get('size'), args.get('period'), args.get('cycles'),
                args.get('plane'), hardware_ok=args.get('hardware_ok') is True)}}
        if op not in ('play', 'loop', 'step'):
            raise Refused('No such sequence command', 'bad_args')
        _require_command(w, args)
        name = args.get('name')
        if name not in ext.list_sequences():
            raise Refused('No such sequence', 'bad_args')
        if op == 'step':
            return {'ok': True, 'result': {'index': ext.step(name)}}
        ext.play(name, loop=(op == 'loop'))
    except ValueError as exc:
        raise Refused(str(exc), 'refused')
    return None


def _quick_action(w, args, client):
    """Run an action an extension added to Quick Actions (ExtensionHost.add_quick_action)."""
    act = (getattr(w, '_quick_actions', None) or {}).get(args.get('key'))
    if act is None:
        raise Refused('No such quick action', 'bad_args')
    _require_command(w, args)
    try:
        msg = act['run'](args.get('choice'))
    except ValueError as exc:
        raise Refused(str(exc), 'refused')
    return {'ok': True, 'result': {'message': str(msg) if msg else '%s: done' % act['label']}}


def _capture(w, args, client):
    ext = _sequences_ext(w)
    if ext is None:
        raise Refused(NO_SEQUENCES, 'unavailable')
    kind, name = args.get('kind'), args.get('sequence')
    if kind not in ('cartesian', 'joint', 'gripper'):
        raise Refused('No such waypoint kind', 'bad_args')
    if name not in ext.list_sequences():
        raise Refused('No such sequence', 'bad_args')
    ext.capture(kind, into=name)


def _calibrate(w, args, client):
    w._cb_on_calibrate()


def _panel(w, args, client):
    """Use one control on an extension's page, as the Studio's own page would."""
    ext_id = str(args.get('extension') or '')
    path, kind = str(args.get('path') or ''), str(args.get('kind') or '')
    if not ext_id or not path or not kind:
        raise Refused('extension, path and kind are required', 'bad_request')
    try:
        return {'ok': True, 'panel': w._web_panel_act(ext_id, path, kind, args.get('value'))}
    except ValueError as exc:
        raise Refused(str(exc), 'stale_panel')


def _teleop_start(w, teleop, args, client):
    """Ask for one station to lead another, as the Multi-robot page's Start does."""
    _estop_gate(w)
    try:
        leader, follower = int(args.get('leader')), int(args.get('follower'))
    except (TypeError, ValueError):
        raise Refused('Pick a leader and a follower station', 'bad_args')
    if leader == follower:
        raise Refused('Pick two different stations', 'bad_args')
    scale = _num(args.get('scale'), 1.0)
    speed = _num(args.get('max_speed_deg'), 35.0)
    if not 0.05 <= scale <= 2.0:
        raise Refused('Scale must be between 0.05 and 2', 'bad_args')
    if not 5.0 <= speed <= 90.0:
        raise Refused('Max speed must be between 5 and 90 deg/s', 'bad_args')
    directory = getattr(w, '_fleet_dir', None)
    lcard = (directory.station(leader) if directory else None) or {}
    fcard = (directory.station(follower) if directory else None) or {}
    if lcard.get('robot') and fcard.get('robot') and lcard['robot'] != fcard['robot']:
        raise Refused('Teleoperation between different robots is not supported yet '
                      '(%s and %s)' % (lcard['robot'], fcard['robot']), 'refused')
    session = teleop.request(leader, follower, scale=scale, max_speed=math.radians(speed),
                             leader_robot=lcard.get('robot', ''),
                             leader_joints=int(lcard.get('joints', 0) or 0))
    w.log_message('INFO', 'Teleop requested from the web UI (%s): station %d leads %d'
                  % (_who(client), leader, follower), 'Teleop')
    audit.record('teleop_request', _who(client), 'station %d leads %d' % (leader, follower))
    return {'ok': True, 'sid': getattr(session, 'sid', None)}


#: Process id -> (running attribute, start method, stop method, label, hardware).
PROCESSES = {
    'simulation': ('simulation_running', '_start_simulation', '_stop_simulation',
                   'Simulation', False),
    'fic': ('fic_running', '_start_fic_simulation', '_stop_fic_simulation',
            'FIC controller (simulation)', False),
    'driver': ('driver_running', '_start_driver', '_stop_driver', 'Robot driver', True),
    'controller': ('controller_running', '_start_controller', '_stop_controller',
                   'FIC controller (hardware)', True),
    'locomotion': ('locomotion_running', '_start_locomotion', '_stop_locomotion',
                   'Locomotion', False),
    'vr': ('vr_relay_running', '_start_vr', '_stop_vr', 'VR relay', False),
    'bags': ('bags_running', '_start_bags', '_stop_bags', 'Bag recording', False),
}


def _process(w, args, client):
    """Start or stop one of the processes the Session page lists."""
    pid, op = args.get('id'), args.get('op')
    if pid not in PROCESSES or op not in ('start', 'stop'):
        raise Refused('No such process command', 'bad_args')
    need_confirm(args, 'Starting or stopping a process needs confirmation')
    attr, start, stop, label, hardware = PROCESSES[pid]
    running = bool(getattr(w, attr, False))
    if (op == 'start') == running:
        return {'ok': True, 'already': True}
    if op == 'start':
        _estop_gate(w)
        if pid == 'driver' and not w._resolve_driver_command(_robot(w)) and \
                _inner_loop_state(w, {}).get('path') != 'mcu':
            raise Refused('No driver command and no inner-loop MCU for %s' % _robot(w), 'no_driver')
        if pid == 'locomotion' and not _cfg(w, _robot(w)).get('robot_category') == 'legged':
            raise Refused('%s is not a legged robot' % _robot(w), 'unavailable')
    w.log_message('WARN' if hardware else 'INFO',
                  'Web remote (%s): %s %s' % (_who(client), op, label), 'Web')
    audit.record('process_' + op, _who(client), label, target=getattr(w, 'control_target', ''))
    getattr(w, start if op == 'start' else stop)()
    update = getattr(w, '_update_status', None)
    if update is not None:
        update()


def _process_list(w):
    out = []
    for pid, (attr, _start, _stop, label, hardware) in PROCESSES.items():
        out.append({'id': pid, 'label': label, 'running': bool(getattr(w, attr, False)),
                    'hardware': hardware})
    return out


# ── Log ──────────────────────────────────────────────────────────────────
LOG_READ_MAX = 500


def _log_entries(w):
    viewer = getattr(w, 'log_viewer', None)
    return list(getattr(viewer, '_all_logs', []) or [])


def _log_row(e):
    return {'seq': e.get('seq'), 'time': e.get('timestamp', ''), 'level': e.get('level', ''),
            'source': e.get('source', ''), 'message': str(e.get('message', ''))[:600]}


def _log_read(w, args, client):
    """Log lines newer than ``after`` (a sequence number), at most ``limit``."""
    after = args.get('after')
    after = int(after) if _num(after) is not None else None
    limit = int(min(LOG_READ_MAX, max(1, _num(args.get('limit'), 200))))
    entries = _log_entries(w)
    if after is not None:
        entries = [e for e in entries if (e.get('seq') or 0) > after]
    entries = entries[-limit:]
    seqs = [e.get('seq') or 0 for e in _log_entries(w)]
    return {'ok': True, 'entries': [_log_row(e) for e in entries], 'last': max(seqs) if seqs else 0}


def _log_clear(w, args, client):
    need_confirm(args, 'Clearing the log needs confirmation')
    viewer = getattr(w, 'log_viewer', None)
    if viewer is None:
        raise Refused('There is no log', 'unavailable')
    viewer.clear_logs()
    w.log_message('INFO', 'Log cleared from the web UI (%s)' % _who(client), 'Web')


#: The most audit lines one read returns.
AUDIT_READ_MAX = 2000


def _audit_read(w, args, client):
    """The audit log, newest last: the last ``limit`` lines (read-only; it cannot be cleared here)."""
    limit = int(min(AUDIT_READ_MAX, max(1, _num(args.get('limit'), 500))))
    return {'ok': True, 'entries': audit.tail(limit), 'path': audit.path()}


# ── Safety and tuning ────────────────────────────────────────────────────

def _safety_set(w, args, client):
    return _safety.set_limits(w, args, client)


def _tuning_info(w, args, client):
    return _tuning.info(w, args, client)


def _tuning_get(w, args, client):
    return _tuning.get(w, args, client)


def _tuning_plot(w, args, client):
    return _tuning.plot(w, args, client)


def _tuning_save(w, args, client):
    return _tuning.save(w, args, client)


def _tuning_trial(w, args, client):
    if args.get('op') in ('start', 'apply'):      # these change what the robot runs now; keep and discard always work
        _require_command(w, args)
    return _tuning.trial(w, args, client)


def _tuning_xml(w, args, client):
    return _tuning.xml_section(w, args, client)


def _tuning_profile(w, args, client):
    return _tuning.profile(w, args, client)


# ── Fleet ────────────────────────────────────────────────────────────────

def _fleet_station(w, args):
    directory = getattr(w, '_fleet_dir', None)
    if directory is None:
        raise Refused('This Studio is not on a fleet', 'unavailable')
    try:
        n = int(args.get('station'))
    except (TypeError, ValueError):
        raise Refused('Pick a station', 'bad_args')
    card = directory.station(n)
    if not card:
        raise Refused('No Studio answers for that station', 'bad_args')
    if n == w.node.station_id:
        raise Refused('That is this Studio — use the Session page', 'bad_args')
    return n, card


def _fleet(w, args, client):
    """Lab-wide and per-station fleet commands."""
    op = args.get('op')
    if op == 'estop_all':                       # like E-STOP: no confirmation
        w._web_estop_time = time.monotonic()
        w.log_message('ERROR', 'E-STOP ALL from web remote (%s)' % _who(client), 'Web')
        audit.record('estop_all', _who(client), 'E-STOP ALL')
        w._estop_source = 'E-STOP ALL: ' + _who(client)
        w._fleet_estop_all()
        return {'ok': True}
    if op == 'pause_all':
        w.log_message('WARN', 'PAUSE ALL from web remote (%s)' % _who(client), 'Web')
        audit.record('pause_all', _who(client), 'PAUSE ALL')
        w._fleet_pause_all()
        return {'ok': True}
    if op == 'resume_all':
        _estop_gate(w)
        w.log_message('INFO', 'RESUME ALL from web remote (%s)' % _who(client), 'Web')
        audit.record('resume_all', _who(client), 'RESUME ALL')
        w._fleet_resume_all()
        return {'ok': True}
    if op == 'session':
        n, card = _fleet_station(w, args)
        action = args.get('session_op')
        target = args.get('target')
        if action not in ('start', 'stop'):
            raise Refused('No such session command', 'bad_args')
        need_confirm(args, 'Starting or stopping another station needs confirmation')
        if action == 'start':
            if target not in ('simulation', 'hardware'):
                raise Refused('Pick simulation or hardware', 'bad_args')
            if target == 'hardware' and args.get('hardware') is not True:
                raise Refused('Starting hardware needs a second confirmation', 'confirm',
                              hardware=True)
        w.log_message('WARN' if target == 'hardware' else 'INFO',
                      'Web remote (%s): station %d %s %s' % (_who(client), n, action, target or ''),
                      'Web')
        w._fleet_session_send(n, action, target if action == 'start' else None)
        return {'ok': True}
    if op == 'drive':
        n, card = _fleet_station(w, args)
        if card.get('target') == 'hardware':
            need_confirm(args, 'That arm is on hardware — confirm to drive it from here')
        w._fleet_drive_now(n)
        return {'ok': True}
    raise Refused('No such fleet command', 'bad_args')


# ── Locomotion ───────────────────────────────────────────────────────────
#: A held velocity lapses after this long unless the page renews it.
LOCO_HOLD_S = 0.6


def _require_legged(w):
    if _cfg(w, _robot(w)).get('robot_category') != 'legged':
        raise Refused('%s is not a legged robot' % _robot(w), 'unavailable')


def _locomotion(w, args, client):
    from fic_studio.mixins.locomotion_mixin import GAIT_COMMANDS, GAIT_TUNING, VELOCITY_AXES
    _require_legged(w)
    op = args.get('op')
    if op == 'stop':
        w._loco_stop()
        return {'ok': True}
    _require_command(w, args)
    if op == 'gait':
        command = args.get('command')
        if command not in GAIT_COMMANDS:
            raise Refused('No such gait command', 'bad_args')
        w._loco_send(command)
    elif op == 'velocity':
        values = []
        for key, _label, limit in VELOCITY_AXES:
            v = _num(args.get(key), 0.0)
            if abs(v) > limit + 1e-9:
                raise Refused('%s is beyond +/-%g' % (key, limit), 'bad_args')
            values.append(v)
        w._loco_velocity = tuple(values)
        w._loco_hold_until = (time.monotonic() + LOCO_HOLD_S) if any(values) else None
    elif op == 'set':
        spec = {k: (lo, hi) for k, _l, lo, hi, _d, _s in GAIT_TUNING}
        key = args.get('key')
        if key not in spec:
            raise Refused('No such gait parameter', 'bad_args')
        value = _arg_num(args, 'value')
        if not spec[key][0] <= value <= spec[key][1]:
            raise Refused('%s must be between %g and %g' % ((key,) + spec[key]), 'bad_args')
        w._loco_set(key, round(value, 3))
    else:
        raise Refused('No such locomotion command', 'bad_args')
    return {'ok': True}


def _ext(w, args, client):
    """Run an action on an extension's own web page (its web_action())."""
    ext_id, name = str(args.get('extension') or ''), str(args.get('name') or '')
    if not ext_id or not name:
        raise Refused('extension and name are required', 'bad_request')
    payload = args.get('args') or {}
    if not isinstance(payload, dict):
        raise Refused('args must be an object', 'bad_args')
    try:
        result = w._web_ext_call(ext_id, name, payload, bool((client or {}).get('local')))
    except LookupError as exc:
        raise Refused(str(exc), 'no_such_extension')
    except PermissionError as exc:
        raise Refused(str(exc), 'local_only')
    except ValueError as exc:
        raise Refused(str(exc), 'refused')
    except Exception as exc:
        w.log_message('ERROR', '%s.%s failed: %s' % (ext_id, name, exc), 'Extensions')
        raise Refused('%s failed: %s' % (name, exc), 'failed')
    return {'ok': True, 'result': result}


HANDLERS = {
    'panel': _panel,
    'ext': _ext,
    'estop': _estop,
    'estop_reset': _estop_reset,
    'notices_clear': _notices_clear,
    'notices_list': _notices_list,
    'set_mode': _set_mode,
    'set_option': _set_option,
    'desired_wrench': _desired_wrench,
    'set_profile': _set_profile,
    'send_joints': _send_joints,
    'send_pose': _send_pose,
    'gripper': _gripper,
    'home': _home,
    'set_target': _set_target,
    'set_robot': _set_robot,
    'kortex': _kortex,
    'session': _session,
    'teleop': _teleop,
    'sequence': _sequence,
    'quick_action': _quick_action,
    'capture': _capture,
    'contact_calibrate': _calibrate,
    'process': _process,
    'log_read': _log_read,
    'log_clear': _log_clear,
    'audit_read': _audit_read,
    'safety_set': _safety_set,
    'tuning_info': _tuning_info,
    'tuning_get': _tuning_get,
    'tuning_plot': _tuning_plot,
    'tuning_save': _tuning_save,
    'tuning_trial': _tuning_trial,
    'tuning_profile': _tuning_profile,
    'tuning_xml': _tuning_xml,
    'fleet': _fleet,
    'locomotion': _locomotion,
}


HANDLERS.update(_chrome.HANDLERS)
HANDLERS.update(_settings.HANDLERS)
HANDLERS.update(_robot_config.HANDLERS)
HANDLERS.update(_topics.HANDLERS)
HANDLERS.update(_help.HANDLERS)

#: Commands only the trusted desktop session may send: they change the Studio's
#: own configuration, files or build, which no phone or remote account should.
LOCAL_ONLY = frozenset((
    'settings_set', 'settings_reset', 'settings_extensions', 'settings_gamepad', 'settings_web', 'settings_vr',
    'robots_save', 'robots_add', 'robots_duplicate', 'robots_delete', 'robots_import', 'robots_upload', 'robots_import_yaml',
    'robots_compile_driver', 'robots_open_folder', 'robots_viewer',
    'import_snapshot', 'build', 'recent_experiments',
))

#: Commands that change what keeps the robot safe, or remove a record of what
#: was done. With accounts on, only an admin may send them from a browser.
ADMIN_ONLY = frozenset(('safety_set', 'tuning_save', 'log_clear'))

#: Commands a simulation-only account may send whatever the target is.
SIM_ONLY_ALWAYS = ('estop',)


def _sim_only_problem(w, action, args):
    """Say why a simulation-only account may not run this, or None.

    Someone who signed themselves up may use the robots in simulation and
    nothing else: they cannot move the target off simulation, and they cannot
    command anything while the Studio is on hardware or VR.
    """
    if action in SIM_ONLY_ALWAYS:
        return None
    if action == 'set_target' and str(args.get('target') or '') != 'simulation':
        return 'This account may use the simulation only — ask an admin'
    target = getattr(w, 'control_target', 'simulation')
    if target != 'simulation':
        return ('This account may use the simulation only, and the Studio is on %s'
                % str(target).upper())
    return None


def handle_command(w, action, args, client):
    """Run one web command on the Studio; always returns a JSON-able dict."""
    handler = HANDLERS.get(action)
    if handler is None:
        return Refused('Unknown command: %s' % action, 'unknown_action').reply()
    if action in LOCAL_ONLY and not (client or {}).get('local'):
        return Refused('That is only available in the Studio window itself', 'local_only').reply()
    limits = (client or {}).get('limits')
    if (limits or {}).get('sim_only'):
        problem = _sim_only_problem(w, action, args or {})
        if problem:
            return Refused(problem, 'simulation_only').reply()
    if action in ADMIN_ONLY and limits is not None and not limits.get('admin'):
        return Refused('Only an admin may do that', 'admin_only').reply()
    try:
        reply = handler(w, args or {}, client)
    except Refused as exc:
        return exc.reply()
    return reply if isinstance(reply, dict) else {'ok': True}


# ── Snapshot ─────────────────────────────────────────────────────────────

def _deg(rad):
    rad = _num(rad)
    return None if rad is None else math.degrees(rad)


def _studio(w, ctx):
    from fic_studio.ros_node import _host_tag
    cfg = ctx['cfg']
    return {
        'app': getattr(w, 'APP_NAME', 'FIC Control Studio'),
        'version': getattr(w, 'APP_VERSION', ''),
        'host': _host_tag(),
        'station': w.node.station_id,
        'robot': ctx['robot'],
        'robot_display': cfg.get('display_name') or ctx['robot'],
        'rid': ctx['rid'],
    }


def _robots_state(w, ctx):
    """The toolbar's robot list, for the web UI's robot selector."""
    combo = w.robot_dropdown
    if not hasattr(combo, 'count'):
        return {'items': [], 'current': ctx['robot']}
    items = []
    for i in range(combo.count()):
        name = combo.itemText(i)
        items.append({'name': name,
                      'label': _cfg(w, name).get('display_name') or name})
    return {'items': items, 'current': ctx['robot']}


#: The processes a session on each target is made of.
SESSION_PARTS = {'simulation': ('simulation', 'fic'), 'hardware': ('driver', 'controller'),
                 'vr': ('vr',)}


def _session_summary(w):
    """idle / partial / running, and a line saying which parts are up."""
    target = getattr(w, 'control_target', 'simulation')
    parts = SESSION_PARTS.get(target, ())
    up = [p for p in parts if bool(getattr(w, PROCESSES[p][0], False))]
    if parts and len(up) == len(parts):
        state = 'running'
    elif up or w._tb_session_running():
        state = 'partial'
    else:
        state = 'idle'
    return {'state': state, 'parts_up': len(up), 'parts_total': len(parts)}


def _session_state(w, ctx):
    return {
        'target': getattr(w, 'control_target', 'simulation'),
        'targets': list(TARGETS),
        'running': bool(w._tb_session_running()),
        'processes': {
            'simulation': bool(getattr(w, 'simulation_running', False)),
            'fic': bool(getattr(w, 'fic_running', False)),
            'driver': bool(getattr(w, 'driver_running', False)),
            'controller': bool(getattr(w, 'controller_running', False)),
            'vr': bool(getattr(w, 'vr_relay_running', False)),
        },
        'driver_configured': bool(w._resolve_driver_command(ctx['robot'])),
        'process_list': _process_list(w),
        **_session_summary(w),
    }


def _teleop_state(w, ctx):
    """Pairings this station is in, and anything waiting for its answer.

    ``pending`` is the point of it: a request addressed to this station that
    the Studio would otherwise only raise as a dialog on its own desktop.
    """
    teleop = getattr(w, '_teleop', None)
    if teleop is None:
        return {'available': False, 'station': None, 'pending': [], 'sessions': []}
    me = w.node.station_id
    rows = []
    for s in teleop.visible():
        row = s.as_dict()
        row['from_host'] = getattr(s, 'from_host', '')
        row['role'] = ('leader' if s.leader == me
                       else 'follower' if s.follower == me else '')
        rows.append(row)
    return {
        'available': True,
        'station': me,
        'pending': [r for r in rows if r['state'] == 'pending' and r['follower'] == me],
        'sessions': rows,
    }


def _kortex_state(w, ctx):
    """The arm's Kortex side: state, faults, camera; available only on the one-switch layout."""
    summary = getattr(w, 'kortex_summary', None)
    return summary() if summary is not None else {'available': False, 'note': 'no Kortex support'}


def _inner_loop_state(w, ctx):
    """Inner loop: the FIC MCU (where, state, rate, faults) or standard."""
    summary = getattr(w, 'inner_loop_summary', None)
    return summary() if summary is not None else {'path': 'standard', 'state': '', 'detail': '',
                                                  'live': False, 'where': '', 'adapters': []}


def _mode_state(w, ctx):
    index = int(getattr(w, '_rc_selected_mode_idx', 0))
    toggle = getattr(w, '_rc_hpmp_toggle', None)
    hpmp = bool(toggle is not None and toggle.isChecked())
    menus = {}
    for kind, (label, indices) in MODE_MENUS.items():
        menus[kind] = {'label': label, 'modes': [
            {'index': i, 'label': MODE_ROW_LABELS[slot], 'name': CONTROL_MODE_DISPLAY[i]}
            for slot, i in enumerate(indices)]}
    return {
        'index': index,
        'name': mode_display_name(index, hpmp),     # names H-PMP when the toggle is on (modes 5-8)
        'kind': getattr(w, '_rc_selected_kind', 'joint'),
        'menus': menus,
    }


def _options_state(w, ctx):
    options = {}
    for name, (attr, kind) in OPTION_TOGGLES.items():
        box = getattr(w, attr, None)
        if box is not None:
            options[name] = {'value': box.isChecked(), 'kind': kind,
                             'label': box.text(), 'tip': box.toolTip()}
    return options


def _desired_wrench_state(w, ctx):
    if not hasattr(w, 'desired_wrench_for'):
        return None
    return w.desired_wrench_for(ctx['rid'])


def _profiles_state(w, ctx):
    combo = getattr(w, '_rc_profile_dropdown', None)
    if combo is None:
        return {'items': [], 'current': 'default'}
    items = [{'name': combo.itemData(i) or combo.itemText(i), 'label': combo.itemText(i)}
             for i in range(combo.count())]
    return {'items': items, 'current': combo.currentData() or combo.currentText()}


def _joints_state(w, ctx):
    rid = ctx['rid']
    rows = w._rc_display_joints(ctx['robot'], ctx['cfg'])
    fields = list(getattr(w, '_rc_joint_fields', []) or [])
    live_fields = (getattr(w, '_rc_selected_kind', 'joint') == 'joint'
                   and len(fields) == len(rows))
    q = w.joint_positions_for(rid)
    dq = w.joint_velocity_for(rid) if hasattr(w, 'joint_velocity_for') else []
    tau = w.joint_torque_for(rid) if hasattr(w, 'joint_torque_for') else []
    sim = w.sim_joint_by_name(rid) if hasattr(w, 'sim_joint_by_name') else {}
    out = []
    for i, row in enumerate(rows):
        ci = row.get('cmd_index')
        actual = velocity = torque = None
        if ci is not None:
            actual = _deg(q[ci]) if ci < len(q) else None
            velocity = _deg(dq[ci]) if ci < len(dq) else None
            torque = _num(tau[ci]) if ci < len(tau) else None
        if actual is None and row['name'] in sim:
            actual = _deg(sim[row['name']][0])
        out.append({
            'label': row['label'], 'lo': _num(row['lo']), 'hi': _num(row['hi']),
            'actuated': bool(row['actuated']),
            'commanded': _num(fields[i].commanded()) if live_fields else None,
            'actual': actual, 'velocity': velocity, 'torque': torque,
        })
    return out


def _pose_state(w, ctx):
    actual = None
    pose = w.ee_pose_for(ctx['rid'])
    if pose is not None:
        p, o = pose.position, pose.orientation
        qx, qy, qz, qw = o.x, o.y, o.z, o.w
        rx, ry, rz = _quat_to_euler_xyz(qx, qy, qz, qw, 'deg')
        if qw < 0.0:                    # the hemisphere the Studio's fields use
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        ax, ay, az, angle = _quat_to_axis_angle(qx, qy, qz, qw, 'deg')
        actual = {'x': p.x, 'y': p.y, 'z': p.z, 'rx': rx, 'ry': ry, 'rz': rz,
                  'ax': ax, 'ay': ay, 'az': az, 'angle': angle}
    commanded = None
    fields = getattr(w, '_rc_pose_fields', {}) or {}
    if getattr(w, '_rc_selected_kind', 'joint') == 'cart' and 'X' in fields:
        commanded = {
            'x': fields['X'].commanded(), 'y': fields['Y'].commanded(),
            'z': fields['Z'].commanded(), 'ax': fields['axis X'].value(),
            'ay': fields['axis Y'].value(), 'az': fields['axis Z'].value(),
            'angle': fields['angle'].value(),
        }
    return {'actual': actual, 'commanded': commanded, 'position_limit': 1.0, 'reach': _reach(w, ctx)}


def _reach(w, ctx):
    """The robot's reach sphere (scene.reach), once per robot; None without a 3D model."""
    robot, frame = ctx['robot'], ctx['cfg'].get('ee_frame_name')
    cached = getattr(w, '_web_reach_cache', None)
    if cached is not None and cached[0] == (robot, frame):
        return cached[1]
    value = None
    provider = getattr(w, '_web_scene_provider', None)
    if provider is not None and frame:
        try:
            from fic_studio.web_remote import scene as scene_mod
            value = scene_mod.reach(provider(None), frame)
        except Exception:
            value = None
    w._web_reach_cache = ((robot, frame), value)
    return value


def _telemetry_state(w, ctx):
    rid = ctx['rid']
    rate, age_ms, health = (w.telemetry_health(rid) if hasattr(w, 'telemetry_health')
                            else (0.0, None, 'offline'))
    wrench = w.wrench_for(rid) if hasattr(w, 'wrench_for') else None
    contact = w.contact_estimate_for(rid) if hasattr(w, 'contact_estimate_for') else None
    return {
        'health': {'state': health, 'rate': rate, 'age_ms': age_ms},
        'wrench': list(wrench) if wrench is not None else None,
        'contact': dict(contact) if contact is not None else None,
    }


def _gripper_state(w, ctx):
    slider = getattr(w, '_rc_gripper_slider', None)
    measured = age = None
    if hasattr(w, 'gripper_state_for'):
        state = w.gripper_state_for(ctx['rid'])
        if state:                      # (width_m, motor current, age_s)
            measured, age = _num(state[0]), _num(state[2])
    return {
        'available': bool(ctx['cfg'].get('has_gripper', True)) and slider is not None
                     and availability(w)['gripper'],
        'reason': availability(w)['gripper_reason'],
        'percent': slider.value() if slider is not None else None,
        'max_width': GRIPPER_MAX_WIDTH_M,
        # what the arm reports (fic_control), so a view can show the real fingers
        'measured': measured,
        'measured_age': age,
    }


def _status_state(w, ctx):
    status = getattr(w, '_cb_status_lbl', None)
    contact = getattr(w, '_cb_contact_lbl', None)
    ext = _sequences_ext(w)
    return {
        'text': status.text() if status is not None else '',
        'contact': contact.text() if contact is not None else '',
        'playing': bool(ext is not None and getattr(ext, 'is_playing', False)),
    }


def _sequences_state(w, ctx):
    ext = _sequences_ext(w)
    if ext is None:
        return {'available': False, 'items': [], 'playing': False}
    out = {'available': True, 'items': list(ext.list_sequences()),
           'playing': bool(getattr(ext, 'is_playing', False))}
    if hasattr(ext, 'trajectory_state'):
        out.update(speed=ext.speed(), trajectory=ext.trajectory_state(),
                   hardware=bool(ext.target_is_hardware()))
    return out


def _quick_state(w, ctx):
    """The actions extensions added to Quick Actions, with their current choices."""
    out = []
    for key, act in sorted((getattr(w, '_quick_actions', None) or {}).items()):
        choices = None
        if act.get('choices') is not None:
            try:
                choices = [str(c) for c in (act['choices']() or [])]
            except Exception:
                choices = []
        out.append({'key': key, 'label': act['label'], 'hint': act.get('hint', ''),
                    'button': act.get('button') or 'Run', 'choices': choices})
    return out


def _log_state(w, ctx):
    viewer = getattr(w, 'log_viewer', None)
    entries = list(getattr(viewer, '_all_logs', []) or [])[-LOG_TAIL:]
    return [{'seq': e.get('seq'), 'time': e.get('timestamp', ''), 'level': e.get('level', ''),
             'source': e.get('source', ''), 'message': str(e.get('message', ''))[:300]}
            for e in entries]


def _fleet_state(w, ctx):
    """The Studios this one works with (its neighbours; a Master: every one), for the Multi-robot page."""
    from fic_studio.fleet.stations import is_master, is_vr, station_label
    directory = getattr(w, '_fleet_dir', None)
    if directory is None:
        return {'available': False, 'studios': [], 'conflicts': []}
    me = w.node.station_id
    studios = []
    modes = getattr(w, '_web_modes', None) or {}
    mine = list(getattr(w.node, '_active_robots', None) or [])
    for card in directory.visible():                 # itself, its neighbours, Masters
        n = card.get('station')
        known = modes.get('robot%d' % n) if n in mine else None
        if known is not None and known < len(CONTROL_MODE_DISPLAY):
            card = dict(card, mode=CONTROL_MODE_DISPLAY[known])  # what this Studio last sent that robot
        viewer = card.get('role') == 'viewer'
        studios.append({
            'station': n, 'label': 'VR viewer' if viewer else station_label(n), 'host': card.get('host', ''),
            'robot': card.get('robot', ''), 'robot_display': card.get('robot_display', ''),
            'target': card.get('target', ''), 'running': bool(card.get('running')),
            'has_joints': bool(card.get('has_joints')), 'mode': card.get('mode', ''),
            'teleop': list(card.get('teleop') or []), 'urls': list(card.get('urls') or []),
            'self': n == me, 'master': bool(is_master(n)), 'vr': bool(is_vr(n) or viewer), 'viewer': viewer,
            'last_cmd': card.get('last_cmd'),
        })
    conflicts = [{'station': n, 'label': station_label(n), 'hosts': list(h)}
                 for n, h in sorted(directory.conflicts().items())]
    return {'available': True, 'station': me, 'studios': studios, 'conflicts': conflicts}


def _locomotion_state(w, ctx):
    from fic_studio.mixins.locomotion_mixin import GAIT_COMMANDS, GAIT_TUNING, VELOCITY_AXES
    legged = ctx['cfg'].get('robot_category') == 'legged'
    vel = tuple(getattr(w, '_loco_velocity', (0.0, 0.0, 0.0)))
    return {
        'available': legged,
        'running': bool(getattr(w, 'locomotion_running', False)),
        'gaits': list(GAIT_COMMANDS),
        'velocity': {k: vel[i] for i, (k, _l, _lim) in enumerate(VELOCITY_AXES)},
        'axes': [{'key': k, 'label': l, 'limit': lim} for k, l, lim in VELOCITY_AXES],
        'tuning': [{'key': k, 'label': l, 'min': lo, 'max': hi, 'default': d, 'step': st}
                   for k, l, lo, hi, d, st in GAIT_TUNING],
        'hold_s': LOCO_HOLD_S,
    }


def _safety_state(w, ctx):
    return _safety.state(w, ctx)


def _theme_state(w, ctx):
    return {'name': theme.current_name(), 'tokens': dict(theme.current())}


def _appearance_state(w, ctx):
    from fic_studio.web_remote import settings_page
    return settings_page.appearance_state(w)


def _estop_state(w, ctx):
    fn = getattr(w, 'estop_latch_state', None)
    return fn() if fn is not None else {'latched': False}


SECTIONS = (
    ('studio', _studio),
    ('robots', _robots_state),
    ('session', _session_state),
    ('teleop', _teleop_state),
    ('inner_loop', _inner_loop_state),
    ('kortex', _kortex_state),
    ('mode', _mode_state),
    ('options', _options_state),
    ('desired_wrench', _desired_wrench_state),
    ('profiles', _profiles_state),
    ('availability', lambda w, ctx: availability(w)),
    ('joints', _joints_state),
    ('pose', _pose_state),
    ('telemetry', _telemetry_state),
    ('gripper', _gripper_state),
    ('status', _status_state),
    ('sequences', _sequences_state),
    ('quick_actions', _quick_state),
    ('log', _log_state),
    ('safety', _safety_state),
    ('fleet', _fleet_state),
    ('estop', _estop_state),
    ('paused', lambda w, ctx: sorted(getattr(w, '_paused_rids', None) or [])),
    ('notices', lambda w, ctx: notices.state(w)),
    ('locomotion', _locomotion_state),
    *_chrome.SECTIONS,
    ('theme', _theme_state),
    ('appearance', _appearance_state),
)


def build_snapshot(w):
    """Everything the web page shows, as plain JSON-able data.

    Each section is built independently: one that fails comes back as None
    with its error listed, rather than blanking the whole page.
    """
    robot = _robot(w)
    ctx = {'robot': robot, 'cfg': _cfg(w, robot), 'rid': _rid(w)}
    state = {'t': time.time()}
    for key, build in SECTIONS:
        try:
            state[key] = build(w, ctx)
        except Exception as exc:
            state[key] = None
            state.setdefault('errors', {})[key] = '%s: %s' % (type(exc).__name__, exc)
    return state
