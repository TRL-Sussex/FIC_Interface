"""The Robot Configuration page, for the web UI.

Reads and writes the same ``robot_configs`` dict the classic page edits and
persists it the same way (the Studio's robots YAML and the controllers' copy,
through ``_rc_persist``). The page sends the parts of a config it changed; this
module validates them, merges them over the stored config (so keys the form
has no field for survive, as in the classic Save) and saves. A save changes
joint limits and safety zones, so it needs an explicit confirm.

Commands: ``robots_list``, ``robots_get``, ``robots_save``, ``robots_add``,
``robots_duplicate``, ``robots_delete``, ``robots_import``, ``robots_yaml``.
"""
import base64
import binascii
import copy
import math
import os
import shutil
import tempfile
import threading
import time
import uuid

from fic_studio.web_remote.common import Refused, _num, _who, need_confirm

ID_HELP = 'A robot ID starts with a letter and uses only letters, digits and _.'


def _ws_src():
    from fic_studio.config import WS_PATH
    return os.path.join(WS_PATH, 'src')


def _configs(w):
    return w.robot_configs


def _check(cfg):
    from fic_studio.robot_import import check as rc_check
    messages = rc_check.check_robot(cfg, _ws_src()) if cfg else []
    return {'messages': [{'level': lv, 'text': t} for lv, t in messages], 'sim_ready': bool(cfg) and rc_check.sim_ready(messages)}


def _model_dir(cfg):
    from fic_studio.robot_import import check as rc_check
    rel = (cfg or {}).get('mujoco_xml_path') or ''
    if not rel:
        return ''
    resolved = rc_check.resolve(_ws_src(), rel)
    return os.path.dirname(resolved) if resolved else ''


def _row(rid, cfg):
    status = _check(cfg)
    return {'id': rid, 'name': cfg.get('display_name') or rid, 'joints': cfg.get('num_joints'),
            'supported': bool(cfg.get('is_supported')), 'sim_ready': status['sim_ready'],
            'category': cfg.get('robot_category') or 'arm'}


def _persist(w):
    persist = getattr(w, '_rc_persist', None)
    if persist is not None:
        return persist() is not False
    return True


def list_robots(w, args, client):
    cur = w.robot_dropdown.currentText()
    return {'ok': True, 'items': [_row(r, c) for r, c in _configs(w).items()], 'current': cur,
            'sync': {'stale': bool(w._rc_check_sync_state()) if hasattr(w, '_rc_check_sync_state') else False}}


def _rid(args, w, must_exist=True):
    rid = str(args.get('id') or '')
    if must_exist and rid not in _configs(w):
        raise Refused('No such robot', 'bad_args')
    return rid


def get(w, args, client):
    rid = _rid(args, w)
    cfg = _configs(w)[rid]
    model_dir = _model_dir(cfg)
    from fic_studio.robot_import import check as rc_check
    xml = rc_check.resolve(_ws_src(), cfg.get('mujoco_xml_path') or '')
    return {'ok': True, 'id': rid, 'config': copy.deepcopy(cfg), 'status': _check(cfg),
            'has_report': bool(model_dir) and os.path.isfile(os.path.join(model_dir, 'import_report.md')),
            'model_dir': model_dir, 'viewer': bool(xml) and os.path.isfile(xml)}


# ── validation ───────────────────────────────────────────────────────────

def _num_list(value, label, n=None, lo=-1e6, hi=1e6):
    if not isinstance(value, list) or (n is not None and len(value) != n):
        raise Refused('%s needs %s numbers' % (label, n if n is not None else 'a list of'), 'bad_args')
    out = []
    for v in value:
        x = _num(v)
        if x is None or isinstance(v, bool) or not lo <= x <= hi:
            raise Refused('%s: "%s" is not a usable number' % (label, v), 'bad_args')
        out.append(x)
    return out


def _text(value, label, maxlen=300):
    if not isinstance(value, str) or len(value) > maxlen or '\n' in value:
        raise Refused('%s must be one line of text' % label, 'bad_args')
    return value.strip()


def clean(patch, current):
    """Check the edited parts of a config; return them typed, ready to merge."""
    out = {}
    n = int(patch.get('num_joints', current.get('num_joints', 7)))
    if 'num_joints' in patch:
        v = _num(patch['num_joints'])
        if v is None or v != int(v) or not 1 <= int(v) <= 40:
            raise Refused('Joints must be a whole number from 1 to 40', 'bad_args')
        n = out['num_joints'] = int(v)
    for key, label in (('display_name', 'Display name'), ('urdf_path', 'URDF path'), ('mujoco_xml_path', 'MuJoCo model path'),
                       ('ee_frame_name', 'End-effector frame'), ('elbow_frame_name', 'Elbow frame'),
                       ('kinematics_solver', 'Kinematics solver'), ('driver_command', 'Driver command'),
                       ('build_directory', 'Build directory'), ('build_command', 'Build command')):
        if key in patch:
            out[key] = _text(patch[key], label)
    if 'display_name' in out and not out['display_name']:
        raise Refused('Give the robot a name', 'bad_args')
    for key in ('is_supported', 'gravity_compensated', 'wrench_gravity_compensated', 'wrench_from_api', 'has_gripper'):
        if key in patch:
            if not isinstance(patch[key], bool):
                raise Refused('%s must be on or off' % key, 'bad_args')
            out[key] = patch[key]
    if 'torque_sign' in patch:
        if patch['torque_sign'] not in (1, -1):
            raise Refused('Torque sign is +1 or -1', 'bad_args')
        out['torque_sign'] = patch['torque_sign']
    if 'joint_names' in patch:
        names = patch['joint_names']
        if not isinstance(names, list) or len(names) != n or any(not isinstance(x, str) or not x.strip() for x in names):
            raise Refused('Give each of the %d joints a name' % n, 'bad_args')
        out['joint_names'] = [x.strip() for x in names]
    if 'q_home_deg' in patch:
        deg = _num_list(patch['q_home_deg'], 'Home pose', n, -720, 720)
        out['q_home_deg'] = deg
        out['q_home_rad'] = [math.radians(d) for d in deg]
    if 'joint_limits_deg' in patch:
        lim = patch['joint_limits_deg']
        lo = _num_list(lim.get('lower'), 'Lower limits', n, -720, 720)
        hi = _num_list(lim.get('upper'), 'Upper limits', n, -720, 720)
        if any(a >= b for a, b in zip(lo, hi)):
            raise Refused('Each joint\'s lower limit must be below its upper limit', 'bad_args')
        out['joint_limits_deg'] = {'lower': lo, 'upper': hi}
    if 'joint_torque_limits' in patch:
        out['joint_torque_limits'] = {'max': _num_list(patch['joint_torque_limits'].get('max'), 'Torque limits', n, 0, 1e5)}
    if 'default_ee_target' in patch:
        t = patch['default_ee_target']
        out['default_ee_target'] = {'position': _num_list(t.get('position'), 'Target position', 3, -10, 10),
                                    'orientation': _num_list(t.get('orientation'), 'Target orientation', 4, -1.0001, 1.0001)}
    if 'gripper_range' in patch:
        out['gripper_range'] = _num_list(patch['gripper_range'], 'Gripper range', 2, -1, 1)
    if 'admittance_deadbands' in patch:
        d = patch['admittance_deadbands']
        out['admittance_deadbands'] = {k: _num_list([d.get(k)], 'Deadband ' + k, 1, 0, 1e4)[0]
                                       for k in ('joint_tau', 'cart_force', 'cart_torque') if k in d}
    if 'safety_zones' in patch:
        out['safety_zones'] = _zones(patch['safety_zones'], n)
    return out


def _zones(z, n):
    out = {}
    if 'cartesian' in z:
        out['cartesian'] = {}
        for axis in ('x', 'y', 'z'):
            a = z['cartesian'].get(axis)
            if a is None:
                continue
            lo, hi = _num(a.get('min')), _num(a.get('max'))
            if lo is None or hi is None or lo >= hi or not isinstance(a.get('enabled'), bool):
                raise Refused('Workspace %s: minimum must be below maximum' % axis, 'bad_args')
            out['cartesian'][axis] = {'min': lo, 'max': hi, 'enabled': a['enabled']}
    if 'joint_safety_deg' in z:
        j = z['joint_safety_deg']
        lo, hi = _num_list(j.get('lower'), 'Safety lower', None, -720, 720), _num_list(j.get('upper'), 'Safety upper', None, -720, 720)
        if len(lo) != len(hi) or any(a >= b for a, b in zip(lo, hi)):
            raise Refused('Joint safety: each lower limit must be below its upper limit', 'bad_args')
        out['joint_safety_deg'] = {'lower': lo, 'upper': hi, 'enabled': bool(j.get('enabled'))}
    if 'velocity' in z:
        v = z['velocity']
        mj, mc = _num(v.get('max_joint_vel')), _num(v.get('max_cart_vel'))
        if mj is None or mc is None or mj < 0 or mc < 0:
            raise Refused('Speed limits must be zero or more', 'bad_args')
        out['velocity'] = {'max_joint_vel': mj, 'max_cart_vel': mc, 'enabled': bool(v.get('enabled'))}
    return out


def save(w, args, client):
    from fic_studio.mixins.robot_config_mixin import merge_robot_config
    rid = _rid(args, w)
    need_confirm(args, 'Saving a robot changes its limits and safety zones and needs confirmation')
    patch = args.get('config')
    if not isinstance(patch, dict) or not patch:
        raise Refused('Nothing to save', 'bad_args')
    current = _configs(w)[rid]
    edits = clean(patch, current)
    merged = merge_robot_config(current, edits)
    n = merged.get('num_joints', 7)
    for key in ('joint_names', 'q_home_deg'):
        if key in merged and len(merged[key]) != n:
            raise Refused('%s must have %d entries (change the joint count and the lists together)' % (key, n), 'bad_args')
    _configs(w)[rid] = merged
    if isinstance(getattr(w, 'robots', None), dict):
        w.robots[rid] = merged.get('num_joints', 7)
    ok = _persist(w)
    if hasattr(w, '_rc_publish_hot_config'):
        w._rc_publish_hot_config(rid)
    w.log_message('INFO', 'Robot %s saved from the web UI (%s)' % (rid, _who(client)), 'Config')
    return {'ok': ok, 'id': rid, 'config': copy.deepcopy(merged), 'status': _check(merged)}


def add(w, args, client):
    from fic_studio.mixins.robot_config_mixin import DEFAULT_ROBOT_TEMPLATE
    from fic_studio.robot_import.importer import valid_robot_id
    rid = str(args.get('id') or '').strip()
    if not valid_robot_id(rid):
        raise Refused(ID_HELP, 'bad_args')
    if rid in _configs(w):
        raise Refused("Robot '%s' already exists" % rid, 'bad_args')
    cfg = copy.deepcopy(DEFAULT_ROBOT_TEMPLATE)
    cfg['display_name'] = rid
    cfg['urdf_path'] = 'menagerie/model/%s/URDF/%s.urdf' % (rid, rid)
    cfg['mujoco_xml_path'] = 'menagerie/model/%s/xml/scene.xml' % rid
    _configs(w)[rid] = cfg
    _persist(w)
    w.log_message('INFO', 'Added robot %s from the web UI (%s)' % (rid, _who(client)), 'Config')
    return {'ok': True, 'id': rid}


def duplicate(w, args, client):
    rid = _rid(args, w)
    new = rid + '_copy'
    i = 1
    while new in _configs(w):
        new = '%s_copy%d' % (rid, i)
        i += 1
    cfg = copy.deepcopy(_configs(w)[rid])
    cfg['display_name'] = cfg.get('display_name', rid) + ' (Copy)'
    _configs(w)[new] = cfg
    _persist(w)
    w.log_message('INFO', 'Duplicated robot %s as %s (web UI)' % (rid, new), 'Config')
    return {'ok': True, 'id': new}


def delete(w, args, client):
    rid = _rid(args, w)
    need_confirm(args, 'Deleting a robot needs confirmation')
    if len(_configs(w)) <= 1:
        raise Refused('The last robot cannot be deleted', 'refused')
    if rid == w.robot_dropdown.currentText():
        raise Refused('Select another robot in the toolbar before deleting this one', 'refused')
    del _configs(w)[rid]
    _persist(w)
    w.log_message('WARN', 'Deleted robot %s from the web UI (%s). Its model folder was left in place.' % (rid, _who(client)),
                  'Config')
    return {'ok': True}


def yaml_view(w, args, client):
    """The controllers' YAML for one robot (View the controllers' YAML), read only."""
    import yaml
    rid = _rid(args, w)
    path = os.path.join(os.path.dirname(w._rc_fic_config_yaml_path()) if hasattr(w, '_rc_fic_config_yaml_path') else '',
                        'robots_config.yaml')
    text = ''
    if os.path.isfile(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        text = yaml.safe_dump({rid: (data.get('robots') or {}).get(rid, {})}, sort_keys=False)
    return {'ok': True, 'path': path, 'text': text}


def report(w, args, client):
    rid = _rid(args, w)
    path = os.path.join(_model_dir(_configs(w)[rid]), 'import_report.md')
    if not os.path.isfile(path):
        raise Refused('This robot has no import report', 'unavailable')
    with open(path, encoding='utf-8') as f:
        return {'ok': True, 'text': f.read(5_000_000)}


# ── Import from a URDF ───────────────────────────────────────────────────

def _plan_view(plan):
    return {
        'id': plan.robot_id, 'name': plan.display_name, 'urdf': plan.urdf_path, 'ee_link': plan.ee_link,
        'elbow_link': plan.elbow_link,
        'joints': [{'name': j.name, 'lower': j.lower_deg, 'upper': j.upper_deg, 'effort': j.effort,
                    'home': j.home_deg} for j in plan.joints],
        'found': [{'what': a, 'value': b, 'source': c} for a, b, c in _found(plan)],
        'messages': [{'level': lv, 'text': t} for lv, t in plan.messages],
        'errors': list(plan.errors),
        'meshes': list(plan.mesh_counts),
        'closed_loop': bool(plan.closed_loop),
        'candidates': list(plan.urdf_candidates),
    }


def _found(plan):
    from fic_studio.robot_import import importer
    return importer._what_was_found(plan)


# ── Upload: a dropped file or folder, staged on the Studio machine ───────

MAX_UPLOAD_BYTES = 200 * 1024 * 1024
MAX_UPLOAD_FILES = 20000
MAX_CHUNK_BYTES = 1024 * 1024
STALE_UPLOAD_S = 3600


def safe_relpath(name):
    """A dropped file's relative path as a tuple of safe parts, or a Refused.

    Rejects absolute paths, '..', empty or NUL-bearing parts and drive letters,
    so nothing can be written outside the staging folder.
    """
    if not isinstance(name, str) or not name or len(name) > 1024 or '\0' in name:
        raise Refused('That file name is not allowed', 'bad_args')
    name = name.replace('\\', '/')
    if name.startswith('/') or (len(name) > 1 and name[1] == ':'):
        raise Refused('Absolute paths are not allowed in an upload: %s' % name[:80], 'bad_args')
    parts = [p for p in name.split('/') if p not in ('', '.')]
    if not parts or any(p == '..' or len(p) > 255 for p in parts):
        raise Refused('Paths with ".." are not allowed in an upload: %s' % name[:80], 'bad_args')
    return tuple(parts)


def _uploads(w):
    if not isinstance(getattr(w, '_web_uploads', None), dict):
        w._web_uploads = {}
    return w._web_uploads


def discard_upload(w, uid):
    up = _uploads(w).pop(uid, None)
    if up:
        shutil.rmtree(up['dir'], ignore_errors=True)


def _sweep_uploads(w):
    for uid, up in list(_uploads(w).items()):
        if time.time() - up['t'] > STALE_UPLOAD_S:
            discard_upload(w, uid)


def _get_upload(w, args):
    uid = str(args.get('upload') or '')
    up = _uploads(w).get(uid)
    if up is None:
        raise Refused('That upload is gone; drop the files again', 'bad_args')
    up['t'] = time.time()
    return uid, up


def upload_robot(w, args, client):
    """Stage a dropped URDF / xacro / package folder / zip: ``begin``, ``chunk`` (many), then
    ``analyse`` via ``robots_import`` with ``upload``; ``cancel`` removes the staging folder."""
    op = args.get('op')
    if op == 'begin':
        _sweep_uploads(w)
        uid = uuid.uuid4().hex[:16]
        _uploads(w)[uid] = {'dir': tempfile.mkdtemp(prefix='fic_robot_upload_'), 'bytes': 0, 'files': {}, 't': time.time()}
        return {'ok': True, 'upload': uid, 'max_bytes': MAX_UPLOAD_BYTES}
    if op == 'cancel':
        discard_upload(w, str(args.get('upload') or ''))
        return {'ok': True}
    if op != 'chunk':
        raise Refused('No such upload command', 'bad_args')
    uid, up = _get_upload(w, args)
    parts = safe_relpath(args.get('path'))
    try:
        data = base64.b64decode(str(args.get('data') or ''), validate=True)
    except (binascii.Error, ValueError):
        raise Refused('Bad upload data', 'bad_args')
    if len(data) > MAX_CHUNK_BYTES:
        raise Refused('Upload chunk too large', 'bad_args')
    rel = '/'.join(parts)
    offset = args.get('offset')
    have = up['files'].get(rel)
    if not isinstance(offset, int) or isinstance(offset, bool) or offset != (have if have is not None else 0):
        raise Refused('Upload chunks must arrive in order', 'bad_args')
    if have is None and len(up['files']) >= MAX_UPLOAD_FILES:
        discard_upload(w, uid)
        raise Refused('Too many files (limit %d)' % MAX_UPLOAD_FILES, 'too_large')
    if up['bytes'] + len(data) > MAX_UPLOAD_BYTES:
        discard_upload(w, uid)
        raise Refused('The upload is over the %d MB limit' % (MAX_UPLOAD_BYTES // 1048576), 'too_large')
    root = os.path.realpath(up['dir'])
    target = os.path.realpath(os.path.join(root, *parts))
    if os.path.commonpath([root, target]) != root or target == root:
        raise Refused('Paths outside the upload are not allowed', 'bad_args')
    try:
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, 'wb' if have is None else 'ab') as f:
            f.write(data)
    except OSError as exc:
        raise Refused('Cannot stage the file: %s' % exc, 'refused')
    up['files'][rel] = (have or 0) + len(data)
    up['bytes'] += len(data)
    return {'ok': True, 'bytes': up['bytes'], 'files': len(up['files'])}


def staged_source(stage_dir):
    """What to hand the import pipeline for a staged tree: (path, mjcf_path or None).

    One top-level entry (a file, a package folder or a zip) is passed as it is. Loose
    files dropped together go in as the staging folder, or, like the classic drop
    zone, the first non-MuJoCo file with the MuJoCo model beside it.
    """
    from fic_studio.robot_import import mujoco_model
    entries = sorted(os.listdir(stage_dir))
    if not entries:
        raise Refused('Nothing was uploaded', 'bad_args')
    if len(entries) == 1:
        return os.path.join(stage_dir, entries[0]), None
    paths = [os.path.join(stage_dir, e) for e in entries]
    files = [p for p in paths if os.path.isfile(p)]
    models = [p for p in files if mujoco_model.is_mjcf(p)]
    rest = [p for p in files if p not in models]
    if models and len(rest) == 1 and len(files) == len(paths):
        return rest[0], models[0]
    return stage_dir, None


def import_robot(w, args, client):
    """Analyse a URDF (or its folder or zip) on the Studio machine, then create the robot from it."""
    from fic_studio.robot_import import importer
    op = args.get('op')
    state = getattr(w, '_web_import', None)
    if op == 'status':
        return _import_status(w)
    path = str(args.get('path') or '').strip()
    mjcf = None
    upload_id = ''
    if op in ('analyse', 'create') and args.get('upload'):
        upload_id, up = _get_upload(w, args)
        path, mjcf = staged_source(up['dir'])
    elif op in ('analyse', 'create'):
        if not path or len(path) > 1000 or not os.path.exists(os.path.expanduser(path)):
            raise Refused('That path does not exist on the Studio machine', 'bad_args')
        path = os.path.expanduser(path)
    if op == 'analyse':
        try:
            plan = importer.analyse(path, existing_ids=list(_configs(w)), mjcf_path=mjcf)
        except importer.ImportError_ as exc:
            raise Refused(str(exc), 'refused')
        try:
            return {'ok': True, 'plan': _plan_view(plan)}
        finally:
            plan.cleanup()
    if op == 'create':
        need_confirm(args, 'Creating a robot from a URDF writes files and needs confirmation')
        if state and state.get('running'):
            raise Refused('An import is already running', 'busy')
        try:
            plan = importer.analyse(path, existing_ids=list(_configs(w)), mjcf_path=mjcf)
        except importer.ImportError_ as exc:
            raise Refused(str(exc), 'refused')
        rid = str(args.get('robot_id') or plan.robot_id).strip()
        if not importer.valid_robot_id(rid) or rid in _configs(w):
            plan.cleanup()
            raise Refused(ID_HELP if not importer.valid_robot_id(rid) else "Robot '%s' already exists" % rid, 'bad_args')
        plan.robot_id = rid
        plan.display_name = _text(str(args.get('name') or plan.display_name), 'Name')
        if plan.errors:
            plan.cleanup()
            raise Refused(plan.errors[0], 'refused')
        model_root = os.path.join(_ws_src(), 'menagerie', 'model')
        state = {'running': True, 'progress': 'Starting', 'result': None, 'error': None, 'registered': False}
        w._web_import = state

        def work():
            try:
                state['result'] = importer.create_robot(plan, model_root,
                                                        progress=lambda text: state.update(progress=text))
            except Exception as exc:
                state['error'] = str(exc)
            finally:
                plan.cleanup()
                if upload_id:
                    discard_upload(w, upload_id)
                state['running'] = False
        threading.Thread(target=work, daemon=True).start()
        w.log_message('INFO', 'Importing %s from %s (web UI, %s)' % (rid, os.path.basename(path), _who(client)), 'Config')
        return {'ok': True, **_import_status(w)}
    raise Refused('No such import command', 'bad_args')


def _import_status(w):
    """The page polls this; the finished robot is registered here, on the Studio's thread."""
    state = getattr(w, '_web_import', None)
    if not state:
        return {'ok': True, 'running': False}
    out = {'ok': True, 'running': state['running'], 'progress': state['progress'], 'error': state['error']}
    if state['result'] is not None and not state['registered']:
        res = state['result']
        _configs(w)[res.robot_id] = res.config
        _persist(w)
        state['registered'] = True
        w.log_message('INFO', 'Added robot %s from its URDF (%d joints, model in %s)'
                      % (res.robot_id, res.config['num_joints'], res.model_dir), 'Config')
    if state['result'] is not None:
        res = state['result']
        out['done'] = {'id': res.robot_id, 'model_dir': res.model_dir,
                       'messages': [{'level': lv, 'text': t} for lv, t in res.messages]}
    return out


def target_from_home(w, args, client):
    """The end effector's pose at the home pose, to fill the default Cartesian target (nothing is saved)."""
    from fic_studio.robot_import import check as rc_check
    from fic_studio.robot_import import fk
    rid = _rid(args, w)
    cfg = _configs(w)[rid]
    names = args.get('joint_names') or cfg.get('joint_names') or []
    deg = args.get('q_home_deg') or cfg.get('q_home_deg') or []
    q = [math.radians(x) for x in _num_list(deg, 'Home pose', len(names), -720, 720)]
    frame = str(args.get('ee_frame_name') or cfg.get('ee_frame_name') or '')
    urdf = rc_check.resolve(_ws_src(), str(args.get('urdf_path') or cfg.get('urdf_path') or ''))
    xml = rc_check.resolve(_ws_src(), str(args.get('mujoco_xml_path') or cfg.get('mujoco_xml_path') or ''))
    try:
        pos, quat = fk.ee_pose(frame, names, q, urdf=urdf if urdf and os.path.isfile(urdf) else None,
                               xml=xml if xml and os.path.isfile(xml) else None)
    except fk.FkError as exc:
        raise Refused('Cannot compute the pose: %s' % exc, 'refused')
    return {'ok': True, 'position': [float(v) for v in pos], 'orientation': [float(v) for v in quat]}


def import_yaml(w, args, client):
    """Import robots from YAML text. Existing robots are kept unless ``overwrite`` lists them."""
    import yaml
    need_confirm(args, 'Importing robots needs confirmation')
    text = args.get('text')
    if not isinstance(text, str) or not text.strip() or len(text) > 4_000_000:
        raise Refused('Send the YAML file as text (up to 4 MB)', 'bad_args')
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise Refused('That is not valid YAML: %s' % exc, 'bad_args')
    imported = data.get('robots', data) if isinstance(data, dict) else None
    if not isinstance(imported, dict) or not all(isinstance(v, dict) for v in imported.values()):
        raise Refused('Expected a mapping of robot IDs to settings', 'bad_args')
    overwrite = set(args.get('overwrite') or [])
    added, skipped = [], []
    for rid, cfg in imported.items():
        if rid in _configs(w) and rid not in overwrite:
            skipped.append(rid)
            continue
        _configs(w)[rid] = cfg
        added.append(rid)
    if added:
        _persist(w)
    w.log_message('INFO', 'Imported %d robot(s) from YAML (web UI, %s)' % (len(added), _who(client)), 'Config')
    return {'ok': True, 'added': added, 'skipped': skipped}


def compile_driver(w, args, client):
    """Build the robot's driver (Hardware tab); the output is read back through ``build`` status."""
    from fic_studio.config import ROBOT_LIB
    from fic_studio.web_remote import chrome
    rid = _rid(args, w)
    need_confirm(args, 'A driver build needs confirmation')
    cfg = _configs(w)[rid]
    build_dir, build_cmd = cfg.get('build_directory', ''), cfg.get('build_command', '')
    if not build_dir or not build_cmd:
        raise Refused('No build configuration for %s: set the build directory and command first' % rid, 'unavailable')
    build_dir = os.path.expanduser(build_dir.replace('{robot_lib}', ROBOT_LIB).replace('{robot_name}', rid))
    build_cmd = build_cmd.replace('{robot_lib}', ROBOT_LIB).replace('{robot_name}', rid)
    return {'ok': True, **chrome.start_build(w, 'driver ' + rid, build_dir, build_cmd, client)}


def open_folder(w, args, client):
    """Open the robot's model folder in the Studio machine's file manager."""
    from fic_studio.platform_utils import open_path
    folder = _model_dir(_configs(w)[_rid(args, w)])
    if not folder or not os.path.isdir(folder):
        raise Refused('This robot has no model folder yet', 'unavailable')
    open_path(folder)
    return {'ok': True, 'folder': folder}


def open_viewer(w, args, client):
    """Open the robot's MuJoCo scene in MuJoCo's own viewer, on the Studio machine."""
    import shutil
    import subprocess
    import sys
    from fic_studio.robot_import import check as rc_check
    rid = _rid(args, w)
    xml = rc_check.resolve(_ws_src(), _configs(w)[rid].get('mujoco_xml_path') or '')
    if not xml or not os.path.isfile(xml):
        raise Refused('This robot has no MuJoCo model yet', 'unavailable')
    python = sys.executable
    if sys.platform == 'darwin' and shutil.which('mjpython'):
        python = shutil.which('mjpython')
    try:
        subprocess.Popen([python, '-m', 'mujoco.viewer', '--mjcf=%s' % xml], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
    except OSError as exc:
        raise Refused('Cannot open the viewer: %s' % exc, 'refused')
    w.log_message('INFO', 'Opened %s in the MuJoCo viewer (web UI, %s)' % (xml, _who(client)), 'Config')
    return {'ok': True}


HANDLERS = {
    'robots_list': list_robots,
    'robots_get': get,
    'robots_save': save,
    'robots_add': add,
    'robots_duplicate': duplicate,
    'robots_delete': delete,
    'robots_import': import_robot,
    'robots_upload': upload_robot,
    'robots_yaml': yaml_view,
    'robots_report': report,
    'robots_target_from_home': target_from_home,
    'robots_import_yaml': import_yaml,
    'robots_compile_driver': compile_driver,
    'robots_open_folder': open_folder,
    'robots_viewer': open_viewer,
}
