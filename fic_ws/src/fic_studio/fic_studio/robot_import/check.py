"""Check a robot's config against its files: can it run in simulation.

Reads the URDF and MuJoCo XML as plain XML (following ``<include>``) rather
than compiling them, so it is quick enough to run every time a robot is
selected, even for large models.
"""
import os
import xml.etree.ElementTree as ET

OK, WARN, ERROR = 'ok', 'warn', 'error'

_cache = {}


def _parse(path):
    """Return the ElementTree root for ``path``, cached until the file changes."""
    try:
        stamp = os.path.getmtime(path)
    except OSError:
        return None
    hit = _cache.get(path)
    if hit and hit[0] == stamp:
        return hit[1]
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError):
        root = None
    _cache[path] = (stamp, root)
    return root


def mjcf_names(path, _main_dir=None, _seen=None):
    """{'joint': set, 'actuator': [names in order], 'sensor': set} for an MJCF file."""
    names = {'joint': set(), 'actuator': [], 'sensor': set(), 'body': set()}
    seen = _seen if _seen is not None else set()
    if path in seen:
        return names
    seen.add(path)
    root = _parse(path)
    if root is None:
        return None
    main_dir = _main_dir or os.path.dirname(path)
    for el in root.iter():
        if el.tag == 'include' and el.get('file'):
            for base in (os.path.dirname(path), main_dir):
                inc = os.path.join(base, el.get('file'))
                if os.path.isfile(inc):
                    sub = mjcf_names(inc, main_dir, seen)
                    if sub:
                        for k in ('joint', 'sensor', 'body'):
                            names[k] |= sub[k]
                        names['actuator'] += sub['actuator']
                    break
    in_default = set()
    for d in root.iter('default'):
        in_default.update(id(e) for e in d.iter())
    for el in root.iter():
        if id(el) in in_default or not el.get('name'):
            continue
        if el.tag in ('joint', 'freejoint'):
            names['joint'].add(el.get('name'))
        elif el.tag == 'body':
            names['body'].add(el.get('name'))
    for group, key in (('actuator', 'actuator'), ('sensor', 'sensor')):
        for block in root.findall(group):
            for el in block:
                if el.get('name'):
                    if key == 'actuator':
                        names[key].append(el.get('name'))
                    else:
                        names[key].add(el.get('name'))
    return names


def urdf_names(path):
    """(link names, joint names) of a URDF, or None when it cannot be read."""
    root = _parse(path)
    if root is None or root.tag != 'robot':
        return None
    return ({link.get('name') for link in root.findall('link')},
            {j.get('name') for j in root.findall('joint')})


def resolve(ws_src, rel):
    if not rel:
        return ''
    return rel if os.path.isabs(rel) else os.path.join(ws_src, rel)


def check_robot(cfg, ws_src):
    """[(level, text)] describing whether ``cfg`` can run, most important first."""
    out = []
    listed = list(cfg.get('joint_names') or [])
    n = int(cfg.get('num_joints') or len(listed))
    joints = [j for j in listed if j]
    if len(listed) != n:
        out.append((ERROR, 'The config lists %d joint names for %d joints.' % (len(listed), n)))
    if len(joints) != len(listed):
        out.append((ERROR, '%d joint name%s blank: fill them in on the Joints tab.'
                    % (len(listed) - len(joints), ' is' if len(listed) - len(joints) == 1
                       else 's are')))

    urdf_rel = cfg.get('urdf_path', '')
    urdf = resolve(ws_src, urdf_rel)
    if not urdf_rel:
        out.append((WARN, 'No URDF set: the controllers need one for Cartesian control.'))
    elif not os.path.isfile(urdf):
        out.append((ERROR, 'URDF not found: %s' % urdf_rel))
    else:
        names = urdf_names(urdf)
        if names is None:
            out.append((ERROR, 'The URDF cannot be read: %s' % urdf_rel))
        else:
            links, ujoints = names
            missing = [j for j in joints if j not in ujoints]
            if missing:
                out.append((ERROR, 'Joint%s not in the URDF: %s' % (
                    '' if len(missing) == 1 else 's', ', '.join(missing))))
            frames = links | ujoints
            for key, label in (('ee_frame_name', 'End effector'),
                               ('elbow_frame_name', 'Elbow')):
                frame = cfg.get(key, '')
                if frame and frame not in frames:
                    out.append((ERROR, '%s frame %r is not in the URDF.' % (label, frame)))
            if not missing:
                out.append((OK, 'URDF: %d joints match, end effector %s.'
                            % (len(joints), cfg.get('ee_frame_name', '?'))))

    xml_rel = cfg.get('mujoco_xml_path', '')
    xml = resolve(ws_src, xml_rel)
    if not xml_rel:
        out.append((ERROR, 'No MuJoCo model set: the robot cannot be simulated.'))
    elif not os.path.isfile(xml):
        out.append((ERROR, 'MuJoCo model not found: %s' % xml_rel))
    else:
        names = mjcf_names(xml)
        if names is None:
            out.append((ERROR, 'The MuJoCo model cannot be read: %s' % xml_rel))
        else:
            missing = [j for j in joints if j not in names['joint']]
            if missing:
                out.append((ERROR, 'Joint%s not in the MuJoCo model: %s' % (
                    '' if len(missing) == 1 else 's', ', '.join(missing))))
            motors = ['torq_j%d' % (i + 1) for i in range(n)]
            absent = [m for m in motors if m not in names['actuator']]
            if absent:
                out.append((ERROR, 'The MuJoCo model has no motor%s %s; the simulator '
                            'drives joint k through a motor named torq_jk.'
                            % ('' if len(absent) == 1 else 's', ', '.join(absent[:4]))))
            if not missing and not absent:
                out.append((OK, 'MuJoCo: motor torq_j1 drives the joint.' if n == 1 else
                            'MuJoCo: %d motors torq_j1..%d drive the joints.' % (n, n)))
            if not {'force_sensor', 'torque_sensor'} <= names['sensor']:
                out.append((WARN, 'No force_sensor / torque_sensor in the MuJoCo model: '
                            'simulated wrench readings will be zero.'))
    hm = cfg.get('hardware_model')
    if isinstance(hm, dict) and hm.get('enabled', True):
        out.append((OK, 'Simulated like the real robot (hardware_model%s).'
                    % (': ' + hm['source'] if hm.get('source') else '')))
    elif cfg.get('is_supported'):
        out.append((WARN, 'Simulated as an ideal arm: tunes made in simulation may not hold on '
                    'the real one. Measure it (python3 -m fic_studio.hardware_model --env '
                    'hardware --robot <name> --write) and turn on "Simulate like the real '
                    'robot" (Hardware tab).'))
    order = {ERROR: 0, WARN: 1, OK: 2}
    return sorted(out, key=lambda m: order[m[0]])


def sim_ready(messages):
    return not any(level == ERROR for level, _ in messages)
