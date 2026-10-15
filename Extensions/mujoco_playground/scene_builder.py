"""MuJoCo Playground scenes: a robot's own simulator scene with a table and objects added.

The robot's scene (robots_config.yaml ``mujoco_xml_path``) is loaded with MuJoCo's model editor
(``mujoco.MjSpec``), the table and objects are added to it, and the result is written as one MJCF
file the Studio's simulator loads instead of the robot's scene. The robot keeps its joint and
actuator names, so the FIC controller, the gripper bridge and the robot's hardware model work as
in any simulation. Objects are bodies named ``obj_<name>``: the simulator publishes their poses on
``robot{N}_simulation/object_poses``.

Plain functions of their arguments (no ROS, no Qt), so they can be tested headless.
"""
import os
import re


def _mujoco():
    """MuJoCo's Python package, imported when a scene is built or checked, so that the
    Playground's page still opens (and says why) where it is missing."""
    try:
        import mujoco
    except ImportError as exc:
        raise ValueError('MuJoCo is not installed for the Studio\'s Python (pip install mujoco)') from exc
    return mujoco

#: Object shapes: size fields (m) and the half-height that puts the shape on a surface.
SHAPES = {
    'box': {'fields': ('half_x', 'half_y', 'half_z'), 'default': (0.02, 0.02, 0.02),
            'half_height': lambda s: s[2]},
    'sphere': {'fields': ('radius',), 'default': (0.025,), 'half_height': lambda s: s[0]},
    'cylinder': {'fields': ('radius', 'half_height'), 'default': (0.02, 0.04),
                 'half_height': lambda s: s[1]},
}
COLORS = {'red': (0.85, 0.2, 0.2, 1), 'green': (0.2, 0.7, 0.3, 1), 'blue': (0.2, 0.4, 0.85, 1),
          'yellow': (0.95, 0.8, 0.2, 1), 'orange': (0.95, 0.5, 0.1, 1), 'grey': (0.6, 0.6, 0.6, 1)}
#: Default table: a fixed block in front of the robot, its top at `height`.
TABLE = {'x': 0.55, 'y': 0.0, 'half_x': 0.25, 'half_y': 0.4, 'height': 0.2}
OBJECT_PREFIX = 'obj_'
MAX_OBJECTS = 12

#: Ready-made scenes: (title, table on, objects).
PRESETS = {
    'pick': ('Pick and place', True, [
        {'name': 'red_cube', 'shape': 'box', 'size': [0.02, 0.02, 0.02], 'x': 0.5, 'y': -0.15, 'color': 'red'},
        {'name': 'blue_cube', 'shape': 'box', 'size': [0.02, 0.02, 0.02], 'x': 0.5, 'y': 0.15, 'color': 'blue'}]),
    'stack': ('Stacking', True, [
        {'name': 'cube_1', 'shape': 'box', 'size': [0.02, 0.02, 0.02], 'x': 0.45, 'y': -0.2, 'color': 'red'},
        {'name': 'cube_2', 'shape': 'box', 'size': [0.02, 0.02, 0.02], 'x': 0.45, 'y': 0.0, 'color': 'green'},
        {'name': 'cube_3', 'shape': 'box', 'size': [0.02, 0.02, 0.02], 'x': 0.45, 'y': 0.2, 'color': 'blue'}]),
    'shapes': ('Shapes', True, [
        {'name': 'cube', 'shape': 'box', 'size': [0.02, 0.02, 0.02], 'x': 0.5, 'y': -0.2, 'color': 'orange'},
        {'name': 'ball', 'shape': 'sphere', 'size': [0.025], 'x': 0.5, 'y': 0.0, 'color': 'yellow'},
        {'name': 'can', 'shape': 'cylinder', 'size': [0.02, 0.04], 'x': 0.5, 'y': 0.2, 'color': 'green'}]),
    'empty': ('Empty table', True, []),
}


def default_out_dir():
    return os.path.join(os.path.expanduser('~'), '.fic_studio', 'playground')


def _scene_path(cfg, ws_src):
    rel = (cfg or {}).get('mujoco_xml_path') or ''
    if not rel:
        return None
    path = rel if os.path.isabs(rel) else os.path.join(ws_src, rel)
    return path if os.path.isfile(path) else None


def robot_status(name, cfg, ws_src):
    """{'name', 'scene', 'gripper', 'ok', 'reason'}: whether ``name`` can be used in the
    Playground. A fixed-base arm (6 joints or more) whose scene loads and has the torque motors
    torq_j1..torq_jN the FIC controller drives; 'gripper' when its scene has the gripper actuator."""
    out = {'name': name, 'scene': None, 'gripper': False, 'ok': False, 'reason': ''}
    n = int((cfg or {}).get('num_joints') or 0)
    scene = _scene_path(cfg, ws_src)
    if scene is None:
        out['reason'] = 'no simulator scene'
        return out
    out['scene'] = scene
    if n < 6:
        out['reason'] = 'not an arm'
        return out
    try:
        mujoco = _mujoco()
    except ValueError as exc:
        out['reason'] = str(exc)
        return out
    try:
        m = mujoco.MjModel.from_xml_path(scene)
    except Exception as exc:                               # a broken scene: say why
        out['reason'] = 'its scene does not load: %s' % str(exc).splitlines()[0][:120]
        return out
    if any(m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(m.njnt)):
        out['reason'] = 'not a fixed-base arm'
        return out
    acts = {m.actuator(i).name for i in range(m.nu)}
    missing = [f'torq_j{k}' for k in range(1, n + 1) if f'torq_j{k}' not in acts]
    if missing:
        out['reason'] = ('its scene has no torque motors (%s...) for the FIC controller to drive'
                         % missing[0])
        return out
    out['gripper'] = 'gripper' in acts
    out['ok'] = True
    return out


def robots(robot_configs, ws_src):
    """robot_status of every robot with a simulator scene, usable ones first."""
    rows = [robot_status(n, c, ws_src) for n, c in sorted((robot_configs or {}).items())
            if _scene_path(c, ws_src)]
    return sorted(rows, key=lambda r: (not r['ok'], r['name']))


def clean_name(name):
    name = re.sub(r'[^a-z0-9_]+', '_', str(name or '').strip().lower()).strip('_')
    if not name:
        raise ValueError('Give the object a name (letters, digits, _)')
    return name[:32]


def check_object(obj):
    """A validated copy of one object: {name, shape, size, x, y, color, mass}."""
    shape = obj.get('shape', 'box')
    if shape not in SHAPES:
        raise ValueError('The shape is box, sphere or cylinder')
    size = obj.get('size') or list(SHAPES[shape]['default'])
    try:
        size = [float(v) for v in size]
    except (TypeError, ValueError):
        raise ValueError('The size is a list of numbers (m)')
    if len(size) != len(SHAPES[shape]['fields']) or not all(0.005 <= v <= 0.15 for v in size):
        raise ValueError('%s: %s, each 0.005 to 0.15 m' % (shape, ', '.join(SHAPES[shape]['fields'])))
    try:
        x, y = float(obj.get('x', 0.5)), float(obj.get('y', 0.0))
        mass = float(obj.get('mass', 0.05))
    except (TypeError, ValueError):
        raise ValueError('Position and mass are numbers')
    if not (0.1 <= abs(x) + abs(y) and abs(x) <= 1.2 and abs(y) <= 1.2):
        raise ValueError('Place objects within 1.2 m of the robot, not on its base')
    if not 0.005 <= mass <= 5.0:
        raise ValueError('Mass 0.005 to 5 kg')
    color = obj.get('color', 'red')
    if color not in COLORS:
        raise ValueError('Colour: %s' % ', '.join(COLORS))
    return {'name': clean_name(obj.get('name')), 'shape': shape, 'size': size, 'x': x, 'y': y,
            'color': color, 'mass': mass}


def surface_height(table, x, y):
    """The height objects rest on at (x, y): the table top over the table, else the floor."""
    if table and abs(x - table['x']) <= table['half_x'] and abs(y - table['y']) <= table['half_y']:
        return table['height']
    return 0.0


def _absolute_dirs(spec, scene):
    d = os.path.dirname(os.path.abspath(scene))
    for attr in ('meshdir', 'texturedir'):
        v = getattr(spec, attr) or ''
        setattr(spec, attr, v if os.path.isabs(v) else os.path.normpath(os.path.join(d, v)))


def build(robot_scene, objects, table=True, out_path=None, name='playground'):
    """Write the Playground scene: ``robot_scene`` (the robot's MJCF) with the table (TABLE when
    ``table`` is True, a dict to change it, False for none) and ``objects`` (check_object dicts)
    resting on it. Returns the path written (default ~/.fic_studio/playground/<name>.xml)."""
    objects = [check_object(o) for o in objects]
    names = [o['name'] for o in objects]
    if len(set(names)) != len(names):
        raise ValueError('Two objects have the same name')
    if len(objects) > MAX_OBJECTS:
        raise ValueError('At most %d objects' % MAX_OBJECTS)
    mujoco = _mujoco()
    spec = mujoco.MjSpec.from_file(robot_scene)
    _absolute_dirs(spec, robot_scene)
    tab = dict(TABLE, **table) if isinstance(table, dict) else (dict(TABLE) if table else None)
    if tab:
        h = tab['height'] / 2.0
        body = spec.worldbody.add_body(name=OBJECT_PREFIX + 'table', pos=[tab['x'], tab['y'], h])
        body.add_geom(name='table_top', type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[tab['half_x'], tab['half_y'], h], rgba=[0.55, 0.45, 0.35, 1],
                      friction=[1.0, 0.01, 0.001])
    for o in objects:
        z = surface_height(tab, o['x'], o['y']) + SHAPES[o['shape']]['half_height'](o['size']) + 0.001
        body = spec.worldbody.add_body(name=OBJECT_PREFIX + o['name'], pos=[o['x'], o['y'], z])
        body.add_freejoint(name=OBJECT_PREFIX + o['name'] + '_free')
        geom_type = {'box': mujoco.mjtGeom.mjGEOM_BOX, 'sphere': mujoco.mjtGeom.mjGEOM_SPHERE,
                     'cylinder': mujoco.mjtGeom.mjGEOM_CYLINDER}[o['shape']]
        size = list(o['size']) + [0.0] * (3 - len(o['size']))
        body.add_geom(name=OBJECT_PREFIX + o['name'] + '_geom', type=geom_type, size=size,
                      mass=o['mass'], rgba=list(COLORS[o['color']]), friction=[1.2, 0.01, 0.001],
                      condim=4)
    spec.compile()                                  # refuses a scene MuJoCo cannot load
    out_path = out_path or os.path.join(default_out_dir(), clean_name(name) + '.xml')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as fh:
        fh.write(spec.to_xml())
    return out_path
