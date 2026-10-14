"""The robot's shape for the web app's 3D view, from its MuJoCo model.

The browser cannot read an MJCF, so the Studio turns the compiled model into
two things:

* ``scene_dict``  a small JSON description: the body tree, each body's joints
  (axis, position, range) and its geoms (box, sphere, capsule, plane, mesh).
* ``mesh_bytes``  one binary buffer per mesh — vertices and triangles, fetched
  once and cached by the browser (about 1.6 MB for a Kinova Gen3).

The browser then poses the bodies from the joint values the state stream
already carries, so nothing is rendered on the robot's PC and a phone can
draw it. ``pose_bodies`` is the same maths the page does, kept here so it can
be tested against MuJoCo's own forward kinematics.

Needs the ``mujoco`` Python module (a FIC dependency); without it the Studio
simply offers no 3D view.
"""
import math
import struct

#: MuJoCo geom types this understands; anything else is skipped.
GEOM_TYPES = {0: 'plane', 2: 'sphere', 3: 'capsule', 4: 'ellipsoid',
              5: 'cylinder', 6: 'box', 7: 'mesh'}
#: MuJoCo joint types: 0 free, 1 ball, 2 slide, 3 hinge.
HINGE, SLIDE = 3, 2
#: MuJoCo equality type 2 couples one joint to another (a gripper's mimics).
EQ_JOINT = 2
#: Buffer layout version the page checks.
MESH_FORMAT = 1


def load_model(xml_path):
    """Compile a MuJoCo model from ``xml_path`` (raises if mujoco is missing)."""
    import mujoco
    return mujoco.MjModel.from_xml_path(str(xml_path))


def _name(model, obj, i):
    import mujoco
    return mujoco.mj_id2name(model, obj, i) or ''


def scene_dict(model, robot='', joint_names=()):
    """Return the JSON-able scene of ``model``.

    ``joint_names`` are the robot's own driven joints (robots_config); their
    indices come back as ``drive_joints``, in that order, so the page can put
    the state stream's joint values on the right ones.
    """
    import mujoco
    bodies, joints, geoms, meshes = [], [], [], []
    for i in range(model.nbody):
        bodies.append({
            'name': _name(model, mujoco.mjtObj.mjOBJ_BODY, i),
            'parent': int(model.body_parentid[i]) if i else -1,
            'pos': [float(v) for v in model.body_pos[i]],
            'quat': [float(v) for v in model.body_quat[i]],     # w, x, y, z
        })
    for i in range(model.njnt):
        jtype = int(model.jnt_type[i])
        if jtype not in (HINGE, SLIDE):
            continue                       # free and ball joints: not driven here
        joints.append({
            'name': _name(model, mujoco.mjtObj.mjOBJ_JOINT, i),
            'body': int(model.jnt_bodyid[i]),
            'type': 'hinge' if jtype == HINGE else 'slide',
            'axis': [float(v) for v in model.jnt_axis[i]],
            'pos': [float(v) for v in model.jnt_pos[i]],
            'range': [float(v) for v in model.jnt_range[i]],
        })
    for i in range(model.ngeom):
        kind = GEOM_TYPES.get(int(model.geom_type[i]))
        if kind is None:
            continue
        mesh_id = int(model.geom_dataid[i]) if kind == 'mesh' else -1
        geoms.append({
            'body': int(model.geom_bodyid[i]),
            'type': kind,
            'size': [float(v) for v in model.geom_size[i]],
            'pos': [float(v) for v in model.geom_pos[i]],
            'quat': [float(v) for v in model.geom_quat[i]],
            'rgba': [float(v) for v in model.geom_rgba[i]],
            'mesh': mesh_id,
        })
    for i in range(model.nmesh):
        meshes.append({
            'id': i,
            'name': _name(model, mujoco.mjtObj.mjOBJ_MESH, i),
            'nvert': int(model.mesh_vertnum[i]),
            'nface': int(model.mesh_facenum[i]),
        })
    by_name = {j['name']: n for n, j in enumerate(joints)}
    drive = [by_name[n] for n in joint_names if n in by_name]
    scene = {'robot': robot, 'format': MESH_FORMAT, 'bodies': bodies, 'joints': joints,
             'geoms': geoms, 'meshes': meshes, 'drive_joints': drive}
    gripper = gripper_spec(model, joints, by_name, drive)
    if gripper:
        scene['gripper'] = gripper
    return scene


def gripper_spec(model, joints, by_name, drive):
    """Return how the gripper opens, or None if this robot has none.

    A gripper is an actuator on a joint the robot does not drive as an arm
    joint; its fingers follow through MuJoCo's joint-to-joint equalities
    (``factor`` each). Which end of the joint's range is *open* is measured
    from the model — the pose whose finger bodies are furthest apart — rather
    than assumed, because models differ on that.
    """
    import mujoco
    driven = None
    for i in range(model.nu):
        joint_id = int(model.actuator_trnid[i][0])
        name = _name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        index = by_name.get(name)
        if index is None or index in drive:
            continue
        driven = index
        break
    if driven is None:
        return None
    driven_name = joints[driven]['name']
    driven_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, driven_name)
    coupled = []
    for i in range(model.neq):
        if int(model.eq_type[i]) != EQ_JOINT or int(model.eq_obj2id[i]) != driven_id:
            continue
        name = _name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.eq_obj1id[i]))
        if name in by_name:
            coupled.append({'joint': by_name[name], 'factor': float(model.eq_data[i][1])})
    lo, hi = joints[driven]['range']
    if lo == hi:
        return None
    spec = {'joint': driven, 'coupled': coupled, 'closed': lo, 'open': hi}
    if _spread(model, joints, spec, lo) > _spread(model, joints, spec, hi):
        spec['closed'], spec['open'] = hi, lo
    return spec


def _spread(model, joints, spec, angle):
    """Return how far apart the gripper's moving bodies sit at ``angle``."""
    import mujoco
    data = mujoco.MjData(model)
    for item in [{'joint': spec['joint'], 'factor': 1.0}] + spec['coupled']:
        name = joints[item['joint']]['name']
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = angle * item['factor']
    mujoco.mj_forward(model, data)
    points = [data.xpos[model.jnt_bodyid[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, joints[c['joint']]['name'])]] for c in spec['coupled']]
    worst = 0.0
    for i, a in enumerate(points):
        for b in points[i + 1:]:
            worst = max(worst, sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5)
    return worst


def gripper_values(scene, fraction):
    """Return {joint index: angle} for a gripper ``fraction`` open (0..1)."""
    spec = scene.get('gripper')
    if not spec:
        return {}
    fraction = min(1.0, max(0.0, float(fraction)))
    angle = spec['closed'] + fraction * (spec['open'] - spec['closed'])
    values = {spec['joint']: angle}
    for item in spec['coupled']:
        values[item['joint']] = angle * item['factor']
    return values


def mesh_bytes(model, mesh_id):
    """Return one mesh as bytes: 'FICM', format, nvert, nface, vertices, triangles."""
    if not 0 <= mesh_id < model.nmesh:
        raise IndexError('no mesh %r' % (mesh_id,))
    va, vn = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
    fa, fn = int(model.mesh_faceadr[mesh_id]), int(model.mesh_facenum[mesh_id])
    verts = model.mesh_vert[va:va + vn].astype('<f4').tobytes()
    faces = model.mesh_face[fa:fa + fn].astype('<u4').tobytes()
    head = struct.pack('<4sIII', b'FICM', MESH_FORMAT, vn, fn)
    return head + verts + faces


# ── the page's maths, so it can be tested here ───────────────────────────
def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def _quat_rot(q, v):
    w, x, y, z = q
    t = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    return (v[0] + w * t[0] + y * t[2] - z * t[1],
            v[1] + w * t[1] + z * t[0] - x * t[2],
            v[2] + w * t[2] + x * t[1] - y * t[0])


def _axis_quat(axis, angle):
    n = math.sqrt(sum(a * a for a in axis)) or 1.0
    s = math.sin(angle / 2.0) / n
    return (math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def pose_bodies(scene, values):
    """Return each body's world (position, quaternion) for joint ``values``.

    ``values`` maps a joint's index in ``scene['joints']`` to its position
    (radians, or metres for a slide joint). A body's joints move it relative
    to its parent, about the joint's own position and axis — the same rule
    MuJoCo uses, and what the page implements.
    """
    joints_of = {}
    for n, j in enumerate(scene['joints']):
        joints_of.setdefault(j['body'], []).append(n)
    out = []
    for i, body in enumerate(scene['bodies']):
        pos = tuple(body['pos'])
        quat = tuple(body['quat'])
        for n in joints_of.get(i, ()):
            j = scene['joints'][n]
            value = float(values.get(n, 0.0))
            if j['type'] == 'slide':
                pos = tuple(p + value * a for p, a in zip(pos, j['axis']))
                continue
            rot = _axis_quat(j['axis'], value)
            anchor = tuple(j['pos'])
            offset = tuple(a - r for a, r in zip(anchor, _quat_rot(rot, anchor)))
            pos = tuple(p + o for p, o in zip(pos, _quat_rot(quat, offset)))
            quat = _quat_mul(quat, rot)
        parent = body['parent']
        if parent >= 0:
            ppos, pquat = out[parent]
            pos = tuple(p + v for p, v in zip(ppos, _quat_rot(pquat, pos)))
            quat = _quat_mul(pquat, quat)
        out.append((pos, quat))
    return out


def reach(scene, ee_name):
    """The sphere the end effector can reach: ``{'center': [x, y, z], 'radius': r}``, or None.

    Centre: the second driven joint (the shoulder pivot; the first joint only
    turns the arm about its own axis). Radius: the chain's full length from
    there to the end-effector frame, i.e. the links laid end to end, which is
    how far the arm reaches when stretched. Taken at the zero pose, in the
    scene's frame (the robot's base frame, as the 3D view uses).
    """
    names = [b['name'] for b in scene['bodies']]
    drive = scene.get('drive_joints') or []
    if ee_name not in names or not drive:
        return None
    posed = pose_bodies(scene, {})
    pivot_joint = scene['joints'][drive[1] if len(drive) > 1 else drive[0]]
    pos, quat = posed[pivot_joint['body']]
    pivot = tuple(p + v for p, v in zip(pos, _quat_rot(quat, tuple(pivot_joint['pos']))))
    later = [scene['joints'][n]['body'] for n in drive[2:]]
    points = [pivot] + [posed[b][0] for b in later] + [posed[names.index(ee_name)][0]]
    radius = sum(sum((a - b) ** 2 for a, b in zip(p, q)) ** 0.5 for p, q in zip(points, points[1:]))
    if radius <= 0:
        return None
    return {'center': [float(v) for v in pivot], 'radius': float(radius)}
