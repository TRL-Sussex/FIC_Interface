"""Closed-loop robots in an import: their loops, joints and what drives them.

The loops come from the robot's own MuJoCo model when there is one
(``mujoco_model.closures``), else from markup in the URDF:

* Drake's ``<drake:ball_constraint>`` (body A / point p_AP / body B / p_BQ);
* onshape-to-robot's pairs of frames ``closing_<name>_1`` / ``closing_<name>_2``;

else the user adds them on the review screen. With the loops known, the
mechanism is analysed on a MuJoCo model -- the robot's own, or a quick
mesh-free one built from the URDF -- to find how its end effector moves,
which joints to drive and whether it only turns.
"""
import copy
import re
import xml.etree.ElementTree as ET

from fic_studio.robot_import import closed_loop
from fic_studio.robot_import.closed_loop import Closure
from fic_studio.robot_import.urdf_info import link_poses
import mujoco
import numpy as np

_CLOSING = re.compile(r'^closing_(.+)_([12])$')


# ── Loops written into the URDF ───────────────────────────────────────────

def _local(tag):
    return tag.rsplit('}', 1)[-1].split(':')[-1]


def _vec(text):
    try:
        v = [float(x) for x in (text or '').split()]
    except ValueError:
        return None
    return v if len(v) == 3 else None


def urdf_closures(info):
    """Loops the URDF itself declares: [Closure], plus notes."""
    out, notes = [], []
    for el in info.root.iter():
        if _local(el.tag) != 'ball_constraint':
            continue
        parts = {_local(c.tag): c for c in el}
        a = parts.get('ball_constraint_body_A')
        b = parts.get('ball_constraint_body_B')
        pa = _vec(parts['ball_constraint_p_AP'].get('value')) \
            if 'ball_constraint_p_AP' in parts else None
        pb = _vec(parts['ball_constraint_p_BQ'].get('value')) \
            if 'ball_constraint_p_BQ' in parts else None
        if a is None or b is None or pa is None or pb is None or \
                a.get('name') not in info.links or b.get('name') not in info.links:
            notes.append(('warn', 'A <drake:ball_constraint> is incomplete or names a link '
                          'the URDF does not have; it was skipped.'))
            continue
        out.append(Closure(a.get('name'), pa, b.get('name'), pb,
                           name=el.get('name', 'loop%d' % (len(out) + 1)),
                           source='the URDF (Drake ball constraint)'))
    ends = {}
    for link in info.links:
        m = _CLOSING.match(link)
        if m:
            ends.setdefault(m.group(1), {})[m.group(2)] = link
    for name, pair in sorted(ends.items()):
        if len(pair) == 2:
            out.append(Closure(pair['1'], [0.0, 0.0, 0.0], pair['2'], [0.0, 0.0, 0.0],
                               name=name, source='the URDF (onshape-to-robot closing frames)'))
        else:
            notes.append(('warn', 'Closing frame closing_%s has no partner.' % name))
    return out, notes


def closure_from_point(info, body_a, pos_a, body_b, q=None):
    """Make a loop joining ``pos_a`` on ``body_a`` to the same spot on ``body_b``.

    The spot is where ``pos_a`` is at the URDF's zero pose (or ``q``), which
    is how a loop cut for a URDF is normally left: assembled.
    """
    poses = link_poses(info, q)

    def arr(p):
        return np.array([row[:3] for row in p[:3]]), np.array([row[3] for row in p[:3]])
    ra, ta = arr(poses[body_a])
    rb, tb = arr(poses[body_b])
    world = ra @ np.asarray(pos_a, float) + ta
    return Closure(body_a, [float(v) for v in pos_a], body_b,
                   [float(v) for v in rb.T @ (world - tb)], source='added by you')


# ── Which joints form the mechanism ───────────────────────────────────────

def mechanism_joints(info, closures, ee_link):
    """Movable joints between the base and the loops' links or the end effector."""
    names = []
    for link in {c.body_a for c in closures} | {c.body_b for c in closures} | {ee_link}:
        for j in info.chain_to(link):
            if j.type not in ('fixed',) and j.name not in names:
                names.append(j.name)
    order = {j.get('name'): i for i, j in enumerate(info.root.findall('joint'))}
    return sorted(names, key=lambda n: order.get(n, 0))


def default_ee(info, closures, tol=1e-3):
    """Pick the link a closed-loop robot is controlled by.

    A free body the loops hold (the CSPM's platform) if there is one; else a
    link sitting where a loop closes, preferring one named like an end
    effector (a five-bar's ``tip``); else the link most loops end on.
    """
    from fic_studio.robot_import.urdf_info import _hint_rank, EE_NAME_HINTS
    if not closures:
        return ''
    held = [c.body_a for c in closures] + [c.body_b for c in closures]
    free = {j.child for j in info.joints.values() if j.type in ('floating', 'planar')}
    loose = [link for link in held if link in free]
    if loose:
        return max(set(loose), key=loose.count)
    poses = link_poses(info)

    def world(link, p=(0.0, 0.0, 0.0)):
        t = poses[link]
        return np.array([sum(t[i][k] * v for k, v in enumerate(p)) + t[i][3] for i in range(3)])
    points = [world(c.body_a, c.pos_a) for c in closures]
    at_loop = [link for link in info.links
               if any(np.linalg.norm(world(link) - p) < tol for p in points)]
    if at_loop:
        return min(at_loop, key=lambda n: (_hint_rank(n) == len(EE_NAME_HINTS),
                                           _hint_rank(n), -len(info.chain_to(n)), n))
    return max(set(held), key=lambda n: (held.count(n),
                                         sum(1 for c in closures if c.body_b == n)))


# ── A MuJoCo model to analyse it on ───────────────────────────────────────

def _strip_geometry(root):
    for link in root.findall('link'):
        for el in link.findall('visual') + link.findall('collision'):
            link.remove(el)


def floating_parents(info):
    """Floating / planar joints not on the base: MuJoCo only frees top-level bodies."""
    top = {j.name for j in info.child_joints.get(info.root_link, [])}
    return [j for j in info.joints.values()
            if j.type in ('floating', 'planar') and j.name not in top]


def add_loops(spec, closures, prefix='loop'):
    """Add a site pair and a connect constraint to ``spec`` for each loop."""
    for k, c in enumerate(closures):
        s1, s2 = '%s%d_a' % (prefix, k + 1), '%s%d_b' % (prefix, k + 1)
        for body, pos, site in ((c.body_a, c.pos_a, s1), (c.body_b, c.pos_b, s2)):
            b = spec.body(body)
            if b is None:
                raise KeyError(body)
            b.add_site(name=site, pos=[float(v) for v in pos], size=[0.004, 0.004, 0.004],
                       rgba=[1, 0.35, 0.1, 1], group=4)
        eq = spec.add_equality(type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_SITE,
                               name1=s1, name2=s2)
        eq.name = c.name or '%s%d' % (prefix, k + 1)
        if c.solref:
            eq.solref = c.solref
        if c.solimp:
            eq.solimp = c.solimp


def free_joints(spec, joints):
    """Give each named floating joint's child body six free dofs (3 slides + ball)."""
    for j in joints:
        body = spec.body(j.child)
        for axis, suffix in (([1, 0, 0], '_x'), ([0, 1, 0], '_y'), ([0, 0, 1], '_z')):
            body.add_joint(name=j.name + suffix, type=mujoco.mjtJoint.mjJNT_SLIDE, axis=axis)
        body.add_joint(name=j.name, type=mujoco.mjtJoint.mjJNT_BALL)


def kinematic_model(info, closures):
    """Build a mesh-free MuJoCo model of the URDF with its loops closed, for analysis."""
    root = copy.deepcopy(info.root)
    _strip_geometry(root)
    inner = {j.name for j in floating_parents(info)}
    for j in root.findall('joint'):
        if j.get('name') in inner:
            j.set('type', 'fixed')
    for old in root.findall('mujoco'):
        root.remove(old)
    ET.SubElement(ET.SubElement(root, 'mujoco'), 'compiler', inertiafromgeom='false',
                  boundmass='0.001', boundinertia='1e-6', fusestatic='false',
                  discardvisual='true', angle='radian')
    spec = mujoco.MjSpec.from_string(ET.tostring(root, encoding='unicode'))
    free_joints(spec, floating_parents(info))
    add_loops(spec, closures)
    return spec.compile()


# ── The analysis ──────────────────────────────────────────────────────────

def analyse(model, ee_body, candidates, driven=None, start=None):
    """Work out what the mechanism does, on ``model`` (loops included).

    Returns a dict: ``ee_dof`` (ways the end effector can move),
    ``orientation_only``, ``driven`` (the given set, else one picked
    base-first from ``candidates``), ``determined`` (the driven joints fix
    the end effector), ``residual`` (m, loops at the reference pose),
    ``extra_dof`` (internal motions the driven joints leave free).
    """
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0 if start is None else start
    residual = closed_loop.assemble(model, data, {})
    closed_loop.generic_pose(model, data, start=data.qpos.copy())
    ee_dof = closed_loop.ee_mobility(model, data, ee_body)
    if driven is None:
        driven = closed_loop.pick_driven(model, data, closed_loop.base_first(model, candidates),
                                         ee_body)
    determined = closed_loop.determines(model, data, driven, ee_body) if driven else False
    return {
        'ee_dof': ee_dof,
        'orientation_only': closed_loop.orientation_only(model, data, ee_body),
        'driven': list(driven),
        'determined': determined,
        'residual': residual,
        'mobility': closed_loop.mobility(model, data),
    }


def body_for_link(src, info, link):
    """Return the MuJoCo body of ``src`` that carries URDF link ``link`` ('' if none)."""
    model = src.model
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link)
    if bid >= 0:
        return link
    parent = info.parent_joint.get(link)
    if parent is not None:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, parent.name)
        if jid >= 0:
            return src.body_name(int(model.jnt_bodyid[jid]))
    return ''


def link_offset(src, info, link, body):
    """4x4 pose of URDF ``link`` in MuJoCo ``body``'s frame (fixed: same rigid piece)."""
    from fic_studio.robot_import.mujoco_model import reference_state
    data, poses = reference_state(src, info)
    bid = mujoco.mj_name2id(src.model, mujoco.mjtObj.mjOBJ_BODY, body)
    tb = np.eye(4)
    tb[:3, :3] = data.xmat[bid].reshape(3, 3)
    tb[:3, 3] = data.xpos[bid]
    return np.linalg.inv(tb) @ np.array(poses[link])


def pose_xyzw(model, data, body, offset=None):
    """(position, [x, y, z, w]) of ``body`` (times ``offset``) in world."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    t = np.eye(4)
    t[:3, :3] = data.xmat[bid].reshape(3, 3)
    t[:3, 3] = data.xpos[bid]
    if offset is not None:
        t = t @ offset
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, t[:3, :3].flatten())
    w, x, y, z = (float(v) for v in q)
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return [float(v) for v in t[:3, 3]], [x, y, z, w]


def view_joints(info, mechanism, balls=()):
    """Joints to show for a closed-loop robot: every mechanism joint, balls as r/p/y."""
    out = []
    for name in mechanism:
        j = info.joints.get(name)
        if j is None:
            continue
        if j.type in ('floating', 'planar') or name in balls:
            out += [name + '_roll', name + '_pitch', name + '_yaw']
        else:
            out.append(name)
    return out


def elbow_for(info, driven):
    """Pick an elbow link for the longest run of driven joints on one branch, or ''."""
    best = []
    for link in info.links:
        chain = [j.name for j in info.movable_chain_to(link) if j.name in driven]
        if len(chain) > len(best):
            best = chain
    if len(best) < 3:
        return ''
    return info.joints[best[(len(best) + 1) // 2 - 1]].child


def home_key(spec, home_driven):
    """Make ``spec``'s ``home`` keyframe the home pose, with the loops closed.

    Starts from the existing ``home`` key (or the reference pose), puts the
    driven joints (name -> value) where asked, and closes the loops with
    the rest. Returns the loops' residual (m).
    """
    model = spec.compile()
    data = mujoco.MjData(model)
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    data.qpos[:] = model.key_qpos[kid] if kid >= 0 else model.qpos0
    residual = closed_loop.assemble(model, data, home_driven or {})
    for key in list(spec.keys):
        if key.name == 'home':
            spec.delete(key)
    spec.add_key(name='home', qpos=list(data.qpos), time=0.0)
    return residual
