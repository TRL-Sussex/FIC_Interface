"""The robot's own MuJoCo model, when there is one, as its simulation model.

A URDF is required (the controllers read it). If the robot also comes with a
MuJoCo model (``.xml`` and its meshes), that model is better than one
generated from the URDF: someone tuned its contacts, damping and solver, and
it can hold what a URDF cannot -- above all the ``<equality><connect>``
constraints that close a parallel mechanism's loops.

This module finds such a model beside the URDF, reads what the import needs
from it (loop closures in the URDF's link frames, the joints its actuators
drive, a ``home`` keyframe), checks that it and the URDF describe the same
robot, and writes an adapted copy: motors renamed ``torq_j1..N`` in the
config's joint order, the wrench sensors FIC reads, a ``home`` keyframe and
its meshes beside it. Nothing else in it is changed.
"""
from dataclasses import dataclass, field
import math
import os
import shutil
from typing import Dict, List, Optional
import xml.etree.ElementTree as ET

from fic_studio.robot_import import check
from fic_studio.robot_import.closed_loop import Closure
from fic_studio.robot_import.urdf_info import link_poses
import mujoco
import numpy as np

_SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', 'node_modules'}


class MujocoModelError(Exception):
    """The MuJoCo model cannot be used; the message is written for the user."""


def is_mjcf(path):
    """Return True for an ``.xml`` file whose top element is ``<mujoco>``."""
    if not path.lower().endswith('.xml') or not os.path.isfile(path):
        return False
    try:
        for _, el in ET.iterparse(path, events=('start',)):
            return el.tag == 'mujoco'
    except (ET.ParseError, OSError):
        return False
    return False


def _is_wrapper(path):
    """Tell a scene file: its worldbody holds no body of its own (the robot is included)."""
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError):
        return True
    world = root.find('worldbody')
    return world is None or world.find('body') is None


def find_mjcf(roots, urdf_joints, max_depth=5):
    """Find the MuJoCo model under ``roots`` sharing the most joints with the URDF, or ''.

    A robot file is preferred over a scene file that merely includes it.
    """
    wanted = set(urdf_joints)
    best, best_key = '', None
    for root in roots:
        root = os.path.abspath(root)
        base = root.rstrip(os.sep).count(os.sep)
        for dirpath, dirnames, filenames in os.walk(root):
            if dirpath.count(os.sep) - base >= max_depth:
                dirnames[:] = []
            dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS and not d.startswith('.')]
            for f in filenames:
                path = os.path.join(dirpath, f)
                if not is_mjcf(path):
                    continue
                names = check.mjcf_names(path)
                overlap = len(set(names['joint']) & wanted) if names else 0
                if not overlap:
                    continue
                key = (overlap, not _is_wrapper(path), -path.count(os.sep), path)
                if best_key is None or key > best_key:
                    best, best_key = path, key
    return best


@dataclass
class Actuator:
    name: str
    joint: str
    motor: bool                       # torque in, no position/velocity servo
    effort: Optional[float]           # N m it can deliver, when the model says


@dataclass
class SourceModel:
    """A user's MuJoCo model and what the import reads from it."""

    path: str
    spec: object
    model: object
    joints: List[str]                          # hinge and slide joints, model order
    balls: List[str]                           # ball joints
    actuators: List[Actuator]
    gripper: Optional[str]                     # name of an actuator called "gripper"
    has_floor: bool
    home: Dict[str, float] = field(default_factory=dict)
    messages: List[tuple] = field(default_factory=list)

    def body_name(self, bid):
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid) or 'body%d' % bid


def load(path):
    """Read a MuJoCo model; MujocoModelError when MuJoCo cannot compile it."""
    try:
        spec = mujoco.MjSpec.from_file(path)
        model = spec.compile()
    except Exception as exc:         # MuJoCo raises plain ValueError / mujoco.FatalError
        raise MujocoModelError('MuJoCo could not load %s: %s'
                               % (os.path.basename(path), str(exc).strip()))
    name = lambda kind, i: mujoco.mj_id2name(model, kind, i) or ''  # noqa: E731
    joints, balls = [], []
    for j in range(model.njnt):
        # int(): from MuJoCo 3.16 an enum is not equal to the NumPy integer of a model array, so
        # `x in (enum, ...)` is always False without it
        t = int(model.jnt_type[j])
        if t in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            joints.append(name(mujoco.mjtObj.mjOBJ_JOINT, j))
        elif t == mujoco.mjtJoint.mjJNT_BALL:
            balls.append(name(mujoco.mjtObj.mjOBJ_JOINT, j))
    actuators, gripper = [], None
    for a in range(model.nu):
        aname = name(mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        if aname == 'gripper':
            gripper = aname
            continue
        if model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT:
            continue
        joint = name(mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[a][0])
        motor = (model.actuator_gaintype[a] == mujoco.mjtGain.mjGAIN_FIXED
                 and model.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_NONE)
        gear = abs(float(model.actuator_gear[a][0])) or 1.0
        effort = None
        if model.actuator_forcelimited[a]:
            effort = float(max(abs(v) for v in model.actuator_forcerange[a]))
        elif motor and model.actuator_ctrllimited[a]:
            effort = float(max(abs(v) for v in model.actuator_ctrlrange[a])) * gear
        actuators.append(Actuator(aname, joint, motor, effort))
    has_floor = any(model.geom_bodyid[g] == 0 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE
                    for g in range(model.ngeom))
    home = {}
    if model.nkey:
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
        qpos = model.key_qpos[kid if kid >= 0 else 0]
        for j in range(model.njnt):
            if int(model.jnt_type[j]) in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                home[name(mujoco.mjtObj.mjOBJ_JOINT, j)] = float(qpos[model.jnt_qposadr[j]])
    return SourceModel(path=os.path.abspath(path), spec=spec, model=model, joints=joints,
                       balls=balls, actuators=actuators, gripper=gripper,
                       has_floor=has_floor, home=home)


# ── Reading it against the URDF ───────────────────────────────────────────

def _inv_apply(pose, p):
    """Point ``p`` (world) in the frame whose world pose is the 4x4 ``pose``."""
    r = np.array([row[:3] for row in pose[:3]])
    t = np.array([row[3] for row in pose[:3]])
    return r.T @ (np.asarray(p) - t)


def _link_for_body(src, info, bid):
    """Return the URDF link a MuJoCo body is rigidly part of, or '' if none matches.

    By name first; else through the joint that moves the body (joint names
    are shared by contract); else the body's parent, since a body welded to
    its parent is part of the same rigid piece.
    """
    model = src.model
    while bid > 0:
        name = src.body_name(bid)
        if name in info.links:
            return name
        for j in range(model.body_jntadr[bid], model.body_jntadr[bid] + model.body_jntnum[bid]):
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if jname in info.joints and model.body_jntnum[bid] == 1:
                return info.joints[jname].child
        if model.body_jntnum[bid]:
            return ''
        bid = model.body_parentid[bid]
    return info.root_link


def reference_state(src, info):
    """(MjData at the model's reference pose, URDF link poses at the same joint values)."""
    model = src.model
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    q = {}
    for j in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jname in info.joints and int(model.jnt_type[j]) in (mujoco.mjtJoint.mjJNT_HINGE,
                                                               mujoco.mjtJoint.mjJNT_SLIDE):
            q[jname] = float(data.qpos[model.jnt_qposadr[j]])
    return data, link_poses(info, q)


def closures(src, info):
    """Read the model's loop closures, in the URDF's link frames, plus notes.

    Each point is taken to world coordinates at the model's reference pose
    and brought back into the matching URDF link's frame, so it does not
    matter where either file puts its body frames.
    """
    model = src.model
    data, poses = reference_state(src, info)
    out, notes = [], []
    for e in range(model.neq):
        if not model.eq_active0[e]:
            continue
        ename = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, e) or 'loop%d' % (e + 1)
        etype = model.eq_type[e]
        if etype == mujoco.mjtEq.mjEQ_JOINT:
            continue                  # a coupling: simulated, not a loop
        if etype != mujoco.mjtEq.mjEQ_CONNECT:
            notes.append(('warn', "The MuJoCo model's %s constraint %s is simulated, but the "
                          'controllers only model point (connect) loops.'
                          % (mujoco.mjtEq(etype).name.replace('mjEQ_', '').lower(), ename)))
            continue
        a, b = int(model.eq_obj1id[e]), int(model.eq_obj2id[e])
        if model.eq_objtype[e] == mujoco.mjtObj.mjOBJ_SITE:
            ba, bb = int(model.site_bodyid[a]), int(model.site_bodyid[b])
            pa, pb = data.site_xpos[a].copy(), data.site_xpos[b].copy()
        else:
            ba, bb = a, b
            pa = data.xpos[a] + data.xmat[a].reshape(3, 3) @ model.eq_data[e][0:3]
            pb = data.xpos[b] + data.xmat[b].reshape(3, 3) @ model.eq_data[e][3:6]
        la, lb = _link_for_body(src, info, ba), _link_for_body(src, info, bb)
        if not la or not lb:
            notes.append(('error', 'Loop %s joins MuJoCo bodies %s and %s, which have no match '
                          'in the URDF.' % (ename, src.body_name(ba), src.body_name(bb))))
            continue
        out.append(Closure(body_a=la, pos_a=list(_inv_apply(poses[la], pa)),
                           body_b=lb, pos_b=list(_inv_apply(poses[lb], pb)), name=ename,
                           solref=[float(v) for v in model.eq_solref[e]],
                           solimp=[float(v) for v in model.eq_solimp[e]],
                           source='the MuJoCo model'))
    return out, notes


def compare(src, info, tol_m=0.005, tol_deg=2.0):
    """Joints where the MuJoCo model and the URDF disagree: [(joint, why)].

    Each shared hinge / slide must have the same axis through the same
    point at the reference pose -- FIC's controllers use the URDF and the
    simulator the MuJoCo model, so a disagreement makes them fight.
    """
    model = src.model
    data, poses = reference_state(src, info)
    elements = {j.get('name'): j for j in info.root.findall('joint')}
    out = []
    for j in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if jname not in info.joints or int(model.jnt_type[j]) not in (
                mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        child = poses[info.joints[jname].child]
        r = np.array([row[:3] for row in child[:3]])
        origin = np.array([row[3] for row in child[:3]])
        axis_el = elements[jname].find('axis')
        axis = np.array([float(v) for v in (axis_el.get('xyz') if axis_el is not None
                                            else '1 0 0').split()])
        axis = r @ (axis / (np.linalg.norm(axis) or 1.0))
        mj_axis = data.xaxis[j] / (np.linalg.norm(data.xaxis[j]) or 1.0)
        angle = math.degrees(math.acos(min(1.0, abs(float(axis @ mj_axis)))))
        if angle > tol_deg:
            out.append((jname, 'its axis differs by %.0f°' % angle))
            continue
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            d = origin - data.xanchor[j]
            miss = float(np.linalg.norm(d - (d @ mj_axis) * mj_axis))
            if miss > tol_m:
                out.append((jname, 'its axis is %.0f mm away' % (1000 * miss)))
    return out


# ── Writing the adapted copy ──────────────────────────────────────────────

def _copy_asset(src_file, assets_dir, used):
    stem, ext = os.path.splitext(os.path.basename(src_file))
    name, n = stem + ext, 2
    while name.lower() in used:
        name = '%s_%d%s' % (stem, n, ext)
        n += 1
    used.add(name.lower())
    shutil.copy2(src_file, os.path.join(assets_dir, name))
    return name


def write_adapted(src, out_dir, robot_id, driven, efforts, ee_body, home_driven=None):
    """Write ``out_dir/<robot_id>.xml`` (+ ``assets/``): the model, adapted for FIC.

    ``driven`` joint k gets the motor ``torq_j<k+1>`` -- the model's own motor
    on it, renamed, or a new one limited to ``efforts[k]`` in place of a
    servo. A ``force_torque_site`` on ``ee_body`` carries ``force_sensor`` and
    ``torque_sensor`` if the model has none. ``home_driven`` (joint -> value)
    becomes the ``home`` keyframe, with the loops closed. Returns
    (file name, notes).
    """
    notes = []
    spec = mujoco.MjSpec.from_file(src.path)
    model = spec.compile()
    driven_set = set(driven)

    # Motors: FIC commands torque on torq_jk, joint k of the config.
    acts = list(spec.actuators)
    for a, act in enumerate(acts):
        on_driven = (model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
                     and act.target in driven_set)
        if act.name.startswith('torq_j') and not on_driven:
            act.name = 'model_' + act.name          # keep the name torq_jk free
    for k, joint in enumerate(driven):
        mine = [(a, act) for a, act in enumerate(acts)
                if act.name != 'gripper' and model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
                and act.target == joint]
        motor = next((act for a, act in mine
                      if model.actuator_gaintype[a] == mujoco.mjtGain.mjGAIN_FIXED
                      and model.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_NONE), None)
        for a, act in mine:
            if act is not motor:
                notes.append(('info', "Replaced the model's actuator %s on %s with a torque "
                              'motor: FIC commands torque.' % (act.name or '#%d' % a, joint)))
                spec.delete(act)
        name = 'torq_j%d' % (k + 1)
        if motor is not None:
            motor.name = name
        else:
            e = efforts[k]
            spec.add_actuator(name=name, target=joint, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                              gaintype=mujoco.mjtGain.mjGAIN_FIXED,
                              biastype=mujoco.mjtBias.mjBIAS_NONE,
                              gainprm=[1] + [0] * 9, ctrlrange=[-e, e],
                              ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE)

    # Wrench sensors on the end effector.
    sensors = {s.name for s in spec.sensors}
    if 'force_sensor' not in sensors or 'torque_sensor' not in sensors:
        body = spec.body(ee_body)
        if body is None:
            raise MujocoModelError('The end effector %s is not a body in the MuJoCo model.'
                                   % ee_body)
        if spec.site('force_torque_site') is None:
            body.add_site(name='force_torque_site', size=[0.01, 0.01, 0.01],
                          rgba=[1, 0, 0, 1], group=4)
        for sname, stype in (('force_sensor', mujoco.mjtSensor.mjSENS_FORCE),
                             ('torque_sensor', mujoco.mjtSensor.mjSENS_TORQUE)):
            if sname not in sensors:
                spec.add_sensor(name=sname, type=stype, objtype=mujoco.mjtObj.mjOBJ_SITE,
                                objname='force_torque_site')

    # Home keyframe, loops closed.
    from fic_studio.robot_import import mechanism
    residual = mechanism.home_key(spec, home_driven)
    if residual > 1e-4:
        notes.append(('warn', 'The loops do not quite close at the home pose (%.1f mm apart).'
                      % (1000 * residual)))

    # Assets beside it, so the robot folder is self-contained.
    assets = os.path.join(out_dir, 'assets')
    os.makedirs(assets, exist_ok=True)
    used = set()
    base = src.spec.modelfiledir or os.path.dirname(src.path)
    for kind, folder in (('meshes', spec.meshdir), ('textures', spec.texturedir),
                         ('hfields', ''), ('skins', '')):
        for item in getattr(spec, kind):
            f = getattr(item, 'file', '')
            if not f:
                continue
            path = f if os.path.isabs(f) else os.path.join(base, folder or '', f)
            if not os.path.isfile(path):
                path = os.path.join(base, f)
            if os.path.isfile(path):
                item.file = _copy_asset(path, assets, used)
            else:
                notes.append(('warn', 'Asset %s of the MuJoCo model was not found.' % f))
    spec.meshdir = 'assets'
    spec.texturedir = 'assets'
    spec.modelfiledir = out_dir + os.sep        # where to_xml's compile finds them now
    spec.modelname = robot_id
    fname = robot_id + '.xml'
    header = ("<!--\n  %s: the robot's own MuJoCo model (%s), adapted by FIC Control Studio's\n"
              '  robot importer: motors torq_j1..%d on %s, wrench sensors, a home keyframe.\n'
              '-->\n' % (robot_id, os.path.basename(src.path), len(driven), ', '.join(driven)))
    xml = spec.to_xml()
    with open(os.path.join(out_dir, fname), 'w') as fh:
        fh.write(xml.replace('?>\n', '?>\n' + header, 1) if xml.startswith('<?xml')
                 else header + xml)
    return fname, notes
