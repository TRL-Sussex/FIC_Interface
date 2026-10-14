"""Add a robot to the Studio from its URDF, ready to simulate.

Two steps, so a review screen can sit between them:

* ``analyse(source)`` takes a ``.urdf`` / ``.xacro`` file, a folder holding
  one (a robot's ROS description package, say) or a ``.zip`` of that folder,
  and works everything out: the arm joints and their limits, the end effector,
  the elbow, where every mesh is, and what will need fixing up. Nothing is
  written.
* ``create_robot(plan, model_root)`` writes the robot into
  ``menagerie/model/<RobotID>/`` in the same layout as the robots already
  there, builds its MuJoCo model, checks it loads and holds still, and
  returns the robot's config entry. It builds in a staging folder and moves
  it into place at the end, so a failure never leaves half a robot behind.

A URDF is required: the controllers read it. When the robot also comes with
its own MuJoCo model (an ``.xml`` beside the URDF, or given explicitly), that
model becomes the simulation model instead of one built from the URDF, and
its ``<equality><connect>`` constraints give the loops of a closed-loop
robot. Without one, loops come from markup in the URDF or from the review
screen (see ``mechanism``).

Registering the config (both YAML files and config.xml) is the Studio's job:
see ``RobotConfigMixin._rc_register_imported_robot``.
"""
from dataclasses import dataclass, field
import datetime
import math
import os
import re
import shutil
import tempfile
from typing import List, Optional
import xml.etree.ElementTree as ET
import zipfile

from fic_studio.robot_import import check, mechanism, mjcf, mujoco_model
from fic_studio.robot_import.closed_loop import Closure
from fic_studio.robot_import.dae import dae_to_obj, DaeError
from fic_studio.robot_import.meshes import CONVERTIBLE_EXTS, MeshFinder, MUJOCO_EXTS
from fic_studio.robot_import.srdf import best_srdf, SrdfInfo
from fic_studio.robot_import.urdf_info import (
    find_packages, iter_mesh_elements, limits_deg, load_urdf_text, package_dir_of,
    package_roots_for, UrdfError, UrdfInfo,
)

#: Joints the config editor can show (its spinner stops here).
MAX_JOINTS = 12

#: URDF joint types that would let the robot fall off its mount in MuJoCo.
FREE_TYPES = ('floating', 'planar')

#: rad/s written into a URDF <limit> that has none (urdfdom requires one).
DEFAULT_VELOCITY = 2.0

#: FIC's default joint speed limit (rad/s); an imported robot never gets more.
SPEED_LIMIT = 1.0

INFO, WARN, ERROR = 'info', 'warn', 'error'

URDF_REQUIRED = ('A URDF is required: the controllers read it. Drop it together with the '
                 'MuJoCo model, or the folder that holds both; the MuJoCo model is optional.')


class ImportError_(Exception):
    """An import that cannot go ahead; the message is written for the user."""


@dataclass
class JointPlan:
    name: str
    lower_deg: float
    upper_deg: float
    effort: float
    effort_from_urdf: bool
    continuous: bool
    home_deg: float = 0.0
    velocity: Optional[float] = None    # rad/s, from the URDF when it says


@dataclass
class GripperPlan:
    """A sliding-finger gripper the simulator can open and close."""

    driver: str                 # finger joint the "gripper" actuator drives
    followers: List[tuple]      # (joint, multiplier, offset) coupled to the driver
    lower: float                # driver travel (m)
    upper: float
    effort: float               # N

    @property
    def fingers(self):
        return 1 + len(self.followers)

    @property
    def max_width(self):
        """Return the opening (m) with every finger at the end of its travel."""
        return self.fingers * (self.upper - self.lower)


@dataclass
class ImportPlan:
    source: str
    urdf_path: str
    info: UrdfInfo
    robot_id: str
    display_name: str
    ee_link: str
    elbow_link: str
    joints: List[JointPlan]
    extra_joints: List[str]
    meshes: list
    messages: List[tuple] = field(default_factory=list)
    urdf_candidates: List[str] = field(default_factory=list)
    temp_dir: Optional[str] = None
    srdf: Optional[SrdfInfo] = None
    initial_positions: dict = field(default_factory=dict)
    ee_source: str = ''
    home_source: str = ''
    gripper: Optional[GripperPlan] = None
    #: (what, value, where it came from) -- shown on the review screen.
    found: List[tuple] = field(default_factory=list)
    roots: List[str] = field(default_factory=list)
    search_roots: List[str] = field(default_factory=list)
    #: The robot's own MuJoCo model (mujoco_model.SourceModel), if it has one.
    mjcf: Optional[object] = None
    mjcf_notes: List[tuple] = field(default_factory=list)
    #: Loop closures; any at all makes this a closed-loop robot.
    closures: List[Closure] = field(default_factory=list)
    closure_notes: List[tuple] = field(default_factory=list)
    #: Driven joints the user picked (closed loops); None = work them out.
    driven_choice: Optional[List[str]] = None
    #: Closed-loop analysis: see mechanism.analyse, plus ee_body / driven_source.
    mechanism: dict = field(default_factory=dict)
    mechanism_joints: List[str] = field(default_factory=list)
    elbow_guess: str = ''

    @property
    def closed_loop(self):
        return bool(self.closures)

    @property
    def errors(self):
        return [m for level, m in self.messages if level == ERROR]

    @property
    def mesh_counts(self):
        found = sum(1 for m in self.meshes if m.found)
        return found, len(self.meshes)

    def cleanup(self):
        if self.temp_dir:
            shutil.rmtree(self.temp_dir, ignore_errors=True)
            self.temp_dir = None


@dataclass
class CreateResult:
    robot_id: str
    config: dict
    model_dir: str
    report_path: str
    stats: dict
    messages: List[tuple]


# ── Naming ────────────────────────────────────────────────────────────────

def make_robot_id(name, existing=()):
    """Make a config-safe robot ID from a URDF robot name, unique among ``existing``."""
    base = re.sub(r'[^A-Za-z0-9_]+', '_', name).strip('_') or 'Robot'
    if not base[0].isalpha():
        base = 'Robot_' + base
    candidate, n = base, 2
    taken = {e.lower() for e in existing}
    while candidate.lower() in taken:
        candidate = '%s_%d' % (base, n)
        n += 1
    return candidate


def valid_robot_id(robot_id):
    return bool(re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', robot_id or ''))


def _display_name(name):
    words = re.sub(r'[_\-]+', ' ', name).strip()
    return ' '.join(w if w.isupper() else w[:1].upper() + w[1:] for w in words.split()) or name


def _preferred_name(robot_name, urdf_path):
    """Pick the URDF's robot name, or its file name when the robot name is CAD noise.

    Exporters write names like GEN3_7DOF_NOVISION_FOR_URDF_ARM_V12 into
    <robot name>, while the file is usually called something a person chose.
    """
    stem = os.path.splitext(os.path.basename(urdf_path))[0]
    if stem.lower().endswith('.urdf'):          # robot.urdf.xacro
        stem = stem[:-5]
    generic = stem.lower() in ('robot', 'model', 'urdf', 'main', 'description', '')
    if len(robot_name) > 24 and not generic and len(stem) < len(robot_name):
        return stem
    return robot_name


def _safe_file_name(name):
    stem, ext = os.path.splitext(os.path.basename(name))
    stem = re.sub(r'[^A-Za-z0-9_.-]+', '_', stem).strip('_') or 'mesh'
    return stem, ext.lower()


# ── Finding the URDF in what the user gave us ─────────────────────────────

def _urdf_files(folder):
    found = []
    for dirpath, dirnames, filenames in os.walk(folder):
        dirnames[:] = [d for d in dirnames
                       if not d.startswith('.') and d not in ('build', 'install')]
        for f in filenames:
            if f.lower().endswith(('.urdf', '.xacro')):
                found.append(os.path.join(dirpath, f))
    return found


def _rank_urdf(path):
    """Prefer plain .urdf over .xacro, shallow over deep, arm over gripper files."""
    name = os.path.basename(path).lower()
    return (
        name.endswith('.xacro'),
        any(w in name for w in ('gripper', 'hand', 'finger', 'macro', 'common', 'transmission')),
        path.count(os.sep),
        name,
    )


def _prepare_source(source):
    """Return (urdf path, search roots, candidates, temp dir) for ``source``."""
    source = os.path.abspath(os.path.expanduser(source))
    if not os.path.exists(source):
        raise ImportError_('%s does not exist.' % source)
    temp_dir = None
    if os.path.isfile(source) and source.lower().endswith('.zip'):
        temp_dir = tempfile.mkdtemp(prefix='fic_robot_import_')
        try:
            with zipfile.ZipFile(source) as zf:
                for member in zf.namelist():
                    target = os.path.realpath(os.path.join(temp_dir, member))
                    if not target.startswith(os.path.realpath(temp_dir) + os.sep):
                        raise ImportError_('The zip file contains an unsafe path: %s' % member)
                zf.extractall(temp_dir)
        except zipfile.BadZipFile:
            shutil.rmtree(temp_dir, ignore_errors=True)
            raise ImportError_('%s is not a valid zip file.' % os.path.basename(source))
        folder = temp_dir
    elif os.path.isdir(source):
        folder = source
    elif source.lower().endswith(('.urdf', '.xacro')):
        return source, [os.path.dirname(source)], [source], None
    elif mujoco_model.is_mjcf(source):
        raise ImportError_(URDF_REQUIRED)
    else:
        raise ImportError_('Drop a .urdf or .xacro file, a folder that contains one, or a '
                           '.zip of that folder.')
    candidates = sorted(_urdf_files(folder), key=_rank_urdf)
    if not candidates:
        if temp_dir:
            shutil.rmtree(temp_dir, ignore_errors=True)
        raise ImportError_('No .urdf or .xacro file was found in %s. %s'
                           % (os.path.basename(source), URDF_REQUIRED))
    return candidates[0], [folder], candidates, temp_dir


# ── Analyse ───────────────────────────────────────────────────────────────

def analyse(source, existing_ids=(), search_roots=(), urdf_path=None, mjcf_path=None):
    """Work out how ``source`` would be imported. Writes nothing.

    ``mjcf_path`` names the robot's own MuJoCo model; without it one is
    looked for beside the URDF.
    """
    chosen, roots, candidates, temp_dir = _prepare_source(source)
    return _analyse(source, urdf_path or chosen, roots, candidates, temp_dir,
                    existing_ids, search_roots, mjcf_path=mjcf_path)


def reanalyse(plan, urdf_path, existing_ids=()):
    """Analyse another of the description files ``plan`` found in its source.

    The new plan takes over the old one's unpacked zip, if any.
    """
    if urdf_path not in plan.urdf_candidates:
        raise ImportError_('%s is not one of the files found.' % urdf_path)
    new = _analyse(plan.source, urdf_path, plan.roots, plan.urdf_candidates, plan.temp_dir,
                   existing_ids, plan.search_roots, cleanup_on_error=False)
    plan.temp_dir = None
    return new


def _analyse(source, urdf_path, roots, candidates, temp_dir, existing_ids, search_roots,
             cleanup_on_error=True, mjcf_path=None):
    # A single file may sit inside a description package: look beside that
    # package for the ones it includes. A folder or zip is searched deeper.
    if os.path.isfile(source) and not temp_dir:
        packages = find_packages(package_roots_for(urdf_path), max_depth=2)
    else:
        packages = find_packages(roots)
    packages.update({k: v for k, v in find_packages(search_roots).items() if k not in packages})
    try:
        info = UrdfInfo(load_urdf_text(urdf_path, packages), urdf_path)
    except UrdfError as exc:
        if temp_dir and cleanup_on_error:
            shutil.rmtree(temp_dir, ignore_errors=True)
        raise ImportError_(str(exc))
    finder = MeshFinder(urdf_path, list(roots) + list(search_roots), packages)
    name = _preferred_name(info.robot_name, urdf_path)
    plan = ImportPlan(
        source=os.path.abspath(source), urdf_path=urdf_path, info=info,
        robot_id=make_robot_id(name, existing_ids),
        display_name=_display_name(name),
        ee_link='', elbow_link='', joints=[], extra_joints=[],
        meshes=finder.resolve_all(info.mesh_refs),
        urdf_candidates=candidates, temp_dir=temp_dir,
        initial_positions=info.initial_positions(),
        roots=list(roots), search_roots=list(search_roots))
    single = os.path.isfile(source) and not temp_dir
    plan.srdf = best_srdf(_srdf_roots(urdf_path, roots, single) + list(search_roots),
                          info.joints, packages)
    if mjcf_path is None:
        mjcf_path = _find_own_mjcf(info, urdf_path, roots, single)
    _load_mjcf(plan, mjcf_path)
    _find_loops(plan)
    set_end_effector(plan, _default_ee(plan))
    return plan


def _default_ee(plan):
    """Return the end effector to start from, with plan.ee_source saying why."""
    info = plan.info
    if plan.closures:
        plan.ee_source = 'the link the loops hold'
        return mechanism.default_ee(info, plan.closures)
    ee, plan.ee_source = info.guess_ee(), 'guessed: the tool / flange frame after the last joint'
    if plan.srdf:
        tool = plan.srdf.tool_link(info.links, [j.name for j in info.arm_joints(ee)])
        if tool and info.movable_chain_to(tool):
            ee, plan.ee_source = tool, 'from the MoveIt SRDF (%s)' % os.path.basename(
                plan.srdf.path)
    return ee


def _find_own_mjcf(info, urdf_path, roots, single_file):
    """Find a MuJoCo model of this robot beside the URDF, or ''.

    A dropped folder or zip is searched whole; for a lone URDF, its folder
    and the one above (menagerie keeps URDF/ and xml/ side by side). It must
    share at least half the URDF's moving joints, so a scene of some other
    robot lying about is not taken for this one.
    """
    here = os.path.dirname(os.path.abspath(urdf_path))
    search = list(roots) if not single_file else [here, os.path.dirname(here)]
    moving = [j.name for j in info.joints.values() if j.movable]
    path = mujoco_model.find_mjcf(search, moving, max_depth=5 if not single_file else 3)
    if not path:
        return ''
    names = check.mjcf_names(path) or {'joint': set()}
    return path if len(set(names['joint']) & set(moving)) * 2 >= max(1, len(moving)) else ''


def _load_mjcf(plan, path):
    """Use the MuJoCo model at ``path`` (or none, for '') for simulation."""
    plan.mjcf, plan.mjcf_notes = None, []
    if not path:
        return
    try:
        plan.mjcf = mujoco_model.load(path)
    except mujoco_model.MujocoModelError as exc:
        plan.mjcf_notes.append((WARN, '%s A model is built from the URDF instead.' % exc))
        return
    bad = mujoco_model.compare(plan.mjcf, plan.info)
    if bad:
        plan.mjcf_notes.append((WARN, 'The MuJoCo model and the URDF disagree about %s. The '
                                'controllers use the URDF and the simulator the MuJoCo model, '
                                'so they would fight: fix whichever is wrong.'
                                % '; '.join('%s (%s)' % b for b in bad[:3])))


def _find_loops(plan):
    """Loops from the robot's MuJoCo model, else from markup in the URDF."""
    if plan.mjcf is not None:
        plan.closures, plan.closure_notes = mujoco_model.closures(plan.mjcf, plan.info)
    else:
        plan.closures, plan.closure_notes = mechanism.urdf_closures(plan.info)


def set_mjcf(plan, path):
    """Use another MuJoCo model (or none, for ''); loops and joints follow."""
    _load_mjcf(plan, path)
    plan.driven_choice = None
    _find_loops(plan)
    set_end_effector(plan, _default_ee(plan))
    return plan


def set_closures(plan, closures):
    """Replace the loops (the review screen's editor); joints follow."""
    was_closed = plan.closed_loop
    plan.closures = list(closures)
    plan.driven_choice = None
    ee = plan.ee_link if plan.closed_loop == was_closed else _default_ee(plan)
    set_end_effector(plan, ee or _default_ee(plan))
    return plan


def set_driven(plan, names):
    """Drive these joints of a closed-loop robot (the review screen's ticks)."""
    plan.driven_choice = list(names)
    set_end_effector(plan, plan.ee_link)
    return plan


def _srdf_roots(urdf_path, roots, single_file):
    """Where a MoveIt SRDF for this robot would be.

    A dropped folder or zip is searched as a whole. For a single file in a
    description package, the sibling packages named like it are (franka_
    description's SRDF is in franka_moveit_config beside it), not the whole
    parent folder, which may be all of /opt/ros/<distro>/share.
    """
    if not single_file:
        return list(roots)
    pkg = package_dir_of(urdf_path)
    if not pkg:
        return []
    parent = os.path.dirname(pkg)
    stem = os.path.basename(pkg).split('_')[0]
    try:
        names = os.listdir(parent)
    except OSError:
        return [pkg]
    return [os.path.join(parent, d) for d in sorted(names)
            if d.startswith(stem) and os.path.isdir(os.path.join(parent, d))]


def _auto_home(plan, arm):
    """{joint: rad} for a home pose the robot's own files suggest, and where from."""
    if plan.mjcf is not None and plan.mjcf.home:
        own = {j.name: plan.mjcf.home[j.name] for j in arm if j.name in plan.mjcf.home}
        if any(abs(v) > 1e-9 for v in own.values()):
            return own, "the MuJoCo model's home keyframe"
    if plan.srdf:
        state = plan.srdf.home_state([j.name for j in arm])
        if state:
            return state[1], 'the MoveIt SRDF pose "%s"' % state[0]
    initial = {j.name: plan.initial_positions[j.name] for j in arm
               if j.name in plan.initial_positions}
    if initial and any(abs(v) > 1e-9 for v in initial.values()):
        return initial, "the initial positions in the URDF's ros2_control block"
    return {}, ''


def _detect_gripper(info, extra_names):
    """Return a GripperPlan for sliding fingers among ``extra_names``, else None."""
    fingers = [info.joints[n] for n in extra_names
               if info.joints[n].type == 'prismatic' and info.is_gripper_joint(info.joints[n])]
    leaders = [j for j in fingers if not j.mimic]
    if not leaders or len(leaders) > 2:
        return None
    driver = leaders[0]
    if driver.lower is None or driver.upper is None or driver.upper <= driver.lower:
        return None
    followers = []
    for j in fingers:
        if j is driver:
            continue
        if j.mimic and j.mimic[0] == driver.name:
            followers.append((j.name, j.mimic[1], j.mimic[2]))
        elif not j.mimic:
            followers.append((j.name, 1.0, 0.0))     # a symmetric pair, unlinked
        else:
            return None
    return GripperPlan(driver=driver.name, followers=followers, lower=driver.lower,
                       upper=driver.upper,
                       effort=driver.effort if driver.effort and driver.effort > 0 else 20.0)


def set_end_effector(plan, ee_link):
    """Point the arm at ``ee_link``; recompute joints and notes. Keeps homes by name."""
    info = plan.info
    if ee_link not in info.links:
        raise ImportError_('%r is not a link in this URDF.' % ee_link)
    old_home = {j.name: j.home_deg for j in plan.joints}
    previous_guess = plan.elbow_guess
    plan.ee_link = ee_link
    if plan.closures:
        driven = _closed_loop_joints(plan)
        guess = mechanism.elbow_for(info, [j.name for j in driven])
    else:
        driven = info.arm_joints(ee_link)
        plan.mechanism, plan.mechanism_joints = {}, []
        guess = info.guess_elbow(ee_link)
    suggested, plan.home_source = _auto_home(plan, driven)
    efforts = {a.joint: a.effort for a in plan.mjcf.actuators if a.effort} if plan.mjcf else {}
    plan.joints = []
    for j in driven:
        lo, hi = limits_deg(j)
        if j.name in old_home:
            home = old_home[j.name]
        elif j.name in suggested:
            home = min(max(math.degrees(suggested[j.name]), lo), hi)
        else:
            home = 0.0 if lo <= 0.0 <= hi else (lo + hi) / 2.0
        effort = efforts.get(j.name) or (j.effort if j.effort and j.effort > 0 else None)
        plan.joints.append(JointPlan(
            name=j.name, lower_deg=lo, upper_deg=hi,
            effort=effort or mjcf.DEFAULT_EFFORT, effort_from_urdf=bool(effort),
            continuous=j.continuous, home_deg=round(home, 3),
            velocity=j.velocity if j.velocity and j.velocity > 0 else None))
    keep = set(plan.mechanism_joints) | {j.name for j in driven}
    plan.extra_joints = [j.name for j in info.joints.values()
                         if j.movable and j.name not in keep]
    # The robot's own MuJoCo model simulates its own gripper, if any.
    plan.gripper = None if plan.mjcf else _detect_gripper(info, plan.extra_joints)
    if not plan.elbow_link or plan.elbow_link == previous_guess:
        plan.elbow_link = guess
    plan.elbow_guess = guess
    _review(plan)
    return plan


def _closed_loop_joints(plan):
    """Analyse the mechanism; return the driven joints (JointInfo, config order)."""
    info = plan.info
    mech = mechanism.mechanism_joints(info, plan.closures, plan.ee_link)
    plan.mechanism_joints = mech
    one_dof = [n for n in mech if info.joints[n].movable]
    actuated = [a.joint for a in plan.mjcf.actuators if a.joint in one_dof] if plan.mjcf else []
    choice = [n for n in (plan.driven_choice or []) if n in one_dof]
    try:
        if plan.mjcf is not None:
            model = plan.mjcf.model
            ee_body = mechanism.body_for_link(plan.mjcf, info, plan.ee_link)
            if not ee_body:
                raise ImportError_('the end effector %s has no body in the MuJoCo model'
                                   % plan.ee_link)
        else:
            model = mechanism.kinematic_model(info, plan.closures)
            ee_body = plan.ee_link
        result = mechanism.analyse(model, ee_body, one_dof, driven=choice or actuated or None)
        result['ee_body'] = ee_body
        result['driven_source'] = (
            'picked by you' if choice else
            "the MuJoCo model's actuators" if actuated else
            'picked: the joints nearest the base, until the end effector is held')
    except Exception as exc:           # a model MuJoCo cannot build is shown, not fatal
        result = {'error': str(exc), 'driven': choice or actuated}
    plan.mechanism = result
    return [info.joints[n] for n in result.get('driven', []) if n in info.joints]


def speed_limit(plan):
    """Return the joint speed limit (rad/s) for the config.

    FIC's gentle 1 rad/s, or the URDF's slowest joint if that is slower still.
    """
    speeds = [j.velocity for j in plan.joints if j.velocity]
    return round(min([SPEED_LIMIT] + speeds), 3)


def workspace_box(plan):
    """Return a box (x, y, z ranges in m) around everything the arm can reach."""
    reach = math.ceil((plan.info.reach(plan.ee_link) + 0.05) * 10.0) / 10.0
    return {'x': (-reach, reach), 'y': (-reach, reach), 'z': (0.0, reach + 0.1)}


def joint_safety(plan, margin_deg=5.0):
    """Joint safety limits just inside the joint limits (continuous joints: ±360)."""
    lower, upper = [], []
    for j in plan.joints:
        lo, hi = j.lower_deg, j.upper_deg
        if not j.continuous and hi - lo > 4 * margin_deg:
            lo, hi = lo + margin_deg, hi - margin_deg
        lower.append(round(lo, 2))
        upper.append(round(hi, 2))
    return lower, upper


def _what_was_found(plan):
    """Return (what, value, source) rows for the review screen."""
    found = []
    n = len(plan.joints)
    speeds = [j.velocity for j in plan.joints if j.velocity]
    found.append(('Joints', '%d: %s' % (n, ', '.join(j.name for j in plan.joints)),
                  'names, limits and torques%s from the URDF'
                  % (', speeds' if speeds else '')))
    if plan.mjcf is not None:
        found.append(('Simulation', os.path.basename(plan.mjcf.path),
                      "your MuJoCo model, used as it is, with FIC's motors and sensors added"))
    if plan.closures:
        sources = sorted({c.source for c in plan.closures if c.source})
        found.append(('Loops', ', '.join('%s–%s' % (c.body_a, c.body_b) for c in plan.closures),
                      'from ' + ' and '.join(sources) if sources else ''))
        m = plan.mechanism
        if 'ee_dof' in m:
            found.append(('Mechanism', '%d DOF at %s%s' % (
                m['ee_dof'], plan.ee_link, ', turns only' if m.get('orientation_only') else ''),
                'worked out from the loops'))
        found.append(('Driven', ', '.join(j.name for j in plan.joints),
                      m.get('driven_source', '')))
    found.append(('End effector', plan.ee_link, plan.ee_source))
    if plan.elbow_link:
        found.append(('Elbow', plan.elbow_link, 'the link after the middle joint'))
    found.append(('Home pose', ', '.join('%.0f°' % j.home_deg for j in plan.joints),
                  plan.home_source or 'no pose in the files: joints at 0° (or mid-range)'))
    found.append(('Start target', "the end effector's pose at home",
                  'computed by the simulator from the home pose'))
    if plan.gripper:
        g = plan.gripper
        found.append(('Gripper', '%d finger%s, opening 0–%.3f m' % (
            g.fingers, '' if g.fingers == 1 else 's', g.max_width),
            '%s travel %.3f m; driven in simulation' % (g.driver, g.upper - g.lower)))
    limit = speed_limit(plan)
    found.append(('Speed limit', '%.2f rad/s' % limit,
                  ("the URDF allows up to %.2f rad/s; %.1f rad/s is FIC's gentle default"
                   % (min(speeds), SPEED_LIMIT)) if speeds and min(speeds) > limit
                  else ('the slowest joint in the URDF' if speeds else "FIC's default")))
    box = workspace_box(plan)
    found.append(('Safety', 'joint limits −5°, workspace box ±%.1f m' % box['x'][1],
                  "filled in from the joint limits and the arm's reach, switched off "
                  'until you enable them'))
    return found


def _loop_messages(plan):
    """Say what the review screen tells about a closed-loop robot's mechanism."""
    m = plan.mechanism
    if m.get('error'):
        return [(ERROR, 'The loops could not be analysed: %s.' % m['error'])]
    out = []
    n, dof = len(plan.joints), m.get('ee_dof', 0)
    if m.get('residual', 0.0) > 1e-3:
        out.append((WARN, 'The loops are %.1f mm open at the reference pose: check the loop '
                    'points.' % (1000 * m['residual'])))
    if not n:
        out.append((ERROR, 'No joint is driven: tick the joints with motors.'))
    elif not m.get('determined'):
        out.append((ERROR, 'With %s driven, %s can still move. Drive more joints (the ones '
                    'nearest the base are best).' % (', '.join(j.name for j in plan.joints),
                                                     plan.ee_link)))
    else:
        out.append((INFO, 'Closed-loop mechanism: %d loop%s; %s moves with %d degree%s of '
                    'freedom%s, held by %d driven joint%s.'
                    % (len(plan.closures), '' if len(plan.closures) == 1 else 's',
                       plan.ee_link, dof, '' if dof == 1 else 's',
                       ' (it only turns)' if m.get('orientation_only') else '',
                       n, '' if n == 1 else 's')))
    if n > dof and m.get('determined'):
        out.append((INFO, '%d driven joints for %d degrees of freedom: a redundant robot.'
                    % (n, dof)))
    return out


def _review(plan):
    """Fill ``plan.messages`` with what the user should know before importing."""
    msgs = []
    info = plan.info
    n = len(plan.joints)
    if n == 0:
        msgs.append((ERROR, 'No moving joints lead to %s. Pick the link the arm ends at.'
                     % plan.ee_link))
    elif n > MAX_JOINTS:
        msgs.append((ERROR, 'The arm has %d joints; the Studio handles up to %d.'
                     % (n, MAX_JOINTS)))
    sliding = [j.name for j in plan.joints if info.joints[j.name].type == 'prismatic']
    if sliding:
        msgs.append((ERROR, '%s slide%s (prismatic). The simulator and controllers drive '
                     'rotating joints only.' % (', '.join(sliding),
                                                's' if len(sliding) == 1 else '')))
    msgs += plan.mjcf_notes + plan.closure_notes
    all_moving = {j.name for j in info.joints.values() if j.movable}
    for j in info.joints.values():
        if j.type not in FREE_TYPES or j.name in plan.mechanism_joints:
            continue
        below = {info.parent_joint[link].name for link in info.descendants(j.child)} & all_moving
        if below == all_moving and len(info.child_joints.get(j.parent, [])) == 1:
            msgs.append((WARN, '%s is a %s joint (a mobile base); it will be fixed so the '
                         'robot stays mounted.' % (j.name, j.type)))
        elif not plan.closures:
            # A free body beside the arm is how a closed loop (a parallel
            # mechanism such as the CSPM) is written as a URDF tree.
            msgs.append((ERROR, '%s is a %s joint beside the arm: this looks like a '
                         "closed-loop mechanism, but no loops were found. Drop the robot's "
                         'MuJoCo model with the URDF, or add the loops below.'
                         % (j.name, j.type)))
    if plan.closures:
        msgs += _loop_messages(plan)
    driven = set()
    if plan.gripper:
        driven = {plan.gripper.driver} | {f[0] for f in plan.gripper.followers}
    held = [n for n in plan.extra_joints if n not in driven]
    gripper_held = [n for n in held if info.is_gripper_joint(info.joints[n])]
    if gripper_held and not plan.gripper:
        names = ', '.join(gripper_held[:3]) + ('…' if len(gripper_held) > 3 else '')
        msgs.append((INFO, 'The gripper (%s) is held fixed in simulation: only grippers with '
                     'sliding fingers are driven.' % names))
    others = [n for n in held if n not in gripper_held]
    if others:
        msgs.append((WARN, '%d joint%s off the arm (%s) will be held fixed: the simulator '
                     'only drives the arm.' % (len(others), '' if len(others) == 1 else 's',
                                               ', '.join(others[:4])
                                               + ('…' if len(others) > 4 else ''))))
    defaulted = [j.name for j in plan.joints if not j.effort_from_urdf]
    if defaulted:
        msgs.append((WARN, 'No effort limit in the URDF for %s — using %g Nm. Set the real '
                     'torque limits before using hardware.'
                     % (', '.join(defaulted[:4]), mjcf.DEFAULT_EFFORT)))
    moving = {j.child for j in info.joints.values() if j.movable}
    moving |= {d for c in list(moving) for d in info.descendants(c)}
    massless = info.massless_links(moving)
    if massless:
        msgs.append((WARN, '%s ha%s no mass in the URDF; MuJoCo will estimate it from the '
                     'mesh. Real inertial values make the simulation match the robot.'
                     % (', '.join(massless[:4]), 's' if len(massless) == 1 else 've')))

    # A missing visual mesh costs nothing when the link's collision mesh is
    # there: MuJoCo draws that instead. Anything else leaves a part out.
    has_collision = {m.ref.link for m in plan.meshes if m.found and m.ref.role == 'collision'}
    missing = [m for m in plan.meshes if not m.found]
    covered = sorted({os.path.basename(m.ref.uri) for m in missing
                      if m.ref.role == 'visual' and m.ref.link in has_collision})
    lost = sorted({os.path.basename(m.ref.uri) for m in missing} - set(covered))

    def _names(names):
        return ', '.join(names[:3]) + ('…' if len(names) > 3 else '')
    if lost:
        msgs.append((WARN, '%d mesh file%s not found (%s). Those parts will be left out; '
                     'drop the folder holding them to include them.'
                     % (len(lost), '' if len(lost) == 1 else 's', _names(lost))))
    if covered:
        msgs.append((INFO, '%d display mesh%s not found (%s); those links are drawn with '
                     'their collision meshes instead.'
                     % (len(covered), '' if len(covered) == 1 else 'es', _names(covered))))
    dae = [m for m in plan.meshes if m.found and m.ext in CONVERTIBLE_EXTS]
    if dae:
        msgs.append((INFO, '%d .dae mesh%s will be converted for MuJoCo, which cannot read '
                     '.dae.' % (len(dae), '' if len(dae) == 1 else 'es')))
    odd = sorted({m.ext for m in plan.meshes if m.found
                  and m.ext not in MUJOCO_EXTS + CONVERTIBLE_EXTS})
    if odd:
        msgs.append((WARN, 'Meshes of type %s cannot be used in MuJoCo and will be left out; '
                     'export them as .stl or .obj.' % ', '.join(odd)))
    if info.source_path.lower().endswith('.xacro'):
        msgs.append((INFO, 'Expanded from xacro.'))
    if len(plan.urdf_candidates) > 1:
        msgs.append((INFO, 'The folder holds %d robot description files; using %s. Pick '
                     'another at the top if that is not the robot you mean.'
                     % (len(plan.urdf_candidates), os.path.basename(plan.urdf_path))))
    plan.messages = msgs
    plan.found = _what_was_found(plan) if plan.joints else []


# ── Create ────────────────────────────────────────────────────────────────

def _unique(name, used):
    """File name for (stem, ext) whose stem is not in ``used``.

    Stems, not whole names, must differ: MuJoCo names each mesh after its
    file's stem, so link.stl and link.obj would clash.
    """
    stem, ext = name
    candidate, n = stem, 2
    while candidate.lower() in used:
        candidate = '%s_%d' % (stem, n)
        n += 1
    used.add(candidate.lower())
    return candidate + ext


def _number(text):
    try:
        return math.isfinite(float(text))
    except (TypeError, ValueError):
        return False


def _fill_limits(urdf_root, plan):
    """Make the controllers' URDF one they can load and index; return what changed.

    * urdfdom (under Pinocchio, which the controllers load the URDF with)
      rejects a moving joint whose <limit> lacks effort or velocity, though
      MuJoCo does not mind. Those are filled in.
    * A ``continuous`` joint is written as a ``revolute`` one with no lower /
      upper, FIC's convention (KinovaGen3_7DOF.urdf does the same): FIC reads
      it as continuous, and Pinocchio gives it one position coordinate
      instead of a cos/sin pair -- the controllers index position limits by
      joint, so a pair shifts every later joint's limits by one.

    Returns (joints given limits, continuous joints rewritten).
    """
    efforts = {j.name: j.effort for j in plan.joints}
    filled, rewritten = [], []
    for j in urdf_root.findall('joint'):
        name = j.get('name')
        if j.get('type') == 'continuous':
            j.set('type', 'revolute')
            limit = j.find('limit')
            if limit is None:
                limit = ET.SubElement(j, 'limit')
            for bound in ('lower', 'upper'):
                limit.attrib.pop(bound, None)
            rewritten.append(name)
        if j.get('type') not in ('revolute', 'prismatic'):
            continue
        limit = j.find('limit')
        if limit is None:
            # No bounds either: FIC reads that as continuous, as we do.
            limit = ET.SubElement(j, 'limit')
        changed = False
        if not _number(limit.get('effort')):
            limit.set('effort', '%g' % efforts.get(name, mjcf.DEFAULT_EFFORT))
            changed = True
        if not _number(limit.get('velocity')):
            limit.set('velocity', '%g' % DEFAULT_VELOCITY)
            changed = True
        if changed:
            filled.append(name)
    return filled, rewritten


def _controllers_can_load(urdf_path, ee_link):
    """None when Pinocchio loads the URDF and finds the EE frame, else why not.

    Returns None as well when Pinocchio's Python bindings are not installed:
    the check is a courtesy, the controllers have their own copy.
    """
    try:
        import pinocchio
    except ImportError:
        return None
    try:
        model = pinocchio.buildModelFromUrdf(urdf_path)
    except Exception as exc:          # urdfdom prints the detail on stderr
        return str(exc)
    if not model.existFrame(ee_link):
        return 'Pinocchio finds no frame named %s' % ee_link
    return None


def _drop(parent_map, element):
    """Remove the <visual>/<collision> that owns ``element`` (a <mesh>)."""
    geometry = parent_map.get(element)
    owner = parent_map.get(geometry)
    link = parent_map.get(owner)
    if owner is not None and link is not None:
        link.remove(owner)


def create_robot(plan, model_root, overwrite=False, progress=None):
    """Write the robot into ``model_root/<robot_id>`` and return its config."""
    say = progress or (lambda text: None)
    if plan.errors:
        raise ImportError_(plan.errors[0])
    if not valid_robot_id(plan.robot_id):
        raise ImportError_('The robot ID must start with a letter and use only letters, '
                           'digits and _ (got %r).' % plan.robot_id)
    target = os.path.join(model_root, plan.robot_id)
    if os.path.exists(target) and not overwrite:
        raise ImportError_('menagerie/model/%s already exists. Choose another robot ID.'
                           % plan.robot_id)

    rid = plan.robot_id
    messages = []
    staging = tempfile.mkdtemp(prefix='.import_%s_' % rid, dir=model_root)
    try:
        urdf_dir = os.path.join(staging, 'URDF')
        mesh_dir = os.path.join(urdf_dir, 'meshes')
        xml_dir = os.path.join(staging, 'xml')
        assets_dir = os.path.join(xml_dir, 'assets')
        for d in (mesh_dir, assets_dir):
            os.makedirs(d)

        # 1. Meshes: originals beside the URDF, MuJoCo-readable copies in assets/.
        say('Copying meshes')
        # Joints off the arm are fixed: in the controllers' URDF so its model
        # has exactly the configured joints, in MuJoCo so nothing flops about
        # -- except a gripper's fingers there, which the simulator drives.
        # A closed loop's free bodies (the CSPM's platform) keep their
        # floating joint: in the URDF for the controllers' solver, in MuJoCo
        # as a free joint at the top level and as three slides and a ball
        # further down (added once the model is built; MuJoCo frees only
        # top-level bodies).
        extra = set(plan.extra_joints)
        fingers = set()
        if plan.gripper:
            fingers = {plan.gripper.driver} | {f[0] for f in plan.gripper.followers}
        mech = set(plan.mechanism_joints)
        inner_free = [j for j in mechanism.floating_parents(plan.info) if j.name in mech]
        saved_urdf = ET.fromstring(ET.tostring(plan.info.root))
        mujoco_urdf = ET.fromstring(ET.tostring(plan.info.root))
        for tree, keep in ((saved_urdf, set()), (mujoco_urdf, fingers)):
            for j in tree.findall('joint'):
                name = j.get('name')
                free = j.get('type') in FREE_TYPES
                if (name in extra and name not in keep) or (free and name not in mech) or \
                        (tree is mujoco_urdf and name in {f.name for f in inner_free}):
                    j.set('type', 'fixed')
        saved_meshes = [el for _, _, el in iter_mesh_elements(saved_urdf)]
        mj_meshes = [el for _, _, el in iter_mesh_elements(mujoco_urdf)]
        mj_parent = {c: p for p in mujoco_urdf.iter() for c in p}
        used_orig, used_assets, originals, assets = set(), set(), {}, {}
        collision_exts = {}
        for m in plan.meshes:
            if m.found and m.ref.role == 'collision':
                collision_exts.setdefault(m.ref.link, set()).add(m.ext)
        for m, s_el, mj_el in zip(plan.meshes, saved_meshes, mj_meshes):
            if not m.found:
                _drop(mj_parent, mj_el)
                continue
            if m.path not in originals:
                originals[m.path] = _unique(_safe_file_name(m.path), used_orig)
                shutil.copy2(m.path, os.path.join(mesh_dir, originals[m.path]))
            s_el.set('filename', 'meshes/' + originals[m.path])
            ext = m.ext
            if ext in MUJOCO_EXTS:
                if m.path not in assets:
                    assets[m.path] = _unique(_safe_file_name(m.path), used_assets)
                    shutil.copy2(m.path, os.path.join(assets_dir, assets[m.path]))
                mj_el.set('filename', assets[m.path])
            elif ext in CONVERTIBLE_EXTS:
                # A drawn .dae beside a readable collision mesh adds nothing
                # MuJoCo needs; skip the conversion and draw the collision mesh.
                drawn = collision_exts.get(m.ref.link, set()) & set(MUJOCO_EXTS)
                if m.ref.role == 'visual' and drawn:
                    _drop(mj_parent, mj_el)
                    continue
                if m.path not in assets:
                    stem, _ = _safe_file_name(m.path)
                    name = _unique((stem, '.obj'), used_assets)
                    try:
                        dae_to_obj(m.path, os.path.join(assets_dir, name))
                        assets[m.path] = name
                    except (DaeError, ValueError, IndexError) as exc:
                        messages.append((WARN, 'Could not convert %s (%s); that part is '
                                         'left out.' % (os.path.basename(m.path), exc)))
                        assets[m.path] = None
                if assets[m.path]:
                    mj_el.set('filename', assets[m.path])
                else:
                    _drop(mj_parent, mj_el)
            else:
                _drop(mj_parent, mj_el)

        # 2. The URDF the controllers read, and an untouched copy for reference.
        say('Writing URDF')
        filled, rewritten = _fill_limits(saved_urdf, plan)
        if filled:
            messages.append((INFO, 'Added the effort/velocity limits the controllers need to '
                             '%s in the saved URDF.' % ', '.join(filled)))
        if rewritten:
            messages.append((INFO, 'Continuous joint%s %s %s written as revolute without '
                             "bounds in the controllers' URDF, the way FIC marks a "
                             'continuous joint.' % ('' if len(rewritten) == 1 else 's',
                                                    ', '.join(rewritten),
                                                    'is' if len(rewritten) == 1 else 'are')))
        saved_tree = ET.ElementTree(saved_urdf)
        ET.indent(saved_tree, space='  ')
        saved_path = os.path.join(urdf_dir, rid + '.urdf')
        saved_tree.write(saved_path, encoding='utf-8', xml_declaration=True)
        problem = _controllers_can_load(saved_path, plan.ee_link)
        if problem:
            messages.append((WARN, 'The controllers may not be able to load this URDF '
                             '(Pinocchio: %s). The simulation will run, but Cartesian '
                             'control needs it fixed.' % problem))
        with open(os.path.join(urdf_dir, rid + '.original.urdf'), 'w') as fh:
            fh.write(plan.info.text)

        # 3. MuJoCo: the robot's own model, adapted; else one built from the URDF.
        arm = [j.name for j in plan.joints]
        home_rad = [math.radians(j.home_deg) for j in plan.joints]
        ee_offset = None
        if plan.mjcf is not None:
            say('Adapting your MuJoCo model')
            ee_body = mechanism.body_for_link(plan.mjcf, plan.info, plan.ee_link)
            if not ee_body:
                raise ImportError_('The end effector %s has no body in the MuJoCo model.'
                                   % plan.ee_link)
            robot_file, notes = mujoco_model.write_adapted(
                plan.mjcf, xml_dir, rid, arm, [j.effort for j in plan.joints], ee_body,
                dict(zip(arm, home_rad)))
            messages += notes
            ee_offset = mechanism.link_offset(plan.mjcf, plan.info, plan.ee_link, ee_body)
            floor = not plan.mjcf.has_floor
        else:
            say('Building the MuJoCo model')
            robot_file, ee_body, floor = rid + '.xml', plan.ee_link, True
            _build_from_urdf(plan, mujoco_urdf, xml_dir, rid, arm, home_rad, inner_free)

        # 4. Scene, then prove the whole thing loads and behaves.
        say('Checking the model')
        probe = mjcf.write_scene(xml_dir, robot_file, rid, (0.0, 0.0, 0.3), 1.0, floor=floor)
        stats = mjcf.validate(probe, arm, ee_body, home_rad, ee_offset=ee_offset)
        ee = stats['ee_pos']
        mjcf.write_scene(xml_dir, robot_file, rid,
                         (ee[0] / 2.0, ee[1] / 2.0, max(ee[2], 0.2) / 2.0),
                         1.5 * max(stats['reach_m'], 0.3), floor=floor)
        if not stats['finite']:
            raise ImportError_('The model became unstable in simulation. Check the joint '
                               'axes and link masses in the URDF.')
        if not plan.closures and stats['reach_m'] > 0.97 * plan.info.reach(plan.ee_link) > 0:
            messages.append((WARN, 'At the home pose the arm is stretched straight out, a '
                             'singularity: Cartesian control cannot move it further out '
                             'from there and may jerk near it. Bend a joint or two in the '
                             'home pose (Joints tab), then Set from home pose (Frames tab).'))
        if stats['hold_drift_deg'] > 2.0:
            messages.append((WARN, 'With gravity compensated the arm still drifted %.1f° in '
                             'half a second: check link masses and inertias in the URDF.'
                             % stats['hold_drift_deg']))

        config = robot_config(plan, stats)
        report = os.path.join(staging, 'import_report.md')
        _write_report(report, plan, stats, config, messages)

        say('Saving')
        if os.path.exists(target):
            shutil.rmtree(target)
        os.rename(staging, target)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return CreateResult(robot_id=rid, config=config, model_dir=target,
                        report_path=os.path.join(target, 'import_report.md'),
                        stats=stats, messages=plan.messages + messages)


def _build_from_urdf(plan, mujoco_urdf, xml_dir, rid, arm, home_rad, inner_free):
    """Build the MuJoCo model from the URDF, with FIC's motors, sensors, damping and loops."""
    import mujoco
    for old in mujoco_urdf.findall('mujoco'):
        mujoco_urdf.remove(old)
    block = ET.SubElement(mujoco_urdf, 'mujoco')
    ET.SubElement(block, 'compiler', meshdir='assets', balanceinertia='true',
                  discardvisual='false', fusestatic='false', strippath='true',
                  inertiafromgeom='auto', boundmass='0.001', boundinertia='1e-6',
                  angle='radian')
    import_urdf = os.path.join(xml_dir, '_import.urdf')
    ET.ElementTree(mujoco_urdf).write(import_urdf, encoding='utf-8')
    model = mjcf.compile_urdf(import_urdf)
    raw = os.path.join(xml_dir, '_raw.xml')
    mujoco.mj_saveLastXML(raw, model)
    out = os.path.join(xml_dir, rid + '.xml')
    mjcf.build_robot_xml(raw, out, model_name=rid, arm_joints=arm,
                         efforts=[j.effort for j in plan.joints], ee_body=plan.ee_link,
                         home_qpos=mjcf.home_qpos(model, arm, home_rad), gripper=plan.gripper)
    for tmp in (import_urdf, raw):
        os.remove(tmp)
    if not plan.closures:
        return
    # Close the loops: sites and connect constraints, the inner free bodies'
    # joints, passive joints damped like the driven ones, an assembled home.
    spec = mujoco.MjSpec.from_file(out)
    mechanism.free_joints(spec, inner_free)
    mechanism.add_loops(spec, plan.closures)
    passive = min(mjcf.joint_damping(j.effort) for j in plan.joints) if plan.joints else 0.1
    for name in plan.mechanism_joints:
        joint = spec.joint(name)
        if name not in arm and joint is not None and \
                joint.type in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            joint.damping = [passive, 0.0, 0.0]       # MuJoCo 3.10: a 3-term polynomial
    header = open(out).read().split('<mujoco', 1)[0]
    mechanism.home_key(spec, dict(zip(arm, home_rad)))
    spec.modelfiledir = xml_dir + os.sep
    with open(out, 'w') as fh:
        fh.write(header + spec.to_xml().split('?>', 1)[-1].lstrip())


# ── Config entry ──────────────────────────────────────────────────────────

def robot_config(plan, stats):
    """Build the robots_config.yaml entry for an imported robot."""
    rid = plan.robot_id
    n = len(plan.joints)
    home_deg = [round(j.home_deg, 4) for j in plan.joints]
    quat = stats['ee_quat_xyzw']
    if quat[3] < 0:                     # same rotation, easier to read
        quat = [-v for v in quat]
    safety_lo, safety_hi = joint_safety(plan)
    # A gripper: sliding fingers found in the URDF, or the robot's own
    # MuJoCo model's "gripper" actuator (sized for the bridge's 0-0.085 m).
    own_gripper = plan.mjcf is not None and plan.mjcf.gripper is not None
    has_gripper = plan.gripper is not None or own_gripper
    gripper_width = round(plan.gripper.max_width, 4) if plan.gripper else \
        (0.085 if own_gripper else 0.08)
    config = {
        'display_name': plan.display_name or rid,
        'num_joints': n,
        'joint_names': [j.name for j in plan.joints],
        'urdf_path': 'menagerie/model/%s/URDF/%s.urdf' % (rid, rid),
        'mujoco_xml_path': 'menagerie/model/%s/xml/scene.xml' % rid,
        'ee_frame_name': plan.ee_link,
        'elbow_frame_name': plan.elbow_link,
        'kinematics_solver': 'pinocchio',
        'default_ee_target': {
            'position': [round(v, 4) + 0.0 for v in stats['ee_pos']],
            'orientation': [round(v, 4) + 0.0 for v in quat],
        },
        'q_home_deg': home_deg,
        'q_home_rad': [math.radians(d) for d in home_deg],
        'joint_limits_deg': {
            'lower': [round(j.lower_deg, 3) for j in plan.joints],
            'upper': [round(j.upper_deg, 3) for j in plan.joints],
        },
        'joint_torque_limits': {'min': [0.0] * n, 'max': [float(j.effort) for j in plan.joints]},
        'torque_sign': 1,
        'gravity_compensated': False,
        'wrench_gravity_compensated': False,
        'wrench_from_api': False,
        'admittance_deadbands': {'joint_tau': 1.5, 'cart_force': 4.0, 'cart_torque': 0.2},
        'has_gripper': has_gripper,
        'gripper_range': [0.0, gripper_width],
        'is_supported': False,
        'driver_command': '',
        'build_directory': '',
        'build_command': '',
        'robot_category': 'arm',
        # Filled in from the URDF, and switched off: limits that stop the
        # robot are the user's call. The speed limit is on, as for every
        # robot, and never faster than FIC's default.
        'safety_zones': {
            'cartesian': {axis: {'min': lo, 'max': hi, 'enabled': False}
                          for axis, (lo, hi) in workspace_box(plan).items()},
            'joint_safety_deg': {'lower': safety_lo, 'upper': safety_hi, 'enabled': False},
            'velocity': {'max_joint_vel': speed_limit(plan), 'max_cart_vel': 0.5,
                         'enabled': True},
        },
    }
    if plan.closures:
        # The controllers' closed-loop solver: the full tree plus these loops,
        # driven through joint_names (see closed_loop_pinocchio_solver.hpp).
        balls = plan.mjcf.balls if plan.mjcf is not None else ()
        config.update({
            'kinematics_solver': 'pinocchio_closed_loop',
            'loop_closures': [c.config() for c in plan.closures],
            'ee_orientation_only': bool(plan.mechanism.get('orientation_only')),
            'view_joint_names': mechanism.view_joints(plan.info, plan.mechanism_joints, balls),
        })
    return config


def _write_report(path, plan, stats, config, messages):
    lines = [
        '# %s — import report' % plan.robot_id, '',
        'Imported by FIC Control Studio on %s from `%s`.'
        % (datetime.datetime.now().strftime('%Y-%m-%d %H:%M'), plan.source), '',
        '## Robot', '',
        '| | |', '|---|---|',
        '| Display name | %s |' % config['display_name'],
        '| Joints | %d |' % config['num_joints'],
        '| End effector | `%s` |' % plan.ee_link,
        '| Elbow | `%s` |' % (plan.elbow_link or '—'),
        '| Mass | %.2f kg |' % stats['mass_kg'],
        '| Reach at home | %.3f m |' % stats['reach_m'],
        '| Meshes | %d of %d found |' % plan.mesh_counts, '',
        '## Joints', '',
        '| Joint | Lower (°) | Upper (°) | Torque (Nm) | Speed (°/s) | Home (°) |',
        '|---|---|---|---|---|---|',
    ]
    for j in plan.joints:
        lines.append('| `%s` | %.1f | %.1f | %g%s | %s | %.1f |' % (
            j.name, j.lower_deg, j.upper_deg, j.effort,
            '' if j.effort_from_urdf else ' (default)',
            '%.0f' % math.degrees(j.velocity) if j.velocity else '—', j.home_deg))
    if plan.found:
        lines += ['', '## Filled in automatically', '',
                  '| | | From |', '|---|---|---|']
        lines += ['| %s | %s | %s |' % row for row in plan.found]
    notes = plan.messages + messages
    if notes:
        lines += ['', '## Notes', '']
        lines += ['- **%s** %s' % (level.upper(), text) for level, text in notes]
    lines += [
        '', '## Files', '',
        '- `URDF/%s.urdf` — read by the controllers (Pinocchio); joints off the arm '
        'are fixed here.' % plan.robot_id,
        '- `URDF/%s.original.urdf` — the URDF as it was imported.' % plan.robot_id,
        '- `URDF/meshes/` — the original mesh files.',
        '- `xml/%s.xml` — the MuJoCo robot, with motors `torq_j1..%d` and the wrench sensors%s. '
        "Its joints are damped like FIC's hand-made models (%s N m s/rad) so the arm "
        'settles rather than falls while the controller starts.'
        % (plan.robot_id, config['num_joints'],
           ', and a `gripper` actuator' if plan.gripper else '',
           ', '.join('%g' % mjcf.joint_damping(j.effort) for j in plan.joints)),
        '- `xml/scene.xml` — floor, light and sky around it; this is what the simulator loads.',
        '- `xml/assets/` — meshes MuJoCo reads (.dae converted to .obj).', '',
        'Re-importing the URDF regenerates everything here.', '',
    ]
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines))
