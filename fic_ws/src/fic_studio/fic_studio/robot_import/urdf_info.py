"""Read a URDF and work out the arm: which joints move it and where it ends.

Plain ElementTree, so it runs anywhere the Studio does. A ``.xacro`` file is
expanded with the ``xacro`` tool first.
"""
from dataclasses import dataclass, field
import math
import os
import re
import subprocess
from typing import Dict, List, Optional
import xml.etree.ElementTree as ET

MOVABLE = ('revolute', 'continuous', 'prismatic')

#: Link names that usually mark the end of an arm, most specific first.
EE_NAME_HINTS = ('tcp', 'tool0', 'tool', 'flange', 'end_effector', 'ee', 'tip',
                 'hand', 'gripper', 'wrist')


#: Joint or link names that belong to a gripper, not the arm.
_GRIPPER_RE = re.compile(r'finger|knuckle|gripper|jaw|claw|thumb|_pad\b|palm', re.I)


def _hint_rank(name):
    """Position of ``name`` in EE_NAME_HINTS (len() when none match)."""
    low = name.lower()
    for i, hint in enumerate(EE_NAME_HINTS):
        if re.search(r'(^|[_\-.])%s([_\-.\d]|$)' % re.escape(hint), low):
            return i
    return len(EE_NAME_HINTS)


class UrdfError(Exception):
    """A URDF that cannot be used; the message is written for the user."""


@dataclass
class JointInfo:
    name: str
    type: str               # noqa: A003 -- the URDF attribute's name
    parent: str
    child: str
    lower: Optional[float] = None       # rad (or m for prismatic)
    upper: Optional[float] = None
    effort: Optional[float] = None      # N m (or N)
    velocity: Optional[float] = None    # rad/s (or m/s)
    origin: tuple = (0.0, 0.0, 0.0)     # xyz of the joint in its parent link
    mimic: Optional[tuple] = None       # (joint, multiplier, offset)

    @property
    def movable(self):
        return self.type in MOVABLE

    @property
    def continuous(self):
        return self.type == 'continuous' or (
            self.type == 'revolute' and self.lower is None and self.upper is None)


@dataclass
class MeshRef:
    link: str
    role: str                  # 'visual' or 'collision'
    uri: str
    element: ET.Element = field(repr=False)


_SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', 'node_modules'}


def find_packages(roots, max_depth=4):
    """ROS packages (name -> folder) found by their package.xml under ``roots``."""
    found = {}
    for root in roots:
        root = os.path.abspath(root)
        base_depth = root.rstrip(os.sep).count(os.sep)
        for dirpath, dirnames, filenames in os.walk(root):
            if dirpath.count(os.sep) - base_depth >= max_depth:
                dirnames[:] = []
            dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS and not d.startswith('.')]
            if 'package.xml' in filenames:
                try:
                    name = ET.parse(os.path.join(dirpath, 'package.xml')).findtext('name')
                except (ET.ParseError, OSError):
                    name = None
                if name and name.strip() not in found:
                    found[name.strip()] = dirpath
    return found


def package_dir_of(path):
    """Return the ROS package folder (the one with package.xml) holding ``path``, or ''."""
    cur = os.path.dirname(os.path.abspath(path))
    for _ in range(4):
        if os.path.isfile(os.path.join(cur, 'package.xml')):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return ''


def package_roots_for(path):
    """Folders to search for the ROS packages a single dropped file may need.

    A file inside a description package (``pkg/urdf/robot.xacro``) usually
    includes files from that package and its siblings, so search from the
    folder holding the package. A file in no package has none to find.
    """
    pkg = package_dir_of(path)
    return [os.path.dirname(pkg)] if pkg else []


def _xacro_env(packages, prefix):
    """Return an environment in which ``$(find pkg)`` also finds ``packages``."""
    index = os.path.join(prefix, 'share', 'ament_index', 'resource_index', 'packages')
    os.makedirs(index, exist_ok=True)
    for name, folder in packages.items():
        open(os.path.join(index, name), 'w').close()
        link = os.path.join(prefix, 'share', name)
        if not os.path.exists(link):
            os.symlink(folder, link)
    env = dict(os.environ)
    env['AMENT_PREFIX_PATH'] = os.pathsep.join(
        p for p in (prefix, env.get('AMENT_PREFIX_PATH', '')) if p)
    return env


def _xacro_failure(output):
    lines = [line.strip() for line in output.strip().splitlines() if line.strip()]
    missing = re.search(r"package '([^']+)' not found", output)
    if missing:
        return ('This xacro needs the ROS package %r, which is not installed and was not in '
                'what you dropped. Drop the folder that holds the whole description package '
                '(the one with package.xml), or a plain .urdf.' % missing.group(1))
    detail = next((line for line in lines if line.lower().startswith('error')),
                  lines[0] if lines else '')
    detail = re.sub(r'^error:\s*', '', detail, flags=re.I)
    arg = re.search(r'Invalid parameter "([^"]+)"', output)
    if arg:
        return ('This xacro file needs the argument %r set. Expand it yourself, e.g. '
                '"xacro %s %s:=<value> > robot.urdf", then drop robot.urdf.'
                % (arg.group(1), 'robot.xacro', arg.group(1)))
    return ('xacro could not expand this file (%s). Many xacro files need arguments: expand '
            'it yourself, e.g. "xacro robot.xacro name:=value > robot.urdf", and drop the '
            '.urdf.' % (detail or 'unknown error'))


def load_urdf_text(path, packages=None):
    """Return the URDF XML for ``path``, expanding ``.xacro`` files.

    ``packages`` (name -> folder, see ``find_packages``) are made visible to
    xacro's ``$(find ...)``, so a description package works without being
    built or installed.
    """
    if path.lower().endswith('.xacro'):
        import tempfile
        packages = packages or {}
        with tempfile.TemporaryDirectory(prefix='fic_xacro_') as prefix:
            env = _xacro_env(packages, prefix)
            try:
                proc = subprocess.run(['xacro', path], capture_output=True, text=True,
                                      timeout=60, env=env)
            except FileNotFoundError:
                raise UrdfError('This is a .xacro file, and the xacro tool is not installed. '
                                'Install ros-$ROS_DISTRO-xacro, or convert it to .urdf first.')
            except subprocess.TimeoutExpired:
                raise UrdfError('xacro took too long to expand this file.')
        if proc.returncode != 0:
            raise UrdfError(_xacro_failure(proc.stderr or proc.stdout))
        text = proc.stdout
        # $(find pkg) expanded to the temporary prefix, which is gone now.
        for name, folder in packages.items():
            text = text.replace(os.path.join(prefix, 'share', name), folder)
        return text
    with open(path, encoding='utf-8', errors='replace') as fh:
        return fh.read()


def _float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def iter_mesh_elements(root):
    """Yield (link name, 'visual'|'collision', <mesh> element) in a fixed order.

    The importer walks two copies of the same tree with this, so the order
    must depend on nothing but the tree.
    """
    for link in root.findall('link'):
        for role in ('visual', 'collision'):
            for el in link.findall(role):
                mesh = el.find('geometry/mesh')
                if mesh is not None and mesh.get('filename'):
                    yield link.get('name'), role, mesh


class UrdfInfo:
    """The parts of a URDF the importer needs, plus the tree to walk them."""

    def __init__(self, text, source_path):
        try:
            self.root = ET.fromstring(text)
        except ET.ParseError as exc:
            raise UrdfError('The URDF is not valid XML: %s' % exc)
        if self.root.tag != 'robot':
            raise UrdfError('This file is not a URDF: its top element is <%s>, not <robot>.'
                            % self.root.tag)
        self.text = text
        self.source_path = os.path.abspath(source_path)
        self.robot_name = self.root.get('name') or os.path.splitext(
            os.path.basename(source_path))[0]
        self.links = [link.get('name') for link in self.root.findall('link')]
        if not self.links:
            raise UrdfError('The URDF defines no links.')
        self.joints: Dict[str, JointInfo] = {}
        for j in self.root.findall('joint'):
            limit = j.find('limit')
            info = JointInfo(
                name=j.get('name'), type=j.get('type', 'fixed'),
                parent=(j.find('parent').get('link') if j.find('parent') is not None else ''),
                child=(j.find('child').get('link') if j.find('child') is not None else ''))
            if limit is not None:
                info.lower = _float(limit.get('lower'))
                info.upper = _float(limit.get('upper'))
                info.effort = _float(limit.get('effort'))
                info.velocity = _float(limit.get('velocity'))
            if info.type == 'continuous':
                info.lower = info.upper = None
            origin = j.find('origin')
            if origin is not None:
                xyz = [_float(v) for v in (origin.get('xyz') or '').split()]
                if len(xyz) == 3 and None not in xyz:
                    info.origin = tuple(xyz)
            mimic = j.find('mimic')
            if mimic is not None and mimic.get('joint'):
                info.mimic = (mimic.get('joint'), _float(mimic.get('multiplier')) or 1.0,
                              _float(mimic.get('offset')) or 0.0)
            self.joints[info.name] = info
        self.child_joints: Dict[str, List[JointInfo]] = {link: [] for link in self.links}
        self.parent_joint: Dict[str, JointInfo] = {}
        for j in self.joints.values():
            self.child_joints.setdefault(j.parent, []).append(j)
            self.parent_joint[j.child] = j
        roots = [link for link in self.links if link not in self.parent_joint]
        if len(roots) != 1:
            raise UrdfError('The URDF must form one tree with one base link; found %d links '
                            'without a parent (%s).' % (len(roots), ', '.join(roots[:5])))
        self.root_link = roots[0]
        self.mesh_refs = self._mesh_refs()

    def _mesh_refs(self):
        return [MeshRef(link, role, mesh.get('filename'), mesh)
                for link, role, mesh in iter_mesh_elements(self.root)]

    # ── Tree queries ──────────────────────────────────────────────────────
    def chain_to(self, link):
        """Joints from the base link down to ``link``, in order."""
        chain = []
        while link in self.parent_joint:
            j = self.parent_joint[link]
            chain.append(j)
            link = j.parent
        return list(reversed(chain))

    def movable_chain_to(self, link):
        return [j for j in self.chain_to(link) if j.movable]

    def leaves(self):
        return [link for link in self.links if not self.child_joints.get(link)]

    def descendants(self, link):
        out, stack = [], [link]
        while stack:
            cur = stack.pop()
            for j in self.child_joints.get(cur, []):
                out.append(j.child)
                stack.append(j.child)
        return out

    def is_gripper_joint(self, joint):
        """Return True for joints that move fingers rather than the arm."""
        return bool(_GRIPPER_RE.search(joint.name) or _GRIPPER_RE.search(joint.child))

    def _arm_movable_count(self, link):
        """Arm (non-gripper) moving joints anywhere below ``link``."""
        count = 0
        for j in self.child_joints.get(link, []):
            count += (j.movable and not self.is_gripper_joint(j))
            count += self._arm_movable_count(j.child)
        return count

    def guess_ee(self):
        """Guess the link the arm ends at.

        Walk down from the base, always into the branch holding the most arm
        joints, until no arm joint is left below. Finger joints never extend
        the arm. From the last arm link, a tool / flange / TCP frame hanging
        off it on fixed joints is preferred; otherwise the deepest such frame
        on a single unbranched path, else the last arm link itself.
        """
        link = self.root_link
        last_arm_link = link
        while True:
            branches = [(self._arm_movable_count(j.child)
                         + (j.movable and not self.is_gripper_joint(j)), j)
                        for j in self.child_joints.get(link, [])]
            branches = [b for b in branches if b[0] > 0]
            if not branches:
                break
            _, joint = max(branches, key=lambda b: (b[0], b[1].name))
            link = joint.child
            if joint.movable and not self.is_gripper_joint(joint):
                last_arm_link = link

        # Fixed frames after the last arm joint.
        frames, stack = [last_arm_link], [last_arm_link]
        while stack:
            cur = stack.pop()
            for j in self.child_joints.get(cur, []):
                if j.type == 'fixed':
                    frames.append(j.child)
                    stack.append(j.child)
        hinted = [f for f in frames if _hint_rank(f) < len(EE_NAME_HINTS)]
        if hinted:
            return min(hinted, key=lambda f: (_hint_rank(f), len(self.chain_to(f)), f))
        path, cur = last_arm_link, last_arm_link
        while True:
            fixed = [j for j in self.child_joints.get(cur, []) if j.type == 'fixed']
            if len(fixed) != 1 or self.child_joints.get(cur, []) != fixed:
                break
            cur = path = fixed[0].child
        return path

    def arm_joints(self, ee_link):
        return self.movable_chain_to(ee_link)

    def extra_joints(self, ee_link):
        """Return the moving joints that are not on the arm's chain (fingers, wheels...)."""
        arm = {j.name for j in self.arm_joints(ee_link)}
        return [j for j in self.joints.values() if j.movable and j.name not in arm]

    def guess_elbow(self, ee_link):
        """Guess the elbow: the link after the middle arm joint (Kinova: ForeArm_Link)."""
        arm = self.arm_joints(ee_link)
        if len(arm) < 3:
            return ''
        return arm[(len(arm) + 1) // 2 - 1].child

    def reach(self, link):
        """Upper bound on how far ``link`` can get from the base (m): chain length."""
        return sum(math.sqrt(sum(c * c for c in j.origin)) for j in self.chain_to(link))

    def initial_positions(self):
        """{joint: value} from ``<ros2_control>`` state interfaces' initial_value."""
        out = {}
        for block in self.root.findall('ros2_control'):
            for j in block.findall('joint'):
                for iface in j.findall('state_interface'):
                    if iface.get('name') != 'position':
                        continue
                    for param in iface.findall('param'):
                        value = _float((param.text or '').strip())
                        if param.get('name') == 'initial_value' and value is not None:
                            out[j.get('name')] = value
        return out

    def massless_links(self, links):
        out = []
        for link in self.root.findall('link'):
            if link.get('name') not in links:
                continue
            mass = link.find('inertial/mass')
            if mass is None or (_float(mass.get('value')) or 0.0) <= 0.0:
                out.append(link.get('name'))
        return out


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _axis_angle(axis, a):
    n = math.sqrt(sum(c * c for c in axis)) or 1.0
    x, y, z = (c / n for c in axis)
    c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def _mat4(rot, xyz):
    return [[rot[0][0], rot[0][1], rot[0][2], xyz[0]],
            [rot[1][0], rot[1][1], rot[1][2], xyz[1]],
            [rot[2][0], rot[2][1], rot[2][2], xyz[2]],
            [0.0, 0.0, 0.0, 1.0]]


def _mul4(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def link_poses(info, q=None):
    """World pose (4x4 nested lists) of every link at joint values ``q`` (name -> value).

    Plain URDF forward kinematics, so it needs neither Pinocchio nor meshes.
    Joints not in ``q`` sit at 0; floating and planar joints at their origin.
    """
    q = q or {}
    poses = {info.root_link: _mat4([[1, 0, 0], [0, 1, 0], [0, 0, 1]], (0, 0, 0))}
    elements = {j.get('name'): j for j in info.root.findall('joint')}
    stack = [info.root_link]
    while stack:
        parent = stack.pop()
        for j in info.child_joints.get(parent, []):
            el = elements[j.name]
            origin = el.find('origin')
            has = origin is not None
            xyz = [float(v) for v in ((has and origin.get('xyz')) or '0 0 0').split()]
            rpy = [float(v) for v in ((has and origin.get('rpy')) or '0 0 0').split()]
            t = _mul4(poses[parent], _mat4(_rpy_matrix(*rpy), xyz))
            value = q.get(j.name, 0.0)
            axis_el = el.find('axis')
            axis = [float(v) for v in
                    ((axis_el is not None and axis_el.get('xyz')) or '1 0 0').split()]
            if j.type in ('revolute', 'continuous') and value:
                t = _mul4(t, _mat4(_axis_angle(axis, value), (0, 0, 0)))
            elif j.type == 'prismatic' and value:
                n = math.sqrt(sum(c * c for c in axis)) or 1.0
                t = _mul4(t, _mat4([[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                   [c / n * value for c in axis]))
            poses[j.child] = t
            stack.append(j.child)
    return poses


def limits_deg(joint):
    """(lower, upper) in degrees; ±360 for continuous joints (as the configs use)."""
    if joint.continuous:
        return -360.0, 360.0
    lo = joint.lower if joint.lower is not None else -math.pi
    hi = joint.upper if joint.upper is not None else math.pi
    return math.degrees(lo), math.degrees(hi)
