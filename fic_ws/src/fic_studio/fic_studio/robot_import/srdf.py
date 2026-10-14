"""Read what a MoveIt SRDF knows about a robot: its named poses and its tool.

A robot's ROS packages often ship a ``*_moveit_config`` beside the
description, and its SRDF says two things a URDF cannot: where the arm ends
(``<end_effector parent_link>``, or the tip of the arm group's
``<chain>``) and which poses are sensible (``<group_state name="ready">``).
The importer uses both when it finds an SRDF for the robot being imported.
"""
import os
import xml.etree.ElementTree as ET

#: Named poses that make a good home, best first.
HOME_STATE_NAMES = ('home', 'ready', 'default', 'start', 'init', 'initial', 'rest',
                    'zero', 'up')

_SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', 'node_modules'}


def find_srdf_files(roots, max_depth=5):
    """``.srdf`` and ``.srdf.xacro`` files under ``roots``, shallowest first."""
    found = []
    for root in roots:
        root = os.path.abspath(root)
        base = root.rstrip(os.sep).count(os.sep)
        for dirpath, dirnames, filenames in os.walk(root):
            if dirpath.count(os.sep) - base >= max_depth:
                dirnames[:] = []
            dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS and not d.startswith('.')]
            for f in filenames:
                if f.lower().endswith(('.srdf', '.srdf.xacro')):
                    found.append(os.path.join(dirpath, f))
    return sorted(dict.fromkeys(found), key=lambda p: (p.count(os.sep), p))


class SrdfInfo:
    """Groups, named poses and end effectors of one SRDF."""

    def __init__(self, text, path=''):
        """Parse ``text``; raises ValueError when it is not an SRDF."""
        try:
            root = ET.fromstring(text)
        except ET.ParseError as exc:
            raise ValueError('not valid XML: %s' % exc)
        if root.tag != 'robot':
            raise ValueError('not an SRDF')
        self.path = path
        self.robot_name = root.get('name', '')
        self.chains = {}          # group -> (base_link, tip_link)
        self.group_joints = {}    # group -> [joint names]
        for g in root.findall('group'):
            name = g.get('name', '')
            chain = g.find('chain')
            if chain is not None:
                self.chains[name] = (chain.get('base_link', ''), chain.get('tip_link', ''))
            self.group_joints[name] = [j.get('name') for j in g.findall('joint')]
        self.states = []          # (name, group, {joint: value})
        for s in root.findall('group_state'):
            values = {}
            for j in s.findall('joint'):
                try:
                    values[j.get('name')] = float(j.get('value', '').split()[0])
                except (ValueError, IndexError):
                    continue
            self.states.append((s.get('name', ''), s.get('group', ''), values))
        self.end_effectors = [(e.get('name', ''), e.get('parent_link', ''),
                               e.get('group', ''), e.get('parent_group', ''))
                              for e in root.findall('end_effector')]

    def relevance(self, urdf_joints):
        """How many of this SRDF's pose joints are in the URDF (0 = not for it)."""
        named = {j for _, _, values in self.states for j in values}
        named |= {j for joints in self.group_joints.values() for j in joints}
        return len(named & set(urdf_joints))

    def tool_link(self, links, arm_joint_names=()):
        """Return the arm's last link by the SRDF's say-so, or '' if it names none we have.

        An end effector's parent_link is the most direct statement; failing
        that, the tip of the chain group that holds the most arm joints.
        """
        for _, parent_link, _, _ in self.end_effectors:
            if parent_link in links:
                return parent_link
        best, best_score = '', -1
        for group, (_, tip) in self.chains.items():
            score = len(set(self.group_joints.get(group, [])) & set(arm_joint_names))
            if tip in links and score > best_score:
                best, best_score = tip, score
        return best

    def home_state(self, arm_joint_names):
        """(state name, {joint: value}) of the best named pose covering the arm, or None."""
        arm = set(arm_joint_names)

        def rank(state):
            name = state[0].lower()
            covered = len(arm & set(state[2]))
            named = HOME_STATE_NAMES.index(name) if name in HOME_STATE_NAMES \
                else len(HOME_STATE_NAMES)
            return (-covered, named)
        usable = [s for s in self.states if arm & set(s[2])]
        if not usable:
            return None
        name, _, values = min(usable, key=rank)
        if rank((name, '', values))[1] == len(HOME_STATE_NAMES) and len(usable) > 1:
            # No state is called anything home-like; a lone state is still
            # a deliberate pose, several unnamed ones are a guess too far.
            return None
        return name, values


def load_srdf(path, packages=None):
    """Parse ``path`` (expanding .srdf.xacro); None when it cannot be read."""
    from fic_studio.robot_import.urdf_info import load_urdf_text, UrdfError
    try:
        text = load_urdf_text(path, packages)
    except (UrdfError, OSError):
        return None
    try:
        return SrdfInfo(text, path)
    except ValueError:
        return None


def best_srdf(roots, urdf_joints, packages=None):
    """Return the SRDF under ``roots`` that describes the most of these joints, or None."""
    best, best_score = None, 0
    for path in find_srdf_files(roots)[:20]:
        info = load_srdf(path, packages)
        if info is None:
            continue
        score = info.relevance(urdf_joints)
        if score > best_score:
            best, best_score = info, score
    return best
