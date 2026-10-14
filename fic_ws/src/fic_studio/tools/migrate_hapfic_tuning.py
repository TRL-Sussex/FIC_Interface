"""Give each robot's HAP-FIC a tuning block of its own in config.xml: <mode id="100" name="HAP-FIC">.

Before it had one, HAP-FIC ran on mode 4's joints (its joint layer) and mode 6's Cartesian block
(its Cartesian layer), so tuning HAP-FIC retuned those modes too. For every robot with a mode 4 or
mode 6 block, "default" and every profile with a mode 4 or 6 block of its own get a HAP-FIC block
copied from exactly what that profile runs HAP-FIC on now: nothing changes on the robot until
HAP-FIC is tuned.

--move-mode4 names profiles whose mode 4 block was there only to tune HAP-FIC (the Kinova's
hw_x2..hw_x6 and sim). Once their damping is in their HAP-FIC block, that mode 4 block is removed,
so plain mode 4 runs the default tuning again under them. It is removed only when it matches
default's mode 4 apart from the admittance damping; otherwise it is kept and the script says so.

Comments and layout are kept: the new blocks are added after each robot's last mode and nothing
else is rewritten. Running it again changes nothing.

    PYTHONPATH=fic_ws/src/fic_studio python3 fic_ws/src/fic_studio/tools/migrate_hapfic_tuning.py \\
        [config.xml] [--move-mode4 hw_x2,hw_x3,hw_x4,hw_x6,sim] [--dry-run]
"""
import argparse
import copy
import sys
import xml.etree.ElementTree as ET

from fic_studio.xml_manager import (HAPFIC_FALLBACK_CART_MODE, HAPFIC_FALLBACK_JOINT_MODE, HAPFIC_MODE,
                                    build_hapfic_block, find_config_path)

INDENT = '    '
LEVEL = 2                                  # <robots> 0, <robot> 1, <mode> 2


def _id(m):
    try:
        return int(m.get('id', -1))
    except ValueError:
        return -1


def _profile(m):
    return m.get('profile', 'default') or 'default'


def _own(robot, mode, profile):
    """The profile's own block for the mode (no fallback to default)."""
    return next((m for m in robot.findall('mode') if _id(m) == mode and _profile(m) == profile), None)


def _norm(el, root=True):
    """A block as values: tags, attributes below the block itself, and numbers compared as numbers."""
    text = (el.text or '').strip()
    try:
        value = tuple(float(v) for v in text.split(',')) if text else ()
    except ValueError:
        value = text
    kids = tuple(_norm(c, False) for c in el if isinstance(c.tag, str))
    return (el.tag, () if root else tuple(sorted(el.attrib.items())), value, kids)


def _without_damping(block):
    block = copy.deepcopy(block)
    for adm in block.iter('admittance'):
        for d in adm.findall('Damping'):
            adm.remove(d)
    return _norm(block)


def _insert_after_modes(robot, block):
    modes = [i for i, c in enumerate(robot) if c.tag == 'mode']
    i = modes[-1] if modes else len(robot) - 1
    prev = robot[i]
    block.tail = prev.tail
    prev.tail = '\n' + INDENT * LEVEL
    ET.indent(block, space=INDENT, level=LEVEL)
    robot.insert(i + 1, block)


def _remove(robot, el):
    kids = list(robot)
    i = kids.index(el)
    gone = [el]
    if i > 0 and kids[i - 1].tag is ET.Comment:          # the comment that described the block goes with it
        gone.insert(0, kids[i - 1])
        i -= 1
    if i > 0:
        kids[i - 1].tail = el.tail
    for g in gone:
        robot.remove(g)


def migrate(root, move_mode4=()):
    """Add the HAP-FIC blocks (and move the named profiles' mode 4 damping into them); return what changed."""
    done = []
    for robot in root.iter('robot'):
        name = robot.get('name')
        modes = robot.findall('mode')
        layers = [m for m in modes if _id(m) in (HAPFIC_FALLBACK_JOINT_MODE, HAPFIC_FALLBACK_CART_MODE)]
        if not layers:
            continue
        profiles = ['default'] + sorted({_profile(m) for m in layers} - {'default'})
        for prof in profiles:
            if _own(robot, HAPFIC_MODE, prof) is not None:
                continue
            block = build_hapfic_block(robot, prof, from_layers=True)
            if block is None:
                continue
            if prof in move_mode4:
                block.insert(0, ET.Comment(' HAP-FIC tuning of profile "%s": its joint layer was this profile\'s mode 4 '
                                           'block, kept only to tune HAP-FIC (moved here by migrate_hapfic_tuning). ' % prof))
            _insert_after_modes(robot, block)
            done.append('%s: HAP-FIC block for "%s"' % (name, prof))
        base = _own(robot, HAPFIC_FALLBACK_JOINT_MODE, 'default')
        for prof in move_mode4:
            own = _own(robot, HAPFIC_FALLBACK_JOINT_MODE, prof)
            if prof == 'default' or own is None or base is None or _own(robot, HAPFIC_MODE, prof) is None:
                continue
            if _without_damping(own) != _without_damping(base):
                print('%s: kept mode 4 of "%s": it differs from default\'s in more than the damping' % (name, prof),
                      file=sys.stderr)
                continue
            _remove(robot, own)
            done.append('%s: mode 4 of "%s" removed (its damping is in its HAP-FIC block)' % (name, prof))
    return done


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('path', nargs='?', default=find_config_path())
    ap.add_argument('--move-mode4', default='', help='profiles whose mode 4 block existed only to tune HAP-FIC')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args(argv)
    tree = ET.parse(a.path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    done = migrate(tree.getroot(), [p for p in a.move_mode4.split(',') if p])
    for d in done:
        print(d)
    if not done:
        print('%s: nothing to do' % a.path)
    elif not a.dry_run:
        tree.write(a.path, encoding='utf-8', xml_declaration=True)
        print('wrote %s' % a.path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
