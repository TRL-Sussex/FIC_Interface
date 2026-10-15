"""Write a robot's hardware_model block into robots_config.yaml and the Studio's robot list
(fic_studio_robots.yaml) by replacing or inserting only that block, so the rest of each file, its
comments included, is kept as it is."""
import os
import re

import yaml

FILES = (os.path.join('fic_library', 'config', 'robots_config.yaml'),
         os.path.join('fic_ws', 'src', 'fic_studio', 'fic_studio', 'fic_studio_robots.yaml'))


def _fmt(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, (list, tuple)):
        return '[' + ', '.join(_fmt(x) for x in v) + ']'
    if isinstance(v, str):
        return yaml.safe_dump(v, default_style=None).strip().removesuffix('...').strip()
    return repr(float(v)) if isinstance(v, float) else str(v)


def render(hm, indent=4):
    pad = ' ' * indent
    keys = ('enabled', 'source', 'joint_friction', 'joint_damping', 'armature', 'torque_noise',
            'torque_filter_hz', 'command_delay_ms')
    lines = [pad + 'hardware_model:']
    for k in keys + tuple(k for k in hm if k not in keys):
        if k in hm:
            lines.append('%s  %s: %s' % (pad, k, _fmt(hm[k])))
    return '\n'.join(lines) + '\n'


def splice(text, robot, hm):
    """`text` with `robot`'s hardware_model block replaced (or added after its admittance
    deadbands, else at the end of the robot's entry)."""
    m = re.search(r'^  %s:\s*$' % re.escape(robot), text, re.M)
    if not m:
        raise KeyError('robot %s not in the file' % robot)
    start = m.end() + 1
    nxt = re.search(r'^  \S', text[start:], re.M)            # the next robot (2-space key)
    end = start + nxt.start() if nxt else len(text)
    entry = text[start:end]
    blk = render(hm)
    old = re.search(r'^    hardware_model:\s*\n(?:^      .*\n|^\s*\n)*', entry, re.M)
    if old:
        entry = entry[:old.start()] + blk + entry[old.end():]
    else:
        db = re.search(r'^    admittance_deadbands:\s*\n(?:^      .*\n)*', entry, re.M)
        at = db.end() if db else len(entry.rstrip('\n')) + 1
        entry = entry[:at] + blk + entry[at:]
    out = text[:start] + entry + text[end:]
    yaml.safe_load(out)                                       # still valid YAML
    return out


def write(repo, robot, hm):
    """Write the block into every file of FILES that lists `robot`; returns the files written."""
    done = []
    for rel in FILES:
        p = os.path.join(repo, rel)
        if not os.path.isfile(p):
            continue
        with open(p) as f:
            text = f.read()
        if not re.search(r'^  %s:\s*$' % re.escape(robot), text, re.M):
            continue
        new = splice(text, robot, hm)
        tmp = p + '.tmp%d' % os.getpid()
        with open(tmp, 'w') as f:
            f.write(new)
        os.replace(tmp, p)
        done.append(p)
    return done
