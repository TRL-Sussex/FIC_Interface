"""Where the repository is, the robot's entry in robots_config.yaml, and the arm's model
(Pinocchio, from the URDF the controllers load) for the hardware_model measurement."""
import os
import subprocess

import numpy as np
import yaml

CONFIG = os.path.join('fic_library', 'config', 'robots_config.yaml')


def repo_root():
    """The FIC repository: $FIC_ROOT, else the checkout this module is in (a symlinked or source
    install), else the working directory's or ~/FIC_Interface."""
    cands = [os.environ.get('FIC_ROOT', '')]
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(8):
        cands.append(d)
        d = os.path.dirname(d)
    cands += [os.getcwd(), os.path.dirname(os.getcwd()), os.path.expanduser('~/FIC_Interface')]
    for c in cands:
        if c and os.path.isfile(os.path.join(c, CONFIG)):
            return c
    raise FileNotFoundError('no FIC repository found (set FIC_ROOT)')


def robot_cfg(repo, robot):
    with open(os.path.join(repo, CONFIG)) as f:
        robots = yaml.safe_load(f)['robots']
    if robot not in robots:
        raise KeyError('%s is not in %s (robots: %s)' % (robot, CONFIG, ', '.join(robots)))
    return robots[robot]


def git_sha(repo):
    try:
        return subprocess.run(['git', '-C', repo, 'rev-parse', 'HEAD'], capture_output=True,
                              text=True, timeout=10).stdout.strip() or None
    except (OSError, subprocess.TimeoutExpired):
        return None


class ArmModel:
    """Inverse dynamics of the arm from the URDF the controllers load (the gripper URDF when the
    robot has one), its joints in the order of joint_names."""

    def __init__(self, repo, cfg):
        import pinocchio as pin
        self.pin = pin
        grip = cfg.get('has_gripper') and cfg.get('urdf_path_gripper')
        rel = cfg['urdf_path_gripper'] if grip else cfg['urdf_path']
        path = rel if os.path.isabs(rel) else os.path.join(repo, 'fic_ws', 'src', rel)
        self.model = pin.buildModelFromUrdf(path)
        self.data = self.model.createData()
        ids = [self.model.getJointId(n) for n in cfg['joint_names']]
        if any(i >= self.model.njoints for i in ids):
            raise KeyError('joint names not in %s' % path)
        self.joints = [self.model.joints[i] for i in ids]

    def rnea(self, q, qd, qdd):
        """M(q) qdd + C(q, qd) qd + g(q) for the named joints (other joints at rest)."""
        m, pin = self.model, self.pin
        Q = pin.neutral(m)
        V = np.zeros(m.nv)
        A = np.zeros(m.nv)
        for k, jm in enumerate(self.joints):
            if jm.nq == 2:                                    # continuous joint as (cos, sin)
                Q[jm.idx_q], Q[jm.idx_q + 1] = np.cos(q[k]), np.sin(q[k])
            else:
                Q[jm.idx_q] = q[k]
            V[jm.idx_v], A[jm.idx_v] = qd[k], qdd[k]
        tau = pin.rnea(m, self.data, Q, V, A)
        return np.array([tau[jm.idx_v] for jm in self.joints])
