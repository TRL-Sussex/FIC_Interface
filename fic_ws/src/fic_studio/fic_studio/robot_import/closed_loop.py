"""Closed kinematic loops: what they are, and what a mechanism with them can do.

A parallel mechanism (the CSPM, a five-bar, a delta) is a tree with some links
joined again by *loop closures*: a point on one link held to a point on
another. FIC stores each as ``{bodyA, posA, bodyB, posB}`` (positions in each
link's own frame) and the controllers' ``pinocchio_closed_loop`` solver closes
them itself. MuJoCo simulates them as ``<equality><connect site1 site2>``.

This module works on a compiled MuJoCo model that has those constraints, and
answers the questions an import needs settled:

* ``assemble`` -- close the loops with the driven joints at given values;
* ``mobility`` -- how many degrees of freedom the mechanism really has;
* ``ee_mobility`` / ``determines`` / ``pick_driven`` -- how the end effector
  can move, whether a set of driven joints fixes it, and a good set if none
  is given (joints nearest the base first, as parallel robots mount their
  motors);
* ``orientation_only`` -- whether the end effector can only turn (a spherical
  mechanism), in which case Cartesian control must not ask it to move.
"""
from dataclasses import dataclass, field
from typing import List

import mujoco
import numpy as np


@dataclass
class Closure:
    """A point on ``body_a`` held to a point on ``body_b`` (link frames, metres)."""

    body_a: str
    pos_a: List[float]
    body_b: str
    pos_b: List[float]
    name: str = ''
    solref: List[float] = field(default_factory=list)   # MuJoCo tuning, when known
    solimp: List[float] = field(default_factory=list)
    source: str = ''

    def config(self):
        """Return the entry for robots_config.yaml's ``loop_closures``."""
        return {'bodyA': self.body_a, 'posA': [round(float(v), 6) for v in self.pos_a],
                'bodyB': self.body_b, 'posB': [round(float(v), 6) for v in self.pos_b]}


# ── Constraint Jacobian ───────────────────────────────────────────────────

def _eq_rows(model, data):
    """Rows of data.efc_* that belong to equality constraints."""
    return np.nonzero(data.efc_type[:data.nefc] == mujoco.mjtConstraint.mjCNSTR_EQUALITY)[0]


def constraint_state(model, data):
    """(residual vector, Jacobian nrow x nv) of the equality constraints at data.qpos."""
    old = model.opt.jacobian
    model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    try:
        mujoco.mj_forward(model, data)
        rows = _eq_rows(model, data)
        jac = data.efc_J[:data.nefc * model.nv].reshape(data.nefc, model.nv)[rows].copy() \
            if data.nefc else np.zeros((0, model.nv))
        res = data.efc_pos[:data.nefc][rows].copy() if data.nefc else np.zeros(0)
    finally:
        model.opt.jacobian = old
    return res, jac


def joint_dofs(model, names):
    """Dof addresses of the named 1-dof joints (hinge / slide)."""
    out = []
    for n in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid < 0:
            raise KeyError(n)
        out.append(int(model.jnt_dofadr[jid]))
    return out


def assemble(model, data, driven=None, iters=60, tol=1e-9):
    """Close the loops by moving every dof except the driven ones.

    ``driven`` maps joint name -> value to hold. Gauss-Newton on the
    equality residual, damped a little so a near-singular pose does not
    fling the passive joints. Returns the final residual norm (m).
    """
    driven = driven or {}
    for name, value in driven.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[jid]] = value
    fixed = set(joint_dofs(model, list(driven)))
    free = [d for d in range(model.nv) if d not in fixed]
    res, jac = constraint_state(model, data)
    for _ in range(iters):
        if not len(res) or np.linalg.norm(res) < tol:
            break
        j = jac[:, free]
        step = -j.T @ np.linalg.solve(j @ j.T + 1e-10 * np.eye(len(res)), res)
        dq = np.zeros(model.nv)
        dq[free] = step
        mujoco.mj_integratePos(model, data.qpos, dq, 1.0)
        res, jac = constraint_state(model, data)
    mujoco.mj_forward(model, data)
    return float(np.linalg.norm(res)) if len(res) else 0.0


def _rank(m, tol=1e-6, scale=None):
    """Numerical rank: singular values above ``tol`` x ``scale`` (default the largest)."""
    if m.size == 0:
        return 0
    s = np.linalg.svd(m, compute_uv=False)
    ref = scale if scale is not None else max(1.0, s[0])
    return int((s > tol * ref).sum())


#: End-effector motion (per unit joint motion) below this fraction of the
#: arm's typical motion counts as none: model coordinates rounded to a micron
#: leave a CSPM leg's "spin" moving the platform by ~1e-4 of that.
EE_TOL = 1e-3


def _null(m, nv):
    """Orthonormal basis (nv x k) of the null space of ``m``."""
    if m.size == 0:
        return np.eye(nv)
    _, s, vt = np.linalg.svd(m)
    r = int((s > 1e-6 * max(1.0, s[0])).sum())
    return vt[r:].T


def _selector(model, names):
    sel = np.zeros((len(names), model.nv))
    for row, dof in enumerate(joint_dofs(model, names)):
        sel[row, dof] = 1.0
    return sel


def ee_jacobian(model, data, ee_body):
    """6 x nv Jacobian (linear over angular) of ``ee_body``'s origin."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body)
    if bid < 0:
        raise KeyError(ee_body)
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, bid)
    return np.vstack([jacp, jacr])


def mobility(model, data):
    """Degrees of freedom left once the loops are closed (at data's pose)."""
    _, jac = constraint_state(model, data)
    return model.nv - _rank(jac)


def ee_mobility(model, data, ee_body, driven=()):
    """How many ways the end effector can still move with ``driven`` held.

    With nothing held this is the mechanism's own end-effector DOF: 3 for
    the CSPM (it only turns), 6 for an arm carrying it. Internal motions
    that leave the end effector where it is do not count -- the CSPM's
    legs, joined by point constraints, can each spin in place.
    """
    _, jac = constraint_state(model, data)
    rows = [jac] + ([_selector(model, list(driven))] if driven else [])
    null = _null(np.vstack(rows), model.nv)
    if null.size == 0:
        return 0
    jee = ee_jacobian(model, data, ee_body)
    return _rank(jee @ null, EE_TOL, max(1e-9, float(np.linalg.norm(jee, 2))))


def determines(model, data, driven_names, ee_body):
    """Return True when holding ``driven_names`` fixes the end effector."""
    return ee_mobility(model, data, ee_body, driven_names) == 0


def pick_driven(model, data, candidates, ee_body):
    """Pick the joints to drive: base-most first, until the end effector is fixed.

    ``candidates`` are 1-dof joint names, best first (nearest the base, as
    parallel robots mount their motors). A candidate is taken when holding
    it removes a degree of freedom from the mechanism; picking stops once
    the end effector cannot move. So a redundant arm keeps all its joints,
    and a leg's passive joints are never picked over its base joint.
    """
    _, jac = constraint_state(model, data)
    picked, rows = [], [jac]
    rank = _rank(jac)
    for name in candidates:
        if ee_mobility(model, data, ee_body, picked) == 0:
            break
        row = _selector(model, [name])
        new_rank = _rank(np.vstack(rows + [row]))
        if new_rank > rank:
            picked.append(name)
            rows.append(row)
            rank = new_rank
    return picked


def orientation_only(model, data, ee_body, tol=1e-6):
    """Return True when no motion the loops allow moves ``ee_body``'s origin.

    The mechanism's feasible velocities are the null space of the
    constraint Jacobian; the end effector translates only if its position
    Jacobian is non-zero on that space.
    """
    _, jac = constraint_state(model, data)
    null = _null(jac, model.nv)
    if null.size == 0:
        return False
    jacp = ee_jacobian(model, data, ee_body)[:3]
    return float(np.linalg.norm(jacp @ null)) < tol * max(1.0, float(np.linalg.norm(jacp)))


def base_first(model, names):
    """``names`` (1-dof joints) ordered by how many joints lie between them and the base."""
    def depth(name):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        bid = int(model.jnt_bodyid[jid])
        count = jid - int(model.body_jntadr[bid])       # joints before it on its own body
        bid = int(model.body_parentid[bid])
        while bid > 0:
            count += int(model.body_jntnum[bid])
            bid = int(model.body_parentid[bid])
        return count
    return sorted(names, key=lambda n: (depth(n), names.index(n)))


def generic_pose(model, data, start=None, scale=0.25, seed=0):
    """Put ``data`` in an assembled pose near ``start`` (default qpos0) that is not special.

    Reference poses are often singular -- an arm stretched straight up, a
    spherical mechanism in its symmetric pose -- where the rank tests above
    under-count. Every hinge and slide is nudged by up to ``scale`` (within
    its range), then the loops are closed again. Returns the residual.
    """
    rng = np.random.default_rng(seed)
    data.qpos[:] = model.qpos0 if start is None else start
    for j in range(model.njnt):
        if model.jnt_type[j] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        adr = model.jnt_qposadr[j]
        value = data.qpos[adr] + rng.uniform(-scale, scale)
        if model.jnt_limited[j]:
            lo, hi = model.jnt_range[j]
            value = min(max(value, lo + 1e-3), hi - 1e-3)
        data.qpos[adr] = value
    return assemble(model, data, {})
