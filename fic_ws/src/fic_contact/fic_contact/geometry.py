"""Per-link contact candidate points and normals, read from the MJCF.

Each link body may carry one or more mesh geoms. Vertices are stored in
each geom's own frame, so the body-frame position of a vertex ``v`` is
``geom_pos + R(geom_quat) @ v``. Normals are indexed by ``mesh_normaladr``
and ``mesh_normalnum``, which are distinct from the vertex fields even
though they coincide for these meshes.

A stored normal with length below ``_DEGENERATE_NORMAL_TOL`` is dropped,
along with its vertex, rather than normalised into a near-zero vector: a
near-zero normal would make the downstream push constraint ``F.n <= 0``
hold for every force, silently accepting physically impossible pushes.
Kinova and Franka meshes measure zero such vertices today, so this is
defensive hardening rather than a fix for an observed failure.

When a link body carries several mesh geoms (Franka FR3's ``fr3_link0``
has eight), ``n_per_link`` is split across those geoms in proportion to
each geom's vertex count, with a floor of one point per geom, instead of
being applied once to their concatenation -- otherwise a small geom
sharing a body with a large one would keep only one or two points.
"""

import mujoco
import numpy as np

#: Vertices whose stored normal is shorter than this are treated as
#: degenerate and dropped rather than normalised; see the module docstring.
_DEGENERATE_NORMAL_TOL = 1e-8


class Candidates:
    """Sampled surface points and outward normals for one link."""

    def __init__(self, points, normals, dropped_degenerate=0):
        """Store body-frame points (m), unit outward normals, and drop count.

        ``dropped_degenerate`` counts vertices whose stored normal was
        shorter than ``_DEGENERATE_NORMAL_TOL`` and were removed instead
        of being normalised into a near-zero vector; see the module
        docstring for why that removal matters downstream.
        """
        self.points = points
        self.normals = normals
        self.dropped_degenerate = dropped_degenerate


def _quat_to_mat(quat):
    """Convert a MuJoCo wxyz quaternion to a 3x3 rotation matrix."""
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=float))
    return mat.reshape(3, 3)


def _allocate_budgets(counts, n_per_link):
    """Split ``n_per_link`` across geoms in proportion to vertex count.

    Every geom keeps at least one point, so a small geom sharing a body
    with a large one is never wholly dropped. A single-geom body gets
    the whole budget, unchanged from decimating the concatenation.
    """
    if len(counts) <= 1:
        return [n_per_link] * len(counts)
    total = sum(counts)
    return [max(1, int(n_per_link * count / total)) for count in counts]


def _sample_geom(points, normals, budget):
    """Return at most ``budget`` points/normals via a uniform stride.

    Uniform stride rather than random sampling: deterministic, and the
    vertex order of these meshes is not spatially clustered.
    """
    if points.shape[0] > budget:
        step = points.shape[0] // budget
        points, normals = points[::step][:budget], normals[::step][:budget]
    return points, normals


def load_candidates(mjcf_path, link_names, n_per_link=200):
    """Return ``{link index 1..n: Candidates}`` sampled from the meshes."""
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    out = {}
    for k, name in enumerate(link_names, start=1):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body < 0:
            raise ValueError('body %r not found in %s' % (name, mjcf_path))

        geoms = []
        start = model.body_geomadr[body]
        for geom in range(start, start + model.body_geomnum[body]):
            mesh = model.geom_dataid[geom]
            if mesh < 0:
                continue
            v0 = model.mesh_vertadr[mesh]
            verts = model.mesh_vert[v0:v0 + model.mesh_vertnum[mesh]]
            n0 = model.mesh_normaladr[mesh]
            norms = model.mesh_normal[n0:n0 + model.mesh_normalnum[mesh]]
            count = min(len(verts), len(norms))
            if count == 0:
                continue

            rot = _quat_to_mat(model.geom_quat[geom])
            pts_g = model.geom_pos[geom] + verts[:count] @ rot.T
            nrm_g = norms[:count] @ rot.T
            geoms.append((pts_g, nrm_g))

        if not geoms:
            continue

        budgets = _allocate_budgets([g[0].shape[0] for g in geoms], n_per_link)
        sampled = [_sample_geom(pts_g, nrm_g, budget)
                   for (pts_g, nrm_g), budget in zip(geoms, budgets)]
        pts = np.concatenate([s[0] for s in sampled])
        nrm = np.concatenate([s[1] for s in sampled])

        lengths = np.linalg.norm(nrm, axis=1)
        keep = lengths >= _DEGENERATE_NORMAL_TOL
        dropped = int(pts.shape[0] - np.count_nonzero(keep))

        pts = pts[keep]
        nrm = nrm[keep] / lengths[keep, np.newaxis]
        out[k] = Candidates(points=pts, normals=nrm, dropped_degenerate=dropped)
    return out
