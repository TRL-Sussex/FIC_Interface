"""A MuJoCo contact rig: independent truth for the residual estimator.

The estimator answers "which link was pushed, and where" from joint
torque alone. Validating that needs torques produced by something other
than the model doing the estimating. `test_integration.py` does not have
that property -- it synthesises its torques through the same Pinocchio
model the estimator inverts, so any model error cancels inside it, which
is exactly how it stayed green through a 25 Nm gravity error.

This rig closes that gap. MuJoCo computes the arm's gravity and the
generalized force of a contact from its own model of the same robot --
one that carries the tool and gripper the URDF omits. An estimate checked
against it is checked against something it has never seen, which is the
property a published accuracy number needs.

No ROS. numpy and mujoco only, so it runs in a plain pytest process and
in offline sweeps.
"""

import mujoco

import numpy as np


class ContactSim:
    """Generate ground-truth joint torques for contacts on a MuJoCo arm.

    Link indices are 1-based and follow the actuated joint chain, exactly
    as ``ContactEstimate.link_index`` does: link ``k`` is the body joint
    ``k`` carries. That correspondence is what lets a reported link be
    compared against the link a force was actually applied to.
    """

    def __init__(self, mjcf_path, joint_names):
        """Load the model and resolve the joint/link chain by name."""
        self._model = mujoco.MjModel.from_xml_path(mjcf_path)
        self._data = mujoco.MjData(self._model)
        self._joint_names = list(joint_names)

        self._qadr, self._dof, self._bodies = [], [], []
        for name in self._joint_names:
            jid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError('joint %r not in the MJCF' % name)
            self._qadr.append(int(self._model.jnt_qposadr[jid]))
            self._dof.append(int(self._model.jnt_dofadr[jid]))
            self._bodies.append(int(self._model.jnt_bodyid[jid]))
        self._qadr = np.asarray(self._qadr, dtype=int)
        self._dof = np.asarray(self._dof, dtype=int)

        self.link_names = tuple(
            mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, b)
            for b in self._bodies)

    # ── pose ──────────────────────────────────────────────────────────

    @property
    def n(self):
        """Return the number of actuated joints."""
        return len(self._joint_names)

    def _pose(self, q):
        """Place the model at ``q`` and run forward kinematics."""
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape != (self.n,):
            raise ValueError('q must have %d entries, got %r'
                             % (self.n, q.shape))
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0
        self._data.qpos[self._qadr] = q
        mujoco.mj_forward(self._model, self._data)

    def _body_of(self, link):
        """Return the MuJoCo body id for 1-based ``link``."""
        if not isinstance(link, (int, np.integer)) or not (
                1 <= int(link) <= self.n):
            raise ValueError('link must be 1..%d, got %r' % (self.n, link))
        return self._bodies[int(link) - 1]

    # ── truth ─────────────────────────────────────────────────────────

    def gravity(self, q):
        """Return MuJoCo's gravity joint torque at ``q``, Nm, ROS order.

        With velocity zeroed, ``qfrc_bias`` is the gravity term alone,
        and in the same sign convention as Pinocchio's
        ``computeGeneralizedGravity`` -- both are the generalized force
        the actuators must supply to hold the pose.
        """
        self._pose(q)
        return np.asarray(self._data.qfrc_bias)[self._dof].copy()

    def contact_torque(self, q, link, point, force):
        """Return the joint torque ``J^T f`` of a contact, Nm, ROS order.

        ``point`` is in world coordinates and ``force`` is the force
        applied TO the arm, matching the convention the estimator fits
        (``solve_push_force`` constrains it to point into the surface).
        """
        self._pose(q)
        body = self._body_of(link)
        qfrc = np.zeros(self._model.nv, dtype=float)
        mujoco.mj_applyFT(
            self._model, self._data,
            np.asarray(force, dtype=float).reshape(3),
            np.zeros(3),
            np.asarray(point, dtype=float).reshape(3),
            body, qfrc)
        return qfrc[self._dof].copy()

    def measured_torque(self, q, link=0, point=None, force=None,
                        noise_nm=0.0, rng=None):
        """Return the joint torque a sensor would read, Nm, ROS order.

        ``tau_meas = g(q) + J^T f``. ``ResidualEstimator.residual``
        subtracts its own ``g(q)``, so what survives is the contact term
        -- plus whatever the two models disagree about, which is the
        error this rig exists to measure.

        ``link=0`` means no contact, giving pure gravity. Noise is
        Gaussian, per joint, and takes an explicit ``rng`` so a published
        sweep can be regenerated exactly.
        """
        tau = self.gravity(q)
        if link and force is not None and point is not None:
            tau = tau + self.contact_torque(q, link, point, force)
        if noise_nm:
            rng = np.random.default_rng() if rng is None else rng
            tau = tau + rng.normal(0.0, float(noise_nm), size=tau.shape)
        return tau

    # ── geometry ──────────────────────────────────────────────────────

    def link_origin(self, q, link):
        """Return the world position of ``link``'s body origin."""
        self._pose(q)
        return np.asarray(self._data.xpos[self._body_of(link)]).copy()

    def surface_point(self, q, link, index=0):
        """Return a world-frame contact site on ``link``.

        Walks that body's geoms and offsets from the geom centre along a
        deterministic direction, so a sweep can visit several distinct
        sites per link without depending on mesh vertex order. Points are
        near the surface rather than exactly on it: what matters for the
        torque is the lever arm, and a point a few millimetres off the
        mesh produces the same physics as one exactly on it.
        """
        self._pose(q)
        body = self._body_of(link)
        start = int(self._model.body_geomadr[body])
        count = int(self._model.body_geomnum[body])
        if count <= 0:
            return self.link_origin(q, link)
        gid = start + (int(index) % count)
        centre = np.asarray(self._data.geom_xpos[gid], dtype=float)
        rot = np.asarray(self._data.geom_xmat[gid], dtype=float).reshape(3, 3)
        radius = float(np.max(self._model.geom_rbound[gid]))
        if not np.isfinite(radius) or radius <= 0.0:
            radius = 0.05
        # Deterministic direction on the unit sphere, varied by index so
        # repeated calls sample around the link rather than one spot.
        angle = 2.399963 * float(index)          # golden angle, radians
        height = 1.0 - 2.0 * ((int(index) % 8) + 0.5) / 8.0
        planar = float(np.sqrt(max(0.0, 1.0 - height * height)))
        local = np.array([planar * np.cos(angle),
                          planar * np.sin(angle),
                          height], dtype=float)
        return centre + rot @ (local * radius)
