"""Joint-torque residual against a dynamics model, for contact detection.

At rest the residual is ``tau_ext = tau_meas - g(q) - b``, where ``b`` is
a bias measured at a specific pose with nobody touching the arm. Because
a push against a stationary arm is quasi-static, a gravity-only model
torque is sufficient there: there is no Coriolis term to speak of.

While the arm is moving (a planner-driven impedance mode), gravity alone
is no longer enough — the Coriolis/centrifugal term ``C(q,dq)*dq`` is
real and a gravity-only model would attribute it to phantom contact — so
the residual switches to ``tau_ext = tau_meas - (C(q,dq)*dq + g(q)) - b``
instead. Neither form ever models the inertial term ``M(q)*ddq``: no
measured joint acceleration exists anywhere in this project to feed it.
"""

import numpy as np
import pinocchio as pin

#: Kinova Gen3 7-DOF link names, proximal to distal. Deprecated: this is a
#: Kinova Gen3 default; prefer ``ArmModel.link_names``, which is derived
#: from the URDF and works on any serial arm. Kept because other modules
#: already import this constant directly and later tasks will migrate off
#: it.
LINK_NAMES = (
    'Shoulder_Link',
    'HalfArm1_Link',
    'HalfArm2_Link',
    'ForeArm_Link',
    'SphericalWrist1_Link',
    'SphericalWrist2_Link',
    'Bracelet_Link',
)


def angular_difference(a, b):
    """Return the shortest signed angular difference ``a - b``, per joint.

    Joint angles live on a circle, so subtracting them as plain numbers
    reports 2*pi of motion for a joint that merely crossed the +-pi
    branch cut and did not move at all. Measured on hardware at 6.2805
    rad, which against a 0.05 rad staleness tolerance latches the
    estimator off for as long as the joint stays on the far side.

    Wrapping into (-pi, pi] is correct for revolute joints and is what
    every comparison of two poses in this module should use.
    """
    diff = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    return (diff + np.pi) % (2.0 * np.pi) - np.pi


def derive_link_names(model, joint_names):
    """Return the body frame each actuated joint moves, proximal to distal.

    For each joint, this looks for the BODY frame whose ``parentJoint`` is
    that joint's id: the URDF link the joint directly carries, before any
    further fixed joints (an end-effector tip bolted onto the last moving
    link, say). Reproduces the Kinova Gen3 link names exactly and also
    works on the CSPM legs (``leg1_prox``, ``leg2_prox``, ``leg3_prox``).

    Raises ``ValueError`` naming the joint if no such BODY frame exists —
    either the joint name is wrong, or this URDF's structure does not fit
    the one-actuated-joint-carries-one-link assumption the method needs.
    """
    names = []
    for joint_name in joint_names:
        jid = model.getJointId(joint_name)
        link = None
        for frame in model.frames:
            if frame.type == pin.FrameType.BODY and frame.parentJoint == jid:
                link = frame.name
                break
        if link is None:
            raise ValueError(
                'joint %r has no BODY frame; link names could not be '
                'derived and must be passed explicitly' % (joint_name,))
        names.append(link)
    return names


def _mjcf_body_inertia(model, body):
    """Return ``(mass, com, inertia)`` of one MuJoCo body, in its own frame.

    MuJoCo stores principal moments in a frame given by ``body_iquat``
    about a centre at ``body_ipos``; rotating them back gives the inertia
    tensor about the centre of mass, expressed in the body's own axes.
    """
    import mujoco

    mass = float(model.body_mass[body])
    com = np.asarray(model.body_ipos[body], dtype=float).copy()
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, np.asarray(model.body_iquat[body], dtype=float))
    rot = rot.reshape(3, 3)
    diag = np.diag(np.asarray(model.body_inertia[body], dtype=float))
    return mass, com, rot @ diag @ rot.T


def link_inertials_from_mjcf(mjcf_path, joint_names):
    """Return per-link ``(mass, com, inertia)`` read across from an MJCF.

    For a URDF that under-describes its own mass distribution, this
    rebuilds it from a MuJoCo model of the same robot. The shipped
    Franka_FR3 URDF is the extreme case: it has no ``<inertial>``
    elements whatsoever, so Pinocchio gives every link zero mass and the
    gravity torque is identically zero -- see ``gravity_is_degenerate``.

    Each entry combines the body an actuated joint carries with every
    descendant that no *later* actuated joint carries, so a wrist entry
    picks up a rigidly attached tool while a shoulder entry does not pick
    up the whole arm below it. Masses are summed, centres of mass
    combined by weight, and inertias carried to the combined centre by
    the parallel-axis theorem -- all expressed in the carrying body's own
    frame, which is the frame Pinocchio wants for that joint.

    The assumption this rests on is that the two models' link frames
    coincide. That is not checkable from inside this function, so it is
    checked from outside, by comparing the resulting gravity against
    MuJoCo's own (see test_link_inertials.py).
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    carriers = []
    for name in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError('joint %r not in %s' % (name, mjcf_path))
        carriers.append(int(model.jnt_bodyid[jid]))
    later = set(carriers)

    def _owned(root, index):
        """Bodies belonging to link ``index``: root plus its own subtree."""
        owned, stack = [root], [root]
        while stack:
            parent = stack.pop()
            for body in range(model.nbody):
                if int(model.body_parentid[body]) != parent or body == 0:
                    continue
                if body in later and body != root:
                    continue          # belongs to a more distal link
                owned.append(body)
                stack.append(body)
        return owned

    out = []
    for index, root in enumerate(carriers):
        rot_root = np.asarray(data.xmat[root], dtype=float).reshape(3, 3)
        pos_root = np.asarray(data.xpos[root], dtype=float)

        total = 0.0
        moment = np.zeros(3)
        parts = []
        for body in _owned(root, index):
            mass, com_local, inertia_local = _mjcf_body_inertia(model, body)
            if mass <= 0.0:
                continue
            rot_b = np.asarray(data.xmat[body], dtype=float).reshape(3, 3)
            pos_b = np.asarray(data.xpos[body], dtype=float)
            # Into the carrying body's frame.
            rot_rel = rot_root.T @ rot_b
            pos_rel = rot_root.T @ (pos_b - pos_root)
            com = rot_rel @ com_local + pos_rel
            inertia = rot_rel @ inertia_local @ rot_rel.T
            parts.append((mass, com, inertia))
            total += mass
            moment += mass * com

        if total <= 0.0:
            out.append((0.0, np.zeros(3), np.zeros((3, 3))))
            continue

        centre = moment / total
        combined = np.zeros((3, 3))
        for mass, com, inertia in parts:
            offset = com - centre
            combined += inertia + mass * (
                float(offset @ offset) * np.eye(3) - np.outer(offset, offset))
        out.append((total, centre, combined))
    return out


def tool_payload_from_mjcf(mjcf_path, joint_names):
    """Return ``(mass, com)`` of everything the MJCF carries past the arm.

    The shipped Kinova URDF ends at ``ee_link`` with ``mass="0"`` and
    omits the Robotiq gripper entirely, while the MJCF describes both.
    Summing the MJCF bodies strictly distal to the last actuated joint's
    body recovers exactly what the URDF's gravity model is missing --
    about 1.34 kg at the wrist for this arm -- without hand-entering a
    constant that would silently rot the next time the tool changes.

    ``com`` is returned in the last arm link's body frame, taken at the
    MJCF reference configuration, and is mass-weighted across those
    bodies. Only mass and centre of mass are extracted because only
    those two enter ``g(q)``.

    Raises ValueError if the last joint or its body cannot be resolved.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    last = joint_names[-1]
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, last)
    if jid < 0:
        raise ValueError('joint %r not in the MJCF' % last)
    root = int(model.jnt_bodyid[jid])

    def _is_distal(body):
        """Return True when ``body`` sits strictly below ``root``."""
        b = int(model.body_parentid[body])
        while b != 0:
            if b == root:
                return True
            b = int(model.body_parentid[b])
        return False

    total = 0.0
    moment = np.zeros(3)
    for body in range(model.nbody):
        if not _is_distal(body):
            continue
        mass = float(model.body_mass[body])
        if mass <= 0.0:
            continue
        total += mass
        moment += mass * np.asarray(data.xipos[body], dtype=float)

    if total <= 0.0:
        return 0.0, np.zeros(3)

    com_world = moment / total
    # Into the last arm link's own frame, so the result is independent of
    # the reference configuration the MJCF happened to be posed in.
    rot = np.asarray(data.xmat[root], dtype=float).reshape(3, 3)
    com_local = rot.T @ (com_world - np.asarray(data.xpos[root], dtype=float))
    return total, com_local


class ArmModel:
    """Pinocchio model wrapper that speaks the ROS joint order.

    Building the shipped Kinova URDF prints
    ``Could not parse inertial element for Link [ee_link]`` to stderr,
    because that link's ``ixx`` is malformed. The model still builds
    correctly (nq = nv = 7, 8.05 kg) and the message is expected.
    """

    def __init__(self, urdf_path, joint_names, link_names=None,
                 tool_payload=None, link_inertials=None):
        """Build the model and map ROS joint order onto Pinocchio indices.

        ``link_names`` overrides the per-joint body names that would
        otherwise be derived from the URDF with ``derive_link_names`` — the
        escape hatch for a robot whose structure the derivation cannot
        handle (see ``derive_link_names`` and ``gravity_is_degenerate``).
        """
        self._model = pin.buildModelFromUrdf(urdf_path)
        if link_inertials is not None:
            self._apply_link_inertials(joint_names, link_inertials)
        if tool_payload is not None:
            self._append_tool_payload(joint_names[-1], *tool_payload)
        self._data = self._model.createData()
        self._n = len(joint_names)

        idx_q, idx_v = [], []
        for name in joint_names:
            if not self._model.existJointName(name):
                known = [self._model.names[i]
                         for i in range(1, len(self._model.names))]
                raise ValueError(
                    'joint %r not in URDF; known joints are %s' % (name, known))
            joint = self._model.joints[self._model.getJointId(name)]
            idx_q.append(joint.idx_q)
            idx_v.append(joint.idx_v)
        self._idx_q = np.asarray(idx_q, dtype=int)
        self._idx_v = np.asarray(idx_v, dtype=int)

        if link_names is None:
            link_names = derive_link_names(self._model, joint_names)
        self.link_names = tuple(link_names)

        self._frame_ids = {}
        for k, link in enumerate(self.link_names, start=1):
            self._frame_ids[k] = self._model.getFrameId(link)

    def _apply_link_inertials(self, joint_names, inertials):
        """Replace each actuated joint's link inertia with a supplied one.

        Replaces rather than adds: the intended use is a URDF whose
        inertials are absent or wrong, so keeping what is already there
        would mean summing a good value onto a bad one.

        Must run before ``createData()`` -- Pinocchio sizes Data from the
        Model, so mutating the model afterwards leaves the two
        inconsistent.
        """
        if len(inertials) != len(joint_names):
            raise ValueError(
                'need one inertial per joint, got %d for %d joints'
                % (len(inertials), len(joint_names)))
        for name, (mass, com, inertia) in zip(joint_names, inertials):
            if not self._model.existJointName(name):
                raise ValueError('joint %r not in URDF' % name)
            mass = float(mass)
            com = np.asarray(com, dtype=float)
            inertia = np.asarray(inertia, dtype=float)
            if not np.isfinite(mass) or mass < 0.0:
                raise ValueError('link mass must be finite and >= 0, got %r'
                                 % (mass,))
            if com.shape != (3,) or inertia.shape != (3, 3):
                raise ValueError('expected a 3-vector com and 3x3 inertia')
            if not (np.isfinite(com).all() and np.isfinite(inertia).all()):
                raise ValueError('non-finite inertial for joint %r' % name)
            jid = self._model.getJointId(name)
            self._model.inertias[jid] = pin.Inertia(mass, com, inertia)

    def _append_tool_payload(self, last_joint, mass, com):
        """Attach an equivalent point mass to the last actuated joint.

        Corrects a URDF that under-describes what the arm carries. A
        point mass is exact for ``g(q)``, which depends only on mass and
        centre of mass; the rotational inertia this omits affects only
        ``M(q)`` and the Coriolis term, neither of which the gravity-only
        residual path uses.

        Must run before ``createData()`` -- Pinocchio's Data is sized
        from the Model, so mutating the model afterwards would leave the
        two inconsistent.
        """
        mass = float(mass)
        com = np.asarray(com, dtype=float)
        if not np.isfinite(mass) or mass < 0.0:
            raise ValueError(
                'tool payload mass must be finite and >= 0, got %r'
                % (mass,))
        if com.shape != (3,) or not np.isfinite(com).all():
            raise ValueError(
                'tool payload com must be 3 finite numbers, got %r'
                % (com,))
        if mass == 0.0:
            return
        if not self._model.existJointName(last_joint):
            raise ValueError('joint %r not in URDF' % last_joint)
        jid = self._model.getJointId(last_joint)
        inertia = pin.Inertia(mass, com, np.zeros((3, 3)))
        self._model.appendBodyToJoint(jid, inertia, pin.SE3.Identity())

    @property
    def n(self):
        """Number of actuated joints."""
        return self._n

    def _to_pin_q(self, q):
        """Scatter a ROS-ordered configuration into Pinocchio's q vector."""
        q_pin = pin.neutral(self._model)
        q_pin[self._idx_q] = np.asarray(q, dtype=float)
        return q_pin

    def _to_pin_v(self, dq):
        """Scatter a ROS-ordered velocity into Pinocchio's v vector.

        Unlike ``q``, Pinocchio's tangent space ``v`` is already Euclidean
        (no quaternion-style packing), so the neutral value is simply
        zero everywhere the scatter does not touch.
        """
        v_pin = np.zeros(self._model.nv)
        v_pin[self._idx_v] = np.asarray(dq, dtype=float)
        return v_pin

    def gravity_free_joints(self, trials=4, step=0.7, tol=1e-3):
        """Return the joint indices that g(q) does not depend on at all.

        On a fixed base whose first axis is vertical -- the Kinova, the
        Franka -- turning that joint swings the whole arm round without
        changing any joint's gravity load. Found numerically, by moving
        each joint at a few configurations and watching g, so it holds for
        whatever arm the URDF describes rather than being assumed.

        ``tol`` (Nm) is not zero because URDFs round: the Kinova's base
        joint is written rpy="3.1416 0 0", tilting its axis by ~7 urad,
        which moves g by 8e-5 Nm -- against 0.04-13 Nm for every other
        joint, and detection thresholds of 0.05 Nm and up.
        """
        rng = np.random.default_rng(0)
        free = []
        for j in range(self._n):
            moved = 0.0
            for _ in range(trials):
                q = rng.uniform(-1.5, 1.5, self._n)
                dq = np.zeros(self._n)
                dq[j] = step
                moved = max(moved, float(np.max(np.abs(
                    self.gravity(q + dq) - self.gravity(q)))))
            if moved < tol:
                free.append(j)
        return free

    def gravity(self, q):
        """Gravity joint torque g(q), in ROS joint order, Nm."""
        g = pin.computeGeneralizedGravity(
            self._model, self._data, self._to_pin_q(q))
        return np.asarray(g)[self._idx_v]

    def nonlinear(self, q, dq):
        """Nonlinear joint torque C(q,dq)*dq + g(q), in ROS order, Nm.

        One Pinocchio call, ``nonLinearEffects``, folds the
        Coriolis/centrifugal term together with gravity. At ``dq == 0``
        this is identical to ``gravity(q)`` — see
        ``test_nonlinear_at_zero_velocity_equals_gravity`` — but it is
        still not used at rest: callers that want the exact legacy
        gravity-only path call ``gravity()`` directly instead, so a
        bit-for-bit-identical calculation is guaranteed rather than
        merely numerically close.
        """
        tau = pin.nonLinearEffects(
            self._model, self._data, self._to_pin_q(q), self._to_pin_v(dq))
        return np.asarray(tau)[self._idx_v]

    def gravity_is_degenerate(self, samples=8, seed=0, tol=1e-6):
        """True when g(q) is ~zero everywhere, despite a non-trivial model.

        Every real serial arm bends under gravity somewhere in its
        workspace. A URDF whose ``<link>`` elements carry no ``<inertial>``
        data still builds successfully in Pinocchio — every body simply
        has zero mass — but its gravity torque is identically zero and any
        residual built on it is meaningless. The Franka_FR3 URDF shipped
        in this repo is the known case: it has zero ``<inertial>`` blocks.
        """
        rng = np.random.default_rng(seed)
        for _ in range(samples):
            q = rng.uniform(-1.0, 1.0, self._n)
            if np.linalg.norm(self.gravity(q)) > tol:
                return False
        return True

    def link_jacobians(self, q):
        """Return {link index 1..n: 6xn Jacobian} at each link's origin.

        Frames are ``LOCAL_WORLD_ALIGNED``, so rows 0:3 are the linear
        velocity of the body origin and rows 3:6 the angular velocity —
        which is what the point-Jacobian shift in ``hypotheses`` expects.
        """
        q_pin = self._to_pin_q(q)
        pin.computeJointJacobians(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)
        out = {}
        for k, fid in self._frame_ids.items():
            jac = pin.getFrameJacobian(
                self._model, self._data, fid, pin.LOCAL_WORLD_ALIGNED)
            out[k] = np.asarray(jac)[:, self._idx_v]
        return out

    def link_poses(self, q):
        """Return {link index 1..n: (position m, 3x3 rotation)} in the base frame.

        The "base frame" here is Pinocchio's root/universe frame -- the
        same frame ``link_jacobians``'s ``LOCAL_WORLD_ALIGNED`` Jacobians
        are aligned to, since this arm has a fixed base (no floating-base
        joint), so root == world == base with no extra transform between
        them. Used by callers that need to carry a link-local point (a
        ``geometry.load_candidates`` surface point, expressed in that
        link's own BODY frame) into the base frame: ``point_base = rot @
        point_body + position``.
        """
        q_pin = self._to_pin_q(q)
        pin.forwardKinematics(self._model, self._data, q_pin)
        pin.updateFramePlacements(self._model, self._data)
        out = {}
        for k, fid in self._frame_ids.items():
            placement = self._data.oMf[fid]
            out[k] = (np.array(placement.translation, dtype=float),
                      np.array(placement.rotation, dtype=float))
        return out


class Calibration:
    """Bias, noise and detection thresholds measured at one pose."""

    def __init__(self, bias, sigma, thresholds, q_ref, samples):
        """Store the measured calibration constants."""
        self.bias = bias
        self.sigma = sigma
        self.thresholds = thresholds
        self.q_ref = q_ref
        self.samples = samples

    def as_dict(self):
        """A JSON-serialisable view, for a run's meta.json."""
        return {
            'bias_nm': self.bias.tolist(),
            'sigma_nm': self.sigma.tolist(),
            'thresholds_nm': self.thresholds.tolist(),
            'q_ref_rad': self.q_ref.tolist(),
            'samples': self.samples,
        }


#: Consecutive noisy windows discarded before the next is believed anyway.
OUTLIER_MAX_DISCARDS = 3


class ResidualEstimator:
    """Turns measured joint torque into a contact residual.

    ``torque_sign`` defaults to 1.0 and should stay there. ``fic_hardware``
    already applies the robot config's ``torque_sign`` before publishing
    (``fic_hardware.cpp:650``), so applying it again silently inverts every
    residual. The parameter exists only for replaying raw shared-memory data.

    ``min_threshold_nm`` floors every per-joint detection threshold. A
    noiseless calibration window (sigma == 0) would otherwise yield a
    threshold of exactly 0.0, and a downstream ``|tau_ext| > threshold`` gate
    would then fire on ordinary floating-point dust. The default, 1e-3 Nm,
    is about three orders of magnitude below the controller's own 2.0 Nm
    ``admittance_deadbands.joint_tau`` deadband, so it is far too small to
    mask a real contact while still keeping a degenerate calibration usable.
    """

    def __init__(self, model, torque_sign=1.0, gravity_compensated=False,
                 sigma_k=6.0, pose_tolerance=0.05, min_threshold_nm=1e-3,
                 sigma_pool=1, outlier_ratio=0.0, outlier_floor_nm=0.05,
                 ignore_joints=()):
        """Configure the estimator against an ArmModel.

        ``outlier_ratio`` discards a calibration window whose noise on any
        joint exceeds that multiple of the recent windows' median, plus
        ``outlier_floor_nm``; see ``finish_calibration``. 0 (the default)
        accepts every window, exactly as before.

        ``ignore_joints`` are left out of the staleness test -- pass
        ``ArmModel.gravity_free_joints()``. Empty (the default) keeps the
        old behaviour exactly.

        ``sigma_pool`` is how many recent calibration windows the noise
        estimate is pooled over. The bias and the noise scale are
        different quantities with different timescales: the bias drifts
        (thermal, sensor) and wants re-measuring often, while sigma is a
        property of the sensor and does not. Measuring both from one
        short window ties them together and makes the thresholds a
        lottery -- measured on this Kinova, per-second sigma varies about
        threefold on a completely still arm, so a calibration landing in
        a quiet second sets a joint-4 threshold near 0.10 Nm and the next
        noisier second walks straight through it.

        Pooling as a root-mean-square over the last ``sigma_pool``
        windows is the estimate over all their samples together, so it
        converges instead of tracking whichever second it happened to
        land in. 1 keeps the original single-window behaviour exactly.
        """
        self._model = model
        self.torque_sign = float(torque_sign)
        self._gravity_compensated = bool(gravity_compensated)
        self._sigma_k = float(sigma_k)
        self._pose_tolerance = float(pose_tolerance)
        self._min_threshold_nm = float(min_threshold_nm)
        self._sigma_pool = max(1, int(sigma_pool))
        self._outlier_ratio = max(0.0, float(outlier_ratio))
        self._outlier_floor_nm = max(0.0, float(outlier_floor_nm))
        self._discards = 0
        #: Joints staleness does not count: moving them changes no gravity
        #: load, so the bias measured before is exactly as good after.
        self._drift_ignored = tuple(int(j) for j in ignore_joints)
        self._sigma_history = []
        self._cal = None
        self._acc = None
        #: Why the most recent window was thrown away, or None.
        self.last_discard = None

    @property
    def calibrated(self):
        """True once a calibration has been captured."""
        return self._cal is not None

    @property
    def calibration(self):
        """The current Calibration, or None."""
        return self._cal

    @property
    def pose_tolerance(self):
        """The default pose tolerance a fresh calibration is checked against.

        Exposed so a caller (the node, in a moving mode) can compare a
        relaxed override against the tolerance the calibration actually
        assumed, rather than duplicating the constructor's value.
        """
        return self._pose_tolerance

    @staticmethod
    def _is_at_rest(dq):
        """True when ``dq`` is absent or every component is exactly zero."""
        return dq is None or not np.any(np.asarray(dq, dtype=float))

    def _model_torque(self, q, tau_meas, dq=None):
        """Measured torque minus the model term, before bias removal.

        ``dq`` selects the model term: gravity-only ``g(q)`` when it is
        None or all-zero — bit-identical to the pre-motion-aware
        behaviour, so every existing caller is unaffected — or the
        velocity-aware ``C(q,dq)*dq + g(q)`` from ``ArmModel.nonlinear``
        once the arm is actually moving.
        """
        tau = self.torque_sign * np.asarray(tau_meas, dtype=float)
        if not self._gravity_compensated:
            if self._is_at_rest(dq):
                tau = tau - self._model.gravity(q)
            else:
                tau = tau - self._model.nonlinear(q, dq)
        return tau

    def start_calibration(self):
        """Begin accumulating a no-contact window at the current pose."""
        self._acc = {'tau': [], 'q': []}

    def accumulate(self, q, tau_meas, dq=None):
        """Add one no-contact sample to the calibration window.

        Pass the same ``dq`` regime the calibration will later be read
        back under: omit it (or pass all-zero) for a calibration taken at
        rest, or pass the measured velocity for one taken while moving, so
        the bias removes exactly the model term ``residual()`` will later
        add back in.
        """
        if self._acc is None:
            raise RuntimeError('start_calibration() first')
        self._acc['tau'].append(self._model_torque(q, tau_meas, dq))
        self._acc['q'].append(np.asarray(q, dtype=float))

    @property
    def calibrating(self):
        """Return True while a window is open."""
        return self._acc is not None

    def abort_calibration(self):
        """Throw the open window away and keep the previous calibration.

        For a window that was disturbed -- the arm pushed or moved while it
        was averaging. Its bias would carry the push and its noise the
        push's variation, so it must not replace a clean calibration.
        """
        self._acc = None

    def finish_calibration(self):
        """Close the window and derive bias, noise and thresholds.

        Returns the new Calibration, or None when ``outlier_ratio`` is set
        and the window's noise shows it was disturbed; the previous
        calibration then stays in force and ``last_discard`` says why.
        After OUTLIER_MAX_DISCARDS windows in a row are discarded, the
        next one is accepted and the noise history restarts from it: the
        sensor's noise can genuinely rise (a new pose, a different mode),
        and a bias that could never be refreshed again would be worse than
        one noisy window. The per-tick disturbance check in contact_node,
        which sees the arm move, is the primary guard against a touch.

        That second line of defence matters because the noise is pooled:
        measured on hardware, one window that caught the start of a push
        had a spread of ~6 Nm on joint 1 against a normal 0.04. Pooled
        into the RMS it set the thresholds to [13.1, 8.2, 6.3, 2.2, ...]
        Nm and kept them there for all eight pooled windows (~28 s),
        leaving the estimator blind to anything but a shove.
        """
        if self._acc is None or not self._acc['tau']:
            raise RuntimeError('no calibration samples accumulated')
        tau = np.asarray(self._acc['tau'])
        bias = tau.mean(axis=0)
        sigma = tau.std(axis=0)
        self.last_discard = None
        if self._outlier_ratio > 0.0 and self._sigma_history:
            # Median, not the RMS pool itself: one bad window already in
            # the pool must not become the yardstick that admits the next.
            usual = np.median(np.asarray(self._sigma_history), axis=0)
            limit = self._outlier_ratio * usual + self._outlier_floor_nm
            over = np.nonzero(sigma > limit)[0]
            if over.size and self._discards < OUTLIER_MAX_DISCARDS:
                j = int(over[np.argmax(sigma[over] / np.maximum(usual[over], 1e-12))])
                self.last_discard = (
                    'joint %d noise %.3f Nm is over %.0fx the recent %.3f Nm'
                    % (j + 1, sigma[j], self._outlier_ratio, usual[j]))
                self._acc = None
                self._discards += 1
                return None
            if over.size:
                # The noise has stayed high: believe it, and start over.
                self._sigma_history = []
        self._discards = 0
        # Pool the noise estimate over recent windows, but never the bias
        # -- the bias is exactly the thing that has to be fresh. See the
        # constructor's sigma_pool note.
        self._sigma_history.append(sigma)
        if len(self._sigma_history) > self._sigma_pool:
            self._sigma_history = self._sigma_history[-self._sigma_pool:]
        pooled = np.sqrt(np.mean(
            np.square(np.asarray(self._sigma_history)), axis=0))
        self._cal = Calibration(
            bias=bias,
            sigma=pooled,
            thresholds=np.maximum(self._sigma_k * pooled, self._min_threshold_nm),
            q_ref=np.asarray(self._acc['q']).mean(axis=0),
            samples=tau.shape[0],
        )
        self._acc = None
        return self._cal

    def residual(self, q, tau_meas, dq=None):
        """External joint torque tau_ext, Nm, in ROS joint order.

        ``dq`` is the current joint velocity. Omitted or all-zero, this
        is bit-identical to the original gravity-only estimator. Given a
        genuinely non-zero velocity, the model term switches to the
        velocity-aware ``C(q,dq)*dq + g(q)`` so the Coriolis/centrifugal
        torque of a moving arm (impedance modes 2 and 6) is not mistaken
        for contact. Neither path models the inertial term ``M(q)*ddq``:
        no measured joint acceleration exists anywhere in this project.
        """
        tau = self._model_torque(q, tau_meas, dq)
        if self._cal is not None:
            tau = tau - self._cal.bias
        return tau

    def pose_drift(self, q):
        """Max absolute joint deviation from the calibration pose, rad.

        Joints in ``ignore_joints`` are left out: the bias is what the
        model misses at this pose, and a joint gravity does not depend on
        cannot change that. Measured on hardware, every turn of the base
        made the estimate 'stale' and blocked it for seconds until a new
        calibration completed, though nothing it depends on had moved.
        """
        if self._cal is None:
            return float('inf')
        # Angular, not linear: a joint crossing +-pi has not moved 2*pi.
        drift = np.abs(angular_difference(np.asarray(q, dtype=float),
                                          self._cal.q_ref))
        if self._drift_ignored:
            keep = np.ones(drift.shape, dtype=bool)
            keep[[j for j in self._drift_ignored if j < drift.size]] = False
            drift = drift[keep]
        return float(np.max(drift)) if drift.size else 0.0

    def is_stale(self, q, pose_tolerance=None):
        """True when the arm has left the pose the bias was measured at.

        ``pose_tolerance`` overrides the estimator's own default for this
        one call. The node uses this in a moving mode (2, 6), where the
        arm leaves the calibration pose almost immediately: relaxing the
        tolerance there keeps the feature usable, at the cost of a bias
        that is only approximate — see ``ContactNode.MOVING_POSE_TOLERANCE``
        and the confidence derating applied alongside it.
        """
        tol = self._pose_tolerance if pose_tolerance is None else float(pose_tolerance)
        return self.pose_drift(q) > tol
