"""Interactive MuJoCo rig: push the arm with the mouse, in the real stack.

Stands in for `fic_control`, so the whole chain runs with no robot:
this node publishes the same topics `contact_node` consumes, the twin
mirrors it, and `following` acts on it. Ctrl + right-drag a link in the
viewer to push it.

    ros2 run fic_contact simtouch --ros-args -p robot_name:=KinovaGen3_7DOF

**How the arm responds.** Not full dynamics. The arm is held at a joint
target by an impedance law and deflects to the equilibrium that law
predicts:

    q = q_target + tau_ext / K

which is what a stiff impedance-controlled arm actually does under a
push -- it yields a little and holds. That also makes the rig faithful to
both estimator channels at once: the torque channel sees
``g(q) + tau_ext`` and the deflection channel sees the same ``tau_ext``
through ``K (q - q_target)``, exactly as on hardware.

**Ground truth.** Because the push comes from the viewer's own
perturbation, the rig knows the true contacted body, the true application
point and the true force. It publishes them on ``{rid}_contact_truth``
(Float64MultiArray: ``[link, px, py, pz, fx, fy, fz]``, link 0 for no
contact) so an estimate can be scored live against the push that caused
it -- the thing hardware cannot give you.

Headless mode (``-p headless:=true``) runs the same loop with no window
and lets a test drive the perturbation programmatically.
"""

import numpy as np

import mujoco

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float64MultiArray, Int32

from fic_contact.deflection import KINOVA_STIFFNESS_NM_PER_RAD

#: Hz. Matches fic_control's published state rate closely enough that
#: contact_node's 100 Hz tick always has fresh data.
RATE_HZ = 200.0

#: Nm, per joint, Gaussian. Roughly the Kinova's torque noise floor; the
#: estimator's thresholds are derived from whatever it measures here, so
#: this sets the detection floor the same way it does on hardware.
TORQUE_NOISE_NM = 0.2

#: rad, per joint, Gaussian. Encoder noise, which is what limits the
#: deflection channel.
ENCODER_NOISE_RAD = 1e-4


class SimTouch(Node):
    """Publish arm state and torques for a mouse-driven contact."""

    def __init__(self):
        """Load the model, resolve the chain, and open the publishers."""
        super().__init__('simtouch')
        self.declare_parameter('station_id', 1)
        self.declare_parameter('robot_name', 'KinovaGen3_7DOF')
        self.declare_parameter('mjcf', '')
        self.declare_parameter('headless', False)
        self.declare_parameter('torque_noise_nm', TORQUE_NOISE_NM)
        self.declare_parameter('encoder_noise_rad', ENCODER_NOISE_RAD)
        self.declare_parameter('control_mode', 5)
        self.declare_parameter(
            'joints', ['Actuator%d' % i for i in range(1, 8)])
        self.declare_parameter('q_home', [0.0, 0.6, 0.0, -1.2, 0.0, 0.9, 0.0])
        self.declare_parameter(
            'joint_stiffness', list(KINOVA_STIFFNESS_NM_PER_RAD))

        sid = int(self.get_parameter('station_id').value)
        self._rid = 'robot%d' % sid
        mjcf = str(self.get_parameter('mjcf').value)
        if not mjcf:
            raise ValueError(
                'mjcf parameter is required: pass the scene.xml path')

        self._model = mujoco.MjModel.from_xml_path(mjcf)
        self._data = mujoco.MjData(self._model)
        self._joints = list(self.get_parameter('joints').value)
        self._n = len(self._joints)

        self._qadr, self._dof, self._bodies = [], [], []
        for name in self._joints:
            jid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError('joint %r not in the MJCF' % name)
            self._qadr.append(int(self._model.jnt_qposadr[jid]))
            self._dof.append(int(self._model.jnt_dofadr[jid]))
            self._bodies.append(int(self._model.jnt_bodyid[jid]))
        self._qadr = np.asarray(self._qadr, dtype=int)
        self._dof = np.asarray(self._dof, dtype=int)

        stiffness = list(self.get_parameter('joint_stiffness').value)
        self._k = np.asarray(stiffness[:self._n], dtype=float)
        self._q_target = np.asarray(
            list(self.get_parameter('q_home').value)[:self._n], dtype=float)
        self._rng = np.random.default_rng(0)

        self._pub_q = self.create_publisher(
            Float64MultiArray, '%s_q_current' % self._rid, 10)
        self._pub_q_raw = self.create_publisher(
            Float64MultiArray, '%s_q_current_raw' % self._rid,
            qos_profile_sensor_data)
        self._pub_target = self.create_publisher(
            Float64MultiArray, '%s_q_target' % self._rid,
            qos_profile_sensor_data)
        self._pub_tau = self.create_publisher(
            Float64MultiArray, '%s_joint_torques' % self._rid,
            qos_profile_sensor_data)
        self._pub_vel = self.create_publisher(
            Float64MultiArray, '%s_joint_velocities' % self._rid, 10)
        self._pub_mode = self.create_publisher(
            Int32, '%s_control_mode' % self._rid, 10)
        self._pub_truth = self.create_publisher(
            Float64MultiArray, '%s_contact_truth' % self._rid, 10)
        self._pub_desired = self.create_publisher(
            Float64MultiArray, '%s_desired_joint_angles_f64' % self._rid, 10)

        self.get_logger().info(
            'simtouch ready for %s on %s -- ctrl + right-drag a link to '
            'push it. Publishing q_current, q_current_raw, q_target, '
            'joint_torques and ground truth on %s_contact_truth.'
            % (self.get_parameter('robot_name').value, self._rid, self._rid))

    # ── model helpers ─────────────────────────────────────────────────

    def _link_of_body(self, body):
        """Return the 1-based link index for a MuJoCo body, or 0.

        A push may land on a body that is not itself an actuated link --
        a gripper finger, say. Walking up to the nearest ancestor that is
        one attributes the contact to the link that actually carries it,
        which is the same attribution the estimator can make.
        """
        node = int(body)
        while node > 0:
            if node in self._bodies:
                return self._bodies.index(node) + 1
            node = int(self._model.body_parentid[node])
        return 0

    def link_point(self, body):
        """Return a world point on ``body`` at the current target pose.

        Exposed for tests and for scripted pushes: the viewer supplies
        its own application point, but a headless caller needs one.
        """
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0
        self._data.qpos[self._qadr] = self._q_target
        mujoco.mj_forward(self._model, self._data)
        origin = np.asarray(self._data.xpos[int(body)], dtype=float)
        rot = np.asarray(
            self._data.xmat[int(body)], dtype=float).reshape(3, 3)
        return origin + rot @ np.array([0.04, 0.0, 0.0])

    def _gravity(self, q):
        """Return MuJoCo's gravity torque at ``q``, ROS order."""
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0
        self._data.qpos[self._qadr] = q
        mujoco.mj_forward(self._model, self._data)
        return np.asarray(self._data.qfrc_bias)[self._dof].copy()

    def _contact_torque(self, q, body, point, force):
        """Return ``J^T f`` for a world-frame push, ROS order."""
        self._data.qpos[:] = 0.0
        self._data.qvel[:] = 0.0
        self._data.qpos[self._qadr] = q
        mujoco.mj_forward(self._model, self._data)
        qfrc = np.zeros(self._model.nv, dtype=float)
        mujoco.mj_applyFT(
            self._model, self._data,
            np.asarray(force, dtype=float).reshape(3), np.zeros(3),
            np.asarray(point, dtype=float).reshape(3), int(body), qfrc)
        return qfrc[self._dof].copy()

    # ── one cycle ─────────────────────────────────────────────────────

    def step(self, body=0, point=None, force=None):
        """Advance one cycle for a given push and publish everything.

        Split out from the viewer loop so a test can drive it with a
        synthetic perturbation and assert on what gets published, with no
        window and no mouse.
        """
        link = self._link_of_body(body) if body else 0
        tau_ext = np.zeros(self._n)
        if link and point is not None and force is not None:
            tau_ext = self._contact_torque(
                self._q_target, body, point, force)

        # Impedance equilibrium: the arm yields until its restoring
        # torque balances the push. This is also what makes the
        # deflection channel work against this rig.
        q = self._q_target + tau_ext / self._k
        noise_q = float(self.get_parameter('encoder_noise_rad').value)
        q_meas = q + self._rng.normal(0.0, noise_q, size=q.shape) \
            if noise_q else q

        noise_t = float(self.get_parameter('torque_noise_nm').value)
        tau = self._gravity(q) + tau_ext
        if noise_t:
            tau = tau + self._rng.normal(0.0, noise_t, size=tau.shape)

        self._publish(self._pub_q, q_meas)
        self._publish(self._pub_q_raw, q_meas)
        self._publish(self._pub_target, self._q_target)
        self._publish(self._pub_tau, tau)
        self._publish(self._pub_vel, np.zeros(self._n))
        self._pub_mode.publish(
            Int32(data=int(self.get_parameter('control_mode').value)))

        truth = [float(link)]
        truth += list(np.asarray(point, dtype=float)) if link else [0.0] * 3
        truth += list(np.asarray(force, dtype=float)) if link else [0.0] * 3
        self._publish(self._pub_truth, truth)
        return {'link': link, 'q': q, 'tau': tau, 'tau_ext': tau_ext}

    @staticmethod
    def _publish(publisher, values):
        """Publish a Float64MultiArray of ``values``."""
        msg = Float64MultiArray()
        msg.data = [float(v) for v in np.asarray(values).reshape(-1)]
        publisher.publish(msg)


def _read_perturbation(viewer, model, data):
    """Return ``(body, world_point, world_force)`` for the mouse push.

    The viewer owns the perturbation state; ``mjv_applyPerturbForce``
    turns it into ``xfrc_applied``, which is where the force and its
    application point can be read back. Returns ``(0, None, None)`` when
    nobody is dragging.
    """
    with viewer.lock():
        pert = viewer.perturb
        active = int(pert.active)
        select = int(pert.select)
        localpos = np.asarray(pert.localpos, dtype=float).copy()
    if not active or select <= 0:
        return 0, None, None

    data.xfrc_applied[:] = 0.0
    mujoco.mjv_applyPerturbForce(model, data, viewer.perturb)
    force = np.asarray(data.xfrc_applied[select][:3], dtype=float).copy()
    data.xfrc_applied[:] = 0.0
    if not np.any(force):
        return 0, None, None

    origin = np.asarray(data.xpos[select], dtype=float)
    rot = np.asarray(data.xmat[select], dtype=float).reshape(3, 3)
    return select, origin + rot @ localpos, force


def main(args=None):
    """Run the rig, with a viewer unless headless."""
    rclpy.init(args=args)
    node = SimTouch()
    headless = bool(node.get_parameter('headless').value)
    period = 1.0 / RATE_HZ
    try:
        if headless:
            timer_state = {}

            def _tick():
                """Publish a quiet cycle; tests call step() directly."""
                timer_state['last'] = node.step()

            node.create_timer(period, _tick)
            rclpy.spin(node)
        else:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(
                    node._model, node._data) as viewer:
                while viewer.is_running() and rclpy.ok():
                    body, point, force = _read_perturbation(
                        viewer, node._model, node._data)
                    out = node.step(body=body, point=point, force=force)
                    # Show the deflected pose, so the arm visibly yields
                    # under the push the way a stiff arm does.
                    node._data.qpos[node._qadr] = out['q']
                    mujoco.mj_forward(node._model, node._data)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    viewer.sync()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
