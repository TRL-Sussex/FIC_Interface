"""A MuJoCo twin that mirrors the real arm and lights the contacted link.

The twin is a *display*, not a simulation: it writes measured joint angles
into qpos and calls mj_forward. Running mj_step would fight the mirror --
the simulated arm would fall under gravity while this is trying to hold it
at the real arm's pose.

macOS requires this process to run under ``mjpython`` with the viewer on
the main thread; on Linux plain ``python3`` works. See
``mujoco_native/launch/simulation_launch.py`` for the interpreter choice.
"""

import argparse
import threading

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from communication.msg import ContactEstimate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


#: Highlight colour for the contacted link.
HOT_RGBA = np.array([1.0, 0.35, 0.05, 1.0], dtype=np.float32)

#: Marker colour/size for the v2 contact point (a sphere in user_scn).
MARKER_RGBA = np.array([1.0, 0.9, 0.2, 1.0], dtype=np.float32)
MARKER_RADIUS_M = 0.015

#: Colour/width for the force-direction arrow drawn alongside the marker.
ARROW_RGBA = np.array([0.2, 0.8, 1.0, 1.0], dtype=np.float32)
ARROW_WIDTH_M = 0.006
#: Fixed on-screen length, not a to-scale N-to-m mapping: a scale picked
#: for a light touch would send a hard push's arrow off the arm, and one
#: picked for a hard push would make a light touch invisible. The arrow
#: shows only the push DIRECTION, never the force magnitude.
ARROW_LENGTH_M = 0.08


def _select_pose(q_raw, q_filtered):
    """Return the joint vector to mirror, preferring the unfiltered one.

    fic_control low-passes a WRAPPED angle, so a +-pi crossing leaves
    `{rid}_q_current` sweeping the entire circle over ~35 ms (measured
    275 deg in 35 ms, ~7873 deg/s -- far past the arm's real limit). The
    raw topic keeps that wrap an instantaneous 2*pi step, which a hinge
    renders identically, so the spin simply does not appear.

    Falls back to the filtered vector until a raw sample arrives, so a
    twin run against a hardware node predating `{rid}_q_current_raw`
    still mirrors the arm instead of freezing.
    """
    return q_raw if q_raw is not None else q_filtered


class TwinState(Node):
    """Caches the arm's pose and the current contact estimate."""

    def __init__(self, rid):
        """Subscribe to the pose and estimate topics for one station."""
        super().__init__('contact_twin')
        self.lock = threading.Lock()
        self.q = None
        #: Unfiltered pose; preferred over `q` whenever present. See
        #: _select_pose for why the filtered one is unusable at a wrap.
        self.q_raw = None
        self.link = -1
        #: v2-only (see contact_node's `estimator` parameter): the fitted
        #: contact point and force, both in the robot base frame, or None
        #: when v1 is active or the current tick has no fit. See _on_est.
        self.point = None
        self.force = None
        self.create_subscription(
            Float64MultiArray, '%s_q_current' % rid,
            self._on_q, qos_profile_sensor_data)
        self.create_subscription(
            Float64MultiArray, '%s_q_current_raw' % rid,
            self._on_q_raw, qos_profile_sensor_data)
        self.create_subscription(
            ContactEstimate, '%s_contact_estimate' % rid, self._on_est, 10)

    def _on_q(self, msg):
        """Cache the measured joint configuration."""
        with self.lock:
            self.q = np.asarray(msg.data, dtype=float)

    def _on_q_raw(self, msg):
        """Cache the unfiltered joint configuration."""
        with self.lock:
            self.q_raw = np.asarray(msg.data, dtype=float)

    def _on_est(self, msg):
        """Cache the estimated contact link index, point and force.

        `point`/`force` are v2-only (empty `float64[]` fields in v1, and
        in v2 whenever no push-consistent fit was found this tick -- see
        `ContactNode._fit_v2`), so an empty array reads back as None here
        rather than a zero vector, giving the draw loop one clean "is
        there a point" test instead of a magic-zero check.
        """
        with self.lock:
            self.link = int(msg.link_index)
            self.point = (np.asarray(msg.point, dtype=float)
                          if len(msg.point) == 3 else None)
            self.force = (np.asarray(msg.force, dtype=float)
                          if len(msg.force) == 3 else None)


def _link_geom_map(model, link_names):
    """Map link index 1..n to the geom ids belonging to that body.

    ``link_names`` is proximal to distal and becomes 1-based in the
    returned map, matching ``ContactEstimate.link_index``. A name absent
    from the model is skipped rather than raising, so a caller may pass
    names for a robot this particular MJCF does not fully cover.
    """
    out = {}
    for k, name in enumerate(link_names, start=1):
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body < 0:
            continue
        start = model.body_geomadr[body]
        out[k] = list(range(start, start + model.body_geomnum[body]))
    return out


def _derive_link_names(model, joint_names):
    """Return the body each actuated joint carries, proximal to distal.

    The MuJoCo counterpart of ``residual.derive_link_names``, which does
    the same job on the Pinocchio model from the URDF. Both walk joint to
    the body that joint moves, so both number the links identically --
    which is what lets a ``link_index`` from ``contact_node`` mean the
    same body here. Verified equal to ``residual.LINK_NAMES`` on the
    Kinova scene; on any other arm it derives that arm's own names
    instead of silently matching nothing.
    """
    names = []
    for name in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError('joint %r not in the MuJoCo model' % name)
        body = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_BODY, model.jnt_bodyid[jid])
        if body is None:
            raise ValueError('joint %r carries an unnamed body' % name)
        names.append(body)
    return names


def _joint_qpos_map(model, joint_names):
    """Map the robot's joint order to qpos addresses, by name.

    The Kinova scene has nq = 13 -- seven arm joints plus six Robotiq
    gripper joints -- so indexing qpos positionally is a trap.
    """
    adr = []
    for name in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError('joint %r not in the MuJoCo model' % name)
        adr.append(model.jnt_qposadr[jid])
    return np.asarray(adr, dtype=int)


def _draw_contact_marker(scn, link, point, force):
    """Draw the v2 contact-point marker and force-direction arrow.

    Draws into ``scn.geoms``/``scn.ngeom`` -- an ``MjvScene`` DEDICATED
    to this overlay, never one that already holds the physical model's
    own geoms. The live viewer's ``user_scn`` is exactly this: a second,
    separate scene the viewer merges into the final frame at
    ``sync()`` time, so resetting it never touches the model geoms
    ``mj_forward`` produced. ``mujoco.Renderer`` has no such second
    scene -- its ``.scene`` IS the physical one, built fresh by each
    ``update_scene()`` call -- so a headless caller verifying this
    function must build its own throwaway ``mujoco.MjvScene(model,
    maxgeom=...)`` and composite it onto the render scene itself; NEVER
    pass ``renderer.scene`` directly, or the line below wipes every
    physical geom ``update_scene()`` just added. Always resets
    ``scn.ngeom = 0`` first, so a marker from an earlier tick with a fit
    never lingers on a tick with none. Only draws when ``link >= 1``
    AND ``point`` is not None -- -1/0 (no statement / no contact) and a
    v1 or fit-less tick (``point`` read back as None, never a zero
    vector -- see ``TwinState._on_est``) all leave the scene clear.

    The marker's position is one estimate on the contact force's LINE OF
    ACTION, not a uniquely determined point -- see ``hypotheses.
    fit_contact``. The arrow shows only the push DIRECTION at a fixed
    on-screen length (``ARROW_LENGTH_M``); it is not a to-scale N-to-m
    reading of the force magnitude, so its length must never be read as
    the push strength.
    """
    scn.ngeom = 0
    if link < 1 or point is None:
        return
    point = np.asarray(point, dtype=float)

    idx = 0
    mujoco.mjv_initGeom(
        scn.geoms[idx], mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([MARKER_RADIUS_M] * 3, dtype=float), point,
        np.eye(3).flatten(), MARKER_RGBA)
    scn.geoms[idx].emission = 0.6
    idx += 1

    if force is not None and np.linalg.norm(force) > 1e-9:
        direction = np.asarray(force, dtype=float)
        direction = direction / np.linalg.norm(direction)
        # The arrowhead sits at the contact point; the tail is offset
        # back along the reverse of the push -- i.e. towards where the
        # force comes from -- by the fixed display length above.
        tail = point - ARROW_LENGTH_M * direction
        mujoco.mjv_initGeom(
            scn.geoms[idx], mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3), np.zeros(3), np.eye(3).flatten(), ARROW_RGBA)
        mujoco.mjv_connector(
            scn.geoms[idx], mujoco.mjtGeom.mjGEOM_ARROW, ARROW_WIDTH_M,
            tail, point)
        scn.geoms[idx].emission = 0.6
        idx += 1

    scn.ngeom = idx


def main():
    """Run the twin viewer until the window is closed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', required=True)
    parser.add_argument('--robot-id', default='robot1')
    parser.add_argument('--joints', default=','.join(
        'Actuator%d' % i for i in range(1, 8)))
    parser.add_argument(
        '--links', default=None,
        help='comma-separated link body names, proximal to distal; '
             'derived from the MJCF via --joints when omitted, so any '
             'serial arm highlights without a code change')
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    joint_names = args.joints.split(',')
    qadr = _joint_qpos_map(model, joint_names)
    # An explicit --links still wins; deriving is only the default, so a
    # model whose bodies do not follow one-joint-carries-one-link can
    # still be driven by hand.
    link_names = (args.links.split(',') if args.links
                  else _derive_link_names(model, joint_names))
    geoms = _link_geom_map(model, link_names)
    original = model.geom_rgba.copy()

    rclpy.init()
    state = TwinState(args.robot_id)
    spin = threading.Thread(
        target=rclpy.spin, args=(state,), daemon=True)
    spin.start()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                with state.lock:
                    q = _select_pose(state.q_raw, state.q)
                    link = state.link
                    point, force = state.point, state.force
                if q is not None and len(q) >= len(qadr):
                    data.qpos[qadr] = q[:len(qadr)]
                    mujoco.mj_forward(model, data)

                # Only 1..n identifies an actual contacted link: geoms has
                # no entry for -1 (no statement) or 0 (no contact), so both
                # leave the model unhighlighted via this same lookup.
                model.geom_rgba[:] = original
                for gid in geoms.get(link, []):
                    model.geom_rgba[gid] = HOT_RGBA

                # user_scn is the viewer's own separate overlay scene --
                # merged into the render at sync() below, never touching
                # the model geoms just painted above. See
                # _draw_contact_marker's docstring for why this must never
                # be the model's own scene.
                _draw_contact_marker(viewer.user_scn, link, point, force)
                viewer.sync()
    finally:
        state.destroy_node()
        rclpy.try_shutdown()


# fic_studio launches this as ``python3 -m fic_contact.twin``, which runs
# the module body rather than the ``twin`` console_script entry point. The
# guard is what makes that invocation reach main() instead of defining
# these symbols and exiting 0 in silence.
if __name__ == '__main__':
    main()
