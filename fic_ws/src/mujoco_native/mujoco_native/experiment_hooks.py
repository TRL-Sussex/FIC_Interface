"""Experiment hooks for the simulated experiment rigs. Pure functions of (model, data)
so they can be unit-tested without ROS; simulation_node.py wires them to topics."""
import numpy as np
import mujoco

_saved_flags = {}   # (id(model), geom id) -> (contype, conaffinity), captured on first set_wall


def ramp_scale(t, t0, ramp_s, hold_s):
    if t < t0:
        return 0.0
    s = t - t0
    if ramp_s > 0 and s < ramp_s:
        return s / ramp_s
    s -= ramp_s
    if s < hold_s:
        return 1.0
    s -= hold_s
    if ramp_s > 0 and s < ramp_s:
        return 1.0 - s / ramp_s
    return 0.0


def set_wall(model, data, body, geom, enabled, pos, quat):
    bid = model.body(body).id
    mid = model.body_mocapid[bid]
    if mid < 0:
        raise ValueError(f"body {body!r} is not a mocap body")
    data.mocap_pos[mid] = np.asarray(pos, float)
    data.mocap_quat[mid] = np.asarray(quat, float)
    gid = model.geom(geom).id
    key = (id(model), gid)
    if key not in _saved_flags:
        _saved_flags[key] = (int(model.geom_contype[gid]), int(model.geom_conaffinity[gid]))
    ct, ca = _saved_flags[key] if enabled else (0, 0)
    model.geom_contype[gid] = ct
    model.geom_conaffinity[gid] = ca


def wall_contact_force(model, data, geom):
    """World-frame force ON the wall geom, summed over its contacts."""
    gid = model.geom(geom).id
    total = np.zeros(3)
    f6 = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if c.geom1 != gid and c.geom2 != gid:
            continue
        mujoco.mj_contactForce(model, data, i, f6)
        frame = np.asarray(c.frame).reshape(3, 3)   # rows = contact axes in world
        f_world = frame.T @ f6[:3]                  # force on geom2 from geom1, world
        total += f_world if c.geom2 == gid else -f_world
    return total


def apply_link_force(model, data, body, force_w, point_w, scale):
    """Set xfrc_applied (world frame, about body CoM). mj_step applies it itself;
    qfrc_applied is deliberately untouched."""
    bid = model.body(body).id
    data.xfrc_applied[bid] = 0.0
    if scale == 0.0:
        return
    f = float(scale) * np.asarray(force_w, float)
    torque = np.cross(np.asarray(point_w, float) - data.xipos[bid], f)
    data.xfrc_applied[bid] = np.concatenate([f, torque])


def body_positions(model, data, bodies):
    return np.array([data.xpos[model.body(b).id] for b in bodies])


def site_wrench(model, data, force_sensor, torque_sensor):
    fa = model.sensor(force_sensor).adr[0]
    ta = model.sensor(torque_sensor).adr[0]
    return np.concatenate([data.sensordata[fa:fa + 3], data.sensordata[ta:ta + 3]])


def step_push(model, data, push, prev_body):
    """One pre-step update of an active push (call before mj_step).

    push: dict(body, force, point, ramp_s, hold_s, t0) or None; t0 None is
    stamped here with data.time (first sight, sim thread). prev_body: body the
    previous call left a force on. Clears prev_body's xfrc_applied if the push
    switched body or ended. Returns (active_push_or_None, active_body_or_None,
    truth_or_None) where truth = [t, body_id, fx, fy, fz]; a returned None push
    means the request is finished and should be dropped.
    """
    zero = (0.0, 0.0, 0.0)
    if prev_body is not None and (push is None or push["body"] != prev_body):
        apply_link_force(model, data, prev_body, zero, zero, 0.0)
    if push is None:
        return None, None, None
    if push.get("t0") is None:
        push["t0"] = float(data.time)
    t = data.time
    s = ramp_scale(t, push["t0"], push["ramp_s"], push["hold_s"])
    if s == 0.0 and t > push["t0"] + 2 * push["ramp_s"] + push["hold_s"]:
        apply_link_force(model, data, push["body"], zero, zero, 0.0)
        return None, None, None
    apply_link_force(model, data, push["body"], push["force"], push["point"], s)
    f = np.asarray(push["force"], float) * s
    return push, push["body"], [float(t), float(model.body(push["body"]).id), *map(float, f)]
