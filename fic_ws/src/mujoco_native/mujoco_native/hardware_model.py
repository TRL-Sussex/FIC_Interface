"""The real arm as the simulator reproduces it: robots_config.yaml `hardware_model:`.

Pure functions of (model, block) so they can be unit-tested without ROS; the launch file reads
the block (`load`), simulation_node.py applies it (`apply`) and adds the torque noise and the
command delay it returns.

    hardware_model:
      enabled: true
      joint_friction: [...]     # N m, Coulomb friction per joint      -> MuJoCo frictionloss
      joint_damping: [...]      # N m s/rad, viscous friction per joint -> MuJoCo damping
      armature: [...]           # kg m^2, rotor inertia at each joint   -> MuJoCo armature
      torque_noise: [...]       # N m, standard deviation of the measured joint torque at rest
      command_delay_ms: 0       # torque command applied this late
      torque_filter_hz: 200     # read by fic_control (the measured-torque filter)
      torque_sensor: motor      # measured joint torque: motor (the actuator torque, default) or
                                # output (`link_side_torque`: less the joint's own friction, damping
                                # and rotor inertia, as a sensor after the gearbox reads it)

Which one the Kinova's sensors are closer to is open (15 Oct 2026): `output` reproduces the H-PMP
instability seen on the arm on modes 7/8, `motor` the ~2 N friction effort its wrench estimate shows in
motion (the friction and damping values were fitted with it).

Lists are per joint, in the order of the robot's joint_names; a missing list keeps the MJCF's
own values. Absent block, or enabled: false: nothing changes (the ideal simulator). The
environment variable FIC_HARDWARE_MODEL=off turns it off for one run (fic_control honours it
too), e.g. to reproduce results made with the ideal simulator.
"""
from __future__ import annotations

import os

PER_JOINT = {'joint_friction': 'dof_frictionloss', 'joint_damping': 'dof_damping',
             'armature': 'dof_armature'}


def config_path() -> str | None:
    """robots_config.yaml as the Studio and fic_control read it: $FIC_ROOT/fic_library/config,
    then ../fic_library/config from the working directory (the Studio starts nodes in fic_ws),
    then fic_control's share (a symlink to the same file on Linux)."""
    cands = []
    if os.environ.get('FIC_ROOT'):
        cands.append(os.path.join(os.environ['FIC_ROOT'], 'fic_library', 'config'))
    cands.append(os.path.join(os.getcwd(), '..', 'fic_library', 'config'))
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('fic_control'), 'config'))
    except Exception:
        pass
    for d in cands:
        p = os.path.join(d, 'robots_config.yaml')
        if os.path.isfile(p):
            return os.path.realpath(p)
    return None


def switched_off() -> bool:
    """FIC_HARDWARE_MODEL=off (or 0, false, ideal): the ideal simulator whatever the config says."""
    return os.environ.get('FIC_HARDWARE_MODEL', '').strip().lower() in ('off', '0', 'false', 'ideal')


def load(robot_name: str, path: str | None = None) -> dict:
    """The robot's hardware_model block with its joint names ({} if absent, disabled or switched
    off by FIC_HARDWARE_MODEL)."""
    if switched_off():
        return {}
    path = path or config_path()
    if not path or not robot_name:
        return {}
    import yaml
    with open(path) as f:
        robots = (yaml.safe_load(f) or {}).get('robots', {})
    cfg = robots.get(robot_name) or {}
    hm = cfg.get('hardware_model')
    if not isinstance(hm, dict) or not hm.get('enabled', True):
        return {}
    return dict(hm, joint_names=list(cfg.get('joint_names') or []))


def apply(model, hm: dict, dt: float) -> dict:
    """Set the per-joint friction, damping and armature of the named hinge joints on `model`.
    Returns {'noise': {dof address: std}, 'delay_steps': n, 'applied': [joint names]}."""
    import mujoco
    out = {'noise': {}, 'delay_steps': 0, 'applied': [], 'torque_sensor': 'motor'}
    if not hm:
        return out
    sensor = str(hm.get('torque_sensor') or 'motor').strip().lower()
    out['torque_sensor'] = sensor if sensor in ('motor', 'output') else 'motor'
    names = hm.get('joint_names') or []
    noise = hm.get('torque_noise') or []
    for k, name in enumerate(names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0 or model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        dof = int(model.jnt_dofadr[jid])
        for key, field in PER_JOINT.items():
            vals = hm.get(key)
            if vals is not None and k < len(vals) and vals[k] is not None:
                getattr(model, field)[dof] = float(vals[k])
        if k < len(noise) and noise[k]:
            out['noise'][dof] = float(noise[k])
        out['applied'].append(name)
    delay = float(hm.get('command_delay_ms') or 0.0)
    out['delay_steps'] = max(0, int(round(delay * 1e-3 / dt))) if dt > 0 else 0
    return out


def link_side_torque(model, data, dofs):
    """Joint torque at the output side of each dof in `dofs` (the arm's torque sensors): the actuator
    torque plus the passive forces (damping, negative) plus the dof friction-loss constraint forces,
    less the rotor-inertia torque armature * qacc. At rest it is gravity less the external torque, so
    the joint-torque wrench estimate does not read the joint's own losses as an external wrench."""
    import mujoco
    import numpy as np
    dofs = np.asarray(dofs, dtype=int)
    tau = (np.asarray(data.qfrc_actuator)[dofs] + np.asarray(data.qfrc_passive)[dofs]
           - np.asarray(model.dof_armature)[dofs] * np.asarray(data.qacc)[dofs])
    n = int(data.nefc)
    if n:
        typ = np.asarray(data.efc_type)[:n]
        rows = np.nonzero(typ == int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF))[0]
        if rows.size:
            fric = np.zeros(model.nv)
            np.add.at(fric, np.asarray(data.efc_id)[rows], np.asarray(data.efc_force)[rows])
            tau = tau + fric[dofs]
    return tau
