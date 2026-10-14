"""Find where the end effector is at a given joint pose.

Used to set a robot's default Cartesian target from its home pose. Pinocchio
on the URDF is tried first because that is exactly what the controllers
compute; MuJoCo on the scene is the fallback (the frame must then be a body).
"""
import math


class FkError(Exception):
    """The pose could not be computed; the message is written for the user."""


def _xyzw(w, x, y, z):
    if w < 0:                       # same rotation, easier to read
        w, x, y, z = -w, -x, -y, -z
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return [round(v / n, 4) + 0.0 for v in (x, y, z, w)]


def _pinocchio(urdf, frame, joint_names, q_rad):
    import numpy as np
    import pinocchio as pin
    model = pin.buildModelFromUrdf(urdf)
    if not model.existFrame(frame):
        raise FkError('The URDF has no frame called %s.' % frame)
    q = pin.neutral(model)
    for name, value in zip(joint_names, q_rad):
        if not model.existJointName(name):
            raise FkError('The URDF has no joint called %s.' % name)
        joint = model.joints[model.getJointId(name)]
        if joint.nq == 2:            # continuous: stored as (cos, sin)
            q[joint.idx_q:joint.idx_q + 2] = [math.cos(value), math.sin(value)]
        else:
            q[joint.idx_q] = value
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    pose = data.oMf[model.getFrameId(frame)]
    quat = pin.Quaternion(np.asarray(pose.rotation))
    return [round(float(v), 4) + 0.0 for v in pose.translation], \
        _xyzw(quat.w, quat.x, quat.y, quat.z)


def _mujoco(xml, frame, joint_names, q_rad):
    import mujoco
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)
    for name, value in zip(joint_names, q_rad):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise FkError('The MuJoCo model has no joint called %s.' % name)
        data.qpos[model.jnt_qposadr[jid]] = value
    mujoco.mj_kinematics(model, data)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, frame)
    if bid < 0:
        raise FkError('The MuJoCo model has no body called %s.' % frame)
    w, x, y, z = (float(v) for v in data.xquat[bid])
    return [round(float(v), 4) + 0.0 for v in data.xpos[bid]], _xyzw(w, x, y, z)


def ee_pose(frame, joint_names, q_rad, urdf=None, xml=None):
    """(position [x,y,z], orientation [x,y,z,w]) of ``frame`` at ``q_rad``."""
    reasons = []
    if urdf:
        try:
            return _pinocchio(urdf, frame, joint_names, q_rad)
        except ImportError:
            reasons.append('Pinocchio is not installed')
        except FkError as exc:
            reasons.append(str(exc))
        except Exception as exc:
            reasons.append('Pinocchio could not read the URDF (%s)' % exc)
    if xml:
        try:
            return _mujoco(xml, frame, joint_names, q_rad)
        except ImportError:
            reasons.append('MuJoCo is not installed')
        except FkError as exc:
            reasons.append(str(exc))
        except Exception as exc:
            reasons.append('MuJoCo could not load the model (%s)' % exc)
    raise FkError(' '.join(reasons) or 'No URDF or MuJoCo model is set.')
