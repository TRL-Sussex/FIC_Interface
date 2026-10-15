"""A pick as a sequence: open, above the object pointing down, down to it, close, lift.

Built from the object's pose (the simulator's object_poses) so that the learner can play it, step
through it and edit it on the Sequences page. Pure functions: no ROS, no Qt.
"""
import math

#: Per robot with a gripper in simulation: the gripper's axes in the controller's end-effector
#: frame (approach: from the wrist to the finger pads; opening: the way the fingers open), the
#: distance from the end-effector origin to the middle of the finger pads, and the open width.
#: Kinova Gen3 + Robotiq 2F-85, measured in its MuJoCo model against the controller's frame.
GRASP = {
    'KinovaGen3_7DOF': {'approach': (0.0, 0.0, 1.0), 'opening': (1.0, 0.0, 0.0), 'pad_offset': 0.115,
                        'open_width': 0.08},
}
ABOVE = 0.12        # m above the grasp height to come down from
#: The grasp target is this much below the height that centres the pads on the object: an
#: impedance-controlled arm stops short of a target (joint friction, gravity), by about 30-40 mm
#: when reaching down in the simulator run like the real arm, so it aims into the object, as one
#: does with a compliant controller; if it does get there, the fingers simply stop on the table.
AIM_BELOW = 0.02
LIFT = 0.15         # m to lift the object


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def top_down_axis_angle(grasp):
    """Axis-angle (deg) of the end-effector orientation that points the gripper straight down,
    its fingers opening along the base frame's y axis."""
    # The end-effector axes, in the world, that put approach -> -z and opening -> +y.
    want = {'approach': (0.0, 0.0, -1.0), 'opening': (0.0, 1.0, 0.0)}
    a_e, o_e = grasp['approach'], grasp['opening']
    t_e = _cross(a_e, o_e)
    t_w = _cross(want['approach'], want['opening'])
    # R maps the end-effector frame to the world: R = W E^T with E, W = [opening, third, approach].
    E = [o_e, t_e, a_e]
    W = [want['opening'], t_w, want['approach']]
    R = [[sum(W[k][i] * E[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    angle = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0)))
    if angle < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    if abs(angle - math.pi) < 1e-6:                 # 180 deg: the axis from the diagonal
        x = math.sqrt(max(0.0, (R[0][0] + 1) / 2))
        y = math.sqrt(max(0.0, (R[1][1] + 1) / 2)) * (1 if R[0][1] >= 0 else -1)
        z = math.sqrt(max(0.0, (R[2][2] + 1) / 2)) * (1 if R[0][2] >= 0 else -1)
        return [x, y, z, 180.0]
    s = 2.0 * math.sin(angle)
    return [(R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, math.degrees(angle)]


def pick_waypoints(robot, position, hold=1.0, home_rad=None):
    """The pick of an object at ``position`` [x, y, z] (its centre) by ``robot``, as sequence
    waypoints, starting from the robot's home joints ``home_rad`` when given (a known posture, from
    which turning the gripper down does not wind a continuous wrist joint round). ValueError when
    the robot has no gripper in simulation."""
    g = GRASP.get(robot)
    if g is None:
        raise ValueError('%s has no gripper in simulation: it cannot pick' % robot)
    aa = top_down_axis_angle(g)
    x, y, z = (float(v) for v in position)
    grasp_z = z + g['pad_offset'] - AIM_BELOW

    def cart(zz, h, **extra):
        return dict({'type': 'cartesian', 'position': [round(x, 4), round(y, 4), round(zz, 4)],
                     'axis_angle': [round(v, 6) for v in aa], 'hold_time': h}, **extra)
    start = ([{'type': 'joint', 'values': [round(math.degrees(q), 3) for q in home_rad], 'hold_time': hold}]
             if home_rad else [])
    return start + [{'type': 'gripper', 'width': g['open_width'], 'hold_time': hold},
            cart(grasp_z + ABOVE, hold),
            # reached within 45 mm, after 8 s at most: the arm stops on the object or short of it
            cart(grasp_z, hold / 2, tolerance_mm=45, timeout_s=8),
            {'type': 'gripper', 'width': 0.0, 'hold_time': hold},
            cart(grasp_z + LIFT, hold)]
