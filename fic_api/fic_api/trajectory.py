# fic_api/trajectory.py — Trajectory data parser (shared with FIC Studio)
#
# /trajectory_data is a Float64MultiArray whose layout depends on the active
# FIC control mode.
#
# JOINT MODES (1-4)  —  block layout:  index = joint_i + block * N
#   Block 0: plannerTargetPos    Block 1: q_current
#   Block 2: error               Block 3: q_target
#   Block 4: controlEffort       Block 5: q_targetAdm   (modes 3-4)
#   Block 6: DesiredEffort       Block 7: current_effort (modes 3-4)
#
# CARTESIAN MODES (5-8)  —  strided layout:  i*8 + offset per joint
#   i*8+0 : q_current[i]        i*8+1 : q_target[i]
#   Cartesian block at base = N*8:
#     +0..2  : Xt_adm (admittance target)
#     +3..5  : current EE position
#     +6..8  : translational error
#     +9..11 : desired EE position
#     +12..14: rotation error (axis-angle xyz)
#     +15    : rotation angle


def parse_trajectory_data(raw: list, num_joints: int = 7) -> dict | None:
    """Parse a raw /trajectory_data float list into a named dict.

    Returns
    -------
    dict with keys:
        is_cartesian, num_joints,
        q_current, q_target, error, control_effort,
        planner_target, q_target_adm, desired_effort, measured_effort,
        ee_position, ee_desired, ee_error, ee_admittance_target,
        rotation_error, rotation_angle, raw
    or None if *raw* is empty.
    """
    n = num_joints
    total = len(raw)
    if total == 0:
        return None

    base = n * 8

    is_cart = False
    if total >= base + 6:
        cart_pos = raw[base + 3: base + 6]
        if any(abs(p) > 0.001 for p in cart_pos):
            is_cart = True

    def _s(idx):
        return raw[idx] if idx < total else 0.0

    result = {'is_cartesian': is_cart, 'num_joints': n, 'raw': raw}

    if is_cart:
        q_current = [_s(i * 8 + 0) for i in range(n)]
        q_target  = [_s(i * 8 + 1) for i in range(n)]
        result['q_current']       = q_current
        result['q_target']        = q_target
        result['error']           = [q_target[i] - q_current[i] for i in range(n)]
        result['control_effort']  = [0.0] * n
        result['planner_target']  = [0.0] * n
        result['q_target_adm']    = [0.0] * n
        result['desired_effort']  = [0.0] * n
        result['measured_effort'] = [0.0] * n
        result['ee_admittance_target'] = [_s(base + i) for i in range(3)]
        result['ee_position']          = [_s(base + 3 + i) for i in range(3)]
        result['ee_error']             = [_s(base + 6 + i) for i in range(3)]
        result['ee_desired']           = [_s(base + 9 + i) for i in range(3)]
        result['rotation_error']       = [_s(base + 12 + i) for i in range(3)]
        result['rotation_angle']       = _s(base + 15)
    else:
        result['planner_target']  = [_s(i + 0 * n) for i in range(n)]
        result['q_current']       = [_s(i + 1 * n) for i in range(n)]
        result['error']           = [_s(i + 2 * n) for i in range(n)]
        result['q_target']        = [_s(i + 3 * n) for i in range(n)]
        result['control_effort']  = [_s(i + 4 * n) for i in range(n)]
        result['q_target_adm']    = [_s(i + 5 * n) for i in range(n)]
        result['desired_effort']  = [_s(i + 6 * n) for i in range(n)]
        result['measured_effort'] = [_s(i + 7 * n) for i in range(n)]
        result['ee_position']          = None
        result['ee_desired']           = None
        result['ee_error']             = None
        result['ee_admittance_target'] = None
        result['rotation_error']       = None
        result['rotation_angle']       = None

    return result
