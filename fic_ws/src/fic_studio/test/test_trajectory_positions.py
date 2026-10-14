"""Current joint positions come from the right trajectory_data layout for the mode."""
from fic_studio.mixins.joint_state_mixin import parse_trajectory_positions


def _joint_layout(n, q_current):
    # fic_control.h, joint modes: blocks of n; block 0 planner target, block 1 q_current
    data = [0.0] * (8 * n)
    for i in range(n):
        data[i] = 9.0                      # planner target: must not be read as position
        data[i + n] = q_current[i]
    return data


def _cartesian_layout(n, q_current):
    # fic_control.h, Cartesian modes: 8 values per joint, q_current first
    data = [0.0] * (8 * n)
    for i in range(n):
        data[i * 8] = q_current[i]
        data[i * 8 + 1] = 7.0              # q_target
    return data


def test_joint_modes_read_block_one():
    q = [0.1, -0.2, 0.3]
    for mode in (1, 2, 3, 4):
        assert parse_trajectory_positions(_joint_layout(3, q), 3, mode) == q


def test_cartesian_modes_read_every_eighth():
    q = [0.4, 0.5, -0.6]
    for mode in (5, 6, 7, 8, 9):
        assert parse_trajectory_positions(_cartesian_layout(3, q), 3, mode) == q


def test_too_short_or_empty():
    assert parse_trajectory_positions([], 7, 1) is None
    assert parse_trajectory_positions([0.0] * 10, 7, 1) is None
    assert parse_trajectory_positions([0.0] * 20, 7, 5) is None
