"""
Tests for parse_cartesian_error: extracting the Cartesian error block.

trajectory_data layout: N*8 joint floats, then a >=16-float Cartesian block
whose error x/y/z are at offsets 6,7,8 and orientation angle at offset 15.
"""
from fic_studio.mixins.joint_state_mixin import parse_cartesian_error


def test_extracts_error_xyz_and_angle():
    n = 7
    data = [0.0] * (n * 8)
    cart = [0.0] * 16
    cart[6], cart[7], cart[8], cart[15] = 0.011, -0.022, 0.033, 0.044
    data += cart
    assert parse_cartesian_error(data, n) == (0.011, -0.022, 0.033, 0.044)


def test_returns_none_when_no_cartesian_block():
    n = 7
    assert parse_cartesian_error([0.0] * (n * 8), n) is None      # the joint modes: 8 blocks of n


def test_returns_none_on_empty_or_short():
    assert parse_cartesian_error([], 7) is None
    assert parse_cartesian_error([0.0] * 10, 7) is None


class _Node:
    station_robot_id = 'robot1'


class _Stub:
    """Minimal carrier for the cartesian-error storage + helper."""

    def __init__(self):
        from fic_studio.mixins.joint_state_mixin import JointStateMixin
        self.current_cartesian_error_by_robot = {}
        self.node = _Node()
        self.cartesian_error_for = JointStateMixin.cartesian_error_for.__get__(self)


def test_cartesian_error_for_returns_stored_tuple():
    s = _Stub()
    s.current_cartesian_error_by_robot['robot1'] = (0.1, 0.2, 0.3, 0.4)
    assert s.cartesian_error_for() == (0.1, 0.2, 0.3, 0.4)
    assert s.cartesian_error_for('robot2') is None
