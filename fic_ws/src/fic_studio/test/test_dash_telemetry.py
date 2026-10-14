"""Tests for the dashboard telemetry parsing helpers."""


def _parse_trajectory_joint(data, n, mode):
    """Mirror of the mixin's joint commanded/effort parse (pure function)."""
    from fic_studio.mixins.joint_state_mixin import parse_trajectory_joint
    return parse_trajectory_joint(data, n, mode)


def test_joint_mode_commanded_and_effort():
    """Joint-mode trajectory data yields correct commanded positions and efforts."""
    n = 2
    # stride i + k*N: build 16 values for 2 joints x 8 fields
    data = [0.0] * (n * 8)
    data[0 + 1 * n], data[1 + 1 * n] = 0.10, 0.20          # actual
    data[0 + 3 * n], data[1 + 3 * n] = 0.15, 0.25          # commanded
    data[0 + 4 * n], data[1 + 4 * n] = 1.5, -2.0           # effort
    cmd, eff = _parse_trajectory_joint(data, n, mode=1)
    assert cmd == [0.15, 0.25]
    assert eff == [1.5, -2.0]


def test_cartesian_mode_commanded():
    """Cartesian-mode trajectory data yields commanded positions per joint."""
    n = 2
    data = [0.0] * (n * 8 + 16)
    data[0 * 8 + 1], data[1 * 8 + 1] = 0.30, 0.40          # q_target per joint
    cmd, eff = _parse_trajectory_joint(data, n, mode=5)
    assert cmd == [0.30, 0.40]
    assert eff == [0.0, 0.0]
