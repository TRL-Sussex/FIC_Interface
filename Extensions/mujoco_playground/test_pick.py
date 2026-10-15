"""The pick sequence: open, above pointing down, down to the object, close, lift."""
import math

import pytest

from Extensions.mujoco_playground import pick


def _rot(aa):
    x, y, z, deg = aa
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    a = math.radians(deg)
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def test_the_orientation_points_the_kinova_gripper_down():
    g = pick.GRASP['KinovaGen3_7DOF']
    R = _rot(pick.top_down_axis_angle(g))
    approach = [sum(R[i][j] * g['approach'][j] for j in range(3)) for i in range(3)]
    opening = [sum(R[i][j] * g['opening'][j] for j in range(3)) for i in range(3)]
    assert approach == pytest.approx([0, 0, -1], abs=1e-6)
    assert opening == pytest.approx([0, 1, 0], abs=1e-6)


def test_the_pick_sequence():
    wps = pick.pick_waypoints('KinovaGen3_7DOF', [0.5, -0.15, 0.221])
    assert [w['type'] for w in wps] == ['gripper', 'cartesian', 'cartesian', 'gripper', 'cartesian']
    assert wps[0]['width'] == 0.08 and wps[3]['width'] == 0.0
    zs = [w['position'][2] for w in wps if w['type'] == 'cartesian']
    assert zs[1] == pytest.approx(0.221 + 0.115 - pick.AIM_BELOW) and zs[0] > zs[1] and zs[2] > zs[1]
    assert all(w['position'][:2] == [0.5, -0.15] for w in wps if w['type'] == 'cartesian')


def test_a_robot_without_a_gripper_cannot_pick():
    with pytest.raises(ValueError, match='no gripper'):
        pick.pick_waypoints('Franka_FR3', [0.5, 0, 0.2])


def test_the_pick_starts_from_home_when_given():
    wps = pick.pick_waypoints('KinovaGen3_7DOF', [0.5, -0.15, 0.221], home_rad=[0, 0.2618, -3.1416, -2.2654, 0, 1.0402, 1.5708])
    assert wps[0]['type'] == 'joint' and wps[0]['values'][1] == pytest.approx(15.0, abs=0.01)
    assert [w['type'] for w in wps[1:]] == ['gripper', 'cartesian', 'cartesian', 'gripper', 'cartesian']
