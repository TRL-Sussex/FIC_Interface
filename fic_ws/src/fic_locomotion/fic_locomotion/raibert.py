"""
Raibert-style foot placement.

The classic hopping-robot heuristic: to hold a velocity, land the foot half a
stance period ahead of its neutral point along the velocity vector, so the foot
sits under the centre of mass at mid-stance and the trunk neither accelerates
nor brakes. A term proportional to the velocity *error* on top of that is what
actually stabilises the gait — it is the only feedback a kinematic trot has
against drift and disturbance.
"""

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]


def touchdown_target(nominal: Vec3, v_cmd: Tuple[float, float],
                     v_meas: Tuple[float, float], yaw_rate: float,
                     stance_time: float, gain: float = 0.06) -> Vec3:
    """Return where this leg should touch down, in the trunk frame."""
    x = nominal[0] + 0.5 * stance_time * v_cmd[0] + gain * (v_meas[0] - v_cmd[0])
    y = nominal[1] + 0.5 * stance_time * v_cmd[1] + gain * (v_meas[1] - v_cmd[1])
    # Turning: the neutral foot point sweeps an arc about the trunk centre over
    # a stance phase, so land on the far end of that arc.
    theta = 0.5 * stance_time * yaw_rate
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return (x * cos_t - y * sin_t, x * sin_t + y * cos_t, nominal[2])
