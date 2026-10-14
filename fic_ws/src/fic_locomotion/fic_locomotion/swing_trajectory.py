"""Swing-foot trajectory: where a foot travels between lift-off and touch-down."""

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]


def swing_foot(start: Vec3, end: Vec3, progress: float,
               step_height: float) -> Vec3:
    """
    Return the foot position `progress` (0..1) of the way through one swing.

    Horizontal motion follows a cycloid so the foot leaves and lands with zero
    horizontal velocity — a linear ramp would scuff the ground on touch-down
    and kick the trunk on lift-off. Vertical motion adds a half sine, which is
    zero at both ends by construction and peaks at mid-swing, so the clearance
    never disturbs where the foot actually lands.
    """
    s = max(0.0, min(1.0, progress))
    blend = s - math.sin(2.0 * math.pi * s) / (2.0 * math.pi)
    return (
        start[0] + (end[0] - start[0]) * blend,
        start[1] + (end[1] - start[1]) * blend,
        start[2] + (end[2] - start[2]) * blend
        + step_height * math.sin(math.pi * s),
    )
