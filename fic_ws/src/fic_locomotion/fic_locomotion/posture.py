"""
Trunk posture correction applied through the stance feet.

A kinematic gait's only authority over the trunk is where it puts the feet, so
height and attitude errors are corrected by pushing planted feet up or down in
the trunk frame. Swing feet are excluded — they are not carrying load and
moving them does nothing to the trunk.
"""

from dataclasses import dataclass
from typing import Tuple

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class PostureGains:
    """Proportional gains and authority limit for posture correction."""

    height: float = 0.6
    roll: float = 0.5
    pitch: float = 0.5
    max_offset: float = 0.08


def stance_offset(foot: Vec3, height_error: float, roll: float, pitch: float,
                  gains: PostureGains) -> float:
    """
    Return the vertical offset in metres to add to a stance foot's trunk-frame z.

    `height_error` is measured minus desired trunk height: positive means the
    trunk is riding high, so the feet are drawn up toward it and the trunk
    settles.

    For small angles a foot at trunk-frame (x, y) sits at world height
    `z + roll * y - pitch * x`. Subtracting that term keeps the foot planted at
    a constant world height while the trunk rotates back to level, which is
    what actually levels the robot.

    The result is clamped so a large transient error — a stumble, a bad
    odometry frame — can never command a foot through the floor.
    """
    dz = gains.height * height_error
    dz += gains.pitch * pitch * foot[0]
    dz -= gains.roll * roll * foot[1]
    return max(-gains.max_offset, min(gains.max_offset, dz))
