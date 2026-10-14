"""fic_api — Python API for FIC-controlled robots.

Usage:
    from fic_api import Robot

    with Robot(target='simulation') as r:
        r.home()
        q = r.get_current_joints()
        print(q)

    # Or without context manager:
    r = Robot(target='hardware', num_joints=7)
    r.move_joints([0, -45, 0, -90, 0, 45, 0])
    r.close()
"""

from .robot import Robot
from .trajectory import parse_trajectory_data

__version__ = "0.1.0"
__all__ = ["Robot", "parse_trajectory_data"]
