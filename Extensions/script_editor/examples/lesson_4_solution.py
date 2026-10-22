"""Lesson 4 - one solution, to compare with yours once you have tried the lesson.

The lesson: lesson_4_cartesian_square.py
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
from scipy.spatial.transform import Rotation

robot.mode.hpmp(True)      # H-PMP on (the Studio's default): quick to each goal
robot.mode.set(6)          # Cartesian Impedance with Planner
robot.wait(1.0)
pose = robot.state.ee_pose
x0, y0, z0 = pose[0], pose[1], pose[2]
rx, ry, rz = Rotation.from_quat(pose[3:7]).as_euler('xyz')
SIDE = 0.05


def move_to(x, y, z):
    """Send the tool to (x, y, z), keeping its orientation."""
    robot.motion.move_cartesian(x, y, z, rx, ry, rz)


def wait_until_near(x, y, z, tolerance=0.015, timeout=10.0):
    """Wait until the tool is within `tolerance` metres of (x, y, z)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if is_stopped():
            return False
        now = robot.state.ee_pose
        if now and math.dist((x, y, z), now[:3]) < tolerance:
            return True
        robot.wait(0.05)
    return False


CORNERS = [(x0, y0), (x0 + SIDE, y0), (x0 + SIDE, y0 + SIDE), (x0, y0 + SIDE), (x0, y0)]
visited = 0
for cx, cy in CORNERS:
    if is_stopped():
        break
    move_to(cx, cy, z0)
    if wait_until_near(cx, cy, z0):
        visited += 1
        print(f'corner {visited}: x={cx:.3f}  y={cy:.3f}')
print(f'{visited} of {len(CORNERS)} corners reached')
