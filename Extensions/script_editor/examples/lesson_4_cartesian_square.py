"""Lesson 4 - Moving the tool in space (draw a square)

So far you moved joints. Now you move the tool itself: its position in metres,
in the robot's base frame (x forward, y left, z up). You write two functions,
then use them to draw a 5 cm square in the horizontal plane.

Before you run: start the simulation. The lesson switches to "Cartesian
Impedance with Planner" (mode 6), which plans a smooth path to each goal.

What you need to know
  robot.state.ee_pose                [x, y, z, qx, qy, qz, qw] (position in m, orientation as a quaternion)
  robot.motion.move_cartesian(x, y, z, rx, ry, rz)
                                     sends the tool to (x, y, z) with orientation rx, ry, rz
                                     (radians, XYZ Euler angles). It does not wait.
  math.dist(p, q)                    the distance between two points, e.g. math.dist((x, y, z), pose[:3])
  time.monotonic()                   a clock in seconds, for timeouts
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
from scipy.spatial.transform import Rotation

robot.mode.hpmp(True)      # H-PMP on (the Studio's default): the tool gets to each goal quickly
robot.mode.set(6)          # Cartesian Impedance with Planner
robot.wait(1.0)
pose = robot.state.ee_pose
if not pose:
    raise RuntimeError('The tool position is not known yet: start the simulation, then run again.')
x0, y0, z0 = pose[0], pose[1], pose[2]
# Keep the tool's orientation as it is now: its quaternion turned into the angles move_cartesian takes.
rx, ry, rz = Rotation.from_quat(pose[3:7]).as_euler('xyz')
print(f'Start: x={x0:.3f}  y={y0:.3f}  z={z0:.3f} m')

SIDE = 0.05                # the square's side, metres

# -- Your turn --------------------------------------------------------------


def move_to(x, y, z):
    """Send the tool to (x, y, z), keeping its orientation (rx, ry, rz)."""
    # TODO 1: one line: call robot.motion.move_cartesian with x, y, z and rx, ry, rz.
    pass


def wait_until_near(x, y, z, tolerance=0.015, timeout=10.0):
    """Wait until the tool is within `tolerance` metres of (x, y, z) (the arm settles within about a centimetre). True if it got there in time."""
    # TODO 2: until `timeout` seconds have passed (use time.monotonic()):
    #           read robot.state.ee_pose, and return True if math.dist((x, y, z), pose[:3]) < tolerance;
    #           return False if is_stopped(); otherwise robot.wait(0.05) and look again.
    #         After the timeout, return False.
    return False


# TODO 3: the square's corners as (x, y) pairs, starting and ending at (x0, y0),
#         e.g. [(x0, y0), (x0 + SIDE, y0), ...]. Five points: the start is also the end.
CORNERS = []

visited = 0
for cx, cy in CORNERS:
    if is_stopped():
        break
    move_to(cx, cy, z0)
    if wait_until_near(cx, cy, z0):
        visited += 1
        print(f'corner {visited}: x={cx:.3f}  y={cy:.3f}')

# -- Checks (no need to change anything below) ------------------------------
print()
if not CORNERS:
    print('[ ] TODO 3: CORNERS is empty')
else:
    xs, ys = [c[0] for c in CORNERS], [c[1] for c in CORNERS]
    square = (len(CORNERS) == 5 and CORNERS[0] == CORNERS[-1] == (x0, y0)
              and abs(max(xs) - min(xs) - SIDE) < 1e-6 and abs(max(ys) - min(ys) - SIDE) < 1e-6)
    print('[x] TODO 3: the corners make a square that starts and ends at the start' if square else
          '[ ] TODO 3: the corners are not a closed square of side SIDE yet')
    print(f'[x] TODO 1 and 2: all {visited} corners reached' if visited == len(CORNERS) else
          f'[ ] TODO 1 and 2: {visited} of {len(CORNERS)} corners reached')
end = robot.state.ee_pose
if visited and math.dist(end[:3], (x0, y0, z0)) < 0.02:
    print('[x] the tool is back where it started')
