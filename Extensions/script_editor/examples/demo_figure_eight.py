"""Demo - A figure eight with the tool

The tool traces a figure eight in the vertical plane in front of it, twice, while
keeping its orientation. Cartesian Impedance (mode 5): the goal moves smoothly and
the controller follows it, 50 updates a second.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
from scipy.spatial.transform import Rotation

WIDTH_M = 0.06            # left-right half width
HEIGHT_M = 0.03           # up-down half height
PERIOD_S = 6.0
LOOPS = 2

robot.mode.hpmp(True)      # H-PMP on (the Studio's default): quick to each goal
robot.mode.set(5)         # Cartesian Impedance
robot.wait(1.0)
pose = robot.state.ee_pose
if not pose:
    raise RuntimeError('The tool position is not known yet: start the simulation, then run again.')
x0, y0, z0 = pose[:3]
rx, ry, rz = Rotation.from_quat(pose[3:7]).as_euler('xyz')

t0 = time.monotonic()
worst = 0.0
while not is_stopped():
    t = time.monotonic() - t0
    if t > LOOPS * PERIOD_S:
        break
    w = 2 * math.pi * t / PERIOD_S
    gx, gz = y0 + WIDTH_M * math.sin(w), z0 + HEIGHT_M * math.sin(2 * w)
    robot.motion.move_cartesian(x0, gx, gz, rx, ry, rz)
    now = robot.state.ee_pose
    if now and t > 1.0:
        worst = max(worst, math.dist(now[:3], (x0, gx, gz)))
    robot.wait(0.02)
robot.motion.move_cartesian(x0, y0, z0, rx, ry, rz)
robot.wait(1.5)
print(f'Two figure eights; the tool stayed within {worst * 1000:.0f} mm of the moving goal.')
