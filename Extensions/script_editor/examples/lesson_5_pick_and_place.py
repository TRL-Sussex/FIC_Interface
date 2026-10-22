"""Lesson 5 - A small program (pick and place, with a safety check and a log)

Everything together: the tool and gripper go through a pick-and-place routine
written as a list of steps; a safety check stops the program if the force on the
tool grows too large; and each step is written to a CSV file you can open in a
spreadsheet. (The simulation has no object to pick: the routine is the point.)

Before you run: start the simulation. Mode 6 (Cartesian Impedance with Planner) is set for you.

What you need to know
  a list of tuples       STEPS = [('name', 0.0, 0.0, 0.05, None), ...]
  for name, dx, ... in STEPS:   unpacks each tuple into names
  robot.state.wrench     (Fx, Fy, Fz, Tx, Ty, Tz) in N and N m, or None when not known
  math.sqrt(x)           square root; the size of a force is sqrt(Fx^2 + Fy^2 + Fz^2)
  with open(path, 'w') as f:  f.write('text\\n')   writes a file and closes it afterwards
  robot.motion.open_gripper(), robot.motion.close_gripper()
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
import os
from scipy.spatial.transform import Rotation

robot.mode.hpmp(True)      # H-PMP on (the Studio's default): quick to each goal
robot.mode.set(6)
robot.wait(1.0)
pose = robot.state.ee_pose
if not pose:
    raise RuntimeError('The tool position is not known yet: start the simulation, then run again.')
x0, y0, z0 = pose[0], pose[1], pose[2]
rx, ry, rz = Rotation.from_quat(pose[3:7]).as_euler('xyz')
LOG = os.path.expanduser('~/.fic_studio_scripts/lesson_5_log.csv')
if os.path.exists(LOG):
    os.remove(LOG)         # a log found after the run is this run's
MAX_FORCE_N = 40.0


def move_to(x, y, z):          # from lesson 4
    robot.motion.move_cartesian(x, y, z, rx, ry, rz)


def wait_until_near(x, y, z, tolerance=0.015, timeout=10.0):    # from lesson 4
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not is_stopped():
        now = robot.state.ee_pose
        if now and math.dist((x, y, z), now[:3]) < tolerance:
            return True
        robot.wait(0.05)
    return False


# -- Your turn --------------------------------------------------------------

# TODO 1: finish the routine. Each step is (name, dx, dy, dz, gripper): where the tool goes, in metres
#         from the start, and what the gripper does there ('open', 'close' or None).
#         After these two: lower 5 cm (pick), close, lift back up, move 10 cm to the side (dy),
#         lower, open, lift, and return to the start. Ten steps in all.
STEPS = [
    ('open the gripper', 0.0, 0.0, 0.0, 'open'),
    ('above the object', 0.0, 0.0, 0.0, None),
]


def safe(wrench):
    """True while the force in `wrench` (robot.state.wrench) is below MAX_FORCE_N, or not known."""
    # TODO 2: if wrench is None, return True. Otherwise work out the size of the force
    #         (Fx, Fy, Fz are its first three numbers) and return whether it is below MAX_FORCE_N.
    return True


# TODO 3: write a log. Open LOG for writing (with open(LOG, 'w') as f:), write a header line
#         'seconds,step,x,y,z', then move the loop below inside the `with` block and, after each
#         step, write one line: the seconds since t0, the step's name, and the tool's x, y, z.
t0 = time.monotonic()
done = 0
for name, dx, dy, dz, gripper in STEPS:
    if is_stopped():
        break
    if not safe(robot.state.wrench):
        print(f'Stopped: the force on the tool is over {MAX_FORCE_N} N')
        break
    move_to(x0 + dx, y0 + dy, z0 + dz)
    wait_until_near(x0 + dx, y0 + dy, z0 + dz)
    if gripper == 'open':
        robot.motion.open_gripper()
    elif gripper == 'close':
        robot.motion.close_gripper()
    if gripper:
        robot.wait(0.8)
    done += 1
    print(f'{done:2d}. {name}')

# -- Checks (no need to change anything below) ------------------------------
print()
kinds = [s[4] for s in STEPS]
if len(STEPS) >= 10 and 'close' in kinds and kinds.count('open') >= 2 and STEPS[-1][1:4] == (0.0, 0.0, 0.0):
    print(f'[x] TODO 1: a full routine of {len(STEPS)} steps that ends at the start')
else:
    print(f'[ ] TODO 1: {len(STEPS)} steps so far (open, close, open again, and end at the start)')
if safe((60.0, 0.0, 0.0, 0.0, 0.0, 0.0)) is False and safe((10.0, 5.0, 0.0, 0.0, 0.0, 0.0)) is True:
    print('[x] TODO 2: safe() stops at 60 N and carries on at 11 N')
else:
    print('[ ] TODO 2: safe() should be False for a 60 N force and True for 11 N')
rows = open(LOG).read().splitlines() if os.path.exists(LOG) else []
print(f'[x] TODO 3: {LOG} has a header and {len(rows) - 1} steps' if len(rows) == done + 1 and done else
      '[ ] TODO 3: no log written by this run yet')
