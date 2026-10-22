"""Lesson 5 - one solution, to compare with yours once you have tried the lesson.

The lesson: lesson_5_pick_and_place.py
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
x0, y0, z0 = pose[0], pose[1], pose[2]
rx, ry, rz = Rotation.from_quat(pose[3:7]).as_euler('xyz')
LOG = os.path.expanduser('~/.fic_studio_scripts/lesson_5_log.csv')
MAX_FORCE_N = 40.0


def move_to(x, y, z):
    robot.motion.move_cartesian(x, y, z, rx, ry, rz)


def wait_until_near(x, y, z, tolerance=0.015, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not is_stopped():
        now = robot.state.ee_pose
        if now and math.dist((x, y, z), now[:3]) < tolerance:
            return True
        robot.wait(0.05)
    return False


STEPS = [
    ('open the gripper', 0.0, 0.0, 0.0, 'open'),
    ('above the object', 0.0, 0.0, 0.0, None),
    ('down to the object', 0.0, 0.0, -0.05, None),
    ('close on it', 0.0, 0.0, -0.05, 'close'),
    ('lift it', 0.0, 0.0, 0.0, None),
    ('carry it to the side', 0.0, 0.10, 0.0, None),
    ('lower it', 0.0, 0.10, -0.05, None),
    ('let go', 0.0, 0.10, -0.05, 'open'),
    ('lift away', 0.0, 0.10, 0.0, None),
    ('back to the start', 0.0, 0.0, 0.0, None),
]


def safe(wrench):
    """True while the force in `wrench` is below MAX_FORCE_N, or not known."""
    if wrench is None:
        return True
    fx, fy, fz = wrench[0], wrench[1], wrench[2]
    return math.sqrt(fx * fx + fy * fy + fz * fz) < MAX_FORCE_N


t0 = time.monotonic()
done = 0
with open(LOG, 'w') as f:
    f.write('seconds,step,x,y,z\n')
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
        x, y, z = robot.state.ee_pose[:3]
        f.write(f'{time.monotonic() - t0:.2f},{name},{x:.4f},{y:.4f},{z:.4f}\n')
        print(f'{done:2d}. {name}')
print(f'Log: {LOG}')
