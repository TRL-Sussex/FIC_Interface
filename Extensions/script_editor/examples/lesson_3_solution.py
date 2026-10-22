"""Lesson 3 - one solution, to compare with yours once you have tried the lesson.

The lesson: lesson_3_loops_and_functions.py
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
robot.mode.set(1)
robot.motion.home()       # start from the robot's home pose
robot.wait(4.0)
start = robot.state.joints

WAYPOINTS = [
    [20, 0, 0, 0],
    [20, 10, 0, -15],
    [-20, 10, 0, -15],
    [0, 0, 0, 0],
]


def go_to(offsets):
    """Move to start + offsets and wait. Return True when the arm got there."""
    target = list(start)
    for i, offset in enumerate(offsets):
        target[i] = start[i] + offset
    return robot.motion.move_joints(target, wait=True, timeout=10)


ROUNDS = 2
reached = 0
for n in range(ROUNDS):
    for k, offsets in enumerate(WAYPOINTS):
        if is_stopped():
            break
        print(f'round {n + 1}, waypoint {k + 1}')
        if go_to(offsets):
            reached += 1
    if is_stopped():
        break
print(f'{reached} of {ROUNDS * len(WAYPOINTS)} waypoints reached')
