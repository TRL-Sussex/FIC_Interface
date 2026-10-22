"""Lesson 2 - one solution, to compare with yours once you have tried the lesson.

The lesson: lesson_2_first_move.py
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
robot.mode.set(1)          # Joint Impedance
robot.motion.home()       # start from the robot's home pose
robot.wait(4.0)

start = robot.state.joints
JOINT = 1
DELTA_DEG = 15

target = list(start)
target[JOINT - 1] += DELTA_DEG                 # joint 1 is item 0

reached = robot.motion.move_joints(target, wait=True, timeout=10)
print('reached the target' if reached else 'did not get there in time')
robot.wait(1.0)

back = robot.motion.move_joints(start, wait=True, timeout=10)
print('back at the start' if back else 'did not get back in time')
