"""Lesson 1 - one solution, to compare with yours once you have tried the lesson.

The lesson: lesson_1_reading_the_robot.py
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')

joints = robot.state.joints
pose = robot.state.ee_pose

number_of_joints = len(joints)

most_bent_joint = None
largest = -1.0
for i, angle in enumerate(joints):
    if abs(angle) > largest:
        largest = abs(angle)
        most_bent_joint = i + 1          # joint numbers start at 1, list indexes at 0

tool_height_cm = round(pose[2] * 100, 1) if pose else None

print(f'{number_of_joints} joints')
print(f'Joint {most_bent_joint} is bent furthest: {largest:.1f} degrees from zero')
print(f'The tool is {tool_height_cm} cm above the base')
