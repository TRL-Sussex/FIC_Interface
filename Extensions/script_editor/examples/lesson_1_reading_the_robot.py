"""Lesson 1 - Reading the robot (variables, lists, f-strings)

Nothing moves in this lesson. You read what the robot reports and answer
three small questions in code.

How to work through it
  1. Start the simulation (Start in the top bar), then press Run once as it is:
     it prints the robot's state, and the checks at the end say "not done yet".
  2. Fill in each TODO, press Run again, and read the checks.
  3. Stuck? lesson_1_solution.py has one way to do it.

What you need to know
  robot.state.joints   list of joint angles in degrees, e.g. [0.0, 15.0, ...]
  robot.state.ee_pose  [x, y, z, qx, qy, qz, qw]: the tool's position in metres
                       and its orientation as a quaternion ([] when not known yet)
  len(a_list)          how many items a list has
  abs(x), round(x, 1)  size of a number, and a number rounded to 1 decimal
  enumerate(a_list)    gives (index, item) pairs, index counting from 0
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')

joints = robot.state.joints
pose = robot.state.ee_pose
print(f'The robot reports {len(joints)} joint angles (degrees):')
for i, angle in enumerate(joints):
    print(f'  joint {i + 1}: {angle:7.1f}')
if pose:
    print(f'Tool position: x={pose[0]:.3f} m, y={pose[1]:.3f} m, z={pose[2]:.3f} m')
else:
    print('The tool position is not known yet: start the simulation, then run again.')

# -- Your turn --------------------------------------------------------------

# TODO 1: how many joints does this robot have?
number_of_joints = None

# TODO 2: which joint is bent furthest from zero? Give its number (1 = the first joint).
#         Hint: go through enumerate(joints) and remember the largest abs(angle) seen so far.
most_bent_joint = None

# TODO 3: how high is the tool above the robot's base, in centimetres, rounded to 1 decimal?
#         The height is z, which is pose[2], in metres.
tool_height_cm = None

# -- Checks (no need to change anything below) ------------------------------


def check(name, answer, expected, close=None):
    if answer is None:
        print(f'[ ] {name}: not done yet')
    elif close is None and answer == expected or close is not None and abs(answer - expected) <= close:
        print(f'[x] {name}: correct ({answer})')
    else:
        print(f'[ ] {name}: {answer} is not it yet, try again')


print()
check('TODO 1, number of joints', number_of_joints, len(joints))
if joints:
    furthest = max(range(len(joints)), key=lambda i: abs(joints[i])) + 1
    check('TODO 2, most bent joint', most_bent_joint, furthest)
if pose:
    check('TODO 3, tool height in cm', tool_height_cm, round(pose[2] * 100, 1), close=0.15)
