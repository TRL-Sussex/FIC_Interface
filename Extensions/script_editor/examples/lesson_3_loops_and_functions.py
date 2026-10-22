"""Lesson 3 - Loops and functions (a little routine of waypoints)

You write a function that moves the arm to a pose given as offsets from its
starting pose, then use a loop to go through a list of such poses, twice.

Before you run: start the simulation. The lesson sets Joint Impedance.

What you need to know
  def name(argument):     a function: a named piece of code you can call again
      ...
      return value
  for item in a_list:     do something for each item
  for n in range(3):      do something 3 times (n = 0, 1, 2)
  is_stopped()            True once you press Stop: check it in long loops
  zip(a, b)               pairs up two lists: (a[0], b[0]), (a[1], b[1]), ...
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
robot.mode.set(1)          # Joint Impedance
robot.motion.home()       # start from the robot's home pose
robot.wait(4.0)
start = robot.state.joints
if not start:
    raise RuntimeError('No joint angles yet: start the simulation, then run again.')

# Each waypoint says how far (degrees) to move each joint away from `start`.
# A list shorter than the number of joints leaves the remaining joints where they started.
WAYPOINTS = [
    [20, 0, 0, 0],
    [20, 10, 0, -15],
    [-20, 10, 0, -15],
    [0, 0, 0, 0],
]

# -- Your turn --------------------------------------------------------------


def go_to(offsets):
    """Move to start + offsets and wait. Return True when the arm got there."""
    # TODO 1: build the target: start[i] + offsets[i] for the joints in `offsets`,
    #         start[i] for the others. Then call move_joints with wait=True and
    #         return its answer.
    #         Hint: target = list(start), then a for loop over enumerate(offsets).
    return None


ROUNDS = 0          # TODO 2: how many times to go through WAYPOINTS (try 2)
reached = 0         # how many waypoints were reached
# TODO 3: loop ROUNDS times; inside, loop over WAYPOINTS and call go_to() for each.
#         Add 1 to `reached` for each True. Leave both loops as soon as is_stopped().
#         Print where you are, e.g. print(f'round {n + 1}, waypoint {k + 1}').


# -- Checks (no need to change anything below) ------------------------------
print()
expected = ROUNDS * len(WAYPOINTS)
if go_to(WAYPOINTS[-1]) is None:
    print('[ ] TODO 1: go_to() does not move the arm yet (it should return what move_joints answers)')
if ROUNDS == 0:
    print('[ ] TODO 2: ROUNDS is still 0')
elif reached == expected:
    print(f'[x] TODO 3: all {expected} waypoints reached')
else:
    print(f'[ ] TODO 3: {reached} of {expected} waypoints reached')
end = robot.state.joints
if max(abs(e - s) for e, s in zip(end, start)) < 2.0 and reached:
    print('[x] the arm finished where it started (the last waypoint is all zeros)')
