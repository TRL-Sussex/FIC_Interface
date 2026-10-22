"""Lesson 2 - Your first move (one joint, there and back)

You move a single joint by a few degrees, wait until it gets there, then bring
it back to where it started.

Before you run
  * Start the simulation (Start in the top bar). This lesson switches the arm to
    Joint Impedance for you.
  * Stop (in the editor) or the E-STOP (Space) ends any script at once.

What you need to know
  robot.motion.move_joints(angles, wait=True, timeout=10)
      sends the arm to `angles` (a list with one angle in degrees per joint).
      With wait=True it returns True once the arm is there, False on a timeout
      or when you press Stop.
  list(a_list)       a copy of a list that you can change without changing the original
  a_list[0]          the first item: lists count from 0, so joint 1 is item 0
  robot.wait(s)      pause for s seconds (ends early when you press Stop)
For the workshop: the simulation with the Kinova Gen3 7DOF (choose it under Model in the
top bar before you press Start). The lessons build on each other: 1 to 5, in order.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
robot.mode.set(1)          # 1 = Joint Impedance
robot.motion.home()       # start from the robot's home pose
robot.wait(4.0)

start = robot.state.joints
if not start:
    raise RuntimeError('No joint angles yet: start the simulation, then run again.')
print('Start (deg):', [round(a, 1) for a in start])

# -- Your turn --------------------------------------------------------------

JOINT = 1        # TODO 1: which joint to move (1 = the first). Later, try the last one, the wrist.
DELTA_DEG = 0    # TODO 2: how far to move it, in degrees. Try 15 (keep it under 30).

# TODO 3: make `target` a copy of `start` with DELTA_DEG added to the joint you chose.
target = list(start)

# TODO 4: send the arm to `target` and wait for it. Keep the True/False answer in `reached`.
reached = None

robot.wait(1.0)

# TODO 5: send the arm back to `start`, waiting again. Keep the answer in `back`.
back = None

# -- Checks (no need to change anything below) ------------------------------
print()
changed = [i + 1 for i, (t, s) in enumerate(zip(target, start)) if abs(t - s) > 0.01]
if DELTA_DEG == 0:
    print('[ ] TODO 2: DELTA_DEG is still 0, so nothing will move')
elif changed == [JOINT] and abs(target[JOINT - 1] - start[JOINT - 1] - DELTA_DEG) < 0.01:
    print(f'[x] TODO 3: target changes joint {JOINT} by {DELTA_DEG} degrees and nothing else')
else:
    print(f'[ ] TODO 3: target should change joint {JOINT} only; it changes {changed or "nothing"}')
print('[x] TODO 4: the arm reached the target' if reached else
      '[ ] TODO 4: not reached yet (is move_joints called with wait=True, and its answer kept?)')
end = robot.state.joints
home_again = back and max(abs(e - s) for e, s in zip(end, start)) < 2.0
print('[x] TODO 5: back where it started' if home_again else '[ ] TODO 5: not back at the start yet')
