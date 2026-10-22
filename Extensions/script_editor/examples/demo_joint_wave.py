"""Demo - A wave through the joints

Every joint swings a little, each a moment after the one before it, so a wave runs
down the arm. The swing grows in, runs for a while, fades out, and the arm returns
to where it started. Joint Impedance; the targets are streamed 50 times a second.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
AMPLITUDE_DEG = 8.0
PERIOD_S = 3.0
DURATION_S = 12.0
LAG = 0.6                 # radians of phase from one joint to the next

robot.mode.set(1)         # Joint Impedance
robot.wait(0.5)
start = robot.state.joints
if not start:
    raise RuntimeError('No joint angles yet: start the simulation, then run again.')

t0 = time.monotonic()
while not is_stopped():
    t = time.monotonic() - t0
    if t > DURATION_S:
        break
    envelope = min(1.0, t / 2.0, (DURATION_S - t) / 2.0)        # grow in and fade out over 2 s
    target = [q + envelope * AMPLITUDE_DEG * math.sin(2 * math.pi * t / PERIOD_S - i * LAG)
              for i, q in enumerate(start)]
    robot.motion.move_joints(target)
    robot.wait(0.02)
robot.motion.move_joints(start, wait=True, timeout=5)
print('Wave done; back at the start.')
