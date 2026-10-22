"""Demo - Soft and stiff: Joint Admittance

The arm switches to Joint Admittance (mode 3): pushed, it gives way, and it drifts
back when you let go. For 20 seconds this prints how far each joint is from where it
started. Push it in the MuJoCo window (double-click a link, Ctrl + right-drag), then
let go. Afterwards the arm returns to Joint Impedance (mode 1) and its start pose.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
DURATION_S = 20
robot.mode.set(3)         # Joint Admittance
robot.wait(0.5)
start = robot.state.joints
if not start:
    raise RuntimeError('No joint angles yet: start the simulation, then run again.')
print('Joint Admittance: push the arm in the MuJoCo window and let go.')
end = time.monotonic() + DURATION_S
furthest = 0.0
while time.monotonic() < end and not is_stopped():
    now = robot.state.joints
    away = [n - s for n, s in zip(now, start)]
    furthest = max(furthest, max(abs(a) for a in away))
    print('away from start (deg): ' + '  '.join(f'{a:+5.1f}' for a in away))
    robot.wait(0.5)
robot.mode.set(1)         # Joint Impedance
robot.motion.move_joints(start, wait=True, timeout=10)
print(f'Furthest any joint was pushed: {furthest:.1f} degrees. Back in Joint Impedance.')
