"""Demo - A live force meter

For 20 seconds this prints the force on the tool as a bar, four times a second.
Push the arm and watch it: in the MuJoCo window, double-click a link of the arm,
then hold Ctrl and drag with the right mouse button.
Nothing is commanded; the arm holds where it is.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
DURATION_S = 20
print('Push the arm in the MuJoCo window: double-click a link, then Ctrl + right-drag.')
end = time.monotonic() + DURATION_S
peak = 0.0
while time.monotonic() < end and not is_stopped():
    w = robot.state.wrench
    if w is None:
        print('no force reading yet (is the simulation running?)')
    else:
        force = math.sqrt(w[0] ** 2 + w[1] ** 2 + w[2] ** 2)
        peak = max(peak, force)
        bar = '#' * min(40, int(force))
        print(f'{force:6.1f} N |{bar:<40}|  Fx {w[0]:6.1f}  Fy {w[1]:6.1f}  Fz {w[2]:6.1f}')
    robot.wait(0.25)
print(f'Largest force seen: {peak:.1f} N')
