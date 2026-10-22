"""Demo - The gripper and the eight control modes

First the gripper: open, half, closed, open. Then a tour of the control modes, two
seconds each, printing what each one does; the arm holds still throughout. It ends in
Joint Impedance (mode 1).
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
print('Gripper: open, half, closed, open')
for width in (0.08, 0.04, 0.0, 0.08):
    if is_stopped():
        break
    robot.motion.set_gripper(width)
    print(f'  width {width * 1000:.0f} mm')
    robot.wait(1.0)

MODES = {
    1: 'Joint Impedance: each joint is held like a spring',
    2: 'Joint Impedance with Planner: a planned, smooth path to each joint goal',
    3: 'Joint Admittance: the arm gives way when pushed',
    4: 'Joint HAP-FIC: impedance and admittance in one law',
    5: 'Cartesian Impedance: the tool is held like a spring in space',
    6: 'Cartesian Impedance with Planner: a planned, smooth path for the tool',
    7: 'Cartesian Admittance: the tool gives way when pushed',
    8: 'Cartesian HAP-FIC: impedance and admittance for the tool',
}
print('\nThe control modes:')
for number, what in MODES.items():
    if is_stopped():
        break
    robot.mode.set(number)
    print(f'  mode {number}: {what}')
    robot.wait(2.0)
robot.mode.set(1)
print('Back in Joint Impedance.')
