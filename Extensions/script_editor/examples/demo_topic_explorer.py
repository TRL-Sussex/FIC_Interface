"""Demo - Exploring the robot's ROS topics

Everything the Studio shows comes from ROS topics. This lists the topics of the robot
the editor drives, then subscribes to its tool pose and prints a few readings.
The Topic Monitor page shows the same topics with their rates.
"""
if robot.target_mode != 'simulation':
    raise RuntimeError('This example runs in the simulation only: set the target to Simulation (top bar).')
mine = sorted(t for t in robot.topics.list_topics() if robot.rid in t[0])
print(f'{len(mine)} topics for {robot.rid}:')
for name, kind in mine:
    print(f'  {name:45s} {kind}')

TOPIC = f'/{robot.rid}_ee_current_pose'
print(f'\nReading {TOPIC} five times:')
robot.topics.subscribe(TOPIC)
seen = 0
deadline = time.monotonic() + 5
while seen < 5 and time.monotonic() < deadline and not is_stopped():
    msg = robot.topics.get(TOPIC)
    if msg:
        print(f'  {msg}')
        seen += 1
    robot.wait(0.3)
robot.topics.unsubscribe(TOPIC)
if not seen:
    print('  nothing yet: start the simulation, then run again.')
