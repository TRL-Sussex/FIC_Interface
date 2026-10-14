"""The hardware stack must bring up contact_node, or nothing estimates.

``fic_hardware/launch/robot_control_launch.py`` is what FIC Studio runs
for a hardware session (``process_mixin._start_robot_control``). Without
contact_launch.py included there, ``{rid}_contact_estimate`` has no
publisher at all: the twin opens, mirrors the pose, and never highlights
because ``link`` never leaves its initial -1.

fic_hardware is a C++ package with no pytest harness of its own, so this
lives with the node whose startup it guards.
"""

import os

_LAUNCH = os.path.join(
    os.path.dirname(__file__), '..', '..', 'fic_hardware', 'launch',
    'robot_control_launch.py')


def _source():
    """Return robot_control_launch.py's text."""
    with open(_LAUNCH) as handle:
        return handle.read()


def test_hardware_launch_includes_contact_launch():
    """The hardware launch must pull in fic_contact's launch file."""
    src = _source()
    assert 'contact_launch.py' in src
    assert 'fic_contact' in src


def test_contact_include_forwards_station_and_robot():
    """A station-2 session must start a station-2 contact node.

    contact_launch.py defaults to station_id 1 and KinovaGen3_7DOF, so an
    include that forwards neither would silently estimate for the wrong
    station on every bench but the first.
    """
    src = _source()
    assert 'station_id' in src and 'robot_name' in src
    include = src[src.index('contact_launch.py'):]
    assert 'launch_arguments' in include
