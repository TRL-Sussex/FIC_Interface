"""The simulator starts a robot at its home pose when its config asks for it."""
import os
import types

import yaml

from fic_studio.mixins.process_mixin import ProcessMixin

_YAML = os.path.join(os.path.dirname(__file__), '..', 'fic_studio', 'fic_studio_robots.yaml')


def _arg(cfg):
    host = types.SimpleNamespace(robot_configs={'Arm': cfg})
    return ProcessMixin._sim_start_pose_arg(host, 'Arm')


def test_home_pose_passed_when_asked():
    """sim_start_at_home with a q_home_rad gives the launch argument."""
    arg = _arg({'sim_start_at_home': True, 'q_home_rad': [0.0, 0.25, -1.5]})
    assert arg == " initial_joint_positions:=0.0,0.25,-1.5"


def test_nothing_passed_otherwise():
    """No flag, no home pose, or a bad home pose: all joints start at 0."""
    assert _arg({'q_home_rad': [0.0, 0.25]}) == ''
    assert _arg({'sim_start_at_home': True}) == ''
    assert _arg({'sim_start_at_home': True, 'q_home_rad': ['x']}) == ''


def test_kinova_starts_at_home():
    """The shipped Kinova Gen3 config starts the simulation at its home pose."""
    with open(_YAML) as f:
        robots = yaml.safe_load(f)['robots']
    kinova = robots['KinovaGen3_7DOF']
    assert kinova['sim_start_at_home'] is True
    host = types.SimpleNamespace(robot_configs=robots)
    arg = ProcessMixin._sim_start_pose_arg(host, 'KinovaGen3_7DOF')
    values = [float(v) for v in arg.split(':=')[1].split(',')]
    assert values == kinova['q_home_rad']


def test_an_extension_can_start_a_robot_at_home():
    """_sim_start_home_override (host.set_simulation_start_at_home) does what the flag does."""
    host = types.SimpleNamespace(robot_configs={'Arm': {'q_home_rad': [0.0, 0.25]}},
                                 _sim_start_home_override={'Arm'})
    assert ProcessMixin._sim_start_pose_arg(host, 'Arm') == " initial_joint_positions:=0.0,0.25"
    host._sim_start_home_override = set()
    assert ProcessMixin._sim_start_pose_arg(host, 'Arm') == ''
