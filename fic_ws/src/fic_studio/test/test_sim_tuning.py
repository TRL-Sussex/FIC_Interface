"""Simulation-only tuning: the velocity limit and the 'sim' config follow the target."""
import os
import types
import xml.etree.ElementTree as ET

import yaml

from fic_studio.mixins.robot_control_mixin import RobotControlMixin
from fic_studio.mixins.safety_mixin import SafetyMixin

_HERE = os.path.dirname(__file__)
_STUDIO_YAML = os.path.join(_HERE, '..', 'fic_studio', 'fic_studio_robots.yaml')
_REPO = os.path.join(_HERE, '..', '..', '..', '..')
_CTRL_YAML = os.path.join(_REPO, 'fic_library', 'config', 'robots_config.yaml')
_CONFIG_XML = os.path.join(_REPO, 'fic_library', 'config', 'config.xml')


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


def _safety_host(targets, velocity):
    pubs = {rid: {'cart': _Pub(), 'joint': _Pub(), 'vel': _Pub()} for rid in targets}
    host = types.SimpleNamespace(
        safety_limits={'cartesian': {}, 'joint_safety_deg': {}, 'velocity': velocity},
        safety_pubs=pubs, robot_target=lambda rid=None: targets[rid])
    return host, pubs


def test_simulated_robots_get_the_sim_velocity_limit():
    """sim_max_joint_vel goes to robots in simulation, max_joint_vel to the rest."""
    host, pubs = _safety_host({'robot1': 'simulation', 'robot2': 'hardware'},
                              {'max_joint_vel': 1.0, 'sim_max_joint_vel': 3.0,
                               'max_cart_vel': 0.5, 'enabled': True})
    SafetyMixin._publish_safety_limits(host)
    assert pubs['robot1']['vel'].sent == [[3.0, 0.5, 1.0]]
    assert pubs['robot2']['vel'].sent == [[1.0, 0.5, 1.0]]


def test_without_a_sim_limit_both_targets_get_max_joint_vel():
    host, pubs = _safety_host({'robot1': 'simulation'},
                              {'max_joint_vel': 1.0, 'max_cart_vel': 0.5, 'enabled': True})
    SafetyMixin._publish_safety_limits(host)
    assert pubs['robot1']['vel'].sent == [[1.0, 0.5, 1.0]]


def _profile_host(target):
    return types.SimpleNamespace(robot_target=lambda rid=None: target)


def test_config_follows_the_target():
    """'sim' in simulation when the robot has it; 'default' otherwise."""
    pick = RobotControlMixin._rc_target_default_profile
    assert pick(_profile_host('simulation'), ['default', 'sim']) == 'sim'
    assert pick(_profile_host('hardware'), ['default', 'sim']) == 'default'
    assert pick(_profile_host('simulation'), ['default', 'Stiff']) == 'default'


def test_kinova_ships_the_sim_tuning():
    """Kinova: a weaker damper in simulation only, and 'sim' entries for modes 3 and 6-8."""
    for path in (_STUDIO_YAML, _CTRL_YAML):
        with open(path) as f:
            vel = yaml.safe_load(f)['robots']['KinovaGen3_7DOF']['safety_zones']['velocity']
        assert vel['max_joint_vel'] == 1.0 and vel['sim_max_joint_vel'] == 3.0, path
    robot = next(r for r in ET.parse(_CONFIG_XML).getroot().iter('robot')
                 if r.get('name') == 'KinovaGen3_7DOF')
    sim = {m.get('id') for m in robot.findall('mode') if m.get('profile') == 'sim'}
    assert {'3', '6', '7', '8', '100'} <= sim
    assert '4' not in sim
