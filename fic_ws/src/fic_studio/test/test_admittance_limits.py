"""Cartesian admittance push limits per robot (robots_config.yaml admittance_limits): the Robot
Config page's checks, and the key reaching the controllers' copy of the robot list."""
import pytest

from fic_studio.mixins.robot_config_mixin import controller_robots
from fic_studio.web_remote.common import Refused
from fic_studio.web_remote.robot_config import clean


def test_the_page_takes_positive_limits_within_reason():
    out = clean({'admittance_limits': {'push_and_stay_m': 0.2, 'haptic_deg': 3}}, {'num_joints': 7})
    assert out['admittance_limits'] == {'push_and_stay_m': 0.2, 'haptic_deg': 3.0}
    for bad in ({'push_and_stay_m': 0}, {'haptic_m': -0.1}, {'push_and_stay_deg': 500}, {'haptic_m': 'far'}):
        with pytest.raises(Refused):
            clean({'admittance_limits': bad}, {'num_joints': 7})


def test_the_limits_reach_the_controllers_yaml():
    robots = controller_robots({'Kinova': {'num_joints': 7, 'admittance_limits': {'haptic_m': 0.03}}},
                               {'Kinova': {'admittance_limits': {'haptic_m': 0.05}}})
    assert robots['Kinova']['admittance_limits'] == {'haptic_m': 0.03}
    kept = controller_robots({'Kinova': {'num_joints': 7}}, {'Kinova': {'admittance_limits': {'haptic_m': 0.05}}})
    assert kept['Kinova']['admittance_limits'] == {'haptic_m': 0.05}   # set by hand there: kept
