"""robots_config.yaml hardware_model in the Studio.

The robot editor's check (web_remote.robot_config.clean) and the controllers' copy written on
Save (controller_robots keeps it).
"""
from fic_studio.mixins.robot_config_mixin import controller_robots
from fic_studio.web_remote.common import Refused
from fic_studio.web_remote.robot_config import clean
import pytest

CUR = {'num_joints': 3}


def test_clean_accepts_a_block():
    hm = {'enabled': True, 'joint_friction': [0.6, 0.6, 0.3], 'joint_damping': [15, 15, 5],
          'armature': [], 'torque_noise': [0.1, 0.05, 0.02], 'torque_filter_hz': 200,
          'command_delay_ms': 1, 'source': ' measured 15 Oct '}
    out = clean({'hardware_model': hm}, CUR)['hardware_model']
    assert out == {'enabled': True, 'joint_friction': [0.6, 0.6, 0.3],
                   'joint_damping': [15.0, 15.0, 5.0],
                   'armature': [], 'torque_noise': [0.1, 0.05, 0.02], 'torque_filter_hz': 200.0,
                   'command_delay_ms': 1.0, 'source': 'measured 15 Oct'}


@pytest.mark.parametrize('hm', [
    {'enabled': 'yes'},
    {'joint_friction': [0.6, 0.6]},                 # not one per joint
    {'joint_damping': [1, -2, 3]},                  # negative
    {'torque_filter_hz': 5000},
    {'command_delay_ms': -1},
    {'source': 'two\nlines'},
    [1, 2],
])
def test_clean_refuses_bad_values(hm):
    with pytest.raises(Refused):
        clean({'hardware_model': hm}, CUR)


def test_controller_copy_keeps_and_updates_the_block():
    old = {'R': {'hardware_model': {'enabled': True, 'joint_friction': [0.6]}}}
    assert controller_robots({'R': {}}, old)['R']['hardware_model']['joint_friction'] == [0.6]
    new = controller_robots({'R': {'hardware_model': {'enabled': False}}}, old)
    assert new['R']['hardware_model'] == {'enabled': False}


def test_torque_sensor_position_is_accepted_and_checked():
    from fic_studio.web_remote.robot_config import Refused, _hardware_model
    assert _hardware_model({'torque_sensor': 'output'}, 7) == {'torque_sensor': 'output'}
    with pytest.raises(Refused):
        _hardware_model({'torque_sensor': 'gearbox'}, 7)


def test_the_arms_own_force_deadband_is_kept_when_deadbands_are_edited():
    """admittance_deadbands.cart_force_hardware (the arm's Cartesian force deadband) survives an edit."""
    out = clean({'admittance_deadbands': {'joint_tau': 2, 'cart_force': 4, 'cart_torque': 0.2,
                                          'cart_force_hardware': 10}}, {'num_joints': 7})
    assert out['admittance_deadbands'] == {'joint_tau': 2.0, 'cart_force': 4.0, 'cart_torque': 0.2,
                                           'cart_force_hardware': 10.0}


def test_the_kinova_arm_has_its_own_force_deadband():
    """The Kinova's joint-torque force estimate needs 10 N on the arm; the simulator keeps 4 N."""
    import pathlib
    import yaml
    cfg = yaml.safe_load(open(pathlib.Path(__file__).resolve().parents[4] / 'fic_library' / 'config'
                              / 'robots_config.yaml'))
    db = cfg['robots']['KinovaGen3_7DOF']['admittance_deadbands']
    assert (db['cart_force'], db['cart_force_hardware']) == (4.0, 10.0)
