"""The FIC law options of 5.11.0 in the Studio: robots_config.yaml ``fic_law`` on the Robot Config page,
the planner fields the Tuning page offers, and the MCU probe telling an old board from no board."""
import pytest

from fic_studio import mcu_probe
from fic_studio.mixins.robot_config_mixin import controller_robots
from fic_studio.web_remote.common import Refused
from fic_studio.web_remote.robot_config import clean
from fic_studio.web_remote import tuning


def test_the_page_takes_the_fic_law_options():
    out = clean({'fic_law': {'restart_on_stall': False, 'planner_max_frequency_hz': 6, 'planner_speed_margin': 1.2}},
                {'num_joints': 7})
    assert out['fic_law'] == {'restart_on_stall': False, 'planner_max_frequency_hz': 6.0, 'planner_speed_margin': 1.2}
    for bad in ({'restart_on_stall': 'yes'}, {'planner_max_frequency_hz': 0}, {'planner_max_frequency_hz': 80},
                {'planner_speed_margin': -1}, {'planner_speed_margin': 'fast'}):
        with pytest.raises(Refused):
            clean({'fic_law': bad}, {'num_joints': 7})


def test_the_options_reach_the_controllers_yaml():
    robots = controller_robots({'Kinova': {'num_joints': 7, 'fic_law': {'restart_on_stall': False}}}, {'Kinova': {}})
    assert robots['Kinova']['fic_law'] == {'restart_on_stall': False}


@pytest.mark.parametrize('kind,mode', [('joint', 2), ('axis', 6), ('postural', 6)])
def test_the_tuning_page_offers_only_planner_fields_that_act(kind, mode):
    keys = [f['key'] for f in tuning._specs(kind, mode)]
    assert 'planner_acc_max' in keys and 'planner_vel_desired' in keys
    assert not {'planner_stiffness', 'planner_stiffness_p', 'planner_frequency'} & set(keys)


def test_an_old_board_is_told_apart_from_no_board(monkeypatch):
    calls = []

    def fake_probe(address=None, timeout=0.3, attempts=3, link_version=mcu_probe._VERSION):
        calls.append(link_version)
        return {'fw': 'fic_mcu_q', 'version': 1, 'adapters': 2, 'max_joints': 16, 'state': 0} \
            if link_version == mcu_probe._VERSION - 1 else None

    monkeypatch.setattr(mcu_probe, 'probe', fake_probe)
    out = mcu_probe.discover({'mcu': {'adapter': 'kortex'}}, bridge=False)
    assert calls[:2] == [mcu_probe._VERSION, mcu_probe._VERSION - 1]
    assert not out.get('use') and 'flash it again' in out['detail']
