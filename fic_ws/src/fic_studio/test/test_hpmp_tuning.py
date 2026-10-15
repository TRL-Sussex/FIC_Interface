"""H-PMP's own tuning block (<mode id="100">) in config.xml, and the migration that adds it.

H-PMP used to run on mode 4's joints and mode 6's Cartesian block. The migration must give it
exactly the values it ran on, for every robot and profile, and leave every control mode as it was,
apart from plain mode 4 under the profiles whose mode 4 block existed only to tune H-PMP.
"""
import dataclasses
import os
import shutil
import sys
import xml.etree.ElementTree as ET

from fic_studio import xml_manager as xm
from fic_studio.xml_manager import HPMP_MODE, XMLManager, find_config_path
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import migrate_hpmp_tuning as mig  # noqa: E402

KINOVA = 'KinovaGen3_7DOF'
MOVED = ('hw_x2', 'hw_x3', 'hw_x4', 'hw_x6', 'sim')


@pytest.fixture
def before(tmp_path):
    """config.xml as it was before H-PMP had a block: the migration undone if it is in.

    A profile with a H-PMP block and no mode 4 block of its own had its mode 4 moved into it
    (--move-mode4), so it gets that mode 4 block back from the H-PMP block's joints.
    """
    import copy
    path = tmp_path / 'before.xml'
    shutil.copy(find_config_path(), path)
    tree = ET.parse(path)
    for robot in tree.getroot().iter('robot'):
        have4 = {m.get('profile') or 'default' for m in robot.findall('mode') if m.get('id') == '4'}
        for m in [m for m in robot.findall('mode') if m.get('id') == str(HPMP_MODE)]:
            profile = m.get('profile') or 'default'
            if profile not in have4:
                mode4 = ET.Element('mode', {'id': '4', 'name': 'Joint Admittance Haptic', 'profile': profile})
                for j in m.findall('joint'):
                    mode4.append(copy.deepcopy(j))
                robot.insert(list(robot).index(m), mode4)
            robot.remove(m)
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return str(path)


@pytest.fixture
def after(before, tmp_path):
    path = tmp_path / 'after.xml'
    shutil.copy(before, path)
    assert mig.main([str(path), '--move-mode4', ','.join(MOVED)]) == 0
    return str(path)


def _runs_on(path, robot, profile, joints):
    """What H-PMP runs on, read the way the controller does (own block, else modes 4 and 6)."""
    m = XMLManager(path)
    return ([dataclasses.asdict(m.load_joint_params(robot, j, HPMP_MODE, profile)) for j in range(joints)],
            dataclasses.asdict(m.load_cartesian_params(robot, HPMP_MODE, profile)))


def _profiles_and_joints(path):
    m = XMLManager(path)
    for robot in m.get_available_robots():
        yield robot, m.get_all_profiles(robot), m.get_num_joints(robot, 4)


def test_hpmp_runs_on_the_same_values_after_the_migration_for_every_robot_and_profile(before, after):
    robots = checked = 0
    for robot, profiles, n in _profiles_and_joints(before):
        robots += 1
        for profile in profiles:
            assert _runs_on(after, robot, profile, n) == _runs_on(before, robot, profile, n), (robot, profile)
            checked += 1
    # Every robot in config.xml was compared, on at least one profile each.
    assert robots >= 4 and checked >= robots


def test_every_robot_has_its_own_block_and_the_moved_profiles_have_theirs(after):
    m = XMLManager(after)
    for robot in m.get_available_robots():
        assert 'default' in m.get_available_profiles(robot, HPMP_MODE), robot
    assert set(MOVED) <= set(m.get_available_profiles(KINOVA, HPMP_MODE))
    assert not set(MOVED) & set(m.get_available_profiles(KINOVA, 4))             # their mode 4 blocks are gone
    sim = m.load_joint_params(KINOVA, 0, HPMP_MODE, 'sim')
    assert sim.admittance_damping == 36.0 and m.load_joint_params(KINOVA, 1, HPMP_MODE, 'hw_x6').admittance_damping == 82.2


def test_the_control_modes_are_untouched_except_plain_mode_4_under_the_moved_profiles(before, after):
    a, b = XMLManager(before), XMLManager(after)
    for robot, profiles, n in _profiles_and_joints(before):
        for profile in profiles:
            for mode in (1, 2, 3, 4):
                for j in range(n):
                    old, new = a.load_joint_params(robot, j, mode, profile), b.load_joint_params(robot, j, mode, profile)
                    if mode == 4 and profile in MOVED and robot == KINOVA:
                        assert new == a.load_joint_params(robot, j, 4, 'default')    # plain mode 4 is default's again
                    else:
                        assert new == old, (robot, profile, mode, j)
            for mode in (5, 6, 7, 8):
                assert b.load_cartesian_params(robot, mode, profile) == a.load_cartesian_params(robot, mode, profile)


def test_the_migration_is_idempotent_and_keeps_comments(before, tmp_path):
    path = tmp_path / 'c.xml'
    text = open(before).read().replace('<robots>', '<robots>\n    <!-- kept -->', 1)
    path.write_text(text)
    assert mig.main([str(path)]) == 0
    once = path.read_text()
    assert '<!-- kept -->' in once and 'id="100" name="H-PMP" profile="default"' in once
    assert mig.main([str(path)]) == 0 and path.read_text() == once


def test_a_mode_4_block_that_is_more_than_damping_is_kept(before, tmp_path):
    path = tmp_path / 'c.xml'
    shutil.copy(before, path)
    tree = ET.parse(path)
    robot = next(r for r in tree.getroot().iter('robot') if r.get('name') == KINOVA)
    block = next(m for m in robot.findall('mode') if m.get('id') == '4' and m.get('profile') == 'hw_x2')
    block.find('joint/controller/stiffness').text = '123.0'
    tree.write(path, encoding='utf-8', xml_declaration=True)
    assert mig.main([str(path), '--move-mode4', 'hw_x2']) == 0
    assert 'hw_x2' in XMLManager(str(path)).get_available_profiles(KINOVA, 4)


def test_without_a_block_hpmp_reads_modes_4_and_6_and_saving_makes_a_whole_block(before):
    m = XMLManager(before)
    assert m.load_joint_params(KINOVA, 2, HPMP_MODE, 'default') == m.load_joint_params(KINOVA, 2, 4, 'default')
    assert m.load_cartesian_axis(KINOVA, HPMP_MODE, 'y', 'default') == m.load_cartesian_axis(KINOVA, 6, 'y', 'default')
    joint = m.load_joint_params(KINOVA, 2, HPMP_MODE, 'default')
    joint.admittance_damping = 40.0
    m.save_joint_params(KINOVA, 2, HPMP_MODE, joint, 'default')
    m.save()
    again = XMLManager(before)
    assert again.load_joint_params(KINOVA, 2, HPMP_MODE).admittance_damping == 40.0
    for j in (0, 1, 3, 4, 5, 6):                                  # every other joint came along, unchanged
        assert again.load_joint_params(KINOVA, j, HPMP_MODE) == m.load_joint_params(KINOVA, j, 4)
    assert again.load_cartesian_params(KINOVA, HPMP_MODE) == again.load_cartesian_params(KINOVA, 6)
    assert again.load_joint_params(KINOVA, 2, 4).admittance_damping != 40.0      # mode 4 untouched


def test_a_profile_without_its_own_block_copies_the_one_it_runs_on(after):
    m = XMLManager(after)
    m.clone_profile(KINOVA, 1, 'default', 'gentle')
    axis = m.load_cartesian_axis(KINOVA, HPMP_MODE, 'x', 'gentle')
    assert axis == m.load_cartesian_axis(KINOVA, HPMP_MODE, 'x', 'default')   # falls back to default's block
    axis.stiffness = 77.0
    m.save_cartesian_axis(KINOVA, HPMP_MODE, 'x', axis, 'gentle')
    assert m.load_cartesian_axis(KINOVA, HPMP_MODE, 'x', 'gentle').stiffness == 77.0
    assert m.load_cartesian_axis(KINOVA, HPMP_MODE, 'y', 'gentle') == m.load_cartesian_axis(KINOVA, HPMP_MODE, 'y')
    assert m.load_cartesian_axis(KINOVA, HPMP_MODE, 'x', 'default').stiffness != 77.0


def test_hpmp_is_not_a_control_mode():
    assert HPMP_MODE not in xm.CONTROL_MODES
    assert xm.mode_name(HPMP_MODE) == 'H-PMP' and xm.is_hpmp_mode(HPMP_MODE)
    assert not xm.is_joint_mode(HPMP_MODE) and not xm.is_cartesian_mode(HPMP_MODE)
