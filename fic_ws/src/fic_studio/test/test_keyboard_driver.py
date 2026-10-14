# Copyright 2025 Adnan Takriti
# SPDX-License-Identifier: MIT
"""Driving the arm from the keyboard: the mapping."""
from fic_studio.keyboard import driver
import pytest


def test_one_key_moves_one_axis():
    step = driver.cartesian_step({'W'}, 1.0)
    assert step['x'] == pytest.approx(driver.CART_SPEED)
    assert step['y'] == step['z'] == step['yaw'] == 0.0


def test_opposite_keys_cancel():
    """W and S together hold still, rather than the arm picking a side."""
    assert driver.cartesian_step({'W', 'S'}, 1.0)['x'] == 0.0
    assert driver.joint_step({'UP', 'DOWN'}, 1.0) == 0.0
    assert driver.gripper_step({'O', 'C'}, 1.0) == 0.0


def test_two_axes_at_once():
    step = driver.cartesian_step({'W', 'A', 'Q'}, 0.5)
    assert step['x'] > 0 and step['y'] > 0 and step['z'] > 0


def test_shift_is_faster_and_ctrl_is_slower():
    plain = driver.cartesian_step({'W'}, 1.0)['x']
    assert driver.cartesian_step({'W', 'SHIFT'}, 1.0)['x'] == pytest.approx(plain * driver.FAST)
    assert driver.cartesian_step({'W', 'CTRL'}, 1.0)['x'] == pytest.approx(plain * driver.SLOW)
    assert driver.joint_step({'UP', 'SHIFT'}, 1.0) == pytest.approx(driver.JOINT_SPEED * driver.FAST)


def test_the_step_follows_the_clock():
    assert driver.cartesian_step({'W'}, 0.1)['x'] == pytest.approx(driver.CART_SPEED * 0.1)
    assert driver.joint_step({'UP'}, 0.1) == pytest.approx(driver.JOINT_SPEED * 0.1)


def test_holding_a_key_twice_over_is_not_faster():
    """Two keys in the same direction are one movement, not two."""
    assert driver.joint_step({'UP', '.'}, 1.0) == pytest.approx(driver.JOINT_SPEED)


def test_a_number_picks_a_joint_within_the_arm():
    assert driver.picked_joint('1', 7) == 0
    assert driver.picked_joint('7', 7) == 6
    assert driver.picked_joint('8', 7) is None      # a 7-joint arm has no J8
    assert driver.picked_joint('W', 7) is None


def test_the_driver_knows_which_keys_are_its_own():
    assert driver.is_ours('W') and driver.is_ours('TAB') and driver.is_ours('3')
    assert not driver.is_ours('F1') and not driver.is_ours('SPACE')


def test_nothing_held_means_nothing_moves():
    assert driver.cartesian_step(set(), 1.0) == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
    assert driver.joint_step(set(), 1.0) == 0.0
    assert driver.gripper_step(set(), 1.0) == 0.0
