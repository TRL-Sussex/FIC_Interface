"""Cartesian modes are unavailable on legged robots; sticks drive locomotion."""
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QWidget  # noqa: E402

from fic_studio.mixins.robot_control_mixin import (  # noqa: E402
    CONTROL_MODE_TABS, RobotControlMixin)

_app = QApplication.instance() or QApplication([])


class _Host:
    def __init__(self, category):
        self.robot_configs = {'R': {'robot_category': category}}
        self._rc_mode_buttons = [QPushButton(name)
                                 for name, _ in CONTROL_MODE_TABS]


def test_cartesian_modes_disabled_for_legged_robots():
    host = _Host('legged')
    RobotControlMixin._rc_apply_mode_availability(host, 'R')
    for button, (name, kind) in zip(host._rc_mode_buttons, CONTROL_MODE_TABS):
        assert button.isEnabled() is (kind == 'joint'), name


def test_all_modes_enabled_for_arms():
    host = _Host('arm')
    RobotControlMixin._rc_apply_mode_availability(host, 'R')
    assert all(button.isEnabled() for button in host._rc_mode_buttons)


def test_unknown_robot_leaves_every_mode_enabled():
    host = _Host('arm')
    RobotControlMixin._rc_apply_mode_availability(host, 'Missing')
    assert all(button.isEnabled() for button in host._rc_mode_buttons)


# ── Finding 2: a stale Cartesian selection must not survive gating ──────────
#
# Reviewer-verified failure: select Franka, choose "Cart Imp", switch to
# Spot — the buttons disable but _rc_selected_kind/the hidden dropdown/the
# custom-group visibility all stay on Cartesian, so the user can still send
# a Cartesian command the C++ node will reject. The fix re-selects Joint Imp
# through the normal _on_rc_mode_selected path when gating a legged robot
# while a Cartesian mode is active.

class _GatingHost(RobotControlMixin):
    """
    Enough real widgets for _on_rc_mode_selected to run its normal path.

    Heavier collaborators (profile dropdown refresh/publish, control-area
    rebuild, node publish) are stubbed out since they are unrelated to the
    Cartesian-gating fix under test; _highlight_rc_mode_tab and
    _update_rc_section_visibility come from RobotControlMixin unmodified so
    the real dependent-state updates (dropdown index, section visibility,
    highlighted tab) are exercised for real.
    """

    def __init__(self, category, initial_mode_idx):
        self.robot_configs = {'R': {'robot_category': category}}
        self._rc_mode_buttons = [QPushButton(name) for name, _ in CONTROL_MODE_TABS]
        self._rc_mode_dropdown = QComboBox()
        self._rc_mode_dropdown.addItems([name for name, _ in CONTROL_MODE_TABS])
        self._rc_joint_group = QWidget()
        self._rc_custom_group = QWidget()
        self._rc_preset_row = QWidget()
        self._rc_pose_row = QWidget()
        self._rc_postural_toggle = QWidget()
        self._rc_elbow_toggle = QWidget()
        self._rc_hapfic_toggle = QWidget()
        self._rc_nullspace_label = QWidget()
        self._rc_admittance_planner_toggle = QWidget()
        self.control_target = 'simulation'
        self.node = types.SimpleNamespace(publish_mode=lambda *_a: None)
        self.mode_select_calls = []

        # Select the initial mode directly (bypasses gating — mirrors the
        # user picking a mode from the UI before the robot switch happens).
        name, _ = CONTROL_MODE_TABS[initial_mode_idx]
        self._on_rc_mode_selected(initial_mode_idx, name)
        self.mode_select_calls = []  # ignore the setup call in assertions

    def _on_rc_mode_selected(self, mode_idx, mode_name):
        self.mode_select_calls.append((mode_idx, mode_name))
        RobotControlMixin._on_rc_mode_selected(self, mode_idx, mode_name)

    # Heavier collaborators, unrelated to this fix.
    def _refresh_rc_profile_dropdown(self):
        pass

    def _publish_rc_profile(self):
        pass

    def _rebuild_rc_control_area(self):
        pass

    def _rc_current_profile(self):
        return 'default'

    def log_message(self, *args):
        pass


def test_cartesian_selection_falls_back_to_joint_on_legged_switch():
    host = _GatingHost('arm', initial_mode_idx=4)   # "Cart Imp" selected on Franka
    assert host._rc_selected_kind == 'cart'

    host.robot_configs['R']['robot_category'] = 'legged'   # user switches to Spot
    RobotControlMixin._rc_apply_mode_availability(host, 'R')

    assert host.mode_select_calls == [(0, 'Joint Imp')]
    assert host._rc_selected_kind == 'joint'
    assert host._rc_mode_dropdown.currentIndex() == 0
    assert host._rc_custom_group.isVisible() is False
    assert host._rc_joint_group.isVisible() is True


def test_joint_selection_is_untouched_on_legged_switch():
    host = _GatingHost('arm', initial_mode_idx=0)   # "Joint Imp" already selected
    host.robot_configs['R']['robot_category'] = 'legged'
    RobotControlMixin._rc_apply_mode_availability(host, 'R')
    assert host.mode_select_calls == []
    assert host._rc_selected_kind == 'joint'


def test_cartesian_selection_survives_switch_between_arms():
    host = _GatingHost('arm', initial_mode_idx=4)   # "Cart Imp" selected
    RobotControlMixin._rc_apply_mode_availability(host, 'R')   # still an arm
    assert host.mode_select_calls == []
    assert host._rc_selected_kind == 'cart'
