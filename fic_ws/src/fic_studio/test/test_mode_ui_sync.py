"""update_mode_ui_only must work on the control-bar GUI.

That GUI has no page tab strip (``_rc_mode_buttons``): its mode rows are
the control bar's own. update_mode_ui_only indexed the strip anyway and
raised AttributeError, which unwound every caller -- on 2026-09-14 the
Following page's START, which then did nothing. The Studio's own
_on_rc_mode_selected already guarded it; this pins the same guard here.
"""
import os
import pathlib
import sys

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from fic_studio.mixins.robot_control_mixin import RobotControlMixin  # noqa: E402


class _Dropdown:
    def __init__(self):
        self.index = None

    def setCurrentIndex(self, idx):  # noqa: N802 -- Qt's name
        self.index = idx


class _Node:
    station_robot_id = 'robot1'
    station_id = 1


class _ControlBarGui:
    """What the control-bar Studio has: no _rc_mode_buttons at all."""

    def __init__(self):
        self.node = _Node()
        self._rc_mode_dropdown = _Dropdown()
        self.synced = 0
        self.kinds = []

    def _cb_sync_mode_buttons(self):
        self.synced += 1

    def _update_rc_section_visibility(self, kind):
        self.kinds.append(kind)


def test_the_control_bar_gui_is_synced_without_raising():
    gui = _ControlBarGui()
    RobotControlMixin.update_mode_ui_only(gui, 'Cartesian Impedance')
    assert gui._rc_selected_mode_idx == 4
    assert gui._rc_selected_kind == 'cart'
    assert gui._rc_mode_dropdown.index == 4
    assert gui.synced == 1
    assert gui.kinds == ['cart']


def test_an_unknown_mode_name_changes_nothing():
    gui = _ControlBarGui()
    RobotControlMixin.update_mode_ui_only(gui, 'No Such Mode')
    assert not hasattr(gui, '_rc_selected_mode_idx')
    assert gui.synced == 0
