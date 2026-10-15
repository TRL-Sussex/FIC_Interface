"""What the Studio calls the control modes: HAP-FIC is mode 4 (joints) and mode 8 (Cartesian); H-PMP
is the toggle that nests a Cartesian mode in the joint HAP-FIC, and on mode 8 it is the H-PMP of
the Whole-Arm experiments."""
from fic_studio.mixins.control_bar_mixin import MODE_MENUS, MODE_ROW_LABELS
from fic_studio.mixins.robot_control_mixin import (
    CONTROL_MODE_DISPLAY, CONTROL_MODE_NAMES, CONTROL_MODE_TABS, mode_display_name,
)


def test_menu_rows_name_hapfic_not_hpmp():
    assert MODE_ROW_LABELS == ['Impedance', 'Imp. + Planner', 'Admittance', 'HAP-FIC']
    joint, cart = MODE_MENUS['joint'][1], MODE_MENUS['cart'][1]
    assert CONTROL_MODE_DISPLAY[joint[-1]] == 'Joint HAP-FIC'
    assert CONTROL_MODE_DISPLAY[cart[-1]] == 'Cartesian HAP-FIC'
    assert CONTROL_MODE_TABS[3][0] == 'Joint HAP-FIC' and CONTROL_MODE_TABS[7][0] == 'Cart HAP-FIC'
    assert not any('H-PMP' in n for n in CONTROL_MODE_DISPLAY)


def test_display_name_with_the_hpmp_toggle():
    assert mode_display_name(7, True) == 'H-PMP'                       # mode 8 + H-PMP
    assert mode_display_name(7, False) == 'Cartesian HAP-FIC'
    assert mode_display_name(4, True) == 'Cartesian Impedance + H-PMP'  # the same overlay nested
    assert mode_display_name(6, True) == 'Cartesian Admittance + H-PMP'
    for i in range(4):                                                  # no H-PMP in joint space
        assert mode_display_name(i, True) == CONTROL_MODE_DISPLAY[i]
    assert mode_display_name(3, True) == 'Joint HAP-FIC'
    assert mode_display_name(9) == '--'


def test_internal_names_unchanged():
    assert CONTROL_MODE_NAMES[3] == 'Joint Admittance Haptic'          # extensions key on these
    assert CONTROL_MODE_NAMES[7] == 'Cartesian Admittance Haptic'
