"""ControlPanelHost — the bottom bar's button row and its slide-up panels.

Modelled on the Kinova Kortex Web App's robot control panel: "Clicking on
one of the buttons in the robot control panel will launch a smaller window
from the bottom of the screen... Clicking the same button again will clear
the smaller window at the bottom."
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt  # noqa: E402
from PyQt5.QtGui import QKeyEvent  # noqa: E402
from PyQt5.QtWidgets import QApplication, QLabel  # noqa: E402

from fic_studio.components.control_panel_host import ControlPanelHost  # noqa: E402

_app = QApplication.instance() or QApplication([])


def _host():
    h = ControlPanelHost()
    h.add_panel('joint', 'Joint', QLabel('joint body'))
    h.add_panel('cart', 'Cart', QLabel('cart body'))
    return h


class TestToggling:
    """One panel at a time, and the button that opened it closes it."""

    def test_starts_closed(self):
        """No panel is open before anything is clicked."""
        assert _host().current() is None

    def test_toggle_opens(self):
        """Toggling a registered key opens that panel."""
        h = _host()
        h.toggle('joint')
        assert h.current() == 'joint'

    def test_toggle_same_key_closes(self):
        """Clicking the active button again clears the panel."""
        h = _host()
        h.toggle('joint')
        h.toggle('joint')
        assert h.current() is None

    def test_opening_another_replaces_the_first(self):
        """At most one panel is ever open."""
        h = _host()
        h.toggle('joint')
        h.toggle('cart')
        assert h.current() == 'cart'

    def test_close_panel_closes_whatever_is_open(self):
        """close_panel is unconditional."""
        h = _host()
        h.toggle('cart')
        h.close_panel()
        assert h.current() is None

    def test_unknown_key_is_ignored(self):
        """Toggling a key that was never registered does nothing."""
        h = _host()
        h.toggle('nope')
        assert h.current() is None


class TestButtons:
    """The bar's buttons reflect and drive panel state."""

    def test_panel_button_is_checkable_and_tracks_state(self):
        """The button for an open panel reads as checked."""
        h = _host()
        h.toggle('joint')
        assert h.button('joint').isChecked() is True

    def test_button_unchecks_when_another_opens(self):
        """Opening a second panel unchecks the first button."""
        h = _host()
        h.toggle('joint')
        h.toggle('cart')
        assert h.button('joint').isChecked() is False

    def test_clicking_the_button_toggles_the_panel(self):
        """The wired button opens its own panel."""
        h = _host()
        h.button('cart').click()
        assert h.current() == 'cart'

    def test_action_button_runs_its_callback(self):
        """add_action wires a plain button that fires immediately."""
        h = _host()
        fired = []
        h.add_action('home', 'Home', lambda: fired.append(1))
        h.button('home').click()
        assert fired == [1]

    def test_action_button_opens_no_panel(self):
        """An action button is not a panel button."""
        h = _host()
        h.add_action('home', 'Home', lambda: None)
        h.button('home').click()
        assert h.current() is None

    def test_panel_can_be_disabled_with_a_reason(self):
        """A panel whose backing feature is missing disables with a tooltip."""
        h = _host()
        h.set_enabled('cart', False, 'Cartesian modes need a kinematics solver')
        assert h.button('cart').isEnabled() is False
        assert 'kinematics solver' in h.button('cart').toolTip()

    def test_disabled_panel_cannot_be_opened(self):
        """Toggling a disabled key is a no-op."""
        h = _host()
        h.set_enabled('cart', False, 'unavailable')
        h.toggle('cart')
        assert h.current() is None


class TestEscape:
    """Esc dismisses the open panel."""

    def test_escape_closes_the_open_panel(self):
        """Pressing Esc on the host clears the panel."""
        h = _host()
        h.toggle('joint')
        h.keyPressEvent(QKeyEvent(QKeyEvent.KeyPress, Qt.Key_Escape, Qt.NoModifier))
        assert h.current() is None
