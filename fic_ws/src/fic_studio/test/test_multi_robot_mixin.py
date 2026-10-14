"""The multi-robot cards live in their own mixin, on their own page.

Robot Control's page was dismantled into the global control bar; the
per-station cards (Robot 2-4) moved here rather than being deleted. The
methods are still resolved on the composed window, so extensions that
reach for card_robot_type or _add_robot_btn keep working.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication  # noqa: E402

from fic_studio.mixins.multi_robot_mixin import MultiRobotMixin  # noqa: E402
from fic_studio.mixins.robot_control_mixin import RobotControlMixin  # noqa: E402

_app = QApplication.instance() or QApplication([])

CARD_API = [
    '_add_robot_card',
    '_remove_robot_card',
    'card_robot_type',
    '_card_topic_rid',
    '_create_extra_robot_section',
    '_on_extra_mode_selected',
    '_build_extra_joint_sliders',
    '_on_card_type_changed',
    '_on_card_station_changed',
    '_reset_extra_robot_defaults',
]


class _Dropdown:
    """Minimal QComboBox stub."""

    def __init__(self, text):
        """Store the fixed robot name."""
        self._text = text

    def currentText(self):
        """Return the fixed robot name."""
        return self._text


class _Host(MultiRobotMixin):
    """Minimal host exercising the moved card helpers."""

    def __init__(self):
        """Wire the per-card state the helpers read."""
        self.robot_dropdown = _Dropdown('KinovaGen3_7DOF')
        self._card_robot_types = {}
        self._card_station = {}


class TestOwnership:
    """The card API moved, and moved cleanly."""

    def test_multi_robot_mixin_owns_the_card_api(self):
        """Every card method is defined on MultiRobotMixin."""
        missing = [n for n in CARD_API if not hasattr(MultiRobotMixin, n)]
        assert missing == []

    def test_robot_control_mixin_no_longer_defines_cards(self):
        """RobotControlMixin keeps none of the card methods."""
        left = [n for n in CARD_API if n in vars(RobotControlMixin)]
        assert left == []


class TestCardRobotType:
    """card_robot_type resolves per-card overrides."""

    def test_card_one_follows_the_global_dropdown(self):
        """Card 1 always reports whatever the toolbar dropdown says."""
        host = _Host()
        host._card_robot_types['robot_1'] = 'Franka_FR3'
        assert host.card_robot_type('robot_1') == 'KinovaGen3_7DOF'

    def test_extra_card_uses_its_own_override(self):
        """Cards 2-4 report their own per-card type."""
        host = _Host()
        host._card_robot_types['robot_2'] = 'Franka_FR3'
        assert host.card_robot_type('robot_2') == 'Franka_FR3'

    def test_extra_card_falls_back_to_dropdown(self):
        """An override-less extra card falls back to the global dropdown."""
        assert _Host().card_robot_type('robot_3') == 'KinovaGen3_7DOF'


class TestCardTopicRid:
    """_card_topic_rid maps a card to the station it drives."""

    def test_uses_assigned_station(self):
        """A card retargeted at station 4 publishes on robot4."""
        host = _Host()
        host._card_station['robot_2'] = 4
        assert host._card_topic_rid('robot_2') == 'robot4'

    def test_defaults_to_its_own_index(self):
        """Without an assignment a card drives its like-numbered station."""
        assert _Host()._card_topic_rid('robot_3') == 'robot3'
