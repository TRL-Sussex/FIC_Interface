"""The H-PMP toggle goes with every Cartesian mode the Studio sends.

The controllers start with H-PMP off and hear only toggle messages sent after they start; the
Studio's first one goes out when the Studio starts. Pressing Cart therefore ran the plain
Cartesian mode while the toggle read H-PMP, until the toggle was clicked. The Studio now sends the
toggle as it shows just before a Cartesian mode (5-8), from the control bar, the toolbar, the
menu, the other robot cards, the fleet page and the mode re-sent when the arm's controller starts;
a Cartesian mode sent any other way (extensions, scripts) carries the last H-PMP setting sent.
"""
import types

from fic_studio.mixins.process_mixin import ProcessMixin
from fic_studio.mixins.robot_control_mixin import RobotControlMixin


class _Toggle:
    def __init__(self, on):
        self.on = on

    def isChecked(self):
        return self.on


class _Host(ProcessMixin, RobotControlMixin):
    def __init__(self, mode_idx=0, hpmp=True):
        self.sent = []
        self.node = types.SimpleNamespace(
            station_id=1,
            publish_mode=lambda m, robot_id=None: self.sent.append(('mode', m, robot_id)),
            publish_hpmp_enable=lambda on, robot_id=None: self.sent.append(('hpmp', on, robot_id)))
        self._rc_selected_mode_idx = mode_idx
        self._rc_hpmp_toggle = _Toggle(hpmp)

    def log_message(self, *a, **k):
        pass


def test_a_cartesian_mode_is_preceded_by_the_toggle():
    """Modes 5-8 send the toggle as it shows."""
    for mode in (5, 6, 7, 8):
        h = _Host()
        h._send_hpmp_for_mode(mode)
        assert h.sent == [('hpmp', True, None)]
    h = _Host(hpmp=False)
    h._send_hpmp_for_mode(8)
    assert h.sent == [('hpmp', False, None)]


def test_joint_modes_send_no_toggle():
    """Modes 1-4 leave H-PMP alone."""
    for mode in (1, 2, 3, 4):
        h = _Host()
        h._send_hpmp_for_mode(mode)
        assert h.sent == []


def test_a_robot_card_sends_its_own_toggle_to_its_robot():
    """Robots 2-4: the card's own toggle, to its robot."""
    h = _Host(hpmp=True)
    h._send_hpmp_for_mode(8, 'robot2', _Toggle(False))
    assert h.sent == [('hpmp', False, 'robot2')]


def test_the_mode_re_sent_at_controller_start_carries_the_toggle_first():
    """The re-sent mode at arm start: toggle first, then the mode."""
    h = _Host(mode_idx=7)
    h._republish_control_mode()
    assert h.sent == [('hpmp', True, None), ('mode', 8, None)]
    h = _Host(mode_idx=0)
    h._republish_control_mode()
    assert h.sent == [('mode', 1, None)]


class _Pub:
    def __init__(self, topic, log):
        self.topic, self.log = topic, log

    def publish(self, msg):
        self.log.append((self.topic, msg.data))


def _node():
    """Return a stand-in running FicStudioNode's publish methods on robot1's publishers."""
    from fic_studio.ros_node import FicStudioNode
    log = []
    n = types.SimpleNamespace(log=log, _hpmp_sent={}, _MODE_NAMES=FicStudioNode._MODE_NAMES,
                              _resolve_rid=lambda rid=None: rid or 'robot1',
                              pubs={'robot1': {k: _Pub(k, log) for k in
                                               ('hpmp_enable', 'mode_select', 'control_mode')}})
    n.publish_mode = lambda m, robot_id=None: FicStudioNode.publish_mode(n, m, robot_id)
    n.publish_hpmp_enable = lambda on, robot_id=None: FicStudioNode.publish_hpmp_enable(n, on, robot_id)
    return n


def test_every_cartesian_mode_any_extension_sends_carries_the_last_hpmp():
    """Extensions that send a mode themselves: the last H-PMP sent (on by default) goes first."""
    n = _node()
    n.publish_mode(8)
    assert n.log[0] == ('hpmp_enable', True) and n.log[-1] == ('control_mode', 8)
    n.log.clear()
    n.publish_hpmp_enable(False)          # e.g. the Script Editor's hpmp(False)
    n.publish_mode(7)
    assert n.log[:2] == [('hpmp_enable', False), ('hpmp_enable', False)]
    n.log.clear()
    n.publish_mode(1)
    assert [t for t, _ in n.log] == ['mode_select', 'control_mode']
