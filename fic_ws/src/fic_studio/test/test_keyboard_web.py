"""Keyboard Control from the web UI.

The web UI (``--web-ui``: the launcher, the Docker image) hides the
classic window, so the keyboard driver used to drop every key ~20 times a second
("is the hidden window the active one?" never), and caught the keys before the
page could see them. Now the page reads the drive keys itself and sends the ones
held down; the Studio lets go of any it has not heard about for 0.3 s.
"""
import math
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from geometry_msgs.msg import Pose  # noqa: E402
from PyQt5.QtCore import QEvent, Qt  # noqa: E402
from PyQt5.QtGui import QKeyEvent  # noqa: E402
from PyQt5.QtWidgets import QApplication, QWidget  # noqa: E402
import pytest  # noqa: E402

from fic_studio.keyboard import driver  # noqa: E402
from fic_studio.mixins import keyboard_mixin  # noqa: E402
from fic_studio.mixins.keyboard_mixin import KeyboardMixin, WEB_STALE_S  # noqa: E402

_app = QApplication.instance() or QApplication([])


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Combo:
    def currentText(self):
        return 'KinovaGen3_7DOF'


class _Studio(KeyboardMixin, QWidget):
    """Just what the keyboard driver reads and writes."""

    def __init__(self):
        QWidget.__init__(self)
        self.logged = []
        self.pubs = {'desired_joint_angles': _Pub(), 'position': _Pub(), 'gripper_width': _Pub()}
        self.node = types.SimpleNamespace(station_robot_id='robot1', pubs={'robot1': self.pubs})
        self.robot_dropdown = _Combo()
        self.robots = {'KinovaGen3_7DOF': 7}
        self.robot_configs = {}
        self.current_joint_positions_by_robot = {'robot1': [0.0] * 7}
        self._rc_selected_mode_idx = 0                  # mode 1: a joint mode
        self.latched = False
        self._init_keyboard()

    def log_message(self, level, text, source=''):
        self.logged.append((level, text))

    def estop_latch_state(self):
        return {'latched': self.latched}

    def ee_pose_for(self, rid):
        pose = Pose()
        pose.position.x, pose.orientation.w = 0.5, 1.0
        return pose


class _Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def clock(monkeypatch):
    c = _Clock()
    monkeypatch.setattr(keyboard_mixin.time, 'monotonic', c)
    return c


@pytest.fixture
def studio(clock):
    s = _Studio()
    s.set_keyboard_control(True)
    yield s
    s._teardown_keyboard()


def _tick(studio, clock, dt=0.05):
    clock.t += dt
    studio._kbd_tick()


def _joint_targets(studio):
    return [list(m.data) for m in studio.pubs['desired_joint_angles'].sent]


def test_keys_from_the_page_drive_the_arm_with_the_classic_window_hidden(studio, clock):
    """The web UI case: the classic window is never shown, let alone active."""
    assert not studio.isActiveWindow()
    assert studio.keyboard_web_keys(['UP'])['ok']
    _tick(studio, clock, 0.1)
    sent = _joint_targets(studio)
    assert len(sent) == 1
    assert sent[0][0] == pytest.approx(math.radians(driver.JOINT_SPEED * 0.1))


def test_keys_the_page_stops_sending_are_let_go(studio, clock):
    """A dropped connection or a hidden tab must not leave the arm moving."""
    studio.keyboard_web_keys(['UP'])
    _tick(studio, clock, 0.1)
    _tick(studio, clock, WEB_STALE_S + 0.05)
    _tick(studio, clock, 0.05)
    assert len(_joint_targets(studio)) == 1


def test_keys_the_page_keeps_sending_keep_driving(studio, clock):
    for _ in range(10):
        studio.keyboard_web_keys(['UP'])
        _tick(studio, clock, 0.1)
    assert len(_joint_targets(studio)) == 10


def test_an_empty_list_stops_at_once(studio, clock):
    studio.keyboard_web_keys(['UP'])
    _tick(studio, clock)
    studio.keyboard_web_keys([])
    _tick(studio, clock)
    assert len(_joint_targets(studio)) == 1


def test_shift_from_the_page_is_faster_and_ctrl_slower(studio, clock):
    studio.keyboard_web_keys(['UP', 'SHIFT'])
    _tick(studio, clock, 0.1)
    studio.keyboard_web_keys(['DOWN', 'CTRL'])
    _tick(studio, clock, 0.1)
    first, second = (t[0] for t in _joint_targets(studio))
    assert first == pytest.approx(math.radians(driver.JOINT_SPEED * driver.FAST * 0.1))
    assert first - second == pytest.approx(math.radians(driver.JOINT_SPEED * driver.SLOW * 0.1))


def test_taps_pick_a_joint_and_swap_the_space(studio, clock):
    reply = studio.keyboard_web_keys([], tap='3')
    assert reply['space'] == 'joint' and reply['joint'] == 2
    studio.keyboard_web_keys(['UP'])
    _tick(studio, clock, 0.1)
    assert _joint_targets(studio)[0][2] > 0 and _joint_targets(studio)[0][0] == 0
    assert studio.keyboard_web_keys([], tap='TAB')['space'] == 'cartesian'
    studio.keyboard_web_keys(['W'])
    _tick(studio, clock, 0.1)
    assert studio.pubs['position'].sent[-1].position.x == pytest.approx(0.5 + driver.CART_SPEED * 0.1)


def test_nothing_moves_while_keyboard_control_is_off(studio, clock):
    studio.set_keyboard_control(False)
    assert studio.keyboard_web_keys(['UP'])['ok'] is False
    studio._kbd_tick()
    assert _joint_targets(studio) == []


def test_a_latched_estop_stops_the_keyboard(studio, clock):
    studio.keyboard_web_keys(['UP'])
    studio.latched = True
    _tick(studio, clock, 0.1)
    studio.latched = False
    _tick(studio, clock, 0.1)                          # and the keys were let go, not held for after Reset
    assert _joint_targets(studio) == []


def _key(kind, key, mods=Qt.NoModifier, text=''):
    return QKeyEvent(kind, key, mods, text)


def test_the_classic_window_leaves_the_keys_to_the_web_page(studio):
    """With the web UI in front, a key goes to the page: the Studio must not swallow it."""
    studio._kbd_classic_active = lambda: False
    assert studio._kbd_on_event(_key(QEvent.KeyPress, Qt.Key_W, text='w')) is False
    assert studio._kbd_keys == set()


def test_the_classic_window_still_drives_when_it_is_in_front(studio):
    studio._kbd_classic_active = lambda: True
    assert studio._kbd_on_event(_key(QEvent.KeyPress, Qt.Key_W, text='w')) is True
    assert studio._kbd_keys == {'W'}


def test_shift_and_ctrl_are_held_in_the_classic_window_too(studio):
    """They never counted: Qt has no text for them, and Ctrl+W has no letter."""
    studio._kbd_classic_active = lambda: True
    studio._kbd_on_event(_key(QEvent.KeyPress, Qt.Key_Shift, Qt.ShiftModifier))
    studio._kbd_on_event(_key(QEvent.KeyPress, Qt.Key_Control, Qt.ShiftModifier | Qt.ControlModifier))
    studio._kbd_on_event(_key(QEvent.KeyPress, Qt.Key_W, Qt.ShiftModifier | Qt.ControlModifier, '\x17'))
    assert studio._kbd_keys == {'SHIFT', 'CTRL', 'W'}
    studio._kbd_on_event(_key(QEvent.KeyRelease, Qt.Key_Shift, Qt.ControlModifier))
    assert studio._kbd_keys == {'CTRL', 'W'}


def test_the_state_says_what_the_keyboard_drives(studio):
    assert studio.keyboard_state() == {'on': True, 'space': 'joint', 'joint': 0}
    studio.keyboard_web_keys([], tap='TAB')
    assert studio.keyboard_state()['space'] == 'cartesian'
