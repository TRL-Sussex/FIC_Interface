# mixins/keyboard_mixin.py — Driving the arm from the keyboard.
"""Keyboard control: the same job as the gamepad, for people without one.

Most visitors to a lab, and most of a workshop room, have no gamepad. This
drives the arm from the keys instead — forward, back, left, right, up, down
and yaw in Cartesian space, one joint at a time in joint space — publishing
exactly what the gamepad publishes, so everything downstream is unchanged.

The mapping itself is in ``keyboard/driver.py``, away from Qt, where it can
be tested. This file is the Studio's side: when the keyboard is allowed to
drive, where the targets come from, and where they go.

**It is off until someone turns it on**, with Ctrl+K or the KBD indicator in
the status bar, and it lets go of the keys when:

  * the window stops being the active one — alt-tabbing away drops the keys,
    so nothing keeps moving while you read something else;
  * the focus is in a box you can type in — a joint number, a file name, the
    script editor. Typing "w" into a field must never drive the arm;
  * an E-stop is latched.

The web UI hides this window, so there the page reads the keys itself (it
knows when someone is typing in one of its boxes) and sends the ones held down
about every 100 ms (``keyboard_web_keys``). Keys it has not repeated for
``WEB_STALE_S`` are let go, so a dropped connection or a hidden tab cannot
leave the arm moving.

Releasing every key stops the motion; the arm holds the last target, as it
does for the gamepad.
"""
import math
import time

from fic_studio.keyboard import driver
from geometry_msgs.msg import Pose
from PyQt5.QtCore import QEvent, QObject, Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QComboBox, QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit,
    QTextEdit, QWidget,
)
from std_msgs.msg import Float32, Float32MultiArray

#: How often the held keys are turned into movement.
TICK_MS = 50
#: A key the web page has not repeated for this long is let go.
WEB_STALE_S = 0.3

#: Qt keys that are not letters or digits.
_SPECIAL = {
    Qt.Key_Up: 'UP', Qt.Key_Down: 'DOWN', Qt.Key_Left: 'LEFT', Qt.Key_Right: 'RIGHT',
    Qt.Key_Tab: 'TAB', Qt.Key_Comma: ',', Qt.Key_Period: '.',
    Qt.Key_Shift: 'SHIFT', Qt.Key_Control: 'CTRL',
}
#: Widgets that swallow the keyboard: typing in one must not move an arm.
_TYPING = (QLineEdit, QTextEdit, QPlainTextEdit, QAbstractSpinBox)


class _KeyWatcher(QObject):
    """Watches the keys for the Studio.

    The window cannot do this itself: QMainWindow comes first in its bases, so
    its own eventFilter is the one Qt finds. This sits on the application,
    which also means a key counts wherever the focus happens to be.
    """

    def __init__(self, studio):
        super().__init__(studio)
        self._studio = studio

    def eventFilter(self, obj, event):
        try:
            if self._studio._kbd_on_event(event):
                return True
        except RuntimeError:
            return False                    # the Studio is going away
        return False


def _key_name(event):
    """Return the driver's name for a key event, or None if it is not one."""
    key = event.key()
    if key in _SPECIAL:
        return _SPECIAL[key]
    # By key, not by text: with Ctrl held the text of W is a control
    # character, and with Shift held the text of 1 is "!".
    if Qt.Key_A <= key <= Qt.Key_Z or Qt.Key_0 <= key <= Qt.Key_9:
        return chr(key)
    return None


class KeyboardMixin:
    """Mixin: the keyboard as an input device for the current robot."""

    # ── Lifecycle ────────────────────────────────────────────────────────
    def _init_keyboard(self):
        """Set up the keyboard driver. Call once the status bar exists."""
        self._kbd_on = False
        self._kbd_keys = set()
        self._kbd_web_keys = set()
        self._kbd_web_time = None
        self._kbd_space = 'cartesian'
        self._kbd_warned = None
        self._kbd_joint = 0
        self._kbd_targets = None
        self._kbd_pose = None
        self._kbd_grip = None
        self._kbd_last_tick = None
        self._kbd_timer = QTimer(self)
        self._kbd_timer.timeout.connect(self._kbd_tick)
        from PyQt5.QtWidgets import QApplication
        self._kbd_watcher = _KeyWatcher(self)
        app = QApplication.instance()
        if app is not None:
            app.installEventFilter(self._kbd_watcher)
        self._kbd_attach_statusbar()
        self._kbd_refresh_indicator()

    def _teardown_keyboard(self):
        timer = getattr(self, '_kbd_timer', None)
        if timer is not None:
            timer.stop()
        watcher = getattr(self, '_kbd_watcher', None)
        if watcher is not None:
            from PyQt5.QtWidgets import QApplication
            app = QApplication.instance()
            if app is not None:
                app.removeEventFilter(watcher)

    # ── On and off ───────────────────────────────────────────────────────
    def keyboard_control_enabled(self):
        """Say whether the keyboard may drive the arm."""
        return bool(getattr(self, '_kbd_on', False))

    def set_keyboard_control(self, on):
        """Turn keyboard control on or off, and say so in the log."""
        on = bool(on)
        if on == getattr(self, '_kbd_on', False):
            return
        self._kbd_on = on
        self._kbd_let_go()
        self._kbd_forget_targets()
        if on:
            self._kbd_space = self._kbd_space_for_mode()
            self._kbd_last_tick = time.monotonic()
            self._kbd_timer.start(TICK_MS)
            # Take the focus off whatever box it was in, or the first W would
            # be a letter in someone's file name rather than a movement.
            from PyQt5.QtWidgets import QApplication
            focused = QApplication.focusWidget()
            if isinstance(focused, _TYPING):
                focused.clearFocus()
                self.setFocus(Qt.OtherFocusReason)
            self.log_message('INFO', 'Keyboard control on — %s space. %s'
                             % (self._kbd_space, driver.HELP[0][1]), 'Keyboard')
        else:
            self._kbd_timer.stop()
            self.log_message('INFO', 'Keyboard control off', 'Keyboard')
        action = getattr(self, 'keyboard_control_action', None)
        if action is not None and action.isChecked() != on:
            action.setChecked(on)
        self._kbd_refresh_indicator()

    def toggle_keyboard_control(self):
        """Flip keyboard control (Ctrl+K, or the KBD indicator)."""
        self.set_keyboard_control(not self.keyboard_control_enabled())

    def _kbd_let_go(self):
        """Release every key, from the window and from the web page."""
        self._kbd_keys.clear()
        self._kbd_web_keys = set()

    def keyboard_state(self):
        """Return what the keyboard drives, for the web UI's status bar."""
        return {'on': self.keyboard_control_enabled(), 'space': self._kbd_space,
                'joint': self._kbd_joint}

    def keyboard_web_keys(self, keys, tap=None):
        """Take the keys the web page holds down, and a key it just pressed.

        ``keys`` replaces the page's previous list: an empty one lets go of
        everything at once. ``tap`` is Tab or a joint number, which act once.
        Returns the state with ``ok`` False while keyboard control is off.
        """
        if not self.keyboard_control_enabled():
            self._kbd_web_keys = set()
            return dict(self.keyboard_state(), ok=False)
        if isinstance(tap, str) and driver.is_tap(tap):
            self._kbd_tap(tap)
        self._kbd_web_keys = driver.held_keys(keys)
        self._kbd_web_time = time.monotonic()
        return dict(self.keyboard_state(), ok=True)

    def _kbd_held(self, now):
        """Return the keys held now: the window's, and the page's while it keeps saying so."""
        if self._kbd_web_keys and (self._kbd_web_time is None
                                   or now - self._kbd_web_time > WEB_STALE_S):
            self._kbd_web_keys = set()
        return self._kbd_keys | self._kbd_web_keys

    def _kbd_estop_latched(self):
        latch = getattr(self, 'estop_latch_state', None)
        try:
            return bool(latch is not None and latch().get('latched'))
        except Exception:
            return False

    def _kbd_classic_active(self):
        """Say whether the classic window is the one in front (never, with the web UI)."""
        from PyQt5.QtWidgets import QApplication
        return QApplication.activeWindow() is self

    def _kbd_forget_targets(self):
        self._kbd_targets = None
        self._kbd_pose = None
        self._kbd_grip = None

    # ── Watching the keys ────────────────────────────────────────────────
    def _kbd_on_event(self, event):
        """Take one application event; return True if the keyboard used it."""
        kind = event.type()
        if kind == QEvent.WindowDeactivate:
            self._kbd_keys.clear()          # alt-tab: let go of the window's keys
            return False
        if kind not in (QEvent.KeyPress, QEvent.KeyRelease):
            return False
        if not self.keyboard_control_enabled():
            return False
        if not self._kbd_classic_active():
            return False                    # the web UI's page reads its own keys
        return self._kbd_handle_key(event, kind == QEvent.KeyPress)

    def _kbd_typing(self):
        """Say whether the focus is somewhere keys mean letters, not motion."""
        from PyQt5.QtWidgets import QApplication
        widget = QApplication.focusWidget()
        if isinstance(widget, QComboBox):
            return widget.isEditable()
        return isinstance(widget, _TYPING)

    def _kbd_handle_key(self, event, pressed):
        """Take one key event; return True if the keyboard driver used it."""
        if event.isAutoRepeat():
            return True                      # we hold it ourselves
        name = _key_name(event)
        if name in driver.MODIFIERS:
            # Held for speed, and passed on: Ctrl+K and the menus still need them.
            (self._kbd_keys.add if pressed else self._kbd_keys.discard)(name)
            return False
        if name is None or not driver.is_ours(name):
            return False
        if self._kbd_typing():
            return False                     # someone is writing, not driving
        if not pressed:
            self._kbd_keys.discard(name)
            return True
        if not self._kbd_tap(name):
            self._kbd_keys.add(name)
        return True

    def _kbd_tap(self, name):
        """Act on Tab or a joint number; return False for any other key."""
        if name == 'TAB':
            self._kbd_space = 'joint' if self._kbd_space == 'cartesian' else 'cartesian'
            self._kbd_warned = None
            self._kbd_forget_targets()
            self.log_message('INFO', 'Keyboard: %s space' % self._kbd_space, 'Keyboard')
            self._kbd_refresh_indicator()
            return True
        picked = driver.picked_joint(name, self._kbd_joint_count())
        if picked is not None:
            self._kbd_joint = picked
            self._kbd_space = 'joint'
            self.log_message('INFO', 'Keyboard: joint J%d' % (picked + 1), 'Keyboard')
            self._kbd_refresh_indicator()
            return True
        return False

    def _kbd_space_for_mode(self):
        """Return the space this robot's control mode actually listens to.

        A Cartesian target means nothing to a joint mode and the other way
        round, so the keyboard starts in whichever space the Studio is in.
        """
        from fic_studio.xml_manager import get_mode_info
        index = int(getattr(self, '_rc_selected_mode_idx', 0))
        return 'cartesian' if get_mode_info(index + 1)['type'] == 'cartesian' else 'joint'

    def _kbd_warn_if_mode_mismatched(self):
        """Say once when the keys are driving a space the mode ignores."""
        wanted = self._kbd_space_for_mode()
        if wanted == self._kbd_space or self._kbd_warned == self._kbd_space:
            return
        self._kbd_warned = self._kbd_space
        self.log_message(
            'WARN', 'Keyboard is in %s space, but this robot is in a %s mode — '
                    'the targets will be ignored until the mode matches'
                    % (self._kbd_space, wanted), 'Keyboard')

    # ── Turning keys into targets ────────────────────────────────────────
    def _kbd_rid(self):
        return getattr(getattr(self, 'node', None), 'station_robot_id', 'robot1')

    def _kbd_joint_count(self):
        name = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
        return int(getattr(self, 'robots', {}).get(name, 7) or 7)

    def _kbd_tick(self):
        now = time.monotonic()
        dt = min(0.25, now - (self._kbd_last_tick or now))
        self._kbd_last_tick = now
        if not self.keyboard_control_enabled():
            return
        if self._kbd_keys and not self._kbd_classic_active():
            self._kbd_keys.clear()
        keys = self._kbd_held(now)
        if not keys or dt <= 0:
            return
        if self._kbd_estop_latched():
            self._kbd_let_go()              # not held over for after Reset
            return
        self._kbd_warn_if_mode_mismatched()
        try:
            if self._kbd_space == 'joint':
                self._kbd_drive_joint(keys, dt)
            else:
                self._kbd_drive_cartesian(keys, dt)
            self._kbd_drive_gripper(keys, dt)
        except Exception as exc:                       # never take the UI down
            self.log_message('WARN', 'Keyboard: %s' % exc, 'Keyboard')

    def _kbd_pubs(self):
        return getattr(getattr(self, 'node', None), 'pubs', {}).get(self._kbd_rid(), {})

    def _kbd_drive_joint(self, keys, dt):
        degrees = driver.joint_step(keys, dt)
        if not degrees:
            return
        rid, count = self._kbd_rid(), self._kbd_joint_count()
        if self._kbd_targets is None:
            live = list(getattr(self, 'current_joint_positions_by_robot', {}).get(rid, []))
            if len(live) < count:
                return                                  # nothing to move from yet
            self._kbd_targets = live[:count]
        index = min(self._kbd_joint, count - 1)
        self._kbd_targets[index] += math.radians(degrees)
        limits = self._kbd_limits(count)
        if limits:
            low, high = limits[index]
            self._kbd_targets[index] = max(low, min(high, self._kbd_targets[index]))
        pub = self._kbd_pubs().get('desired_joint_angles')
        if pub is not None:
            pub.publish(Float32MultiArray(data=[float(v) for v in self._kbd_targets]))

    def _kbd_limits(self, count):
        """Return [(low, high)] in radians from the robot's config, or None."""
        name = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
        limits = (getattr(self, 'robot_configs', {}).get(name, {}) or {}).get('joint_limits_deg')
        if not limits:
            return None
        low, high = limits.get('lower') or [], limits.get('upper') or []
        if len(low) < count or len(high) < count:
            return None
        return [(math.radians(low[i]), math.radians(high[i])) for i in range(count)]

    def _kbd_drive_cartesian(self, keys, dt):
        step = driver.cartesian_step(keys, dt)
        if not any(step.values()):
            return
        rid = self._kbd_rid()
        if self._kbd_pose is None:
            live = self.ee_pose_for(rid) if hasattr(self, 'ee_pose_for') else None
            if live is None:
                return
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (
                live.position.x, live.position.y, live.position.z)
            pose.orientation = live.orientation
            self._kbd_pose = pose
        self._kbd_pose.position.x += step['x']
        self._kbd_pose.position.y += step['y']
        self._kbd_pose.position.z += step['z']
        if step['yaw']:
            self._kbd_yaw(math.radians(step['yaw']))
        pub = self._kbd_pubs().get('position')
        if pub is not None:
            pub.publish(self._kbd_pose)

    def _kbd_yaw(self, radians):
        """Turn the target about the world z axis."""
        half = radians / 2.0
        qz, qw = math.sin(half), math.cos(half)
        o = self._kbd_pose.orientation
        o.x, o.y, o.z, o.w = (
            qw * o.x - qz * o.y,
            qw * o.y + qz * o.x,
            qw * o.z + qz * o.w,
            qw * o.w - qz * o.z,
        )

    def _kbd_drive_gripper(self, keys, dt):
        step = driver.gripper_step(keys, dt)
        if not step:
            return
        if self._kbd_grip is None:
            self._kbd_grip = 0.5            # half open: no reading to start from
        self._kbd_grip = max(0.0, min(1.0, self._kbd_grip + step))
        pub = self._kbd_pubs().get('gripper_width')
        if pub is not None:
            pub.publish(Float32(data=float(self._kbd_grip)))

    # ── The status bar ───────────────────────────────────────────────────
    def _kbd_attach_statusbar(self):
        bar = getattr(self, 'statusbar', None)
        if bar is None:
            return
        from fic_studio import theme
        from fic_studio.widgets import StatusLED
        holder = QWidget()
        row = QHBoxLayout(holder)
        row.setContentsMargins(8, 0, 8, 0)
        row.setSpacing(6)
        self._kbd_led = StatusLED()
        row.addWidget(self._kbd_led)
        self._kbd_label = QLabel('KBD')
        theme.bind(self._kbd_label, lambda t: (
            'color: %s; font-family: monospace; font-size: 10px;' % t['text_secondary']))
        row.addWidget(self._kbd_label)
        holder.setCursor(Qt.PointingHandCursor)
        holder.mousePressEvent = lambda _event: self.toggle_keyboard_control()
        bar.insertWidget(1, holder)

    def _kbd_refresh_indicator(self):
        led, label = getattr(self, '_kbd_led', None), getattr(self, '_kbd_label', None)
        if led is None or label is None:
            return
        if not self.keyboard_control_enabled():
            led.set_status('idle')
            label.setText('KBD off')
            label.setToolTip('Drive the arm from the keyboard (Ctrl+K).')
            return
        led.set_status('active')
        where = ('J%d' % (self._kbd_joint + 1)) if self._kbd_space == 'joint' else 'CART'
        label.setText('KBD %s' % where)
        label.setToolTip('\n'.join('%s: %s' % (k, v) for k, v in driver.HELP))
