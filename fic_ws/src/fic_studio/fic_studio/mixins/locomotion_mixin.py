"""
Locomotion panel for legged robots.

Shown only when the selected robot declares `robot_category: legged`.
Publishes a Twist velocity command at 20 Hz plus discrete gait commands
on a String topic; the fic_locomotion node turns those into joint
targets that FIC tracks.
"""

import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSlider, QVBoxLayout, QWidget)

# Command grammar understood by the locomotion node.
GAIT_COMMANDS = ('sit', 'stand', 'walk')
VELOCITY_PUBLISH_MS = 50
#: Velocity axes: (key, label, limit). The slider spans -limit..+limit.
VELOCITY_AXES = (
    ('vx', 'Forward (m/s)', 0.6),
    ('vy', 'Strafe (m/s)', 0.4),
    ('yaw', 'Turn (rad/s)', 1.0),
)
#: Live gait parameters: (key, label, low, high, default, step).
GAIT_TUNING = (
    ('step_height', 'Step height (m)', 0.02, 0.20, 0.08, 0.01),
    ('cycle_time', 'Cycle time (s)', 0.20, 0.80, 0.36, 0.02),
    ('stand_height', 'Stand height (m)', 0.30, 0.55, 0.46, 0.01),
)


class LocomotionMixin:
    """Velocity and gait control for quadrupeds."""

    def _init_locomotion(self):
        """Set up locomotion state. Call before the panel is built."""
        self._loco_velocity = (0.0, 0.0, 0.0)
        self._loco_timer = QTimer(self)
        self._loco_timer.timeout.connect(self._loco_publish_velocity)

    # -- robot category --------------------------------------------------
    def _loco_is_legged(self, robot_name: str) -> bool:
        """Return True when this robot is driven by the locomotion controller."""
        configs = getattr(self, 'robot_configs', {})
        return configs.get(robot_name, {}).get('robot_category') == 'legged'

    def _loco_current_is_legged(self):
        """Return True when the robot currently selected in the dropdown is legged."""
        if not hasattr(self, 'robot_dropdown'):
            return False
        return self._loco_is_legged(self.robot_dropdown.currentText())

    def _loco_refresh_visibility(self):
        """Show or hide the sidebar entry to match the selected robot."""
        button = getattr(self, '_locomotion_sidebar_btn', None)
        if button is not None:
            button.setVisible(self._loco_current_is_legged())
        if self._loco_current_is_legged():
            self._loco_timer.start(VELOCITY_PUBLISH_MS)
        else:
            self._loco_timer.stop()

    # -- publishing ------------------------------------------------------
    def _loco_pub(self, key):
        """Return the named locomotion publisher for the active robot, or None."""
        node = getattr(self, 'node', None)
        if node is None:
            return None
        return node.pubs.get(node.station_robot_id, {}).get(key)

    def _loco_publish_velocity(self):
        """Publish the current velocity command as a Twist."""
        from geometry_msgs.msg import Twist
        # A velocity set from the web UI is a hold: it lapses unless renewed,
        # so a closed tab or a dropped connection cannot leave the robot walking.
        until = getattr(self, '_loco_hold_until', None)
        if until is not None and time.monotonic() > until:
            self._loco_velocity = (0.0, 0.0, 0.0)
            self._loco_hold_until = None
        pub = self._loco_pub('cmd_vel')
        if pub is None:
            return
        vx, vy, yaw = self._loco_velocity
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(yaw)
        pub.publish(msg)

    def _loco_send(self, command: str):
        """Send a discrete gait command."""
        from std_msgs.msg import String
        pub = self._loco_pub('gait_command')
        if pub is None:
            return
        pub.publish(String(data=command))
        if hasattr(self, 'log_message'):
            self.log_message('INFO', 'Gait: %s' % command, 'Locomotion')

    def _loco_set(self, key: str, value: float):
        """Push a live gait parameter change to the locomotion node."""
        self._loco_send('set %s %s' % (key, value))

    def _loco_stop(self):
        """Zero the velocity command immediately."""
        self._loco_velocity = (0.0, 0.0, 0.0)
        self._loco_hold_until = None
        for slider in getattr(self, '_loco_sliders', {}).values():
            slider.setValue(0)
        self._loco_publish_velocity()

    # -- panel -----------------------------------------------------------
    def _build_locomotion_page(self) -> QWidget:
        """Build the locomotion control page."""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(self._section_header('Locomotion'))

        layout.addWidget(self._loco_build_gait_group())
        layout.addWidget(self._loco_build_velocity_group())
        layout.addWidget(self._loco_build_tuning_group())
        layout.addStretch()
        return page

    def _loco_build_gait_group(self) -> QGroupBox:
        """Build the discrete gait-command button row."""
        group = QGroupBox('Gait')
        row = QHBoxLayout(group)
        for command in GAIT_COMMANDS:
            button = QPushButton(command.upper())
            button.setCursor(Qt.PointingHandCursor)
            button.clicked.connect(
                lambda _, c=command: self._loco_send(c))
            row.addWidget(button)
        stop = QPushButton('STOP')
        stop.setCursor(Qt.PointingHandCursor)
        stop.clicked.connect(self._loco_stop)
        row.addWidget(stop)
        row.addStretch()
        return group

    def _loco_build_velocity_group(self) -> QGroupBox:
        """Build the forward/strafe/turn velocity slider group."""
        group = QGroupBox('Velocity command')
        grid = QGridLayout(group)
        self._loco_sliders = {}
        self._loco_labels = {}
        for row, (key, label, limit) in enumerate(VELOCITY_AXES):
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-100, 100)
            slider.setValue(0)
            slider.valueChanged.connect(
                lambda value, k=key, s=limit: self._loco_on_slider(k, value, s))
            value_label = QLabel('0.00')
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(value_label, row, 2)
            self._loco_sliders[key] = slider
            self._loco_labels[key] = value_label
        return group

    def _loco_on_slider(self, key: str, value: int, limit: float):
        """Scale a slider's raw int value into the velocity command."""
        scaled = value / 100.0 * limit
        vx, vy, yaw = self._loco_velocity
        if key == 'vx':
            vx = scaled
        elif key == 'vy':
            vy = scaled
        else:
            yaw = scaled
        self._loco_velocity = (vx, vy, yaw)
        self._loco_labels[key].setText('%.2f' % scaled)

    def _loco_build_tuning_group(self) -> QGroupBox:
        """Build the live gait-parameter tuning group."""
        group = QGroupBox('Gait tuning')
        grid = QGridLayout(group)
        for row, (key, label, low, high, default, step) in enumerate(GAIT_TUNING):
            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setSingleStep(step)
            spin.setDecimals(3)
            spin.setValue(default)
            spin.valueChanged.connect(
                lambda value, k=key: self._loco_set(k, round(value, 3)))
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(spin, row, 1)
        return group
