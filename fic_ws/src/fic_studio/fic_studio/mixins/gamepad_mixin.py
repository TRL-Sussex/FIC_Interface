"""USB gamepad (Xbox/compatible) Kinova-style teleop, integrated into the studio.

Reads /dev/input/jsN directly via the Linux joystick protocol (no extra ROS
package required). One controller drives one robot card; plugging in a second
controller assigns it to the next active robot.

Lives as a studio mixin (not an extension) so:
  - configuration sits in the main Settings dialog under a "Gamepad" tab
  - a connection LED can sit in the studio's persistent status strip
  - the joystick read loop and publish loop are always running, even
    when the user isn't on a particular page

Controls follow the Kinova gamepad paradigm:
  - Back button toggles Joint / Cartesian control space
  - Start cycles FIC sub-modes (Impedance/Planner/Admittance/Admittance+Haptic)
  - Both joint and Cartesian spaces are fully functional from the sticks

Joint mode (Kinova-style single-joint):
  - Left stick Y drives the currently selected joint (velocity control)
  - LB/RB cycles which joint is selected (J1 through Jn)
  - Publishes Float32MultiArray on {rid}_desired_joint_angles

Cartesian mode:
  - All 4 DOF on sticks simultaneously (XYZ translation + yaw)
  - No group cycling needed; LB/RB ignored
  - Publishes geometry_msgs/Pose on command_ee_position_change_{rid}
"""
import glob
import math
import os
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from PyQt5.QtCore import QObject, QSocketNotifier, QTimer, Qt, pyqtSignal
from fic_studio import theme
from PyQt5.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QScrollArea, QSizePolicy,
    QVBoxLayout, QWidget,
)
from geometry_msgs.msg import Pose, Twist
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32, Float32MultiArray, Int32, String as StringMsg


# ── Linux joystick protocol ─────────────────────────────────────────────
_JS_EVENT_FMT = "IhBB"
_JS_EVENT_SIZE = struct.calcsize(_JS_EVENT_FMT)
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80

_XPAD_BUTTON_NAMES = {
    0: "A", 1: "B", 2: "X", 3: "Y",
    4: "LB", 5: "RB", 6: "Back", 7: "Start",
    8: "Guide", 9: "LStick", 10: "RStick",
    102: "LT", 105: "RT",
}
_XPAD_AXIS_NAMES = {
    0: "LX", 1: "LY", 2: "LT", 3: "RX", 4: "RY", 5: "RT",
    6: "DPadX", 7: "DPadY",
}

# All assignable button actions
_ACTION_IDS = [
    "none",
    "toggle_space",           # Joint ↔ Cartesian
    "cycle_sub_mode",         # Impedance → Planner → Admittance → …
    "prev_group",             # previous axis/joint group
    "next_group",             # next axis/joint group
    "home",
    "e_stop",
    "toggle_postural",
    "toggle_elbow",
    "cycle_robot_prev",       # previous robot (multi-robot)
    "cycle_robot_next",       # next robot (multi-robot)
    "speed_up",
    "speed_down",
    "reset_target_pose",
]

# Default button-to-action map (Kinova-style layout)
_DEFAULT_BUTTON_MAP = {
    "Back":   "toggle_space",
    "Start":  "cycle_sub_mode",
    "LB":     "prev_group",
    "RB":     "next_group",
    "A":      "home",
    "B":      "e_stop",
    "X":      "toggle_postural",
    "Y":      "toggle_elbow",
    "Guide":  "reset_target_pose",
    "LStick": "none",
    "RStick": "none",
}

_TRIGGER_VIRT_BTN_BASE = 100
_TRIGGER_AXIS_TO_NAME = {2: "LT", 5: "RT"}
_TRIGGER_PRESS_THRESHOLD = 0.5
_TRIGGER_RELEASE_THRESHOLD = 0.3

# D-pad axes → synthetic directional events
_DPAD_X_AXIS = 6
_DPAD_Y_AXIS = 7

PUBLISH_RATE_HZ = 100
SCAN_RATE_HZ = 2
LIVE_TIMEOUT_S = 0.75

# ── Control modes (must match fic_simulation.h ControlMode enum) ─────
_JOINT_MODES = [
    (1, "Joint Impedance"),
    (2, "Joint Impedance With Planner"),
    (3, "Joint Admittance"),
    (4, "Joint Admittance Haptic"),
]
_CARTESIAN_MODES = [
    (5, "Cartesian Impedance"),
    (6, "Cartesian Planner"),
    (7, "Cartesian Admittance"),
    (8, "Cartesian Admittance Haptic"),
]

# ── Speed presets ────────────────────────────────────────────────────────
_SPEED_PRESETS = [
    {"name": "Slow",   "step_mm": 0.5, "step_deg": 0.25, "joint_vel": 10.0},
    {"name": "Medium", "step_mm": 1.0, "step_deg": 0.5,  "joint_vel": 25.0},
    {"name": "Fast",   "step_mm": 2.0, "step_deg": 1.0,  "joint_vel": 50.0},
]

# ── Joint mode ───────────────────────────────────────────────────────────
# Left stick Y drives the selected joint, right stick Y drives the next one.
# LB/RB cycles which joint is selected. Scale sets stick-up = positive joint dir.
_JOINT_SCALES = {"LY": -1.0, "RY": -1.0}

# ── Cartesian axes ───────────────────────────────────────────────────────
# All 4 stick axes active simultaneously (no group cycling).
# (stick_axis_name, output_index, scale)
# output_index: 0=tx, 1=ty, 2=tz, 3=rx, 4=ry, 5=rz
_CART_GROUPS = [
    {
        "label": "XYZ+Yaw",
        "axes": [
            ("LX", 1, -1.0),  # LStick X → Y translation (left/right)
            ("LY", 2, -1.0),  # LStick Y → Z translation (up/down)
            ("RY", 0, +1.0),  # RStick Y → X translation (forward/back)
            ("RX", 5, -1.0),  # RStick X → yaw rotation
        ],
    },
]

# Legged robots: sticks map to a body-velocity command, not to a pose.
_LEGGED_AXES = (
    ("LY", 'vx', +1.0),   # left stick forward/back
    ("LX", 'vy', -1.0),   # left stick left/right (strafe)
    ("RX", 'yaw', -1.0),  # right stick left/right (turn)
)
_LEGGED_SCALE = {'vx': 0.6, 'vy': 0.4, 'yaw': 1.0}


@dataclass
class _PadState:
    path: str
    fd: int = -1
    notifier: Optional[QSocketNotifier] = None
    name: str = ""
    axes: dict = field(default_factory=dict)
    buttons: dict = field(default_factory=dict)
    last_event_ts: float = 0.0
    target_robot_n: int = 1

    # Kinova-style state
    space: str = "cartesian"      # "joint" or "cartesian"
    selected_joint: int = 0       # joint mode: which joint (0..n-1) is active
    sub_mode_index: int = 0       # index into _JOINT_MODES or _CARTESIAN_MODES
    speed_level: int = 1          # index into _SPEED_PRESETS

    # Joint control state
    joint_targets: Optional[list] = None
    joint_seeded: bool = False

    # Cartesian control state
    target_pose: Optional[Pose] = None
    cart_seeded: bool = False
    seed_source: str = ""

    # Gripper state (0.0 = closed, 1.0 = open)
    gripper_width: float = 0.0
    _last_gripper_published: float = -1.0

    # Toggle states
    postural_on: bool = False
    elbow_on: bool = False
    mode_published: bool = False

    # D-pad edge detection: track previous axis values
    _dpad_prev_x: int = 0
    _dpad_prev_y: int = 0

    _warned_no_live: bool = False
    _warned_no_seed: bool = False
    _warned_no_joints: bool = False


class GamepadSignals(QObject):
    connected_changed = pyqtSignal(bool)
    state_changed = pyqtSignal(str)   # emits summary string for status strip


class GamepadMixin:
    """Mix-in for ProfessionalRobotGUI: USB gamepad Kinova-style teleop.

    Public surface:
        _init_gamepad()                — call from MainWindow.__init__
        _teardown_gamepad()            — call from MainWindow.closeEvent
        _build_gamepad_settings_tab()  — used by SettingsMixin to add a tab
        gamepad_signals.connected_changed — pyqtSignal(bool) for the LED
        gamepad_signals.state_changed     — pyqtSignal(str) for status text
    """

    # ─── Lifecycle ────────────────────────────────────────────────────
    def _init_gamepad(self) -> None:
        self.gamepad_signals = GamepadSignals(self)
        self._gp_pads: dict[str, _PadState] = {}
        self._gp_pose_pubs: dict[int, object] = {}
        self._gp_last_connected = False

        gp = self._gp_settings()
        self._gp_deadzone = float(gp.get("deadzone", 0.12))
        self._gp_invert = dict(gp.get("invert_axes", {}))
        saved_map = gp.get("button_map", {})
        self._gp_button_map = dict(_DEFAULT_BUTTON_MAP)
        self._gp_button_map.update(saved_map)

        self._gp_scan_timer = QTimer(self)
        self._gp_scan_timer.timeout.connect(self._gp_scan_devices)
        self._gp_scan_timer.start(int(1000 / SCAN_RATE_HZ))

        self._gp_publish_timer = QTimer(self)
        self._gp_publish_timer.timeout.connect(self._gp_publish_tick)
        self._gp_publish_timer.start(int(1000 / PUBLISH_RATE_HZ))

        self._gp_prev_targets: dict[int, str] = {}
        self._gp_scan_devices()

    def _teardown_gamepad(self) -> None:
        for t in (getattr(self, "_gp_scan_timer", None),
                  getattr(self, "_gp_publish_timer", None)):
            if t is not None:
                t.stop()
        for path in list(getattr(self, "_gp_pads", {}).keys()):
            self._gp_drop_pad(path)
        self._gp_pose_pubs.clear()

    def _gp_settings(self) -> dict:
        return self.settings.setdefault("gamepad", {})

    def _gp_save_settings(self) -> None:
        self._save_settings()

    # ─── Hot-plug scanning ────────────────────────────────────────────
    def _gp_scan_devices(self) -> None:
        present = set(glob.glob("/dev/input/js*"))
        added = sorted(present - set(self._gp_pads.keys()))
        removed = [p for p in list(self._gp_pads.keys()) if p not in present]
        for path in added:
            self._gp_add_pad(path)
        for path in removed:
            self._gp_drop_pad(path)
        self._gp_emit_connected()
        if added or removed:
            self._gp_refresh_cards()

    def _gp_add_pad(self, path: str) -> None:
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            self.log_message("WARN", f"Failed to open {path}: {e}", "Gamepad")
            return
        pad = _PadState(path=path, fd=fd, name=_pad_display_name(path))
        notifier = QSocketNotifier(fd, QSocketNotifier.Read)
        notifier.activated.connect(lambda _f, p=pad: self._gp_on_pad_readable(p))
        pad.notifier = notifier
        active = list(getattr(self.node, "active_robots", [1])) or [1]
        pad.target_robot_n = active[min(len(self._gp_pads), len(active) - 1)]
        self._gp_pads[path] = pad
        self.log_message(
            "INFO",
            f"Gamepad connected: {path} ({pad.name}) -> robot {pad.target_robot_n}",
            "Gamepad")

    def _gp_drop_pad(self, path: str) -> None:
        pad = self._gp_pads.pop(path, None)
        if pad is None:
            return
        if pad.notifier is not None:
            pad.notifier.setEnabled(False)
            pad.notifier.deleteLater()
        if pad.fd >= 0:
            try:
                os.close(pad.fd)
            except OSError:
                pass
        self.log_message("INFO", f"Gamepad disconnected: {path}", "Gamepad")

    def _gp_emit_connected(self) -> None:
        connected = bool(self._gp_pads)
        if connected != self._gp_last_connected:
            self._gp_last_connected = connected
            self.gamepad_signals.connected_changed.emit(connected)
            if not connected:
                self.gamepad_signals.state_changed.emit("")

    # ─── Event read path ──────────────────────────────────────────────
    def _gp_on_pad_readable(self, pad: _PadState) -> None:
        try:
            while True:
                buf = os.read(pad.fd, _JS_EVENT_SIZE * 32)
                if not buf:
                    break
                for off in range(0, len(buf), _JS_EVENT_SIZE):
                    chunk = buf[off:off + _JS_EVENT_SIZE]
                    if len(chunk) != _JS_EVENT_SIZE:
                        break
                    _, value, etype, number = struct.unpack(_JS_EVENT_FMT, chunk)
                    pad.last_event_ts = time.monotonic()
                    is_init = bool(etype & _JS_EVENT_INIT)
                    etype &= ~_JS_EVENT_INIT
                    if etype == _JS_EVENT_AXIS:
                        norm = max(-1.0, min(1.0, value / 32767.0))
                        pad.axes[number] = norm
                        if number in _TRIGGER_AXIS_TO_NAME and not is_init:
                            virt_idx = _TRIGGER_VIRT_BTN_BASE + number
                            prev_t = pad.buttons.get(virt_idx, False)
                            if not prev_t and norm >= _TRIGGER_PRESS_THRESHOLD:
                                pad.buttons[virt_idx] = True
                                self._gp_fire_button(pad, virt_idx)
                            elif prev_t and norm <= _TRIGGER_RELEASE_THRESHOLD:
                                pad.buttons[virt_idx] = False
                        if not is_init:
                            self._gp_handle_dpad(pad, number, value)
                    elif etype == _JS_EVENT_BUTTON:
                        prev = pad.buttons.get(number, False)
                        now = bool(value)
                        pad.buttons[number] = now
                        if not is_init and now and not prev:
                            self._gp_fire_button(pad, number)
        except BlockingIOError:
            pass
        except OSError as e:
            self.log_message("WARN", f"Pad read error on {pad.path}: {e}", "Gamepad")

    # ─── D-pad edge detection ─────────────────────────────────────────
    def _gp_handle_dpad(self, pad: _PadState, axis_num: int, raw_value: int) -> None:
        if axis_num == _DPAD_X_AXIS:
            direction = 1 if raw_value > 0 else (-1 if raw_value < 0 else 0)
            if direction != 0 and pad._dpad_prev_x == 0:
                self._gp_fire_dpad(pad, "x", direction)
            pad._dpad_prev_x = direction
        elif axis_num == _DPAD_Y_AXIS:
            direction = 1 if raw_value > 0 else (-1 if raw_value < 0 else 0)
            if direction != 0 and pad._dpad_prev_y == 0:
                self._gp_fire_dpad(pad, "y", direction)
            pad._dpad_prev_y = direction

    def _gp_fire_dpad(self, pad: _PadState, axis: str, direction: int) -> None:
        if axis == "y":
            # D-pad Y: up (negative raw) = speed up, down (positive) = speed down
            if direction < 0:
                self._gp_change_speed(pad, +1)
            else:
                self._gp_change_speed(pad, -1)
        elif axis == "x":
            # D-pad X: left = prev robot, right = next robot
            self._gp_cycle_robot(pad, direction)

    def _gp_change_speed(self, pad: _PadState, direction: int) -> None:
        new_level = pad.speed_level + direction
        if new_level < 0 or new_level >= len(_SPEED_PRESETS):
            return
        pad.speed_level = new_level
        preset = _SPEED_PRESETS[pad.speed_level]
        self.log_message(
            "INFO",
            f"Gamepad: Speed -> {preset['name']} "
            f"({preset['step_mm']}mm, {preset['step_deg']}deg, "
            f"{preset['joint_vel']}deg/s)",
            "Gamepad")
        self._gp_emit_state(pad)

    def _gp_cycle_robot(self, pad: _PadState, direction: int) -> None:
        active = list(getattr(self.node, "active_robots", []))
        if not active:
            return
        try:
            idx = active.index(pad.target_robot_n)
        except ValueError:
            idx = 0
        pad.target_robot_n = active[(idx + direction) % len(active)]
        self._gp_reset_pad_state(pad)
        self.log_message(
            "INFO", f"Gamepad: Target -> robot{pad.target_robot_n}", "Gamepad")
        self._gp_emit_state(pad)

    # ─── Publish loop (100 Hz) ────────────────────────────────────────
    def _gp_publish_tick(self) -> None:
        for pad in self._gp_pads.values():
            if not self._gp_is_live(pad):
                continue
            if self._gp_is_legged_pad(pad):
                self._gp_publish_locomotion_tick(pad)
                continue
            self._gp_publish_gripper_tick(pad)
            if pad.space == "joint":
                self._gp_publish_joint_tick(pad)
            else:
                self._gp_publish_cartesian_tick(pad)

    def _gp_is_legged_pad(self, pad: _PadState) -> bool:
        """Return True when this pad targets a legged robot."""
        if not hasattr(self, '_loco_is_legged'):
            return False
        return self._loco_is_legged(
            self._gp_robot_type_for(f"robot{pad.target_robot_n}"))

    def _gp_publish_locomotion_tick(self, pad: _PadState) -> None:
        """Map the sticks to a Twist for the locomotion controller."""
        rid = f"robot{pad.target_robot_n}"
        pub = self.node.pubs.get(rid, {}).get('cmd_vel')
        if pub is None:
            return
        command = {'vx': 0.0, 'vy': 0.0, 'yaw': 0.0}
        for axis_name, key, sign in _LEGGED_AXES:
            command[key] = (self._gp_stick(axis_name, pad) * sign
                            * _LEGGED_SCALE[key])
        msg = Twist()
        msg.linear.x = command['vx']
        msg.linear.y = command['vy']
        msg.angular.z = command['yaw']
        pub.publish(msg)

    def _gp_publish_gripper_tick(self, pad: _PadState) -> None:
        """Velocity-based variable gripper from trigger axes.

        RT (axis 5) = open, LT (axis 2) = close.
        Trigger deflection controls the rate of gripper movement:
        harder press = faster open/close.  No trigger = gripper stays put.
        Trigger raw range: -1.0 (released) to +1.0 (fully pressed).
        """
        rt_raw = pad.axes.get(5, -1.0)
        lt_raw = pad.axes.get(2, -1.0)
        # Normalize: 0 = released, 1 = fully pressed
        rt = max(0.0, (rt_raw + 1.0) / 2.0)
        lt = max(0.0, (lt_raw + 1.0) / 2.0)
        # Net velocity: positive = opening, negative = closing
        net = rt - lt
        if abs(net) < 0.05:
            return
        dt = 1.0 / PUBLISH_RATE_HZ
        # Full trigger deflection opens/closes fully in ~2.5 seconds
        rate = 0.4
        pad.gripper_width = max(0.0, min(1.0, pad.gripper_width + net * rate * dt))
        if abs(pad.gripper_width - pad._last_gripper_published) < 0.005:
            return
        pad._last_gripper_published = pad.gripper_width
        rid = f"robot{pad.target_robot_n}"
        pubs = self.node.pubs.get(rid, {})
        pub = pubs.get("gripper_width")
        if pub is not None:
            pub.publish(Float32(data=float(pad.gripper_width * 0.08)))

    def _gp_publish_joint_tick(self, pad: _PadState) -> None:
        rid = f"robot{pad.target_robot_n}"
        num_joints = self._gp_num_joints(rid)
        if num_joints <= 0:
            return

        if not pad.joint_seeded:
            seeded = self._gp_seed_joint_targets(pad, rid, num_joints)
            if not seeded:
                return

        if not pad.mode_published:
            self._gp_publish_current_mode(pad)

        j1 = pad.selected_joint
        j2 = j1 + 1

        sticks = [
            ("LY", j1),
            ("RY", j2),
        ]
        deflections = [
            (j, self._gp_stick(name, pad) * _JOINT_SCALES[name])
            for name, j in sticks
        ]

        if all(abs(d) < 1e-9 for _, d in deflections):
            return

        preset = _SPEED_PRESETS[pad.speed_level]
        max_vel = preset["joint_vel"]
        dt = 1.0 / PUBLISH_RATE_HZ
        limits = self._gp_joint_limits(rid)

        for j_idx, deflection in deflections:
            if j_idx >= num_joints or abs(deflection) < 1e-9:
                continue
            vel_deg = deflection * max_vel
            pad.joint_targets[j_idx] += math.radians(vel_deg) * dt
            if limits and j_idx < len(limits):
                lo, hi = limits[j_idx]
                pad.joint_targets[j_idx] = max(lo, min(hi, pad.joint_targets[j_idx]))

        pubs = self.node.pubs.get(rid, {})
        pub = pubs.get("desired_joint_angles")
        if pub is not None:
            msg = Float32MultiArray()
            msg.data = [float(v) for v in pad.joint_targets]
            pub.publish(msg)

    def _gp_publish_cartesian_tick(self, pad: _PadState) -> None:
        rid = f"robot{pad.target_robot_n}"

        if not pad.cart_seeded:
            seeded = self._gp_seed_cartesian_target(pad, rid)
            if not seeded:
                return

        if not pad.mode_published:
            self._gp_publish_current_mode(pad)

        group = _CART_GROUPS[0]
        disp = [0.0] * 6  # tx, ty, tz, rx, ry, rz

        for axis_name, out_idx, scale in group["axes"]:
            v = self._gp_stick(axis_name, pad)
            if abs(v) > 1e-9:
                disp[out_idx] = v * scale

        if all(abs(d) < 1e-9 for d in disp):
            return

        preset = _SPEED_PRESETS[pad.speed_level]
        mm = preset["step_mm"] * 0.001
        deg = math.radians(preset["step_deg"])

        dx = disp[0] * mm
        dy = disp[1] * mm
        dz = disp[2] * mm
        rx = disp[3] * deg
        ry = disp[4] * deg
        rz = disp[5] * deg

        _apply_displacement(pad.target_pose, dx, dy, dz, rx, ry, rz)

        pub = self._gp_get_pose_pub(pad.target_robot_n)
        if pub is not None:
            pub.publish(pad.target_pose)

    # ─── Stick reading with deadzone + inversion ──────────────────────
    def _gp_stick(self, axis_name: str, pad: _PadState) -> float:
        idx = _axis_index(axis_name)
        if idx is None:
            return 0.0
        v = pad.axes.get(idx, 0.0)
        if self._gp_invert.get(axis_name, False):
            v = -v
        if abs(v) < self._gp_deadzone:
            return 0.0
        sign = 1.0 if v >= 0 else -1.0
        return sign * (abs(v) - self._gp_deadzone) / (1.0 - self._gp_deadzone)

    # ─── Seeding ──────────────────────────────────────────────────────
    def _gp_seed_joint_targets(self, pad: _PadState, rid: str, num_joints: int) -> bool:
        positions = getattr(self, "current_joint_positions_by_robot", {}).get(rid, [])
        if positions and len(positions) >= num_joints:
            pad.joint_targets = list(positions[:num_joints])
            pad.joint_seeded = True
            self.log_message(
                "INFO", f"{rid}: seeded joint targets from live state", "Gamepad")
            return True
        if not pad._warned_no_joints:
            pad._warned_no_joints = True
            self.log_message(
                "WARN",
                f"{rid}: no joint state feedback yet -- can't seed joint targets",
                "Gamepad")
        return False

    def _gp_seed_cartesian_target(self, pad: _PadState, rid: str) -> bool:
        ee_pose_for = getattr(self, "ee_pose_for", None)
        if callable(ee_pose_for):
            live = ee_pose_for(rid)
            if live is not None:
                pad.target_pose = _clone_pose(live)
                pad.cart_seeded = True
                pad.seed_source = "live"
                return True

        cfg_name = self._gp_robot_type_for(rid)
        cfg = self.robot_configs.get(cfg_name, {}) if hasattr(self, "robot_configs") else {}
        target = cfg.get("default_ee_target")
        if target and target.get("position") and target.get("orientation"):
            pos, ori = target["position"], target["orientation"]
            if len(pos) >= 3 and len(ori) >= 4:
                p = Pose()
                p.position.x = float(pos[0])
                p.position.y = float(pos[1])
                p.position.z = float(pos[2])
                p.orientation.x = float(ori[0])
                p.orientation.y = float(ori[1])
                p.orientation.z = float(ori[2])
                p.orientation.w = float(ori[3])
                pad.target_pose = p
                pad.cart_seeded = True
                pad.seed_source = "yaml default"
                if not pad._warned_no_live:
                    pad._warned_no_live = True
                    self.log_message(
                        "INFO",
                        f"{rid}: seeded target pose from default_ee_target "
                        f"(no live ee_current_pose).",
                        "Gamepad")
                return True

        if not pad._warned_no_seed:
            pad._warned_no_seed = True
            self.log_message(
                "WARN",
                f"{rid}: no live pose and no default_ee_target -- can't "
                f"seed Cartesian target.",
                "Gamepad")
        pad.seed_source = "(none)"
        return False

    # ─── Robot config helpers ─────────────────────────────────────────
    def _gp_robot_type_for(self, rid: str) -> str:
        if hasattr(self, "_card_robot_types"):
            widget_id = rid.replace("robot", "robot_") if rid.startswith("robot") else rid
            t = self._card_robot_types.get(widget_id)
            if t:
                return t
        return self.robot_dropdown.currentText() if hasattr(self, "robot_dropdown") else ""

    def _gp_num_joints(self, rid: str) -> int:
        cfg_name = self._gp_robot_type_for(rid)
        cfg = self.robot_configs.get(cfg_name, {}) if hasattr(self, "robot_configs") else {}
        return int(cfg.get("num_joints", 7))

    def _gp_joint_limits(self, rid: str) -> list:
        """Return list of (lo_rad, hi_rad) tuples, or empty list if unavailable."""
        cfg_name = self._gp_robot_type_for(rid)
        cfg = self.robot_configs.get(cfg_name, {}) if hasattr(self, "robot_configs") else {}
        limits_deg = cfg.get("joint_limits_deg", {})
        if not limits_deg:
            return []
        result = []
        for i in range(self._gp_num_joints(rid)):
            key = f"joint_{i+1}"
            lim = limits_deg.get(key)
            if lim and len(lim) >= 2:
                result.append((math.radians(lim[0]), math.radians(lim[1])))
            else:
                result.append((-math.pi, math.pi))
        return result

    # ─── Mode publishing ──────────────────────────────────────────────
    def _gp_publish_current_mode(self, pad: _PadState) -> None:
        rid = f"robot{pad.target_robot_n}"
        modes = _JOINT_MODES if pad.space == "joint" else _CARTESIAN_MODES
        mode_int, mode_name = modes[pad.sub_mode_index % len(modes)]
        pubs = self.node.pubs.get(rid, {})
        if "control_mode" in pubs:
            pubs["control_mode"].publish(Int32(data=mode_int))
        if "mode_select" in pubs:
            pubs["mode_select"].publish(StringMsg(data=mode_name))
        pad.mode_published = True
        self.log_message(
            "INFO",
            f"{rid}: set {mode_name} (control_mode={mode_int}) for gamepad",
            "Gamepad")

    # ─── Action registry (button presses) ─────────────────────────────
    def _gp_fire_button(self, pad: _PadState, button_idx: int) -> None:
        name = _XPAD_BUTTON_NAMES.get(button_idx, f"Btn{button_idx}")
        if name in ("LT", "RT"):
            return  # triggers drive variable gripper in the publish tick
        action = self._gp_button_map.get(name, "none")
        if action == "none":
            return
        self._gp_execute_action(pad, action, name)

    def _gp_execute_action(self, pad: _PadState, action: str, source: str = "") -> None:
        rid = f"robot{pad.target_robot_n}"
        pubs = self.node.pubs.get(rid, {})

        if action == "toggle_space":
            self._gp_toggle_space(pad)
        elif action == "cycle_sub_mode":
            self._gp_cycle_sub_mode(pad)
        elif action == "prev_group":
            self._gp_cycle_group(pad, -1)
        elif action == "next_group":
            self._gp_cycle_group(pad, +1)
        elif action == "home":
            self._gp_go_home(pad, rid, pubs)
            self.log_message("INFO", f"{source} -> Home on {rid}", "Gamepad")
        elif action == "e_stop":
            if hasattr(self, "_emergency_stop"):
                self._emergency_stop()
            else:
                self._gp_broadcast_stop()
            self.log_message("INFO", f"{source} -> E-STOP", "Gamepad")
        elif action == "toggle_postural":
            pad.postural_on = not pad.postural_on
            try:
                self.node.publish_postural_enable(pad.postural_on, robot_id=rid)
            except Exception as e:
                self.log_message("WARN", f"{rid}: postural toggle failed: {e}", "Gamepad")
                return
            self.log_message(
                "INFO",
                f"{source} -> postural {'ON' if pad.postural_on else 'OFF'} on {rid}",
                "Gamepad")
        elif action == "toggle_elbow":
            pad.elbow_on = not pad.elbow_on
            try:
                self.node.publish_elbow_enable(pad.elbow_on, robot_id=rid)
            except Exception as e:
                self.log_message("WARN", f"{rid}: elbow toggle failed: {e}", "Gamepad")
                return
            self.log_message(
                "INFO",
                f"{source} -> elbow {'ON' if pad.elbow_on else 'OFF'} on {rid}",
                "Gamepad")
        elif action == "cycle_robot_prev":
            self._gp_cycle_robot(pad, -1)
        elif action == "cycle_robot_next":
            self._gp_cycle_robot(pad, +1)
        elif action == "speed_up":
            self._gp_change_speed(pad, +1)
        elif action == "speed_down":
            self._gp_change_speed(pad, -1)
        elif action == "reset_target_pose":
            self._gp_reset_pad_state(pad)
            self.log_message("INFO", f"{source} -> reset target pose on {rid}", "Gamepad")

    # ─── Home ─────────────────────────────────────────────────────────
    def _gp_go_home(self, pad: _PadState, rid: str, pubs: dict) -> None:
        """Send the robot to its q_home_deg joint configuration."""
        cfg_name = self._gp_robot_type_for(rid)
        cfg = self.robot_configs.get(cfg_name, {}) if hasattr(self, "robot_configs") else {}
        q_home_deg = cfg.get("q_home_deg")
        if not q_home_deg:
            self.log_message("WARN", f"{rid}: no q_home_deg in config", "Gamepad")
            return
        q_home_rad = [math.radians(float(d)) for d in q_home_deg]
        if "control_mode" in pubs:
            pubs["control_mode"].publish(Int32(data=1))
        if "mode_select" in pubs:
            pubs["mode_select"].publish(StringMsg(data="Joint Impedance"))
        pub = pubs.get("desired_joint_angles")
        if pub is not None:
            msg = Float32MultiArray()
            msg.data = [float(v) for v in q_home_rad]
            pub.publish(msg)
        pad.space = "joint"
        pad.sub_mode_index = 0
        pad.selected_joint = 0
        pad.joint_targets = list(q_home_rad)
        pad.joint_seeded = True
        pad.mode_published = True
        pad.cart_seeded = False
        pad.target_pose = None
        self._gp_emit_state(pad)

    # ─── Space / group / sub-mode cycling ─────────────────────────────
    def _gp_toggle_space(self, pad: _PadState) -> None:
        if pad.space == "joint":
            pad.space = "cartesian"
            pad.cart_seeded = False
            pad.target_pose = None
            pad._warned_no_live = False
            pad._warned_no_seed = False
        else:
            pad.space = "joint"
            pad.selected_joint = 0
            pad.joint_seeded = False
            pad.joint_targets = None
            pad._warned_no_joints = False
        pad.mode_published = False
        self._gp_publish_current_mode(pad)
        self.log_message(
            "INFO",
            f"Gamepad: switched to {pad.space.title()} mode",
            "Gamepad")
        self._gp_emit_state(pad)

    def _gp_cycle_group(self, pad: _PadState, direction: int) -> None:
        if pad.space != "joint":
            return
        rid = f"robot{pad.target_robot_n}"
        num_joints = self._gp_num_joints(rid)
        if num_joints <= 0:
            return
        pad.selected_joint = (pad.selected_joint + direction) % num_joints
        self.log_message(
            "INFO",
            f"Gamepad: Joint {pad.selected_joint + 1} selected",
            "Gamepad")
        self._gp_emit_state(pad)

    def _gp_cycle_sub_mode(self, pad: _PadState) -> None:
        modes = _JOINT_MODES if pad.space == "joint" else _CARTESIAN_MODES
        pad.sub_mode_index = (pad.sub_mode_index + 1) % len(modes)
        pad.mode_published = False
        self._gp_publish_current_mode(pad)
        self._gp_emit_state(pad)

    # ─── State summary for status strip ───────────────────────────────
    def _gp_emit_state(self, pad: _PadState) -> None:
        modes = _JOINT_MODES if pad.space == "joint" else _CARTESIAN_MODES
        _, mode_name = modes[pad.sub_mode_index % len(modes)]
        short_mode = mode_name.split()[-1]
        speed = _SPEED_PRESETS[pad.speed_level]["name"]
        if pad.space == "joint":
            j1 = pad.selected_joint + 1
            j2 = pad.selected_joint + 2
            summary = f"Joint J{j1},J{j2} {speed} | {short_mode}"
        else:
            summary = f"Cart XYZ+Yaw {speed} | {short_mode}"
        self.gamepad_signals.state_changed.emit(summary)

    # ─── Helpers ──────────────────────────────────────────────────────
    def _gp_reset_pad_state(self, pad: _PadState) -> None:
        pad.joint_targets = None
        pad.joint_seeded = False
        pad.target_pose = None
        pad.cart_seeded = False
        pad.seed_source = ""
        pad.mode_published = False
        pad._warned_no_live = False
        pad._warned_no_seed = False
        pad._warned_no_joints = False

    def _gp_get_pose_pub(self, robot_n: int):
        pub = self._gp_pose_pubs.get(robot_n)
        if pub is not None:
            return pub
        rid = f"robot{robot_n}"
        pubs = self.node.pubs.get(rid)
        if pubs and "position" in pubs:
            self._gp_pose_pubs[robot_n] = pubs["position"]
            return self._gp_pose_pubs[robot_n]
        return None

    def _gp_publish_mode_select(self, pubs: dict, mode: str) -> None:
        pub = pubs.get("mode_select")
        if pub is not None:
            pub.publish(StringMsg(data=mode))

    def _gp_publish_gripper(self, pubs: dict, width: float) -> None:
        pub = pubs.get("gripper_width")
        if pub is not None:
            pub.publish(Float32(data=float(width)))

    def _gp_broadcast_stop(self) -> None:
        for n in getattr(self.node, "active_robots", []):
            pubs = self.node.pubs.get(f"robot{n}", {})
            self._gp_publish_mode_select(pubs, "Stop Controller")

    @staticmethod
    def _gp_is_live(pad: _PadState) -> bool:
        if pad.last_event_ts <= 0.0:
            return False
        return (time.monotonic() - pad.last_event_ts) < LIVE_TIMEOUT_S

    # ─── Settings panel ───────────────────────────────────────────────
    def _build_gamepad_settings_tab(self) -> QWidget:
        tab = QWidget()
        outer = QVBoxLayout(tab)
        outer.setAlignment(Qt.AlignTop)

        # ── Tuning ──
        tune_grp = QGroupBox("TUNING")
        grid = QGridLayout(tune_grp)
        grid.addWidget(QLabel("Deadzone"), 0, 0)
        self._gp_deadzone_spin = self._gp_make_spin(0.0, 0.5, 0.01, self._gp_deadzone)
        self._gp_deadzone_spin.valueChanged.connect(self._gp_on_deadzone)
        grid.addWidget(self._gp_deadzone_spin, 0, 1)
        outer.addWidget(tune_grp)

        # ── Axis inversion ──
        invert_grp = QGroupBox("AXIS INVERSION")
        invert_layout = QGridLayout(invert_grp)
        self._gp_invert_checks = {}
        axes = ["LX", "LY", "RX", "RY"]
        labels = ["Left Stick X", "Left Stick Y", "Right Stick X", "Right Stick Y"]
        for i, (ax, label) in enumerate(zip(axes, labels)):
            cb = QCheckBox(label)
            cb.setChecked(self._gp_invert.get(ax, False))
            cb.stateChanged.connect(lambda state, a=ax: self._gp_on_invert_changed(a, state))
            invert_layout.addWidget(cb, i // 2, i % 2)
            self._gp_invert_checks[ax] = cb
        outer.addWidget(invert_grp)

        # ── Button mapping ──
        outer.addWidget(self._gp_build_mapping_editor())

        # ── D-pad + Triggers reference ──
        ref_grp = QGroupBox("FIXED CONTROLS (not remappable)")
        ref_layout = QVBoxLayout(ref_grp)
        ref_text = (
            "LT/RT = Variable gripper (close/open)\n"
            "D-pad Up/Down = Speed level    D-pad Left/Right = Cycle robot"
        )
        ref_label = QLabel(ref_text)
        ref_label.setWordWrap(True)
        theme.bind(
            ref_label,
            lambda t: "color: %s; font-family: monospace; font-size: 11px; padding: 8px;"
            % t["text_secondary"],
        )
        ref_layout.addWidget(ref_label)
        outer.addWidget(ref_grp)

        # ── Live controllers list ──
        cards_grp = QGroupBox("CONNECTED CONTROLLERS")
        cards_outer = QVBoxLayout(cards_grp)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        cards_host = QWidget()
        self._gp_cards_layout = QVBoxLayout(cards_host)
        self._gp_cards_layout.setSpacing(8)
        self._gp_cards_layout.setAlignment(Qt.AlignTop)
        self._gp_empty_label = QLabel("No controllers detected. Plug in a USB gamepad.")
        theme.bind(
            self._gp_empty_label,
            lambda t: "color: %s; padding: 12px;" % t["text_disabled"],
        )
        self._gp_cards_layout.addWidget(self._gp_empty_label)
        scroll.setWidget(cards_host)
        scroll.setMinimumHeight(120)
        cards_outer.addWidget(scroll)
        outer.addWidget(cards_grp)

        self._gp_ui_timer = QTimer(tab)
        self._gp_ui_timer.timeout.connect(self._gp_refresh_settings_ui)
        self._gp_ui_timer.start(200)

        self._gp_refresh_cards()
        return tab

    def _gp_refresh_cards(self) -> None:
        layout = getattr(self, "_gp_cards_layout", None)
        if layout is None:
            return
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w is not None and w is not self._gp_empty_label:
                w.setParent(None)
                w.deleteLater()
            elif w is self._gp_empty_label:
                w.setParent(None)
        if not self._gp_pads:
            layout.addWidget(self._gp_empty_label)
            self._gp_empty_label.show()
            return
        self._gp_empty_label.hide()
        for pad in self._gp_pads.values():
            layout.addWidget(self._gp_make_card(pad))

    def _gp_make_card(self, pad: _PadState) -> QFrame:
        card = QFrame()
        card.setFrameShape(QFrame.StyledPanel)
        layout = QGridLayout(card)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setHorizontalSpacing(10)
        pad_name_html = pad.name
        pad_path_html = pad.path
        title = QLabel()
        title.setTextFormat(Qt.RichText)
        theme.bind(
            title,
            lambda t, n=pad_name_html, p=pad_path_html: (
                "<b>%s</b><br>"
                "<span style='color:%s;font-size:10px;'>%s</span>"
            ) % (n, t["text_disabled"], p),
        )
        layout.addWidget(title, 0, 0, 1, 2)
        layout.addWidget(QLabel("Robot:"), 0, 2)
        combo = QComboBox()
        active = list(getattr(self.node, "active_robots", [1])) or [1]
        for n in active:
            combo.addItem(f"Robot {n}", n)
        idx = combo.findData(pad.target_robot_n)
        if idx >= 0:
            combo.setCurrentIndex(idx)
        combo.currentIndexChanged.connect(
            lambda _i, p=pad, c=combo: self._gp_on_pad_robot_changed(p, c))
        layout.addWidget(combo, 0, 3)
        status = QLabel("waiting...")
        theme.bind(
            status,
            lambda t: (
                "color: %s; font-family: monospace; font-size: 10px;" % t["text_disabled"]
            ),
        )
        layout.addWidget(status, 1, 0, 1, 4)
        layout.setColumnStretch(1, 1)
        card._status = status
        card._pad_path = pad.path
        return card

    def _gp_on_pad_robot_changed(self, pad: _PadState, combo: QComboBox) -> None:
        n = combo.currentData()
        if isinstance(n, int) and n != pad.target_robot_n:
            pad.target_robot_n = n
            self._gp_reset_pad_state(pad)
            self.log_message("INFO", f"{pad.path} -> robot {n}", "Gamepad")

    def _gp_refresh_settings_ui(self) -> None:
        layout = getattr(self, "_gp_cards_layout", None)
        if layout is None:
            return
        for i in range(layout.count()):
            w = layout.itemAt(i).widget()
            if not isinstance(w, QFrame) or not hasattr(w, "_pad_path"):
                continue
            pad = self._gp_pads.get(w._pad_path)
            if pad is None:
                continue
            live = self._gp_is_live(pad)
            axes = " ".join(f"{_XPAD_AXIS_NAMES.get(i, f'A{i}')}:{v:+.2f}"
                            for i, v in sorted(pad.axes.items())[:4])
            held = [_XPAD_BUTTON_NAMES.get(i, f"B{i}")
                    for i, v in pad.buttons.items() if v]
            held_str = ",".join(held) if held else "--"
            speed = _SPEED_PRESETS[pad.speed_level]["name"]
            if not live:
                state = "no events"
            elif pad.space == "joint":
                state = f"joint J{pad.selected_joint+1},J{pad.selected_joint+2} {speed}"
            else:
                state = f"cart XYZ+Yaw {speed}"
            w._status.setText(f"[{state}] {axes}   buttons: {held_str}")

    def _gp_build_mapping_editor(self) -> QGroupBox:
        box = QGroupBox("BUTTON MAPPING")
        grid = QGridLayout(box)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(6)
        # Only show remappable buttons (exclude LT/RT — those are triggers)
        mappable = [n for n in _DEFAULT_BUTTON_MAP.keys()]
        per_col = (len(mappable) + 1) // 2
        for i, name in enumerate(mappable):
            col = (i // per_col) * 2
            row = i % per_col
            grid.addWidget(QLabel(name), row, col)
            combo = QComboBox()
            combo.addItems(_ACTION_IDS)
            current = self._gp_button_map.get(name, "none")
            idx = combo.findText(current)
            combo.setCurrentIndex(idx if idx >= 0 else 0)
            combo.currentTextChanged.connect(
                lambda txt, n=name: self._gp_on_mapping_changed(n, txt))
            grid.addWidget(combo, row, col + 1)
        hint = QLabel(
            "Tip: open the controllers list below while pressing buttons to "
            "confirm which physical button maps to each label."
        )
        theme.bind(hint, lambda t: "color: %s; font-size: 10px;" % t["text_disabled"])
        grid.addWidget(hint, per_col, 0, 1, 4)
        return box

    # ─── Setting writebacks ───────────────────────────────────────────
    def _gp_on_mapping_changed(self, button_name: str, action: str) -> None:
        self._gp_button_map[button_name] = action
        self._gp_settings()["button_map"] = dict(self._gp_button_map)
        self._gp_save_settings()

    def _gp_on_deadzone(self, v: float) -> None:
        self._gp_deadzone = v
        self._gp_settings()["deadzone"] = v
        self._gp_save_settings()

    def _gp_on_invert_changed(self, axis_name: str, state: int) -> None:
        checked = state == Qt.Checked
        self._gp_invert[axis_name] = checked
        self._gp_settings()["invert_axes"] = self._gp_invert
        self._gp_save_settings()

    @staticmethod
    def _gp_make_spin(lo, hi, step, value) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(3)
        s.setValue(value)
        s.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Fixed)
        return s


# ── helpers ────────────────────────────────────────────────────────────
def _axis_index(name: str) -> Optional[int]:
    for idx, n in _XPAD_AXIS_NAMES.items():
        if n == name:
            return idx
    return None


def _pad_display_name(path: str) -> str:
    js_name = os.path.basename(path)
    sys_path = f"/sys/class/input/{js_name}/device/name"
    try:
        with open(sys_path, "r") as f:
            return f.read().strip() or js_name
    except OSError:
        return js_name


def _clone_pose(src: Pose) -> Pose:
    p = Pose()
    p.position.x = src.position.x
    p.position.y = src.position.y
    p.position.z = src.position.z
    p.orientation.x = src.orientation.x
    p.orientation.y = src.orientation.y
    p.orientation.z = src.orientation.z
    p.orientation.w = src.orientation.w
    return p


def _apply_displacement(pose: Pose, dx, dy, dz, rx, ry, rz) -> None:
    """Add a body-frame displacement to `pose` in place.

    dx/dy/dz are in metres (already scaled from mm by the caller).
    rx/ry/rz are in radians.
    """
    q = np.array([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    ])
    rot = R.from_quat(q)
    world_d = rot.apply([dx, dy, dz])
    pose.position.x += world_d[0]
    pose.position.y += world_d[1]
    pose.position.z += world_d[2]
    omega = np.array([rx, ry, rz])
    if np.linalg.norm(omega) > 1e-9:
        delta = R.from_rotvec(omega)
        new_rot = rot * delta
        nq = new_rot.as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            float(nq[0]), float(nq[1]), float(nq[2]), float(nq[3])
        )
