from __future__ import annotations
from typing import TYPE_CHECKING, Any, Optional, Callable

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QPushButton, QLabel, QComboBox, QListView

if TYPE_CHECKING:
    from fic_studio.main_window_pro import ProfessionalRobotGUI
    from fic_studio.ros_node import FicStudioNode


class ExtensionHost(QObject):
    """One per loaded extension. Provides controlled access to studio state
    and the multi-robot API. Extensions should never reach past this object
    into MainWindow internals."""

    # Emitted when this extension's selected target robot changes.
    # Payload: the new target rid string ('robot1', 'robot2', …).
    target_robot_changed = pyqtSignal(str)

    # Emitted when the set of active robots changes (add or remove).
    robots_changed = pyqtSignal()

    def __init__(self, main_window: "ProfessionalRobotGUI", extension_id: str):
        super().__init__(main_window)
        self._mw = main_window
        self._id = extension_id
        # Numeric id of the currently selected target robot. None means
        # "follow the first active robot" (legacy single-robot behaviour).
        self._target_robot_n: Optional[int] = None
        # Combos created via make_target_robot_combo, kept for repopulation
        # when robots are added/removed.
        self._target_combos: list[QComboBox] = []

    # ── ROS ──────────────────────────────────────────────────────────────
    @property
    def node(self) -> "FicStudioNode":
        return self._mw.node

    def set_control_mode(self, mode: str) -> None:
        """Set the control mode (a CONTROL_MODE_NAMES name) on this extension's target robot, as
        the Studio's mode buttons do: the name on {rid}_mode_select (the hardware node) and the
        number on {rid}_control_mode (the simulation node, which reads only the number)."""
        from std_msgs.msg import String as StringMsg
        from ..mixins.robot_control_mixin import CONTROL_MODE_NAMES
        rid = self.target_robot_id
        node = self._mw.node
        if mode in CONTROL_MODE_NAMES and hasattr(node, 'publish_mode'):
            node.publish_mode(CONTROL_MODE_NAMES.index(mode) + 1, robot_id=rid)
            return
        pubs = node.pubs.get(rid)
        if pubs and 'mode_select' in pubs:
            pubs['mode_select'].publish(StringMsg(data=mode))

    def set_mode(self, mode: str) -> None:
        """Set control mode on this extension's target robot AND sync the UI selector.

        Publishes the mode string to fic_hardware once, then calls
        update_mode_ui_only() so the studio's mode buttons reflect the change
        without triggering a second publish.
        """
        self.set_control_mode(mode)
        self._mw.update_mode_ui_only(mode, self.target_robot_id)

    def control_mode(self) -> str:
        """The selected robot's control mode as the Studio names it internally (one of
        robot_control_mixin.CONTROL_MODE_NAMES, e.g. "Cartesian Planner"), or '' if unknown."""
        from ..mixins.robot_control_mixin import CONTROL_MODE_NAMES
        idx = getattr(self._mw, "_rc_selected_mode_idx", None)
        return CONTROL_MODE_NAMES[idx] if isinstance(idx, int) and 0 <= idx < len(CONTROL_MODE_NAMES) else ''

    def hpmp_enabled(self) -> bool:
        """The H-PMP toggle as the control bar shows it (on by default)."""
        t = getattr(self._mw, "_rc_hpmp_toggle", None)
        return bool(t.isChecked()) if t is not None else True

    def send_hpmp(self, enabled: Optional[bool] = None) -> None:
        """Send the H-PMP toggle (default: as the control bar shows it) to this extension's target
        robot, e.g. to a controller started after the Studio: the nodes start with H-PMP off and
        do not receive the Studio's earlier, latched message."""
        on = self.hpmp_enabled() if enabled is None else bool(enabled)
        self._mw.node.publish_hpmp_enable(on, robot_id=self.target_robot_id)

    def set_admittance_planner(self, enabled: bool) -> None:
        """Enable or disable the motion planner inside admittance modes."""
        self._mw._publish_admittance_planner(enabled, self.target_robot_id)

    def set_desired_wrench(self, wrench) -> None:
        """Set the Cartesian admittance's desired wrench [Fx, Fy, Fz, Mx, My, Mz] (N, N m)."""
        self._mw._publish_desired_wrench(wrench, self.target_robot_id)

    # ── Robot type & config (read-only) ──────────────────────────────────
    def current_robot_name(self) -> str:
        """Robot type of this extension's target. Falls back to the global
        dropdown for the first/active robot."""
        return self.robot_type_for(self.target_robot_id)

    def current_robot_config(self) -> dict:
        return self._mw.robot_configs.get(self.current_robot_name(), {})

    def joint_count(self) -> int:
        return self._mw.robots.get(self.current_robot_name(), 7)

    def robot_type_for(self, robot_id) -> str:
        """Robot type name for the given rid. Card 1 follows the global
        dropdown; cards 2–4 use their own per-card type override."""
        rid = self._normalize_rid(robot_id)
        # Card 1 uses the global dropdown.
        first_rid = self._mw.node.station_robot_id
        if rid == first_rid:
            return self._mw.robot_dropdown.currentText()
        # Cards 2-4: widget prefix is 'robot_N'.
        widget_id = self._widget_prefix(rid)
        return getattr(self._mw, '_card_robot_types', {}).get(
            widget_id, self._mw.robot_dropdown.currentText())

    @property
    def robots(self) -> dict:
        return self._mw.robots

    @property
    def robot_configs(self) -> dict:
        return self._mw.robot_configs

    @property
    def robot_dropdown(self):
        # Direct widget access — extensions sometimes need to subscribe to
        # currentTextChanged. Treat as read-only; never call setCurrentText.
        return self._mw.robot_dropdown

    # ── Multi-robot API ──────────────────────────────────────────────────
    def active_robot_ids(self) -> list[int]:
        """Numeric ids of currently active robots, in insertion order."""
        return list(self._mw.node.active_robots)

    def active_robot_rids(self) -> list[str]:
        return [f'robot{n}' for n in self.active_robot_ids()]

    @property
    def target_robot_id(self) -> str:
        """Topic-prefix rid ('robotN') of this extension's target.

        If no explicit target has been set, or the previously selected target
        has been removed, falls back to the first active robot."""
        active = self.active_robot_ids()
        if not active:
            return self._mw.node.station_robot_id
        if self._target_robot_n is None or self._target_robot_n not in active:
            return f'robot{active[0]}'
        return f'robot{self._target_robot_n}'

    @property
    def target_robot_n(self) -> int:
        return int(self.target_robot_id[len("robot"):])

    def set_target_robot(self, n) -> None:
        """Pick the target robot for this extension. Pass int N or
        'robotN' / 'robot_N'. Emits target_robot_changed."""
        rid = self._normalize_rid(n)
        try:
            self._target_robot_n = int(rid[len("robot"):])
        except (ValueError, IndexError):
            return
        # Refresh any combos this extension has open.
        for combo in list(self._target_combos):
            self._populate_target_combo(combo)
        self.target_robot_changed.emit(self.target_robot_id)

    def broadcast(self, fn: Callable[[str], Any]) -> None:
        """Run fn(rid) once per active robot. Use for global ops (e-stop,
        pause-all) where one extension action should hit every robot."""
        for rid in self.active_robot_rids():
            fn(rid)

    def broadcast_set_mode(self, mode: str) -> None:
        """Publish a mode-select message to every active robot."""
        from std_msgs.msg import String as StringMsg
        msg = StringMsg(data=mode)
        for rid in self.active_robot_rids():
            pubs = self._mw.node.pubs.get(rid)
            if pubs and 'mode_select' in pubs:
                pubs['mode_select'].publish(msg)

    def broadcast_stop(self) -> None:
        """Send 'Stop Controller' to every active robot."""
        self.broadcast_set_mode("Stop Controller")

    def make_target_robot_combo(self, parent=None) -> QComboBox:
        """Build a 'Target Robot' combo box bound to this extension's target.

        The combo lists every active robot, defaults to the current target,
        and updates target_robot_id on selection. Repopulates automatically
        when set_target_robot is called from elsewhere. Extensions that
        need to react to changes should connect to `target_robot_changed`."""
        combo = QComboBox(parent)
        self._populate_target_combo(combo)
        combo.currentIndexChanged.connect(
            lambda _i, c=combo: self._on_target_combo_changed(c))
        self._target_combos.append(combo)
        combo.destroyed.connect(
            lambda _=None, c=combo: self._target_combos.remove(c)
            if c in self._target_combos else None)
        return combo

    def _populate_target_combo(self, combo: QComboBox) -> None:
        combo.blockSignals(True)
        combo.clear()
        ids = self.active_robot_ids() or [1]
        for n in ids:
            combo.addItem(f"Robot {n}", n)
        idx = combo.findData(self.target_robot_n)
        if idx >= 0:
            combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    def notify_robots_changed(self) -> None:
        """Called by the main window when robots are added or removed."""
        for combo in list(self._target_combos):
            self._populate_target_combo(combo)
        self.robots_changed.emit()

    def _on_target_combo_changed(self, combo: QComboBox) -> None:
        n = combo.currentData()
        if isinstance(n, int):
            self._target_robot_n = n
            self.target_robot_changed.emit(self.target_robot_id)

    @staticmethod
    def _normalize_rid(robot_id) -> str:
        """Accept int N | 'robotN' | 'robot_N' → 'robotN'."""
        if isinstance(robot_id, int):
            return f'robot{robot_id}'
        s = str(robot_id)
        if s.startswith("robot_") and s[len("robot_"):].isdigit():
            return f'robot{s[len("robot_"):]}'
        return s

    @staticmethod
    def _widget_prefix(rid: str) -> str:
        """Convert topic rid 'robot1' → widget prefix 'robot_1'."""
        if rid.startswith("robot") and not rid.startswith("robot_"):
            n = rid[len("robot"):]
            if n.isdigit():
                return f"robot_{n}"
        return rid

    # ── Per-robot state ──────────────────────────────────────────────────
    @property
    def current_ee_pose(self):
        """End-effector pose of the target robot (or first active)."""
        rid = self.target_robot_id
        return self.ee_pose_for(rid)

    @property
    def current_joint_positions(self):
        """Joint positions of the target robot (or first active)."""
        rid = self.target_robot_id
        return self.joint_positions_for(rid)

    def joint_positions_for(self, robot_id) -> list:
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'joint_positions_for'):
            return self._mw.joint_positions_for(rid)
        return list(getattr(self._mw, 'current_joint_positions', None) or [])

    def ee_pose_for(self, robot_id):
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'ee_pose_for'):
            return self._mw.ee_pose_for(rid)
        return getattr(self._mw, 'current_ee_pose', None)

    # The rest of JointStateMixin's per-robot telemetry. Without these an
    # extension that wants a torque reading has to reach through
    # `main_window`, which the comment below explicitly says not to do — so
    # every extension needing torque was quietly breaking the boundary this
    # class exists to draw. Read-only, same shape as the two above: an empty
    # list (or None for the wrench) when the robot has published nothing yet,
    # never an exception.
    def joint_commanded_for(self, robot_id) -> list:
        """Commanded joint positions for one robot, in radians."""
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'joint_commanded_for'):
            return list(self._mw.joint_commanded_for(rid) or [])
        return []

    def joint_velocity_for(self, robot_id) -> list:
        """Measured joint velocities for one robot, in rad/s."""
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'joint_velocity_for'):
            return list(self._mw.joint_velocity_for(rid) or [])
        return []

    def joint_effort_for(self, robot_id) -> list:
        """Commanded joint control effort for one robot, in Nm.

        This is what the FIC controller COMMANDED (trajectory_data block
        4), not what the arm measured. For the measured torque sensor
        reading use joint_torque_for(), which reads {rid}_joint_torques.
        """
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'joint_effort_for'):
            return list(self._mw.joint_effort_for(rid) or [])
        return []

    def joint_torque_for(self, robot_id) -> list:
        """Measured joint torque for one robot, in Nm.

        The arm's real torque sensor reading, from {rid}_joint_torques.
        Do not confuse this with joint_effort_for()'s commanded effort.
        """
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'joint_torque_for'):
            return list(self._mw.joint_torque_for(rid) or [])
        return []

    def contact_estimate_for(self, robot_id):
        """Latest contact estimate dict for one robot, or None.

        See JointStateMixin.contact_estimate_for()'s docstring for the
        link_index state table (-1 uncalibrated, 0 no contact, 1..n a
        link; None means no message has arrived at all).
        """
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'contact_estimate_for'):
            return self._mw.contact_estimate_for(rid)
        return None

    def wrench_for(self, robot_id):
        """Tool wrench (Fx, Fy, Fz, Tx, Ty, Tz) for one robot, or None."""
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'wrench_for'):
            return self._mw.wrench_for(rid)
        return None

    def gripper_state_for(self, robot_id):
        """Return the measured gripper state ``(width_m, current, age_s)``.

        ``width_m`` is the MEASURED finger opening in metres (0 = closed,
        ~0.085 = fully open on a Robotiq 2F-85) -- compare it with the
        width you commanded on ``{rid}_gripper_width``: fingers that stop
        clearly short of a close command mean the gripper has closed on
        something. ``current`` is the gripper motor current as the driver
        reports it (Kinova: mA; higher while squeezing). ``age_s`` is the
        seconds since the sample arrived (~50 Hz while live), so a large
        age means the feed has stopped. None means no gripper feedback at
        all: simulation, an arm without a gripper, or an older driver.
        """
        rid = self._normalize_rid(robot_id)
        if hasattr(self._mw, 'gripper_state_for'):
            return self._mw.gripper_state_for(rid)
        return None

    def safety_limits(self):
        """Return the safety limits in force, as the Safety page holds them.

        A copy of ``{'cartesian': {'x': {'min', 'max', 'enabled'}, 'y':
        ..., 'z': ...}, 'joint_safety_deg': ..., 'velocity': ...}``, or
        None before any are loaded. Read-only: an extension that wants to
        stay inside the Cartesian zone -- rather than run into it and trip
        the controller stop it guards -- reads it here.
        """
        limits = getattr(self._mw, 'safety_limits', None)
        if not limits:
            return None
        import copy
        return copy.deepcopy(limits)

    @property
    def main_window(self):
        # Used as a parent for QMessageBox dialogs spawned from extensions so
        # they center on the studio. Treat as opaque — extensions must not
        # poke MainWindow internals through this handle.
        return self._mw

    # ── Per-robot target (sim / hardware / vr) ──────────────────────────────
    def robot_target(self, rid: str = None) -> str:
        """Return 'simulation', 'hardware', or 'vr' for the given robot."""
        if rid is None:
            rid = self.target_robot_id
        return self._mw.robot_target(rid)

    def set_robot_target(self, rid: str, target: str) -> None:
        """Set per-robot sim/hardware/vr target."""
        self._mw.set_robot_target(rid, target)

    # ── System flags (read-only) ─────────────────────────────────────────
    @property
    def controller_running(self) -> bool:
        return getattr(self._mw, "controller_running", False)

    @property
    def simulation_running(self) -> bool:
        return getattr(self._mw, "simulation_running", False)

    @property
    def fic_running(self) -> bool:
        return getattr(self._mw, "fic_running", False)

    # ── Process control (used by experiments replay and the Playground) ──
    def start_simulation(self) -> None:
        """Start the Studio's simulator for the selected robot, as its SIM button does."""
        self._mw._start_simulation()

    def start_fic_simulation(self) -> None:
        """Start the FIC controller against the simulator, as the FIC button does."""
        self._mw._start_fic_simulation()

    def stop_simulation(self) -> None:
        """Stop the Studio's simulator (and its gripper bridge)."""
        if getattr(self._mw, "simulation_running", False):
            self._mw._stop_simulation()

    def stop_fic_simulation(self) -> None:
        """Stop the FIC controller of the simulator."""
        if getattr(self._mw, "fic_running", False):
            self._mw._stop_fic_simulation()

    def set_simulation_scene(self, path: Optional[str], robot_name: Optional[str] = None) -> None:
        """Make the simulator load ``path`` (an MJCF scene) for ``robot_name`` (default: the
        selected robot) instead of the robot's own scene, from its next start; None goes back
        to the robot's scene. The scene must keep the robot's joint and actuator names."""
        name = robot_name or self.current_robot_name()
        scenes = getattr(self._mw, "_sim_scene_override", None)
        if scenes is None:
            scenes = self._mw._sim_scene_override = {}
        if path:
            scenes[name] = str(path)
        else:
            scenes.pop(name, None)

    def set_simulation_env(self, env: Optional[dict]) -> None:
        """Environment variables for the simulator and its FIC controller from their next start
        (e.g. {"FIC_HARDWARE_MODEL": "off"} for the ideal simulator); None or {} clears them."""
        self._mw._sim_env_override = {str(k): str(v) for k, v in (env or {}).items()}

    def set_simulation_start_at_home(self, on: bool, robot_name: Optional[str] = None) -> None:
        """From the simulator's next start, spawn ``robot_name`` (default: the selected robot) at
        its home (robots_config.yaml ``q_home_rad``) instead of every joint at 0; False goes back
        to the robot's own setting (``sim_start_at_home``)."""
        name = robot_name or self.current_robot_name()
        homes = getattr(self._mw, "_sim_start_home_override", None)
        if homes is None:
            homes = self._mw._sim_start_home_override = set()
        (homes.add if on else homes.discard)(name)

    def refresh_profile(self) -> None:
        """Re-pick the tuning config the Studio uses until the user picks one ('sim' or 'default',
        which follows whether the simulator runs like the real robot) and send it to the robot."""
        if hasattr(self._mw, "_refresh_rc_profile_dropdown"):
            self._mw._refresh_rc_profile_dropdown()
        if hasattr(self._mw, "_publish_rc_profile"):
            self._mw._publish_rc_profile()

    def simulation_scene(self, robot_name: Optional[str] = None) -> Optional[str]:
        """The scene set with set_simulation_scene for ``robot_name``, or None."""
        name = robot_name or self.current_robot_name()
        return (getattr(self._mw, "_sim_scene_override", None) or {}).get(name)

    def select_robot(self, robot_name: str) -> None:
        """Select ``robot_name`` in the Studio (its robot menu), as the user would."""
        dd = self._mw.robot_dropdown
        if dd.findText(robot_name) < 0:
            raise ValueError("No such robot: %s" % robot_name)
        dd.setCurrentText(robot_name)

    # ── Quick Actions (the control bar's drawer, desktop and web) ──────────
    def add_quick_action(self, key: str, label: str, run: Callable[[Optional[str]], Any],
                         choices: Optional[Callable[[], list]] = None, hint: str = '',
                         button: str = '') -> None:
        """Add a row to Quick Actions: ``label``, a drop-down of ``choices()`` (when given, read
        each time the drawer opens) and a button that calls ``run(choice)`` on the Qt thread.
        ``run`` returns a short message for the drawer's status line, or raises ValueError("why").
        Replaces an action of the same key; remove it with remove_quick_action."""
        actions = getattr(self._mw, "_quick_actions", None)
        if actions is None:
            actions = self._mw._quick_actions = {}
        actions["%s.%s" % (self._id, key)] = {"label": str(label), "run": run, "choices": choices,
                                              "hint": str(hint or ''), "button": str(button or ''),
                                              "extension": self._id}
        if hasattr(self._mw, "_cb_rebuild_quick_actions"):
            self._mw._cb_rebuild_quick_actions()

    def remove_quick_action(self, key: str) -> None:
        actions = getattr(self._mw, "_quick_actions", None) or {}
        if actions.pop("%s.%s" % (self._id, key), None) is not None and hasattr(self._mw, "_cb_rebuild_quick_actions"):
            self._mw._cb_rebuild_quick_actions()

    # ── Logging ──────────────────────────────────────────────────────────
    def log(self, level: str, msg: str, category: Optional[str] = None) -> None:
        self._mw.log_message(level, msg, category or self._id)

    # ── UI factories ─────────────────────────────────────────────────────
    def make_button(self, label: str, style: Optional[str] = None) -> QPushButton:
        return self._mw._btn(label, style)

    def make_page_title(self, text: str) -> QLabel:
        return self._mw._page_title(text)

    # ── Settings (auto-namespaced under settings.extensions.data.<id>) ──
    def get_setting(self, key: str, default: Any = None) -> Any:
        block = self._mw.settings.setdefault("extensions", {}).setdefault("data", {})
        return block.get(self._id, {}).get(key, default)

    def set_setting(self, key: str, value: Any) -> None:
        block = self._mw.settings.setdefault("extensions", {}).setdefault("data", {})
        block.setdefault(self._id, {})[key] = value
        self._mw._save_settings()

    # ── XMLManager (autotuning needs URDF read/write) ────────────────────
    def xml_manager(self):
        from fic_studio.xml_manager import XMLManager
        return XMLManager()

    # ── Identity ─────────────────────────────────────────────────────────
    @property
    def extension_id(self) -> str:
        return self._id
