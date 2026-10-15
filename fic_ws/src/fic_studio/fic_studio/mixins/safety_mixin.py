# mixins/safety_mixin.py — Safety zone publishing and enforcement

import math
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import QMessageBox


class SafetySignals(QObject):
    """Signals for safety zone violations."""
    limit_warning = pyqtSignal(str, str, float, float)  # axis, type, current, limit
    limit_violation = pyqtSignal(str, str, float, float)  # axis, type, current, limit


class SafetyMixin:
    """Mixin for safety zone management and enforcement."""
    
    def _init_safety(self):
        """Initialize safety system."""
        self.safety_signals = SafetySignals()
        self.safety_signals.limit_warning.connect(self._on_safety_warning)
        self.safety_signals.limit_violation.connect(self._on_safety_violation)
        
        self.safety_enabled = True
        self.safety_limits = {}  # Current robot's safety limits
        self.warning_threshold = 0.9  # Warn at 90% of limit
        self.safety_enforce = True   # violations command a graceful stop, not just warn
        self._safety_stop_until = 0.0  # debounce cooldown timestamp (monotonic)

        # Safety check timer
        self.safety_check_timer = QTimer()
        self.safety_check_timer.timeout.connect(self._check_safety_limits)
        self.safety_check_timer.start(100)  # Check at 10Hz
        
        # Setup safety publishers
        self._setup_safety_publishers()
    
    def _setup_safety_publishers(self):
        """Create per-robot safety publishers for every currently active robot.

        Stored as `safety_pubs[rid] = {'cart','joint','vel': Publisher}`.
        Limits are broadcast to every robot — a master station with N robots
        applies the same workspace bounds to all of them. Use
        `_add_robot_safety_publishers(n)` / `_remove_...` from the robot-card
        lifecycle to track runtime add/remove."""
        if not (hasattr(self, 'node') and self.node):
            return
        self.safety_pubs: dict = {}
        for n in self.node.active_robots:
            self._add_robot_safety_publishers(n)
        self.node.get_logger().info("Safety publishers initialized")

    def _add_robot_safety_publishers(self, n: int):
        from std_msgs.msg import Float64MultiArray
        rid = f'robot{n}'
        if not hasattr(self, 'safety_pubs'):
            self.safety_pubs = {}
        if rid in self.safety_pubs:
            return
        try:
            self.safety_pubs[rid] = {
                'cart': self.node.create_publisher(
                    Float64MultiArray, f'{rid}_safety/cartesian_limits', 10),
                'joint': self.node.create_publisher(
                    Float64MultiArray, f'{rid}_safety/joint_limits', 10),
                'vel': self.node.create_publisher(
                    Float64MultiArray, f'{rid}_safety/velocity_limits', 10),
            }
        except Exception as e:
            print(f"Could not setup safety publishers for {rid}: {e}")

    def _remove_robot_safety_publishers(self, n: int):
        rid = f'robot{n}'
        pubs = getattr(self, 'safety_pubs', {}).pop(rid, None)
        if not pubs:
            return
        for pub in pubs.values():
            try:
                self.node.destroy_publisher(pub)
            except Exception:
                pass
    
    def load_robot_safety_limits(self, robot_name: str):
        """Load safety limits for a specific robot."""
        if not hasattr(self, 'robot_configs'):
            return
        
        config = self.robot_configs.get(robot_name, {})
        self.safety_limits = config.get('safety_zones', self._get_default_safety_limits())
        # Simulated like the real robot (hardware_model): the robot's velocity damper in
        # simulation too.
        self._safety_like_hardware = bool(
            (config.get('hardware_model') or {}).get('enabled'))

        # Publish the limits
        self._publish_safety_limits()
        if getattr(self, '_safety_panel', None) is not None:
            n = getattr(self, 'robots', {}).get(robot_name, 7) or 7
            self._safety_panel.set_joint_count(n, joint_limits_deg=config.get('joint_limits_deg'))
            self._safety_panel.from_config(self.safety_limits)

        if hasattr(self, 'log_message'):
            self.log_message("INFO", f"Loaded safety limits for {robot_name}", "Safety")
    
    def _get_default_safety_limits(self) -> dict:
        """Return default safety limits."""
        return {
            'cartesian': {
                'x': {'min': -1.0, 'max': 1.0, 'enabled': True},
                'y': {'min': -1.0, 'max': 1.0, 'enabled': True},
                'z': {'min': 0.0, 'max': 1.5, 'enabled': True}
            },
            'joint_safety_deg': {
                'lower': [],
                'upper': [],
                'enabled': True
            },
            'velocity': {
                'max_joint_vel': 1.0,
                'max_cart_vel': 0.5,
                'enabled': True
            }
        }
    
    def _publish_safety_limits(self):
        """Broadcast current safety limits to every active robot."""
        if not self.safety_limits:
            return

        try:
            from std_msgs.msg import Float64MultiArray

            # Build the three messages once; same content goes to every robot.
            cart = self.safety_limits.get('cartesian', {})
            x = cart.get('x', {})
            y = cart.get('y', {})
            z = cart.get('z', {})
            cart_msg = Float64MultiArray()
            cart_msg.data = [
                x.get('min', -1.0), x.get('max', 1.0), float(x.get('enabled', True)),
                y.get('min', -1.0), y.get('max', 1.0), float(y.get('enabled', True)),
                z.get('min', 0.0), z.get('max', 1.5), float(z.get('enabled', True))
            ]

            joints = self.safety_limits.get('joint_safety_deg', {})
            lower = joints.get('lower', [])
            upper = joints.get('upper', [])
            joint_data = []
            for lo, hi in zip(lower, upper):
                joint_data.extend([math.radians(lo), math.radians(hi)])
            joint_data.append(float(joints.get('enabled', True)))
            joint_msg = Float64MultiArray()
            joint_msg.data = joint_data

            vel = self.safety_limits.get('velocity', {})

            def vel_msg_for(rid):
                # A robot driven in simulation gets sim_max_joint_vel when its
                # config has one (a weaker velocity damper than on hardware),
                # unless it is simulated like the real robot (hardware_model).
                joint_vel = vel.get('max_joint_vel', 1.0)
                target = (self.robot_target(rid) if hasattr(self, 'robot_target')
                          else getattr(self, 'control_target', 'simulation'))
                if (target == 'simulation' and vel.get('sim_max_joint_vel')
                        and not getattr(self, '_safety_like_hardware', False)):
                    joint_vel = vel['sim_max_joint_vel']
                msg = Float64MultiArray()
                msg.data = [float(joint_vel), float(vel.get('max_cart_vel', 0.5)),
                            float(vel.get('enabled', True))]
                return msg

            for rid, pubs in getattr(self, 'safety_pubs', {}).items():
                pubs['cart'].publish(cart_msg)
                pubs['joint'].publish(joint_msg)
                pubs['vel'].publish(vel_msg_for(rid))

        except Exception as e:
            if hasattr(self, 'log_message'):
                self.log_message("ERROR", f"Failed to publish safety limits: {e}", "Safety")
    
    def _check_safety_limits(self):
        """Check current robot state against safety limits."""
        if (getattr(self, '_safety_page', None) is not None
                and getattr(self, 'main_stack', None) is not None
                and self.main_stack.currentWidget() is self._safety_page):
            self._refresh_safety_panel()
        if not self.safety_enabled or not self.safety_limits:
            return
        
        # Check Cartesian limits
        cart_limits = self.safety_limits.get('cartesian', {})
        if hasattr(self, 'current_ee_pose') and self.current_ee_pose:
            pose = self.current_ee_pose
            if hasattr(pose, 'position'):
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                
                self._check_axis_limit('X', x, cart_limits.get('x', {}))
                self._check_axis_limit('Y', y, cart_limits.get('y', {}))
                self._check_axis_limit('Z', z, cart_limits.get('z', {}))
        
        # Check joint limits
        joint_limits = self.safety_limits.get('joint_safety_deg', {})
        if joint_limits.get('enabled', False) and hasattr(self, 'current_joint_positions'):
            lower = joint_limits.get('lower', [])
            upper = joint_limits.get('upper', [])
            
            for i, pos_rad in enumerate(self.current_joint_positions):
                if i < len(lower) and i < len(upper):
                    pos_deg = math.degrees(pos_rad)
                    if pos_deg < lower[i]:
                        self.safety_signals.limit_violation.emit(
                            f'Joint {i+1}', 'lower', pos_deg, lower[i]
                        )
                    elif pos_deg > upper[i]:
                        self.safety_signals.limit_violation.emit(
                            f'Joint {i+1}', 'upper', pos_deg, upper[i]
                        )
                    # Warning check
                    elif pos_deg < lower[i] + (upper[i] - lower[i]) * (1 - self.warning_threshold):
                        self.safety_signals.limit_warning.emit(
                            f'Joint {i+1}', 'lower', pos_deg, lower[i]
                        )
                    elif pos_deg > upper[i] - (upper[i] - lower[i]) * (1 - self.warning_threshold):
                        self.safety_signals.limit_warning.emit(
                            f'Joint {i+1}', 'upper', pos_deg, upper[i]
                        )
    
    def _check_axis_limit(self, axis: str, value: float, limits: dict):
        """Check a single axis against its limits."""
        if not limits.get('enabled', False):
            return
        
        min_val = limits.get('min', -float('inf'))
        max_val = limits.get('max', float('inf'))
        range_val = max_val - min_val
        
        if value < min_val:
            self.safety_signals.limit_violation.emit(axis, 'min', value, min_val)
        elif value > max_val:
            self.safety_signals.limit_violation.emit(axis, 'max', value, max_val)
        elif value < min_val + range_val * (1 - self.warning_threshold):
            self.safety_signals.limit_warning.emit(axis, 'min', value, min_val)
        elif value > max_val - range_val * (1 - self.warning_threshold):
            self.safety_signals.limit_warning.emit(axis, 'max', value, max_val)
    
    def _on_safety_warning(self, axis: str, limit_type: str, current: float, limit: float):
        """Handle safety warning (approaching limit)."""
        # Could update UI to show warning color
        pass
    
    def _on_safety_violation(self, axis: str, limit_type: str, current: float, limit: float):
        """Handle safety violation (exceeded limit).

        The C++ control loop enforces Cartesian boundaries via a virtual wall
        (restoring J^T force) and joint-limit clamping, so the robot is bounded
        at the limit rather than faulted.  The GUI only logs a warning here.
        """
        if hasattr(self, 'log_message'):
            self.log_message("WARN",
                f"Safety limit exceeded: {axis} {limit_type} = {current:.3f} (limit: {limit:.3f})",
                "Safety"
            )

    def _enforce_safety_stop(self, reason: str):
        """Command a debounced graceful controller stop on a zone violation.

        A runtime backstop to the C++ target clamping: if the robot is actually
        past a safety limit, ramp the controller down to a safe hold. Debounced
        (one stop per episode) so the 10 Hz check cannot spam stop commands, and
        skipped when ``safety_enforce`` is False.
        """
        if not getattr(self, 'safety_enforce', True):
            return
        import time
        now = time.monotonic()
        if now < getattr(self, '_safety_stop_until', 0.0):
            return
        self._safety_stop_until = now + 3.0
        if hasattr(self, 'log_message'):
            self.log_message(
                "ERROR", f"SAFETY VIOLATION \u2014 stopping robot: {reason}", "Safety")
        if hasattr(self, 'broadcast_stop'):
            self.broadcast_stop()
    
    def toggle_safety(self, enabled: bool):
        """Enable or disable safety checking."""
        self.safety_enabled = enabled
        if hasattr(self, 'log_message'):
            status = "enabled" if enabled else "disabled"
            self.log_message("INFO", f"Safety system {status}", "Safety")
    
    def get_safety_status(self) -> dict:
        """Get current safety status."""
        return {
            'enabled': self.safety_enabled,
            'limits': self.safety_limits,
            'warnings': [],  # Could track active warnings
            'violations': []  # Could track active violations
        }

    # ── Live Safety Zones page (joint + Cartesian) ───────────────────────────
    def _safety_joint_count(self):
        """Joint count for the currently selected robot (default 7)."""
        try:
            name = self.robot_dropdown.currentText()
        except Exception:
            return 7
        return getattr(self, 'robots', {}).get(name, 7) or 7

    def _safety_joint_limits_deg(self):
        """Mechanical ``joint_limits_deg`` for the currently selected robot."""
        try:
            name = self.robot_dropdown.currentText()
        except Exception:
            return None
        return getattr(self, 'robot_configs', {}).get(name, {}).get('joint_limits_deg')

    def _build_safety_page(self):
        """Build the live Safety Zones page (joint + Cartesian config + viz)."""
        from fic_studio.components.safety_widgets import SafetyZonePanel
        from PyQt5.QtWidgets import QScrollArea, QWidget, QVBoxLayout
        page = QScrollArea()
        page.setWidgetResizable(True)
        page.setFrameShape(QScrollArea.NoFrame)
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(0, 0, 0, 0)
        self._safety_panel = SafetyZonePanel(
            joint_count=self._safety_joint_count(),
            joint_limits_deg=self._safety_joint_limits_deg())
        self._safety_panel.changed.connect(self._apply_safety_panel)
        lay.addWidget(self._safety_panel)
        page.setWidget(inner)
        self._safety_page = page
        if getattr(self, 'safety_limits', None):
            self._safety_panel.from_config(self.safety_limits)
        return page

    def show_safety_panel(self):
        """Navigate to the Safety Zones page and refresh its live readout."""
        if hasattr(self, '_safety_page'):
            self.main_stack.setCurrentWidget(self._safety_page)
        if hasattr(self, '_safety_btn') and hasattr(self, '_set_active_sidebar_btn'):
            self._set_active_sidebar_btn(self._safety_btn)
        self._refresh_safety_panel()

    def _apply_safety_panel(self):
        """Push edited zone config to the controllers (store + publish)."""
        panel = getattr(self, '_safety_panel', None)
        if panel is None:
            return
        self.safety_limits = panel.to_config()
        self._publish_safety_limits()

    def _refresh_safety_panel(self):
        """Feed live telemetry + active-mode emphasis to the safety panel."""
        panel = getattr(self, '_safety_panel', None)
        if panel is None:
            return
        # A joint-count (or robot) change rebuilds the rows at defaults;
        # reload the real limits so a refresh tick never leaves/publishes
        # stale defaults.
        if panel.set_joint_count(self._safety_joint_count(),
                                 joint_limits_deg=self._safety_joint_limits_deg()):
            panel.from_config(getattr(self, 'safety_limits', None) or {})
        from fic_studio.components.safety_widgets import SafetyZonePanel
        ee = None
        pose = getattr(self, 'current_ee_pose', None)
        if pose is not None and hasattr(pose, 'position'):
            ee = (pose.position.x, pose.position.y, pose.position.z)
        jpos = getattr(self, 'current_joint_positions', None)
        jdeg = SafetyZonePanel.joint_pos_deg_from_rad(jpos) if jpos else None
        panel.set_live(ee_pos=ee, joint_pos_deg=jdeg)
        try:
            mode = self._get_current_mode()
            from fic_studio.mixins.tuning_mixin import CONTROL_MODES
            is_joint = CONTROL_MODES.get(mode, CONTROL_MODES[1])['type'] == 'joint'
            panel.set_active_mode(is_joint)
        except Exception:
            pass