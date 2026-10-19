# mixins/safety_mixin.py — Safety zone publishing and enforcement

import math
from PyQt5.QtCore import QTimer, pyqtSignal, QObject
from PyQt5.QtWidgets import QMessageBox

#: How far past a limit a robot must be before "Stop on violation" stops it. Closer than
#: this the controller holds it at the limit by itself (the joint modes' virtual wall is
#: 2000 N/m: 60 N at 3 cm).
STOP_PAST_M = 0.03
STOP_PAST_DEG = 2.0
#: Closer than this to a Cartesian limit or a safety zone, the robot is touching it: a warning in Notices.
TOUCH_M = 0.005


class SafetySignals(QObject):
    """Signals for safety zone violations."""
    limit_warning = pyqtSignal(str, str, float, float)  # axis, type, current, limit
    limit_violation = pyqtSignal(str, str, float, float)  # axis, type, current, limit


class SafetyMixin:
    """Mixin for safety zone management and enforcement."""

    # ── E-STOP latch ─────────────────────────────────────────────────────
    # An E-STOP stays in force until someone resets it: the web UI shows a red
    # banner with when and by whom, and refuses Start, processes, teleop and
    # motion until Reset. Resetting starts nothing.
    def _estop_latch_set(self, by=''):
        import time
        from fic_studio import audit
        by = by or 'Studio window'
        self._estop_latch = {'at': time.time(), 'by': by}
        audit.record('estop', by, 'E-STOP', target=getattr(self, 'control_target', ''),
                     station=getattr(getattr(self, 'node', None), 'station_id', None))
        from fic_studio import notices
        notices.alert(self, 'error', 'E-STOP pressed by %s' % by,
                      'The controllers were told to ramp down and hold, and this Studio\'s processes stopped. '
                      'Start, motion and teleoperation are refused until Reset.', 'Safety', window_box=False)

    def _estop_latch_reset(self, by=''):
        from fic_studio import audit
        latch = getattr(self, '_estop_latch', None)
        if latch is None:
            return False
        self._estop_latch = None
        audit.record('estop_reset', by or 'Studio window', 'E-STOP of %s reset' % latch.get('by', '?'))
        from fic_studio import notices
        notices.alert(self, 'info', 'E-STOP reset by %s' % (by or 'Studio window'),
                      'Nothing was started; start the controllers again to move.', 'Safety', window_box=False)
        return True

    def estop_latch_state(self):
        """{'latched': bool, 'at': epoch s, 'by': str} for the web UI."""
        latch = getattr(self, '_estop_latch', None)
        if latch is None:
            return {'latched': False}
        return {'latched': True, 'at': latch['at'], 'by': latch['by']}

    
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
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            # The zones are latched: a controller started later gets them on connecting.
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.safety_pubs[rid] = {
                'cart': self.node.create_publisher(
                    Float64MultiArray, f'{rid}_safety/cartesian_limits', 10),
                'joint': self.node.create_publisher(
                    Float64MultiArray, f'{rid}_safety/joint_limits', 10),
                'vel': self.node.create_publisher(
                    Float64MultiArray, f'{rid}_safety/velocity_limits', 10),
                'zones': self.node.create_publisher(
                    Float64MultiArray, f'{rid}_safety/zones', latched),
            }
            if not hasattr(self, 'safety_subs'):
                self.safety_subs = {}
            self.safety_subs[rid] = self.node.create_subscription(
                Float64MultiArray, f'{rid}_safety/zone_state',
                lambda msg, rid=rid: self._on_zone_state(rid, msg), 10)
        except Exception as e:
            print(f"Could not setup safety publishers for {rid}: {e}")

    def _remove_robot_safety_publishers(self, n: int):
        rid = f'robot{n}'
        sub = getattr(self, 'safety_subs', {}).pop(rid, None)
        if sub is not None:
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass
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
        from fic_studio.safety_zones import DEFAULT_BODY
        return {
            'zones': [],
            'body': dict(DEFAULT_BODY),
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
                # 4th value: the joint target speed of joint impedance (rad/s, 0: off), when the robot
                # has one; without it the controller keeps its own (robots_config.yaml on the arm).
                if vel.get('joint_target_speed') is not None:
                    msg.data.append(float(vel['joint_target_speed']))
                return msg

            from fic_studio.safety_zones import encode
            zones_msg = Float64MultiArray()
            zones_msg.data = encode(self.safety_limits.get('zones') or [], self.safety_limits.get('body'))

            for rid, pubs in getattr(self, 'safety_pubs', {}).items():
                pubs['cart'].publish(cart_msg)
                pubs['joint'].publish(joint_msg)
                pubs['vel'].publish(vel_msg_for(rid))
                if 'zones' in pubs:
                    pubs['zones'].publish(zones_msg)

        except Exception as e:
            if hasattr(self, 'log_message'):
                self.log_message("ERROR", f"Failed to publish safety limits: {e}", "Safety")
    
    def _on_zone_state(self, rid, msg):
        """robotN_safety/zone_state: the arm's worst clearance against the safety zones (from its controller)."""
        import time
        if not hasattr(self, '_zone_state_by_robot'):
            self._zone_state_by_robot = {}
        self._zone_state_by_robot[rid] = (list(msg.data), time.monotonic())

    def zone_state_for(self, rid):
        """Where robot ``rid`` is against the safety zones (fic_studio.safety_zones.zone_state), or None."""
        import time
        from fic_studio.safety_zones import zone_state
        data, stamp = (getattr(self, '_zone_state_by_robot', None) or {}).get(rid, (None, None))
        return zone_state(data, (self.safety_limits or {}).get('zones') or [], stamp, time.monotonic())

    def _warn_safety_contacts(self):
        """A warning in Notices when a robot touches a safety limit (within TOUCH_M) or goes past it.

        The box workspace is checked at the tool point, the safety zones along the arm (their clearance comes
        from the robot's controller). Each robot and limit warns once as it touches and once more if it then
        goes past; it warns again only after it has been clear of it by 2 cm."""
        from fic_studio import notices
        lim = self.safety_limits or {}
        armed = getattr(self, '_safety_contact', None)
        if armed is None:
            armed = self._safety_contact = {}
        for rid, pose, _joints in self._safety_robots():
            seen = []      # (key, distance from the limit in m, negative past it, what)
            if pose is not None and hasattr(pose, 'position'):
                for axis in ('x', 'y', 'z'):
                    zone = (lim.get('cartesian') or {}).get(axis) or {}
                    if not zone.get('enabled', False):
                        continue
                    v = getattr(pose.position, axis)
                    for side, d in (('min', v - zone.get('min', -math.inf)), ('max', zone.get('max', math.inf) - v)):
                        seen.append(((axis, side), d, 'the workspace %s %s (%.3f m)' % (axis.upper(), side, zone[side])))
            zs = self.zone_state_for(rid) if hasattr(self, 'zone_state_for') else None
            if zs is not None:
                seen.append((('zone', zs['zone']), zs['clearance'],
                             "safety zone %d '%s'" % (zs['zone'] + 1, zs['name'])))
            for key, d, what in seen:
                k = (rid,) + key
                level = 2 if d < 0 else 1 if d < TOUCH_M else 0
                if level == 0:
                    if d > 0.02:
                        armed.pop(k, None)
                    continue
                if level <= armed.get(k, 0):
                    continue
                armed[k] = level
                who = rid or 'The robot'
                if level == 2:
                    notices.alert(self, 'warn', '%s is past %s' % (who, what),
                                  '%.0f mm past it. The controller pushes it back; past %.0f mm, Stop on violation '
                                  'stops it.' % (-1000 * d, 1000 * STOP_PAST_M), 'Safety', passing=True)
                else:
                    notices.alert(self, 'warn', '%s is touching %s' % (who, what),
                                  'Within %.0f mm of it.' % (1000 * TOUCH_M), 'Safety', passing=True)

    def _check_safety_limits(self):
        """Check current robot state against safety limits."""
        if (getattr(self, '_safety_page', None) is not None
                and getattr(self, 'main_stack', None) is not None
                and self.main_stack.currentWidget() is self._safety_page):
            self._refresh_safety_panel()
        if self.safety_limits:
            self._enforce_safety_limits()
        if not self.safety_enabled or not self.safety_limits:
            return
        self._warn_safety_contacts()
        
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
        at the limit rather than faulted.  This only logs; a robot clearly past
        a limit is stopped by ``_enforce_safety_limits``.
        """
        if hasattr(self, 'log_message'):
            self.log_message("WARN",
                f"Safety limit exceeded: {axis} {limit_type} = {current:.3f} (limit: {limit:.3f})",
                "Safety"
            )

    def _safety_robots(self):
        """(rid, EE pose, joint positions in rad) for every robot the Studio has state for."""
        poses = getattr(self, 'current_ee_pose_by_robot', None) or {}
        joints = getattr(self, 'current_joint_positions_by_robot', None) or {}
        rids = sorted(set(poses) | set(joints))
        if rids:
            return [(r, poses.get(r), list(joints.get(r) or [])) for r in rids]
        rid = getattr(getattr(self, 'node', None), 'station_robot_id', None)
        return [(rid, getattr(self, 'current_ee_pose', None),
                 list(getattr(self, 'current_joint_positions', None) or []))]

    def _safety_past(self, pose, joints_rad, rid=None):
        """Every enabled limit the robot is past: (what, how far past, stop distance).

        Distances are metres for the Cartesian zones and degrees for the joints. The safety zones are
        checked along the arm by the robot's controller, which reports how far past the worst point is."""
        out = []
        lim = self.safety_limits or {}
        zs = self.zone_state_for(rid) if rid is not None and hasattr(self, 'zone_state_for') else None
        if zs is not None and zs['clearance'] < 0:
            out.append(("safety zone %d '%s'" % (zs['zone'] + 1, zs['name']), -zs['clearance'], STOP_PAST_M))
        if pose is not None and hasattr(pose, 'position'):
            for axis in ('x', 'y', 'z'):
                zone = (lim.get('cartesian') or {}).get(axis) or {}
                if not zone.get('enabled', False):
                    continue
                v = getattr(pose.position, axis)
                if v < zone.get('min', -math.inf):
                    out.append(('%s min %.3f m' % (axis.upper(), zone['min']), zone['min'] - v, STOP_PAST_M))
                elif v > zone.get('max', math.inf):
                    out.append(('%s max %.3f m' % (axis.upper(), zone['max']), v - zone['max'], STOP_PAST_M))
        joint = lim.get('joint_safety_deg') or {}
        if joint.get('enabled', False):
            lower, upper = joint.get('lower') or [], joint.get('upper') or []
            for i, q in enumerate(joints_rad or []):
                if i >= len(lower) or i >= len(upper) or q is None:
                    continue
                d = math.degrees(q)
                if d < lower[i]:
                    out.append(('Joint %d lower %.1f deg' % (i + 1, lower[i]), lower[i] - d, STOP_PAST_DEG))
                elif d > upper[i]:
                    out.append(('Joint %d upper %.1f deg' % (i + 1, upper[i]), d - upper[i], STOP_PAST_DEG))
        return out

    def _enforce_safety_limits(self):
        """Stop a robot that is clearly past a limit: once per episode.

        Inside STOP_PAST_M / STOP_PAST_DEG the controller holds the arm at the limit by
        itself (target clamping, and a 2000 N/m virtual wall in the joint modes), so a
        robot pressed against a limit is not stopped. Past that, something is pushing
        the arm through the limit and the robot is given the graceful stop. It is not
        stopped again until it has come back within half that distance, so the operator
        can restart the controller and move the arm back in.
        """
        if not getattr(self, 'safety_enforce', True):
            return
        latched = getattr(self, '_safety_stop_latched', None)
        if latched is None:
            latched = self._safety_stop_latched = set()
        for rid, pose, joints in self._safety_robots():
            past = self._safety_past(pose, joints, rid)
            worst = max(past, key=lambda p: p[1] / p[2], default=None)
            if worst is None or worst[1] < 0.5 * worst[2]:
                latched.discard(rid)
                continue
            if worst[1] <= worst[2] or rid in latched:
                continue
            unit = 'm' if worst[2] == STOP_PAST_M else 'deg'
            latched.add(rid)
            self._enforce_safety_stop('%s: %.3g %s past %s' % (rid or 'robot', worst[1], unit, worst[0]), rid)

    def _enforce_safety_stop(self, reason: str, rid=None):
        """Command a debounced graceful controller stop on a zone violation.

        A runtime backstop to the C++ target clamping: if the robot is actually
        past a safety limit, ramp the controller down to a safe hold. The stop is
        E-STOP's first step without its process kill: 'Stop Controller' to the
        robot (all robots when ``rid`` is None), then its joint target set to
        where it is, so a restart does not drive it back to an old target.
        Debounced per robot so the 10 Hz check cannot spam stop commands, and
        skipped when ``safety_enforce`` is False.
        """
        if not getattr(self, 'safety_enforce', True):
            return
        import time
        now = time.monotonic()
        if rid is None or not hasattr(self, 'stop_for'):
            if now < getattr(self, '_safety_stop_until', 0.0):
                return
            self._safety_stop_until = now + 3.0
        else:
            until = getattr(self, '_safety_stop_until_by_robot', None)
            if until is None:
                until = self._safety_stop_until_by_robot = {}
            if now < until.get(rid, 0.0):
                return
            until[rid] = now + 3.0
        if hasattr(self, 'log_message'):
            self.log_message(
                "ERROR", f"SAFETY VIOLATION \u2014 stopping robot: {reason}", "Safety")
        if rid is None or not hasattr(self, 'stop_for'):
            if hasattr(self, 'broadcast_stop'):
                self.broadcast_stop()
            return
        try:
            self.stop_for(rid)
        except Exception as e:
            if hasattr(self, 'log_message'):
                self.log_message("ERROR", f"Safety stop failed: {e}", "Safety")
            return
        pos = (getattr(self, 'current_joint_positions_by_robot', None) or {}).get(rid)
        pub = (getattr(getattr(self, 'node', None), 'real_joint_publishers', None) or {}).get(rid)
        if pos and pub is not None:
            from std_msgs.msg import Float32MultiArray
            pub.publish(Float32MultiArray(data=[float(j) for j in pos]))

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
        new = panel.to_config()
        # the panel has no widgets for the whole-arm zones: keep them
        for key in ('zones', 'body'):
            if key in (self.safety_limits or {}) and key not in new:
                new[key] = self.safety_limits[key]
        self.safety_limits = new
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