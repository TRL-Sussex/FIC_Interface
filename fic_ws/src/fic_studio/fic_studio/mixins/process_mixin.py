# mixins/process_mixin.py — Process management (controller, simulation, bags, plotter, driver)

import ctypes
import os
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QPushButton, QMessageBox

from fic_studio.mixins.settings_mixin import (
    SIM_BACKEND_CPU, SIM_BACKEND_GPU, normalize_sim_backend)
from fic_studio import notices
from fic_studio.mcu_probe import discover as mcu_discover

from std_msgs.msg import Int32, String
from fic_studio.config import WS_PATH as _FIC_WS, ROBOT_LIB as _FIC_ROBOT_LIB, FIC_INTERFACE_DIR as _FIC_ROOT


# Gripper bridge — translates {rid}_gripper_width → simulation/actuators_cmds for
# the "gripper" actuator. Spawned alongside the simulator so the Robot Control
# slider drives the sim gripper end-to-end (matches hardware-flow semantics).
_GRIPPER_BRIDGE = str(_FIC_ROOT / "Extensions" / "mujoco_playground" / "gripper_bridge.py")


# ── Crash-safe child cleanup ─────────────────────────────────────────────────
# Goal: if this GUI process dies for ANY reason — clean exit, uncaught Python
# exception, SIGTERM from the OS, segfault, OOM-kill — every subprocess we
# spawned (ros2 launch, mujoco, vendor driver, ros2 bag) must die with us.
# Otherwise the robot keeps receiving the last torque command from an orphaned
# controller node, which is unsafe.
#
# Two defences, applied to every Popen call:
#   1. preexec_fn → os.setsid(): child is the leader of its own process
#      group, so os.killpg(pgid, SIG) reaches the whole tree (bash + ros2
#      launch + spawned nodes), not just the bash wrapper.
#   2. preexec_fn → prctl(PR_SET_PDEATHSIG, SIGTERM): the kernel sends SIGTERM
#      to the child the moment its parent (us) dies, including the
#      uncatchable cases (SIGKILL, segfault) where Python-level atexit /
#      excepthook never runs. We also `exec` bash → ros2 in the command
#      strings so this signal lands directly on ros2 launch, not on a bash
#      shell that wouldn't forward it.
_PR_SET_PDEATHSIG = 1
try:
    _LIBC = ctypes.CDLL("libc.so.6", use_errno=True)
except OSError:
    _LIBC = None


#: Backstop for re-asserting the control mode after launching the arm.
#: `{rid}_control_mode` is VOLATILE, so the message only reaches a subscriber
#: that already exists; contact_node takes ~1 s to load its Pinocchio and
#: MuJoCo models before subscribing.
#:
#: This used to be the whole mechanism: wait a flat 5 s and hope. It is a
#: backstop now, because the wait was most of the delay operators were seeing
#: before FIC engaged. Measured on a Kinova (9 Oct 2026), the controller came
#: up and then sat idle: +8.4 s before the robot selection reached it, and
#: another +8.4 s before the mode did -- the 5 s timer plus DDS discovery,
#: twice. The arm was home and powered the whole time, waiting on messages
#: that had been broadcast before it was listening.
_MODE_REPUBLISH_MS = 5000
#: How often to look for the controller's subscription before giving up and
#: publishing anyway. Cheap: it is a local count, not a network round trip.
_MODE_REPUBLISH_POLL_MS = 250


def _driver_log_dir():
    """Return where driver output is saved: the ROS log directory."""
    return os.environ.get('ROS_LOG_DIR') or os.path.join(
        os.path.expanduser('~'), '.ros', 'log')


def _open_driver_log(robot_name):
    """Open ``driver_<robot>_<date-time>.log`` for the driver's output.

    Line-buffered, so what the driver said last is on disk however the
    Studio ends. None if it cannot be opened: the log panel still works.
    """
    safe = ''.join(c if c.isalnum() or c in '-_' else '_'
                   for c in (robot_name or 'robot'))
    path = os.path.join(_driver_log_dir(), 'driver_%s_%s.log' % (
        safe, datetime.now().strftime('%Y%m%d-%H%M%S')))
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return open(path, 'a', buffering=1)
    except OSError:
        return None


def _child_preexec():
    """Run in the child between fork() and execve(). Must stay async-signal-
    safe — no logging, no Qt, no Python objects beyond the bare minimum."""
    os.setsid()
    if _LIBC is not None:
        try:
            _LIBC.prctl(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0)
        except Exception:
            # Non-Linux or restricted seccomp — fall through. We still have
            # setsid() and the Python-level cleanup hooks.
            pass


def _kill_session(sid, sig=signal.SIGKILL):
    """Signal every process still in session `sid`.

    Children started with _child_preexec lead their own session, so this
    reaches everything they spawned, including a process that later moved to
    another process group, and nothing else on the PC (unlike `pkill -f`, which
    also hit other users' simulators). Returns the number of processes signalled.
    """
    if not sid or sid <= 1:
        return 0
    hit = 0
    try:
        entries = os.listdir('/proc')
    except OSError:
        return 0
    for name in entries:
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            with open(f'/proc/{pid}/stat') as f:
                stat = f.read()
            # Field 6 (session id) comes after the parenthesised command name,
            # which may itself contain spaces or parentheses.
            fields = stat[stat.rindex(')') + 2:].split()
            if int(fields[3]) == sid and pid != os.getpid():
                os.kill(pid, sig)
                hit += 1
        except (OSError, ValueError, IndexError):
            continue
    return hit


# (proc-attr, running-flag-attr, human label) — single source of truth for the
# set of subprocesses this GUI owns. Consumed by _kill_all_processes() and by
# the crash hooks installed in entry_pro.py / entry_dev.py.
#
# ORDER MATTERS: _kill_all_processes() stops these one at a time, waiting on
# each. The vendor driver goes first so it leaves torque mode (and parks the
# arm in position mode) while its controller is still alive. Killing the
# controller first left the driver applying the controller's last torque,
# open loop, for as long as it took to reach the driver in this list.
_PROC_ATTRS = (
    ('driver_process',     'driver_running',     'Driver'),
    ('controller_process', 'controller_running', 'Controller'),
    ('simulation_process', 'simulation_running', 'Simulation'),
    ('fic_process',        'fic_running',        'FIC controller'),
    ('vr_relay_process',   'vr_relay_running',   'VR relay'),
    ('vr_bg_relay_process', 'vr_bg_relay_running', 'VR relay (background)'),
    ('bags_process',       'bags_running',       'Bag recorder'),
    ('plotter_process',    'plotter_running',    'Plotter'),
    ('locomotion_process', 'locomotion_running', 'Locomotion'),
    ('contact_twin_process', 'contact_twin_running', 'Contact twin'),
)


def install_crash_cleanup(window, app):
    """Wire window._kill_all_processes() into every crash exit path.

    Hooks (in order of likelihood):
      • app.aboutToQuit       → normal Qt shutdown (covers Ctrl+C via the
                                existing SIGINT→app.quit handler).
      • atexit                → interpreter shutdown, including most
                                Python-level exceptions that bubble out of main().
      • SIGTERM/SIGHUP/SIGQUIT → external `kill`, terminal hangup, or core dump
                                request.

    sys.excepthook is also replaced, but it only REPORTS — see _excepthook.

    SIGKILL and native segfaults can't be caught here — those are covered by
    PR_SET_PDEATHSIG in _child_preexec.
    """
    import atexit
    import sys

    state = {'done': False}

    def _do(reason):
        if state['done']:
            return
        state['done'] = True
        try:
            window._kill_all_processes(reason=reason)
        except Exception as exc:
            sys.stderr.write(f"[fic_studio] cleanup error during '{reason}': {exc}\n")

    app.aboutToQuit.connect(lambda: _do("Qt aboutToQuit"))
    atexit.register(lambda: _do("interpreter exit"))

    prev_excepthook = sys.excepthook

    def _excepthook(exc_type, exc, tb):
        # Report only; do NOT tear down the robot processes from here. With a
        # custom hook installed, PyQt5 routes exceptions raised in any Qt slot
        # or timer (ROS callbacks included: they run inside _spin_ros) through
        # this function and then keeps the event loop running. So it fires
        # for non-fatal GUI bugs, not only for a dying GUI. Killing here
        # turned a dashboard AttributeError into a mid-run hardware stop, and
        # _do()'s one-shot latch meant it hit only the first run after each
        # launch. An exception that is really fatal escapes main(), so atexit
        # still reaps the children; SIGKILL/segfault are covered by PDEATHSIG.
        # The hook stays installed because without one PyQt5 abort()s the
        # whole GUI on the first slot exception.
        try:
            prev_excepthook(exc_type, exc, tb)
        except Exception:
            sys.__excepthook__(exc_type, exc, tb)

    sys.excepthook = _excepthook

    def _signal_cleanup(signum, _frame):
        _do(f"signal {signum}")
        # Restore default disposition and re-raise so the process exits with
        # the conventional 128+signum status. Don't sys.exit() — that would
        # bypass any remaining native-side cleanup.
        try:
            signal.signal(signum, signal.SIG_DFL)
        except Exception:
            pass
        os.kill(os.getpid(), signum)

    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT):
        try:
            signal.signal(sig, _signal_cleanup)
        except (ValueError, OSError):
            # ValueError: not in main thread. OSError: signal not available
            # on this platform. Either way, the other hooks still cover us.
            pass


# ── Shared button style strings ──────────────────────────────────────────────
_STYLE_BASE = """
    border: none; border-radius: 5px;
    font-size: 10px; font-weight: 700;
    letter-spacing: 1px; min-height: 26px; padding: 0px 14px;
"""
# Per-category accent token for the idle state (Warmth).
_BTN_ACCENT = {
    'hw': 'accent_teal', 'sim': 'accent_blue',
    'vr': 'accent_green', 'util': 'accent_orange',
}


def sim_headless(env=None):
    """Does the simulator have to run without its viewer? Yes when this Studio has no display: offscreen
    (the headless Studio of the MCU board or a robot PC), or no X/Wayland display on Linux. The viewer would
    otherwise die at start (GLFW: DISPLAY missing) and take the simulation with it."""
    env = os.environ if env is None else env
    if env.get('QT_QPA_PLATFORM', '') in ('offscreen', 'minimal'):
        return True
    if sys.platform == 'darwin':
        return False
    return not (env.get('DISPLAY') or env.get('WAYLAND_DISPLAY'))


def _apply_btn_style(btn: QPushButton, running: bool, category: str, text: str):
    """Apply the shared slim toolbar-button style from theme tokens.

    Matches main_window_pro._tb_styles so the launch/utility toolbar buttons
    stay consistent whichever code path restyles them.
    """
    if btn is None:
        return
    from fic_studio import theme
    btn.setText(text)
    t = theme.current()
    if running:
        btn.setStyleSheet(theme.button_qss(t, 'accent', t['accent_red']))
    else:
        btn.setStyleSheet(theme.button_qss(t, 'secondary'))


class ProcessMixin:
    """Mixin for managing external processes (controller, simulation, bags, plotter, driver)."""

    # Class-level alias so external callers (entry scripts, hooks) can
    # introspect the registry without importing the module-level constant.
    _PROC_ATTRS = _PROC_ATTRS

    # ══════════════════════════════════════════════════════════════════════════
    #  CRASH-SAFE TEARDOWN
    # ══════════════════════════════════════════════════════════════════════════
    def _owned_sessions(self):
        """Session ids of every process this Studio started and that is still alive: the ones in
        _PROC_ATTRS, the gripper bridge and the VR controllers. Each leads its own session
        (_child_preexec), so its session id is its pid."""
        procs = [getattr(self, attr, None) for attr, _run, _label in self._PROC_ATTRS]
        procs.append(getattr(self, 'gripper_bridge_process', None))
        procs.extend(getattr(self, '_vr_fic_processes', None) or [])
        return [p.pid for p in procs if p is not None and getattr(p, 'pid', None) and p.poll() is None]

    def _kill_all_processes(self, reason: str = "GUI exiting") -> None:
        """Forcefully tear down every subprocess this GUI started.

        SIGTERM the whole process group (so ros2 launch + all spawned nodes
        receive the signal, not just the bash wrapper), wait briefly, then
        SIGKILL anything still alive. Safe to call repeatedly: dead procs are
        skipped. Single source of truth — invoked by closeEvent (graceful),
        atexit (interpreter shutdown) and the SIGTERM/SIGHUP/SIGQUIT
        handlers. Processes are stopped in _PROC_ATTRS order (driver first).
        The PR_SET_PDEATHSIG safety net in _child_preexec covers the cases
        this method can't run in (SIGKILL, segfault).
        """
        for attr, run_attr, label in self._PROC_ATTRS:
            proc = getattr(self, attr, None)
            if proc is None or proc.poll() is not None:
                if hasattr(self, attr):
                    setattr(self, attr, None)
                if hasattr(self, run_attr):
                    setattr(self, run_attr, False)
                continue

            try:
                pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                pgid = None

            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                except Exception:
                    pass

            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                if pgid is not None:
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    except Exception:
                        pass
                try:
                    proc.wait(timeout=1.0)
                except Exception:
                    pass
            except Exception:
                pass

            setattr(self, attr, None)
            if hasattr(self, run_attr):
                setattr(self, run_attr, False)

            try:
                if hasattr(self, 'log_message'):
                    self.log_message("INFO", f"{label} terminated ({reason})", "Cleanup")
            except Exception:
                # Logging failure must not block remaining cleanup —
                # e.g. log widget is already torn down during interpreter exit.
                pass

        if hasattr(self, '_kill_vr_fic_processes'):
            try:
                self._kill_vr_fic_processes()
            except Exception:
                pass

    # ══════════════════════════════════════════════════════════════════════════
    #  CONTROLLER
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_controller(self):
        if self.controller_running:
            self._stop_controller()
        else:
            self._start_controller()
        self._update_status()

    def _start_controller(self):
        robot_name = self.robot_dropdown.currentText()
        try:
            ws_path = _FIC_WS
            sid = self.node.station_id
            # `exec ros2 ...` lets bash replace itself, so PR_SET_PDEATHSIG
            # delivers SIGTERM to ros2 launch directly if the GUI dies.
            cmd = f"source {ws_path}/install/setup.bash && exec ros2 launch fic_control robot_control_launch.py robot_name:={robot_name} station_id:={sid}"
            env = os.environ.copy()
            mcu = getattr(self, '_mcu_info', None) or {}
            if mcu.get('address'):
                # The MCU the Studio found (Ethernet or this board's Bridge):
                # fic_control must probe the same one.
                env['FIC_MCU_ADDRESS'] = mcu['address']
            if mcu.get('use'):
                # The MCU is this robot's inner loop: no PC fic_inner_loop beside it.
                cmd += " inner_loop:=false"
            self.controller_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=_child_preexec, cwd=ws_path, env=env
            )
            if getattr(self, '_mcu_driver', False):
                QTimer.singleShot(3000, self._mcu_send_start)
            self.controller_running = True
            _apply_btn_style(self.controller_toggle_btn, True, 'hw', '■ FIC')
            self.log_message("INFO", f"Robot controller started for {robot_name} (background)", "Process")
            # Deferred: the subscriber must exist before a VOLATILE
            # message will reach it. See _republish_control_mode.
            self._mode_republish_waited_ms = 0
            QTimer.singleShot(_MODE_REPUBLISH_POLL_MS, self._mode_republish_when_ready)
        except Exception as e:
            notices.alert(self, 'error', 'Controller failed to start', str(e), 'Process')

    def _mode_republish_when_ready(self):
        """Re-assert the mode as soon as the controller is listening.

        A VOLATILE message reaches only a subscriber that already exists, so
        this waits for one rather than guessing how long the controller takes
        to start. ``get_subscription_count()`` is a local count maintained by
        the DDS discovery the controller has to complete anyway, so polling it
        costs nothing and fires the instant the controller is reachable --
        typically a few hundred ms after it finishes loading, instead of the
        flat 5 s this replaces.

        Falls through to publishing at ``_MODE_REPUBLISH_MS`` whatever happens:
        a controller that is slow to appear, or a count this Studio cannot see,
        must not leave the arm without its mode.
        """
        waited = getattr(self, '_mode_republish_waited_ms', 0) + _MODE_REPUBLISH_POLL_MS
        self._mode_republish_waited_ms = waited
        ready = False
        try:
            pub = self.node.pubs[self.node.station_robot_id]['control_mode']
            ready = pub.get_subscription_count() > 0
        except Exception:
            ready = True        # cannot tell: do not hold the arm's mode back
        if ready or waited >= _MODE_REPUBLISH_MS:
            if ready and waited < _MODE_REPUBLISH_MS:
                self.log_message('INFO', 'Controller subscribed after %d ms; '
                                 'sending the mode now' % waited, 'Process')
            self._republish_control_mode()
            return
        QTimer.singleShot(_MODE_REPUBLISH_POLL_MS, self._mode_republish_when_ready)

    def _republish_control_mode(self):
        """Re-assert the mode the UI already shows, for late subscribers.

        Publishes what the control bar is displaying, so this only ever
        confirms the mode the operator can already see -- never selects a
        new one. Without it, every node that starts with the arm (today
        contact_node) sits blocked until the operator touches the mode
        selector, because the topic is VOLATILE and fires only on change.
        """
        mode = int(getattr(self, '_rc_selected_mode_idx', 0)) + 1
        if hasattr(self, '_send_hpmp_for_mode'):
            self._send_hpmp_for_mode(mode)
        self.node.publish_mode(mode)
        self.log_message(
            'INFO', f'Re-asserted control mode {mode} for late subscribers',
            'Process')
        # The safety limits too: published when the robot is selected --
        # usually before FIC is started -- on a VOLATILE topic, so a
        # controller started after that never had its joint or Cartesian
        # limits (found 14 Sep 2026, turning the Kinova's joint limits on).
        if getattr(self, 'safety_limits', None) and \
                hasattr(self, '_publish_safety_limits'):
            self._publish_safety_limits()
            self.log_message('INFO', 'Re-sent the safety limits to the '
                             'controller', 'Process')

    def _stop_controller(self):
        try:
            if hasattr(self, 'controller_process') and self.controller_process:
                os.killpg(os.getpgid(self.controller_process.pid), signal.SIGTERM)
                self.controller_process = None
            self.controller_running = False
            _apply_btn_style(self.controller_toggle_btn, False, 'hw', '▶ FIC')
            self.log_message("INFO", "Controller stopped", "Process")
        except Exception as e:
            self.log_message("WARN", f"Error stopping controller: {e}", "Process")
            self.controller_running = False
            _apply_btn_style(self.controller_toggle_btn, False, 'hw', '▶ FIC')

    # ══════════════════════════════════════════════════════════════════════════
    #  SIMULATION
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_simulation(self):
        if self.simulation_running:
            self._stop_simulation()
        else:
            self._start_simulation()
        self._update_status()

    def _sim_model_file_arg(self, robot_name):
        """`` model_file:=<scene>`` for the launch, or '' to use its built-in map.

        An extension may set its own scene for a robot
        (``ExtensionHost.set_simulation_scene``, e.g. the MuJoCo Playground's
        objects); it is used until the extension clears it.

        The scene comes from the robot's config, straight from the source
        tree: a robot imported from a URDF is in neither the launch files'
        maps nor the install space, and this way needs no rebuild.
        """
        override = (getattr(self, '_sim_scene_override', None) or {}).get(robot_name)
        if override and os.path.isfile(override):
            # an extension's scene for this robot (ExtensionHost.set_simulation_scene)
            return ' model_file:=%s' % shlex.quote(override)
        rel = getattr(self, 'robot_configs', {}).get(robot_name, {}).get('mujoco_xml_path', '')
        if not rel:
            return ''
        path = rel if os.path.isabs(rel) else os.path.join(_FIC_WS, 'src', rel)
        return ' model_file:=%s' % shlex.quote(path) if os.path.isfile(path) else ''

    def _sim_env(self):
        """The environment of the simulator and its FIC controller: the Studio's, with what an
        extension set (ExtensionHost.set_simulation_env, e.g. FIC_HARDWARE_MODEL=off for the ideal
        simulator); None (inherit) when nothing is set."""
        extra = getattr(self, '_sim_env_override', None) or {}
        return dict(os.environ, **extra) if extra else None

    def _sim_start_pose_arg(self, robot_name):
        """Return the launch argument that spawns the arm at its home pose.

        Only for a robot whose config sets ``sim_start_at_home``, or that an
        extension asked to start at home (``_sim_start_home_override``, e.g. the
        MuJoCo Playground), and has a ``q_home_rad``; otherwise the simulator
        starts with every joint at 0.
        """
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {}) or {}
        home = cfg.get('q_home_rad') or []
        at_home = cfg.get('sim_start_at_home') or robot_name in (
            getattr(self, '_sim_start_home_override', None) or ())
        if not at_home or not home:
            return ''
        try:
            values = ','.join(repr(float(v)) for v in home)
        except (TypeError, ValueError):
            return ''
        return ' initial_joint_positions:=%s' % shlex.quote(values)

    def _start_simulation(self):
        robot_name = self.robot_dropdown.currentText()
        num_joints = self.robots[robot_name]
        backend = normalize_sim_backend(
            getattr(self, 'settings', {}).get('simulation', {}).get('backend'))
        backend_label = None
        # Per-robot override: a robot's config may name the backend it needs
        # via `sim_backend` (`mujoco_native` or `newton`). Floating-base and
        # closed-loop robots (Spot, leg_spherical_quad, KinovaGen3_CSPM) pin
        # the CPU simulator, so they still run when Newton is selected.
        # Config-driven, so user-added robots route without code changes.
        required = getattr(self, 'robot_configs', {}).get(robot_name, {}).get('sim_backend')
        if required:
            backend = (SIM_BACKEND_GPU if required in (SIM_BACKEND_GPU, 'mujoco_newton')
                       else SIM_BACKEND_CPU)
            backend_label = f'{required} (required by {robot_name})'
        # Newton lives in the optional mujoco_newton package; fall back to the
        # CPU simulator if it wasn't built.
        if backend == SIM_BACKEND_GPU and not os.path.exists(
                os.path.join(_FIC_WS, 'install', 'mujoco_newton')):
            self.log_message(
                "WARN",
                "Newton backend requested but the mujoco_newton package is not "
                "built — falling back to MuJoCo (CPU). Run colcon build to "
                "enable Newton.",
                "Process")
            backend = SIM_BACKEND_CPU
            backend_label = 'MuJoCo (CPU, fallback)'
        if backend == SIM_BACKEND_GPU:
            launch_pkg = 'mujoco_newton'
            backend_label = backend_label or 'Newton (NVIDIA Warp)'
        else:
            launch_pkg = 'mujoco_native'
            backend_label = backend_label or 'MuJoCo (CPU)'
        model_arg = self._sim_model_file_arg(robot_name) + self._sim_start_pose_arg(robot_name)
        try:
            ws_path = _FIC_WS
            cmd = (
                f"source {ws_path}/install/setup.bash && "
                f"exec ros2 launch {launch_pkg} simulation_launch.py robot_name:={robot_name} station_id:={getattr(self.node, 'station_id', 1)}"
                f"{model_arg}"
            ) + (" headless:=true" if sim_headless() else "")
            # Capture subprocess output to a per-launch log so silent failures
            # (e.g. Newton crashing on mesh-heavy MJCFs) are diagnosable instead
            # of just leaving the user with a button that toggles to running but
            # nothing visible happens.
            log_path = f"/tmp/fic_sim_{launch_pkg}_{int(time.time())}.log"
            self._sim_log_path = log_path
            self._sim_log_fh = open(log_path, "w")
            self.simulation_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=self._sim_log_fh, stderr=subprocess.STDOUT,
                preexec_fn=_child_preexec, cwd=ws_path, env=self._sim_env()
            )
            self.simulation_running = True
            self._sim_backend_pkg = launch_pkg
            _apply_btn_style(self.simulation_toggle_btn, True, 'sim', '■ SIM')
            self.log_message(
                "INFO",
                f"{backend_label} simulation started for {robot_name} "
                f"(log: {log_path})",
                "Process")

            # Gripper bridge — drives the "gripper" actuator (if the loaded MJCF
            # has one) from the same {rid}_gripper_width topic the hardware uses.
            # Harmless on robots without a gripper actuator (messages are dropped).
            try:
                rid = f"robot{getattr(self.node, 'station_id', 1)}" if hasattr(self, 'node') else "robot1"
                bridge_log = open(f"/tmp/fic_gripper_bridge_{int(time.time())}.log", "w")
                self._gripper_bridge_log_fh = bridge_log
                self.gripper_bridge_process = subprocess.Popen(
                    ["python3", "-u", _GRIPPER_BRIDGE, "--robot-id", rid],
                    stdout=bridge_log, stderr=subprocess.STDOUT,
                    preexec_fn=_child_preexec,
                )
            except Exception as e:
                self.log_message("WARN", f"Gripper bridge failed to start: {e}", "Process")

            time.sleep(0.5)
            if hasattr(self, 'node'):
                sid = getattr(self.node, 'station_id', 1)
                self.node.publish_robot(robot_name, robot_id=sid)
                self.node.publish_joints(num_joints)

            # Async health check: Newton's first launch JIT-compiles ~6 s of
            # CUDA kernels, so wait longer for it. The CPU simulator is up in
            # well under a second.
            warmup_ms = 8000 if backend == SIM_BACKEND_GPU else 2000
            launched_proc = self.simulation_process
            QTimer.singleShot(
                warmup_ms,
                lambda p=launched_proc, lbl=backend_label, b=backend, lp=log_path:
                    self._check_sim_alive(p, lbl, b, lp))
        except Exception as e:
            notices.alert(self, 'error', 'Simulation failed to start', str(e), 'Process')

    def _check_sim_alive(self, expected_proc, backend_label, backend, log_path):
        """Surface an error if the simulation subprocess died during warmup.

        Runs once via QTimer after the launch warmup window. Guards against
        race conditions where the user has stopped/restarted the sim by
        comparing the captured handle to the live one.
        """
        # Stale fire — user stopped the sim, or a new launch happened.
        if self.simulation_process is not expected_proc:
            return
        # Still alive: nothing to do.
        if expected_proc.poll() is None:
            return

        # Subprocess died. Read the tail of the log and flip the UI back.
        try:
            with open(log_path, 'r') as fh:
                log_tail = fh.read()
        except Exception:
            log_tail = "(log file unreadable)"
        # Prefer the most informative tail (Tracebacks / NotImplementedError
        # land at the bottom).
        log_tail = log_tail[-3000:] if len(log_tail) > 3000 else log_tail

        self.simulation_running = False
        self.simulation_process = None
        if hasattr(self, '_sim_log_fh') and self._sim_log_fh:
            try:
                self._sim_log_fh.close()
            except Exception:
                pass
            self._sim_log_fh = None
        _apply_btn_style(self.simulation_toggle_btn, False, 'sim', '▶ SIM')

        self.log_message(
            "ERROR",
            f"{backend_label} simulation died during startup. Full log: {log_path}",
            "Process")
        for line in log_tail.splitlines()[-12:]:
            if line.strip():
                self.log_message("ERROR", f"  | {line}", "Process")

        hint = ""
        if backend == 'newton':
            hint = (
                "\n\nNewton 1.1.0 cannot load mesh-heavy MJCFs (Franka, Kinova, "
                "IIWA, UR5e, XArm7). Switch to MuJoCo in Settings → Simulation, "
                "or pick the Pendulum robot to test Newton.")
        notices.alert(
            self, 'error', f"{backend_label} failed to start",
            f"The simulation subprocess exited during startup.\n\n"
            f"Log: {log_path}\n\nLast lines:\n{log_tail[-1200:]}" + hint, 'Process')

    def _stop_simulation(self):
        try:
            if hasattr(self, 'simulation_process') and self.simulation_process:
                # The launcher leads its own session (_child_preexec): kill its
                # group, then anything left in its session. Nothing outside it.
                sid = self.simulation_process.pid
                try:
                    os.killpg(sid, signal.SIGKILL)
                except Exception:
                    pass
                _kill_session(sid)
                self.simulation_process = None
            if getattr(self, 'gripper_bridge_process', None):
                try:
                    self.gripper_bridge_process.terminate()
                    self.gripper_bridge_process.wait(timeout=2)
                except Exception:
                    try:
                        self.gripper_bridge_process.kill()
                    except Exception:
                        pass
                _kill_session(self.gripper_bridge_process.pid)
                self.gripper_bridge_process = None
            if getattr(self, '_gripper_bridge_log_fh', None):
                try:
                    self._gripper_bridge_log_fh.close()
                except Exception:
                    pass
                self._gripper_bridge_log_fh = None
            if getattr(self, '_sim_log_fh', None):
                try:
                    self._sim_log_fh.close()
                except Exception:
                    pass
                self._sim_log_fh = None
            self.simulation_running = False
            _apply_btn_style(self.simulation_toggle_btn, False, 'sim', '▶ SIM')
            self.log_message("INFO", "MuJoCo simulation forcefully stopped", "Process")
        except Exception as e:
            self.log_message("WARN", f"Error stopping simulation: {e}", "Process")
            self.simulation_running = False
            _apply_btn_style(self.simulation_toggle_btn, False, 'sim', '▶ SIM')

    # ══════════════════════════════════════════════════════════════════════════
    #  CONTACT TWIN (MuJoCo viewer mirroring the real arm, highlighting the
    #  joint-torque-residual contact estimate)
    # ══════════════════════════════════════════════════════════════════════════
    def _start_contact_twin(self):
        """Launch the MuJoCo contact twin for the active robot.

        A no-op if the twin is already running, or the active robot has
        no mujoco_xml_path configured (no MJCF, so nothing to render).
        """
        if getattr(self, 'contact_twin_running', False):
            return
        robot_name = self.robot_dropdown.currentText()
        cfg = getattr(self, 'robot_configs', {}).get(robot_name, {})
        xml = cfg.get('mujoco_xml_path')
        if not xml:
            self.log_message(
                'WARN', f'{robot_name} has no mujoco_xml_path', 'Process')
            return
        ws_path = _FIC_WS
        # The passive viewer needs mjpython on macOS, which gives it the
        # process main thread; plain python3 is fine on Linux.
        interpreter = 'mjpython' if sys.platform == 'darwin' else 'python3'
        rid = f"robot{getattr(self.node, 'station_id', 1)}"
        # twin's --joints defaults to the Kinova Actuator1..7 names and
        # raises on any joint the MJCF lacks, so every other arm needs its
        # own names passed through or the viewer dies before it opens.
        joints = ','.join(cfg.get('joint_names') or [])
        joints_arg = f' --joints {joints}' if joints else ''
        cmd = (
            f'source {ws_path}/install/setup.bash && '
            f'exec {interpreter} -m fic_contact.twin '
            f'--xml {ws_path}/src/{xml} --robot-id {rid}{joints_arg}'
        )
        log_path = f'/tmp/fic_contact_twin_{int(time.time())}.log'
        try:
            self._twin_log_path = log_path
            self._twin_log_fh = open(log_path, 'w')
            self.contact_twin_process = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=self._twin_log_fh, stderr=subprocess.STDOUT,
                preexec_fn=_child_preexec, cwd=ws_path)
            self.contact_twin_running = True
            _apply_btn_style(
                getattr(self, 'tb_viewer_btn', None), True, 'util', '■ VIEW')
            self.log_message(
                'INFO', f'Contact twin started (log: {log_path})', 'Process')
        except Exception as e:
            self.log_message('ERROR', f'Failed to start contact twin: {e}', 'Process')
            self.contact_twin_running = False

    def _stop_contact_twin(self):
        """Stop the contact twin process, if one is running."""
        proc = getattr(self, 'contact_twin_process', None)
        if proc is not None:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
            except Exception:
                pass
        if proc is not None:
            _kill_session(proc.pid)    # anything it spawned; nothing outside it
        self.contact_twin_process = None
        self.contact_twin_running = False
        _apply_btn_style(
            getattr(self, 'tb_viewer_btn', None), False, 'util', '▶ VIEW')
        fh = getattr(self, '_twin_log_fh', None)
        if fh is not None:
            try:
                fh.close()
            except Exception:
                pass
            self._twin_log_fh = None
        self.log_message('INFO', 'Contact twin stopped', 'Process')

    # ══════════════════════════════════════════════════════════════════════════
    #  FIC CONTROLLER
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_fic_simulation(self):
        if self.fic_running:
            self._stop_fic_simulation()
        else:
            self._start_fic_simulation()
        self._update_status()

    def _start_fic_simulation(self):
        robot_name = self.robot_dropdown.currentText()
        num_joints = self.robots.get(robot_name, 7)
        try:
            ws_path = _FIC_WS
            sid = self.node.station_id
            target = self.robot_target(self.node.station_robot_id)
            if target == "vr" and self._vr_drives_station(sid):
                self.log_message("INFO", "The VR relay already runs this station's FIC controller", "Process")
                return
            source_prefix = "vr" if target == "vr" else f"robot{sid}_simulation"
            cmd = f"source {ws_path}/install/setup.bash && exec ros2 launch fic_control sim_control_launch.py robot_name:={robot_name} num_joints:={num_joints} station_id:={sid} source_prefix:={source_prefix}"
            if target == "vr":
                # The headset's scene composer names every robot's joints r{i}_, one robot or many.
                cmd += " joint_name_prefix:=r0_"
            elif getattr(self, '_sim_backend_pkg', None) == 'mujoco_newton':
                # Newton does not step the inner loop: the node feeds it from Newton's joint_states.
                cmd += " plant:=topics"
            self.fic_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=_child_preexec, cwd=ws_path, env=self._sim_env()
            )
            self.fic_running = True
            _apply_btn_style(self.fic_toggle_btn, True, 'sim', '■ FIC')
            self.log_message("INFO", f"FIC controller started for {robot_name} ({num_joints} joints)", "Process")
        except Exception as e:
            notices.alert(self, 'error', 'FIC controller failed to start', str(e), 'Process')

    def _stop_fic_simulation(self):
        try:
            if hasattr(self, 'fic_process') and self.fic_process:
                os.killpg(os.getpgid(self.fic_process.pid), signal.SIGTERM)
                self.fic_process = None
            self.fic_running = False
            _apply_btn_style(self.fic_toggle_btn, False, 'sim', '▶ FIC')
            self.log_message("INFO", "FIC controller stopped", "Process")
        except Exception as e:
            self.log_message("WARN", f"Error stopping FIC controller: {e}", "Process")
            self.fic_running = False
            _apply_btn_style(self.fic_toggle_btn, False, 'sim', '▶ FIC')

    # ══════════════════════════════════════════════════════════════════════════
    #  LOCOMOTION (gait controller for legged robots)
    # ══════════════════════════════════════════════════════════════════════════
    def _start_locomotion(self):
        """Start the gait controller for a legged robot."""
        robot_name = self.robot_dropdown.currentText()
        try:
            ws_path = _FIC_WS
            sid = self.node.station_id
            cmd = (f"source {ws_path}/install/setup.bash && "
                   f"exec ros2 launch fic_locomotion locomotion_launch.py "
                   f"robot_name:={robot_name} station_id:={sid}")
            log_path = f"/tmp/fic_locomotion_{int(time.time())}.log"
            self._locomotion_log_fh = open(log_path, "w")
            self.locomotion_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=self._locomotion_log_fh, stderr=subprocess.STDOUT,
                preexec_fn=_child_preexec, cwd=ws_path
            )
            self.locomotion_running = True
            self.log_message(
                "INFO", f"Locomotion controller started for {robot_name}",
                "Process")
        except Exception as e:
            notices.alert(self, 'error', 'Locomotion controller failed to start', str(e), 'Process')

    def _stop_locomotion(self):
        """Stop the gait controller."""
        try:
            if hasattr(self, 'locomotion_process') and self.locomotion_process:
                os.killpg(os.getpgid(self.locomotion_process.pid),
                          signal.SIGTERM)
                self.locomotion_process = None
            if getattr(self, '_locomotion_log_fh', None):
                try:
                    self._locomotion_log_fh.close()
                except Exception:
                    pass
                self._locomotion_log_fh = None
            self.locomotion_running = False
            self.log_message("INFO", "Locomotion controller stopped", "Process")
        except Exception as e:
            self.log_message(
                "WARN", f"Error stopping locomotion controller: {e}", "Process")

    # ══════════════════════════════════════════════════════════════════════════
    #  VR (relay + FIC controller with source_prefix=vr)
    # ══════════════════════════════════════════════════════════════════════════
    # The headset-app repo (VRAPK): $FIC_VRAPK, else a sibling of FIC_Interface, else ~/VRAPK.
    _VRAPK_DIRS = [d for d in (os.environ.get('FIC_VRAPK'), str(_FIC_ROOT.parent / "VRAPK"),
                               os.path.expanduser('~/VRAPK')) if d]
    _VR_RELAY_SCRIPT = next(
        (os.path.join(d, "tools", "vr_relay_node.py") for d in _VRAPK_DIRS
         if os.path.isfile(os.path.join(d, "tools", "vr_relay_node.py"))),
        os.path.join(_VRAPK_DIRS[0], "tools", "vr_relay_node.py"))

    def _vr_settings(self):
        """The Settings page's VR card (``settings['vr']``), filled with the defaults."""
        from fic_studio.mixins.settings_mixin import DEFAULT_SETTINGS
        out = dict(DEFAULT_SETTINGS['vr'])
        out.update((getattr(self, 'settings', None) or {}).get('vr') or {})
        return out

    def _vr_relay_env(self, fic_station=0):
        """The relay's environment: the Studio's own, plus the VR settings."""
        env = dict(os.environ)
        for k in ('FIC_VR_FIC_STATION', 'FIC_VR_HEADSET_IP', 'FIC_VR_KEY'):
            env.pop(k, None)
        if fic_station:
            env['FIC_VR_FIC_STATION'] = str(fic_station)
        s = self._vr_settings()
        env['FIC_VR_VIEWER'] = '1' if s.get('allow_viewers') else '0'
        env['FIC_VR_STATIONS'] = '1' if s.get('fleet_stations') else '0'
        if (s.get('headset_ip') or '').strip():
            env['FIC_VR_HEADSET_IP'] = s['headset_ip'].strip()
        from fic_studio import features
        if features.headset():
            from fic_studio.fleet import vr_pairing
            key = vr_pairing.relay_key(s)       # Settings > VR > Require a pairing key
            if key:
                env['FIC_VR_KEY'] = key
        return env

    def _spawn_vr_relay(self, env, tag=''):
        """Start tools/vr_relay_node.py; return (process, log file handle, log path)."""
        relay_cmd = (
            f"source {_FIC_WS}/install/setup.bash && "
            f"exec python3 -u {self._VR_RELAY_SCRIPT}"
        )
        log = f"/tmp/fic_vr_relay_{tag}{int(time.time())}.log"
        fh = open(log, "w")
        proc = subprocess.Popen(
            ["bash", "-c", relay_cmd],
            stdout=fh, stderr=subprocess.STDOUT,
            preexec_fn=_child_preexec, cwd=_FIC_WS, env=env
        )
        return proc, fh, log

    # The background relay (Settings > VR "Relay always on"): while the Studio is open and
    # VR is not started, headsets find this Studio, their robots join the Multi-Robot page
    # as VR stations and Watch the lab works. Starting VR swaps it for VR's own relay.
    def _apply_vr_settings(self, from_settings=False):
        if from_settings:
            self._vr_bg_failures = 0
        if getattr(self, 'vr_relay_running', False):
            return              # VR's own relay is up; it took the settings when it started
        s = self._vr_settings()
        from fic_studio import features
        want = (bool(s.get('relay_always_on')) and os.path.isfile(self._VR_RELAY_SCRIPT)
                and features.headset())     # a build without the headset station runs no background relay
        key = tuple(sorted(s.items()))
        proc = getattr(self, 'vr_bg_relay_process', None)
        running = proc is not None and proc.poll() is None
        if running and (not want or key != getattr(self, '_vr_bg_key', None)):
            self._stop_vr_background()
            running = False
        if want and not running:
            self._start_vr_background(key)

    def _start_vr_background(self, key=None):
        try:
            self.vr_bg_relay_process, self._vr_bg_log_fh, log = self._spawn_vr_relay(self._vr_relay_env(), 'bg_')
        except Exception as e:
            self.log_message("WARN", f"VR relay (background) did not start: {e}", "VR")
            return
        self.vr_bg_relay_running = True
        self._vr_bg_started = time.monotonic()
        self._vr_bg_key = key or tuple(sorted(self._vr_settings().items()))
        self.log_message("INFO", "VR relay running in the background: headsets find this Studio, "
                         f"show on Multi-Robot and can watch the lab (Settings > VR; log: {log})", "VR")

    def _vr_bg_watchdog(self):
        """Bring the background relay back when it is not running and should be: after an E-STOP
        (which stops every process of this Studio) or if it stopped by itself. Every 5 s."""
        proc = getattr(self, 'vr_bg_relay_process', None)
        if proc is not None and proc.poll() is None:
            return
        if proc is not None:
            # it exited by itself: once quickly after starting is a port in use, not worth a loop
            quick = time.monotonic() - getattr(self, '_vr_bg_started', 0.0) < 10.0
            self._vr_bg_failures = getattr(self, '_vr_bg_failures', 0) + 1 if quick else 0
            self._stop_vr_background()
            if self._vr_bg_failures >= 3:
                self.log_message("WARN", "VR relay (background) keeps stopping: is another relay using "
                                 "UDP 9095 on this PC? Not restarting it (Settings > VR to try again)", "VR")
                self._vr_bg_key = None
                return
            self.log_message("WARN", f"VR relay (background) stopped (exit {proc.returncode}): restarting", "VR")
        elif getattr(self, '_vr_bg_failures', 0) >= 3 and getattr(self, '_vr_bg_key', 1) is None:
            return                  # gave up; saving the VR settings tries again
        self._apply_vr_settings()

    def _stop_vr_background(self):
        proc = getattr(self, 'vr_bg_relay_process', None)
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
            _kill_session(proc.pid)
            self.vr_bg_relay_process = None
        fh = getattr(self, '_vr_bg_log_fh', None)
        if fh is not None:
            try:
                fh.close()
            except Exception:
                pass
            self._vr_bg_log_fh = None
        self.vr_bg_relay_running = False

    def _init_vr_robots(self):
        if not hasattr(self, '_vr_robots'):
            self._vr_robots = []
            self._vr_fic_processes = []

    def _update_vr_label(self):
        n = len(getattr(self, '_vr_robots', []))
        running = getattr(self, 'vr_relay_running', False)
        if running and n == 0:
            text = "■ VR (auto)"
        elif running:
            text = f"■ VR ({n})"
        else:
            text = "▶ VR"
        if hasattr(self, 'vr_toggle_btn'):
            self.vr_toggle_btn.setText(text)
            tip_lines = [f"RID {s}: {r} ({nj}J)" for s, r, nj in getattr(self, '_vr_robots', [])]
            self.vr_toggle_btn.setToolTip('\n'.join(tip_lines) if tip_lines else "Auto-detect from headset")

    def _toggle_vr(self):
        if getattr(self, 'vr_relay_running', False):
            self._stop_vr()
        else:
            self._start_vr()
        self._update_status()

    def _start_vr(self):
        self._init_vr_robots()
        self._vr_robots = []
        self._vr_fic_processes = []

        relay_script = self._VR_RELAY_SCRIPT

        if not os.path.isfile(relay_script):
            notices.alert(self, 'error', 'VR relay not found',
                          "Looked for tools/vr_relay_node.py in:\n" + "\n".join(self._VRAPK_DIRS)
                          + "\nSet FIC_VRAPK to the VRAPK folder.", 'Process')
            return

        self._stop_vr_background()      # VR's own relay takes the port
        try:
            # On the VR target the headset's first robot IS this station: FIC drives it, the
            # control area commands it, the HUD's mode buttons reach this station's controller.
            # Otherwise the headset's robots are VR stations only (fleet/vr_station.py).
            self._vr_fic_target = self.robot_target(self.node.station_robot_id) == 'vr'
            env = self._vr_relay_env(self.node.station_id if self._vr_fic_target else 0)
            self.vr_relay_process, self._vr_relay_log_fh, relay_log = self._spawn_vr_relay(env)
            self.vr_relay_running = True
            self._update_vr_label()

            self._vr_pending_mode = None
            self._vr_pending_scene_config = None
            from rclpy.qos import QoSProfile
            self._vr_scene_config_sub = self.node.create_subscription(
                String, '/vr/scene_config',
                self._on_vr_scene_config, QoSProfile(depth=10))

            self.log_message(
                "INFO",
                f"VR relay started (auto-detect mode, log: {relay_log})",
                "Process")
        except Exception as e:
            notices.alert(self, 'error', 'VR relay failed to start', str(e), 'Process')

    def _on_vr_scene_config(self, msg):
        self._vr_pending_scene_config = msg.data

    def _poll_vr_scene_config(self):
        data = getattr(self, '_vr_pending_scene_config', None)
        if data is None:
            return
        self._vr_pending_scene_config = None
        try:
            import json
            config = json.loads(data)
            robots = config.get('robots', [])
        except Exception as e:
            self.log_message("WARN", f"Bad scene_config JSON: {e}", "VR")
            return

        new_robots = []
        for r in robots:
            sid = r.get('station_id', 0)
            name = r.get('name', 'KinovaGen3_7DOF')
            nj = r.get('num_joints', 7)
            new_robots.append((sid, name, nj))

        current = getattr(self, '_vr_robots', [])
        if new_robots == current:
            return

        self._kill_vr_fic_processes()

        self._vr_robots = new_robots
        if not getattr(self, '_vr_fic_target', False):
            names = ', '.join(r for _, r, _ in new_robots)
            self.log_message("INFO", f"VR headset: {len(new_robots)} robot(s) [{names}], "
                             "on the fleet as VR stations (FIC drives them on the VR target)", "VR")
            return

        multi = len(new_robots) > 1
        ws_path = _FIC_WS
        me = self.node.station_id

        self._vr_fic_processes = []
        for i, (sid, robot_name, num_joints) in enumerate(new_robots):
            # Only this Studio's own station runs FIC on the PC; the headset runs FIC for its
            # other robots itself (the VR stations 1002...), commanded on their station topics.
            if sid != me:
                continue
            if getattr(self, 'fic_running', False):
                continue        # the Session page's FIC controller already drives this station
            source_prefix = f'vr{i}' if multi else 'vr'
            # The headset's scene composer names every robot's joints r{i}_, one robot or many.
            fic_cmd = (
                f"source {ws_path}/install/setup.bash && "
                f"exec ros2 launch fic_control sim_control_launch.py "
                f"robot_name:={robot_name} num_joints:={num_joints} "
                f"station_id:={sid} source_prefix:={source_prefix} joint_name_prefix:=r{i}_"
            )
            proc = subprocess.Popen(
                ["bash", "-c", fic_cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=_child_preexec, cwd=ws_path, env=self._sim_env()
            )
            self._vr_fic_processes.append(proc)

        self.vr_fic_running = True
        self._update_vr_label()

        own = [(sid, rname, nj) for sid, rname, nj in new_robots if sid == me]
        names = ', '.join(f'{r}(RID{s})' for s, r, _ in new_robots)
        self.log_message(
            "INFO",
            f"VR auto-detected {len(new_robots)} robot(s): [{names}]"
            + ("" if own else f" — none is station {me}: is the relay this Studio's?"),
            "VR")
        if not own:
            return
        rid = self.node.station_robot_id
        if hasattr(self, 'set_robot_target'):
            self.set_robot_target(rid, "vr")
        _, rname, nj = own[0]
        self.node.publish_robot(rname, robot_id=me)
        self.node.publish_joints(nj, robot_id=me)

        # The HUD's mode buttons: the relay publishes them on this station's mode topic,
        # which the controller acts on; the Studio only shows them (_poll_vr_mode).
        if getattr(self, '_vr_mode_sub', None) is not None:
            self.node.destroy_subscription(self._vr_mode_sub)
        from rclpy.qos import QoSProfile
        self._vr_mode_sub = self.node.create_subscription(
            Int32, f'{rid}_control_mode',
            self._on_vr_mode_received, QoSProfile(depth=10))

    def _vr_drives_station(self, sid):
        """True while the VR relay's FIC controllers include station ``sid``'s."""
        return (getattr(self, 'vr_relay_running', False) and getattr(self, '_vr_fic_target', False)
                and any(s == sid for s, _, _ in getattr(self, '_vr_robots', []))
                and any(p.poll() is None for p in getattr(self, '_vr_fic_processes', [])))

    def _stop_vr(self):
        self._kill_vr_fic_processes()

        proc = getattr(self, 'vr_relay_process', None)
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass
            _kill_session(proc.pid)
            self.vr_relay_process = None

        if getattr(self, '_vr_relay_log_fh', None):
            try:
                self._vr_relay_log_fh.close()
            except Exception:
                pass
            self._vr_relay_log_fh = None
        self.vr_relay_running = False
        self.vr_fic_running = False

        if hasattr(self, '_vr_mode_sub') and self._vr_mode_sub is not None:
            self.node.destroy_subscription(self._vr_mode_sub)
            self._vr_mode_sub = None
        if hasattr(self, '_vr_scene_config_sub') and self._vr_scene_config_sub is not None:
            self.node.destroy_subscription(self._vr_scene_config_sub)
            self._vr_scene_config_sub = None
        self._vr_pending_mode = None
        self._vr_pending_scene_config = None

        self._update_vr_label()

        # This station goes back to the toolbar's target (the per-robot override is VR's own).
        targets = getattr(self, '_robot_targets', None)
        if targets is not None and hasattr(self, 'node'):
            targets.pop(self.node.station_robot_id, None)
        self._vr_robots = []
        self._vr_fic_target = False
        self.log_message("INFO", "VR stopped (relay + controllers)", "Process")
        self._apply_vr_settings()       # back to the background relay, if it is on

    def _kill_vr_fic_processes(self):
        for proc in getattr(self, '_vr_fic_processes', []):
            if proc is None or proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
            _kill_session(proc.pid)
        self._vr_fic_processes = []
        self.vr_fic_running = False

    def _on_vr_mode_received(self, msg):
        self._vr_pending_mode = msg.data

    def _poll_vr_mode(self):
        mode = getattr(self, '_vr_pending_mode', None)
        if mode is None:
            return
        self._vr_pending_mode = None
        if not 1 <= mode <= 8 or mode - 1 == getattr(self, '_rc_selected_mode_idx', -1):
            return          # this Studio's own publish, or nothing new
        from fic_studio.mixins.robot_control_mixin import CONTROL_MODE_NAMES
        if hasattr(self, 'update_mode_ui_only'):
            self.update_mode_ui_only(CONTROL_MODE_NAMES[mode - 1])
        elif hasattr(self, 'mode_dropdown') and mode - 1 < self.mode_dropdown.count():
            self.mode_dropdown.setCurrentIndex(mode - 1)
        self.log_message("INFO", f"Mode {mode} ({CONTROL_MODE_NAMES[mode - 1]}) selected on the headset", "VR")

    # ══════════════════════════════════════════════════════════════════════════
    #  BAGS
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_bags(self):
        if self.bags_running:
            self._stop_bags()
        else:
            self._start_bags()
        self._update_status()

    def _start_bags(self):
        try:
            from fic_studio.config import EXPERIMENTS_ROOT
            bags_dir = os.path.join(EXPERIMENTS_ROOT, "bags")
            os.makedirs(bags_dir, exist_ok=True)
            today = datetime.now().strftime("%Y_%m_%d")
            base = f"{today}_session_"
            # Probe the filesystem for the next free name instead of parsing
            # the existing ones. rosbag2 refuses to write into a directory that
            # already exists, and a sibling whose suffix didn't parse used to
            # reset the counter to 001 and collide with a real recording.
            next_num = 1
            while os.path.exists(os.path.join(bags_dir, f"{base}{next_num:03d}")):
                next_num += 1
            session_name = f"{base}{next_num:03d}"
            self.current_bag_path = os.path.join(bags_dir, session_name)

            ws_path = _FIC_WS
            # Capture stdout/stderr to a per-run log. rosbag2 reports
            # QoS-incompatibility warnings, unwritable paths and missing
            # storage plugins on stderr; with those discarded a recording that
            # never started looks exactly like one that worked.
            log_path = f"/tmp/fic_bag_{int(time.time())}.log"
            self._bag_log_path = log_path
            self._bag_log_fh = open(log_path, "w")
            cmd = (
                f"source {shlex.quote(str(ws_path))}/install/setup.bash && "
                f"exec ros2 bag record -a -o {shlex.quote(self.current_bag_path)}"
            )
            self.bags_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=self._bag_log_fh, stderr=subprocess.STDOUT,
                preexec_fn=_child_preexec, cwd=ws_path
            )
            self.bags_running = True
            _apply_btn_style(self.bags_toggle_btn, True, 'util', '■ BAG')
            self.log_message(
                "INFO",
                f"Started bag recording: {self.current_bag_path} (log: {log_path})",
                "Process")
            # rosbag2 dies within about a second on the common failures. Confirm
            # it is still alive before letting the button keep claiming that a
            # recording is in progress.
            QTimer.singleShot(1500, self._verify_bag_started)
        except Exception as e:
            self.bags_running = False
            _apply_btn_style(self.bags_toggle_btn, False, 'util', '▶ BAG')
            notices.alert(self, 'error', 'Bag recording failed to start', str(e), 'Process')

    def _bag_log_tail(self, lines: int = 12) -> str:
        """Last few lines of the recorder's log, for error reporting."""
        path = getattr(self, '_bag_log_path', None)
        if not path or not os.path.exists(path):
            return ""
        try:
            with open(path, errors="replace") as fh:
                return "".join(fh.readlines()[-lines:]).strip()
        except OSError:
            return ""

    def _verify_bag_started(self):
        """Catch a recorder that exited immediately after launch."""
        proc = getattr(self, 'bags_process', None)
        if proc is None or not getattr(self, 'bags_running', False):
            return                      # already stopped by the user

        if proc.poll() is None:
            # Alive. The output directory should exist by now; if it doesn't,
            # rosbag2 is running but not writing where we think it is.
            path = getattr(self, 'current_bag_path', '')
            if path and not os.path.isdir(path):
                self.log_message(
                    "WARN",
                    f"Bag recorder is running but {path} has not appeared — "
                    f"check {getattr(self, '_bag_log_path', 'the recorder log')}",
                    "Process")
            return

        # Exited already — the failure this whole check exists to catch.
        tail = self._bag_log_tail()
        self.bags_running = False
        self.bags_process = None
        _apply_btn_style(self.bags_toggle_btn, False, 'util', '▶ BAG')
        detail = f"\n\n{tail}" if tail else ""
        self.log_message(
            "ERROR",
            f"Bag recording failed to start (exit code {proc.returncode}). "
            f"See {getattr(self, '_bag_log_path', 'the recorder log')}",
            "Process")
        notices.alert(
            self, 'error', "Bag recording failed",
            f"ros2 bag record exited immediately (code {proc.returncode}), so "
            f"nothing is being recorded.{detail}", 'Process')
        self._update_status()

    def _summarise_bag(self, path: str):
        """Report what actually landed in the bag after recording stops.

        A bag that recorded zero messages, or a topic that recorded zero
        messages while others recorded plenty, is the failure mode that
        otherwise goes unnoticed until the data is needed — usually a QoS
        mismatch, or the recorder having started before the nodes did.
        """
        meta = os.path.join(path, "metadata.yaml")
        if not os.path.isdir(path):
            self.log_message(
                "ERROR",
                f"Bag directory {path} was never created — nothing was recorded.",
                "Process")
            return
        if not os.path.exists(meta):
            self.log_message(
                "ERROR",
                f"{path} has no metadata.yaml — the recording did not close "
                f"cleanly and the bag may be unreadable.",
                "Process")
            return
        try:
            import yaml
            with open(meta) as fh:
                info = yaml.safe_load(fh)["rosbag2_bagfile_information"]
        except Exception as e:
            self.log_message("WARN", f"Could not read {meta}: {e}", "Process")
            return

        total = int(info.get("message_count", 0))
        entries = info.get("topics_with_message_count", []) or []
        duration_s = int(info.get("duration", {}).get("nanoseconds", 0)) / 1e9
        size_mb = int(info.get("bag_size", 0)) / 1e6

        if total == 0:
            self.log_message(
                "ERROR",
                f"Bag recorded 0 messages over {duration_s:.1f}s. Nothing was "
                f"captured — were the controller and simulator running?",
                "Process")
            notices.alert(
                self, 'warn', "Empty bag",
                f"{os.path.basename(path)} contains no messages. "
                f"Start the simulator and controller before recording.", 'Process')
            return

        empty = [t["topic_metadata"]["name"] for t in entries
                 if int(t.get("message_count", 0)) == 0]
        self.log_message(
            "INFO",
            f"Bag {os.path.basename(path)}: {total} messages on "
            f"{len(entries) - len(empty)} topics, {duration_s:.1f}s, "
            f"{size_mb:.1f} MB",
            "Process")
        if empty:
            self.log_message(
                "WARN",
                f"{len(empty)} topic(s) recorded no messages "
                f"(often a QoS mismatch or a node that started late): "
                f"{', '.join(empty[:8])}{' …' if len(empty) > 8 else ''}",
                "Process")

    def _stop_bags(self):
        path = getattr(self, 'current_bag_path', None)
        stopped_cleanly = False
        if hasattr(self, 'bags_process') and self.bags_process is not None:
            try:
                # SIGINT, not SIGTERM: rosbag2 traps it to flush and write
                # metadata.yaml. A killed recorder leaves an unreadable bag.
                os.killpg(os.getpgid(self.bags_process.pid), signal.SIGINT)
                self.bags_process.wait(timeout=10)
                stopped_cleanly = True
                self.log_message("INFO", f"Stopped bag recording: {path or 'unknown'}", "Process")
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.bags_process.pid), signal.SIGKILL)
                self.log_message(
                    "WARN",
                    "Bag recorder did not exit within 10s and was killed — "
                    "the bag may be missing its final messages.",
                    "Process")
            except Exception as e:
                self.log_message("WARN", f"Error stopping bags: {e}", "Process")
            finally:
                self.bags_process = None
        for attr in ('_bag_log_fh',):
            fh = getattr(self, attr, None)
            if fh is not None:
                try:
                    fh.close()
                except Exception:
                    pass
                setattr(self, attr, None)
        self.bags_running = False
        _apply_btn_style(self.bags_toggle_btn, False, 'util', '▶ BAG')
        if path and stopped_cleanly:
            self._summarise_bag(path)

    # ══════════════════════════════════════════════════════════════════════════
    #  PLOTTER
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_plotter(self):
        if self.plotter_running:
            self._stop_plotter()
        else:
            self._start_plotter()
        self._update_status()

    def _start_plotter(self):
        try:
            ws_path = _FIC_WS
            sid = self.node.station_id
            robot = self.robot_dropdown.currentText() if hasattr(self, 'robot_dropdown') else ''
            nj = getattr(self, 'robots', {}).get(robot, 7) or 7
            cmd = (f"source {ws_path}/install/setup.bash && "
                   f"exec ros2 run fic_control plotter_node.py --ros-args "
                   f"-p station_id:={sid} -p num_joints:={nj}")
            self.plotter_process = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=_child_preexec, cwd=ws_path
            )
            self.plotter_running = True
            _apply_btn_style(self.plotter_toggle_btn, True, 'util', '■ PLT')
            self.log_message("INFO", "Plotter started (background)", "Process")
        except Exception as e:
            notices.alert(self, 'error', 'Plotter failed to start', str(e), 'Process')

    def _stop_plotter(self):
        try:
            if hasattr(self, 'plotter_process') and self.plotter_process:
                os.killpg(os.getpgid(self.plotter_process.pid), signal.SIGTERM)
                self.plotter_process = None
            self.plotter_running = False
            _apply_btn_style(self.plotter_toggle_btn, False, 'util', '▶ PLT')
            self.log_message("INFO", "Plotter stopped", "Process")
        except Exception as e:
            self.log_message("WARN", f"Error stopping plotter: {e}", "Process")
            self.plotter_running = False
            _apply_btn_style(self.plotter_toggle_btn, False, 'util', '▶ PLT')

    # ══════════════════════════════════════════════════════════════════════════
    #  DRIVER
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_driver(self):
        if self.driver_running:
            self._stop_driver()
        else:
            self._start_driver()
        self._update_status()

    def _resolve_driver_command(self, robot_name: str) -> str:
        config = self.get_robot_config(robot_name) if hasattr(self, 'get_robot_config') else {}
        driver_cmd = config.get('driver_command', '')
        if not driver_cmd:
            return ''
        driver_cmd = driver_cmd.replace('{robot_lib}', _FIC_ROBOT_LIB)
        driver_cmd = driver_cmd.replace('{robot_name}', robot_name)
        return driver_cmd

    def _start_driver(self):
        robot_name = self.robot_dropdown.currentText()
        # Inner-loop MCU: if one answers and can drive this robot, it IS the
        # driver — launching the vendor driver too would put two masters on the
        # arm. fic_control gets "Start" once it is up (see _mcu_send_start).
        config = self.get_robot_config(robot_name) if hasattr(self, 'get_robot_config') else {}
        info = mcu_discover(config, robot_name=robot_name)
        info['robot'] = robot_name
        self._mcu_info = info
        use_mcu, detail = info['use'], info['detail']
        self._mcu_driver = use_mcu
        if use_mcu:
            self.driver_process = None
            self.driver_running = True
            _apply_btn_style(self.send_start_mode_btn, True, 'hw', '■ MCU')
            self.log_message("INFO", f"{robot_name}: inner-loop MCU found ({detail}) — "
                             f"no vendor driver; the MCU drives the arm", "Process")
            if getattr(self, 'controller_running', False):
                QTimer.singleShot(1000, self._mcu_send_start)
            return
        if info['adapter']:
            self.log_message("INFO", f"{robot_name}: {detail} — standard vendor driver", "Process")
        driver_cmd = self._resolve_driver_command(robot_name)
        if not driver_cmd:
            notices.alert(
                self, 'warn', f"No driver command for {robot_name}",
                f"In Robot Configuration, open {robot_name}'s Hardware tab and "
                f"drop its driver library there, or set the driver command "
                f"({{robot_lib}} is the robot_library directory, {{robot_name}} the robot ID).",
                'Process')
            return
        try:
            # Note: driver_cmd is user-supplied (vendor binary), so we don't
            # auto-prepend `exec` here — but setsid + killpg cleanup still
            # nukes the whole process group through the Python-level hooks.
            driver_env = os.environ.copy()
            driver_env["FIC_ROOT"] = str(_FIC_ROOT)
            self.driver_process = subprocess.Popen(
                ["bash", "-c", driver_cmd],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
                preexec_fn=_child_preexec,
                env=driver_env,
            )
            self.driver_running = True
            _apply_btn_style(self.send_start_mode_btn, True, 'hw', '■ API')
            self.log_message("INFO", f"Driver started for {robot_name} (PID: {self.driver_process.pid})", "Process")
            self._start_driver_log_pump()
        except Exception as e:
            notices.alert(self, 'error', f'Driver for {robot_name} failed to start',
                          f"{e}\n\nCommand was:\n{driver_cmd}", 'Process')

    def _mcu_send_start(self, attempt=0):
        """Tell fic_control to start on the MCU: it runs the robot's
        mcu.setup_command, then hands the arm to the MCU."""
        if not getattr(self, '_mcu_driver', False) or not getattr(self, 'controller_running', False):
            return
        pub = getattr(self.node, 'mode_select_publisher', None)
        if pub is None or pub.get_subscription_count() == 0:
            if attempt < 20:                           # fic_control still coming up
                QTimer.singleShot(500, lambda: self._mcu_send_start(attempt + 1))
            else:
                self.log_message("ERROR", "MCU start: fic_control never subscribed to mode_select", "Process")
            return
        pub.publish(String(data="Start"))
        self.log_message("INFO", "MCU start: sent Start to fic_control", "Process")

    def _stop_driver(self):
        if getattr(self, '_mcu_driver', False):
            # Nothing to kill: stopping fic_control (the controller) tells the
            # MCU to ramp out of torque mode; the arm keeps its position.
            self._mcu_driver = False
            self.driver_running = False
            self.log_message("INFO", "MCU driver released", "Process")
            return
        if hasattr(self, 'driver_process') and self.driver_process is not None:
            try:
                os.killpg(os.getpgid(self.driver_process.pid), signal.SIGTERM)
                self.driver_process.wait(timeout=3)
                self.log_message("INFO", "Driver stopped", "Process")
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.driver_process.pid), signal.SIGKILL)
                self.log_message("WARN", "Force killed driver", "Process")
            except Exception as e:
                self.log_message("WARN", f"Error stopping driver: {e}", "Process")
            finally:
                self.driver_process = None
        # Flush any remaining driver output, then stop the log pump.
        try:
            self._drain_driver_log()
        except Exception:
            pass
        if getattr(self, '_driver_log_timer', None) is not None:
            try:
                self._driver_log_timer.stop()
            except Exception:
                pass
            self._driver_log_timer = None
        self.driver_running = False
        _apply_btn_style(self.send_start_mode_btn, False, 'hw', '▶ API')

    def _start_driver_log_pump(self):
        """Stream the driver's stdout into the GUI log (thread-safe).

        The driver is a separate process, so this is the only way its prints
        reach the studio. A daemon thread reads lines and pushes them onto a
        queue; a QTimer on the GUI thread drains the queue into ``log_message``
        so driver output --- especially the ``[FAULT]`` lines --- appears in the
        log panel.
        """
        import queue as _queue
        import threading as _threading
        from PyQt5.QtCore import QTimer
        # Stop any previous pump (restart without an intervening stop).
        if getattr(self, '_driver_log_timer', None) is not None:
            try:
                self._driver_log_timer.stop()
            except Exception:
                pass
        self._driver_log_queue = _queue.Queue()
        proc = self.driver_process
        # Also to a file: the panel is gone with the Studio, and the lines
        # that say why the driver quit -- [SESSION] communication lost,
        # [FAULT] -- were then nowhere (a Gaze run on 14 Sep 2026 lost its
        # arm this way, and only the joint readings falling to zero told).
        log_file = _open_driver_log(self.robot_dropdown.currentText())
        if log_file is not None:
            self.log_message("INFO", f"Driver output is also saved to "
                             f"{log_file.name}", "Process")

        def _reader():
            last, repeats = None, 0
            try:
                for line in iter(proc.stdout.readline, ''):
                    s = line.rstrip('\n')
                    if not s.strip():             # skip per-cycle blank lines
                        continue
                    self._driver_log_queue.put(s)
                    if log_file is None:
                        continue
                    if s == last:                 # a faulted arm repeats
                        repeats += 1
                        continue
                    if repeats:
                        log_file.write(f'  (repeated x{repeats})\n')
                    repeats, last = 0, s
                    log_file.write(f'{time.strftime("%H:%M:%S")} {s}\n')
            except Exception:
                pass
            finally:
                try:
                    proc.stdout.close()
                except Exception:
                    pass
                if log_file is not None:
                    try:
                        if repeats:
                            log_file.write(f'  (repeated x{repeats})\n')
                        log_file.write(f'{time.strftime("%H:%M:%S")} '
                                       '(driver output ended)\n')
                        log_file.close()
                    except Exception:
                        pass

        self._driver_log_thread = _threading.Thread(target=_reader, daemon=True)
        self._driver_log_thread.start()

        self._driver_log_timer = QTimer(self)
        self._driver_log_timer.setInterval(200)
        self._driver_log_timer.timeout.connect(self._drain_driver_log)
        self._driver_log_timer.start()

    def _drain_driver_log(self):
        """Drain queued driver-stdout lines into the GUI log (GUI thread only).

        Consecutive identical lines are collapsed into one entry plus a
        ``repeated x N`` summary. A faulted arm can spam the same error every
        control cycle (~1 kHz); without this, the log viewer re-renders on every
        line and the UI hangs/crashes.
        """
        q = getattr(self, '_driver_log_queue', None)
        if q is None:
            return
        drained = 0
        while drained < 500:   # cap per tick to keep the UI responsive
            try:
                line = q.get_nowait()
            except Exception:
                break
            drained += 1
            if not line:
                continue
            # Collapse runs of an identical line (per-cycle fault spam).
            if line == getattr(self, '_driver_log_last', None):
                self._driver_log_repeat = getattr(self, '_driver_log_repeat', 0) + 1
                continue
            rep = getattr(self, '_driver_log_repeat', 0)
            if rep:
                self.log_message("INFO", f"  ↑ previous line repeated ×{rep}",
                                 "Driver")
                self._driver_log_repeat = 0
            self._driver_log_last = line
            level = "WARN" if "[FAULT]" in line or "[SESSION]" in line else "INFO"
            self.log_message(level, line, "Driver")

        # Auto-stop: if the driver exited on its own (fault, session loss,
        # watchdog), clean up so the user can click Start again.
        proc = getattr(self, 'driver_process', None)
        if proc is not None and proc.poll() is not None:
            rc = proc.returncode
            self.log_message(
                "WARN",
                f"Driver exited (code {rc}) — click Start to restart.",
                "Driver",
            )
            self.driver_process = None
            self.driver_running = False
            if getattr(self, '_driver_log_timer', None) is not None:
                self._driver_log_timer.stop()
                self._driver_log_timer = None
            _apply_btn_style(self.send_start_mode_btn, False, 'hw', '▶ API')

    # ══════════════════════════════════════════════════════════════════════════
    #  COMPILE
    # ══════════════════════════════════════════════════════════════════════════
    def _compile_robot_api(self, robot_name: str = None):
        """Build the robot's driver in a BuildDialog; return its exit code (None: not run)."""
        if robot_name is None:
            robot_name = self.robot_dropdown.currentText()
        config = self.get_robot_config(robot_name) if hasattr(self, 'get_robot_config') else {}
        build_dir = config.get('build_directory', '')
        build_cmd = config.get('build_command', '')
        if not build_dir or not build_cmd:
            notices.alert(
                self, 'warn', f"No build configuration for {robot_name}",
                f"In Robot Configuration, open {robot_name}'s Hardware tab and "
                f"drop its driver library there, or set its build directory and build command.",
                'Process')
            return
        build_dir = os.path.expanduser(build_dir.replace('{robot_lib}', _FIC_ROBOT_LIB).replace('{robot_name}', robot_name))
        build_cmd = build_cmd.replace('{robot_lib}', _FIC_ROBOT_LIB).replace('{robot_name}', robot_name)
        if not os.path.isdir(build_dir):
            reply = QMessageBox.question(
                self, "Build Directory Missing",
                f"Build directory does not exist:\n{build_dir}\n\nCreate it and proceed?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                try:
                    os.makedirs(build_dir, exist_ok=True)
                except Exception as e:
                    QMessageBox.critical(self, "Error", f"Could not create directory:\n{e}")
                    return
            else:
                return
        # Join multi-line build commands with &&
        lines = [l.strip() for l in build_cmd.strip().splitlines() if l.strip()]
        cmd = ' && '.join(lines)

        from fic_studio.widgets import BuildDialog
        dlg = BuildDialog(
            title=f"Compile Driver — {robot_name}",
            command=cmd,
            work_dir=build_dir,
            parent=self,
        )
        self.log_message("INFO", f"Compiling driver for {robot_name}", "Build")
        dlg.exec_()
        return dlg.exit_code


