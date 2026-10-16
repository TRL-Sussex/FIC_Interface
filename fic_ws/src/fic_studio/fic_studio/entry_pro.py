#!/usr/bin/env python3
# entry_pro.py — Entry point for FIC Control Studio GUI

import sys
import os
import signal

SETTINGS_PATH = os.path.expanduser('~/.robot_selector_gui_settings.yaml')


def _load_settings() -> dict:
    """Load saved settings (returns empty dict on failure). yaml is imported
    lazily so the splash can paint before this 60ms import lands."""
    if not os.path.exists(SETTINGS_PATH):
        return {}
    import yaml
    try:
        with open(SETTINGS_PATH) as f:
            return yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        sys.stderr.write(
            f"[fic_studio] WARNING: failed to parse {SETTINGS_PATH}: {e}\n"
            f"[fic_studio] Falling back to default settings. Fix or delete the file to keep your preferences.\n"
        )
        return {}
    except OSError as e:
        sys.stderr.write(f"[fic_studio] WARNING: could not read {SETTINGS_PATH}: {e}\n")
        return {}


def main():
    # ── Splash phase ─────────────────────────────────────────────────────────
    # Import only what's needed to paint the splash. The heavy imports (rclpy,
    # ros_node, main_window_pro) come after the splash is on screen so the
    # user sees something within ~300 ms instead of ~800 ms.
    from PyQt5.QtWidgets import QApplication
    from fic_studio.splash import make_splash, splash_status

    from fic_studio import qt_webengine
    qt_webengine.prepare()      # AA_ShareOpenGLContexts + QtWebEngine import, before QApplication
    app = QApplication(sys.argv)
    app.setApplicationName("FIC Control Studio")
    app.setOrganizationName("RoboticsLab")
    app.setDesktopFileName("fic-control-studio")

    splash = make_splash("FIC Control Studio", version="v5.10.0")

    # ── Settings (lazy yaml import) ──────────────────────────────────────────
    splash_status(splash, "Loading settings…")
    settings = _load_settings()

    # Apply ROS_DOMAIN_ID from settings BEFORE rclpy.init().
    # Only set if not already in the environment (bashrc / launch script wins).
    domain_id = settings.get('network', {}).get('ros_domain_id')
    if domain_id is not None and 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = str(int(domain_id))

    # ── Heavy imports — splash is up, user sees progress ────────────────────
    splash_status(splash, "Connecting to ROS…")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from PyQt5.QtCore import QTimer
    from fic_studio.ros_node import FicStudioNode
    from fic_studio.main_window_pro import ProfessionalRobotGUI
    from fic_studio.mixins.process_mixin import install_crash_cleanup

    rclpy.init(args=sys.argv)
    # station_id 0 = Master: each Master runs on its own internal station (see
    # fleet.stations), so several can share the lab network.
    from fic_studio.fleet.stations import resolve_station
    from fic_studio.ros_node import _host_tag
    # FIC_STATION_ID wins over the settings file, the same way ROS_DOMAIN_ID
    # does: it is how a second Studio on one machine — a container beside
    # another, a test rig — is given a station of its own without editing
    # anybody's settings.
    station_setting = settings.get('general', {}).get('station_id', 1)
    from_env = os.environ.get('FIC_STATION_ID', '').strip()
    if from_env:
        try:
            station_setting = int(from_env)
        except ValueError:
            print('[fic_studio] FIC_STATION_ID=%r is not a number; ignoring' % from_env,
                  file=sys.stderr)
    station_id = resolve_station(station_setting, _host_tag())
    node = FicStudioNode(station_id=station_id)
    # Robot-state topics (1 kHz from a controller or simulator) on their own
    # node and thread, so they cannot starve the UI. See start_state_node.
    node.start_state_node()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    splash_status(splash, "Building UI…")
    window = ProfessionalRobotGUI(node)

    splash_status(splash, "Ready")
    web_window = getattr(window, '_web_ui_window', None)
    if web_window is not None:
        # --web-ui: the web window IS the app. The classic window stays alive
        # but never shown (showing then hiding it left no visible window for a
        # moment, which can end the event loop). Quitting is explicit: closing
        # the web window calls app.quit() (see WebUIMixin), so nothing depends
        # on Qt's last-window-closed rule.
        app.setQuitOnLastWindowClosed(False)
        # The splash stays until the web window has its page (WebUIMixin._web_ui_reveal).
        if getattr(window, '_web_ui_shown', False):
            splash.close()
        else:
            window._web_ui_on_shown = splash.close
    elif getattr(window, '_web_ui_quit_requested', False):
        splash.close()
        QTimer.singleShot(0, app.quit)      # QtWebEngine missing and the user chose Quit
    else:
        window.show()
        splash.finish(window)

    # Make sure every spawned ros2 launch / mujoco / driver dies with the GUI
    # — including on uncaught exceptions, external SIGTERM, or terminal HUP.
    # Hard-kill paths (SIGKILL, segfault) are covered by PR_SET_PDEATHSIG in
    # the children themselves. See process_mixin.install_crash_cleanup.
    install_crash_cleanup(window, app)

    import time as _time

    def _spin_ros():
        # Process callbacks for up to 5ms, then yield back to Qt event loop
        deadline = _time.monotonic() + 0.008
        while _time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0)

    timer = QTimer()
    timer.timeout.connect(_spin_ros)
    timer.start(10)  # tick every 10ms, spin for up to 5ms each

    signal.signal(signal.SIGINT, lambda *args: app.quit())

    exit_code = app.exec_()

    executor.shutdown()
    node.stop_state_node()
    node.destroy_node()
    rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
