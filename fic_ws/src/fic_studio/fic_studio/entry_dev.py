#!/usr/bin/env python3
# entry_dev.py — Entry point for FIC Dev Console (hardware-focused lightweight GUI)

import sys
import os
import signal

SETTINGS_PATH = os.path.expanduser('~/.robot_selector_gui_settings.yaml')


def _load_settings() -> dict:
    """yaml is imported lazily so the splash paints first."""
    if not os.path.exists(SETTINGS_PATH):
        return {}
    import yaml
    try:
        with open(SETTINGS_PATH) as f:
            return yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        sys.stderr.write(
            f"[fic_dev] WARNING: failed to parse {SETTINGS_PATH}: {e}\n"
            f"[fic_dev] Falling back to default settings. Fix or delete the file to keep your preferences.\n"
        )
        return {}
    except OSError as e:
        sys.stderr.write(f"[fic_dev] WARNING: could not read {SETTINGS_PATH}: {e}\n")
        return {}


def main():
    # ── Splash phase: only PyQt5 + splash module imported so far ────────────
    from PyQt5.QtWidgets import QApplication
    from fic_studio.splash import make_splash, splash_status

    from fic_studio import qt_webengine
    qt_webengine.prepare()      # AA_ShareOpenGLContexts + QtWebEngine import, before QApplication
    app = QApplication(sys.argv)
    app.setApplicationName("FIC Dev Console")
    app.setOrganizationName("RoboticsLab")

    splash = make_splash("FIC Dev Console", version="v5.9.0")

    splash_status(splash, "Loading settings…")
    settings = _load_settings()

    # Apply ROS_DOMAIN_ID from settings BEFORE rclpy.init().
    domain_id = settings.get('network', {}).get('ros_domain_id')
    if domain_id is not None and 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = str(int(domain_id))

    # ── Heavy imports happen under the splash ───────────────────────────────
    splash_status(splash, "Connecting to ROS…")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from PyQt5.QtCore import QTimer
    from fic_studio.ros_node import FicStudioNode
    from fic_studio.main_window_dev import FICDevConsole
    from fic_studio.mixins.process_mixin import install_crash_cleanup

    rclpy.init(args=sys.argv)
    # station_id 0 = Master: each Master runs on its own internal station (see
    # fleet.stations), so several can share the lab network.
    from fic_studio.fleet.stations import resolve_station
    from fic_studio.ros_node import _host_tag
    station_id = resolve_station(settings.get('general', {}).get('station_id', 1), _host_tag())
    node = FicStudioNode(station_id=station_id)
    # Robot-state topics (1 kHz from a controller or simulator) on their own
    # node and thread, so they cannot starve the UI. See start_state_node.
    node.start_state_node()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    splash_status(splash, "Building UI…")
    window = FICDevConsole(node)

    splash_status(splash, "Ready")
    window.show()
    splash.finish(window)

    # Make sure every spawned ros2 launch / driver / bag process dies with
    # the GUI — including on uncaught exceptions, external SIGTERM, or
    # terminal HUP. Hard-kill paths (SIGKILL, segfault) are covered by
    # PR_SET_PDEATHSIG in the children themselves. See
    # process_mixin.install_crash_cleanup.
    install_crash_cleanup(window, app)

    import time as _time

    def _spin_ros():
        deadline = _time.monotonic() + 0.008
        while _time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0)

    timer = QTimer()
    timer.timeout.connect(_spin_ros)
    timer.start(10)

    signal.signal(signal.SIGINT, lambda *args: app.quit())

    exit_code = app.exec_()

    executor.shutdown()
    node.stop_state_node()
    node.destroy_node()
    rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
