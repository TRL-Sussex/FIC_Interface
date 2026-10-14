"""My Extension: the template for an extension with a web page.

Copy it with ``python3 fic_install/new_extension.py <id> "<Name>"`` rather than by
hand: that also renames the id, the module path and the class. The guide is
fic_docs/pages/extensions.html ("Writing Extensions").

The Studio side is Python: ``web_state()`` says what the page shows and
``web_action()`` does what the page asks. The page (web/page.js) only draws.
"""
import math
import time

from fic_studio.extensions.base import Extension as _Base

#: How many log lines the page keeps.
LOG_LINES = 200


class Extension(_Base):
    # The same values as manifest.json (discovery reads the manifest without importing this).
    ID = "my_extension"
    NAME = "My Extension"
    DESCRIPTION = "A starting point: live robot readouts, a setting, an on/off switch, actions and a log, on a web page."
    SIDEBAR_SECTION = "Extensions"

    def __init__(self, host):
        super().__init__(host)
        self._log = []
        self._publisher = None
        # Settings persist in ~/.robot_selector_gui_settings.yaml under this extension's id.
        self._gain = float(host.get_setting("gain", 0.5))
        self._enabled = bool(host.get_setting("enabled", False))

    # ── No build_page(): this extension only has a web page. The classic Qt
    #    window shows a note saying where to find it. Add build_page() returning
    #    a QWidget if you also want a classic page.

    def on_activate(self):
        """The page was opened (first web_state or web_action). Start what the page needs."""
        self._say("Page opened")

    def teardown(self):
        """The Studio is closing: stop timers, processes and publishers."""
        if self._publisher is not None:
            try:
                self.host.node.destroy_publisher(self._publisher)
            except Exception:
                pass
            self._publisher = None

    def on_emergency_stop(self):
        """E-STOP: stop anything this extension is driving, at once."""
        if self._enabled:
            self._enabled = False
            self.host.set_setting("enabled", False)
            self._say("E-STOP: switched off")

    # ── The web page ──────────────────────────────────────────────────────
    def web_state(self):
        """What the page shows, every poll_ms (manifest). Read only; keep it quick."""
        host = self.host
        pose = host.current_ee_pose
        joints = host.current_joint_positions or []
        return {
            "robot": host.current_robot_name(),
            "target": host.target_robot_id,
            "running": bool(host.controller_running),
            "ee": None if pose is None else [round(pose.position.x, 4), round(pose.position.y, 4), round(pose.position.z, 4)],
            "joints_deg": [round(math.degrees(q), 1) for q in joints],
            "gain": self._gain,
            "enabled": self._enabled,
            "log": self._log[-LOG_LINES:],
        }

    def web_action(self, name, args):
        """Do what the page asked. Raise ValueError("why") to refuse; return a JSON-able result."""
        if name == "set_gain":
            value = float(args.get("value"))
            if not 0.0 <= value <= 1.0:
                raise ValueError("Gain must be between 0 and 1")
            self._gain = value
            self.host.set_setting("gain", value)
            self._say("Gain set to %.2f" % value)
            return {"gain": value}
        if name == "set_enabled":
            self._enabled = bool(args.get("value"))
            self.host.set_setting("enabled", self._enabled)
            self._say("Switched %s" % ("on" if self._enabled else "off"))
            return {"enabled": self._enabled}
        if name == "hello":
            self._say("Hello from the web page")
            self.host.log("INFO", "Hello from %s" % self.NAME)        # the Studio's Logs page
            return {"time": time.time()}
        if name == "publish":
            return {"sent": self._publish(str(args.get("text") or "hello"))}
        if name == "clear_log":
            self._log.clear()
            return {}
        if name == "reset_settings":
            # Listed in manifest.json "local_only": refused for remote clients and phones.
            self._gain, self._enabled = 0.5, False
            self.host.set_setting("gain", self._gain)
            self.host.set_setting("enabled", self._enabled)
            self._say("Settings reset")
            return {}
        raise ValueError("No such action: %s" % name)

    # ── Helpers ───────────────────────────────────────────────────────────
    def _publish(self, text):
        """Publish a std_msgs/String on /my_extension/message (create the publisher once)."""
        from std_msgs.msg import String
        if self._publisher is None:
            self._publisher = self.host.node.create_publisher(String, "/%s/message" % self.ID, 10)
        self._publisher.publish(String(data=text))
        self._say("Published %r" % text)
        return text

    def _say(self, text):
        self._log.append("%s  %s" % (time.strftime("%H:%M:%S"), text))
        del self._log[:-LOG_LINES]
