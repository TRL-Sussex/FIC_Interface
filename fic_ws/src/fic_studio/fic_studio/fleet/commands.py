"""Lab-wide commands from any Studio: E-STOP ALL, PAUSE ALL / RESUME ALL, and starting a robot.

A command names the stations it is for (or 'all'). Every Studio listens,
and one whose own station is named runs it through its normal local path
(the same E-stop the toolbar button runs). The sender also runs it at once
on itself when named, rather than waiting for its own message to come back.

``session`` starts or stops one station's session (``op``: start/stop, ``target``:
simulation/hardware), carried out by that station's Studio with the same checks
as its web remote; it is addressed to exactly one station.

E-stop over the network is a convenience, NOT a safety-rated stop: it needs
the network and the receiving Studio to be up. The local E-stop on each
robot remains the primary stop.
"""
import json
import time
import uuid

TOPIC = '/fic/fleet/command'
COMMANDS = ('estop', 'pause', 'resume', 'session')
SESSION_OPS = ('start', 'stop')
TARGETS = ('simulation', 'hardware')
#: Remember this many command ids, to act on each command once.
_SEEN_MAX = 256


class FleetCommands:
    """Send and receive lab-wide commands for this Studio's station."""

    def __init__(self, node, station, host, handler, clock=time.time):
        """Listen on TOPIC.

        ``handler(cmd, info)`` runs a command addressed to ``station``;
        ``info`` is the message dict (who sent it, which stations).
        """
        from std_msgs.msg import String
        self._string = String
        self._node = node
        self._station = station
        self._host = host
        self._handler = handler
        self._clock = clock
        self._seen = []
        self._pub = node.create_publisher(String, TOPIC, 10)
        self._sub = node.create_subscription(String, TOPIC, self.on_message, 10)

    def send(self, cmd, stations='all', reason='', **args):
        """Send ``cmd`` to ``stations`` ('all' or a list of numbers); return the message.

        ``session`` takes ``op`` (start/stop) and, to start, ``target``
        (simulation/hardware), and must name exactly one station.
        """
        if cmd not in COMMANDS:
            raise ValueError('unknown fleet command %r' % cmd)
        if stations != 'all':
            stations = sorted({int(n) for n in stations})
        if cmd == 'session':
            if stations == 'all' or len(stations) != 1:
                raise ValueError('a session command names exactly one station')
            if args.get('op') not in SESSION_OPS:
                raise ValueError('session op must be start or stop')
            if args.get('op') == 'start' and args.get('target') not in TARGETS:
                raise ValueError('session start needs a target: simulation or hardware')
        msg = dict(args, id=uuid.uuid4().hex, cmd=cmd, stations=stations,
                   from_station=self._station, from_host=self._host,
                   reason=reason, ts=self._clock())
        self._remember(msg['id'])
        self._pub.publish(self._string(data=json.dumps(msg)))
        if self._addressed(msg):
            self._handler(cmd, msg)
        return msg

    def on_message(self, msg):
        """Handle a command from the network (ROS callback)."""
        self.on_json(msg.data)

    def on_json(self, text):
        """Run a command from its JSON text if it is for this station and new."""
        try:
            msg = json.loads(text)
        except (TypeError, ValueError):
            return
        if not isinstance(msg, dict) or msg.get('cmd') not in COMMANDS:
            return
        if msg['cmd'] == 'session' and (msg.get('stations') == 'all'
                                        or msg.get('op') not in SESSION_OPS):
            return
        if msg.get('id') in self._seen or not self._addressed(msg):
            return
        self._remember(msg.get('id'))
        self._handler(msg['cmd'], msg)

    def _addressed(self, msg):
        stations = msg.get('stations')
        return stations == 'all' or (isinstance(stations, list) and self._station in stations)

    def _remember(self, cmd_id):
        self._seen.append(cmd_id)
        del self._seen[:-_SEEN_MAX]

    def close(self):
        """Stop listening."""
        for destroy, handle in ((self._node.destroy_subscription, self._sub),
                                (self._node.destroy_publisher, self._pub)):
            try:
                destroy(handle)
            except Exception:
                pass
