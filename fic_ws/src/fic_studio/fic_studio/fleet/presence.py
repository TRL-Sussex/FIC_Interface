"""Presence: every Studio announces itself; each keeps a live list of the rest.

Each Studio publishes a small JSON card on ``TOPIC`` every ``ANNOUNCE_S``
seconds: who it is (host, station), what it drives (robot type, simulation or
hardware), what it is doing (running, mode, teleop) and where its web remote
is. A Studio not heard from for ``EXPIRE_S`` is dropped.

The same list feeds the Multi-Robot page's station cards, the teleop station
pickers and the web remote's Studio switcher.
"""
import json
import time

TOPIC = '/fic/fleet/presence'
ANNOUNCE_S = 1.0
#: A Studio not heard from for this long is dropped from the list.
EXPIRE_S = 3.5 * ANNOUNCE_S

_REQUIRED = ('studio_id', 'station', 'host')


class FleetDirectory:
    """Announce this Studio's card and keep a live list of the others."""

    def __init__(self, node, own_card, clock=time.monotonic):
        """Start announcing and listening.

        ``own_card()`` returns this Studio's card: a dict with at least
        studio_id, station and host.
        """
        from std_msgs.msg import String
        self._string = String
        self._node = node
        self._own_card = own_card
        self._clock = clock
        self._peers = {}
        self._pub = node.create_publisher(String, TOPIC, 10)
        self._sub = node.create_subscription(String, TOPIC, self.on_message, 10)

    def announce(self):
        """Publish this Studio's card now."""
        self._pub.publish(self._string(data=json.dumps(self._own_card())))

    def on_message(self, msg):
        """Record a peer's card (ROS callback)."""
        self.on_card_json(msg.data)

    def on_card_json(self, text):
        """Record a peer's card from its JSON text; ignore anything malformed."""
        try:
            card = json.loads(text)
        except (TypeError, ValueError):
            return
        if not isinstance(card, dict) or not all(k in card for k in _REQUIRED):
            return
        if not isinstance(card.get('station'), int):
            return
        if card['studio_id'] == self._own_card().get('studio_id'):
            return
        self._peers[card['studio_id']] = (card, self._clock())

    def _live_peers(self):
        now = self._clock()
        self._peers = {sid: (card, seen) for sid, (card, seen) in self._peers.items()
                       if now - seen <= EXPIRE_S}
        return [dict(card, self=False, age_s=round(now - seen, 1))
                for card, seen in self._peers.values()]

    def studios(self):
        """Return this Studio's card first, then every live peer's, by station."""
        own = dict(self._own_card(), self=True, age_s=0.0)
        peers = sorted(self._live_peers(),
                       key=lambda c: (c.get('station', 0), c.get('host', '')))
        return [own] + peers

    def station(self, n):
        """Return the card of the Studio on station ``n`` (this one first), or None."""
        for card in self.studios():
            if card.get('station') == n:
                return card
        return None

    def conflicts(self):
        """Return {station: [hosts]} for every station claimed by two or more Studios.

        Two Studios on one station both command robotN_*: a real hazard when
        one of them drives hardware.
        """
        claims = {}
        for card in self.studios():
            claims.setdefault(card.get('station'), []).append(card.get('host', '?'))
        return {n: hosts for n, hosts in claims.items() if len(hosts) > 1}

    def close(self):
        """Stop announcing and listening."""
        for destroy, handle in ((self._node.destroy_subscription, self._sub),
                                (self._node.destroy_publisher, self._pub)):
            try:
                destroy(handle)
            except Exception:
                pass


#: Where FIC API scripts announce themselves (fic_api.robot, every 2 s).
CLIENTS_TOPIC = '/fic/fleet/clients'
CLIENT_EXPIRE_S = 7.0


class ClientList:
    """Keep a live list of FIC API scripts on the network and the station each commands."""

    def __init__(self, node, clock=time.monotonic):
        """Listen on CLIENTS_TOPIC."""
        from std_msgs.msg import String
        self._node = node
        self._clock = clock
        self._clients = {}
        self._sub = node.create_subscription(String, CLIENTS_TOPIC, self.on_message, 10)

    def on_message(self, msg):
        """Record a client's card (ROS callback)."""
        self.on_card_json(msg.data)

    def on_card_json(self, text):
        """Record a client's card from its JSON text; ignore anything malformed."""
        try:
            card = json.loads(text)
        except (TypeError, ValueError):
            return
        if not isinstance(card, dict) or not card.get('client_id') \
                or not isinstance(card.get('station'), int):
            return
        self._clients[card['client_id']] = (card, self._clock())

    def clients(self, station=None):
        """List the live clients, only those commanding ``station`` when it is given."""
        now = self._clock()
        self._clients = {k: v for k, v in self._clients.items() if now - v[1] <= CLIENT_EXPIRE_S}
        return [c for c, _ in self._clients.values()
                if station is None or c.get('station') == station]

    def close(self):
        """Stop listening."""
        try:
            self._node.destroy_subscription(self._sub)
        except Exception:
            pass
