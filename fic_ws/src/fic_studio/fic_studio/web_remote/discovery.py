"""Let every FIC Studio on the ROS network find the others' web remotes.

Each Studio publishes a small JSON card about its own page every
``ANNOUNCE_S`` seconds and listens for everyone else's, so any Studio's page
can offer a switcher to the rest. The Studios already share a ROS domain, so
this needs no extra discovery service. Runs on the Qt thread: the Studio's
executor delivers ROS callbacks there.
"""
import json
import time

from std_msgs.msg import String

TOPIC = '/fic_studio/web_remotes'
ANNOUNCE_S = 2.0
#: A Studio not heard from for this long is dropped from the list.
EXPIRE_S = 3.5 * ANNOUNCE_S

_REQUIRED = ('gui_id', 'label', 'urls')


class StudioDirectory:
    """Announce this Studio's page and keep a live list of the others."""

    def __init__(self, node, own_card, clock=time.monotonic):
        """``own_card()`` returns this Studio's card: gui_id, label, urls..."""
        self._node = node
        self._own_card = own_card
        self._clock = clock
        self._peers = {}
        self._pub = node.create_publisher(String, TOPIC, 10)
        self._sub = node.create_subscription(String, TOPIC, self._on_card, 10)

    def announce(self):
        self._pub.publish(String(data=json.dumps(self._own_card())))

    def _on_card(self, msg):
        try:
            card = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(card, dict) or not all(k in card for k in _REQUIRED):
            return
        if card['gui_id'] == self._own_card().get('gui_id'):
            return
        self._peers[card['gui_id']] = (card, self._clock())

    def studios(self):
        """Return this Studio first, then every live peer, as cards with 'self' set."""
        now = self._clock()
        self._peers = {gid: (card, seen) for gid, (card, seen) in self._peers.items()
                       if now - seen <= EXPIRE_S}
        own = dict(self._own_card(), self=True)
        peers = sorted((dict(card, self=False) for card, _ in self._peers.values()),
                       key=lambda c: c.get('label', ''))
        return [own] + peers

    def close(self):
        for destroy, handle in ((self._node.destroy_subscription, self._sub),
                                (self._node.destroy_publisher, self._pub)):
            try:
                destroy(handle)
            except Exception:
                pass
