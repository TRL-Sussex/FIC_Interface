"""Share one lab's user list between its Studios, edited from the Master.

An admin should not have to add a user on four machines. Each Studio publishes
its store on a ROS topic every ``SHARE_S`` seconds and takes anyone else's when
it carries a newer ``version``, so an edit made on the Master reaches every
Studio within a few seconds and survives a restart (each one saves what it
takes).

Only the Master publishes edits as they happen; every Studio repeats what it
holds, so a Studio started later is caught up by the next repeat without asking
anyone. Hashes travel, never passwords — and only between Studios on the lab's
own ROS domain, never to a browser.
"""
import json
import time

from std_msgs.msg import String

TOPIC = '/fic_studio/accounts'
#: How often a Studio repeats what it holds, for the ones that just started.
SHARE_S = 5.0


class AccountsSync:
    """Keep one ``Accounts`` store in step with the other Studios'."""

    def __init__(self, node, accounts, on_changed=None, clock=time.monotonic):
        """Share ``accounts``; ``on_changed()`` runs when someone else's wins."""
        self.accounts = accounts
        self._on_changed = on_changed
        self._clock = clock
        self._last_share = 0.0
        self._node = node
        # Transient-local: a Studio starting later is caught up at once by the
        # last message, without waiting for the next repeat.
        self._pub = node.create_publisher(String, TOPIC, 10)
        self._sub = node.create_subscription(String, TOPIC, self._on_store, 10)

    def share(self, force=False):
        """Publish what this Studio holds (at most every ``SHARE_S``, unless forced)."""
        now = self._clock()
        if not force and now - self._last_share < SHARE_S:
            return False
        self._last_share = now
        self._pub.publish(String(data=json.dumps(self.accounts.as_dict())))
        return True

    def _on_store(self, msg):
        try:
            blob = json.loads(msg.data)
        except ValueError:
            return
        if not self.accounts.merge(blob):
            return                      # ours is the same or newer: nothing to do
        try:
            self.accounts.save()
        except OSError:
            pass
        if self._on_changed:
            self._on_changed()

    def close(self):
        for destroy, handle in ((self._node.destroy_subscription, self._sub),
                                (self._node.destroy_publisher, self._pub)):
            try:
                destroy(handle)
            except Exception:
                pass
