"""Who may command the robot from the web: one client at a time.

Any browser can watch. To command, a client takes the lease and keeps it by
sending a heartbeat; a holder that goes quiet for ``LEASE_TIMEOUT_S`` (phone
locked, tab hidden, Wi-Fi dropped) loses it and another client may take it.
Nobody can take it from a live holder. The E-stop never needs the lease.
"""
import threading
import time

#: A holder that has sent no heartbeat for this long no longer holds control.
LEASE_TIMEOUT_S = 3.0


class ControlLease:
    """Thread-safe single-holder lease; every HTTP request thread shares one."""

    def __init__(self, timeout_s=LEASE_TIMEOUT_S, clock=time.monotonic):
        self._timeout = float(timeout_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._holder = None

    @property
    def timeout_s(self):
        return self._timeout

    def _live(self, now):
        """Return the holder, dropping it first if its heartbeat has lapsed."""
        holder = self._holder
        if holder is not None and now - holder['beat'] > self._timeout:
            self._holder = holder = None
        return holder

    def take(self, client, name='', addr=''):
        """Give ``client`` control unless another live client holds it.

        Returns ``(granted, holder)`` where ``holder`` is a copy of whoever
        holds control afterwards. Taking a lease you already hold renews it.
        """
        with self._lock:
            now = self._clock()
            holder = self._live(now)
            if holder is not None and holder['client'] != client:
                return False, dict(holder)
            if holder is None:
                self._holder = holder = {
                    'client': client, 'name': name, 'addr': addr,
                    'since': now, 'beat': now,
                }
            else:
                holder['beat'] = now
            return True, dict(holder)

    def heartbeat(self, client):
        """Renew ``client``'s lease. False if it does not hold control."""
        with self._lock:
            holder = self._live(self._clock())
            if holder is None or holder['client'] != client:
                return False
            holder['beat'] = self._clock()
            return True

    def release(self, client):
        """Give up control. False if ``client`` did not hold it."""
        with self._lock:
            holder = self._live(self._clock())
            if holder is None or holder['client'] != client:
                return False
            self._holder = None
            return True

    def revoke(self):
        """Take control away from whoever holds it (Studio side, or E-stop)."""
        with self._lock:
            holder = self._live(self._clock())
            self._holder = None
            return dict(holder) if holder else None

    def holds(self, client):
        with self._lock:
            holder = self._live(self._clock())
            return holder is not None and holder['client'] == client

    def holder(self):
        """Return a copy of the live holder, or None."""
        with self._lock:
            holder = self._live(self._clock())
            return dict(holder) if holder else None
