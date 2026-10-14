"""The web remote's control lease: one commanding browser at a time."""
from fic_studio.web_remote.lease import ControlLease


class _Clock:
    """A clock the test moves by hand."""

    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def _lease():
    clock = _Clock()
    return ControlLease(timeout_s=3.0, clock=clock), clock


def test_first_taker_gets_control():
    lease, _ = _lease()
    granted, holder = lease.take('a', 'Phone A', '10.0.0.2')
    assert granted
    assert holder['name'] == 'Phone A'
    assert lease.holds('a')


def test_nobody_can_take_control_from_a_live_holder():
    lease, clock = _lease()
    lease.take('a')
    clock.now += 2.9
    granted, holder = lease.take('b')
    assert not granted
    assert holder['client'] == 'a'
    assert lease.holds('a') and not lease.holds('b')


def test_a_silent_holder_loses_control():
    lease, clock = _lease()
    lease.take('a')
    clock.now += 3.1
    assert not lease.holds('a')
    assert lease.holder() is None
    assert lease.take('b')[0]


def test_heartbeat_keeps_control():
    lease, clock = _lease()
    lease.take('a')
    for _ in range(5):
        clock.now += 2.0
        assert lease.heartbeat('a')
    assert lease.holds('a')


def test_heartbeat_from_a_non_holder_does_not_grant():
    lease, _ = _lease()
    lease.take('a')
    assert not lease.heartbeat('b')
    assert not lease.holds('b')


def test_release_frees_control_only_for_the_holder():
    lease, _ = _lease()
    lease.take('a')
    assert not lease.release('b')
    assert lease.release('a')
    assert lease.take('b')[0]


def test_revoke_returns_who_held_it():
    lease, _ = _lease()
    lease.take('a', 'Phone A', '10.0.0.2')
    revoked = lease.revoke()
    assert revoked['addr'] == '10.0.0.2'
    assert lease.holder() is None
    assert lease.revoke() is None
