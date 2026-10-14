"""Tests for re-asserting the control mode when a session starts.

`{rid}_control_mode` is VOLATILE and published only when the operator
picks a mode, so any node that starts alongside the arm never learns the
mode already in force. contact_node is exactly that node now that
contact_launch.py rides along with the hardware stack, and it refuses to
estimate without a mode -- which showed up as contact silently not
working after every restart.
"""
import subprocess
import sys
import types

from fic_studio.mixins import process_mixin
from fic_studio.mixins.process_mixin import ProcessMixin


class _Host(ProcessMixin):
    """Stand-in carrying just what the controller-start path reads.

    Subclasses ProcessMixin rather than standing apart from it:
    _start_controller calls _republish_control_mode on self, so the real
    method has to resolve for the scheduling to be exercised at all.
    """

    def __init__(self, mode_idx=0):
        self.node = types.SimpleNamespace(
            station_id=1, published_modes=[])
        self.node.publish_mode = self.node.published_modes.append
        self.robot_dropdown = types.SimpleNamespace(
            currentText=lambda: 'KinovaGen3_7DOF')
        self._rc_selected_mode_idx = mode_idx
        self.controller_toggle_btn = None
        self.logs = []

    def log_message(self, level, msg, source=''):
        """Record what would have gone to the log widget."""
        self.logs.append((level, msg, source))


def test_republish_sends_the_selected_mode_one_based():
    """_rc_selected_mode_idx is 0-based; the topic carries 1..8."""
    h = _Host(mode_idx=0)
    h._republish_control_mode()
    assert h.node.published_modes == [1]

    h = _Host(mode_idx=4)
    h._republish_control_mode()
    assert h.node.published_modes == [5]


def test_republish_is_safe_with_no_selection_recorded():
    """A host that never set the attribute must still assert mode 1."""
    h = _Host()
    del h._rc_selected_mode_idx
    h._republish_control_mode()
    assert h.node.published_modes == [1]


def test_start_controller_schedules_a_republish(monkeypatch):
    """Starting the arm must re-assert the mode once the stack is up.

    Deferred, not immediate: the subscription has to exist before a
    VOLATILE message will reach it, and contact_node takes about a
    second to load its Pinocchio and MuJoCo models.
    """
    h = _Host(mode_idx=2)
    scheduled = {}

    class _FakeTimer:
        @staticmethod
        def singleShot(ms, cb):
            scheduled['ms'] = ms
            scheduled['cb'] = cb

    monkeypatch.setattr(process_mixin, 'QTimer', _FakeTimer)
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **kw: object())
    monkeypatch.setattr(sys, 'platform', 'linux')

    h._start_controller()

    assert scheduled.get('ms', 0) > 0, 'republish was not deferred'
    assert h.node.published_modes == [], 'published before the stack was up'
    scheduled['cb']()
    assert h.node.published_modes == [3]


def test_republish_also_resends_the_safety_limits():
    """Published at robot selection, before FIC starts, they never landed."""
    h = _Host(mode_idx=4)
    sent = []
    h.safety_limits = {'joint_safety_deg': {'enabled': True}}
    h._publish_safety_limits = lambda: sent.append(True)
    h._republish_control_mode()
    assert h.node.published_modes == [5] and sent == [True]


def test_no_safety_limits_loaded_is_no_reason_to_fail():
    h = _Host()
    h._republish_control_mode()
    assert h.node.published_modes == [1]


# ── Waiting for the subscriber instead of guessing ────────────────────────
#
# The flat 5 s wait was most of the delay operators saw before FIC engaged.
# Measured on a Kinova (9 Oct 2026) the controller came up and then sat idle
# for 8.4 s waiting for the robot selection, and another 8.4 s for the mode:
# the timer plus DDS discovery, twice, with the arm home and powered
# throughout. These cover the replacement.

class _ReadyHost(_Host):
    """A host whose controller subscribes after ``appears_after`` polls."""

    def __init__(self, appears_after=0, countable=True):
        super().__init__()
        self._polls = 0
        self._appears_after = appears_after
        rid = 'robot1'
        self.node.station_robot_id = rid
        if countable:
            pub = types.SimpleNamespace(
                get_subscription_count=self._count)
            self.node.pubs = {rid: {'control_mode': pub}}
        else:
            self.node.pubs = {}          # nothing to count: must not stall

    def _count(self):
        self._polls += 1
        return 1 if self._polls > self._appears_after else 0


def _drain(h, scheduled):
    """Run the single-shot chain the way Qt would, without a Qt loop."""
    while scheduled:
        scheduled.pop(0)()


def test_mode_is_sent_as_soon_as_the_controller_subscribes(monkeypatch):
    scheduled = []
    monkeypatch.setattr(process_mixin, 'QTimer',
                        types.SimpleNamespace(
                            singleShot=lambda ms, fn: scheduled.append(fn)))
    h = _ReadyHost(appears_after=2)
    h._mode_republish_waited_ms = 0
    h._mode_republish_when_ready()
    _drain(h, scheduled)
    assert h.node.published_modes == [1]
    # Three polls at 250 ms: far inside the 5 s backstop.
    assert h._mode_republish_waited_ms <= 750
    assert any('Controller subscribed after' in m for _, m, _ in h.logs)


def test_backstop_still_publishes_if_nobody_ever_subscribes(monkeypatch):
    scheduled = []
    monkeypatch.setattr(process_mixin, 'QTimer',
                        types.SimpleNamespace(
                            singleShot=lambda ms, fn: scheduled.append(fn)))
    h = _ReadyHost(appears_after=10**6)        # never appears
    h._mode_republish_waited_ms = 0
    h._mode_republish_when_ready()
    _drain(h, scheduled)
    assert h.node.published_modes == [1], 'the arm must still get its mode'
    assert h._mode_republish_waited_ms >= process_mixin._MODE_REPUBLISH_MS


def test_an_uncountable_publisher_does_not_hold_the_mode_back(monkeypatch):
    """If the count cannot be read, publish rather than wait out the backstop."""
    scheduled = []
    monkeypatch.setattr(process_mixin, 'QTimer',
                        types.SimpleNamespace(
                            singleShot=lambda ms, fn: scheduled.append(fn)))
    h = _ReadyHost(countable=False)
    h._mode_republish_waited_ms = 0
    h._mode_republish_when_ready()
    _drain(h, scheduled)
    assert h.node.published_modes == [1]
    assert h._mode_republish_waited_ms == process_mixin._MODE_REPUBLISH_POLL_MS
