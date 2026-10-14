"""
Safety regression: a zone violation ENFORCES a graceful stop (not just a warn).

WS3a of the safety remediation. The 10 Hz monitor used to only log + (teleop
only) stop. It now commands a debounced graceful controller stop on any
violation, for every mode, as a runtime backstop to the target clamping.
"""
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.safety_mixin import SafetyMixin  # noqa: E402


class _SafetyStub:
    def __init__(self):
        self.safety_enforce = True
        self._safety_stop_until = 0.0
        self.broadcast_calls = 0
        self.logs = []

    def broadcast_stop(self):
        self.broadcast_calls += 1

    def log_message(self, *a):
        self.logs.append(a)


def test_violation_enforces_graceful_stop_once():
    s = _SafetyStub()
    SafetyMixin._enforce_safety_stop(s, 'Z max 0.9 (limit 0.5)')
    assert s.broadcast_calls == 1            # first violation -> graceful stop
    SafetyMixin._enforce_safety_stop(s, 'Z max 0.9 (limit 0.5)')
    assert s.broadcast_calls == 1            # debounced within the cooldown


def test_enforce_disabled_does_not_stop():
    s = _SafetyStub()
    s.safety_enforce = False
    SafetyMixin._enforce_safety_stop(s, 'Z max')
    assert s.broadcast_calls == 0
