"""Gait supervisor: which behaviour is active, and how transitions blend."""

import enum
from dataclasses import dataclass
from typing import Optional


class GaitState(enum.Enum):
    """Top-level locomotion behaviour."""

    SIT = 'sit'
    STANDING_UP = 'standing_up'
    STAND = 'stand'
    TROT = 'trot'
    SITTING_DOWN = 'sitting_down'
    FAULT = 'fault'


@dataclass(frozen=True)
class TransitionParams:
    """Timing and thresholds governing state changes."""

    stand_time: float = 2.0
    sit_time: float = 2.0
    walk_deadband: float = 0.02
    stop_hold: float = 0.4
    fault_tilt: float = 0.7  # rad, roughly 40 degrees


class GaitStateMachine:
    """Sequence sit, stand, trot and fault recovery."""

    def __init__(self, params: Optional[TransitionParams] = None):
        self.params = params or TransitionParams()
        self.state = GaitState.SIT
        self.blend = 0.0  # 0..1 through STANDING_UP / SITTING_DOWN
        self._idle = 0.0
        # Set by `request('stand')`, cleared by `request('walk')` or by the
        # operator letting go of the stick. Without it, STAND promotes itself
        # back to TROT on the very tick the stand was requested, so the stand
        # button did nothing at all while a command was being held.
        self._hold_stand = False

    def request(self, command: str) -> None:
        """Apply an operator command: 'sit', 'stand' or 'walk'."""
        settled = (GaitState.SIT, GaitState.SITTING_DOWN)
        if command == 'sit' and self.state not in settled:
            self.state = GaitState.SITTING_DOWN
            self.blend = 0.0
        elif command == 'stand':
            self._hold_stand = True
            if self.state is GaitState.TROT:
                self.state = GaitState.STAND
                self._idle = 0.0
            elif self.state in (GaitState.SIT, GaitState.FAULT,
                                GaitState.SITTING_DOWN):
                self.state = GaitState.STANDING_UP
                self.blend = 0.0
        elif command == 'walk':
            self._hold_stand = False
            if self.state is GaitState.STAND:
                self.state = GaitState.TROT
                self._idle = 0.0

    def update(self, dt: float, speed: float, tilt: float,
               feedback_ok: bool) -> GaitState:
        """
        Advance one tick and return the resulting state.

        `speed` is a scalar summary of how hard the operator is asking for
        motion; `tilt` is the trunk's total deviation from level in radians.
        """
        # A fall, or state feedback we cannot trust, drops everything to FAULT.
        # Recovery is deliberately manual: an automatic retry on a robot that
        # has just fallen over is how you break hardware.
        if not feedback_ok or tilt > self.params.fault_tilt:
            if self.state is not GaitState.FAULT:
                self.state = GaitState.FAULT
                self.blend = 0.0
            return self.state

        if self.state is GaitState.STANDING_UP:
            self.blend = min(1.0, self.blend + dt / self.params.stand_time)
            if self.blend >= 1.0:
                self.state = GaitState.STAND
        elif self.state is GaitState.SITTING_DOWN:
            self.blend = min(1.0, self.blend + dt / self.params.sit_time)
            if self.blend >= 1.0:
                self.state = GaitState.SIT
        elif self.state is GaitState.STAND:
            if speed < self.params.walk_deadband:
                # Stick released: an explicit stand has been honoured, so let
                # the next deflection start a trot again by itself.
                self._hold_stand = False
            elif not self._hold_stand:
                self.state = GaitState.TROT
                self._idle = 0.0
        elif self.state is GaitState.TROT:
            # Hold the gait briefly through a zero command so that easing off
            # the stick between pushes does not restart the whole trot.
            if speed < self.params.walk_deadband:
                self._idle += dt
                if self._idle >= self.params.stop_hold:
                    self.state = GaitState.STAND
            else:
                self._idle = 0.0
        return self.state
