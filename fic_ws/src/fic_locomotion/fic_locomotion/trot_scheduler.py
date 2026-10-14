"""
Phase bookkeeping for a diagonal trot.

The gait clock is a single normalised phase in [0, 1). Each leg reads that
clock through a fixed offset: the diagonal pairs (FL, HR) and (FR, HL) sit half
a cycle apart, which is what makes a trot a trot. A duty factor above 0.5 keeps
the diagonals overlapping so at least two feet are always down — a kinematic
gait has no flight-phase recovery, so it must never leave the ground.
"""

from dataclasses import dataclass

# Fraction of a cycle each leg lags the gait clock.
LEG_PHASE_OFFSET = {'fl': 0.0, 'fr': 0.5, 'hl': 0.5, 'hr': 0.0}


@dataclass(frozen=True)
class GaitParams:
    """Timing and shape of one gait cycle."""

    cycle_time: float = 0.30
    duty_factor: float = 0.65
    step_height: float = 0.12


@dataclass(frozen=True)
class LegPhase:
    """Where one leg is within its own stance or swing interval."""

    stance: bool
    progress: float  # 0..1 through the current sub-phase


def advance(phase: float, dt: float, cycle_time: float) -> float:
    """Move the gait clock forward by `dt`, wrapping at one full cycle."""
    return (phase + dt / cycle_time) % 1.0


def leg_phase(phase: float, leg_name: str, duty_factor: float) -> LegPhase:
    """Resolve the shared gait clock into one leg's stance/swing state."""
    local = (phase + LEG_PHASE_OFFSET[leg_name]) % 1.0
    if local < duty_factor:
        return LegPhase(True, local / duty_factor)
    return LegPhase(False, (local - duty_factor) / (1.0 - duty_factor))


def stance_duration(params: GaitParams) -> float:
    """Return the seconds one foot spends planted per cycle."""
    return params.cycle_time * params.duty_factor


def swing_duration(params: GaitParams) -> float:
    """Return the seconds one foot spends in the air per cycle."""
    return params.cycle_time * (1.0 - params.duty_factor)
