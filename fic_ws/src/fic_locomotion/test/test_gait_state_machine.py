"""Gait supervision: transitions, blend timing, and fall handling."""
import pytest

from fic_locomotion.gait_state_machine import (
    GaitState, GaitStateMachine, TransitionParams)


def _settle(machine, seconds, speed=0.0, tilt=0.0, dt=0.005):
    for _ in range(int(seconds / dt)):
        machine.update(dt, speed, tilt, True)
    return machine.state


def test_starts_sitting():
    assert GaitStateMachine().state is GaitState.SIT


def test_stand_command_blends_over_the_configured_time():
    machine = GaitStateMachine(TransitionParams(stand_time=2.0))
    machine.request('stand')
    assert machine.state is GaitState.STANDING_UP
    assert _settle(machine, 1.0) is GaitState.STANDING_UP
    assert machine.blend == pytest.approx(0.5, abs=0.02)
    assert _settle(machine, 1.1) is GaitState.STAND
    assert machine.blend == pytest.approx(1.0)


def test_stick_input_starts_a_trot_from_standing():
    machine = GaitStateMachine()
    machine.request('stand')
    _settle(machine, 2.1)
    assert machine.update(0.005, 0.3, 0.0, True) is GaitState.TROT


def test_trot_stops_only_after_the_idle_hold_expires():
    machine = GaitStateMachine(TransitionParams(stop_hold=0.4))
    machine.request('stand')
    _settle(machine, 2.1)
    _settle(machine, 0.5, speed=0.3)
    # A brief zero crossing between joystick pushes must not break the gait.
    assert _settle(machine, 0.2) is GaitState.TROT
    assert _settle(machine, 0.3) is GaitState.STAND


def test_excess_tilt_faults_immediately():
    machine = GaitStateMachine(TransitionParams(fault_tilt=0.7))
    machine.request('stand')
    _settle(machine, 2.1)
    assert machine.update(0.005, 0.0, 0.9, True) is GaitState.FAULT


def test_dead_feedback_faults():
    machine = GaitStateMachine()
    machine.request('stand')
    _settle(machine, 2.1)
    assert machine.update(0.005, 0.0, 0.0, False) is GaitState.FAULT


def test_fault_requires_an_explicit_stand_to_clear():
    machine = GaitStateMachine()
    machine.update(0.005, 0.0, 0.9, True)
    assert _settle(machine, 1.0) is GaitState.FAULT
    machine.request('stand')
    assert machine.state is GaitState.STANDING_UP


def test_stand_command_holds_the_gait_while_the_stick_is_held():
    # `request('stand')` set TROT -> STAND, and `update` put it straight back
    # into TROT on the same tick because the stick was still deflected. The
    # operator's stand button was a no-op for as long as anyone held the stick.
    machine = GaitStateMachine()
    machine.request('stand')
    _settle(machine, 2.1)
    _settle(machine, 0.1, speed=0.3)
    assert machine.state is GaitState.TROT
    machine.request('stand')
    assert machine.state is GaitState.STAND
    assert _settle(machine, 1.0, speed=0.3) is GaitState.STAND


def test_walk_command_resumes_a_held_stand():
    # The counterpart: `request('walk')` was unreachable dead code, because
    # STAND auto-promoted to TROT before anyone could send it.
    machine = GaitStateMachine()
    machine.request('stand')
    _settle(machine, 2.1)
    _settle(machine, 0.1, speed=0.3)
    machine.request('stand')
    _settle(machine, 0.5, speed=0.3)
    machine.request('walk')
    assert machine.state is GaitState.TROT
    assert _settle(machine, 0.5, speed=0.3) is GaitState.TROT


def test_releasing_the_stick_rearms_automatic_walking():
    machine = GaitStateMachine()
    machine.request('stand')
    _settle(machine, 2.1)
    _settle(machine, 0.1, speed=0.3)
    machine.request('stand')
    _settle(machine, 0.5, speed=0.0)  # stick released: the hold is spent
    assert _settle(machine, 0.1, speed=0.3) is GaitState.TROT


def test_sit_command_works_from_a_trot():
    machine = GaitStateMachine()
    machine.request('stand')
    _settle(machine, 2.1)
    _settle(machine, 0.1, speed=0.3)
    machine.request('sit')
    assert machine.state is GaitState.SITTING_DOWN
    assert _settle(machine, 2.1) is GaitState.SIT
