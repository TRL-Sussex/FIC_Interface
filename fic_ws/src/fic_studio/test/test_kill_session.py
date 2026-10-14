"""Stopping the simulation kills only the Studio's own process tree.

The simulator used to be stopped with `pkill -9 -f mujoco`, which also killed
any other simulator on the PC (another user's, another session's). Children are
started as session leaders (_child_preexec), and _kill_session signals only the
processes in that session.
"""
import os
import signal
import subprocess
import sys
import time

from fic_studio.mixins.process_mixin import _child_preexec, _kill_session


def _alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    # A killed child of this process lingers as a zombie until reaped.
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().split(') ', 1)[1][0] != 'Z'
    except OSError:
        return False


def test_kills_own_session_only():
    # Unrelated process whose command line mentions mujoco: must survive.
    decoy = subprocess.Popen([sys.executable, '-c',
                              'import time; time.sleep(60)  # mujoco decoy'])
    # "Simulator": a session leader whose child moves to its own process group,
    # which killpg on the leader's group would miss.
    sim = subprocess.Popen(
        ['bash', '-c', '(set -m; sleep 60 & echo $! > /dev/stdout; wait) ; sleep 60'],
        stdout=subprocess.PIPE, text=True, preexec_fn=_child_preexec)
    child = int(sim.stdout.readline())
    try:
        assert os.getsid(child) == sim.pid
        assert os.getpgid(child) != sim.pid       # moved out of the leader's group
        assert _kill_session(sim.pid) >= 2
        deadline = time.time() + 5
        while time.time() < deadline and (_alive(child) or sim.poll() is None):
            time.sleep(0.05)
        assert not _alive(child)
        assert sim.poll() is not None
        assert _alive(decoy.pid)
    finally:
        decoy.kill()
        for p in (sim,):
            if p.poll() is None:
                p.kill()
        try:
            os.kill(child, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_refuses_init_and_empty():
    assert _kill_session(0) == 0
    assert _kill_session(1) == 0
