# ros_pump.py — ROS callbacks on the Qt thread, without idling on it
"""
ROS callbacks on the Qt thread, without idling on it.

The Studio runs its ROS callbacks on the Qt main thread (they touch widgets),
from a 10 ms QTimer. Each tick runs what is ready *now* and returns: it used to
call ``spin_once(timeout_sec=0)`` in a loop for 8 ms whatever had arrived,
which kept the main thread ~80 % busy doing nothing, and the web UI's commands,
dispatched on the same thread, waited behind it (seconds on the MCU board).
"""
import time

from rclpy.executors import ShutdownException, TimeoutException

#: Timer interval while callbacks are arriving, and while none are: an idle pass
#: still rebuilds rclpy's wait set (~3 ms on the MCU board), so poll less then.
BUSY_MS = 5
IDLE_MS = 25


def next_interval_ms(ran):
    """Return the pump timer's next interval after a pass that ran ``ran`` callbacks."""
    return BUSY_MS if ran else IDLE_MS


def pace(executor, stop, period_s, budget_s=None):
    """Read what is ready every ``period_s`` until ``stop`` (a threading.Event) is set.

    For a thread of its own (the Studio's state node): with depth-1
    subscriptions each pass takes only the newest message of each topic, so a
    1 kHz stream costs ``1 / period_s`` callbacks a second, not 1000. A
    callback's exception is printed and the loop goes on.
    """
    import traceback
    budget = period_s / 2.0 if budget_s is None else budget_s
    while not stop.is_set():
        t0 = time.monotonic()
        try:
            drain(executor, budget)
        except ShutdownException:
            return
        except Exception:                       # a bad message must not end the state thread
            traceback.print_exc()
        stop.wait(max(0.0, period_s - (time.monotonic() - t0)))


def drain(executor, budget_s=0.008, clock=time.monotonic):
    """Run the callbacks ready now, for at most ``budget_s``; return how many ran.

    Returns as soon as nothing is ready. A callback's exception is raised, as
    ``spin_once`` does.
    """
    deadline = clock() + budget_s
    ran = 0
    while clock() < deadline:
        try:
            handler, _entity, _node = executor.wait_for_ready_callbacks(timeout_sec=0)
        except (TimeoutException, ShutdownException):
            break
        handler()
        if handler.exception() is not None:
            raise handler.exception()
        ran += 1
    return ran
