"""The real ContactNode must not absorb a push into a calibration window.

Hardware session 2026-09-13: auto-calibration opened a window at rest, a
push began 0.5 s into it, and the window averaged the push in. It came out
with thresholds of [13.1, 8.2, 6.3, 2.2, ...] Nm -- a clean window gives
about [0.25, 0.5, 0.2, 0.2, ...] -- and because the noise is pooled over
eight windows the estimator stayed blind for ~28 s. The Following
extension could not detect a push or its release for the rest of the run.

These tests drive the node's own ``_tick`` with synthetic feedback, so
they need rclpy and the workspace sourced, like test_integration.py. They
run on their own ROS domain and station, and never spin: nothing they
publish can reach a live robot's graph. (test_integration.py does publish
robot1 topics on the default domain; do not run that one beside a live
arm.)
"""

import numpy as np
import pytest

contact_node_mod = pytest.importorskip(
    'fic_contact.contact_node',
    reason='fic_contact.contact_node needs rclpy and the communication '
           'ROS package built and sourced')

import rclpy  # noqa: E402

#: Well away from the default domain 0 a live stack runs on.
_ISOLATED_DOMAIN = 87

_Q0 = np.array([0.003, 0.251, -3.139, -2.263, -0.017, 1.032, 1.568])


@pytest.fixture(scope='module')
def node():
    """One real ContactNode, isolated by domain and station."""
    rclpy.init(args=['--ros-args', '-p', 'station_id:=99',
                     '-p', 'auto_calibrate:=false'],
               domain_id=_ISOLATED_DOMAIN)
    made = contact_node_mod.ContactNode()
    made._mode = 5
    made._gate.set_mode(5)
    yield made
    made.destroy_node()
    rclpy.try_shutdown()


def _feed(node, q, extra=None, dq=None, rng=None, noise=0.03):
    """Set feedback: gravity at q, plus any external torque and noise."""
    tau = node._model.gravity(q).copy()
    if extra is not None:
        tau = tau + np.asarray(extra, dtype=float)
    if rng is not None:
        tau = tau + rng.normal(0.0, noise, tau.shape)
    node._q = np.asarray(q, dtype=float)
    node._tau = tau
    node._dq = np.zeros(7) if dq is None else np.asarray(dq, dtype=float)


def _calibrate_clean(node, rng):
    """Run one undisturbed window to completion and return its calibration."""
    _feed(node, _Q0, rng=rng)
    node._begin_calibration()
    while node._calibrating:
        _feed(node, _Q0, rng=rng)
        node._tick()
    return node._est.calibration


def test_a_clean_window_is_adopted(node):
    rng = np.random.default_rng(0)
    cal = _calibrate_clean(node, rng)
    assert cal is not None
    assert np.all(cal.thresholds < 1.0), cal.thresholds


def test_a_push_arriving_mid_window_is_not_absorbed(node):
    """The arm moves under the push: the window must be discarded at once."""
    rng = np.random.default_rng(1)
    clean = _calibrate_clean(node, rng)
    node._begin_calibration()
    for _ in range(30):                       # 0.3 s of quiet window
        _feed(node, _Q0, rng=rng)
        node._tick()
    push = np.array([10.0, 4.0, 8.0, -2.0, -1.5, -0.5, 0.0])
    for i in range(1, 20):                    # the push arrives
        q = _Q0 + np.array([0.004, 0.0, 0.003, 0.0, 0.0, 0.0, 0.0]) * i
        _feed(node, q, extra=push * min(1.0, i / 10.0),
              dq=np.array([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0]), rng=rng)
        node._tick()
    assert node._calibrating is False
    assert node._est.calibration is clean
    assert np.all(node._est.calibration.thresholds < 1.0)


def test_a_window_with_a_still_but_loaded_arm_is_discarded_by_its_noise(node):
    """No motion to catch -- the noise check is what throws it away.

    A push ramping on while the joints happen not to move (a very stiff
    hold, or a stuck encoder) passes the motion check; its variation in
    the torque still gives it away when the window closes.
    """
    rng = np.random.default_rng(2)
    for _ in range(3):
        clean = _calibrate_clean(node, rng)
    node._begin_calibration()
    ticks = 0
    while node._calibrating:
        ramp = np.array([20.0, 8.0, 18.0, 1.5, 3.0, 0.3, 0.1]) * min(
            1.0, ticks / 100.0)
        _feed(node, _Q0, extra=ramp, rng=rng)
        node._tick()
        ticks += 1
    assert node._est.calibration is clean
    assert node._est.last_discard is not None


def test_the_stale_tolerance_parameter_is_read_live(node):
    """Following widens it while it samples a push held deflected."""
    from rclpy.parameter import Parameter
    rng = np.random.default_rng(3)
    _calibrate_clean(node, rng)
    # The shoulder: a base turn no longer counts (see the test below).
    deflected = _Q0 + np.array([0.0, 0.08, 0.0, 0.0, 0.0, 0.0, 0.0])
    _feed(node, deflected, rng=rng)
    node._tick()
    assert 'stale' in (node._gate.blocked_reason() or '')
    node.set_parameters([Parameter('stale_pose_tolerance', value=0.15)])
    try:
        node._tick()
        assert node._gate.blocked_reason() is None
    finally:
        node.set_parameters([Parameter('stale_pose_tolerance', value=0.05)])


def test_a_turn_of_the_base_leaves_the_estimate_live(node):
    """Gravity does not depend on the base joint, so its bias stays valid.

    Counting it made every turn of the Following page block the estimate
    ('calibration stale') until a fresh calibration completed.
    """
    rng = np.random.default_rng(4)
    _calibrate_clean(node, rng)
    turned = _Q0 + np.array([1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    _feed(node, turned, rng=rng)
    node._tick()
    assert node._gate.blocked_reason() is None
