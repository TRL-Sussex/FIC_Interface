"""Integration test: the real ``ContactNode`` on a real ROS graph.

Everything else in ``test/`` exercises pure helpers with no ROS graph at
all (see ``test_purity.py``). This module is the exception: it runs the
actual node, publishes the topics ``fic_hardware`` publishes, calls the
real ``calibrate`` service, and synthesises a physically-consistent push
via ``J_p(q)^T F`` on a chosen link -- then asserts the node names that
link back on ``{rid}_contact_estimate``.

Adapted from a stand-alone harness written while diagnosing the ``ContactNode`` cwd-dependent config/URDF/
MJCF path bug (see ``contact_node.py``'s ``_resolve_repo_root`` and
``_first_existing``) -- 106 unit tests missed that bug because none of
them ran the node for real. This module is what closes that gap
permanently.

Like ``test_twin.py``, ``fic_contact.contact_node`` imports ``rclpy`` and
``communication.msg`` at module level, so importing it needs the ROS
workspace built and sourced (``colcon build`` + ``install/setup.bash``)
even though nothing here touches a display. ``pytest.importorskip``
turns a missing ``rclpy``/``communication`` into a skip instead of a
collection error -- this must not break the documented headless test
command, which does not source the workspace.

Links 1 and 2 are deliberately excluded from the push coverage below:
their torque signature at the calibration pose is too close to every
other link's for ``support_link``/``fit_contact`` to separate reliably
(near-undetectable, not a test bug) -- only links 3..7 are covered.
"""

import os
import threading
import time

import numpy as np
import pytest

contact_node_mod = pytest.importorskip(
    'fic_contact.contact_node',
    reason='fic_contact.contact_node needs rclpy and the communication '
           'ROS package built and sourced')

import rclpy  # noqa: E402
from communication.msg import ContactEstimate  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from std_msgs.msg import Float64MultiArray, Int32  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from fic_contact.geometry import load_candidates  # noqa: E402
from fic_contact.residual import ArmModel  # noqa: E402

ContactNode = contact_node_mod.ContactNode

_HERE = os.path.dirname(__file__)
_URDF = os.path.join(
    _HERE, '..', '..', 'menagerie', 'model', 'KinovaGen3_7DOF', 'URDF',
    'KinovaGen3_7DOF.urdf')
_MJCF = os.path.join(
    _HERE, '..', '..', 'menagerie', 'model', 'KinovaGen3_7DOF', 'xml', 'scene.xml')
_JOINTS = ['Actuator%d' % i for i in range(1, 8)]
_POSE = np.array([0.0, 0.5, 0.0, 1.5, 0.0, 1.0, 0.0])
_RID = 'robot1'  # matches ContactNode's default station_id=1 ("robot%s" % 1)

#: Must match ContactNode's own n_per_link default (see contact_node.py's
#: ``declare_parameter('n_per_link', 20)``) -- the push below targets one
#: of the node's own decimated surface candidates exactly (see
#: ``FakeArm.push_link``), which only works if both sides sample the
#: mesh identically.
_N_PER_LINK = 20

#: Reliably localizable at _POSE -- see the module docstring for why 1
#: and 2 are excluded.
_TESTABLE_LINKS = (3, 4, 5, 6, 7)


def _skew(r):
    """Return the 3x3 skew-symmetric matrix of vector ``r``."""
    return np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])


def _settle(seconds):
    """Sleep, generously, for the ROS graph to converge on a new state."""
    time.sleep(seconds)


class FakeArm(Node):
    """Publish ``{rid}_q_current``/``joint_torques``/``joint_velocities``/
    ``control_mode`` exactly as ``fic_hardware`` does, so ``ContactNode``
    cannot tell this apart from the real driver.
    """

    def __init__(self, model, candidates):
        """Wire up the publishers and the 100 Hz feed at a fixed pose."""
        super().__init__('fake_arm')
        self.model = model
        self.candidates = candidates
        self.gravity = model.gravity(_POSE)
        self.extra = np.zeros(7)  # the synthetic contact torque
        self.pub_q = self.create_publisher(
            Float64MultiArray, '%s_q_current' % _RID, qos_profile_sensor_data)
        self.pub_tau = self.create_publisher(
            Float64MultiArray, '%s_joint_torques' % _RID,
            qos_profile_sensor_data)
        self.pub_dq = self.create_publisher(
            Float64MultiArray, '%s_joint_velocities' % _RID,
            qos_profile_sensor_data)
        self.pub_mode = self.create_publisher(Int32, '%s_control_mode' % _RID, 10)
        self.create_timer(0.01, self._tick)

    def _tick(self):
        """Emit one sample of the arm's state."""
        self.pub_mode.publish(Int32(data=1))  # Joint Impedance -- stationary
        self.pub_q.publish(Float64MultiArray(data=_POSE.tolist()))
        self.pub_dq.publish(Float64MultiArray(data=[0.0] * 7))
        tau = self.gravity + self.extra
        self.pub_tau.publish(Float64MultiArray(data=tau.tolist()))

    def push_link(self, k, newtons=30.0, candidate_index=0):
        """Set the measured torque to a real push on one of link ``k``'s own
        decimated surface candidates (see ``geometry.load_candidates``).

        Pushing at one of the node's own candidate points/normals -- not
        an arbitrary offset point -- means ``fit_contact``'s exhaustive
        sweep finds an (almost) exact, near-zero-residual match at link
        ``k`` itself, rather than a neighbouring link's candidate that
        merely happens to fit an arbitrary point's torque pattern better.
        An early version of this harness used one fixed generic offset
        for every link and could not reliably separate links 5 and 7
        from 6 -- not because those links are undetectable, but because
        that offset was not an actual point on either link's surface.
        Builds ``tau_ext = J_p(q)^T F`` via the same point-Jacobian shift
        ``hypotheses._point_jacobians`` uses to invert it, with ``F``
        pointing into the surface (opposite the outward normal), exactly
        as ``solve_push_force``'s ``F.n <= 0`` push constraint requires.
        """
        cand = self.candidates[k]
        point = cand.points[candidate_index]
        normal = cand.normals[candidate_index]
        force = -normal * newtons
        jac = self.model.link_jacobians(_POSE)[k]
        jp = jac[:3] - _skew(point) @ jac[3:]
        self.extra = jp.T @ force

    def release(self):
        """Stop pushing."""
        self.extra = np.zeros(7)


class Listener(Node):
    """Collect every ``{rid}_contact_estimate`` the node under test publishes."""

    def __init__(self):
        """Subscribe to the estimate topic."""
        super().__init__('listener')
        self.latest = None
        self.seen = []
        self.create_subscription(
            ContactEstimate, '%s_contact_estimate' % _RID, self._on, 10)

    def _on(self, msg):
        """Record one estimate."""
        self.latest = msg
        self.seen.append(msg.link_index)


@pytest.fixture(scope='module')
def harness():
    """Bring up FakeArm + the real ContactNode + Listener on one ROS graph.

    Module-scoped so calibration (expensive to repeat, and required
    before any push test) happens exactly once, with nobody touching the
    arm, and every test below shares that single calibrated node --
    order matters here and relies on pytest's default top-to-bottom
    collection order within a file (no random-order plugin is active
    under ``PYTEST_DISABLE_PLUGIN_AUTOLOAD=1``).
    """
    rclpy.init()
    model = ArmModel(_URDF, _JOINTS)
    candidates = load_candidates(_MJCF, model.link_names, n_per_link=_N_PER_LINK)
    arm = FakeArm(model, candidates)
    listener = Listener()
    node = ContactNode()  # default 'workspace' param -- see contact_node.py

    executor = MultiThreadedExecutor()
    for n in (arm, listener, node):
        executor.add_node(n)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    _settle(1.5)  # let discovery + the first few 100 Hz samples land

    yield arm, listener, node

    executor.shutdown()
    for n in (arm, listener, node):
        n.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=5.0)


def test_a_before_calibration_reports_no_statement(harness):
    """Before ``calibrate`` is ever called, the node must say -1, not 0.

    -1 ("no statement possible") and 0 ("no contact") are different
    claims -- see ``ContactEstimate``'s own regime -- and an uncalibrated
    node must never be mistaken for one that has actively confirmed no
    contact.
    """
    _, listener, _ = harness
    _settle(1.0)
    assert listener.latest is not None, 'node never published anything'
    assert listener.latest.link_index == -1, (
        'uncalibrated node reported link_index %d, not -1'
        % listener.latest.link_index)


def test_b_calibrated_and_untouched_reports_no_contact(harness):
    """Calibrated + nobody touching: steady 0, with no false positives.

    Checked over many samples (not just the latest one) so a single
    lucky/unlucky tick cannot hide an intermittent false positive.
    """
    arm, listener, _ = harness
    cli = arm.create_client(Trigger, '%s/contact/calibrate' % _RID)
    assert cli.wait_for_service(timeout_sec=5.0), 'calibrate service never came up'
    future = cli.call_async(Trigger.Request())
    _settle(2.5)  # >> calibration_seconds (default 1.0 s)
    assert future.done(), 'calibrate call never completed'
    assert future.result().success, 'calibrate call failed: %r' % future.result().message

    listener.seen.clear()
    _settle(1.5)
    samples = [x for x in listener.seen if x is not None]
    assert len(samples) > 50, 'too few samples collected: %d' % len(samples)
    false_positives = [x for x in samples if x != 0]
    assert not false_positives, (
        'false positive(s) with nobody touching the arm: %r' % false_positives)


@pytest.mark.parametrize('link', _TESTABLE_LINKS)
def test_c_push_on_link_is_reported_as_that_link(harness, link):
    """A synthetic push on link ``k`` must be reported back as link ``k``."""
    arm, listener, _ = harness
    arm.push_link(link, newtons=30.0)
    _settle(0.8)
    got = listener.latest.link_index
    arm.release()
    _settle(0.4)
    assert got == link, (
        'pushing link %d was reported as link_index %d, expected %d'
        % (link, got, link))


def test_d_after_release_returns_to_no_contact(harness):
    """Once nobody is pushing again, the node returns to reporting 0."""
    arm, listener, _ = harness
    arm.release()
    _settle(0.8)
    assert listener.latest.link_index == 0, (
        'after release, node still reports link_index %d, not 0'
        % listener.latest.link_index)
