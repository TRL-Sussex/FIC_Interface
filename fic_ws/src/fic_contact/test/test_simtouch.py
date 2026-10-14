"""Tests for the interactive rig, driven headlessly.

The mouse cannot be automated here, so these exercise the cycle the
viewer loop calls -- `SimTouch.step` with a synthetic perturbation --
and check the two things that would silently break the rig: that a push
is attributed to the right link, and that the arm deflects the way the
impedance law says, since both estimator channels are read off that.
"""

import os

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
rclpy = pytest.importorskip('rclpy')

_HERE = os.path.dirname(__file__)
_MJCF = os.path.join(_HERE, '..', '..', 'menagerie', 'model',
                     'KinovaGen3_7DOF', 'xml', 'scene.xml')


@pytest.fixture(scope='module')
def node():
    """Bring up a SimTouch on an isolated ROS context."""
    from fic_contact.simtouch import SimTouch
    rclpy.init(args=['--ros-args', '-p', 'mjcf:=%s' % _MJCF,
                     '-p', 'station_id:=9', '-p', 'headless:=true',
                     '-p', 'torque_noise_nm:=0.0',
                     '-p', 'encoder_noise_rad:=0.0'])
    instance = SimTouch()
    yield instance
    instance.destroy_node()
    rclpy.try_shutdown()


def test_quiet_cycle_is_pure_gravity_and_no_contact(node):
    """With nobody pushing, the rig must report link 0 and no deflection."""
    out = node.step()
    assert out['link'] == 0
    assert np.allclose(out['tau_ext'], 0.0)
    assert np.allclose(out['q'], node._q_target)


def test_push_is_attributed_to_the_link_that_carries_it(node):
    """A push on a link's own body is that link."""
    for link in range(1, node._n + 1):
        body = node._bodies[link - 1]
        point = node.link_point(body)
        out = node.step(body=body, point=point, force=[20.0, 0.0, 0.0])
        assert out['link'] == link, (link, out['link'])


def test_push_on_a_gripper_body_is_attributed_to_the_last_link(node):
    """A finger is not an actuated link; the wrist that carries it is.

    Without this the rig would report link 0 for a perfectly real push on
    the gripper, and the estimate would be scored against nothing.
    """
    finger = mujoco.mj_name2id(
        node._model, mujoco.mjtObj.mjOBJ_BODY, 'left_inner_finger')
    if finger < 0:
        pytest.skip('this MJCF has no Robotiq finger bodies')
    point = node.link_point(finger)
    out = node.step(body=finger, point=point, force=[10.0, 0.0, 0.0])
    assert out['link'] == node._n


def test_contact_torque_is_zero_distal_to_the_pushed_link(node):
    """The estimator's premise, asserted through the rig's own path."""
    link = 3
    body = node._bodies[link - 1]
    point = node.link_point(body)
    out = node.step(body=body, point=point, force=[0.0, 25.0, 0.0])
    assert np.abs(out['tau_ext'][link:]).max() < 1e-9
    assert np.abs(out['tau_ext'][:link]).max() > 1e-6


def test_arm_deflects_by_the_impedance_law(node):
    """Check q = q_target + tau_ext / K, what both channels read."""
    body = node._bodies[3]
    point = node.link_point(body)
    out = node.step(body=body, point=point, force=[0.0, 30.0, 0.0])
    expected = node._q_target + out['tau_ext'] / node._k
    assert np.allclose(out['q'], expected)
    assert not np.allclose(out['q'], node._q_target)


def test_deflection_channel_recovers_the_push_from_the_rig(node):
    """The rig must be consistent with the deflection estimator.

    If these disagree the rig is not a valid test bed for the fused
    estimator, however plausible its torques look.
    """
    from fic_contact.deflection import tau_ext_from_deflection
    body = node._bodies[4]
    point = node.link_point(body)
    out = node.step(body=body, point=point, force=[0.0, 0.0, -25.0])
    recovered = tau_ext_from_deflection(node._q_target, out['q'], node._k)
    assert np.allclose(recovered, out['tau_ext'])
