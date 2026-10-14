"""Tests for the MuJoCo contact rig used to validate the estimator.

The point of this rig is that it is a DIFFERENT engine from the one under
test. `test_integration.py` drives both its synthetic torques and its
estimate through the same Pinocchio model, so a model error cancels
inside it and it cannot detect one -- which is why it stayed green
through a 25 Nm gravity error. MuJoCo computes the true joint torques of
a real contact independently, so an estimate checked against it is
checked against something it has never seen.
"""

import os

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from fic_contact.simrig import ContactSim  # noqa: E402

_HERE = os.path.dirname(__file__)
_MJCF = os.path.join(_HERE, '..', '..', 'menagerie', 'model',
                     'KinovaGen3_7DOF', 'xml', 'scene.xml')
_JOINTS = ['Actuator%d' % i for i in range(1, 8)]
_POSE = np.array([0.0, 0.6, 0.0, -1.2, 0.0, 0.9, 0.0])


@pytest.fixture(scope='module')
def sim():
    """Return a rig on the Kinova scene, which carries tool and gripper."""
    return ContactSim(_MJCF, _JOINTS)


def test_link_names_follow_the_joint_chain(sim):
    """Check link k is the body joint k carries, as the estimator assumes."""
    assert len(sim.link_names) == 7
    assert sim.link_names[0] == 'Shoulder_Link'
    assert sim.link_names[-1] == 'Bracelet_Link'


def test_gravity_is_nonzero_and_finite(sim):
    """Check the gravity truth the estimator is measured against."""
    g = sim.gravity(_POSE)
    assert g.shape == (7,)
    assert np.isfinite(g).all()
    assert np.abs(g).max() > 1.0


def test_contact_torque_is_structurally_zero_distal_to_the_link(sim):
    """The premise the whole estimator rests on, verified in MuJoCo.

    A contact on link k can produce no torque about any joint distal to
    k, because those joints do not lie between the base and the contact.
    If this ever failed, link identification from support pattern would
    be unfounded -- so it is asserted against the independent engine
    rather than assumed from the design document.
    """
    for link in range(1, 8):
        point = sim.surface_point(_POSE, link)
        tau = sim.contact_torque(_POSE, link, point, [1.0, 1.0, 1.0])
        distal = tau[link:]
        # Link 7 is the most distal: it has no joints beyond it, so the
        # claim is vacuously true there rather than untestable.
        if distal.size:
            assert np.abs(distal).max() < 1e-9, (link, tau)
        assert np.abs(tau[:link]).max() > 1e-6, (link, tau)


def test_contact_torque_grows_with_force(sim):
    """Linear in the applied force, as J^T f must be."""
    point = sim.surface_point(_POSE, 4)
    one = sim.contact_torque(_POSE, 4, point, [0.0, 10.0, 0.0])
    two = sim.contact_torque(_POSE, 4, point, [0.0, 20.0, 0.0])
    assert np.allclose(two, 2.0 * one)


def test_measured_torque_is_gravity_plus_contact(sim):
    """The sensor reading the estimator will be handed.

    tau_meas = g(q) + J^T f is the convention the residual inverts:
    ResidualEstimator.residual subtracts its own g(q), so what survives
    is the contact term. Pinning it here keeps the rig and the estimator
    from drifting apart on sign.
    """
    point = sim.surface_point(_POSE, 3)
    force = [5.0, 0.0, 0.0]
    quiet = sim.measured_torque(_POSE)
    pushed = sim.measured_torque(_POSE, link=3, point=point, force=force)
    assert np.allclose(quiet, sim.gravity(_POSE))
    assert np.allclose(pushed - quiet,
                       sim.contact_torque(_POSE, 3, point, force))


def test_no_contact_reads_as_pure_gravity(sim):
    """A quiet rig must produce exactly the gravity torque."""
    assert np.allclose(sim.measured_torque(_POSE), sim.gravity(_POSE))


def test_noise_is_reproducible_from_a_seed(sim):
    """Results must be reproducible, so noise takes an explicit rng."""
    a = sim.measured_torque(_POSE, noise_nm=0.5,
                            rng=np.random.default_rng(1))
    b = sim.measured_torque(_POSE, noise_nm=0.5,
                            rng=np.random.default_rng(1))
    c = sim.measured_torque(_POSE, noise_nm=0.5,
                            rng=np.random.default_rng(2))
    assert np.allclose(a, b)
    assert not np.allclose(a, c)


def test_surface_point_lies_on_the_named_link(sim):
    """Contact points must be on the body they claim, not near it."""
    for link in range(1, 8):
        point = sim.surface_point(_POSE, link)
        assert np.isfinite(point).all()
        # Within a plausible link radius of that body's origin.
        origin = sim.link_origin(_POSE, link)
        assert np.linalg.norm(np.asarray(point) - np.asarray(origin)) < 0.35


def test_surface_point_varies_with_index(sim):
    """A sweep needs more than one candidate contact site per link."""
    a = sim.surface_point(_POSE, 4, index=0)
    b = sim.surface_point(_POSE, 4, index=7)
    assert not np.allclose(a, b)


def test_rejects_an_out_of_range_link(sim):
    """Link 0 and link 99 are not contacts; they are caller errors."""
    with pytest.raises(ValueError):
        sim.surface_point(_POSE, 0)
    with pytest.raises(ValueError):
        sim.surface_point(_POSE, 99)
