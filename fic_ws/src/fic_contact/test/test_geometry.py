"""Tests for per-link contact candidate extraction from the MJCF."""

import os

import mujoco
import numpy as np
import pytest

from fic_contact.geometry import load_candidates
from fic_contact.residual import LINK_NAMES

_MJCF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'KinovaGen3_7DOF', 'xml', 'scene.xml')


@pytest.fixture(scope='module')
def candidates():
    """Candidates for all seven Kinova links."""
    return load_candidates(_MJCF, LINK_NAMES, n_per_link=150)


def test_every_link_has_candidates(candidates):
    """All seven moving links must be represented."""
    assert sorted(candidates) == list(range(1, 8))


def test_candidate_counts_are_capped(candidates):
    """Decimation must respect n_per_link."""
    for cand in candidates.values():
        assert 0 < cand.points.shape[0] <= 150
        assert cand.points.shape == cand.normals.shape


def test_normals_are_unit_length(candidates):
    """Surface normals must be normalised for the push constraint."""
    for cand in candidates.values():
        norms = np.linalg.norm(cand.normals, axis=1)
        assert np.allclose(norms, 1.0, atol=1e-5)


def test_candidates_are_within_arm_scale(candidates):
    """Body-frame points must be centimetres-to-decimetres, not metres away."""
    for cand in candidates.values():
        assert np.max(np.abs(cand.points)) < 0.5


def test_unknown_link_name_is_rejected():
    """A body that is not in the model must fail loudly."""
    with pytest.raises(ValueError, match='not found'):
        load_candidates(_MJCF, ('NoSuchLink',), n_per_link=10)


def test_normals_are_paired_with_the_matching_vertex():
    """A stored normal must belong to its same-index vertex, not a shuffle.

    Independently reconstructs each mesh face's geometric normal from the
    vertex winding (``cross(v1 - v0, v2 - v0)``) and checks it agrees with
    the average of the *stored* normals at that face's three vertices. A
    large ``n_per_link`` disables decimation, so candidate index ``i``
    still corresponds to raw mesh vertex/normal index ``i`` and the mesh's
    own face indices can be used directly against the candidate arrays.
    """
    cand = load_candidates(_MJCF, ('ForeArm_Link',), n_per_link=100_000)[1]

    model = mujoco.MjModel.from_xml_path(_MJCF)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'ForeArm_Link')
    geom = model.body_geomadr[body]
    mesh = model.geom_dataid[geom]
    f0 = model.mesh_faceadr[mesh]
    faces = model.mesh_face[f0:f0 + model.mesh_facenum[mesh]]

    v_a, v_b, v_c = (cand.points[faces[:, i]] for i in range(3))
    face_normal = np.cross(v_b - v_a, v_c - v_a)
    face_normal /= np.linalg.norm(face_normal, axis=1, keepdims=True)

    stored = sum(cand.normals[faces[:, i]] for i in range(3)) / 3.0
    stored /= np.linalg.norm(stored, axis=1, keepdims=True)

    assert np.mean(np.sum(face_normal * stored, axis=1)) > 0.8


def test_points_are_in_the_body_frame():
    """The geom's translation must actually be applied, not just its rotation.

    If ``model.geom_pos[geom] +`` were dropped from the transform, the
    candidate centroid would sit near the body origin instead of near the
    geom's own offset. ``ForeArm_Link``'s single geom sits about 8 cm from
    the body origin, so the two are easy to tell apart.
    """
    model = mujoco.MjModel.from_xml_path(_MJCF)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'ForeArm_Link')
    geom_pos = model.geom_pos[model.body_geomadr[body]]

    cand = load_candidates(_MJCF, ('ForeArm_Link',), n_per_link=100_000)[1]
    centroid = cand.points.mean(axis=0)

    assert np.linalg.norm(centroid - geom_pos) < 0.01
    assert np.linalg.norm(centroid) > 0.03
