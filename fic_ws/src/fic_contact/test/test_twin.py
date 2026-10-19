"""Tests for the MuJoCo twin's pure helpers: geometry maps and highlighting.

``fic_contact.twin`` imports ``rclpy`` and ``communication.msg`` at module
level, so importing it needs the ROS workspace sourced (``colcon build`` +
``install/setup.bash``) even though nothing here touches a ROS graph or a
display. ``pytest.importorskip`` turns a missing ``communication`` package
into a skip instead of a collection error.
"""

import os
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from fic_contact.residual import LINK_NAMES

twin = pytest.importorskip(
    'fic_contact.twin',
    reason='fic_contact.twin needs the communication ROS package built '
           'and sourced')

_MJCF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'KinovaGen3_7DOF', 'xml', 'scene.xml')
_JOINTS = ['Actuator%d' % i for i in range(1, 8)]


@pytest.fixture(scope='module')
def model():
    """The Kinova scene: nq = 13 (7 arm joints + 6 Robotiq gripper joints)."""
    return mujoco.MjModel.from_xml_path(_MJCF)


def test_link_geom_map_covers_all_seven_links(model):
    """Every Kinova link resolves to exactly one geom."""
    geoms = twin._link_geom_map(model, LINK_NAMES)
    assert sorted(geoms) == list(range(1, 8))
    for gids in geoms.values():
        assert len(gids) == 1


def test_link_geom_map_skips_a_name_not_in_the_model(model):
    """An unknown body name is skipped, not raised -- see the docstring."""
    geoms = twin._link_geom_map(model, ['not_a_real_body'] + list(LINK_NAMES))
    assert 1 not in geoms
    assert sorted(geoms) == list(range(2, 9))


def test_joint_qpos_map_resolves_by_name_not_position(model):
    """The first seven qpos slots belong to Actuator1..7, by name lookup."""
    adr = twin._joint_qpos_map(model, _JOINTS)
    np.testing.assert_array_equal(adr, np.arange(7))


def test_joint_qpos_map_raises_on_unknown_joint(model):
    """A typo'd or robot-mismatched joint name must fail loudly."""
    with pytest.raises(ValueError, match='not_a_real_joint'):
        twin._joint_qpos_map(model, ['not_a_real_joint'])


def test_highlight_then_restore_is_bit_exact(model):
    """Painting a link hot and restoring must return the original colours."""
    geoms = twin._link_geom_map(model, LINK_NAMES)
    original = model.geom_rgba.copy()
    gid = geoms[4][0]

    model.geom_rgba[gid] = twin.HOT_RGBA
    assert np.array_equal(model.geom_rgba[gid], twin.HOT_RGBA)
    assert not np.array_equal(model.geom_rgba[gid], original[gid])

    model.geom_rgba[:] = original
    assert np.array_equal(model.geom_rgba, original)


def test_only_link_index_at_least_one_selects_geoms(model):
    """Mirrors the viewer loop's lookup: -1 and 0 must highlight nothing.

    ``ContactEstimate.link_index`` uses -1 for "no statement possible" and
    0 for genuine no-contact; only 1..n names an actual contacted link.
    The viewer loop keys highlighting off ``geoms.get(link, [])``, and
    since ``geoms`` only ever has entries for 1..n, that single lookup is
    what makes -1 and 0 both highlight nothing.
    """
    geoms = twin._link_geom_map(model, LINK_NAMES)
    assert geoms.get(-1, []) == []
    assert geoms.get(0, []) == []
    assert geoms.get(1, []) != []


def test_module_runs_main_when_executed_with_dash_m():
    """``python -m fic_contact.twin`` must actually start the viewer.

    fic_studio launches the twin as ``python3 -m fic_contact.twin``, so a
    module without a ``__main__`` guard imports its definitions and exits
    0 without ever calling ``main()`` -- the viewer never opens and the
    log stays empty, which reads like a crash with no traceback. Asking
    for ``--help`` is the cheapest proof that argument parsing, and
    therefore ``main()``, was reached.
    """
    proc = subprocess.run(
        [sys.executable, '-m', 'fic_contact.twin', '--help'],
        capture_output=True, text=True, timeout=60)
    assert 'usage' in proc.stdout.lower(), (
        'main() never ran under -m; stdout was %r' % proc.stdout)
    assert '--xml' in proc.stdout


_FRANKA_MJCF = os.path.join(
    os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
    'Franka_FR3', 'xml', 'scene.xml')


def test_derived_link_names_reproduce_the_kinova_set(model):
    """Deriving from the MJCF must agree with residual.LINK_NAMES.

    ``contact_node`` numbers links by ``ArmModel.link_names``, which
    Pinocchio derives from the URDF; the twin numbers them by whatever
    ``--links`` holds. The two must produce the same order or link 3 in
    the message highlights a different body than link 3 on the real arm.
    """
    derived = twin._derive_link_names(model, _JOINTS)
    assert derived == list(LINK_NAMES)


def test_derived_link_names_highlight_a_non_kinova_arm():
    """A non-Kinova MJCF must still map geoms, not silently map none.

    ``_link_geom_map`` skips names absent from the model rather than
    raising, so the hardcoded Kinova default yields an empty map on any
    other arm -- the twin mirrors the pose and never highlights, with
    nothing in the log to say why.
    """
    m = mujoco.MjModel.from_xml_path(_FRANKA_MJCF)
    joints = ['fr3_joint%d' % i for i in range(1, 8)]
    derived = twin._derive_link_names(m, joints)
    assert twin._link_geom_map(m, LINK_NAMES) == {}, \
        'premise: the hardcoded Kinova names match nothing here'
    assert len(twin._link_geom_map(m, derived)) == 7


def test_pose_prefers_raw_over_filtered():
    """The twin must mirror the unfiltered joint positions when it has them.

    ``fic_control`` low-passes a wrapped angle, so every +-pi crossing
    leaves ``{rid}_q_current`` sweeping the whole circle over ~35 ms --
    measured at 275 deg in 35 ms, which the twin renders as a spin the
    real arm never made. The raw topic keeps the wrap an instantaneous
    2*pi step, and a hinge at -pi and +pi is the same pose, so it draws
    identically and the artifact disappears.
    """
    assert twin._select_pose([1.0], [2.0]) == [1.0]


def test_pose_falls_back_to_filtered_before_any_raw_sample():
    """A twin run against a hardware node without the raw topic still works.

    fic_control is C++ and only publishes {rid}_q_current_raw once
    rebuilt; until then the twin must mirror the filtered pose rather
    than freeze at its start configuration.
    """
    assert twin._select_pose(None, [2.0]) == [2.0]
    assert twin._select_pose(None, None) is None
