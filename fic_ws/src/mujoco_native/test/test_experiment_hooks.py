import os, numpy as np, mujoco, pytest
from mujoco_native.experiment_hooks import (ramp_scale, set_wall, wall_contact_force,
                                            apply_link_force, body_positions)
FIX = os.path.join(os.path.dirname(__file__), "fixtures", "box_on_wall.xml")

def _settled():
    m = mujoco.MjModel.from_xml_path(FIX); d = mujoco.MjData(m)
    for _ in range(2000): mujoco.mj_step(m, d)
    return m, d

def test_ramp_scale_profile():
    assert ramp_scale(0.5, 1.0, 1.0, 2.0) == 0.0
    assert ramp_scale(1.5, 1.0, 1.0, 2.0) == pytest.approx(0.5)
    assert ramp_scale(2.5, 1.0, 1.0, 2.0) == 1.0
    assert ramp_scale(4.5, 1.0, 1.0, 2.0) == pytest.approx(0.5)
    assert ramp_scale(9.0, 1.0, 1.0, 2.0) == 0.0

def test_wall_force_equals_box_weight():
    m, d = _settled()
    f = wall_contact_force(m, d, "wall_geom")
    assert f[2] == pytest.approx(-9.81, rel=0.02)   # box pushes the wall down

def test_release_drops_box():
    m, d = _settled()
    z0 = d.xpos[m.body("box").id][2]
    set_wall(m, d, "wall", "wall_geom", False, (0, 0, 0), (1, 0, 0, 0))
    for _ in range(200): mujoco.mj_step(m, d)
    assert d.xpos[m.body("box").id][2] < z0 - 0.1
    assert np.allclose(wall_contact_force(m, d, "wall_geom"), 0.0)

def test_wall_reenable_restores_contact_flags():
    m, d = _settled()
    set_wall(m, d, "wall", "wall_geom", False, (0, 0, 0), (1, 0, 0, 0))
    set_wall(m, d, "wall", "wall_geom", True, (0, 0, 0), (1, 0, 0, 0))
    g = m.geom("wall_geom").id
    assert m.geom_contype[g] == 2 and m.geom_conaffinity[g] == 2

def test_apply_link_force_moves_body():
    m, d = _settled()
    apply_link_force(m, d, "box", (5.0, 0, 0), d.xpos[m.body("box").id].copy(), 1.0)
    assert np.allclose(d.xfrc_applied[m.body("box").id][:3], (5.0, 0, 0))
    apply_link_force(m, d, "box", (5.0, 0, 0), d.xpos[m.body("box").id].copy(), 0.0)
    assert np.allclose(d.xfrc_applied[m.body("box").id], 0.0)

def test_apply_link_force_offset_torque():
    m, d = _settled()
    bid = m.body("box").id
    off = np.array([0.0, 0.0, 0.1])
    apply_link_force(m, d, "box", (5.0, 0, 0), d.xipos[bid] + off, 2.0)
    f = np.array([10.0, 0, 0])
    assert np.allclose(d.xfrc_applied[bid][:3], f)
    assert np.allclose(d.xfrc_applied[bid][3:], np.cross(off, f))
    assert np.allclose(d.qfrc_applied, 0.0)

def test_body_positions_shape():
    m, d = _settled()
    p = body_positions(m, d, ["box", "wall"])
    assert p.shape == (2, 3)

_HINGE_XML = """
<mujoco><worldbody>
  <body name="arm" pos="0 0 1">
    <joint name="h" type="hinge" axis="0 0 1"/>
    <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.02"/>
  </body>
</worldbody></mujoco>
"""

def test_applied_wrench_torques_maps_push_to_joint_torque():
    from mujoco_native.experiment_hooks import applied_wrench_torques
    m = mujoco.MjModel.from_xml_string(_HINGE_XML); d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert np.allclose(applied_wrench_torques(m, d), 0.0)
    # 5 N along +y at the tip (0.5 m from the hinge axis): 2.5 N m about +z
    apply_link_force(m, d, "arm", (0.0, 5.0, 0.0), (0.5, 0.0, 1.0), 1.0)
    assert np.allclose(applied_wrench_torques(m, d), [2.5])
    assert np.allclose(d.qfrc_applied, 0.0)

def test_apply_link_force_at_centre_of_mass():
    m, d = _settled()
    bid = m.body("box").id
    apply_link_force(m, d, "box", (5.0, 0, 0), None, 1.0)
    assert np.allclose(d.xfrc_applied[bid], (5.0, 0, 0, 0, 0, 0))


def test_object_poses_name_the_obj_bodies():
    import mujoco
    from mujoco_native.experiment_hooks import object_bodies, object_poses
    m = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body name="obj_cube" pos="0.5 0 0.1"><freejoint/><geom type="box" size="0.02 0.02 0.02"/></body>
      <body name="arm" pos="0 0 0"><geom type="sphere" size="0.05"/></body></worldbody></mujoco>""")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    objs = object_bodies(m)
    assert [n for n, _ in objs] == ['obj_cube']
    assert object_poses(d, objs) == {'cube': [0.5, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0]}
