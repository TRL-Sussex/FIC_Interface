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
