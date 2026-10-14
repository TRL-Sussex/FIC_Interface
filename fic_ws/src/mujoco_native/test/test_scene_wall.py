import os

import mujoco
import numpy as np

from mujoco_native.experiment_hooks import set_wall, wall_contact_force

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "menagerie",
                   "model", "KinovaGen3_7DOF", "xml", "scene_wall.xml")


def test_scene_loads_and_probe_touches_raised_wall():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    probe = d.geom_xpos[m.geom("probe").id].copy()
    # wall 2 mm into the probe from below
    set_wall(m, d, "wall", "wall_geom", True,
             (probe[0], probe[1], probe[2] - 0.01 - 0.005 + 0.002), (1, 0, 0, 0))
    mujoco.mj_forward(m, d)
    assert np.linalg.norm(wall_contact_force(m, d, "wall_geom")) > 0.0


def test_gripper_geoms_do_not_collide():
    m = mujoco.MjModel.from_xml_path(XML)
    base = m.body("robotiq_85_base_link").id
    n = 0
    for g in range(m.ngeom):
        b = m.geom_bodyid[g]
        while b != 0 and b != base:
            b = m.body_parentid[b]
        if b == base:
            n += 1
            assert m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0
    assert n > 0


def test_wall_cannot_collide_with_arm_links():
    m = mujoco.MjModel.from_xml_path(XML)
    w = m.geom("wall_geom").id
    for g in range(m.ngeom):
        if g in (w, m.geom("probe").id):
            continue
        assert not (m.geom_contype[w] & m.geom_conaffinity[g])
        assert not (m.geom_contype[g] & m.geom_conaffinity[w])


def test_push_switching_body_clears_old_body():
    from mujoco_native.experiment_hooks import step_push
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mk = lambda b: dict(body=b, force=[0, 10, 0], point=[0.3, 0, 0.4],
                        ramp_s=0.0, hold_s=1.0, t0=None)
    p1 = mk("ForeArm_Link")
    p1, active, truth = step_push(m, d, p1, None)
    assert p1["t0"] == d.time and truth is not None
    b1, b2 = m.body("ForeArm_Link").id, m.body("HalfArm2_Link").id
    assert np.abs(d.xfrc_applied[b1]).sum() > 0
    p2, active, truth = step_push(m, d, mk("HalfArm2_Link"), active)
    assert active == "HalfArm2_Link"
    assert np.abs(d.xfrc_applied[b1]).sum() == 0
    assert np.abs(d.xfrc_applied[b2]).sum() > 0
    # ending / dropping clears the active body too
    _, active, _ = step_push(m, d, None, active)
    assert active is None and np.abs(d.xfrc_applied).sum() == 0


def test_push_ramp_and_end_paths():
    from mujoco_native.experiment_hooks import step_push
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    b = m.body("ForeArm_Link").id
    p = dict(body="ForeArm_Link", force=[0, 10, 0], point=[0.3, 0, 0.4],
             ramp_s=1.0, hold_s=1.0, t0=None)
    d.time = 5.0
    p, active, truth = step_push(m, d, p, None)
    assert p["t0"] == 5.0 and abs(truth[3]) < 1e-9
    d.time = 5.5
    p, active, truth = step_push(m, d, p, active)
    assert abs(truth[3] - 5.0) < 1e-6 and abs(d.xfrc_applied[b][1] - 5.0) < 1e-6
    d.time = 6.5   # hold
    p, active, truth = step_push(m, d, p, active)
    assert abs(truth[3] - 10.0) < 1e-6
    d.time = 8.1   # past t0 + 2*ramp + hold = 8.0
    p, active, truth = step_push(m, d, p, active)
    assert p is None and active is None and truth is None
    assert np.abs(d.xfrc_applied[b]).sum() == 0
