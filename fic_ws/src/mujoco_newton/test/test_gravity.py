"""mujoco_newton publishes true gravity, not the actuator command.

The old recipe (mj_forward, then mj_inverse with qvel/qacc zeroed) returned the
command itself, because mj_forward recomputes qacc from ctrl first. The FIC
controller adds this vector to its output, so that was positive feedback.
"""
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
from mujoco_newton.simulation_node import gravity_from_qpos  # noqa: E402

MODEL = (Path(__file__).resolve().parents[2] / "menagerie" / "model"
         / "KinovaGen3_7DOF" / "xml" / "KinovaGen3_7DOF.xml")


def _old_recipe(m, d):
    d.qvel[:] = 0.0
    d.qacc[:] = 0.0
    mujoco.mj_forward(m, d)
    old = int(m.opt.disableflags)
    m.opt.disableflags = (old | int(mujoco.mjtDisableBit.mjDSBL_DAMPER)
                          | int(mujoco.mjtDisableBit.mjDSBL_SPRING)
                          | int(mujoco.mjtDisableBit.mjDSBL_FRICTIONLOSS))
    mujoco.mj_inverse(m, d)
    m.opt.disableflags = old
    return d.qfrc_inverse[:m.nv].copy()


@pytest.mark.skipif(not MODEL.exists(), reason="Kinova model not in the workspace")
def test_gravity_ignores_the_command():
    m = mujoco.MjModel.from_xml_path(str(MODEL))
    q = np.linspace(-0.6, 0.6, m.nq)
    stepped = mujoco.MjData(m)
    stepped.qpos[:] = q
    stepped.ctrl[:] = np.linspace(-15.0, 15.0, m.nu)   # a large command

    # Reference g(q): bias force at zero velocity on an independent MjData.
    ref = mujoco.MjData(m)
    ref.qpos[:] = q
    mujoco.mj_forward(m, ref)
    g_ref = ref.qfrc_bias[:m.nv].copy()

    scratch = mujoco.MjData(m)
    g = np.array(gravity_from_qpos(m, scratch, stepped.qpos.copy(), m.nv))
    assert np.allclose(g, g_ref, atol=1e-9)
    assert np.any(np.abs(g) > 1.0)                    # the arm does feel gravity

    # The stepped data is not touched.
    assert np.allclose(stepped.ctrl, np.linspace(-15.0, 15.0, m.nu))

    # The old recipe was off by roughly the command (documents the bug).
    old = _old_recipe(m, stepped)
    assert np.max(np.abs(old - g_ref)) > 5.0
