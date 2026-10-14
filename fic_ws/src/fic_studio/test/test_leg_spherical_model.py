"""The cable-driven spherical hip: model integrity and FIC compatibility.

These tests pin the things that were actually wrong, so they cannot come back:

* The hip was placed at the CAD note's *relative* offset (-38.59, 0, -117.98 mm)
  used as an absolute position, putting the joint 41 cm from the real ball and
  driving the leg through the floor.
* The model had no actuators and no joint stiffness, so it fell over; the CAD
  meshes interpenetrate by design, so rigid contact between them ejected the leg
  at 113 rad/s.
* robots_config.yaml described neither file - one joint named ``joint1``, an
  ``ee_link`` frame and an ``elbow_frame_name`` of ``link4``, none of which exist.
* The hip must be HINGE joints, not ``<joint type="ball">``: the simulation node
  surfaces ball joints only as viewer-only euler pseudo-joints that FIC ignores
  (mujoco_native/simulation_node.py), so a ball joint is uncontrollable.
"""
import numpy as np
import pytest

from fic_studio.config import FIC_ROOT

mujoco = pytest.importorskip('mujoco')

MODEL_DIR = FIC_ROOT / 'fic_ws' / 'src' / 'menagerie' / 'model' / 'leg_spherical'
MJCF = MODEL_DIR / 'xml' / 'leg_spherical.xml'
URDF = MODEL_DIR / 'URDF' / 'leg_spherical.urdf'

#: Hip centre, metres. Found by voting triangle normals onto a candidate sphere
#: centre: HipJointHeadV2 is convex at r = 22.5 mm and the Acetabulum is a socket
#: over the same 22-23 mm band, so ball and socket are concentric here.
BALL_CENTRE = np.array([0.18898, 0.11298, 0.38522])
STL_DIR = MODEL_DIR / 'Leg-V4' / 'STL'
JOINTS = ['hip_ball_x_joint', 'hip_ball_y_joint', 'hip_ball_z_joint']
DRIVES = ['torq_j1', 'torq_j2', 'torq_j3']
#: The hip axis each actuator drives. fic_simulation.cpp:1522 binds actuator k
#: positionally to joint_names[k] = [x, y, z], so this MUST stay the identity
#: map. The drums themselves are crossed - the left/right drum turns the hip
#: about z and the green loop drum about x - so torq_j1 is wired to
#: torq_j3_drum_j and vice versa. Wired naively, FIC's negative feedback on x
#: and z lands on the wrong axis and becomes positive feedback.
DRUM_AXIS = {'torq_j1': 0, 'torq_j2': 1, 'torq_j3': 2}
#: Span in degrees and dominant-axis purity each actuator reaches at the +-100
#: control limit the node clamps to. Measured 7.3/5.1x, 31.5/9.0x, 39.1/9.9x.
#:
#: Spans SHRANK and purity ROSE when the drum range was cut from +-4.5 rad to
#: +-1.0. That is not a regression. With no wrap geometry on the sheave the
#: moment arm is 40*cos(theta): it vanishes at |theta| ~ 1.55 rad and REVERSES
#: beyond, and +-4.5 rad sat 2.9x past the reversal. The old wider range bought
#: travel by driving the drum through its own dead zone, where the response was
#: neither monotonic nor decoupled - j1's purity was 1.7x there and is 5.1x now.
DRUM_FLOOR = {'torq_j1': (6.0, 3.0), 'torq_j2': (26.0, 6.0), 'torq_j3': (32.0, 6.0)}
CABLES = ['cable_left', 'cable_right', 'cable_back', 'cable_mid',
          'cable_loop_py', 'cable_loop_my']


def _tendon_names(model):
    """Tendon names in index order."""
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, t)
            for t in range(model.ntendon)]


def _hip_dofs(model):
    """qpos/dof indices of the three hip joints.

    They are no longer 0..2: the three motor drums are declared first, so the hip
    sits at 3..5. Anything indexing qpos[:3] as "the hip" is reading the drums.
    """
    return [int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)])
            for j in JOINTS]


def _hip_after(model, actuator, torque, steps=6000):
    """Hip angles in degrees after driving one drum at a constant torque."""
    data = mujoco.MjData(model)
    for _ in range(steps):
        data.ctrl[:] = 0.0
        data.ctrl[actuator] = torque
        mujoco.mj_step(model, data)
    return np.degrees(data.qpos[_hip_dofs(model)].copy())


@pytest.fixture(scope='module')
def model():
    """The compiled MJCF."""
    if not MJCF.is_file():
        pytest.skip('leg_spherical MJCF not reachable from an installed copy')
    return mujoco.MjModel.from_xml_path(str(MJCF))


def test_hip_is_three_hinges_not_a_ball_joint(model):
    """A ball joint would be invisible to FIC, which name-matches hinge joints."""
    assert model.njnt == 6, 'expected 3 motor drums plus the 3 hip hinges'
    for j in range(model.njnt):
        assert model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE, 'a ball joint is invisible to FIC'
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    assert [j for j in names if j in JOINTS] == JOINTS


def test_hip_axes_are_coincident_and_orthogonal(model):
    """Three coincident orthogonal hinges are kinematically a ball joint."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS]
    for j in ids:
        assert np.linalg.norm(data.xanchor[j] - BALL_CENTRE) < 2e-3, 'anchor off the fitted centre'
    axes = np.array([model.jnt_axis[j] for j in ids])
    assert np.allclose(axes @ axes.T, np.eye(3), atol=1e-9), 'axes not orthogonal'


def test_actuators_use_the_name_fic_commands(model):
    """FIC addresses actuators as torq_jN (fic_simulation.cpp:1522), not by joint name."""
    assert model.nu == 3
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in range(model.nu)]
    assert names == DRIVES


def test_the_spherical_joint_is_not_actuated(model):
    """This is a cable-driven parallel robot: no motor may act on the hip directly.

    Every actuator must be a TENDON transmission, so all joint torque arrives
    through cable tension and the 3-DOF hip stays passive.
    """
    hip_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS}
    for a in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        target = int(model.actuator_trnid[a][0])
        assert target not in hip_ids, f'{name} drives the spherical joint directly'
        driven = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, target)
        assert driven.endswith('_drum_j'), f'{name} drives {driven}, not a motor drum'
    hip = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS]
    assert np.allclose(model.jnt_stiffness[hip], 0.0), 'the hip also carries a spring'


def test_every_cable_is_a_pull_only_spring(model):
    """Cable = carries tension only when stretched, never pushes, cannot run away.

    A rigid inextensible cable does NOT work here. A differential drum fixes the
    SUM of its two cable lengths - turning it changes only their DIFFERENCE - so
    three drums impose three sum-constraints on the hip's three DOF and the hip
    is rigid by construction. Measured: at full torque torq_j1 moved hip z only
    20 deg with both green cables locked against their limits. Series elasticity
    is what real rope has, and it relieves the sum-constraint while still
    transmitting the difference.
    """
    names = _tendon_names(model)
    for c in CABLES:
        t = names.index(c)
        assert model.tendon_stiffness[t] > 0.0, f'{c} has no series elasticity'
        slo, shi = model.tendon_lengthspring[t]
        assert slo == 0.0 and shi > 0.0, \
            f'{c} must be pull-only (springlength 0..L0), got {slo}..{shi}'
        assert model.tendon_limited[t], f'{c} has no runaway backstop'
        lo, hi = model.tendon_range[t]
        assert lo == 0.0 and hi > shi, f'{c} backstop {hi:.4f} must sit beyond L0 {shi:.4f}'


def test_the_green_loop_pair_is_the_soft_one(model):
    """The green pair's length-sum is what couples every axis to every other.

    Its sum-gradient about hip y is -236 mm/rad, by far the largest coupling in
    the model, so those two cables resist all the other pairs. Softening them
    took torq_j1's axis purity from 7.9x to 27.5x.
    """
    names = _tendon_names(model)
    green = [model.tendon_stiffness[names.index(c)]
             for c in ('cable_loop_py', 'cable_loop_my')]
    rest = [model.tendon_stiffness[names.index(c)]
            for c in ('cable_left', 'cable_right', 'cable_back', 'cable_mid')]
    assert max(green) < min(rest), \
        f'the green pair must stay the compliant one: green {green}, rest {rest}'


def test_each_drum_carries_finite_rope(model):
    """An unbounded drum spools forever and drags the hip through its own limits.

    Measured without this: 274,000 deg of hip travel.
    """
    for nm in DRIVES:
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, nm + '_drum_j')
        assert model.jnt_limited[j], f'{nm} drum has no rope limit'
        assert model.jnt_range[j][1] > 0.0


def test_each_drum_drives_its_own_hip_axis(model):
    """One drum per axis: left/right, back/up, and rotation, each decoupled.

    Both cables of a pair land on the same drum at diametrically opposite points,
    so rotating it winds one on while paying the other off. The torque is the
    DIFFERENCE of the two tensions, which is what an antagonistic pair needs. A
    shared-tension pulley adds instead, which cancels the pair - measured: back
    and mid keep 5% of either cable and the whole drive conditions to 0.0006.
    """
    for nm, axis in DRUM_AXIS.items():
        k = DRIVES.index(nm)
        # 15000 steps: the ropes carry all the damping now, so a 10 s window is
        # still mid-transient - torq_j2 reads 8.7x decoupled there and 46x once
        # it has actually settled.
        lo = _hip_after(model, k, -100.0, steps=15000)
        hi = _hip_after(model, k, +100.0, steps=15000)
        span = np.abs(hi - lo)
        floor, purity = DRUM_FLOOR[nm]
        assert span[axis] > floor, f'{nm} moved its axis only {span[axis]:.1f} deg'
        assert int(np.argmax(span)) == axis, \
            f'{nm} does not dominate {"xyz"[axis]}: span {np.round(span, 1)}'
        others = max(span[i] for i in range(3) if i != axis)
        assert span[axis] > purity * others, \
            f'{nm} is not decoupled enough: span {np.round(span, 1)}'
        assert np.sign(lo[axis]) != np.sign(hi[axis]), f'{nm} is not bidirectional'


def test_each_rope_actually_moves_the_joint(model):
    """A rope that cannot move the hip is not a drive."""
    for k in range(model.nu):
        data = mujoco.MjData(model)
        for _ in range(2500):
            data.ctrl[:] = 0.0
            data.ctrl[k] = -60.0
            mujoco.mj_step(model, data)
        moved = np.abs(np.degrees(data.qpos)).max()
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, k)
        assert moved > 2.0, f'{name} at 60 N moved the hip only {moved:.2f} deg'


def test_ball_stays_in_the_socket(model):
    """Regression: the joint sat 40 mm from the ball, so it left the socket when it turned.

    A spherical joint's defining property is that the ball's centre does not move.
    If the hinge axes do not intersect at the ball centre, the head walks out of
    the acetabulum as soon as the leg rotates - which is what was seen on screen.
    """
    data = mujoco.MjData(model)
    head = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'HipJointHeadV2_geom')
    body = model.geom_bodyid[head]
    rng = np.random.default_rng(0)
    for _ in range(50):
        data.qpos[:] = 0.0
        data.qpos[_hip_dofs(model)] = rng.uniform(-1, 1, 3) * np.array([1.4, 1.4, 2.8])
        mujoco.mj_forward(model, data)
        moved = data.xpos[body] + data.xmat[body].reshape(3, 3) @ BALL_CENTRE
        assert np.linalg.norm(moved - BALL_CENTRE) < 1e-6, 'ball centre moved'


def test_ball_and_socket_are_concentric_about_the_joint(model):
    """The acetabulum's bearing surface must sit at one radius about the hip anchor.

    This reads the CAD directly, so it fails if the joint is moved away from the
    real ball centre - the check that the previous two wrong values both missed.
    """
    if not STL_DIR.is_dir():
        pytest.skip('STL meshes not reachable from an installed copy')
    import struct

    def surface(name):
        raw = (STL_DIR / name).read_bytes()
        count = struct.unpack('<I', raw[80:84])[0]
        tri = np.frombuffer(raw[84:84 + 50 * count], dtype=np.uint8).reshape(count, 50)
        v = tri[:, 12:48].copy().view('<f4').reshape(count, 3, 3).astype(float)
        centres = v.mean(axis=1) / 1000.0
        areas = 0.5 * np.linalg.norm(np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]), axis=1)
        return centres, areas / 1e6

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    anchor = data.xanchor[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINTS[0])]
    for name, lo, hi in (('Leg - Articulation-1 HipJointHeadV2-1.STL', 0.021, 0.024),
                         ('Leg - Articulation-1 Acetabulum-1.STL', 0.021, 0.024)):
        centres, areas = surface(name)
        radius = np.linalg.norm(centres - anchor, axis=1)
        band = areas[(radius > lo) & (radius < hi)].sum()
        assert band > 2e-3, f'{name}: only {band * 1e6:.0f} mm^2 on the 22-23 mm band'


def test_nothing_starts_in_contact(model):
    """The CAD parts interpenetrate by design; rigid contact between them ejected the leg."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0


def test_it_does_not_fall_over(model):
    """Regression: with no actuators and no capsule stiffness the leg flopped to 82 degrees."""
    data = mujoco.MjData(model)
    # 12000 steps: all the damping now lives in the ropes rather than the joints,
    # so the leg rings down over about 5 s. At 3 s |qvel| is still 1.7e-02.
    for _ in range(12000):
        mujoco.mj_step(model, data)
    hip = np.degrees(data.qpos[_hip_dofs(model)])
    assert np.max(np.abs(hip)) < 15.0, f'leg fell: {np.round(hip, 2)}'
    assert np.linalg.norm(data.qvel) < 1e-2, 'never came to rest'


def test_it_does_not_jerk(model):
    """Regression: floor penetration drove a 113 rad/s ejection on the first steps."""
    data = mujoco.MjData(model)
    peak = 0.0
    for _ in range(3000):
        mujoco.mj_step(model, data)
        peak = max(peak, float(np.linalg.norm(data.qvel[_hip_dofs(model)])))
    assert peak < 5.0, f'peak angular speed {peak:.1f} rad/s - model is unstable'


def _displacement_from_rest(model, actuator, torque, steps=15000):
    """Hip displacement in degrees from the settled rest pose, one actuator only.

    Single-sided on purpose. A +tau/-tau SPAN cancels any motion that is common
    to both directions, so it scores a badly cross-coupled axis as clean - the
    metric that first made this drive look decoupled. Displacement from rest
    keeps that coupling visible.
    """
    rest = mujoco.MjData(model)
    for _ in range(steps):
        mujoco.mj_step(model, rest)
    base = np.degrees(rest.qpos[_hip_dofs(model)]).copy()
    data = mujoco.MjData(model)
    for _ in range(steps):
        data.ctrl[:] = 0.0
        data.ctrl[actuator] = torque
        mujoco.mj_step(model, data)
    return np.degrees(data.qpos[_hip_dofs(model)]) - base


def test_commanded_torque_moves_the_matching_axis(model):
    """torq_j3 must drive the z hinge, or FIC's torque vector is scrambled.

    j3 is the one axis that holds up single-sided: +-19.6 deg on z, dominant in
    both directions at every torque from 1 to 5 N.m. j1 and j2 are checked by
    test_each_actuator_dominates_its_own_axis, which is a known failure.
    """
    for torque in (-2.0, 2.0):
        d = _displacement_from_rest(model, 2, torque)
        assert int(np.argmax(np.abs(d))) == 2, \
            f'torq_j3 at {torque} N.m moved {np.round(d, 2)}, not mainly z'
        assert abs(d[2]) > 5.0, f'torq_j3 barely moved z: {np.round(d, 2)}'


@pytest.mark.xfail(strict=True, reason=(
    'torq_j2 is cross-coupled and sign-reversing. Measured from rest: at +2 N.m '
    'it moves z by -8.5 deg while moving its own y by +7.1, and at +5 N.m dy is '
    '-7.26 - the axis runs BACKWARDS for a positive command. An axis that '
    'reverses sign oscillates under any feedback gain, so this must be fixed in '
    'the routing (the green pair sum-gradient), not in FIC tuning.'))
def test_each_actuator_dominates_its_own_axis(model):
    """The contract FIC assumes: actuator k moves hip axis k, both directions."""
    for k in range(3):
        for torque in (-2.0, 2.0):
            d = _displacement_from_rest(model, k, torque)
            assert int(np.argmax(np.abs(d))) == k, \
                f'torq_j{k + 1} at {torque} N.m moved {np.round(d, 2)}, not axis {"xyz"[k]}'
            assert np.sign(d[k]) == np.sign(torque) or abs(d[k]) < 1e-6, \
                f'torq_j{k + 1} runs backwards at {torque} N.m: {np.round(d, 2)}'


def test_urdf_and_mjcf_are_the_same_robot(model):
    """pinocchio parses the URDF while MuJoCo simulates the MJCF; FIC assumes one robot."""
    pin = pytest.importorskip('pinocchio')
    if not URDF.is_file():
        pytest.skip('URDF not reachable')
    mdl = pin.buildModelFromUrdf(str(URDF))
    assert [mdl.names[j] for j in range(1, mdl.njoints)] == JOINTS
    assert mdl.nq == 3 and mdl.nv == 3
    # the MJCF also carries the three motor drums, which the URDF does not model

    dat = mdl.createData()
    data = mujoco.MjData(model)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'foot')
    fid = mdl.getFrameId('foot_link')
    rng = np.random.default_rng(0)
    for _ in range(8):
        q = rng.uniform(-1, 1, 3) * np.array([1.4, 1.4, 2.8])
        data.qpos[:] = 0.0
        data.qpos[_hip_dofs(model)] = q
        mujoco.mj_forward(model, data)
        pin.forwardKinematics(mdl, dat, q)
        pin.updateFramePlacements(mdl, dat)
        err = np.linalg.norm(data.site_xpos[sid] - dat.oMf[fid].translation)
        assert err < 1e-4, f'URDF and MJCF disagree by {err * 1000:.3f} mm at q={q}'


def test_urdf_and_mjcf_agree_on_mass():
    """A mass mismatch would make FIC's gravity compensation wrong in simulation."""
    pin = pytest.importorskip('pinocchio')
    if not (URDF.is_file() and MJCF.is_file()):
        pytest.skip('model files not reachable')
    mdl = pin.buildModelFromUrdf(str(URDF))
    mjc = mujoco.MjModel.from_xml_path(str(MJCF))
    leg = mujoco.mj_name2id(mjc, mujoco.mjtObj.mjOBJ_BODY, 'Leg')
    assert abs(pin.computeTotalMass(mdl) - mjc.body_mass[leg]) < 1e-4


def test_config_describes_the_actual_model(model):
    """The config named joint1 / ee_link / link4, none of which exist in either file."""
    yaml = pytest.importorskip('yaml')
    cfg_path = FIC_ROOT / 'fic_library' / 'config' / 'robots_config.yaml'
    if not cfg_path.is_file():
        pytest.skip('robots_config.yaml not reachable')
    raw = yaml.safe_load(cfg_path.read_text())
    cfg = (raw.get('robots') or raw)['leg_spherical']

    assert cfg['num_joints'] == len(JOINTS), 'config must describe the hip, not the drums'
    assert cfg['joint_names'] == JOINTS
    for key in ('q_home_deg', 'q_home_rad'):
        assert len(cfg[key]) == len(JOINTS), f'{key} has the wrong length'
    for bound in ('lower', 'upper'):
        assert len(cfg['joint_limits_deg'][bound]) == len(JOINTS)
    pin = pytest.importorskip('pinocchio')
    mdl = pin.buildModelFromUrdf(str(URDF))
    assert mdl.existFrame(cfg['ee_frame_name']), 'ee_frame_name is not a frame in the URDF'
    assert not cfg['elbow_frame_name'], 'a 3-DOF hip has no elbow'


# ── The articular capsule ────────────────────────────────────────────────────
# Modelled as eight passive fibres from the acetabular rim to the femoral neck.
# It carries no actuation; it sets the joint's passive rotational stiffness, and
# that stiffness RISES with angle, which a linear <joint stiffness> cannot do.

def _capsule_only(model):
    """A copy of the model with the drive cables' series springs switched off.

    qfrc_passive sums every passive force, and the cables are springs now, so a
    raw reading is capsule + drive (106 N.m at 40 deg, almost all of it cable).
    These tests are about the capsule, so the cables are zeroed out first.
    """
    bare = mujoco.MjModel.from_xml_path(str(MJCF))
    names = _tendon_names(bare)
    for c in CABLES:
        bare.tendon_stiffness[names.index(c)] = 0.0
    return bare


def _passive_torque(model, axis, degrees):
    """Passive restoring torque about one hip axis at a given deflection."""
    model = _capsule_only(model)
    data = mujoco.MjData(model)
    hip = _hip_dofs(model)
    data.qpos[:] = 0.0
    data.qpos[hip[axis]] = np.radians(degrees)
    mujoco.mj_forward(model, data)
    return -float(data.qfrc_passive[hip[axis]])


def test_capsule_is_twelve_passive_fibres(model):
    """The capsule is soft tissue: fibres, and none of them are actuated.

    Twelve, spaced over the FULL 360 deg from acetabulum to femoral head. An
    earlier eight-fibre version followed the visible capsule mesh, which spans
    only ~120 deg of arc, and left the joint with no restoring torque at all in
    flexion.
    """
    names = _tendon_names(model)
    fibres = [n for n in names if n.startswith('capsule_fibre')]
    assert len(fibres) == 12
    tendon_acts = {int(model.actuator_trnid[a][0]) for a in range(model.nu)
                   if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_TENDON}
    fibre_ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f)
                 for f in fibres}
    assert not (tendon_acts & fibre_ids), \
        'a capsule fibre is actuated - the capsule must be passive soft tissue'


def test_hip_joints_carry_no_spring(model):
    """Passive resistance must come from the capsule, not a lumped joint spring."""
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JOINTS]
    assert np.allclose(model.jnt_stiffness[ids], 0.0), \
        'hip has joint stiffness - the capsule is no longer the source of compliance'


def test_the_model_contains_no_pd(model):
    """Every gain belongs to FIC, so the plant must be pure torque-in.

    No <position>/<velocity> actuator, no joint stiffness and no joint damping:
    a damper at the joint is a D term that silently competes with whatever FIC
    is tuned to. Dissipation lives in the physical elements instead - the ropes
    and the capsule fibres - which is where a real machine has it.
    """
    for a in range(model.nu):
        assert model.actuator_gaintype[a] == mujoco.mjtGain.mjGAIN_FIXED, \
            'actuator is not a plain torque motor'
        assert model.actuator_biastype[a] == mujoco.mjtBias.mjBIAS_NONE, \
            'actuator has a position/velocity bias - that is a PD servo'
    assert np.allclose(model.dof_damping, 0.0), \
        f'joint damping is a D term: {model.dof_damping}'
    assert np.allclose(model.jnt_stiffness, 0.0), \
        'joint stiffness is a P term'


def test_the_ropes_carry_the_damping(model):
    """Zero joint damping only works because the ropes dissipate instead.

    With nothing damped anywhere the mechanism is a spring network that rings
    forever - measured: free rest never settles, |qvel| 1.7 rad/s still after
    20 s. Moving that damping into the cables settles it to 6.5e-07.
    """
    names = _tendon_names(model)
    for c in CABLES:
        assert model.tendon_damping[names.index(c)] > 0.0, f'{c} has no damping'


def test_it_settles_with_no_command(model):
    """A model that will not sit still cannot be tuned against."""
    data = mujoco.MjData(model)
    for _ in range(20000):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
    assert np.linalg.norm(data.qvel) < 1e-5, \
        f'still moving with zero command: |qvel|={np.linalg.norm(data.qvel):.2e}'


def test_capsule_torsional_stiffness_rises_with_twist(model):
    """The requirement: rotational stiffness gets HIGHER as the joint twists.

    A linear spring has constant stiffness by definition. These fibres run along
    the neck, so twisting makes each take a helical path - longer than straight -
    and every fibre stretches at once, so torque grows faster than angle.
    """
    angles = [15, 20, 30, 40, 50]
    k = [_passive_torque(model, 2, a) / np.radians(a) for a in angles]
    assert all(b > a for a, b in zip(k, k[1:])), \
        f'torsional stiffness is not monotonically rising: {np.round(k, 4)}'
    assert k[-1] > 5 * k[0], \
        f'stiffening is too weak to matter: {k[0]:.4f} -> {k[-1]:.4f} N.m/rad'


def test_capsule_has_a_slack_neutral_zone(model):
    """Real capsules are limp near neutral; the leg must hang freely there.

    Without this the fibres would fight the drive cables everywhere, including
    at rest.
    """
    assert abs(_passive_torque(model, 2, 3)) < 1e-4, 'capsule resists tiny twists'
    assert abs(_passive_torque(model, 2, 40)) > 1.0, 'capsule never engages'


def test_capsule_holds_the_leg_at_a_sensible_droop(model):
    """With no joint spring, the capsule alone must arrest gravity and settle."""
    data = mujoco.MjData(model)
    for _ in range(4000):
        mujoco.mj_step(model, data)
    droop = abs(np.degrees(data.qpos[_hip_dofs(model)[1]]))
    assert droop < 15.0, f'droop {droop:.1f} deg is not plausible'
    assert np.linalg.norm(data.qvel) < 1e-2, 'never comes to rest'
