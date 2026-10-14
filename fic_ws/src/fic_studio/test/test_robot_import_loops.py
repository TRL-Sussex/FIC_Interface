"""Tests for importing closed-loop robots and robots that bring their own MuJoCo model.

The fixture is a planar five-bar linkage: two motors on the base, each driving
a proximal and a distal link, the two distal links joined at the tip -- the
smallest parallel mechanism (2 DOF). It is written as a URDF (the loop cut)
and as a MuJoCo model (the loop closed by a connect constraint), so each way
a loop can reach the importer is exercised: the MuJoCo model, Drake ball
constraints, onshape-to-robot closing frames, and a loop added by hand. The
repository's own CSPM, when present, checks the result against the config
that was written for it by hand.
"""
import os

import pytest

mujoco = pytest.importorskip('mujoco')

from fic_studio.robot_import import closed_loop, importer, mechanism  # noqa: E402, I100
from fic_studio.robot_import.urdf_info import UrdfInfo  # noqa: E402

PKG = 'five_bar_description'
INERTIA = ('<inertial><origin xyz="0 0.05 0"/><mass value="0.1"/>'
           '<inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial>')


def _link(name, mass=True):
    return '<link name="%s">%s</link>' % (name, INERTIA if mass else '')


def _rev(name, parent, child, xyz, effort=5):
    return ('<joint name="%s" type="revolute"><parent link="%s"/><child link="%s"/>'
            '<origin xyz="%s"/><axis xyz="0 0 1"/>'
            '<limit lower="-2" upper="2" effort="%g" velocity="3"/></joint>'
            % (name, parent, child, xyz, effort))


def five_bar_urdf(extra_links='', extra_joints='', head='<robot name="five_bar">'):
    """Motors m1/m2 at x=-/+0.1, elbows 0.2 up, both distal links reach (0, 0.35)."""
    links = [_link('base_link'), _link('l_prox'), _link('l_dist'), _link('r_prox'),
             _link('r_dist'), _link('tip', mass=False)]
    joints = [_rev('m1', 'base_link', 'l_prox', '-0.1 0 0'),
              _rev('p1', 'l_prox', 'l_dist', '0 0.2 0', effort=1),
              _rev('m2', 'base_link', 'r_prox', '0.1 0 0'),
              _rev('p2', 'r_prox', 'r_dist', '0 0.2 0', effort=1),
              '<joint name="tip_joint" type="fixed"><parent link="l_dist"/>'
              '<child link="tip"/><origin xyz="0.1 0.15 0"/></joint>']
    return '<?xml version="1.0"?>\n%s%s%s%s%s</robot>' % (
        head, ''.join(links), extra_links, ''.join(joints), extra_joints)


FIVE_BAR_MJCF = """<mujoco model="five_bar">
  <option timestep="0.001" integrator="implicitfast"/>
  <default><joint damping="0.05" armature="0.001"/></default>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.01"/>
    <body name="base_link">
      <inertial pos="0 0.05 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
      <body name="l_prox" pos="-0.1 0 0">
        <joint name="m1" axis="0 0 1" range="-2 2"/>
        <inertial pos="0 0.05 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
        <geom type="capsule" fromto="0 0 0 0 0.2 0" size="0.01" contype="0" conaffinity="0"/>
        <body name="l_dist" pos="0 0.2 0">
          <joint name="p1" axis="0 0 1" range="-2 2"/>
          <inertial pos="0 0.05 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
          <geom type="capsule" fromto="0 0 0 0.1 0.15 0" size="0.01" contype="0" conaffinity="0"/>
          <site name="left_tip" pos="0.1 0.15 0"/>
          <body name="tip" pos="0.1 0.15 0"/>
        </body>
      </body>
      <body name="r_prox" pos="0.1 0 0">
        <joint name="m2" axis="0 0 1" range="-2 2"/>
        <inertial pos="0 0.05 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
        <geom type="capsule" fromto="0 0 0 0 0.2 0" size="0.01" contype="0" conaffinity="0"/>
        <body name="r_dist" pos="0 0.2 0">
          <joint name="p2" axis="0 0 1" range="-2 2"/>
          <inertial pos="0 0.05 0" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
          <geom type="capsule" fromto="0 0 0 -0.1 0.15 0" size="0.01" contype="0" conaffinity="0"/>
          <site name="right_tip" pos="-0.1 0.15 0"/>
        </body>
      </body>
    </body>
  </worldbody>
  <equality>
    <connect name="tip_loop" site1="left_tip" site2="right_tip" solref="0.005 1"/>
  </equality>
  <actuator>
    <position name="left_servo" joint="m1" kp="10" ctrlrange="-2 2"/>
    <motor name="right_motor" joint="m2" ctrlrange="-3 3"/>
  </actuator>
</mujoco>
"""

DRAKE_LOOP = ('<drake:ball_constraint name="tip_loop">'
              '<drake:ball_constraint_body_A name="tip"/>'
              '<drake:ball_constraint_p_AP value="0 0 0"/>'
              '<drake:ball_constraint_body_B name="r_dist"/>'
              '<drake:ball_constraint_p_BQ value="-0.1 0.15 0"/>'
              '</drake:ball_constraint>')


def make_five_bar(root, urdf_text=None, mjcf=True):
    pkg = os.path.join(root, PKG)
    os.makedirs(os.path.join(pkg, 'urdf'))
    with open(os.path.join(pkg, 'urdf', 'five_bar.urdf'), 'w') as fh:
        fh.write(urdf_text or five_bar_urdf())
    if mjcf:
        os.makedirs(os.path.join(pkg, 'mjcf'))
        with open(os.path.join(pkg, 'mjcf', 'five_bar.xml'), 'w') as fh:
            fh.write(FIVE_BAR_MJCF)
    return pkg


@pytest.fixture
def model_root(tmp_path):
    root = tmp_path / 'menagerie' / 'model'
    root.mkdir(parents=True)
    return str(root)


def _load(res):
    return mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))


# ── The robot's own MuJoCo model ──────────────────────────────────────────

class TestOwnMujocoModel:

    def test_found_beside_the_urdf_and_its_loop_read(self, tmp_path):
        plan = importer.analyse(make_five_bar(str(tmp_path)))
        assert plan.mjcf is not None and plan.mjcf.path.endswith('five_bar.xml')
        assert len(plan.closures) == 1
        c = plan.closures[0]
        assert (c.body_a, c.body_b) == ('l_dist', 'r_dist')     # where the sites are
        assert c.pos_a == pytest.approx([0.1, 0.15, 0], abs=1e-9)
        assert c.pos_b == pytest.approx([-0.1, 0.15, 0], abs=1e-9)
        assert c.solref == pytest.approx([0.005, 1])

    def test_mechanism_worked_out(self, tmp_path):
        plan = importer.analyse(make_five_bar(str(tmp_path)))
        m = plan.mechanism
        assert m['ee_dof'] == 2 and m['determined'] and not m['orientation_only']
        assert [j.name for j in plan.joints] == ['m1', 'm2']
        assert m['driven_source'] == "the MuJoCo model's actuators"
        assert plan.ee_link == 'tip'
        assert set(plan.mechanism_joints) == {'m1', 'p1', 'm2', 'p2'}
        assert not plan.errors

    def test_created_robot(self, tmp_path, model_root):
        plan = importer.analyse(make_five_bar(str(tmp_path)))
        res = importer.create_robot(plan, model_root)
        m = _load(res)
        acts = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i):
                mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[i][0])
                for i in range(m.nu)}
        assert acts == {'torq_j1': 'm1', 'torq_j2': 'm2'}     # servo replaced, motor renamed
        assert m.neq == 1 and m.nkey == 1
        sensors = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i) for i in range(m.nsensor)}
        assert {'force_sensor', 'torque_sensor'} <= sensors
        # The model's own tuning survives: its damping, not the importer's.
        assert m.dof_damping[0] == pytest.approx(0.05)
        # One floor: the model brought its own, so scene.xml adds none.
        floors = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
        assert len(floors) == 1
        cfg = res.config
        assert cfg['kinematics_solver'] == 'pinocchio_closed_loop'
        assert cfg['joint_names'] == ['m1', 'm2'] and cfg['num_joints'] == 2
        assert cfg['loop_closures'] == [{'bodyA': 'l_dist', 'posA': [0.1, 0.15, 0.0],
                                         'bodyB': 'r_dist', 'posB': [-0.1, 0.15, 0.0]}]
        assert cfg['ee_orientation_only'] is False
        assert cfg['view_joint_names'] == ['m1', 'p1', 'm2', 'p2']
        assert cfg['joint_torque_limits']['max'][1] == pytest.approx(3.0)   # from the motor
        assert cfg['default_ee_target']['position'] == pytest.approx([0.0, 0.35, 0.0], abs=1e-4)

    def test_home_keyframe_closes_the_loop(self, tmp_path, model_root):
        plan = importer.analyse(make_five_bar(str(tmp_path)))
        plan.joints[0].home_deg, plan.joints[1].home_deg = 10.0, -10.0
        res = importer.create_robot(plan, model_root)
        m = _load(res)
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, 0)
        res_vec, _ = closed_loop.constraint_state(m, d)
        assert abs(res_vec).max() < 1e-6
        adr = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'm1')]
        assert d.qpos[adr] == pytest.approx(0.1745, abs=1e-3)

    def test_explicit_model_and_none(self, tmp_path):
        pkg = make_five_bar(str(tmp_path))
        plan = importer.analyse(os.path.join(pkg, 'urdf', 'five_bar.urdf'), mjcf_path='')
        assert plan.mjcf is None and not plan.closures      # two open legs: a tree
        importer.set_mjcf(plan, os.path.join(pkg, 'mjcf', 'five_bar.xml'))
        assert plan.mjcf is not None and plan.closed_loop

    def test_a_lone_mujoco_model_is_refused(self, tmp_path):
        pkg = make_five_bar(str(tmp_path))
        with pytest.raises(importer.ImportError_, match='URDF is required'):
            importer.analyse(os.path.join(pkg, 'mjcf', 'five_bar.xml'))

    def test_disagreement_with_the_urdf_is_flagged(self, tmp_path):
        pkg = make_five_bar(str(tmp_path))
        path = os.path.join(pkg, 'mjcf', 'five_bar.xml')
        text = open(path).read().replace('<joint name="p1" axis="0 0 1"',
                                         '<joint name="p1" axis="1 0 0"')
        open(path, 'w').write(text)
        plan = importer.analyse(pkg)
        assert any('disagree' in m and 'p1' in m for _, m in plan.messages)


# ── Loops from the URDF, or added by hand ─────────────────────────────────

class TestUrdfLoops:

    def test_drake_ball_constraint(self, tmp_path, model_root):
        text = five_bar_urdf(
            extra_joints=DRAKE_LOOP,
            head='<robot name="five_bar" xmlns:drake="http://drake.mit.edu">')
        plan = importer.analyse(make_five_bar(str(tmp_path), text, mjcf=False))
        assert plan.mjcf is None and len(plan.closures) == 1
        assert plan.closures[0].source == 'the URDF (Drake ball constraint)'
        assert [j.name for j in plan.joints] == ['m1', 'm2']     # base-first pick
        assert plan.mechanism['determined'] and plan.mechanism['ee_dof'] == 2
        res = importer.create_robot(plan, model_root)
        m = _load(res)
        assert m.neq == 1
        d = mujoco.MjData(m)
        mujoco.mj_resetDataKeyframe(m, d, 0)
        res_vec, _ = closed_loop.constraint_state(m, d)
        assert abs(res_vec).max() < 1e-6
        assert res.config['kinematics_solver'] == 'pinocchio_closed_loop'
        # Passive joints are damped like the driven ones.
        p1 = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'p1')]
        assert m.dof_damping[p1] > 0

    def test_onshape_closing_frames(self, tmp_path):
        frames = (_link('closing_tip_1', mass=False) + _link('closing_tip_2', mass=False))
        joints = ('<joint name="c1" type="fixed"><parent link="l_dist"/>'
                  '<child link="closing_tip_1"/><origin xyz="0.1 0.15 0"/></joint>'
                  '<joint name="c2" type="fixed"><parent link="r_dist"/>'
                  '<child link="closing_tip_2"/><origin xyz="-0.1 0.15 0"/></joint>')
        plan = importer.analyse(make_five_bar(str(tmp_path), five_bar_urdf(frames, joints),
                                              mjcf=False))
        assert [(c.body_a, c.body_b) for c in plan.closures] == [
            ('closing_tip_1', 'closing_tip_2')]
        assert plan.mechanism['determined']

    def test_loop_added_by_hand(self, tmp_path, model_root):
        plan = importer.analyse(make_five_bar(str(tmp_path), mjcf=False))
        assert not plan.closed_loop
        loop = mechanism.closure_from_point(plan.info, 'tip', [0, 0, 0], 'r_dist')
        assert loop.pos_b == pytest.approx([-0.1, 0.15, 0], abs=1e-9)
        importer.set_closures(plan, [loop])
        assert plan.closed_loop and plan.ee_link in ('tip', 'r_dist')
        importer.set_end_effector(plan, 'tip')
        assert [j.name for j in plan.joints] == ['m1', 'm2']
        importer.create_robot(plan, model_root)

    def test_too_few_driven_joints_is_an_error(self, tmp_path):
        plan = importer.analyse(make_five_bar(str(tmp_path)))
        importer.set_driven(plan, ['m1'])
        assert any('can still move' in e for e in plan.errors)
        importer.set_driven(plan, ['m1', 'p2'])       # odd, but it does hold the tip
        assert not plan.errors

    def test_free_body_without_loops_asks_for_them(self, tmp_path):
        text = five_bar_urdf(
            extra_links=_link('plate') + '<link name="world"/>',
            extra_joints=('<joint name="mount" type="fixed"><parent link="world"/>'
                          '<child link="base_link"/></joint>'
                          '<joint name="plate_free" type="floating"><parent link="world"/>'
                          '<child link="plate"/></joint>'))
        plan = importer.analyse(make_five_bar(str(tmp_path), text, mjcf=False))
        assert any('no loops were found' in e for e in plan.errors)


# ── Pieces ────────────────────────────────────────────────────────────────

class TestAnalysisPieces:

    def _model(self):
        spec = mujoco.MjSpec.from_string(FIVE_BAR_MJCF)
        return spec.compile()

    def test_mobility_and_orientation(self):
        m = self._model()
        d = mujoco.MjData(m)
        closed_loop.generic_pose(m, d)
        assert closed_loop.mobility(m, d) == 2
        assert closed_loop.ee_mobility(m, d, 'tip') == 2
        assert not closed_loop.orientation_only(m, d, 'tip')

    def test_pick_driven_prefers_the_base(self):
        m = self._model()
        d = mujoco.MjData(m)
        closed_loop.generic_pose(m, d)
        order = closed_loop.base_first(m, ['p1', 'p2', 'm1', 'm2'])
        assert order[:2] == ['m1', 'm2']
        assert closed_loop.pick_driven(m, d, order, 'tip') == ['m1', 'm2']

    def test_assemble_closes_the_loop(self):
        m = self._model()
        d = mujoco.MjData(m)
        assert closed_loop.assemble(m, d, {'m1': 0.3, 'm2': -0.2}) < 1e-8
        adr = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'm1')]
        assert d.qpos[adr] == pytest.approx(0.3)

    def test_link_poses_match_mujoco(self, tmp_path):
        info = UrdfInfo(five_bar_urdf(), 'five_bar.urdf')
        from fic_studio.robot_import.urdf_info import link_poses
        poses = link_poses(info, {'m1': 0.4, 'p1': -0.3})
        m = self._model()
        d = mujoco.MjData(m)
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'm1')]] = 0.4
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'p1')]] = -0.3
        mujoco.mj_kinematics(m, d)
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'tip')
        assert [row[3] for row in poses['tip'][:3]] == pytest.approx(list(d.xpos[bid]), abs=1e-9)


# ── The repository's CSPM against its hand-written config ─────────────────

MENAGERIE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'menagerie',
                                         'model'))


@pytest.mark.skipif(not os.path.isdir(os.path.join(MENAGERIE, 'CSPM')), reason='no CSPM')
def test_cspm_matches_its_hand_written_config(tmp_path):
    import yaml
    root = os.path.abspath(os.path.join(MENAGERIE, '..', '..', '..', '..'))
    ref = yaml.safe_load(open(os.path.join(root, 'fic_library', 'config',
                                           'robots_config.yaml')))['robots']['CSPM']
    plan = importer.analyse(os.path.join(MENAGERIE, 'CSPM'), existing_ids=['CSPM'])
    model_root = tmp_path / 'model'
    model_root.mkdir()
    cfg = importer.create_robot(plan, str(model_root)).config
    for key in ('joint_names', 'ee_frame_name', 'kinematics_solver', 'ee_orientation_only',
                'view_joint_names'):
        assert cfg[key] == ref[key], key
    for mine, theirs in zip(cfg['loop_closures'], ref['loop_closures']):
        assert (mine['bodyA'], mine['bodyB']) == (theirs['bodyA'], theirs['bodyB'])
        assert mine['posA'] == pytest.approx(theirs['posA'], abs=1e-6)
        assert mine['posB'] == pytest.approx(theirs['posB'], abs=1e-6)
