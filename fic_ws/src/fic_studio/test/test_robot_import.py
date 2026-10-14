"""Tests for fic_studio.robot_import: URDF in, simulation-ready robot out.

Every fixture is generated here -- a small arm with a two-finger gripper, its
meshes written as real STL / COLLADA files inside a ROS description package
that is not installed -- so the tests exercise the same paths a user's drop
does: package:// and foreign absolute mesh paths, xacro $(find ...), zips,
gripper joints held fixed, and the files the simulator and controllers read.
"""
import os
import shutil
import struct
import xml.etree.ElementTree as ET
import zipfile

import pytest

mujoco = pytest.importorskip('mujoco')

from fic_studio.robot_import import importer, mjcf  # noqa: E402, I100
from fic_studio.robot_import.dae import dae_to_obj  # noqa: E402
from fic_studio.robot_import.meshes import MeshFinder  # noqa: E402
from fic_studio.robot_import.urdf_info import (  # noqa: E402
    find_packages, UrdfError, UrdfInfo,
)

PKG = 'toy_arm_description'


# ── Fixture builders ──────────────────────────────────────────────────────

def _box_triangles(sx, sy, sz):
    x, y, z = sx / 2, sy / 2, sz / 2
    v = [(-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
         (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v, faces


def write_stl(path, size=(0.05, 0.05, 0.2)):
    """Write a binary STL box."""
    v, faces = _box_triangles(*size)
    with open(path, 'wb') as fh:
        fh.write(b'toy'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(faces)))
        for a, b, c in faces:
            fh.write(struct.pack('<3f', 0, 0, 0))
            for idx in (a, b, c):
                fh.write(struct.pack('<3f', *v[idx]))
            fh.write(struct.pack('<H', 0))


def write_dae(path, size=(0.05, 0.05, 0.2), up='Z_UP', meter=1.0):
    """Write a minimal COLLADA box as <triangles>."""
    v, faces = _box_triangles(*size)
    pos = ' '.join('%g %g %g' % p for p in v)
    idx = ' '.join('%d %d %d' % f for f in faces)
    text = f"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="{meter}"/><up_axis>{up}</up_axis></asset>
  <library_geometries><geometry id="g"><mesh>
    <source id="p"><float_array id="pa" count="24">{pos}</float_array>
      <technique_common><accessor source="#pa" count="8" stride="3"/></technique_common></source>
    <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
    <triangles count="12"><input semantic="VERTEX" source="#v" offset="0"/><p>{idx}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node id="n">
    <instance_geometry url="#g"/></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>"""
    with open(path, 'w') as fh:
        fh.write(text)


def _inertial(mass=1.0):
    if mass is None:
        return ''
    return ('<inertial><origin xyz="0 0 0.1"/><mass value="%g"/>'
            '<inertia ixx="0.01" iyy="0.01" izz="0.005" ixy="0" ixz="0" iyz="0"/>'
            '</inertial>' % mass)


def _link(name, mesh_uri=None, mass=1.0, visual_uri=None):
    body = _inertial(mass)
    if visual_uri:
        body += ('<visual><geometry><mesh filename="%s"/></geometry></visual>' % visual_uri)
    if mesh_uri:
        body += ('<collision><geometry><mesh filename="%s"/></geometry></collision>'
                 % mesh_uri)
    return '<link name="%s">%s</link>' % (name, body)


def _joint(name, parent, child, jtype='revolute', xyz='0 0 0.2', lower=-2.0,
           upper=2.0, effort=40.0):
    limit = ''
    if jtype in ('revolute', 'prismatic'):
        limit = '<limit lower="%g" upper="%g" effort="%s" velocity="1"/>' % (
            lower, upper, '' if effort is None else '%g' % effort)
    elif jtype == 'continuous' and effort is not None:
        limit = '<limit effort="%g" velocity="1"/>' % effort
    return ('<joint name="%s" type="%s"><parent link="%s"/><child link="%s"/>'
            '<origin xyz="%s"/><axis xyz="0 1 0"/>%s</joint>'
            % (name, jtype, parent, child, xyz, limit))


def arm_urdf(mesh=lambda n: 'package://%s/meshes/%s.stl' % (PKG, n), gripper=True,
             visual=None, extra_links='', extra_joints='', joint2_type='revolute',
             name='toy_arm'):
    """Return a 3-joint arm on a base, a tool0 frame, and optionally two fingers."""
    links = [
        _link('base_link', mesh('base')),
        _link('link1', mesh('link1'), visual_uri=visual and visual('link1')),
        _link('link2', mesh('link2')),
        _link('link3', mesh('link3')),
        _link('tool0', mass=None),
    ]
    joints = [
        _joint('joint1', 'base_link', 'link1', xyz='0 0 0.1'),
        _joint('joint2', 'link1', 'link2', jtype=joint2_type, effort=None),
        _joint('joint3', 'link2', 'link3', jtype='continuous'),
        _joint('tool0_joint', 'link3', 'tool0', jtype='fixed', xyz='0 0 0.15'),
    ]
    if gripper:
        links += [_link('hand', mesh('hand'), mass=0.3),
                  _link('left_finger', mesh('finger'), mass=0.05),
                  _link('right_finger', mesh('finger'), mass=0.05)]
        joints += [
            _joint('hand_joint', 'link3', 'hand', jtype='fixed', xyz='0 0 0.2'),
            _joint('left_finger_joint', 'hand', 'left_finger', jtype='prismatic',
                   xyz='0 0.02 0.05', lower=0, upper=0.04),
            _joint('right_finger_joint', 'hand', 'right_finger', jtype='prismatic',
                   xyz='0 -0.02 0.05', lower=0, upper=0.04),
        ]
    return '<?xml version="1.0"?>\n<robot name="%s">%s%s%s%s</robot>' % (
        name, ''.join(links), extra_links, ''.join(joints), extra_joints)


def make_package(root, urdf_text=None, xacro=False, dae=False):
    """Write a ROS description package (not installed) holding the toy arm."""
    pkg = os.path.join(root, PKG)
    os.makedirs(os.path.join(pkg, 'urdf'))
    os.makedirs(os.path.join(pkg, 'meshes', 'visual'))
    with open(os.path.join(pkg, 'package.xml'), 'w') as fh:
        fh.write('<package format="3"><name>%s</name><version>0.0.1</version>'
                 '</package>' % PKG)
    for n in ('base', 'link1', 'link2', 'link3', 'hand', 'finger'):
        write_stl(os.path.join(pkg, 'meshes', n + '.stl'))
    if dae:
        write_dae(os.path.join(pkg, 'meshes', 'visual', 'link1.dae'))
    text = urdf_text or arm_urdf()
    if xacro:
        body = text.split('<robot name="toy_arm">', 1)[1].rsplit('</robot>', 1)[0]
        with open(os.path.join(pkg, 'urdf', 'parts.xacro'), 'w') as fh:
            fh.write('<?xml version="1.0"?><robot xmlns:xacro="http://www.ros.org/wiki/xacro">'
                     '<xacro:macro name="toy">%s</xacro:macro></robot>' % body)
        with open(os.path.join(pkg, 'urdf', 'toy_arm.urdf.xacro'), 'w') as fh:
            fh.write('<?xml version="1.0"?><robot name="toy_arm" '
                     'xmlns:xacro="http://www.ros.org/wiki/xacro">'
                     '<xacro:include filename="$(find %s)/urdf/parts.xacro"/>'
                     '<xacro:toy/></robot>' % PKG)
    else:
        with open(os.path.join(pkg, 'urdf', 'toy_arm.urdf'), 'w') as fh:
            fh.write(text)
    return pkg


@pytest.fixture
def pkg(tmp_path):
    return make_package(str(tmp_path / 'src'))


@pytest.fixture
def model_root(tmp_path):
    root = tmp_path / 'menagerie' / 'model'
    root.mkdir(parents=True)
    return str(root)


# ── Analyse ───────────────────────────────────────────────────────────────

class TestAnalyse:

    def test_folder_finds_arm_and_gripper(self, pkg):
        plan = importer.analyse(pkg)
        assert os.path.basename(plan.urdf_path) == 'toy_arm.urdf'
        assert plan.ee_link == 'tool0'
        assert [j.name for j in plan.joints] == ['joint1', 'joint2', 'joint3']
        assert set(plan.extra_joints) == {'left_finger_joint', 'right_finger_joint'}
        assert plan.mesh_counts == (7, 7)
        assert plan.robot_id == 'toy_arm'
        assert plan.display_name == 'Toy Arm'
        assert plan.elbow_link == 'link2'
        assert not plan.errors
        g = plan.gripper
        assert g and g.driver == 'left_finger_joint' and g.fingers == 2
        assert g.max_width == pytest.approx(0.08)

    def test_limits_efforts_and_notes(self, pkg):
        plan = importer.analyse(pkg)
        j1, j2, j3 = plan.joints
        assert j1.lower_deg == pytest.approx(-114.59, abs=0.01)
        assert j1.effort == 40.0 and j1.effort_from_urdf
        assert j2.effort == mjcf.DEFAULT_EFFORT and not j2.effort_from_urdf
        assert j3.continuous and (j3.lower_deg, j3.upper_deg) == (-360.0, 360.0)
        text = ' '.join(m for _, m in plan.messages)
        assert 'joint2' in text
        assert 'held fixed' not in text        # the fingers are driven, not held

    def test_single_file_in_package(self, pkg):
        plan = importer.analyse(os.path.join(pkg, 'urdf', 'toy_arm.urdf'))
        assert plan.mesh_counts == (7, 7)

    def test_existing_ids_get_suffix(self, pkg):
        plan = importer.analyse(pkg, existing_ids=['Toy_Arm', 'toy_arm_2'])
        assert plan.robot_id == 'toy_arm_3'

    def test_set_end_effector_recomputes(self, pkg):
        plan = importer.analyse(pkg)
        plan.joints[0].home_deg = 12.0
        importer.set_end_effector(plan, 'link2')
        assert [j.name for j in plan.joints] == ['joint1', 'joint2']
        assert plan.joints[0].home_deg == 12.0
        assert 'joint3' in plan.extra_joints

    def test_zip(self, pkg, tmp_path):
        zpath = str(tmp_path / 'toy.zip')
        with zipfile.ZipFile(zpath, 'w') as zf:
            for dirpath, _, files in os.walk(pkg):
                for f in files:
                    full = os.path.join(dirpath, f)
                    zf.write(full, os.path.relpath(full, os.path.dirname(pkg)))
        plan = importer.analyse(zpath)
        try:
            assert plan.mesh_counts == (7, 7)
            assert plan.temp_dir and os.path.isdir(plan.temp_dir)
        finally:
            plan.cleanup()
        assert plan.temp_dir is None

    def test_zip_with_unsafe_path_refused(self, tmp_path):
        zpath = str(tmp_path / 'evil.zip')
        with zipfile.ZipFile(zpath, 'w') as zf:
            zf.writestr('../evil.urdf', arm_urdf())
        with pytest.raises(importer.ImportError_, match='unsafe'):
            importer.analyse(zpath)

    def test_folder_without_urdf(self, tmp_path):
        (tmp_path / 'empty').mkdir()
        with pytest.raises(importer.ImportError_, match='No .urdf'):
            importer.analyse(str(tmp_path / 'empty'))

    def test_wrong_file_type(self, tmp_path):
        p = tmp_path / 'robot.stl'
        p.write_text('x')
        with pytest.raises(importer.ImportError_, match='Drop a .urdf'):
            importer.analyse(str(p))

    def test_not_a_urdf(self, tmp_path):
        p = tmp_path / 'thing.urdf'
        p.write_text('<sdf version="1.7"/>')
        with pytest.raises(importer.ImportError_, match='not a URDF'):
            importer.analyse(str(p))

    def test_missing_mesh_is_a_warning(self, tmp_path):
        pkg = make_package(str(tmp_path), arm_urdf(
            mesh=lambda n: 'package://%s/meshes/%s.stl' % (PKG, 'nope' if n == 'link2' else n)))
        plan = importer.analyse(pkg)
        assert plan.mesh_counts == (6, 7)
        assert any(lvl == importer.WARN and 'nope.stl' in m for lvl, m in plan.messages)
        assert not plan.errors

    def test_missing_visual_with_collision_is_info(self, tmp_path):
        pkg = make_package(str(tmp_path), arm_urdf(visual=lambda n: 'meshes/gone.dae'))
        plan = importer.analyse(pkg)
        assert any(lvl == importer.INFO and 'collision meshes instead' in m
                   for lvl, m in plan.messages)

    def test_sliding_arm_joint_is_an_error(self, tmp_path):
        pkg = make_package(str(tmp_path), arm_urdf(joint2_type='prismatic'))
        plan = importer.analyse(pkg)
        assert any('prismatic' in e for e in plan.errors)

    def test_closed_loop_is_an_error(self, tmp_path):
        text = arm_urdf(
            gripper=False,
            extra_links=_link('platform', mass=0.1) + '<link name="world"/>',
            extra_joints=('<joint name="base_mount" type="fixed"><parent link="world"/>'
                          '<child link="base_link"/></joint>'
                          '<joint name="platform_ball" type="floating">'
                          '<parent link="world"/><child link="platform"/></joint>'))
        plan = importer.analyse(make_package(str(tmp_path), text))
        assert any('closed-loop' in e for e in plan.errors)

    def test_floating_mount_is_fixed(self, tmp_path, model_root):
        text = arm_urdf(gripper=False, extra_links='<link name="world"/>',
                        extra_joints=('<joint name="mount" type="floating"><parent link="world"/>'
                                      '<child link="base_link"/></joint>'))
        plan = importer.analyse(make_package(str(tmp_path), text))
        assert not plan.errors
        assert any('mobile base' in m for _, m in plan.messages)
        res = importer.create_robot(plan, model_root)
        m = mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))
        assert m.njnt == 3              # no free joint: the robot stays put

    def test_xacro_with_find_of_unbuilt_package(self, tmp_path):
        if not shutil.which('xacro'):
            pytest.skip('xacro not installed')
        pkg = make_package(str(tmp_path / 'src'), xacro=True)
        plan = importer.analyse(os.path.join(pkg, 'urdf', 'toy_arm.urdf.xacro'))
        assert [j.name for j in plan.joints] == ['joint1', 'joint2', 'joint3']
        assert plan.mesh_counts == (7, 7)
        assert any('xacro' in m for _, m in plan.messages)

    def test_xacro_missing_package_message(self, tmp_path):
        if not shutil.which('xacro'):
            pytest.skip('xacro not installed')
        p = tmp_path / 'r.xacro'
        p.write_text('<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro">'
                     '<xacro:include filename="$(find no_such_pkg_xyz)/a.xacro"/></robot>')
        with pytest.raises(importer.ImportError_, match='no_such_pkg_xyz'):
            importer.analyse(str(p))


# ── Create ────────────────────────────────────────────────────────────────

class TestCreate:

    def _create(self, pkg, model_root, **kw):
        plan = importer.analyse(pkg)
        for k, v in kw.items():
            setattr(plan, k, v)
        return plan, importer.create_robot(plan, model_root)

    def test_layout_matches_menagerie(self, pkg, model_root):
        plan, res = self._create(pkg, model_root)
        d = os.path.join(model_root, 'toy_arm')
        assert res.model_dir == d
        for rel in ('URDF/toy_arm.urdf', 'URDF/toy_arm.original.urdf', 'URDF/meshes/link1.stl',
                    'xml/toy_arm.xml', 'xml/scene.xml', 'xml/assets/link1.stl',
                    'import_report.md'):
            assert os.path.isfile(os.path.join(d, rel)), rel
        assert not [n for n in os.listdir(model_root) if n.startswith('.import_')]
        assert not os.path.exists(os.path.join(d, 'xml', '_raw.xml'))

    def test_model_meets_simulator_contract(self, pkg, model_root):
        _, res = self._create(pkg, model_root)
        m = mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))
        name = lambda kind, i: mujoco.mj_id2name(m, kind, i)  # noqa: E731
        assert [name(mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)] == \
            ['torq_j1', 'torq_j2', 'torq_j3', 'gripper']
        # Motor k drives the k-th configured joint.
        for k, jn in enumerate(res.config['joint_names']):
            assert name(mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[k][0]) == jn
        sensors = {name(mujoco.mjtObj.mjOBJ_SENSOR, i) for i in range(m.nsensor)}
        assert {'force_sensor', 'torque_sensor'} <= sensors
        hinges = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
        assert len(hinges) == 3         # the simulator publishes hinges: just the arm
        assert m.njnt == 5              # plus the two sliding fingers
        assert m.opt.timestep == pytest.approx(mjcf.TIMESTEP)
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'home') == 0
        assert m.nmesh >= 1

    def test_arm_joints_are_damped_like_the_house_models(self, pkg, model_root):
        _, res = self._create(pkg, model_root)
        m = mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))
        dof = {n: m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
               for n in ('joint1', 'joint2', 'joint3')}
        assert m.dof_damping[dof['joint1']] == pytest.approx(min(45, 1.15 * 40))
        assert m.dof_damping[dof['joint2']] == pytest.approx(45.0)   # 50 Nm default: capped
        # Left alone, the arm settles instead of falling: it barely moves in 3 s.
        d = mujoco.MjData(m)
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'joint2')]] = 0.5
        start = d.qpos.copy()
        for _ in range(3000):
            mujoco.mj_step(m, d)
        assert abs(d.qpos - start).max() < 0.5

    def test_ctrlrange_from_effort(self, pkg, model_root):
        _, res = self._create(pkg, model_root)
        m = mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))
        assert list(m.actuator_ctrlrange[0]) == [-40.0, 40.0]
        assert list(m.actuator_ctrlrange[1]) == [-mjcf.DEFAULT_EFFORT, mjcf.DEFAULT_EFFORT]

    def test_saved_urdf_for_controllers(self, pkg, model_root):
        _, res = self._create(pkg, model_root)
        path = os.path.join(res.model_dir, 'URDF', 'toy_arm.urdf')
        info = UrdfInfo(open(path).read(), path)
        assert info.joints['left_finger_joint'].type == 'fixed'
        assert info.joints['joint1'].type == 'revolute'
        uris = {r.uri for r in info.mesh_refs}
        assert all(u.startswith('meshes/') for u in uris)
        assert all(os.path.isfile(os.path.join(os.path.dirname(path), u)) for u in uris)
        # joint2 came without an effort, which urdfdom (and so the
        # controllers) would refuse; the saved copy has one.
        assert info.joints['joint2'].effort == mjcf.DEFAULT_EFFORT
        assert info.joints['joint2'].velocity == 1.0          # kept as given
        assert any('joint2' in m and 'effort' in m for _, m in res.messages)

    def test_fill_limits_adds_what_urdfdom_needs(self):
        root = ET.fromstring(
            '<robot name="r"><joint name="a" type="revolute"/>'
            '<joint name="b" type="prismatic"><limit lower="0" upper="1" effort="3"/></joint>'
            '<joint name="c" type="continuous"><limit lower="-9" effort="4" velocity="1"/>'
            '</joint></robot>')
        plan = type('P', (), {'joints': []})()
        assert importer._fill_limits(root, plan) == (['a', 'b'], ['c'])
        a, b = root.find("joint[@name='a']/limit"), root.find("joint[@name='b']/limit")
        assert a.get('effort') == '50' and a.get('velocity') == '2'
        assert a.get('lower') is None               # unbounded: continuous to FIC
        assert b.get('effort') == '3' and b.get('velocity') == '2'
        c = root.find("joint[@name='c']")
        assert c.get('type') == 'revolute' and c.find('limit').get('lower') is None
        assert c.find('limit').get('effort') == '4'

    def test_continuous_joint_is_one_coordinate_for_the_controllers(self, pkg, model_root):
        """joint3 is continuous: Pinocchio must see one q per joint, as FIC indexes."""
        pin = pytest.importorskip('pinocchio')
        _, res = self._create(pkg, model_root)
        path = os.path.join(res.model_dir, 'URDF', 'toy_arm.urdf')
        model = pin.buildModelFromUrdf(path)
        assert model.nq == model.nv == 3
        info = UrdfInfo(open(path).read(), path)
        assert info.joints['joint3'].type == 'revolute' and info.joints['joint3'].continuous
        # MuJoCo keeps it a free-turning hinge.
        m = mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'joint3')
        assert not m.jnt_limited[jid]

    def test_config_entry(self, pkg, model_root):
        plan, res = self._create(pkg, model_root)
        cfg = res.config
        required = {
            'display_name', 'num_joints', 'joint_names', 'urdf_path', 'mujoco_xml_path',
            'ee_frame_name', 'elbow_frame_name', 'default_ee_target', 'q_home_deg',
            'q_home_rad', 'joint_limits_deg', 'joint_torque_limits', 'torque_sign',
            'gravity_compensated', 'wrench_gravity_compensated', 'admittance_deadbands',
            'has_gripper', 'gripper_range', 'is_supported', 'driver_command',
            'build_directory', 'build_command', 'safety_zones', 'kinematics_solver',
        }
        assert required <= set(cfg)
        assert cfg['urdf_path'] == 'menagerie/model/toy_arm/URDF/toy_arm.urdf'
        assert cfg['mujoco_xml_path'] == 'menagerie/model/toy_arm/xml/scene.xml'
        assert cfg['num_joints'] == 3 and len(cfg['q_home_deg']) == 3
        assert cfg['joint_torque_limits']['max'] == [40.0, 50.0, 40.0]
        assert cfg['joint_limits_deg']['upper'][2] == 360.0
        assert cfg['is_supported'] is False
        assert cfg['has_gripper'] is True and cfg['gripper_range'] == [0.0, 0.08]
        sz = cfg['safety_zones']
        assert sz['joint_safety_deg']['lower'][0] == pytest.approx(-114.59 + 5, abs=0.01)
        assert sz['joint_safety_deg']['upper'][2] == 360.0      # continuous: untouched
        assert sz['joint_safety_deg']['enabled'] is False
        assert sz['velocity']['max_joint_vel'] == 1.0 and sz['velocity']['enabled']
        reach = sz['cartesian']['x']['max']
        assert 0.6 < reach < 1.0 and sz['cartesian']['x']['min'] == -reach
        assert not any(sz['cartesian'][a]['enabled'] for a in 'xyz')
        q = cfg['default_ee_target']['orientation']
        assert sum(v * v for v in q) == pytest.approx(1.0, abs=1e-3) and q[3] >= 0

    def test_default_target_is_the_home_pose(self, pkg, model_root):
        plan = importer.analyse(pkg)
        plan.joints[1].home_deg = 30.0
        res = importer.create_robot(plan, model_root)
        # tool0 sits 0.15 m past joint3 along link3's z; with joint2 at 30°
        # about y the tool leans forward: check with an independent FK.
        import math
        a = math.radians(30.0)
        x = math.sin(a) * (0.2 + 0.15)
        z = 0.1 + 0.2 + math.cos(a) * (0.2 + 0.15)
        pos = res.config['default_ee_target']['position']
        assert pos == pytest.approx([x, 0.0, z], abs=1e-3)
        assert res.config['q_home_deg'][1] == 30.0

    def test_default_target_matches_pinocchio(self, pkg, model_root):
        pin = pytest.importorskip('pinocchio')
        import numpy as np
        _, res = self._create(pkg, model_root)
        pm = pin.buildModelFromUrdf(os.path.join(res.model_dir, 'URDF', 'toy_arm.urdf'))
        pd = pm.createData()
        pin.framesForwardKinematics(pm, pd, pin.neutral(pm))
        T = pd.oMf[pm.getFrameId('tool0')]
        assert np.allclose(T.translation, res.config['default_ee_target']['position'],
                           atol=1e-3)

    def test_gripper_opens_to_the_width_the_bridge_asks_for(self, pkg, model_root):
        """Open to the width asked: ctrl = 10 x width, each finger takes half."""
        import numpy as np
        _, res = self._create(pkg, model_root)
        m = mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))
        d = mujoco.MjData(m)
        grip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper')
        left, right = (m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                       for n in ('left_finger_joint', 'right_finger_joint'))
        arm = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'torq_j%d' % k)
               for k in (1, 2, 3)]
        for width in (0.06, 0.02):
            d.ctrl[grip] = 10.0 * width
            for _ in range(1500):
                d.ctrl[arm] = 0.0
                d.qfrc_applied[:] = d.qfrc_bias       # hold the arm still
                mujoco.mj_step(m, d)
            assert np.isclose(d.qpos[left], width / 2, atol=2e-3), (width, d.qpos[left])
            assert np.isclose(d.qpos[right], width / 2, atol=2e-3)

    def test_srdf_gives_home_and_end_effector(self, tmp_path, model_root):
        """A sibling moveit_config's SRDF sets the home pose and the tool."""
        import math
        src = str(tmp_path / 'src')
        pkg = make_package(src)
        cfg_dir = os.path.join(src, 'toy_arm_moveit_config', 'config')
        os.makedirs(cfg_dir)
        with open(os.path.join(cfg_dir, 'toy_arm.srdf'), 'w') as fh:
            fh.write('<robot name="toy_arm"><group name="arm"><chain base_link="base_link" '
                     'tip_link="tool0"/><joint name="joint1"/><joint name="joint2"/>'
                     '<joint name="joint3"/></group>'
                     '<group_state name="extended" group="arm"><joint name="joint1" value="0"/>'
                     '<joint name="joint2" value="0"/><joint name="joint3" value="0"/>'
                     '</group_state>'
                     '<group_state name="ready" group="arm"><joint name="joint1" value="0.5"/>'
                     '<joint name="joint2" value="-0.3"/><joint name="joint3" value="1.0"/>'
                     '</group_state>'
                     '<end_effector name="hand" parent_link="link3" group="hand"/></robot>')
        plan = importer.analyse(os.path.join(pkg, 'urdf', 'toy_arm.urdf'))
        assert plan.ee_link == 'link3' and 'SRDF' in plan.ee_source
        assert [j.home_deg for j in plan.joints] == pytest.approx(
            [math.degrees(0.5), math.degrees(-0.3), math.degrees(1.0)], abs=1e-3)
        assert '"ready"' in plan.home_source
        assert any(row[0] == 'Home pose' and 'ready' in row[2] for row in plan.found)
        res = importer.create_robot(plan, model_root)
        assert res.config['q_home_deg'][0] == pytest.approx(math.degrees(0.5), abs=1e-3)

    def test_ros2_control_initial_positions_give_home(self, tmp_path):
        text = arm_urdf().replace('</robot>', (
            '<ros2_control name="toy" type="system">'
            '<joint name="joint1"><state_interface name="position">'
            '<param name="initial_value">0.25</param></state_interface></joint>'
            '<joint name="joint2"><state_interface name="position">'
            '<param name="initial_value">$(arg j2)</param></state_interface></joint>'
            '</ros2_control></robot>'))
        plan = importer.analyse(make_package(str(tmp_path), text))
        assert plan.joints[0].home_deg == pytest.approx(14.324, abs=1e-3)
        assert plan.joints[1].home_deg == 0.0         # not a number: ignored
        assert 'ros2_control' in plan.home_source

    def test_home_outside_limits_is_clamped(self, tmp_path):
        text = arm_urdf().replace('</robot>', (
            '<ros2_control name="toy" type="system"><joint name="joint1">'
            '<state_interface name="position"><param name="initial_value">9</param>'
            '</state_interface></joint></ros2_control></robot>'))
        plan = importer.analyse(make_package(str(tmp_path), text))
        assert plan.joints[0].home_deg == pytest.approx(114.592, abs=1e-3)

    def test_revolute_gripper_is_held_and_said_so(self, tmp_path):
        text = arm_urdf().replace('type="prismatic"', 'type="revolute"')
        plan = importer.analyse(make_package(str(tmp_path), text))
        assert plan.gripper is None
        assert any('held fixed in simulation' in m for _, m in plan.messages)

    def test_speed_limit_never_above_default_or_urdf(self, tmp_path):
        text = arm_urdf().replace('velocity="1"', 'velocity="0.4"', 1)
        plan = importer.analyse(make_package(str(tmp_path), text))
        assert importer.speed_limit(plan) == 0.4
        assert plan.joints[0].velocity == 0.4

    def test_holds_still_with_gravity_compensation(self, pkg, model_root):
        _, res = self._create(pkg, model_root)
        assert res.stats['finite']
        assert res.stats['hold_drift_deg'] < 0.5

    def test_dae_converted(self, tmp_path, model_root):
        pkg = make_package(str(tmp_path), arm_urdf(
            mesh=lambda n: 'package://%s/meshes/%s.stl' % (PKG, n) if n != 'link2'
            else 'package://%s/meshes/visual/link1.dae' % PKG), dae=True)
        plan = importer.analyse(pkg)
        assert any('.dae' in m for _, m in plan.messages)
        res = importer.create_robot(plan, model_root)
        assets = os.listdir(os.path.join(res.model_dir, 'xml', 'assets'))
        assert 'link1_2.obj' in assets or 'link1.obj' in assets
        mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))

    def test_missing_mesh_part_left_out(self, tmp_path, model_root):
        pkg = make_package(str(tmp_path), arm_urdf(
            mesh=lambda n: 'package://%s/meshes/%s.stl' % (PKG, 'nope' if n == 'link2' else n)))
        res = importer.create_robot(importer.analyse(pkg), model_root)
        mujoco.MjModel.from_xml_path(os.path.join(res.model_dir, 'xml', 'scene.xml'))

    def test_refuses_existing_folder(self, pkg, model_root):
        os.makedirs(os.path.join(model_root, 'toy_arm'))
        plan = importer.analyse(pkg)
        with pytest.raises(importer.ImportError_, match='already exists'):
            importer.create_robot(plan, model_root)
        res = importer.create_robot(plan, model_root, overwrite=True)
        assert os.path.isfile(os.path.join(res.model_dir, 'xml', 'scene.xml'))

    def test_failure_leaves_nothing_behind(self, pkg, model_root, monkeypatch):
        def boom(*a, **k):
            raise mjcf.MjcfError('simulated failure')
        monkeypatch.setattr(mjcf, 'validate', boom)
        with pytest.raises(mjcf.MjcfError):
            importer.create_robot(importer.analyse(pkg), model_root)
        assert os.listdir(model_root) == []

    def test_refuses_bad_id_and_errors(self, pkg, model_root):
        plan = importer.analyse(pkg)
        plan.robot_id = '9 lives'
        with pytest.raises(importer.ImportError_, match='robot ID'):
            importer.create_robot(plan, model_root)
        plan.robot_id = 'ok'
        plan.messages.append((importer.ERROR, 'blocked'))
        with pytest.raises(importer.ImportError_, match='blocked'):
            importer.create_robot(plan, model_root)

    def test_report_lists_joints(self, pkg, model_root):
        _, res = self._create(pkg, model_root)
        text = open(res.report_path).read()
        assert '`joint2`' in text and '(default)' in text and 'tool0' in text


# ── Pieces ────────────────────────────────────────────────────────────────

class TestPieces:

    def test_make_robot_id(self):
        assert importer.make_robot_id('my robot-v2') == 'my_robot_v2'
        assert importer.make_robot_id('7axis') == 'Robot_7axis'
        assert importer.make_robot_id('arm', ['ARM']) == 'arm_2'
        assert importer.valid_robot_id('Franka_FR3')
        assert not importer.valid_robot_id('has space')

    def test_preferred_name(self):
        assert importer._preferred_name('GEN3_7DOF_NOVISION_FOR_URDF_ARM_V12',
                                        '/x/KinovaGen3_7DOF.urdf') == 'KinovaGen3_7DOF'
        assert importer._preferred_name('fr3', '/x/Franka_FR3.urdf') == 'fr3'
        assert importer._preferred_name('A_VERY_LONG_CAD_EXPORT_NAME_V3',
                                        '/x/robot.urdf') == 'A_VERY_LONG_CAD_EXPORT_NAME_V3'

    def test_guess_ee_ignores_fingers(self):
        info = UrdfInfo(arm_urdf(), 'toy.urdf')
        assert info.guess_ee() == 'tool0'
        assert info.is_gripper_joint(info.joints['left_finger_joint'])

    def test_two_bases_rejected(self):
        with pytest.raises(UrdfError, match='one tree'):
            UrdfInfo('<robot name="r"><link name="a"/><link name="b"/></robot>', 'r.urdf')

    def test_mesh_finder_foreign_absolute_path(self, pkg):
        finder = MeshFinder(os.path.join(pkg, 'urdf', 'toy_arm.urdf'), packages={PKG: pkg})
        uri = 'file:///root/other_ws/install/%s/share/%s/meshes/link1.stl' % (PKG, PKG)
        assert finder.resolve(uri) == os.path.join(pkg, 'meshes', 'link1.stl')

    def test_mesh_finder_case_insensitive_name(self, pkg):
        finder = MeshFinder(os.path.join(pkg, 'urdf', 'toy_arm.urdf'))
        assert finder.resolve('package://gone_pkg/meshes/LINK1.STL') == \
            os.path.join(pkg, 'meshes', 'link1.stl')

    def test_find_packages(self, pkg):
        assert find_packages([os.path.dirname(pkg)]) == {PKG: pkg}

    def test_dae_to_obj_units_and_up_axis(self, tmp_path):
        dae = str(tmp_path / 'm.dae')
        obj = str(tmp_path / 'm.obj')
        write_dae(dae, size=(10, 20, 30), up='Y_UP', meter=0.01)
        assert dae_to_obj(dae, obj) == 12
        verts = [tuple(map(float, line.split()[1:])) for line in open(obj)
                 if line.startswith('v ')]
        xs, ys, zs = zip(*verts)
        # 10 x 20 x 30 cm box, Y up -> Z up: the 20 cm side becomes vertical.
        assert max(xs) - min(xs) == pytest.approx(0.10)
        assert max(zs) - min(zs) == pytest.approx(0.20)
        assert max(ys) - min(ys) == pytest.approx(0.30)
