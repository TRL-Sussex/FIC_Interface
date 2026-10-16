"""Playground scenes load in MuJoCo for every usable robot, keep the robot's names, and put the
objects on the table; robots the controller cannot drive are listed with the reason."""
import os

import mujoco
import pytest
import yaml

from Extensions.mujoco_playground import scene_builder as sb

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
WS_SRC = os.path.join(REPO, 'fic_ws', 'src')
CONFIGS = yaml.safe_load(open(os.path.join(REPO, 'fic_library', 'config', 'robots_config.yaml')))
CONFIGS = CONFIGS.get('robots', CONFIGS)


def _usable():
    return [r for r in sb.robots(CONFIGS, WS_SRC) if r['ok']]


def test_the_arms_are_usable_and_the_others_say_why():
    rows = {r['name']: r for r in sb.robots(CONFIGS, WS_SRC)}
    assert rows['KinovaGen3_7DOF']['ok'] and rows['KinovaGen3_7DOF']['gripper']
    assert rows['Franka_FR3']['ok'] and not rows['Franka_FR3']['gripper']
    if 'IIWA_14' in rows:                      # robots the public repository may leave out
        assert not rows['IIWA_14']['ok'] and 'torque motors' in rows['IIWA_14']['reason']
    if 'Spot' in rows:
        assert not rows['Spot']['ok'] and rows['Spot']['reason']


@pytest.mark.parametrize('preset', sorted(sb.PRESETS))
def test_every_preset_loads_for_every_usable_robot(tmp_path, preset):
    _, table, objects = sb.PRESETS[preset]
    for r in _usable():
        path = sb.build(r['scene'], objects, table=table, out_path=str(tmp_path / (r['name'] + '.xml')))
        m = mujoco.MjModel.from_xml_path(path)
        robot = mujoco.MjModel.from_xml_path(r['scene'])
        # the robot is unchanged: its joints and actuators keep their names, in order
        assert [m.actuator(i).name for i in range(robot.nu)] == [robot.actuator(i).name for i in range(robot.nu)]
        assert [m.joint(i).name for i in range(robot.njnt)] == [robot.joint(i).name for i in range(robot.njnt)]
        assert {m.body(i).name for i in range(m.nbody)} >= {sb.OBJECT_PREFIX + o['name'] for o in objects}


def test_objects_rest_on_the_table_and_stay_there(tmp_path):
    _, table, objects = sb.PRESETS['shapes']
    kinova = next(r for r in _usable() if r['name'] == 'KinovaGen3_7DOF')
    m = mujoco.MjModel.from_xml_path(sb.build(kinova['scene'], objects, table=table,
                                              out_path=str(tmp_path / 'k.xml')))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    start = {o['name']: d.xpos[m.body(sb.OBJECT_PREFIX + o['name']).id].copy() for o in objects}
    for o in objects:                                   # bottom on the table top (+1 mm)
        half = sb.SHAPES[o['shape']]['half_height'](o['size'])
        assert start[o['name']][2] - half == pytest.approx(sb.TABLE['height'] + 0.001, abs=1e-6)
    m.opt.gravity[:] = (0, 0, -9.81)
    for _ in range(500):                                # 1 s with the arm limp: nothing falls or sinks
        mujoco.mj_step(m, d)
    for o in objects:
        assert abs(d.xpos[m.body(sb.OBJECT_PREFIX + o['name']).id][2] - start[o['name']][2]) < 0.005


def test_bad_objects_are_refused():
    with pytest.raises(ValueError):
        sb.check_object({'name': 'x', 'shape': 'cone'})
    with pytest.raises(ValueError):
        sb.check_object({'name': 'x', 'shape': 'box', 'size': [1, 1, 1]})
    with pytest.raises(ValueError):
        sb.check_object({'name': '', 'shape': 'box'})
    with pytest.raises(ValueError):
        sb.build('/nonexistent.xml', [{'name': 'a'}, {'name': 'a'}])


def test_the_scene_is_written_outside_the_repository(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    kinova = next(r for r in _usable() if r['name'] == 'KinovaGen3_7DOF')
    path = sb.build(kinova['scene'], [], table=True)
    assert path.startswith(str(tmp_path)) and not path.startswith(REPO)


def _box_stl(lo, hi):
    """A binary STL of the box lo..hi (12 triangles)."""
    import struct
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    out = b'\0' * 80 + struct.pack('<I', len(faces))
    for f in faces:
        out += struct.pack('<3f', 0, 0, 0) + b''.join(struct.pack('<3f', *v[i]) for i in f) + b'\0\0'
    return out


# An L-shaped block as OBJ text, in metres, its corner at the origin
_L_OBJ = '\n'.join(['v 0 0 0', 'v 0.06 0 0', 'v 0.06 0.02 0', 'v 0.02 0.02 0', 'v 0.02 0.05 0', 'v 0 0.05 0',
                    'v 0 0 0.03', 'v 0.06 0 0.03', 'v 0.06 0.02 0.03', 'v 0.02 0.02 0.03', 'v 0.02 0.05 0.03',
                    'v 0 0.05 0.03', 'f 1 2 3', 'f 1 3 4', 'f 1 4 5', 'f 1 5 6', 'f 7 9 8', 'f 7 10 9',
                    'f 7 11 10', 'f 7 12 11', 'f 1 2 8', 'f 1 8 7', 'f 2 3 9', 'f 2 9 8', 'f 3 4 10',
                    'f 3 10 9', 'f 4 5 11', 'f 4 11 10', 'f 5 6 12', 'f 5 12 11', 'f 6 1 7', 'f 6 7 12']) + '\n'


def test_dropped_meshes_are_stored_with_their_units_guessed(tmp_path):
    stl = sb.save_mesh('My Part.STL', _box_stl((100, 200, 0), (140, 230, 50)), mesh_dir=str(tmp_path))
    assert stl['mesh'] == 'my_part.stl' and stl['units'] == 'mm'
    assert stl['size'] == pytest.approx([0.04, 0.03, 0.05], abs=1e-6)
    obj = sb.save_mesh('bracket.obj', _L_OBJ.encode(), mesh_dir=str(tmp_path))
    assert obj['units'] == 'm' and obj['size'] == pytest.approx([0.06, 0.05, 0.03], abs=1e-6)
    assert sb.meshes(str(tmp_path)) == ['bracket.obj', 'my_part.stl']


def test_files_mujoco_cannot_use_are_refused(tmp_path):
    with pytest.raises(ValueError, match='STL, OBJ'):
        sb.save_mesh('part.step', b'ISO-10303-21;', mesh_dir=str(tmp_path))
    with pytest.raises(ValueError, match='cannot read'):
        sb.save_mesh('junk.stl', b'not a mesh at all', mesh_dir=str(tmp_path))
    with pytest.raises(ValueError, match='empty'):
        sb.save_mesh('empty.obj', b'', mesh_dir=str(tmp_path))
    assert sb.meshes(str(tmp_path)) == []
    with pytest.raises(ValueError):                      # only stored files, by bare name
        sb.check_object({'name': 'x', 'shape': 'mesh', 'mesh': '../etc/passwd.stl'}, mesh_dir=str(tmp_path))
    sb.save_mesh('big.stl', _box_stl((0, 0, 0), (900, 900, 900)), mesh_dir=str(tmp_path))
    with pytest.raises(ValueError, match='across'):      # 0.9 m: too big for the table
        sb.check_object({'name': 'x', 'shape': 'mesh', 'mesh': 'big.stl', 'units': 'mm'}, mesh_dir=str(tmp_path))


def _sphere_tris(n):
    """A UV sphere of radius 30 (mm) with 2*n*n triangles, as an (m, 3, 3) array."""
    import numpy as np
    th, ph = np.linspace(0, np.pi, n + 1), np.linspace(0, 2 * np.pi, n + 1)
    P = 30 * np.stack([np.sin(th)[:, None] * np.cos(ph), np.sin(th)[:, None] * np.sin(ph),
                       np.cos(th)[:, None] * np.ones_like(ph)], axis=-1)
    a, b, c, d = P[:-1, :-1], P[1:, :-1], P[1:, 1:], P[:-1, 1:]
    return np.concatenate([np.stack([a, b, c], -2).reshape(-1, 3, 3), np.stack([a, c, d], -2).reshape(-1, 3, 3)])


def test_a_mesh_over_mujocos_triangle_limit_is_simplified(tmp_path, monkeypatch):
    monkeypatch.setattr(sb, 'MAX_STL_FACES', 2000)          # MuJoCo's own limit is 200,000: the same path, faster
    monkeypatch.setattr(sb, 'SIMPLIFY_TO', 1500)
    data = sb.stl_bytes(_sphere_tris(60))                     # 7,200 triangles
    assert sb.stl_faces(data) == 7200 and len(sb.read_stl(data)) == 7200
    info = sb.save_mesh('Ball.stl', data, mesh_dir=str(tmp_path))
    kept = sb.stl_faces(open(tmp_path / info['mesh'], 'rb').read())
    assert 300 < kept <= 1500 and 'simplified from 7,200' in info['note']
    assert info['units'] == 'mm' and info['size'] == pytest.approx([0.06, 0.06, 0.06], abs=0.003)


def test_text_stl_reads_and_writes_back_as_binary():
    tris = sb.read_stl(_box_stl((0, 0, 0), (1, 2, 3)))
    text = 'solid t\n' + ''.join('facet normal 0 0 0\nouter loop\n' + ''.join('vertex %g %g %g\n' % tuple(v) for v in t)
                                  + 'endloop\nendfacet\n' for t in tris) + 'endsolid t\n'
    assert sb.stl_faces(text.encode()) == 12
    assert sb.read_stl(text.encode()) == pytest.approx(tris)
    assert sb.read_stl(sb.stl_bytes(tris)) == pytest.approx(tris)


def test_other_mesh_formats_become_stl(tmp_path):
    trimesh = pytest.importorskip('trimesh')
    ply = trimesh.creation.box((40, 30, 50)).export(file_type='ply')
    info = sb.save_mesh('Block.ply', ply, mesh_dir=str(tmp_path))
    assert info['mesh'] == 'block.stl' and 'converted from PLY' in info['note']
    assert info['size'] == pytest.approx([0.04, 0.03, 0.05], abs=1e-6)
    assert '.ply' in sb.upload_types()


def test_other_formats_without_trimesh_say_what_to_do(monkeypatch):
    monkeypatch.setattr(sb, '_trimesh', lambda: None)
    assert sb.upload_types() == ('.stl', '.obj')
    with pytest.raises(ValueError, match='need trimesh'):
        sb.check_upload_name('part.glb')


def test_mesh_objects_sit_centred_on_their_pose_and_rest_on_the_table(tmp_path):
    sb.save_mesh('part.stl', _box_stl((100, 200, 0), (140, 230, 50)), mesh_dir=str(tmp_path))
    sb.save_mesh('bracket.obj', _L_OBJ.encode(), mesh_dir=str(tmp_path))
    objects = [{'name': 'part', 'shape': 'mesh', 'mesh': 'part.stl', 'units': 'mm', 'x': 0.5, 'y': -0.1, 'mass': 0.1},
               {'name': 'bracket', 'shape': 'mesh', 'mesh': 'bracket.obj', 'units': 'm', 'x': 0.5, 'y': 0.1}]
    kinova = next(r for r in _usable() if r['name'] == 'KinovaGen3_7DOF')
    m = mujoco.MjModel.from_xml_path(sb.build(kinova['scene'], objects, table=True, mesh_dir=str(tmp_path),
                                              out_path=str(tmp_path / 'k.xml')))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for o, height in (('part', 0.05), ('bracket', 0.03)):
        g = m.geom(sb.OBJECT_PREFIX + o + '_geom').id
        adr, n = m.mesh_vertadr[m.geom_dataid[g]], m.mesh_vertnum[m.geom_dataid[g]]
        pts = d.geom_xpos[g] + m.mesh_vert[adr:adr + n] @ d.geom_xmat[g].reshape(3, 3).T
        body = d.xpos[m.body(sb.OBJECT_PREFIX + o).id]
        centre = (pts.min(0) + pts.max(0)) / 2
        assert centre == pytest.approx(body, abs=1e-5)                              # pose = middle of the mesh
        assert pts[:, 2].min() == pytest.approx(sb.TABLE['height'] + 0.001, abs=1e-5)   # bottom on the table
        assert pts[:, 2].max() - pts[:, 2].min() == pytest.approx(height, abs=1e-5)
    assert m.body(sb.OBJECT_PREFIX + 'part').mass[0] == pytest.approx(0.1)
    m.opt.gravity[:] = (0, 0, -9.81)
    start = d.xpos.copy()
    for _ in range(500):                                # 1 s: they stay on the table
        mujoco.mj_step(m, d)
    for o in ('part', 'bracket'):
        i = m.body(sb.OBJECT_PREFIX + o).id
        assert abs(d.xpos[i][2] - start[i][2]) < 0.005
