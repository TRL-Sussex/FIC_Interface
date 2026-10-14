"""Convert a COLLADA (.dae) mesh to OBJ, which MuJoCo can load.

Handles what robot exporters produce: <triangles>, <polylist> and <polygons>
geometry, placed by the <visual_scene> node tree (matrix / translate / rotate
/ scale), with the file's <unit> and <up_axis> applied the way RViz applies
them. Materials, normals and textures are dropped: MuJoCo recomputes normals
and the importer colours geoms from the URDF.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


class DaeError(Exception):
    pass


def _strip_ns(root):
    for el in root.iter():
        if '}' in el.tag:
            el.tag = el.tag.split('}', 1)[1]
    return root


def _floats(text):
    return np.array([float(v) for v in (text or '').split()], dtype=float)


def _node_matrix(node):
    m = np.eye(4)
    for child in node:
        tag = child.tag
        if tag == 'matrix':
            m = m @ _floats(child.text).reshape(4, 4)
        elif tag == 'translate':
            t = np.eye(4)
            t[:3, 3] = _floats(child.text)[:3]
            m = m @ t
        elif tag == 'scale':
            m = m @ np.diag(list(_floats(child.text)[:3]) + [1.0])
        elif tag == 'rotate':
            x, y, z, ang = _floats(child.text)[:4]
            n = math.sqrt(x * x + y * y + z * z) or 1.0
            x, y, z = x / n, y / n, z / n
            c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))
            r = np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                          [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                          [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])
            t = np.eye(4)
            t[:3, :3] = r
            m = m @ t
    return m


def _geometry_triangles(geom):
    """(vertices Nx3, faces Mx3) for one <geometry>."""
    mesh = geom.find('mesh')
    if mesh is None:
        return None
    sources = {s.get('id'): s for s in mesh.findall('source')}
    vertices = mesh.find('vertices')
    pos_src = None
    if vertices is not None:
        for inp in vertices.findall('input'):
            if inp.get('semantic') == 'POSITION':
                pos_src = sources.get(inp.get('source', '').lstrip('#'))
    if pos_src is None:
        return None
    arr = pos_src.find('float_array')
    acc = pos_src.find('technique_common/accessor')
    stride = int(acc.get('stride', 3)) if acc is not None else 3
    positions = _floats(arr.text).reshape(-1, stride)[:, :3]

    faces = []
    for prim in list(mesh):
        if prim.tag not in ('triangles', 'polylist', 'polygons'):
            continue
        inputs = prim.findall('input')
        if not inputs:
            continue
        n_off = max(int(i.get('offset', 0)) for i in inputs) + 1
        v_off = next((int(i.get('offset', 0)) for i in inputs
                      if i.get('semantic') == 'VERTEX'), None)
        if v_off is None:
            continue
        if prim.tag == 'polygons':
            polys = [_floats(p.text).astype(int).reshape(-1, n_off)[:, v_off]
                     for p in prim.findall('p')]
        else:
            p = prim.find('p')
            if p is None:
                continue
            idx = _floats(p.text).astype(int).reshape(-1, n_off)[:, v_off]
            if prim.tag == 'triangles':
                polys = list(idx.reshape(-1, 3))
            else:
                counts = _floats(prim.find('vcount').text).astype(int)
                polys, start = [], 0
                for c in counts:
                    polys.append(idx[start:start + c])
                    start += c
        for poly in polys:           # fan-triangulate
            for k in range(1, len(poly) - 1):
                faces.append((poly[0], poly[k], poly[k + 1]))
    if not faces:
        return None
    return positions, np.array(faces, dtype=int)


def dae_to_obj(dae_path, obj_path):
    """Write an OBJ of every mesh in ``dae_path``. Returns the triangle count."""
    try:
        root = _strip_ns(ET.parse(dae_path).getroot())
    except (ET.ParseError, OSError) as exc:
        raise DaeError('cannot read %s: %s' % (dae_path, exc))
    unit_el = root.find('asset/unit')
    unit = float(unit_el.get('meter', 1.0)) if unit_el is not None else 1.0
    up = (root.findtext('asset/up_axis') or 'Y_UP').strip().upper()
    fix = np.eye(4)
    if up == 'Y_UP':                  # rotate Y-up into Z-up
        fix[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    elif up == 'X_UP':
        fix[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    fix = np.diag([unit, unit, unit, 1.0]) @ fix

    geoms = {g.get('id'): g for g in root.iter('geometry')}
    placements = []                   # (geometry id, 4x4 world matrix)

    def walk(node, parent):
        m = parent @ _node_matrix(node)
        for inst in node.findall('instance_geometry'):
            placements.append((inst.get('url', '').lstrip('#'), m))
        for child in node.findall('node'):
            walk(child, m)

    scene = root.find('library_visual_scenes/visual_scene')
    if scene is not None:
        for node in scene.findall('node'):
            walk(node, np.eye(4))
    if not placements:                # no scene graph: take geometry as-is
        placements = [(gid, np.eye(4)) for gid in geoms]

    all_v, all_f, offset = [], [], 0
    for gid, m in placements:
        geom = geoms.get(gid)
        tri = _geometry_triangles(geom) if geom is not None else None
        if tri is None:
            continue
        v, f = tri
        world = (fix @ m @ np.c_[v, np.ones(len(v))].T).T[:, :3]
        all_v.append(world)
        all_f.append(f + offset)
        offset += len(v)
    if not all_f:
        raise DaeError('no triangle geometry found in %s' % dae_path)
    v = np.vstack(all_v)
    f = np.vstack(all_f)
    with open(obj_path, 'w') as fh:
        fh.write('# converted from %s by FIC Control Studio\n' % dae_path.split('/')[-1])
        fh.writelines('v %.7g %.7g %.7g\n' % tuple(p) for p in v)
        fh.writelines('f %d %d %d\n' % (a + 1, b + 1, c + 1) for a, b, c in f)
    return len(f)
