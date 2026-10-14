"""Robot Config drag and drop: the upload action stages a dropped tree, then the
import pipeline (mocked here) is handed the staged path."""
import base64
import io
import os
import types
import zipfile

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest  # noqa: E402
from fic_studio.web_remote import actions, robot_config  # noqa: E402
from test_web_parity import LOCAL, WHO, _Studio, run  # noqa: E402,F401


@pytest.fixture
def studio(tmp_path):
    w = _Studio(tmp_path)
    yield w
    for uid in list(getattr(w, '_web_uploads', {})):
        robot_config.discard_upload(w, uid)


def b64(data):
    return base64.b64encode(data).decode()


def begin(studio, who=LOCAL):
    r = run(studio, 'robots_upload', who=who, op='begin')
    assert r['ok'], r
    return r['upload']


def put(studio, uid, path, data, offset=0):
    return run(studio, 'robots_upload', op='chunk', upload=uid, path=path, offset=offset, data=b64(data))


def staged(studio, uid):
    return studio._web_uploads[uid]['dir']


def test_multi_file_nested_folder_and_chunked_file_are_staged(studio):
    uid = begin(studio)
    assert put(studio, uid, 'ur5/ur5.urdf', b'<robot name="ur5"/>')['ok']
    assert put(studio, uid, 'ur5/meshes/visual/base.stl', b'\x00' * 10)['ok']
    assert put(studio, uid, 'ur5/empty.txt', b'')['ok']
    assert put(studio, uid, 'ur5/big.bin', b'a' * 5)['ok']
    assert put(studio, uid, 'ur5/big.bin', b'b' * 5, offset=5)['ok']                 # a second chunk of the same file
    root = staged(studio, uid)
    assert open(os.path.join(root, 'ur5/ur5.urdf'), 'rb').read() == b'<robot name="ur5"/>'
    assert os.path.isfile(os.path.join(root, 'ur5/meshes/visual/base.stl'))
    assert open(os.path.join(root, 'ur5/big.bin'), 'rb').read() == b'aaaaabbbbb'
    assert os.path.getsize(os.path.join(root, 'ur5/empty.txt')) == 0


def test_a_zip_is_staged_byte_for_byte(studio):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, 'w') as z:
        z.writestr('arm/arm.urdf', '<robot name="arm"/>')
    data = buf.getvalue()
    uid = begin(studio)
    assert put(studio, uid, 'arm.zip', data)['ok']
    path = os.path.join(staged(studio, uid), 'arm.zip')
    assert zipfile.ZipFile(path).namelist() == ['arm/arm.urdf']
    assert robot_config.staged_source(staged(studio, uid)) == (path, None)


def test_a_remote_client_is_refused(studio):
    for op in ('begin', 'chunk', 'cancel'):
        assert run(studio, 'robots_upload', who=WHO, op=op)['error'] == 'local_only'
    assert not getattr(studio, '_web_uploads', {})


@pytest.mark.parametrize('name', ['../evil.urdf', 'a/../../evil', '/etc/passwd', 'C:/x/y.urdf', 'a/./../..', '..', '', 'a\\..\\..\\evil', 'x\0y'])
def test_path_traversal_in_an_uploaded_name_is_rejected(studio, name):
    uid = begin(studio)
    reply = put(studio, uid, name, b'x')
    assert reply['ok'] is False and reply['error'] == 'bad_args'
    assert os.listdir(staged(studio, uid)) == []
    parent = os.path.dirname(staged(studio, uid))
    assert not os.path.exists(os.path.join(parent, 'evil.urdf')) and not os.path.exists(os.path.join(parent, 'evil'))


def test_chunks_must_arrive_in_order_and_unknown_uploads_are_refused(studio):
    uid = begin(studio)
    assert put(studio, uid, 'a.urdf', b'123', offset=7)['error'] == 'bad_args'
    assert put(studio, 'nope', 'a.urdf', b'1')['error'] == 'bad_args'
    assert run(studio, 'robots_upload', op='chunk', upload=uid, path='a', offset=0, data='***')['error'] == 'bad_args'


def test_the_size_limit_discards_the_staging_folder(studio, monkeypatch):
    monkeypatch.setattr(robot_config, 'MAX_UPLOAD_BYTES', 10)
    uid = begin(studio)
    root = staged(studio, uid)
    assert put(studio, uid, 'a.bin', b'x' * 8)['ok']
    reply = put(studio, uid, 'b.bin', b'x' * 8)
    assert reply['error'] == 'too_large'
    assert not os.path.exists(root) and uid not in studio._web_uploads


def test_cancel_removes_the_staging_folder(studio):
    uid = begin(studio)
    put(studio, uid, 'a.urdf', b'1')
    root = staged(studio, uid)
    assert run(studio, 'robots_upload', op='cancel', upload=uid)['ok']
    assert not os.path.exists(root)


def test_loose_files_with_a_mujoco_model_go_in_like_the_classic_drop_zone(studio, tmp_path):
    uid = begin(studio)
    put(studio, uid, 'arm.urdf', b'<robot name="arm"/>')
    put(studio, uid, 'arm.xml', b'<mujoco model="arm"><worldbody/></mujoco>')
    path, mjcf = robot_config.staged_source(staged(studio, uid))
    assert path.endswith('arm.urdf') and mjcf.endswith('arm.xml')


class _FakePlan:
    robot_id, display_name, ee_link, elbow_link = 'dropped', 'Dropped', 'tool', 'elbow'
    urdf_path, messages, errors, mesh_counts, closed_loop, urdf_candidates = '/x.urdf', [], [], [], False, []
    joints = []

    def cleanup(self):
        pass


def _mock_pipeline(monkeypatch):
    from fic_studio.robot_import import importer
    calls = []

    def analyse(path, existing_ids=(), mjcf_path=None, **kw):
        calls.append((path, mjcf_path, os.path.exists(path)))
        return _FakePlan()
    monkeypatch.setattr(importer, 'analyse', analyse)
    monkeypatch.setattr(importer, '_what_was_found', lambda plan: [])
    return calls, importer


def test_the_staged_path_reaches_the_import_pipeline(studio, monkeypatch):
    calls, _ = _mock_pipeline(monkeypatch)
    uid = begin(studio)
    put(studio, uid, 'pkg/urdf/arm.urdf', b'<robot name="arm"/>')
    put(studio, uid, 'pkg/meshes/a.stl', b'x')
    reply = run(studio, 'robots_import', op='analyse', upload=uid)
    assert reply['ok'] and reply['plan']['id'] == 'dropped'
    path, mjcf, existed = calls[0]
    assert path == os.path.join(staged(studio, uid), 'pkg') and mjcf is None and existed     # the dropped folder itself


def test_create_hands_over_the_staged_path_and_cleans_up_after(studio, monkeypatch):
    calls, importer = _mock_pipeline(monkeypatch)
    done = {}

    def create_robot(plan, model_root, overwrite=False, progress=None):
        done['ran'] = True
        return types.SimpleNamespace(robot_id='dropped', config={'num_joints': 1}, model_dir='/m', messages=[])
    monkeypatch.setattr(importer, 'create_robot', create_robot)
    uid = begin(studio)
    put(studio, uid, 'arm.urdf', b'<robot name="arm"/>')
    root = staged(studio, uid)
    assert run(studio, 'robots_import', op='create', upload=uid)['error'] == 'confirm'
    assert os.path.exists(root)                                                  # a refused create keeps the upload
    reply = run(studio, 'robots_import', op='create', upload=uid, confirm=True)
    assert reply['ok']
    for _ in range(100):
        if not studio._web_import['running']:
            break
        import time
        time.sleep(0.02)
    assert done['ran'] and calls[-1][0] == os.path.join(root, 'arm.urdf')
    assert not os.path.exists(root) and uid not in studio._web_uploads


def test_the_local_server_takes_a_large_chunk_but_a_remote_one_stays_small():
    from fic_studio.web_remote import server
    assert server.MAX_LOCAL_BODY_BYTES > 1024 * 1024 * 4 // 3 and server.MAX_BODY_BYTES == 16 * 1024
    assert 'robots_upload' in actions.LOCAL_ONLY


def test_the_page_has_a_drop_zone_with_folder_walking_and_a_picker():
    from pathlib import Path
    js = (Path(actions.__file__).parent / 'ui' / 'pages-robots.js').read_text()
    for needle in ('webkitGetAsEntry', 'createReader', 'webkitdirectory', "classList.add('over')", 'robots_upload', "'dragover'"):
        assert needle in js
