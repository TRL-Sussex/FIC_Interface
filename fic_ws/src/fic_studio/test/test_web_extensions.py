"""Extensions with their own web page: manifest, discovery, server routes, the `ext`
command, the template and fic_install/new_extension.py."""
import http.client
import importlib.util
import json
import shutil
import sys
import types
from pathlib import Path

import pytest

from fic_studio.extensions.discovery import discover_extensions
from fic_studio.extensions.manifest import ExtensionManifest
from fic_studio.web_remote import actions
from fic_studio.web_remote.server import WebRemoteServer

REPO = Path(__file__).resolve().parents[4]
TEMPLATE = REPO / 'Extensions' / '_template'
WEB_UI = Path(__file__).resolve().parents[1] / 'fic_studio' / 'web_remote' / 'ui'


def _write_ext(root, ext_id, web=None):
    folder = root / ext_id
    (folder / 'web').mkdir(parents=True)
    manifest = {'id': ext_id, 'name': ext_id.title()}
    if web is not None:
        manifest['web'] = web
    (folder / 'manifest.json').write_text(json.dumps(manifest))
    (folder / 'web' / 'page.js').write_text('export default function build(root, api) { return {}; }\n')
    (folder / 'web' / 'secret.html').write_text('<script>alert(1)</script>')
    return folder


# ── Manifest and discovery ──────────────────────────────────────────────

def test_manifest_web_block_is_checked_and_normalised(tmp_path):
    folder = _write_ext(tmp_path, 'demo', web={'page': 'page.js', 'poll_ms': 20, 'local_only': ['wipe']})
    m = ExtensionManifest.from_file(folder / 'manifest.json')
    assert m.web == {'page': 'page.js', 'poll_ms': 100, 'local_only': ['wipe']}   # poll floor 100 ms
    plain = _write_ext(tmp_path, 'plain')
    assert ExtensionManifest.from_file(plain / 'manifest.json').web == {}
    bad = _write_ext(tmp_path, 'bad', web={'css': 'x.css'})
    with pytest.raises(ValueError):
        ExtensionManifest.from_file(bad / 'manifest.json')


def test_discovery_skips_template_and_helper_folders(tmp_path):
    _write_ext(tmp_path, 'real')
    _write_ext(tmp_path, '_template')
    assert [m.id for m in discover_extensions(tmp_path)] == ['real']


# ── Server routes ───────────────────────────────────────────────────────

@pytest.fixture
def ext_server(tmp_path):
    folder = _write_ext(tmp_path, 'demo', web={'page': 'page.js'})
    web = (folder / 'web').resolve()

    def ext_file(ext_id, rel):                       # what WebRemoteMixin._web_ext_file does
        if ext_id != 'demo':
            return None
        target = (web / rel).resolve()
        return target if web in target.parents and target.is_file() else None

    def ext_state(ext_id):
        return {'count': 3} if ext_id == 'demo' else None

    server = WebRemoteServer(dispatch=lambda a, b, c: {'ok': True}, snapshot=lambda: {},
                             host='127.0.0.1', extensions=(ext_file, ext_state))
    port = server.start(0)
    yield port
    server.stop()


def _get(port, path):
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    conn.request('GET', path)
    res = conn.getresponse()
    body = res.read()
    conn.close()
    return res.status, res.getheader('Content-Type') or '', body


def test_serves_an_extensions_web_files_and_nothing_else(ext_server):
    status, ctype, body = _get(ext_server, '/ext/demo/page.js')
    assert status == 200 and ctype.startswith('text/javascript') and b'export default' in body
    assert _get(ext_server, '/ext/demo/secret.html')[0] == 404          # never HTML from an extension
    assert _get(ext_server, '/ext/demo/../manifest.json')[0] == 404     # stays inside web/
    assert _get(ext_server, '/ext/demo/%2e%2e/manifest.json')[0] == 404
    assert _get(ext_server, '/ext/other/page.js')[0] == 404


def test_ext_state_route(ext_server):
    status, _, body = _get(ext_server, '/api/ext_state?id=demo')
    assert status == 200 and json.loads(body) == {'ok': True, 'state': {'count': 3}}
    assert _get(ext_server, '/api/ext_state?id=nope')[0] == 404


# ── The `ext` command ───────────────────────────────────────────────────

class _Studio:
    def __init__(self):
        self.logs = []

    def log_message(self, level, msg, source=''):
        self.logs.append((level, msg))

    def _web_ext_call(self, ext_id, name, args, local):
        if ext_id != 'demo':
            raise LookupError('No such extension page')
        if name == 'wipe' and not local:
            raise PermissionError('That is only available in the Studio window itself')
        if name == 'bad':
            raise ValueError('Gain must be between 0 and 1')
        if name == 'boom':
            raise RuntimeError('kaboom')
        return {'did': name, 'args': args}


def test_ext_command_runs_refuses_and_reports():
    w = _Studio()
    run = lambda name, local=False, **kw: actions.handle_command(  # noqa: E731
        w, 'ext', dict({'extension': 'demo', 'name': name}, **kw), {'local': local})
    assert run('go', args={'v': 1}) == {'ok': True, 'result': {'did': 'go', 'args': {'v': 1}}}
    assert run('wipe')['error'] == 'local_only'
    assert run('wipe', local=True)['ok'] is True
    assert run('bad')['message'] == 'Gain must be between 0 and 1'
    assert run('boom')['error'] == 'failed' and w.logs[-1][0] == 'ERROR'
    assert actions.handle_command(w, 'ext', {'extension': 'x', 'name': 'go'}, {})['error'] == 'no_such_extension'
    assert actions.handle_command(w, 'ext', {'extension': 'demo'}, {})['error'] == 'bad_request'
    assert run('go', args=[1])['error'] == 'bad_args'


# ── The template ────────────────────────────────────────────────────────

class _Pose:
    position = types.SimpleNamespace(x=0.1, y=-0.2, z=0.3)


class _Host:
    def __init__(self):
        self.settings, self.logged = {}, []
        self.current_ee_pose = _Pose()
        self.current_joint_positions = [0.0, 0.5]
        self.target_robot_id = 'robot1'
        self.controller_running = True

    def current_robot_name(self):
        return 'KinovaGen3_7DOF'

    def get_setting(self, key, default=None):
        return self.settings.get(key, default)

    def set_setting(self, key, value):
        self.settings[key] = value

    def log(self, level, msg, category=None):
        self.logged.append(msg)


def _load_template():
    spec = importlib.util.spec_from_file_location('template_extension', TEMPLATE / 'extension.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_template_manifest_and_files():
    m = ExtensionManifest.from_file(TEMPLATE / 'manifest.json')
    assert m.id == 'my_extension' and m.web['page'] == 'page.js' and 'reset_settings' in m.web['local_only']
    page = (TEMPLATE / 'web' / 'page.js').read_text()
    assert 'export default function build(root, api)' in page and 'api.call(' in page


def test_template_web_state_and_actions():
    ext = _load_template().Extension(_Host())
    s = ext.web_state()
    assert s['robot'] == 'KinovaGen3_7DOF' and s['ee'] == [0.1, -0.2, 0.3] and s['joints_deg'] == [0.0, 28.6]
    assert ext.web_action('set_gain', {'value': 0.8}) == {'gain': 0.8} and ext.host.settings['gain'] == 0.8
    with pytest.raises(ValueError):
        ext.web_action('set_gain', {'value': 3})
    ext.web_action('set_enabled', {'value': True})
    ext.on_emergency_stop()                                   # E-STOP switches it off
    assert ext.web_state()['enabled'] is False
    ext.web_action('hello', {})
    assert ext.host.logged and any('Hello' in line for line in ext.web_state()['log'])
    with pytest.raises(ValueError):
        ext.web_action('nope', {})


# ── new_extension.py ────────────────────────────────────────────────────

def test_new_extension_copies_and_renames(tmp_path, monkeypatch, capsys):
    spec = importlib.util.spec_from_file_location('new_extension', REPO / 'fic_install' / 'new_extension.py')
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    exts = tmp_path / 'Extensions'
    exts.mkdir()
    shutil.copytree(TEMPLATE, exts / '_template', ignore=shutil.ignore_patterns('__pycache__'))
    monkeypatch.setattr(tool, 'ROOT', tmp_path)
    monkeypatch.setattr(tool, 'EXTENSIONS', exts)
    monkeypatch.setattr(tool, 'TEMPLATE', exts / '_template')
    assert tool.main(['bench_tool', 'Bench Tool', '--section', 'Programming']) == 0
    made = exts / 'bench_tool'
    m = ExtensionManifest.from_file(made / 'manifest.json')
    assert (m.id, m.name, m.sidebar_section, m.entry_module) == ('bench_tool', 'Bench Tool', 'Programming', 'bench_tool.extension')
    code = (made / 'extension.py').read_text()
    assert 'ID = "bench_tool"' in code and 'NAME = "Bench Tool"' in code and 'SIDEBAR_SECTION = "Programming"' in code
    assert 'my_extension' not in code and 'My Extension' not in (made / 'web' / 'page.js').read_text()
    assert [d.id for d in discover_extensions(exts)] == ['bench_tool']
    with pytest.raises(SystemExit):
        tool.main(['bench_tool', 'Again'])                   # exists already
    with pytest.raises(SystemExit):
        tool.main(['Bad-Id', 'X'])


# ── The web UI pieces ───────────────────────────────────────────────────

def test_ui_loads_the_kit_and_runs_web_pages():
    index = (WEB_UI / 'index.html').read_text()
    assert index.index('/ui/kit.js') < index.index('/ui/pages-extensions.js')
    page = (WEB_UI / 'pages-extensions.js').read_text()
    assert "import(meta.web.page)" in page and "action: 'ext'" in page and '/api/ext_state?id=' in page
    kit = (WEB_UI / 'kit.js').read_text()
    for name in ('section', 'grid', 'stats', 'field', 'button', 'buttons', 'toggle', 'select',
                 'number', 'text', 'slider', 'bar', 'log', 'note', 'table'):
        assert name in kit.split('window.FIC.ui = {')[1]


if __name__ == '__main__':
    sys.exit(pytest.main([__file__]))
