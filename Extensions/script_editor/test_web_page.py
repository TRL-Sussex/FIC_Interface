"""
The Script Editor's web page, Studio side: the shared run path and output,
web_state() and web_action(), and the files the web interface loads. Qt and
fic_studio are stubbed by test_page_render, imported below.
"""
import json
import pathlib
import re
import time

import pytest

from .test_page_render import TestPageRender, _ext_mod

HERE = pathlib.Path(__file__).resolve().parent
WEB = HERE / 'web'
ACTIONS = {'open', 'example', 'save', 'save_as', 'delete', 'run', 'stop', 'clear'}


def _ext(tmp_path):
    """An extension on a fake host, with the Classic page built (as in the Studio)."""
    ext = TestPageRender()._make_extension(tmp_path)
    ext.build_page()
    return ext


def _wait(ext, timeout_s=5.0):
    """Let the script thread finish, then deliver its output as the flush timer does."""
    if ext.script_runner is not None:
        ext.script_runner.wait(int(timeout_s * 1000))
    ext._flush_script_output()


# ── One run path and one output ──────────────────────────────

def test_a_run_reports_its_output_and_status(tmp_path):
    ext = _ext(tmp_path)
    seq = ext._output.seq
    ext._start_run("print('hello')\nprint('there')\n", 'simulation')
    _wait(ext)
    lines = ext._output.tail()
    assert lines[0] == '=== Script Started (Target: SIMULATION) ==='
    assert 'hello' in lines and 'there' in lines
    assert ext._run_status == 'finished' and ext._error_line is None
    assert ext._output.seq > seq


def test_a_failed_run_gives_its_line(tmp_path):
    ext = _ext(tmp_path)
    ext._start_run("x = 1\n\nraise ValueError('boom')\n", 'simulation')
    _wait(ext)
    assert ext._run_status == 'failed' and ext._error_line == 3
    assert any('ValueError: boom' in line for line in ext._output.tail())


def test_a_syntax_error_gives_its_line(tmp_path):
    ext = _ext(tmp_path)
    ext._start_run('x = 1\nfor y in\n', 'simulation')
    _wait(ext)
    assert ext._run_status == 'failed' and ext._error_line == 2


def test_empty_and_second_runs_are_refused(tmp_path):
    ext = _ext(tmp_path)
    with pytest.raises(ValueError, match='Nothing to run'):
        ext._start_run('  \n', 'simulation')
    ext._start_run('while not is_stopped():\n    wait(0.05)\n', 'simulation')
    with pytest.raises(ValueError, match='Stop the script first'):
        ext._start_run('x = 1', 'simulation')
    ext._stop_script()
    _wait(ext)
    assert ext._run_status == 'stopped' and not ext.script_running


def test_the_classic_run_keeps_its_own_target(tmp_path):
    ext = _ext(tmp_path)
    seen = []
    real = ext._create_robot_api
    ext._create_robot_api = lambda target=None: (seen.append(target), real(target))[1]
    ext.script_target_mode = 'vr'
    ext.script_editor.toPlainText = lambda: 'x = 1'
    ext._run_script()
    _wait(ext)
    assert seen == ['vr']


def test_both_pages_show_the_same_output(tmp_path):
    ext = _ext(tmp_path)
    console = []
    ext.script_output.append = console.append
    ext._start_run("print('both')\n", 'simulation')
    _wait(ext)
    assert 'both' in ext._output.tail()
    assert any('both' in str(m) for m in console)
    ext._clear_output()
    assert ext._output.tail() == []


# ── web_state and web_action ─────────────────────────────────

def test_state_lists_scripts_examples_and_where_runs_go(tmp_path):
    ext = _ext(tmp_path)
    (tmp_path / 'b.py').write_text('x = 2\n')
    (tmp_path / 'a.py').write_text('x = 1\n')
    _ext_mod._seed_examples(_ext_mod.EXAMPLES_DIR)
    s = ext.web_state()
    assert s['scripts'] == ['a', 'b']
    assert [e['name'] for e in s['examples']] == sorted(n[:-3] for n in _ext_mod._EXAMPLES)
    assert all(e['description'] and not e['description'].endswith('.py') for e in s['examples'])
    assert s['examples'][0]['description'].startswith('Read robot state')
    assert s['target'] == 'simulation' and s['robot'] == 'Robot 1 (fake_robot)'
    assert s['status'] == 'idle' and s['running'] is False and s['output'] == []
    json.dumps(s)                                    # what the web server sends


def test_open_save_save_as_delete(tmp_path):
    ext = _ext(tmp_path)
    assert ext.web_action('save_as', {'name': 'demo', 'code': 'print(1)\n'}) == {'name': 'demo'}
    assert (tmp_path / 'demo.py').read_text() == 'print(1)\n'
    with pytest.raises(ValueError, match='A script with that name exists'):
        ext.web_action('save_as', {'name': 'demo', 'code': 'x'})
    assert ext.web_action('save', {'name': 'demo', 'code': 'print(2)\n'}) == {'name': 'demo'}
    assert ext.web_action('open', {'name': 'demo'}) == {'name': 'demo', 'code': 'print(2)\n'}
    assert 'demo' in ext.web_state()['scripts']
    assert ext.web_action('delete', {'name': 'demo'}) == {'name': 'demo'}
    assert not (tmp_path / 'demo.py').exists()
    for action in ('open', 'delete'):
        with pytest.raises(ValueError, match='No such script'):
            ext.web_action(action, {'name': 'demo'})


@pytest.mark.parametrize('bad', ['', '../x', 'a/b', 'x.py', '/etc/passwd'])
@pytest.mark.parametrize('action', ['open', 'example', 'save', 'save_as', 'delete'])
def test_names_that_could_leave_the_folder_are_refused(tmp_path, bad, action):
    ext = _ext(tmp_path)
    with pytest.raises(ValueError, match='Use letters, digits, _ or - in the name'):
        ext.web_action(action, {'name': bad, 'code': 'x = 1'})
    assert not (tmp_path.parent / 'x.py').exists()


def test_example_opens_its_text(tmp_path):
    ext = _ext(tmp_path)
    _ext_mod._seed_examples(_ext_mod.EXAMPLES_DIR)
    r = ext.web_action('example', {'name': '01_read_state'})
    assert r == {'name': '01_read_state', 'code': _ext_mod._EXAMPLES['01_read_state.py']}


def test_a_web_run_goes_to_the_studios_target(tmp_path):
    ext = _ext(tmp_path)
    seen = []
    real = ext._create_robot_api
    ext._create_robot_api = lambda target=None: (seen.append(target), real(target))[1]
    ext.script_target_mode = 'simulation'             # the Classic dropdown does not matter here
    ext.host.robot_target = lambda rid=None: 'vr'
    assert ext.web_action('run', {'code': 'x = 1'}) == {'target': 'vr'}
    _wait(ext)
    assert seen == ['vr']


def test_a_hardware_run_needs_its_confirmation(tmp_path):
    ext = _ext(tmp_path)
    ext.host.robot_target = lambda rid=None: 'hardware'
    with pytest.raises(ValueError, match='Confirm running on the real robot'):
        ext.web_action('run', {'code': 'x = 1'})
    assert not ext.script_running
    assert ext.web_action('run', {'code': 'x = 1', 'hardware_ok': True}) == {'target': 'hardware'}
    _wait(ext)


def test_state_during_a_run_and_stop(tmp_path):
    ext = _ext(tmp_path)
    ext.web_action('run', {'code': 'while not is_stopped():\n    wait(0.05)\n'})
    first = ext.web_state()
    assert first['running'] is True and first['status'] == 'running'
    time.sleep(0.2)
    assert ext.web_state()['elapsed_s'] >= first['elapsed_s']
    with pytest.raises(ValueError, match='Stop the script first'):
        ext.web_action('run', {'code': 'x = 1'})
    assert ext.web_action('stop', {}) == {'running': False}
    _wait(ext)
    assert ext.web_state()['status'] == 'stopped'


def test_failed_run_state(tmp_path):
    ext = _ext(tmp_path)
    ext.web_action('run', {'code': 'x = 1\nraise RuntimeError("no")\n'})
    _wait(ext)
    s = ext.web_state()
    assert s['status'] == 'failed' and s['error_line'] == 2


def test_output_is_capped_for_the_page(tmp_path):
    ext = _ext(tmp_path)
    ext.web_action('run', {'code': 'for i in range(3000):\n    print(i)\n'})
    _wait(ext)
    s = ext.web_state()
    assert len(s['output']) == 500 and len(ext._output.tail(5000)) <= 2000
    seq = s['output_seq']
    assert ext.web_action('clear', {}) == {}
    s = ext.web_state()
    assert s['output'] == [] and s['output_seq'] > seq


def test_unknown_actions_are_refused(tmp_path):
    with pytest.raises(ValueError, match='No such action: nope'):
        _ext(tmp_path).web_action('nope', {})


def test_manifest_points_the_web_interface_at_the_page():
    m = json.loads((HERE / 'manifest.json').read_text())
    assert m['web'] == {'page': 'page.js', 'css': 'page.css', 'poll_ms': 300}     # no local_only actions


def test_api_reference_json_is_up_to_date():
    from Extensions.script_editor import make_api_reference
    assert (WEB / 'api_reference.json').read_text(encoding='utf-8') == make_api_reference.build(), (
        'web/api_reference.json is out of date: run '
        'python3 -m Extensions.script_editor.make_api_reference from the repository root')


# ── The page files ───────────────────────────────────────────

def test_every_call_in_the_page_is_an_action(tmp_path):
    names = set(re.findall(r"api\.call\('([a-z_]+)'", (WEB / 'page.js').read_text()))
    assert names == ACTIONS
    ext = _ext(tmp_path)
    for name in names:
        try:
            ext.web_action(name, {})
        except ValueError as exc:
            assert 'No such action' not in str(exc)


def test_the_page_files_exist_and_load_the_editor():
    for f in ('page.js', 'page.css', 'editor.js', 'api_reference.json'):
        assert (WEB / f).is_file(), f
    page = (WEB / 'page.js').read_text()
    assert "from './editor.js'" in page and 'api_reference.json' in page
