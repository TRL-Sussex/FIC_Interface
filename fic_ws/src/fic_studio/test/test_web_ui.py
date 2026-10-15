"""The redesigned web UI: its local-only server, its API and the E-STOP path.

The E-STOP test goes the whole way a click does: HTTP POST /api/command
'estop' with no lease, through the server, into the real
``actions.handle_command``, onto the Studio's ``_emergency_stop``.
"""
import http.client
import json
import re

from fic_studio.web_remote import actions
from fic_studio.web_remote.lease import ControlLease
from fic_studio.web_remote.server import UI_DIR, UI_PAGES, WebRemoteServer
import pytest
from test_web_remote_actions import _Studio, ROBOT
from PyQt5.QtWidgets import QApplication, QComboBox

TOKEN = 'test-token-123'
#: Pages that have landed; each phase-B commit adds its page here.
READY = {'dashboard', 'session', 'view3d', 'log', 'safety', 'tuning', 'multi_robot', 'locomotion', 'extensions',
         'robot_config', 'topic_monitor', 'settings', 'help', 'users'}


class _UIStudio(_Studio):
    """The stand-in Studio with a real robot selector."""

    def __init__(self):
        super().__init__()
        self.robot_dropdown = QComboBox()
        self.robot_dropdown.addItems([ROBOT, 'FrankaPanda'])
        self.robot_configs['FrankaPanda'] = {'display_name': 'Franka Panda'}


@pytest.fixture
def stack():
    studio = _UIStudio()
    lease = ControlLease(timeout_s=30.0)
    server = WebRemoteServer(
        dispatch=lambda action, args, who: actions.handle_command(studio, action, args, who),
        snapshot=lambda: actions.build_snapshot(studio), lease=lease,
        host='127.0.0.1', local_token=TOKEN)
    port = server.start(0)
    yield studio, server, lease, port
    server.stop()
    QApplication.processEvents()      # flush signals queued from the server thread


def _req(port, method, path, body=None, cookie=None):
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
    headers = {}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        headers['Content-Type'] = 'application/json'
    if cookie:
        headers['Cookie'] = cookie
    conn.request(method, path, body=data, headers=headers)
    res = conn.getresponse()
    raw = res.read()
    conn.close()
    return res.status, raw, res


def _json(raw):
    return json.loads(raw)


COOKIE = 'fic_local=' + TOKEN


# ── Local-only access ────────────────────────────────────────────────────
def test_binds_to_loopback_only(stack):
    _, server, _, _ = stack
    assert server._httpd.server_address[0] == '127.0.0.1'


def test_ui_needs_the_token(stack):
    *_, port = stack
    assert _req(port, 'GET', '/ui/')[0] == 403
    assert _req(port, 'GET', '/ui/?t=wrong')[0] == 403
    assert _req(port, 'GET', '/api/state')[0] == 403
    assert _req(port, 'GET', '/api/ui')[0] == 403
    status, _, res = _req(port, 'POST', '/api/command', {'client': 'x', 'action': 'estop'})
    assert status == 403


def test_token_sets_cookie_and_opens_the_ui(stack):
    *_, port = stack
    status, _, res = _req(port, 'GET', '/ui/?t=' + TOKEN)
    assert status == 302
    assert res.getheader('Location') == '/ui/'
    cookie = res.getheader('Set-Cookie')
    assert 'fic_local=' + TOKEN in cookie and 'HttpOnly' in cookie and 'SameSite=Strict' in cookie
    status, raw, res = _req(port, 'GET', '/ui/', cookie=COOKIE)
    assert status == 200 and b'FIC Studio' in raw
    assert 'text/html' in res.getheader('Content-Type')
    assert _req(port, 'GET', '/ui/fic.css', cookie=COOKIE)[0] == 200
    assert _req(port, 'GET', '/ui/app.js', cookie=COOKIE)[0] == 200


def test_ui_files_cannot_escape_their_folder(stack):
    *_, port = stack
    for path in ('/ui/../server.py', '/ui/%2e%2e/server.py', '/ui/missing.css', '/ui/app.py'):
        assert _req(port, 'GET', path, cookie=COOKIE)[0] == 404


def test_without_a_token_the_ui_is_open_like_the_phone_page():
    server = WebRemoteServer(dispatch=lambda *a: {'ok': True}, snapshot=lambda: {}, host='127.0.0.1')
    port = server.start(0)
    try:
        assert _req(port, 'GET', '/ui/')[0] == 200
    finally:
        server.stop()


# ── API ───────────────────────────────────────────────────────────────────
def test_api_ui_lists_pages(stack):
    *_, port = stack
    status, raw, _ = _req(port, 'GET', '/api/ui', cookie=COOKIE)
    body = _json(raw)
    assert status == 200 and body['local'] is True
    assert [p['id'] for p in body['pages']] == [p[0] for p in UI_PAGES]
    assert {p['id'] for p in body['pages'] if p['ready']} == READY


def test_state_carries_the_robot_list(stack):
    *_, port = stack
    status, raw, _ = _req(port, 'GET', '/api/state?client=c', cookie=COOKIE)
    robots = _json(raw)['state']['robots']
    assert status == 200
    assert robots['current'] == ROBOT
    assert [r['name'] for r in robots['items']] == [ROBOT, 'FrankaPanda']
    assert robots['items'][1]['label'] == 'Franka Panda'


def test_local_session_takes_control_from_a_remote_holder(stack):
    _, _, lease, port = stack
    lease.take('phone', 'Phone', '10.0.0.9')
    status, raw, _ = _req(port, 'POST', '/api/control',
                          {'client': 'desk', 'op': 'take'}, cookie=COOKIE)
    assert status == 409                       # a plain take is refused, as before
    status, raw, _ = _req(port, 'POST', '/api/control',
                          {'client': 'desk', 'op': 'take', 'force': True}, cookie=COOKIE)
    assert status == 200 and lease.holds('desk') and not lease.holds('phone')


def test_force_is_ignored_on_a_server_without_a_local_token():
    lease = ControlLease(timeout_s=30.0)
    server = WebRemoteServer(dispatch=lambda *a: {'ok': True}, snapshot=lambda: {},
                             lease=lease, host='127.0.0.1')
    port = server.start(0)
    try:
        lease.take('phone')
        status, _, _ = _req(port, 'POST', '/api/control', {'client': 'x', 'op': 'take', 'force': True})
        assert status == 409 and lease.holds('phone')
    finally:
        server.stop()


def _take(port, client='desk'):
    assert _req(port, 'POST', '/api/control', {'client': client, 'op': 'take', 'force': True},
                cookie=COOKIE)[0] == 200


def _command(port, action, client='desk', **args):
    status, raw, _ = _req(port, 'POST', '/api/command',
                          {'client': client, 'action': action, 'args': args}, cookie=COOKIE)
    return status, _json(raw)


def test_commands_need_the_lease(stack):
    studio, _, _, port = stack
    status, body = _command(port, 'set_mode', index=1, robot=ROBOT)
    assert status == 403 and body['error'] == 'not_in_control'
    assert not [c for c in studio.calls if c[0] == 'mode']


def test_set_mode_and_profile_reach_the_qt_handlers(stack):
    studio, _, _, port = stack
    _take(port)
    assert _command(port, 'set_mode', index=1, robot=ROBOT)[1]['ok']
    assert ('mode', 1, CONTROL_NAME(1)) in studio.calls
    assert _command(port, 'set_profile', name='Stiff', robot=ROBOT)[1]['ok']
    QApplication.processEvents()     # Qt queues the combo's signal across threads
    assert ('profile', 'Stiff') in studio.calls


def CONTROL_NAME(i):
    from fic_studio.mixins.robot_control_mixin import CONTROL_MODE_TABS
    return CONTROL_MODE_TABS[i][0]


def test_send_joints_goes_through_the_published_handler(stack):
    studio, _, _, port = stack
    _take(port)
    status, body = _command(port, 'send_joints', degrees=[1, 2, 3, 4, 5, 6, 7], robot=ROBOT)
    assert body['ok'] and ('joints', [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]) in studio.calls


def test_set_robot(stack):
    studio, _, _, port = stack
    _take(port)
    assert _command(port, 'set_robot', name='FrankaPanda')[1]['ok']
    assert studio.robot_dropdown.currentText() == 'FrankaPanda'
    assert _command(port, 'set_robot', name='Nope')[1]['error'] == 'bad_args'
    studio.running = True
    assert _command(port, 'set_robot', name=ROBOT)[1]['error'] == 'session_running'
    assert studio.robot_dropdown.currentText() == 'FrankaPanda'


# ── E-STOP ────────────────────────────────────────────────────────────────
def test_estop_from_the_web_ui_reaches_the_studio(stack):
    """The command a click on E-STOP sends: no lease, no robot, no confirmation."""
    studio, _, lease, port = stack
    assert not lease.holds('desk')
    status, body = _command(port, 'estop')
    assert status == 200 and body['ok'] is True
    assert ('estop',) in studio.calls
    assert studio.logs[-1][0] == 'ERROR' and 'E-STOP' in studio.logs[-1][1]


def test_estop_works_while_another_client_holds_control(stack):
    studio, _, lease, port = stack
    lease.take('phone', 'Phone', '10.0.0.9')
    assert _command(port, 'estop', client='desk')[1]['ok']
    assert ('estop',) in studio.calls


def test_estop_blocks_motion_that_follows(stack):
    studio, _, _, port = stack
    _take(port)
    _command(port, 'estop')
    status, body = _command(port, 'send_joints', degrees=[0] * 7, robot=ROBOT)
    assert body['error'] == 'estop_holdoff'
    assert not [c for c in studio.calls if c[0] == 'joints']


def test_estop_needs_the_local_token(stack):
    studio, _, _, port = stack
    status, _, _ = _req(port, 'POST', '/api/command', {'client': 'x', 'action': 'estop'})
    assert status == 403 and ('estop',) not in studio.calls


def test_the_page_wires_its_estop_button_to_the_estop_command():
    """The JS contract: the button exists, and its handler posts 'estop' to /api/command."""
    html = (UI_DIR / 'index.html').read_text()
    js = (UI_DIR / 'app.js').read_text()
    assert re.search(r'<button id="estop" class="estop"', html)
    assert "$('#estop').addEventListener('click', fireEstop)" in js
    fire = js[js.index('async function fireEstop'):].split('\n}\n')[0]
    assert "post('/api/command', { action: 'estop', args: {} })" in fire
    assert 'confirm' not in fire and 'takeControl' not in fire          # one click, no lease


def test_no_clickable_element_has_a_box_but_estop():
    css = (UI_DIR / 'fic.css').read_text()
    for selector in ('.btn {', '.seg button {'):
        rule = css[css.index(selector):].split('}')[0]
        assert re.search(r'border:\s*0', rule) and 'background: none' in rule


# ── Launch flag ───────────────────────────────────────────────────────────
def test_launch_flag_defaults_to_the_classic_ui():
    from fic_studio.mixins.web_ui_mixin import web_ui_requested
    assert web_ui_requested(argv=['studio'], settings={}, environ={}) is False
    assert web_ui_requested(argv=['studio', '--web-ui'], settings={}, environ={}) is True
    assert web_ui_requested(argv=['studio'], settings={}, environ={'FIC_WEB_UI': '1'}) is True
    assert web_ui_requested(argv=['studio'], settings={'general': {'ui': 'web'}}, environ={}) is True
    assert web_ui_requested(argv=['studio', '--classic', '--web-ui'], settings={}, environ={}) is False


# ── Shell: laid out like the classic window (phase C) ─────────────────────
def test_the_page_list_is_the_classic_sidebar_in_its_order_and_has_no_control_page():
    ids = [p[0] for p in UI_PAGES]
    assert 'control' not in ids
    main = [p for p in UI_PAGES if p[4] == 'main']
    bottom = [p for p in UI_PAGES if p[4] == 'bottom']
    system = [p for p in UI_PAGES if p[4] == 'system']
    assert [p[1] for p in main] == ['Dashboard', 'Multi-Robot', 'Locomotion', 'Tuning Panel']
    assert [p[1] for p in system] == ['Robot Config', 'Topic Monitor', 'Safety']
    # Users sits with Settings: both are administration, not operating the arm.
    assert [p[1] for p in bottom] == ['Users', 'Settings', 'Logs', 'Help']
    assert {p[0] for p in UI_PAGES if p[4] == 'hidden'} == {'extensions', 'session', 'view3d'}
    js = (UI_DIR / 'app.js').read_text()
    assert 'buildControl' not in js and "#/control" not in (UI_DIR / 'dock.js').read_text()


def test_menu_bar_then_toolbar_then_pages_then_control_bar_then_status_bar():
    html = (UI_DIR / 'index.html').read_text()
    top = html[html.index('<header'):html.index('</header>')]
    assert top.index('id="menus"') < top.index('id="toolbar"')
    order = [top.index(i) for i in ('id="target"', 'id="sess"', 'id="tb-exp"', 'id="tb-bags"', 'id="tb-plot"',
                                    'id="tb-view"', 'id="station"', 'id="robot"', 'id="estop"')]
    assert order == sorted(order)                    # target, Start, EXP BAG PLT VIEW, station, robot, Teleop, E-STOP
    assert 'class="kbd' in top and 'Space' in top
    assert html.index('</header>') < html.index('id="side"') < html.index('id="main"') < html.index('id="dock"') < html.index('id="statusbar"')


def test_the_six_classic_menus_are_there_with_their_entries():
    js = (UI_DIR / 'shell.js').read_text()
    menus = re.findall(r"^  \['(File|View|Robot|Simulation|Tools|Help)'", js, re.M)
    assert menus == ['File', 'View', 'Robot', 'Simulation', 'Tools', 'Help']
    for entry in ('New Script', 'Config Snapshot', 'Recent Experiments', 'Exit', 'Keyboard Control', 'Show Control Bar',
                  'Compact Mode', 'Minimal Mode', 'UI Scale', 'Toggle Fullscreen', 'Move to Home', 'Emergency Stop', 'Select Robot',
                  'Control Mode', 'Robot Info', 'Toggle Simulation', 'Launch Sim + FIC', 'Stop All Processes', 'Toggle Controller',
                  'Toggle Robot Driver', 'Quick Experiment', 'Toggle Plotter', 'Topic Monitor', 'Robot Configuration',
                  'Open Experiments Folder', 'Compile Workspace', 'Compile Package', 'Clean Build', 'Open Documentation',
                  'Keyboard Shortcuts', 'Re-run Setup Wizard', 'About'):
        assert entry in js, entry


def test_status_bar_has_the_classic_indicators():
    js = (UI_DIR / 'shell.js').read_text()
    items = re.search(r"SB_ITEMS = \[(.*?)\];", js).group(1)
    assert re.findall(r"'([A-Z0-9]+)'\]", items) == ['ROS2', 'FIC', 'SIM', 'BAGS', 'INNER', 'TELEOP', 'GAMEPAD', 'WEB']
    assert "sb-robot" in js and 'statusbar' in js


def test_the_control_bar_has_the_classic_buttons_and_every_control_lives_in_it():
    js = (UI_DIR / 'dock.js').read_text()
    labels = re.findall(r"\['(joint|cart|gripper|contact|snapshot|quick)', '([^']+)'\]", js)
    assert [l for _, l in labels] == ['Joint', 'Cart', 'Gripper', 'Contact', 'Snapshot', 'Quick Actions']
    assert "icon: 'home', text: 'Home'" in js and "if (id === 'gripper') buttons.append(h('button', { class: 'btn', id: 'd-home'" in js
    for command in ('set_mode', 'set_option', 'set_profile', 'send_joints', 'send_pose', 'gripper', 'home', 'live_publish',
                    'reset_defaults', 'robot_pause', 'robot_stop', 'preset', 'contact_calibrate', 'contact_truth',
                    'sequence', 'capture'):
        assert "'%s'" % command in js, command
    for needle in ("id: 'd-mode'", "id: 'd-opts'", "id: 'd-prof'", "id: 'd-live'", 'class: \'drawer\''):
        assert needle in js.replace('"', "'")
    keys = (UI_DIR / 'shell.js').read_text()
    keys = keys[keys.index('function onKey'):keys.index('// ── Wiring')]
    for k in ("e.key === ' '", "'Escape'", "k === 'j'", "k === 'g'", "k === 'H'", "k === 's'", "'['"):
        assert k in keys
    assert 'typing' in keys                                  # never fires from a text field


def test_estop_stays_above_everything_and_is_the_only_boxed_control():
    css = (UI_DIR / 'fic.css').read_text()
    z = {sel.strip(): int(n) for sel, n in re.findall(r'([^{}]+)\{[^}]*?z-index:\s*(\d+)', css)}
    wrap = z['.estop-wrap']
    assert wrap == max(z.values()) and all(v < wrap for k, v in z.items() if k != '.estop-wrap')
    estop = css[css.index('.estop {'):].split('}')[0]
    assert 'background: var(--estop)' in estop
    for sel in ('.btn {', '.seg button {', '.input, .select {'):
        rule = css[css.index(sel):].split('}')[0]
        assert 'box-shadow' not in rule
    # no tab, menu button or bar button paints a box either
    for sel in ('.menu-btn {', '.rl-row {', '.help-link {', '.tm-ns {'):
        rule = css[css.index(sel):].split('}')[0]
        assert re.search(r'background:\s*none', rule) and ('border: 0' in rule or 'border:0' in rule or 'border-bottom' in rule)


def test_narrow_layout_keeps_estop_and_the_menu_bar():
    css = (UI_DIR / 'fic.css').read_text()
    narrow = css[css.index('@media (max-width: 860px)'):css.index('@media (prefers-reduced-motion')]
    assert '.only-s { display: inline-block; }' in narrow and 'body:not(.side-collapsed) .side { display: flex; }' in narrow
    assert not re.search(r'\.estop-wrap[^{]*\{[^}]*display:\s*none', narrow)          # never hidden
    assert '.menubar' in narrow and '.drawer' in narrow


def test_the_view_menu_toggles_hide_the_bars_by_class():
    js = (UI_DIR / 'shell.js').read_text()
    css = (UI_DIR / 'fic.css').read_text()
    for cls in ('no-toolbar', 'no-bar', 'side-collapsed', 'no-status', 'compact'):
        assert cls in js and ('.' + cls) in css


# ── Phase B pages: what the scripts promise the Studio ────────────────────
def _scripts():
    return sorted(UI_DIR.glob('*.js'))


def test_every_ready_page_has_its_script_and_registers_a_builder():
    html = (UI_DIR / 'index.html').read_text()
    core = (UI_DIR / 'app.js').read_text()
    for page in READY:
        scripts = [f for f in _scripts() if f.name.startswith('pages-')]
        owner = [f for f in scripts if 'FIC.pages.%s =' % page in f.read_text()]
        assert owner, 'no script registers the %s page' % page
        assert '/ui/%s' % owner[0].name in html
    for shell in ('shell.js', 'dock.js'):
        assert '/ui/%s' % shell in html
    assert not [p for p in UI_PAGES if p[3] and p[0] not in READY]       # no page is ready without a test


def test_pages_only_send_commands_the_studio_has():
    names = set()
    for f in _scripts():
        text = f.read_text()
        names |= set(re.findall(r"\bcmd\('([a-z_]+)'", text))
        names |= set(re.findall(r"action: '([a-z_]+)'", text))
    assert names and names <= set(actions.HANDLERS), names - set(actions.HANDLERS)
    assert {'process', 'log_read', 'safety_set', 'tuning_save', 'tuning_plot', 'fleet', 'locomotion', 'settings_get',
            'settings_set', 'robots_list', 'robots_save', 'topics', 'help', 'toolbar', 'station', 'menu', 'build'} <= names


def test_edits_that_matter_are_sent_with_confirm_after_a_review_step():
    safety = (UI_DIR / 'pages-safety.js').read_text()
    tuning = (UI_DIR / 'pages-tuning.js').read_text()
    # the page asks (inlineConfirm) before the command that carries confirm: true
    assert safety.index('inlineConfirm(askHost') < safety.index("cmd('safety_set', out")
    assert "var confirm" not in safety and "out = { confirm: true }" in safety
    assert tuning.index('inlineConfirm(askHost') < tuning.index("cmd('tuning_save'")
    assert 'confirm: true' in tuning


def test_extension_page_draws_every_widget_kind_the_mirror_describes():
    from fic_studio.web_remote import qtmirror
    js = (UI_DIR / 'pages-extensions.js').read_text()
    drawn = set(re.findall(r"case '([a-z]+)'", js))
    from PyQt5.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QLabel, QLineEdit,
                                 QPlainTextEdit, QProgressBar, QPushButton, QSlider, QTabWidget,
                                 QVBoxLayout, QWidget)
    page = QWidget()
    lay = QVBoxLayout(page)
    box = QGroupBox('Group', page)
    inner = QVBoxLayout(box)
    for w in (QPushButton('Go'), QCheckBox('Tick'), QComboBox(), QDoubleSpinBox(), QSlider(), QLineEdit('x'),
              QLabel('hello'), QPlainTextEdit('log'), QProgressBar()):
        inner.addWidget(w)
    lay.addWidget(box)
    lay.addWidget(QTabWidget())
    kinds = set()

    def walk(nodes):
        for n in nodes:
            kinds.add(n['kind'])
            walk(n.get('nodes') or [])
    walk(qtmirror.describe(page)['nodes'])
    assert {'group', 'button', 'toggle', 'choice', 'number', 'slider', 'text', 'label', 'log', 'bar'} <= kinds
    assert (kinds | {'box', 'tabs'}) <= drawn, (kinds | {'box', 'tabs'}) - drawn


def test_tuning_page_draws_the_live_operating_point_the_bars_and_a_draggable_plot():
    js = (UI_DIR / 'pages-tuning.js').read_text()
    # live telemetry from the snapshot
    for needle in ('s.live', 'joint_error', 'joint_effort', 'cart_error', 'live.wrench', 'TRAIL', "class: 'op'", "class: 'trail'",
                   'Tracking error', 'Effort', 'Wrench', 'Δx', 'dev '):
        assert needle in js, needle
    # the curve follows edits before saving: a field, a dragged handle and the wheel all end in a replot
    assert js.count('replot()') >= 3 and "cmd('tuning_plot'" in js and 'st.edits' in js
    for needle in ("'pointerdown'", "'pointermove'", "'wheel'", 'syncForm', 'setEdit'):
        assert needle in js, needle
    for needle in ('Load from XML', 'Save to XML', 'View XML', '+ Config', '- Config', "'tuning_profile'", "'tuning_xml'"):
        assert needle in js, needle
    # a drag holds the axes still and keeps one svg, so the pointer capture survives a new curve
    assert 'setPointerCapture' in js and 'st.view' in js and 'view: [st.view.xr, st.view.yr]' in js
    assert "class: 'plot big'" in js and js.count("el('svg'") == 1
    assert "'hd ' + hd.kind" not in js                        # 'sr' is the screen-reader-only class: it hid the S·x_b handles
    # the live trial: applied as you go, kept or discarded
    # H-PMP's own tuning: a mode entry with a joint and a Cartesian layer, opened when H-PMP is on
    for needle in ('HPMP = 100', "t.layer = st.sel.layer", "'Joint layer'", "'Cartesian layer'", 's.options.hpmp', 'hpmp_layers'):
        assert needle in js, needle
    assert 'diff_effort_max' not in js                                  # no controller reads it
    for needle in ("'tuning_trial'", "op: 'apply'", "op: 'start'", "op: 'keep'", "op: 'discard'", 'Live trial', 'LIVE TRIAL', 'APPLY_MS'):
        assert needle in js, needle


def test_stylesheet_and_scripts_call_no_robot_logic():
    """A guard on the rule that no robot logic lives in the browser: no force law or kinematics in any page script."""
    for f in _scripts():
        text = f.read_text()
        assert 'Math.tanh' not in text and 'convergence_force' not in text and 'fic_force' not in text, f.name


def test_configuration_commands_work_from_the_desktop_session_and_not_from_a_phone(stack):
    studio, _, _, port = stack
    _take(port)
    status, body = _command(port, 'settings_reset')
    assert body['error'] == 'confirm'                                  # it got as far as the confirm rule: it was trusted
    phone = WebRemoteServer(dispatch=lambda action, args, who: actions.handle_command(studio, action, args, who),
                            snapshot=lambda: {}, lease=ControlLease(timeout_s=30.0), host='127.0.0.1')
    pport = phone.start(0)
    try:
        assert _req(pport, 'POST', '/api/control', {'client': 'p', 'op': 'take'})[0] == 200
        status, raw, _ = _req(pport, 'POST', '/api/command', {'client': 'p', 'action': 'settings_reset', 'args': {'confirm': True}})
        assert _json(raw)['error'] == 'local_only'
    finally:
        phone.stop()


# ── Icons and buttons ─────────────────────────────────────────────────────
def test_no_symbol_glyph_is_left_to_the_emoji_font():
    """Icons are inline SVG; the CSS asks for text presentation as a backstop; labels never keep a leading glyph."""
    import re as _re
    css = (UI_DIR / 'fic.css').read_text()
    assert 'font-variant-emoji: text' in css
    assert 'emoji' not in css.split('--sans:')[1].split(';')[0].lower()
    glyphs = _re.compile('[■-◿⌀-⏿☀-➿⬀-⯿\U0001f000-\U0001ffff]')
    for path in UI_DIR.glob('*.js'):
        if path.name == 'icons.js':
            continue
        for line in path.read_text().splitlines():
            for m in glyphs.finditer(line):
                # only status bullets inside strings the icon observer converts (leading) remain
                assert m.group() in '●○✓✕', (path.name, line.strip()[:90])
    assert 'icons.js' in (UI_DIR / 'index.html').read_text()


def test_primary_buttons_are_tinted_chips_not_links():
    css = (UI_DIR / 'fic.css').read_text()
    assert "content: '\\2192'" not in css.split('/* ── Buttons')[1].split('.btn-row')[0]
    primary = css[css.index('.btn-primary {'):].split('}')[0]
    assert 'background: var(--accent-soft)' in primary and 'underline' not in primary
    assert 'text-decoration: none' in css[css.index('.btn {'):].split('}')[0]
    assert (UI_DIR / 'icons.js').read_text().count('tidyLabel') >= 2


def test_a_settings_event_reaches_every_open_event_stream(stack):
    import threading
    import time
    studio, server, _, port = stack
    got = []

    def listen():
        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=8)
        conn.request('GET', '/api/events?client=l%d' % len(got), headers={'Cookie': COOKIE})
        res = conn.getresponse()
        buf = b''
        while b'event: settings' not in buf:
            chunk = res.read1(4096)
            if not chunk:
                break
            buf += chunk
        got.append(buf)
        conn.close()
    threads = [threading.Thread(target=listen) for _ in range(2)]
    for t in threads:
        t.start()
    time.sleep(1.0)
    server.emit('settings', {'appearance': {'rev': 'x'}, 'restart': False})
    for t in threads:
        t.join(8)
    assert len(got) == 2 and all(b'event: settings' in b and b'"rev": "x"' in b.replace(b'"rev":"x"', b'"rev": "x"') for b in got)


def test_collapsible_sections_are_wired_into_settings_safety_and_robot_config():
    app = (UI_DIR / 'app.js').read_text()
    css = (UI_DIR / 'fic.css').read_text()
    assert 'aria-expanded' in app and 'aria-controls' in app and "'Expand all'" in app and "'Collapse all'" in app
    assert 'fic-sections' in app and re.search(r'try \{ SEC\.map = JSON\.parse', app) and 'secSave' in app
    assert 'prefers-reduced-motion' in css and '.csec.collapsed' in css
    for page in ('settings', 'safety', 'robots'):
        src = (UI_DIR / ('pages-%s.js' % page)).read_text()
        assert 'csection(' in src and 'csectionTools' in src, page
    assert "aria-invalid" in (UI_DIR / 'pages-settings.js').read_text()


def test_appearance_is_applied_in_place_without_a_reload():
    app = (UI_DIR / 'app.js').read_text()
    assert "addEventListener('settings'" in app and 'applyAppearance' in app
    assert "setProperty('--accent'" in app and 'applyRootFont' in app and 'location.reload' not in app


def test_settings_sections_start_folded_and_scrollbars_are_thin():
    app = (UI_DIR / 'app.js').read_text()
    css = (UI_DIR / 'fic.css').read_text()
    assert "!key.startsWith('settings/')" in app                      # Settings folded by default; other pages open
    assert "f && !flagged && !open" in app                            # a change or an error still opens its section
    assert 'scrollbar-width: thin' in css and '::-webkit-scrollbar-thumb' in css and 'scrollbar-gutter: stable' in css
    assert '::-webkit-scrollbar-button { display: none' in css


def test_polish_focus_selection_scroll_contract():
    """Focus ring only for keyboard, chrome unselectable, every region scrolls alone."""
    css = (UI_DIR / 'fic.css').read_text()
    app = (UI_DIR / 'app.js').read_text()
    assert ':focus { outline: none; }' in css
    assert ':focus-visible' in css and 'color-mix(in srgb, var(--accent)' in css
    assert not re.search(r'outline: [3-9]px', css.split('.estop:focus-visible')[0])
    assert re.search(r'body \{[^}]*user-select: none', css)
    assert re.search(r'\.logview[^{]*\{\s*user-select: text', css)
    m = re.search(r'\nhtml \{[^}]*\}', css)
    assert 'overflow: hidden' in m.group(0)
    assert re.search(r'\nmain \{[^}]*position: fixed[^}]*overflow-y: auto', css)
    assert 'overscroll-behavior: contain' in css
    for sel in ('.side', '.drawer', '.logview', '.tm-tree'):
        assert re.search(r'overscroll-behavior: contain', css.split(sel + ',')[-1][:400]) or sel in css
    assert "addEventListener('wheel'" in app and "'range'" in app       # sliders never take the wheel
    assert 'window.scrollTo' not in app
    assert "preventDefault" in (UI_DIR / 'pages-tuning.js').read_text()
    view3d = (UI_DIR.parent / 'static' / 'view3d.js').read_text()
    assert re.search(r"'wheel'.*\n\s*e\.preventDefault", view3d)


@pytest.fixture
def remote_stack():
    """The remote server: no local token, so every client is a remote one."""
    studio = _UIStudio()
    lease = ControlLease(timeout_s=30.0)
    server = WebRemoteServer(
        dispatch=lambda action, args, who: actions.handle_command(studio, action, args, who),
        snapshot=lambda: actions.build_snapshot(studio), lease=lease, host='127.0.0.1')
    port = server.start(0)
    yield studio, port
    server.stop()
    QApplication.processEvents()


def test_remote_clients_get_the_new_ui(remote_stack):
    _, port = remote_stack
    for path in ('/', '/index.html', '/ui/'):
        status, raw, res = _req(port, 'GET', path)
        assert status == 200 and b'/ui/app.js' in raw and b'/ui/fic.css' in raw and b'id="estop"' in raw
        assert 'text/html' in res.getheader('Content-Type')
    assert _req(port, 'GET', '/ui/remote.js')[0] == 200
    assert _req(port, 'GET', '/app.css')[0] == 404          # the old phone page is gone
    status, raw, _ = _req(port, 'GET', '/api/ui')
    info = _json(raw)
    assert info['local'] is False and 'settings_set' in info['local_only'] and 'build' in info['local_only']


def test_remote_local_only_stays_refused_and_estop_allowed(remote_stack):
    studio, port = remote_stack
    assert _json(_req(port, 'POST', '/api/control', {'client': 'p', 'op': 'take', 'force': True})[1])['ok']
    for action in ('settings_set', 'build', 'robots_delete', 'import_snapshot'):
        status, raw, _ = _req(port, 'POST', '/api/command', {'client': 'p', 'action': action, 'args': {}})
        assert _json(raw).get('error') == 'local_only', action
    status, raw, _ = _req(port, 'POST', '/api/command', {'client': 'other', 'action': 'estop', 'args': {}})
    assert _json(raw).get('error') != 'local_only' and status == 200     # E-STOP needs no lease


def test_remote_ui_contract():
    js = (UI_DIR / 'remote.js').read_text()
    app = (UI_DIR / 'app.js').read_text()
    css = (UI_DIR / 'fic.css').read_text()
    assert "'/api/login'" in js and "'/api/register'" in js and "'/api/logout'" in js
    assert 'robot_config' not in js.split('const NEEDS')[1].split('};')[0] and 'settings' not in js.split('const NEEDS')[1].split('};')[0]
    assert 'local_only' in app and 'FIC.local' in (UI_DIR / 'shell.js').read_text()
    assert '.signed-out' in css and '.view-only' in css and '.rnote' in css
    assert 'remote.js' in (UI_DIR / 'index.html').read_text()


def test_typography_two_families_one_scale():
    css = (UI_DIR / 'fic.css').read_text()
    assert '--serif' not in css and 'Georgia' not in css
    assert len(re.findall(r'^\s*--(?:sans|mono):', css, re.M)) == 2
    for tok in ('xs', 's', 'm', 'l', 'xl', 'xxl'):
        assert '--fs-%s:' % tok in css
    # no ad-hoc family or rem size outside the tokens
    assert not re.search(r"font-family: (?!var\(--(?:sans|mono)\)|inherit)", css)
    assert not re.search(r'font-size:\s*[0-9.]+rem', css)
    for f in UI_DIR.glob('*.js'):
        assert not re.search(r'font-size:\s*[0-9.]+rem', f.read_text()), f.name
    assert 'font-family: var(--sans)' in css.split('\nbody {')[1].split('}')[0] or 'body { font-family: var(--sans)' in css
