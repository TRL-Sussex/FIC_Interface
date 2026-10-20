"""Phase C of the web UI: full parity with the classic Studio window.

An API test for every command and snapshot section added to bring the classic
menus, toolbar, status bar, control-bar drawer, Dashboard, Settings, Robot
Configuration, Topic Monitor and Help into the browser. Same stand-in Studio as
the other web tests (real Qt checkboxes and combos), no ROS, no robot.
"""
import copy
import math
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.settings_mixin import DEFAULT_SETTINGS  # noqa: E402
from fic_studio.web_remote import actions  # noqa: E402
from fic_studio.web_remote.actions import build_snapshot  # noqa: E402
import pytest  # noqa: E402
from test_web_pages_b import _PageStudio  # noqa: E402
from test_web_remote_actions import ROBOT, WHO  # noqa: E402

LOCAL = dict(WHO, local=True)


class _Led:
    def __init__(self, status='idle'):
        self._status = status


class _Label:
    def __init__(self, text=''):
        self._t = text

    def text(self):
        return self._t


class _Box:
    def __init__(self, on=False):
        self.on = on

    def isChecked(self):
        return self.on

    def setChecked(self, v):
        self.on = bool(v)


class _GT:
    def __init__(self):
        self.items, self.cur = ['none', 'Shoulder_Link', 'ForeArm_Link'], 'none'

    def count(self):
        return len(self.items)

    def itemText(self, i):
        return self.items[i]

    def currentText(self):
        return self.cur

    def setCurrentText(self, t):
        self.cur = t


class _Studio(_PageStudio):
    """The page studio plus everything the classic window's chrome owns."""

    def __init__(self, tmp_path):
        super().__init__(tmp_path)
        self.tmp = tmp_path
        self.ros_led, self.fic_led, self.sim_led = _Led('connected'), _Led('idle'), _Led('idle')
        self.bags_led, self.il_led, self.teleop_led = _Led('idle'), _Led('connected'), _Led('active')
        self.gamepad_led, self._web_led = _Led('idle'), _Led('connected')
        self.il_label, self.teleop_status_bar_label = _Label('standard'), _Label('LEADER')
        self.gamepad_state_label, self._web_status_label = _Label('GAMEPAD'), _Label('WEB 1')
        self.robot_status_label = _Label('Robot: Kinova Gen3 7DOF (7 DOF)')
        self._kbd_on = False
        self.bags_running = self.plotter_running = self.contact_twin_running = False
        self.node = types.SimpleNamespace(
            station_id=1, station_robot_id='robot1', _active_robots=[1], active_robots=[1],
            add_robot=lambda n: self.node._active_robots.append(n),
            remove_robot=lambda n: self.node._active_robots.remove(n),
            publish_position=lambda p: self.calls.append(('preset', p)),
            get_topic_names_and_types=lambda: [('/robot1/joint_states', ['sensor_msgs/msg/JointState']),
                                               ('/rosout', ['rcl_interfaces/msg/Log'])],
            count_publishers=lambda n: 1, count_subscribers=lambda n: 2)
        self.presets = [types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0, z=1.31))]
        self._rc_live_publish = _Box()
        self._cb_gt_combo = _GT()
        self.settings = copy.deepcopy(DEFAULT_SETTINGS)
        self.saved = []
        self.persisted = 0
        self._extensions_manifests = [types.SimpleNamespace(id='sequences', name='Sequences', description='d', sidebar_icon='')]
        self.robots = {ROBOT: 7}
        self.robot_configs[ROBOT].update({'joint_names': ['j%d' % i for i in range(1, 8)], 'q_home_deg': [0.0] * 7,
                                          'joint_torque_limits': {'max': [39.0] * 7}, 'ee_frame_name': 'ee',
                                          'safety_zones': {'cartesian': {'x': {'min': -1.0, 'max': 1.0, 'enabled': True}}}})
        self.monitored_topics, self.topic_last_msg, self.topic_msg_count, self.topic_cached_msg = {}, {}, {}, {}
        self._tree_topic_types = {}
        self._selected_topic = None
        self.topic_signals = types.SimpleNamespace(message_received=types.SimpleNamespace(connect=lambda f: None))
        self._web_experiments_root = str(tmp_path / 'experiments')

    # settings and robot files
    def _save_settings(self):
        self.saved.append(copy.deepcopy(self.settings))

    def _apply_settings(self):
        self.calls.append(('apply_settings',))

    def _save_extensions_enabled(self, ids):
        self.settings.setdefault('extensions', {})['enabled'] = list(ids)

    def _rc_persist(self):
        self.persisted += 1
        return True

    def _rc_check_sync_state(self):
        return False

    def _rc_publish_hot_config(self, rid):
        self.calls.append(('hot', rid))

    # toolbar, menu and bar handlers
    def get_extension(self, ext_id):
        return types.SimpleNamespace(state_running=True) if ext_id == 'experiments' else None

    def _toggle_bags(self):
        self.calls.append(('bags',))
        self.bags_running = not self.bags_running

    def _toggle_plotter(self):
        self.calls.append(('plotter',))

    def _cb_on_contact_twin(self):
        self.calls.append(('viewer',))

    def _quick_start_experiment(self):
        self.calls.append(('exp',))

    def set_keyboard_control(self, on):
        self._kbd_on = on
        self.calls.append(('keyboard', on))

    def keyboard_state(self):
        return {'on': self._kbd_on, 'space': 'joint', 'joint': 2}

    def keyboard_web_keys(self, keys, tap=None):
        self.calls.append(('keys', list(keys), tap))
        return dict(self.keyboard_state(), ok=self._kbd_on)

    def _toggle_simulation(self):
        self.calls.append(('toggle_sim',))

    def _toggle_fic_simulation(self):
        self.calls.append(('toggle_fic',))

    def _toggle_controller(self):
        self.calls.append(('toggle_ctrl',))

    def _toggle_driver(self):
        self.calls.append(('toggle_drv',))

    def _dash_toggle_sim(self):
        self.calls.append(('launch_all',))

    def _stop_simulation(self):
        self.calls.append(('stop_sim',))
        self.simulation_running = False

    def _new_script(self):
        self.calls.append(('new_script',))

    def _open_experiments_folder(self):
        self.calls.append(('open_exp_folder',))

    def _open_experiment_dir(self, d):
        self.calls.append(('open_dir', d))

    def serve_docs(self, page):
        self.calls.append(('docs', page))
        return True

    def _web_close(self):
        self.calls.append(('close',))

    def _web_restart(self):
        self.calls.append(('restart',))

    def pause_for(self, rid):
        self.calls.append(('pause', rid))

    def stop_for(self, rid):
        self.calls.append(('stop', rid))

    def _reset_rc_defaults(self):
        self.calls.append(('reset',))

    def _rc_rebuild_robot_selector(self):
        self.calls.append(('selector',))

    def _rebuild_rc_control_area(self):
        self.calls.append(('area',))

    def _get_workspace_path(self):
        return str(self.tmp)

    # telemetry
    def joint_commanded_for(self, rid):
        return [0.10, 0.20, 0.30] + [0.0] * 4

    def joint_effort_for(self, rid):
        return [1.5, -2.5, 3.5] + [0.0] * 4

    def cartesian_error_for(self, rid):
        return (0.01, 0.02, 0.03, 0.04)

    def _rc_postural_toggle_state(self):
        return False


@pytest.fixture
def studio(tmp_path):
    return _Studio(tmp_path)


def run(studio, cmd_name, /, who=LOCAL, **args):
    args.setdefault('robot', ROBOT)
    return actions.handle_command(studio, cmd_name, args, who)


# ── Snapshot sections ────────────────────────────────────────────────────

def test_snapshot_carries_the_status_bar_the_toolbar_and_the_drawer(studio):
    snap = build_snapshot(studio)
    assert not snap.get('errors'), snap.get('errors')
    sb = snap['statusbar']
    assert [k for k in sb if k != 'robot'] == ['ros', 'fic', 'sim', 'bags', 'inner', 'teleop', 'gamepad', 'web']
    assert sb['ros']['status'] == 'connected' and sb['fic']['status'] == 'idle'
    assert sb['teleop'] == {'status': 'active', 'text': 'LEADER'} and sb['robot'].endswith('(7 DOF)')
    tb = snap['toolbar']
    assert tb['exp'] == {'visible': True, 'running': True} and tb['bags'] is False
    assert [s['label'] for s in tb['stations']][:2] == ['Robot 1', 'Robot 2'] and tb['stations'][-1]['label'] == 'Master'
    assert snap['stations']['items'][0]['rid'] == 'robot1' and snap['stations']['next'] == 2
    drawer = snap['drawer']
    assert drawer['presets'] == [{'label': '(0.0, 0.0, 1.31)'}] and drawer['hardware'] is False
    assert drawer['ground_truth']['items'] == ['none', 'Shoulder_Link', 'ForeArm_Link']


def test_presets_only_show_in_simulation_and_hardware_shows_pause_and_stop(studio):
    studio.control_target = 'hardware'
    drawer = build_snapshot(studio)['drawer']
    assert drawer['presets'] == [] and drawer['hardware'] is True


def test_dashboard_section_follows_the_classic_tabs(studio):
    studio.joint_positions_for = lambda rid: [0.05, 0.10, 0.28] + [0.0] * 4
    d = build_snapshot(studio)['dashboard']
    j = d['joints'][0]
    assert j['label'] == 'J1' and j['commanded'] == pytest.approx(math.degrees(0.10))
    assert j['error'] == pytest.approx(math.degrees(0.10 - 0.05)) and j['torque'] == 1.5
    assert j['tau_max'] == 39.0 and j['pct_tau'] == pytest.approx(1.5 / 39.0 * 100)
    assert set(d['ee']) == {'commanded', 'pose', 'error', 'velocity', 'quaternion'}
    assert d['ee']['pose'][:3] == [0.4, 0.0, 0.5]
    assert d['wrench']['force'] == [1.0, 2.0, 2.0] and d['wrench']['force_mag'] == pytest.approx(3.0)
    assert d['wrench']['contact'] is False
    assert set(d['safety']) == {'joints', 'velocity', 'torque'} and d['safety']['torque'] == 'ok'
    assert d['robot']['model'] == 'Kinova Gen3 7DOF' and d['robot']['gripper'] == 'none' or d['robot']['gripper']
    assert 'joint states' in d['channels'] and d['channels']['joint effort'] == '7 values'


def test_dashboard_flags_a_torque_near_its_limit(studio):
    studio.joint_effort_for = lambda rid: [38.0] + [0.0] * 6
    assert build_snapshot(studio)['dashboard']['safety']['torque'] == 'warn'


def test_dashboard_safety_is_unknown_without_fresh_data(studio):
    studio.joint_update_time = lambda rid: None                      # never heard from the robot
    assert set(build_snapshot(studio)['dashboard']['safety'].values()) == {'unknown'}
    import time as _t
    studio.joint_update_time = lambda rid: _t.monotonic() - 5.0      # 5 s old
    assert set(build_snapshot(studio)['dashboard']['safety'].values()) == {'unknown'}
    studio.joint_update_time = lambda rid: _t.monotonic()
    studio.robot_configs[ROBOT]['joint_limits_deg'] = {'lower': [-10] * 7, 'upper': [10] * 7}
    studio.joint_positions_for = lambda rid: [0.5] + [0.0] * 6       # 28.6 deg: past the 10 deg limit
    assert build_snapshot(studio)['dashboard']['safety']['joints'] == 'violation'


def test_live_section_has_what_the_tuning_page_needs(studio):
    studio.joint_positions_for = lambda rid: [0.05, 0.10, 0.28] + [0.0] * 4
    live = build_snapshot(studio)['live']
    assert live['joint_error'][:3] == pytest.approx([0.05, 0.10, 0.02])
    assert live['joint_effort'][:3] == [1.5, -2.5, 3.5]
    assert live['wrench'][:3] == [1.0, 2.0, 2.0] and live['cart_error'] == [0.01, 0.02, 0.03, 0.04]


def test_the_tuning_curve_has_both_sides_the_references_and_the_classic_stats(studio):
    reply = run(studio, 'tuning_get', mode=1, joint=0)
    sym = reply['plot']['sym']
    assert sym['x'][0] == -sym['xr'] and sym['x'][-1] == sym['xr']
    assert sym['y'][0] == pytest.approx(sym['y'][-1])                       # the classic plots |effort|: a V/W
    assert min(sym['y']) >= 0 and sym['y'][len(sym['y']) // 2] == pytest.approx(0, abs=1e-6)
    assert sym['y'][1] == pytest.approx(sym['y'][-2]) and sym['y'][-1] == pytest.approx(reply['plot']['breakpoints'][-1]['y'], rel=1e-3)
    assert len(sym['linear']) == len(sym['x']) and sym['convergence']['x']
    assert max(abs(v) for v in sym['convergence']['x']) <= reply['plot']['breakpoints'][-1]['x'] * 0.8 + 1e-9
    stats = reply['plot']['stats']
    assert stats['n_s'] == 2 * len(stats['xb']) and stats['k_conv'] > 0
    assert {l['x'] for l in sym['lines'] if l['sat']} == set(stats['xb'])


def test_the_curve_can_be_drawn_over_the_axes_a_drag_holds(studio):
    sym = run(studio, 'tuning_plot', mode=1, joint=0, values={'stiffness': 50.0}, view=[0.4, 60.0])['plot']['sym']
    assert sym['xr'] == 0.4 and sym['yr'] == 60.0 and sym['x'][0] == -0.4 and sym['x'][-1] == 0.4
    for bad in ([0, 1], [1], ['a', 1], [float('inf'), 1], 'x'):       # anything odd: the curve's own range
        own = run(studio, 'tuning_plot', mode=1, joint=0, view=bad)['plot']['sym']
        assert own['xr'] == pytest.approx(2.8 * max(line['x'] for line in own['lines'] if line['sat']))


def test_the_curve_follows_unsaved_edits_on_both_sides(studio):
    a = run(studio, 'tuning_plot', mode=1, joint=0, values={'stiffness': 50.0})['plot']['sym']
    b = run(studio, 'tuning_plot', mode=1, joint=0, values={'stiffness': 150.0})['plot']['sym']
    i = len(a['x']) // 2 + 3
    assert b['y'][i] > a['y'][i] and b['y'][-i - 1] == pytest.approx(b['y'][i])


# ── Local only ───────────────────────────────────────────────────────────

@pytest.mark.parametrize('action', ['settings_set', 'settings_reset', 'settings_extensions', 'settings_gamepad', 'settings_web',
                                    'robots_save', 'robots_add', 'robots_duplicate', 'robots_delete', 'robots_import', 'robots_upload',
                                    'robots_import_yaml', 'robots_compile_driver', 'robots_open_folder', 'robots_viewer', 'import_snapshot', 'build',
                                    'recent_experiments'])
def test_configuration_commands_refuse_a_remote_client(studio, action):
    reply = run(studio, action, who=WHO, confirm=True)
    assert reply['error'] == 'local_only'


def test_exit_and_station_switch_are_local_only(studio):
    assert run(studio, 'menu', who=WHO, id='file.exit', confirm=True)['error'] == 'local_only'
    assert run(studio, 'station', who=WHO, op='set', n=2, confirm=True)['error'] == 'local_only'


# ── Toolbar ──────────────────────────────────────────────────────────────

def test_toolbar_toggles_need_confirmation(studio):
    for op, call in (('bags', 'bags'), ('plotter', 'plotter'), ('viewer', 'viewer'), ('exp', 'exp')):
        assert run(studio, 'toolbar', op=op)['error'] == 'confirm'
        assert (call,) not in studio.calls
        assert run(studio, 'toolbar', op=op, confirm=True)['ok']
        assert (call,) in studio.calls


def test_toolbar_exp_needs_the_experiments_extension(studio):
    studio.get_extension = lambda _id: None
    assert run(studio, 'toolbar', op='exp', confirm=True)['error'] == 'unavailable'


def test_keyboard_control(studio):
    assert run(studio, 'toolbar', op='teleop', value=True)['error'] == 'bad_args'
    assert 'teleop' not in build_snapshot(studio)['toolbar']
    assert run(studio, 'toolbar', op='keyboard', value=True)['ok'] and ('keyboard', True) in studio.calls
    assert run(studio, 'toolbar', op='nope')['error'] == 'bad_args'


def test_keyboard_keys_from_the_page_reach_the_driver(studio):
    assert run(studio, 'keyboard_keys', keys=['W'])['error'] == 'keyboard_off'
    studio._kbd_on = True
    reply = run(studio, 'keyboard_keys', keys=['W', 'SHIFT'], tap='3')
    assert reply['ok'] and reply['space'] == 'joint' and reply['joint'] == 2
    assert studio.calls[-1] == ('keys', ['W', 'SHIFT'], '3')
    assert build_snapshot(studio)['toolbar']['keyboard_drive'] == {'on': True, 'space': 'joint', 'joint': 2}


def test_keyboard_keys_are_refused_and_let_go_after_an_estop(studio):
    studio._kbd_on = True
    studio.estop_latch_state = lambda: {'latched': True}
    assert run(studio, 'keyboard_keys', keys=['W'])['error'] == 'estop_latched'
    assert studio.calls[-1] == ('keys', [], None)                # whatever was held is let go
    assert run(studio, 'keyboard_keys', keys=[])['ok']           # letting go is never refused


def test_keyboard_keys_need_control(studio):
    """Driving is a command like any other: a viewer without the lease cannot."""
    from fic_studio.web_remote.server import NO_LEASE
    assert 'keyboard_keys' not in NO_LEASE


def test_station_add_select_remove_and_set(studio):
    assert run(studio, 'station', op='add', n=2)['ok']
    assert studio.node._active_robots == [1, 2] and studio._rc_active_rid == 'robot2'
    assert run(studio, 'station', op='add', n=2)['error'] == 'bad_args'
    assert run(studio, 'station', op='select', n=1)['ok'] and studio._rc_active_rid == 'robot1'
    assert run(studio, 'station', op='select', n=9)['error'] == 'bad_args'
    assert run(studio, 'station', op='remove', n=2)['error'] == 'confirm'
    assert run(studio, 'station', op='remove', n=1, confirm=True)['error'] == 'refused'      # this Studio's own
    assert run(studio, 'station', op='remove', n=2, confirm=True)['ok'] and studio.node._active_robots == [1]
    assert run(studio, 'station', op='set', n=2)['error'] == 'confirm'
    assert run(studio, 'station', op='set', n=2, confirm=True)['restarting'] is True
    assert studio.settings['general']['station_id'] == 2 and ('restart',) in studio.calls
    assert run(studio, 'station', op='set', n=36, confirm=True)['error'] == 'bad_args'   # stations are 1-35
    studio.running = True
    assert run(studio, 'station', op='set', n=3, confirm=True)['error'] == 'session_running'


def test_pause_and_stop_a_station(studio):
    assert run(studio, 'robot_pause')['ok'] and ('pause', 'robot1') in studio.calls
    studio.node._active_robots.append(2)
    assert run(studio, 'robot_stop', n=2)['ok'] and ('stop', 'robot2') in studio.calls
    assert run(studio, 'robot_stop', n=4)['error'] == 'bad_args'


def test_drawer_commands(studio):
    assert run(studio, 'live_publish', value=True)['ok'] and studio._rc_live_publish.on
    assert run(studio, 'reset_defaults')['ok'] and ('reset',) in studio.calls
    assert run(studio, 'preset', index=0)['ok'] and studio.calls[-1][0] == 'preset'
    assert run(studio, 'preset', index=5)['error'] == 'bad_args'
    studio.control_target = 'hardware'
    assert run(studio, 'preset', index=0)['error'] == 'unavailable'
    assert run(studio, 'contact_truth', link='ForeArm_Link')['ok'] and studio._cb_gt_combo.cur == 'ForeArm_Link'
    assert run(studio, 'contact_truth', link='Nope')['error'] == 'bad_args'


# ── Menus ────────────────────────────────────────────────────────────────

def test_menu_toggles_go_to_the_studios_handlers_and_need_confirmation(studio):
    for item, call in (('sim.toggle_simulation', 'toggle_sim'), ('sim.toggle_fic', 'toggle_fic'),
                       ('sim.launch_all', 'launch_all'), ('sim.toggle_controller', 'toggle_ctrl'),
                       ('sim.toggle_driver', 'toggle_drv')):
        assert run(studio, 'menu', id=item)['error'] == 'confirm'
        assert run(studio, 'menu', id=item, confirm=True)['ok'] and (call,) in studio.calls


def test_menu_plain_entries_robot_info_and_unknown(studio):
    assert run(studio, 'menu', id='file.new_script')['ok'] and ('new_script',) in studio.calls
    assert run(studio, 'menu', id='robot.home')['ok'] and ('home',) in studio.calls
    assert run(studio, 'menu', id='tools.open_experiments_folder')['ok'] and ('open_exp_folder',) in studio.calls
    info = run(studio, 'menu', id='robot.info')['info']['rows']
    assert ['Robot', ROBOT] in info and ['Joints', 7] in info
    assert run(studio, 'menu', id='nope')['error'] == 'bad_args'
    assert run(studio, 'menu', id='help.wizard')['error'] == 'unavailable'


def test_stop_all_and_exit_need_confirmation(studio):
    studio.simulation_running = True
    assert run(studio, 'menu', id='sim.stop_all')['error'] == 'confirm' and studio.simulation_running
    assert run(studio, 'menu', id='sim.stop_all', confirm=True)['ok'] and not studio.simulation_running
    assert run(studio, 'menu', id='file.exit')['error'] == 'confirm' and ('close',) not in studio.calls
    assert run(studio, 'menu', id='file.exit', confirm=True)['ok'] and ('close',) in studio.calls


def test_help_docs_menu_entry_opens_the_docs(studio):
    assert run(studio, 'menu', id='help.docs', page='home')['opened'] is True and ('docs', 'home') in studio.calls


# ── Files: export, import snapshot, recent experiments ───────────────────

def test_export_snapshot_params_and_robots(studio):
    snap = run(studio, 'export', what='snapshot')
    assert snap['filename'].endswith('.xml') and '<robots>' in snap['text']
    params = run(studio, 'export', what='params')
    assert params['filename'].endswith('.yaml') and ROBOT in params['text'] and 'modes:' in params['text'] and 'joints:' in params['text']
    robots = run(studio, 'export', what='robots')
    assert ROBOT in robots['text']
    assert run(studio, 'export', what='nothing')['error'] == 'bad_args'


def test_import_snapshot_needs_confirm_valid_xml_and_keeps_a_backup(studio):
    xml = '<robots><robot name="X"/></robots>'
    assert run(studio, 'import_snapshot', text=xml)['error'] == 'confirm'
    assert run(studio, 'import_snapshot', text='<not xml', confirm=True)['error'] == 'bad_args'
    assert run(studio, 'import_snapshot', text='', confirm=True)['error'] == 'bad_args'
    before = open(studio._web_config_path).read()
    assert run(studio, 'import_snapshot', text=xml, confirm=True)['ok']
    assert open(studio._web_config_path).read() == xml
    assert open(studio._web_config_path + '.web-backup').read() == before


def test_recent_experiments_lists_and_opens(studio, tmp_path):
    root = tmp_path / 'experiments'
    (root / 'exp_a').mkdir(parents=True)
    (root / 'exp_a' / 'experiment.yaml').write_text('name: Pick and place\n')
    assert run(studio, 'recent_experiments')['items'] == [{'id': 'exp_a', 'name': 'Pick and place'}]
    assert run(studio, 'recent_experiments', open='exp_a')['ok'] and studio.calls[-1][0] == 'open_dir'
    assert run(studio, 'recent_experiments', open='nope')['error'] == 'bad_args'


# ── Build ────────────────────────────────────────────────────────────────

def test_build_needs_confirm_and_reports_output(studio, tmp_path, monkeypatch):
    import time
    (tmp_path / 'src' / 'pkg_a').mkdir(parents=True)
    monkeypatch.setattr('fic_studio.platform_utils.colcon_build_command', lambda pk=None: 'echo built %s' % (pk or 'all'))
    assert run(studio, 'build', op='workspace')['error'] == 'confirm'
    assert run(studio, 'build', op='packages')['packages'] == ['pkg_a']
    assert run(studio, 'build', op='package', package='nope', confirm=True)['error'] == 'bad_args'
    assert run(studio, 'build', op='workspace', confirm=True)['ok']
    for _ in range(50):
        status = run(studio, 'build', op='status')
        if not status['running']:
            break
        time.sleep(0.1)
    assert status['code'] == 0 and 'built all' in '\n'.join(status['lines'])
    assert run(studio, 'build', op='bogus', confirm=True)['error'] == 'bad_args'


# ── Settings ─────────────────────────────────────────────────────────────

def test_settings_get_describes_every_classic_card(studio):
    reply = run(studio, 'settings_get')
    titles = [c['title'] for c in reply['cards']]
    # Data (where recordings go) is the web page's own; the rest are the classic window's cards.
    from fic_studio import features
    vr = ['VR'] if features.headset() else []      # the public build has no headset support
    assert titles == ['Appearance', 'Startup', 'Network', 'Data', 'Simulation'] + vr + ['Playback tolerances', 'Shortcuts', 'Developer']
    assert [c['restart'] for c in reply['cards'] if c['title'] == 'Network'] == [True]
    keys = {f['key'] for c in reply['cards'] for f in c['fields']}
    assert {'general.default_robot', 'network.ros_domain_id', 'tolerances.timeout_seconds', 'shortcuts.estop',
            'advanced.save_rosbag', 'appearance.palette', 'appearance.brightness'} <= keys
    assert reply['values']['network.ros_domain_id'] == 0 and reply['extensions'][0]['id'] == 'sequences'
    assert reply['gamepad']['available'] is False and 'serving' in reply['web']


def test_settings_set_validates_saves_and_applies(studio):
    reply = run(studio, 'settings_set', values={'tolerances.timeout_seconds': 45, 'general.auto_save': False})
    assert reply['ok'] and studio.settings['tolerances']['timeout_seconds'] == 45 and studio.settings['general']['auto_save'] is False
    assert studio.saved and ('apply_settings',) in studio.calls
    for bad in ({'tolerances.timeout_seconds': 500}, {'general.auto_save': 'yes'}, {'nope.key': 1},
                {'general.default_robot': 'Nobody'}, {'general.auto_save_interval': 12.5}):
        assert run(studio, 'settings_set', values=bad)['error'] == 'bad_args', bad


def test_settings_theme_is_composed_from_palette_and_brightness(studio):
    run(studio, 'settings_set', values={'appearance.palette': 'cool', 'appearance.brightness': 'light'})
    assert studio.settings['appearance']['theme'] == 'cool_light'


def test_network_settings_need_confirmation_and_say_restart(studio):
    assert run(studio, 'settings_set', values={'network.ros_domain_id': 7})['error'] == 'confirm'
    assert studio.settings['network']['ros_domain_id'] == 0
    reply = run(studio, 'settings_set', values={'network.ros_domain_id': 7}, confirm=True)
    assert reply['ok'] and reply['restart'] is True and studio.settings['network']['ros_domain_id'] == 7


def test_settings_reset_needs_confirmation(studio):
    studio.settings['tolerances']['timeout_seconds'] = 99
    assert run(studio, 'settings_reset')['error'] == 'confirm'
    assert run(studio, 'settings_reset', confirm=True)['ok']
    assert studio.settings['tolerances']['timeout_seconds'] == DEFAULT_SETTINGS['tolerances']['timeout_seconds']


def test_settings_extensions_needs_confirm_and_real_ids(studio):
    assert run(studio, 'settings_extensions', enabled=['sequences'])['error'] == 'confirm'
    assert run(studio, 'settings_extensions', enabled=['nope'], confirm=True)['error'] == 'bad_args'
    assert run(studio, 'settings_extensions', enabled=['sequences'], confirm=True)['restart'] is True
    assert studio.settings['extensions']['enabled'] == ['sequences']


def test_settings_gamepad_and_web_remote(studio, monkeypatch):
    assert run(studio, 'settings_gamepad', op='deadzone', value=0.1)['error'] == 'unavailable'
    from fic_studio.mixins import gamepad_mixin as gp
    studio._gp_settings = lambda: studio.settings.setdefault('gamepad', {})
    studio._gp_save_settings = lambda: None
    studio._gp_deadzone, studio._gp_invert, studio._gp_pads = 0.12, {}, {}
    studio._gp_button_map = dict(gp._DEFAULT_BUTTON_MAP)
    studio._gp_on_deadzone = lambda v: setattr(studio, '_gp_deadzone', v)
    studio._gp_on_mapping_changed = lambda b, a: studio._gp_button_map.__setitem__(b, a)
    assert run(studio, 'settings_gamepad', op='deadzone', value=0.2)['gamepad']['deadzone'] == 0.2
    assert run(studio, 'settings_gamepad', op='deadzone', value=0.9)['error'] == 'bad_args'
    assert run(studio, 'settings_gamepad', op='invert', axis='LX', value=True)['gamepad']['invert']['LX'] is True
    assert run(studio, 'settings_gamepad', op='map', button='A', action='speed_up')['gamepad']['buttons'][4]['action'] == 'speed_up'
    assert run(studio, 'settings_gamepad', op='map', button='A', action='nonsense')['error'] == 'bad_args'
    saved = {}
    studio._web_settings = lambda: {'enabled': True, 'port': 8088}
    studio._web_save_setting = lambda k, v: saved.__setitem__(k, v)
    studio._web_start = studio._web_stop = studio._web_revoke_control = lambda: None
    assert run(studio, 'settings_web', op='port', value=9000)['ok'] and saved['port'] == 9000
    assert run(studio, 'settings_web', op='port', value=80)['error'] == 'bad_args'
    assert run(studio, 'settings_web', op='enable', value=False)['ok'] and saved['enabled'] is False
    assert run(studio, 'settings_web', op='revoke')['ok']


# ── Robot configuration ──────────────────────────────────────────────────

def test_robots_list_and_get(studio):
    listing = run(studio, 'robots_list')
    assert listing['items'][0]['id'] == ROBOT and listing['items'][0]['joints'] == 7 and listing['current'] == ROBOT
    got = run(studio, 'robots_get', id=ROBOT)
    assert got['config']['display_name'] == 'Kinova Gen3 7DOF' and 'messages' in got['status']
    assert run(studio, 'robots_get', id='Nobody')['error'] == 'bad_args'


def test_robots_save_needs_confirm_validates_and_merges(studio):
    patch = {'display_name': 'Kinova (lab)', 'safety_zones': {'cartesian': {'x': {'min': -0.5, 'max': 0.5, 'enabled': False}}}}
    assert run(studio, 'robots_save', id=ROBOT, config=patch)['error'] == 'confirm'
    assert studio.robot_configs[ROBOT]['display_name'] == 'Kinova Gen3 7DOF' and studio.persisted == 0
    reply = run(studio, 'robots_save', id=ROBOT, config=patch, confirm=True)
    assert reply['ok'] and studio.persisted == 1 and ('hot', ROBOT) in studio.calls
    cfg = studio.robot_configs[ROBOT]
    assert cfg['display_name'] == 'Kinova (lab)' and cfg['safety_zones']['cartesian']['x']['max'] == 0.5
    assert cfg['has_gripper'] is True and cfg['num_joints'] == 7                    # keys the form has no field for survive


@pytest.mark.parametrize('patch', [
    {'num_joints': 99}, {'display_name': ''}, {'q_home_deg': [0, 1]}, {'torque_sign': 2}, {'has_gripper': 'no'},
    {'joint_limits_deg': {'lower': [10] * 7, 'upper': [5] * 7}},
    {'safety_zones': {'cartesian': {'x': {'min': 1, 'max': 0, 'enabled': True}}}},
    {'safety_zones': {'velocity': {'max_joint_vel': -1, 'max_cart_vel': 1, 'enabled': True}}},
    {'default_ee_target': {'position': [0, 0], 'orientation': [0, 0, 0, 1]}},
])
def test_robots_save_rejects_bad_values_and_writes_nothing(studio, patch):
    before = copy.deepcopy(studio.robot_configs[ROBOT])
    assert run(studio, 'robots_save', id=ROBOT, config=patch, confirm=True)['error'] == 'bad_args'
    assert studio.robot_configs[ROBOT] == before and studio.persisted == 0


def test_robots_home_is_converted_to_radians(studio):
    run(studio, 'robots_save', id=ROBOT, config={'q_home_deg': [90.0] + [0.0] * 6}, confirm=True)
    assert studio.robot_configs[ROBOT]['q_home_rad'][0] == pytest.approx(math.pi / 2)


def test_robots_add_duplicate_delete(studio):
    assert run(studio, 'robots_add', id='bad id')['error'] == 'bad_args'
    assert run(studio, 'robots_add', id=ROBOT)['error'] == 'bad_args'
    assert run(studio, 'robots_add', id='MyArm')['ok'] and 'MyArm' in studio.robot_configs
    assert studio.robot_configs['MyArm']['urdf_path'] == 'menagerie/model/MyArm/URDF/MyArm.urdf'
    new = run(studio, 'robots_duplicate', id='MyArm')['id']
    assert new == 'MyArm_copy' and studio.robot_configs[new]['display_name'].endswith('(Copy)')
    assert run(studio, 'robots_delete', id=new)['error'] == 'confirm'
    assert run(studio, 'robots_delete', id=new, confirm=True)['ok'] and new not in studio.robot_configs
    assert run(studio, 'robots_delete', id=ROBOT, confirm=True)['error'] == 'refused'        # the robot in use


def test_robots_import_yaml_skips_existing_robots(studio):
    text = 'robots:\n  Newcomer:\n    display_name: New\n    num_joints: 3\n  %s:\n    display_name: Clash\n' % ROBOT
    assert run(studio, 'robots_import_yaml', text=text)['error'] == 'confirm'
    reply = run(studio, 'robots_import_yaml', text=text, confirm=True)
    assert reply['added'] == ['Newcomer'] and reply['skipped'] == [ROBOT]
    assert studio.robot_configs[ROBOT]['display_name'] == 'Kinova Gen3 7DOF'
    assert run(studio, 'robots_import_yaml', text='- a\n- b', confirm=True)['error'] == 'bad_args'


def test_robots_import_from_a_urdf_reports_what_it_cannot_read(studio, tmp_path):
    assert run(studio, 'robots_import', op='analyse', path=str(tmp_path / 'missing.urdf'))['error'] == 'bad_args'
    empty = tmp_path / 'empty'
    empty.mkdir()
    assert run(studio, 'robots_import', op='analyse', path=str(empty))['error'] == 'refused'
    assert run(studio, 'robots_import', op='create', path=str(empty))['error'] == 'confirm'
    assert run(studio, 'robots_import', op='status')['running'] is False


def test_robots_compile_driver_needs_confirm_and_a_build_setup(studio):
    assert run(studio, 'robots_compile_driver', id=ROBOT)['error'] == 'confirm'
    assert run(studio, 'robots_compile_driver', id=ROBOT, confirm=True)['error'] == 'unavailable'


def test_open_folder_and_viewer_need_a_model_on_disk(studio):
    assert run(studio, 'robots_open_folder', id=ROBOT)['error'] == 'unavailable'
    assert run(studio, 'robots_viewer', id=ROBOT)['error'] == 'unavailable'
    assert run(studio, 'robots_viewer', id='Nobody')['error'] == 'bad_args'


def test_robots_report_and_yaml_views_answer_without_files(studio):
    assert run(studio, 'robots_report', id=ROBOT)['error'] == 'unavailable'
    assert run(studio, 'robots_yaml', id=ROBOT)['ok']


# ── Topic monitor ────────────────────────────────────────────────────────

def test_topics_list_monitor_select_inspect(studio):
    listing = run(studio, 'topics', op='list')
    assert [t['name'] for t in listing['topics']] == ['/robot1/joint_states', '/rosout']
    assert listing['topics'][0]['ns'] == '/robot1' and listing['topics'][0]['pubs'] == 1 and listing['topics'][0]['kind'] == 'topic'
    started = []
    studio._start_monitoring = lambda n, k: (started.append((n, k)), studio.monitored_topics.__setitem__(n, object()))
    studio._save_monitored_topics = lambda: None
    studio._toggle_monitor = lambda n, k: studio._start_monitoring(n, k) if n not in studio.monitored_topics else studio.monitored_topics.pop(n)
    assert run(studio, 'topics', op='monitor', name='/rosout', value=True)['ok'] and started == [('/rosout', 'rcl_interfaces/msg/Log')]
    assert run(studio, 'topics', op='monitor', name='/nope', value=True)['error'] == 'bad_args'
    assert run(studio, 'topics', op='select', name='/robot1/joint_states')['ok']
    assert studio._selected_topic == '/robot1/joint_states' and ('/robot1/joint_states', 'sensor_msgs/msg/JointState') in started
    studio.topic_cached_msg['/robot1/joint_states'] = types.SimpleNamespace(
        get_fields_and_field_types=lambda: {'position': 'double[]', 'name': 'string[]'}, position=[0.5], name=['j1'])
    shown = run(studio, 'topics', op='inspect', since=0)
    assert shown['topic'] == '/robot1/joint_states' and {f['name'] for f in shown['fields']} == {'position', 'name'}
    assert run(studio, 'topics', op='clear')['ok']
    assert run(studio, 'topics', op='bogus')['error'] == 'bad_args'


def test_inspect_with_nothing_selected(studio):
    assert run(studio, 'topics', op='inspect')['topic'] is None


# ── Help ─────────────────────────────────────────────────────────────────

def test_help_lists_every_classic_section_and_serves_them(studio):
    sections = run(studio, 'help', op='list')['sections']
    assert [s['key'] for s in sections][:3] == ['getting_started', 'window_layout', 'pages']
    assert len(sections) == 13 and sections[-1]['key'] == 'about'
    body = run(studio, 'help', op='section', key='getting_started')
    assert body['title'] == 'Getting Started' and '<h2' in body['html']
    assert run(studio, 'help', op='section', key='keyboard_shortcuts')['ok']
    assert run(studio, 'help', op='section', key='nope')['error'] == 'bad_args'


def test_help_search_and_docs(studio):
    hits = run(studio, 'help', op='search', text='troubleshooting')['keys']
    assert 'troubleshooting' in hits and 'glossary' not in hits
    assert run(studio, 'help', op='docs', page='home')['opened'] is True


# ── Raw XML (the classic VIEW XML) ───────────────────────────────────────

def test_tuning_xml_reads_and_replaces_this_robots_block(studio):
    got = run(studio, 'tuning_xml', op='read')
    assert got['text'].startswith('<robot name="%s"' % ROBOT)
    edited = got['text'].replace('<stiffness>200.000</stiffness>', '<stiffness>201.000</stiffness>', 1)
    assert run(studio, 'tuning_xml', op='save', text=edited)['error'] == 'confirm'
    assert run(studio, 'tuning_xml', op='save', text='<robot', confirm=True)['error'] == 'bad_args'
    assert run(studio, 'tuning_xml', op='save', text='<robot name="Other"/>', confirm=True)['error'] == 'bad_args'
    assert run(studio, 'tuning_xml', op='save', text=edited, confirm=True)['ok']
    assert '201.000' in run(studio, 'tuning_xml', op='read')['text']


def test_trace_section_gives_measured_and_target_end_effector(studio):
    studio.ee_target_for = lambda rid: (0.5, 0.1, 0.4)
    t = build_snapshot(studio)['trace']
    assert t['ee'] == [0.4, 0.0, 0.5] and t['target'] == [0.5, 0.1, 0.4] and t['source'] == 'streamed target'
    assert t['error'] == pytest.approx(math.dist([0.4, 0.0, 0.5], [0.5, 0.1, 0.4]))
    assert t['force'] == [1.0, 2.0, 2.0] and t['frame'] == 'base' and t['contact_n'] > 0


def test_trace_target_is_forward_kinematics_of_the_commanded_joints(studio):
    # one arm link of 1 m on a hinge about z: commanded 90 degrees puts the tip at y = 1
    scene = {'bodies': [{'name': 'world', 'parent': -1, 'pos': [0, 0, 0], 'quat': [1, 0, 0, 0]},
                        {'name': 'hand', 'parent': 0, 'pos': [1, 0, 0], 'quat': [1, 0, 0, 0]}],
             'joints': [{'body': 0, 'type': 'hinge', 'axis': [0, 0, 1], 'pos': [0, 0, 0]}],
             'drive_joints': [0]}
    studio._web_scene_provider = lambda i: scene
    studio.joint_commanded_for = lambda rid: [math.pi / 2]
    studio.robot_configs[ROBOT] = dict(studio.robot_configs.get(ROBOT, {}), ee_frame_name='hand')
    t = build_snapshot(studio)['trace']
    assert t['source'] == 'commanded joints'
    assert t['target'] == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)


def test_trace_target_is_none_without_a_source_and_never_breaks_the_snapshot(studio):
    snap = build_snapshot(studio)
    assert not snap.get('errors'), snap.get('errors')
    assert snap['trace']['target'] is None and snap['trace']['error'] is None
    studio.ee_target_for = lambda rid: (_ for _ in ()).throw(RuntimeError('boom'))
    assert build_snapshot(studio)['trace']['source'] is None


# ── Live appearance ──────────────────────────────────────────────────────

class _Sink:
    def __init__(self):
        self.events = []

    def emit(self, name, payload):
        self.events.append((name, payload))


def test_an_appearance_change_is_pushed_live_and_needs_no_restart(studio):
    sink = _Sink()
    studio._web_event_sinks = [sink]
    before = build_snapshot(studio)['appearance']
    reply = run(studio, 'settings_set', values={'appearance.palette': 'cool', 'appearance.brightness': 'light',
                                                'appearance.font_size': 15})
    assert reply['ok'] and reply['restart'] is False
    (name, payload), = sink.events
    assert name == 'settings' and payload['restart'] is False
    assert payload['changed'] == ['appearance.brightness', 'appearance.font_size', 'appearance.palette']
    a = payload['appearance']
    assert (a['palette'], a['brightness'], a['font_size']) == ('cool', 'light', 15)
    assert a['rev'] != before['rev'] and build_snapshot(studio)['appearance'] == a
    assert set(a['accent']) == {'light', 'dark'}


def test_non_appearance_settings_emit_nothing_and_network_still_says_restart(studio):
    sink = _Sink()
    studio._web_event_sinks = [sink]
    run(studio, 'settings_set', values={'tolerances.timeout_seconds': 45})
    assert sink.events == []
    reply = run(studio, 'settings_set', values={'network.ros_domain_id': 7}, confirm=True)
    assert reply['restart'] is True and sink.events == []
    cards = {c['title']: c for c in run(studio, 'settings_get')['cards']}
    assert cards['Network']['restart'] is True and cards['Appearance']['restart'] is False


def test_editorial_is_gone_and_a_saved_editorial_theme_migrates(studio):
    from fic_studio.web_remote import settings_page
    cards = {c['title']: c for c in run(studio, 'settings_get')['cards']}
    palette = next(f for f in cards['Appearance']['fields'] if f['key'] == 'appearance.palette')
    assert [c['value'] for c in palette['choices']] == ['warmth', 'cool']
    studio.settings['appearance']['theme'] = 'editorial_light'
    a = settings_page.appearance_state(studio)
    assert (a['palette'], a['brightness']) == ('warmth', 'light')
