"""Phase B of the web UI: the commands and snapshot sections behind each page.

Stand-in Studio (see test_web_remote_actions), a temporary copy of config.xml
for tuning, and Qt-free fakes for the fleet and locomotion. No ROS, no robot.
"""
import math
import os
import shutil
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.web_remote import actions, safety  # noqa: E402
from fic_studio.web_remote.actions import build_snapshot  # noqa: E402
from fic_studio.xml_manager import XMLManager, find_config_path  # noqa: E402
import pytest  # noqa: E402
from test_web_remote_actions import ROBOT, WHO, _Studio  # noqa: E402


class _PageStudio(_Studio):
    def __init__(self, tmp_path=None):
        super().__init__()
        self.published = []
        self.param_updates = []
        self.safety_enabled = True
        self.safety_enforce = True
        self.safety_limits = {}
        self.warning_threshold = 0.9
        self.robots = {ROBOT: 7}
        self.log_viewer = types.SimpleNamespace(
            _all_logs=[{'seq': i, 'timestamp': '12:00:%02d' % i, 'level': lv, 'source': 'S',
                        'message': 'm%d' % i} for i, lv in enumerate(
                            ['INFO', 'WARN', 'ERROR', 'INFO'], start=1)],
            clear_logs=lambda: setattr(self.log_viewer, '_all_logs', []))
        if tmp_path is not None:
            copy = tmp_path / 'config.xml'
            shutil.copy(find_config_path(), copy)
            self._web_config_path = str(copy)

    def _publish_safety_limits(self):
        self.published.append(dict(self.safety_limits))

    def _publish_param_update(self, kind, value=''):
        self.param_updates.append((kind, value))

    def _ee(self, x, y, z):
        ns = types.SimpleNamespace
        self.ee_pose_for = lambda rid: ns(position=ns(x=x, y=y, z=z),
                                          orientation=ns(x=0, y=0, z=0, w=1))


def run(studio, action, **args):
    args.setdefault('robot', ROBOT)
    return actions.handle_command(studio, action, args, WHO)


@pytest.fixture
def studio(tmp_path):
    return _PageStudio(tmp_path)


# ── Session ──────────────────────────────────────────────────────────────
def test_session_state_says_idle_partial_running(studio):
    snap = build_snapshot(studio)['session']
    assert snap['state'] == 'idle' and snap['parts_total'] == 2
    assert [p['id'] for p in snap['process_list']] == list(actions.PROCESSES)
    studio.simulation_running = True
    assert build_snapshot(studio)['session']['state'] == 'partial'
    studio.fic_running = True
    assert build_snapshot(studio)['session']['state'] == 'running'


def test_process_needs_confirmation(studio):
    started = []
    studio._start_simulation = lambda: started.append(1)
    reply = run(studio, 'process', id='simulation', op='start')
    assert reply['error'] == 'confirm' and not started


def test_process_start_and_stop_call_the_studio_handlers(studio):
    calls = []
    studio.simulation_running = False
    studio._start_simulation = lambda: (calls.append('start'), setattr(studio, 'simulation_running', True))
    studio._stop_simulation = lambda: (calls.append('stop'), setattr(studio, 'simulation_running', False))
    assert run(studio, 'process', id='simulation', op='start', confirm=True)['ok']
    assert run(studio, 'process', id='simulation', op='start', confirm=True).get('already')
    assert run(studio, 'process', id='simulation', op='stop', confirm=True)['ok']
    assert calls == ['start', 'stop']


def test_process_rejects_unknown_ids_and_ops(studio):
    assert run(studio, 'process', id='nuclear', op='start', confirm=True)['error'] == 'bad_args'
    assert run(studio, 'process', id='fic', op='restart', confirm=True)['error'] == 'bad_args'


def test_process_start_refused_just_after_estop_and_without_driver(studio):
    studio._start_driver = lambda: None
    run(studio, 'estop')
    assert run(studio, 'process', id='fic', op='start', confirm=True)['error'] == 'estop_holdoff'
    studio._web_estop_time = -1e9
    studio.driver_cmd = ''
    assert run(studio, 'process', id='driver', op='start', confirm=True)['error'] == 'no_driver'


def test_teleop_start_pairs_two_stations(studio):
    requests = []
    studio._teleop = types.SimpleNamespace(
        request=lambda *a, **k: (requests.append((a, k)), types.SimpleNamespace(sid='abc'))[1],
        sessions={})
    studio._fleet_dir = types.SimpleNamespace(
        station=lambda n: {'robot': ROBOT, 'joints': 7}, studios=lambda: [], conflicts=lambda: {})
    assert run(studio, 'teleop', op='start', leader=1, follower=2)['error'] == 'confirm'
    assert run(studio, 'teleop', op='start', leader=1, follower=1, confirm=True)['error'] == 'bad_args'
    assert run(studio, 'teleop', op='start', leader=1, follower=2, scale=9, confirm=True)['error'] == 'bad_args'
    reply = run(studio, 'teleop', op='start', leader=1, follower=2, scale=0.5, max_speed_deg=20, confirm=True)
    assert reply['ok'] and reply['sid'] == 'abc'
    (args, kwargs), = requests
    assert args == (1, 2) and kwargs['scale'] == 0.5 and math.isclose(kwargs['max_speed'], math.radians(20))


def test_teleop_start_refuses_different_robots(studio):
    studio._teleop = types.SimpleNamespace(request=lambda *a, **k: None, sessions={})
    cards = {1: {'robot': 'A'}, 2: {'robot': 'B'}}
    studio._fleet_dir = types.SimpleNamespace(station=cards.get)
    assert run(studio, 'teleop', op='start', leader=1, follower=2, confirm=True)['error'] == 'refused'


# ── Log ──────────────────────────────────────────────────────────────────
def test_log_read_returns_newer_lines_and_the_last_sequence(studio):
    reply = run(studio, 'log_read')
    assert [e['seq'] for e in reply['entries']] == [1, 2, 3, 4] and reply['last'] == 4
    reply = run(studio, 'log_read', after=2)
    assert [e['message'] for e in reply['entries']] == ['m3', 'm4']
    assert run(studio, 'log_read', limit=1)['entries'][0]['seq'] == 4
    assert build_snapshot(studio)['log'][-1]['seq'] == 4


def test_log_clear_needs_confirmation(studio):
    assert run(studio, 'log_clear')['error'] == 'confirm'
    assert len(studio.log_viewer._all_logs) == 4
    assert run(studio, 'log_clear', confirm=True)['ok']
    assert len(studio.log_viewer._all_logs) == 0


# ── Safety ───────────────────────────────────────────────────────────────
def test_safety_state_reports_zones_against_the_end_effector(studio):
    studio._ee(0.5, 0.0, 1.45)
    st = build_snapshot(studio)['safety']
    assert st['cartesian']['x']['state'] == 'ok'
    assert st['cartesian']['z']['state'] == 'warn'            # within 10 % of z max 1.5
    studio._ee(2.0, 0.0, 0.5)
    st = build_snapshot(studio)['safety']
    assert st['cartesian']['x']['state'] == 'violation' and st['overall'] == 'violation'
    studio.safety_enabled = False
    assert build_snapshot(studio)['safety']['overall'] == 'off'


def test_axis_state_follows_the_monitors_rule():
    lim = {'min': 0.0, 'max': 1.0, 'enabled': True}
    assert safety.axis_state(0.5, lim, 0.9) == 'ok'
    assert safety.axis_state(0.05, lim, 0.9) == 'warn'
    assert safety.axis_state(-0.1, lim, 0.9) == 'violation'
    assert safety.axis_state(0.5, dict(lim, enabled=False), 0.9) == 'off'
    assert safety.axis_state(None, lim, 0.9) == 'off'


def test_safety_edit_needs_confirm_and_changes_nothing_without_it(studio):
    reply = run(studio, 'safety_set', cartesian={'x': {'max': 0.5}})
    assert reply['error'] == 'confirm'
    assert studio.safety_limits == {} and studio.published == []


def test_safety_edit_applies_publishes_and_logs(studio):
    reply = run(studio, 'safety_set', confirm=True,
                cartesian={'x': {'min': -0.4, 'max': 0.6}, 'z': {'enabled': False}})
    assert reply['ok']
    assert studio.safety_limits['cartesian']['x'] == {'min': -0.4, 'max': 0.6, 'enabled': True}
    assert studio.safety_limits['cartesian']['z']['enabled'] is False
    assert studio.safety_limits['cartesian']['y']['max'] == 1.0          # untouched
    assert len(studio.published) == 1
    assert reply['safety']['cartesian']['x']['max'] == 0.6
    level, text, source = studio.logs[-1]
    assert level == 'WARN' and source == 'Safety' and 'X -1.000..1.000' in text


@pytest.mark.parametrize('edit', [
    {'cartesian': {'x': {'min': 1.0, 'max': 0.5}}},                    # min above max
    {'cartesian': {'x': {'max': 'far'}}},                              # not a number
    {'cartesian': {'x': {'max': float('nan')}}},
    {'cartesian': {'x': {'max': 50.0}}},                               # beyond the workspace bound
    {'cartesian': {'w': {'max': 1.0}}},                                # no such axis
    {'cartesian': {'x': {'colour': 1}}},                               # unknown field
    {'cartesian': {'x': {'enabled': 'yes'}}},                          # not a boolean
    {'velocity': {'max_joint_vel': 0}},
    {'velocity': {'max_joint_vel': 99}},
    {'velocity': {'max_cart_vel': -1}},
    {'joints': {'lower': [-1.0] * 7}},                                 # upper missing
    {'joints': {'lower': [-5.0] * 7, 'upper': [5.0] * 6}},             # wrong length
    {'joints': {'lower': [5.0] * 7, 'upper': [-5.0] * 7}},             # lower above upper
    {'joints': {'lower': [-50.0] * 7, 'upper': [5.0] * 7}},            # outside mechanical -10..10
    {'monitor': 'off'},
    {},
])
def test_safety_edit_rejects_bad_values_without_applying(studio, edit):
    reply = run(studio, 'safety_set', confirm=True, **edit)
    assert reply['ok'] is False and reply['error'] == 'bad_args'
    assert studio.safety_limits == {} and studio.published == []


def test_safety_joint_limits_inside_the_mechanical_range_apply(studio):
    reply = run(studio, 'safety_set', confirm=True,
                joints={'lower': [-8.0] * 7, 'upper': [8.0] * 7})
    assert reply['ok']
    rows = reply['safety']['joints']['rows']
    assert rows[0]['lower'] == -8.0 and rows[0]['mech_upper'] == 10.0


def test_safety_velocity_and_switches(studio):
    reply = run(studio, 'safety_set', confirm=True, velocity={'max_joint_vel': 0.5, 'max_cart_vel': 0.2},
                monitor=False, enforce=False)
    assert reply['ok'] and studio.safety_enabled is False and studio.safety_enforce is False
    assert studio.safety_limits['velocity']['max_joint_vel'] == 0.5
    assert reply['safety']['overall'] == 'off'


def test_safety_edit_is_blocked_for_a_simulation_only_account_on_hardware(studio):
    studio.control_target = 'hardware'
    who = dict(WHO, limits={'sim_only': True})
    reply = actions.handle_command(studio, 'safety_set', {'confirm': True, 'monitor': False}, who)
    assert reply['error'] == 'simulation_only' and studio.safety_enabled is True


# ── Tuning ───────────────────────────────────────────────────────────────
def test_tuning_info_lists_configs_modes_and_axes(studio):
    info = run(studio, 'tuning_info')
    assert info['ok'] and info['robot'] == ROBOT and 'default' in info['profiles']
    assert [m['id'] for m in info['modes']] == list(range(1, 9)) + [100]          # and H-PMP's own tuning
    assert info['axes'] == ['x', 'y', 'z', 'q', 'elbow', 'postural']


def test_tuning_get_reads_the_xml_and_describes_the_form(studio):
    reply = run(studio, 'tuning_get', mode=1, joint=0)
    assert reply['ok']
    mgr = XMLManager(studio._web_config_path)
    expected = mgr.load_joint_params(ROBOT, 0, 1, 'default')
    assert reply['values']['stiffness'] == expected.stiffness
    assert reply['values']['error_max'] == expected.error_max
    keys = {f['key'] for f in reply['fields']}
    assert 'stiffness' in keys and 'planner_frequency' not in keys        # impedance: no planner
    assert reply['plot']['x'][0] == 0 and len(reply['plot']['x']) == len(reply['plot']['y'])
    assert [b['x'] for b in reply['plot']['breakpoints']] == expected.error_max


def test_tuning_get_shows_planner_and_admittance_groups_by_mode(studio):
    keys = lambda **a: {f['group'] for f in run(studio, 'tuning_get', **a)['fields']}  # noqa: E731
    assert keys(mode=2, joint=0) == {'controller', 'planner'}
    assert keys(mode=3, joint=0) == {'controller', 'admittance'}
    assert keys(mode=7, axis='x') == {'controller', 'planner', 'admittance'}
    assert keys(axis='elbow') == {'controller'}
    assert keys(axis='postural') == {'controller', 'planner'}


@pytest.mark.parametrize('args', [
    {'mode': 9}, {'mode': 'one'}, {'mode': 1, 'joint': 99}, {'mode': 1, 'joint': 'a'},
    {'mode': 5, 'axis': 'w'}, {'mode': 1, 'joint': 0, 'profile': ' x'},
])
def test_tuning_get_rejects_bad_targets(studio, args):
    assert run(studio, 'tuning_get', **args)['error'] == 'bad_args'


def test_tuning_save_needs_confirm_and_leaves_the_file_alone(studio):
    before = open(studio._web_config_path).read()
    reply = run(studio, 'tuning_save', mode=1, joint=0, values={'stiffness': 150.0})
    assert reply['error'] == 'confirm'
    assert open(studio._web_config_path).read() == before and studio.param_updates == []


def test_tuning_save_writes_the_xml_and_tells_the_controller(studio):
    reply = run(studio, 'tuning_save', mode=1, joint=0, confirm=True,
                values={'stiffness': 150.0, 'force_max': [10.0, 20.0, 40.0]})
    assert reply['ok'] and 'joint 0' in reply['saved']
    saved = XMLManager(studio._web_config_path).load_joint_params(ROBOT, 0, 1, 'default')
    assert saved.stiffness == 150.0 and saved.force_max == [10.0, 20.0, 40.0]
    assert studio.param_updates == [('joint', 0)]
    assert studio.logs[-1][2] == 'Tuning'
    # another joint is untouched
    other = XMLManager(studio._web_config_path).load_joint_params(ROBOT, 1, 1, 'default')
    assert other.stiffness != 150.0 or other.force_max != [10.0, 20.0, 40.0]


def test_tuning_save_breakpoint_lists_and_symmetry(studio):
    reply = run(studio, 'tuning_save', mode=1, joint=0, confirm=True,
                values={'stiffness': 100.0, 'error_max': '0.05, 0.1', 'force_max': [10, 20]})
    assert reply['ok']
    saved = XMLManager(studio._web_config_path).load_joint_params(ROBOT, 0, 1, 'default')
    assert saved.error_max == [0.05, 0.1] and saved.force_max == [10.0, 20.0]
    assert saved.n_saturations == 4                                         # 2 x the list, symmetric


@pytest.mark.parametrize('values', [
    {'error_max': [0.1, 0.05], 'force_max': [10, 20]},                     # not ascending
    {'error_max': [0.05, 0.1], 'force_max': [20, 10]},
    {'error_max': [0.05, 0.1], 'force_max': [10]},                         # lengths differ
    {'stiffness': 1e5},                                                    # above the field bound
    {'stiffness': 5000.0, 'error_max': [0.05], 'force_max': [10.0]},       # exceeds force/error
    {'stiffness': 'stiff'}, {'stiffness': float('nan')}, {'stiffness': True},
    {'saturation_ratio': 1.5}, {'n_saturations': 3.5}, {'asymmetric': 'yes'},
    {'nonsense': 1}, {'error_max': 'a,b'},
])
def test_tuning_save_rejects_bad_values_and_writes_nothing(studio, values):
    before = open(studio._web_config_path).read()
    reply = run(studio, 'tuning_save', mode=1, joint=0, confirm=True, values=values)
    assert reply['ok'] is False and reply['error'] == 'bad_args'
    assert open(studio._web_config_path).read() == before and studio.param_updates == []


def test_tuning_save_cartesian_axis_and_robot_wide_blocks(studio):
    assert run(studio, 'tuning_save', mode=6, axis='y', confirm=True,
               values={'stiffness': 40.0, 'planner_vel_desired': 0.2})['ok']
    saved = XMLManager(studio._web_config_path).load_cartesian_axis(ROBOT, 6, 'y', 'default')
    assert saved.stiffness == 40.0 and saved.planner_vel_desired == 0.2
    assert run(studio, 'tuning_save', axis='elbow', confirm=True, values={'stiffness': 90.0})['ok']
    assert XMLManager(studio._web_config_path).load_elbow_params(ROBOT).stiffness == 90.0
    assert run(studio, 'tuning_save', axis='postural', confirm=True, values={'stiffness': 2.0})['ok']
    assert studio.param_updates == [('cartesian', 'y'), ('elbow', ''), ('postural', '')]


def test_tuning_plot_follows_unsaved_values_without_writing(studio):
    before = open(studio._web_config_path).read()
    a = run(studio, 'tuning_plot', mode=1, joint=0, values={'stiffness': 50.0})['plot']
    b = run(studio, 'tuning_plot', mode=1, joint=0, values={'stiffness': 150.0})['plot']
    assert a['y'][3] < b['y'][3]
    assert open(studio._web_config_path).read() == before


@pytest.fixture
def trial_studio(studio):
    """The stand-in Studio whose Config selector lists what config.xml holds, as the real one does."""
    combo = studio._rc_profile_dropdown

    def refresh():
        prev = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        for p in XMLManager(studio._web_config_path).get_all_profiles(ROBOT):
            combo.addItem(p, p)
        combo.setCurrentIndex(max(0, combo.findData(prev)))
        combo.blockSignals(False)
    studio._refresh_rc_profile_dropdown = refresh
    refresh()
    return studio


def test_a_live_trial_runs_a_copy_and_leaves_the_saved_config_alone(trial_studio):
    studio = trial_studio
    saved = XMLManager(studio._web_config_path).load_joint_params(ROBOT, 0, 1, 'default').stiffness
    assert run(studio, 'tuning_trial', op='start', profile='default')['error'] == 'confirm'
    reply = run(studio, 'tuning_trial', op='start', profile='default', confirm=True)
    assert reply['ok'] and reply['trial']['name'] == 'default~trial' and reply['active'] == 'default~trial'
    assert ('profile', 'default~trial') in studio.calls                     # the controller was switched to the copy
    assert run(studio, 'tuning_info')['trial'] == reply['trial']
    assert run(studio, 'tuning_trial', op='start', profile='default', confirm=True)['error'] == 'refused'
    # every edit goes to the copy and the controller reloads it; no confirm, the saved config is untouched
    applied = run(studio, 'tuning_trial', op='apply', mode=1, joint=0, profile='default', values={'stiffness': saved / 2})
    assert applied['ok'] and 'default~trial' in applied['applied'] and studio.param_updates == [('joint', 0)]
    mgr = XMLManager(studio._web_config_path)
    assert mgr.load_joint_params(ROBOT, 0, 1, 'default~trial').stiffness == saved / 2
    assert mgr.load_joint_params(ROBOT, 0, 1, 'default').stiffness == saved
    assert run(studio, 'tuning_trial', op='apply', axis='elbow', values={'stiffness': 1.0})['error'] == 'bad_args'
    assert run(studio, 'tuning_profile', op='delete', name='default~trial', confirm=True)['error'] == 'refused'
    # discard: back on the config it ran before, copy gone, nothing saved
    assert run(studio, 'tuning_trial', op='discard')['error'] == 'confirm'
    ended = run(studio, 'tuning_trial', op='discard', confirm=True)
    assert ended['ok'] and ended['trial'] is None and ended['active'] == 'default'
    mgr = XMLManager(studio._web_config_path)
    assert 'default~trial' not in mgr.get_all_profiles(ROBOT)
    assert mgr.load_joint_params(ROBOT, 0, 1, 'default').stiffness == saved
    assert run(studio, 'tuning_trial', op='apply', mode=1, joint=0, values={'stiffness': 1.0})['error'] == 'refused'


def test_keeping_a_trial_writes_it_over_the_config_and_makes_it_active(trial_studio):
    studio = trial_studio
    assert run(studio, 'tuning_profile', op='add', name='gentle', mode=1, confirm=True)['ok']
    studio._refresh_rc_profile_dropdown()
    other = XMLManager(studio._web_config_path).load_joint_params(ROBOT, 0, 2, 'default').stiffness
    assert run(studio, 'tuning_trial', op='start', profile='gentle', confirm=True)['trial']['back'] == 'default'
    assert run(studio, 'tuning_trial', op='apply', mode=1, joint=0, values={'stiffness': 21.0})['ok']
    assert run(studio, 'tuning_trial', op='apply', mode=2, joint=0, values={'stiffness': 22.0})['ok']
    ended = run(studio, 'tuning_trial', op='keep', confirm=True)
    assert ended['ok'] and ended['trial'] is None and ended['active'] == 'gentle'
    mgr = XMLManager(studio._web_config_path)
    assert mgr.load_joint_params(ROBOT, 0, 1, 'gentle').stiffness == 21.0
    assert mgr.load_joint_params(ROBOT, 0, 2, 'gentle').stiffness == 22.0      # a mode gentle had no block for
    assert mgr.load_joint_params(ROBOT, 0, 2, 'default').stiffness == other    # default untouched
    assert 'gentle~trial' not in mgr.get_all_profiles(ROBOT)


def test_a_trial_left_behind_is_found_again_from_the_file(trial_studio):
    studio = trial_studio
    assert run(studio, 'tuning_trial', op='start', profile='default', confirm=True)['ok']
    studio._web_tuning_trial = None                                         # the Studio restarted
    assert run(studio, 'tuning_info')['trial'] == {'name': 'default~trial', 'source': 'default', 'back': 'default'}
    assert run(studio, 'tuning_trial', op='discard', confirm=True)['ok']


def test_hpmp_is_a_tuning_set_of_its_own_with_a_joint_and_a_cartesian_layer(studio):
    info = run(studio, 'tuning_info')
    assert {'id': 100, 'name': 'H-PMP', 'type': 'hpmp', 'planner': True, 'admittance': True} in info['modes']
    joint = run(studio, 'tuning_get', mode=100, layer='joint', joint=1)
    assert joint['ok'] and joint['target']['kind'] == 'joint'
    assert {f['group'] for f in joint['fields']} == {'controller', 'planner', 'admittance'}
    axis = run(studio, 'tuning_get', mode=100, layer='cartesian', axis='y')
    assert axis['ok'] and axis['target']['kind'] == 'axis'
    assert {f['group'] for f in axis['fields']} == {'controller', 'planner'}         # no admittance in that layer
    assert 'diff_effort_max' not in {f['key'] for f in joint['fields'] + axis['fields']}   # no controller reads it


def test_saving_hpmp_leaves_modes_4_and_6_alone(studio):
    mgr = XMLManager(studio._web_config_path)
    before4, before6 = mgr.load_joint_params(ROBOT, 1, 4), mgr.load_cartesian_axis(ROBOT, 6, 'x')
    assert run(studio, 'tuning_save', mode=100, layer='joint', joint=1, confirm=True,
               values={'admittance_damping': 50.0})['ok']
    assert run(studio, 'tuning_save', mode=100, layer='cartesian', axis='x', confirm=True,
               values={'stiffness': 90.0})['ok']
    mgr = XMLManager(studio._web_config_path)
    assert mgr.load_joint_params(ROBOT, 1, 100).admittance_damping == 50.0
    assert mgr.load_cartesian_axis(ROBOT, 100, 'x').stiffness == 90.0
    assert mgr.load_joint_params(ROBOT, 1, 4) == before4 and mgr.load_cartesian_axis(ROBOT, 6, 'x') == before6
    assert mgr.load_joint_params(ROBOT, 1, 100).diff_effort_max == before4.diff_effort_max   # kept, not offered


def test_a_config_can_be_added_from_hpmp(studio):
    assert run(studio, 'tuning_profile', op='add', name='soft', mode=100, confirm=True)['ok']
    assert 'soft' in XMLManager(studio._web_config_path).get_available_profiles(ROBOT, 100)


def test_without_a_hpmp_block_the_page_shows_what_hpmp_runs_on(studio):
    import xml.etree.ElementTree as ET
    tree = ET.parse(studio._web_config_path)
    for robot in tree.getroot().iter('robot'):
        for m in [m for m in robot.findall('mode') if m.get('id') == '100']:
            robot.remove(m)
    tree.write(studio._web_config_path)
    reply = run(studio, 'tuning_get', mode=100, layer='joint', joint=2)
    assert reply['hpmp_layers'] and not reply['fallback']
    assert reply['values']['admittance_damping'] == XMLManager(studio._web_config_path).load_joint_params(ROBOT, 2, 4).admittance_damping
    assert run(studio, 'tuning_save', mode=100, layer='joint', joint=2, confirm=True, values={'stiffness': 150.0})['ok']
    mgr = XMLManager(studio._web_config_path)
    assert mgr.get_available_profiles(ROBOT, 100) == ['default']
    assert mgr.load_joint_params(ROBOT, 5, 100) == mgr.load_joint_params(ROBOT, 5, 4)   # the whole block came along
    assert not run(studio, 'tuning_get', mode=100, layer='joint', joint=2)['hpmp_layers']


def test_tuning_profile_add_and_delete(studio):
    assert run(studio, 'tuning_profile', op='add', name='gentle')['error'] == 'confirm'
    assert run(studio, 'tuning_profile', op='add', name='gentle', mode=1, confirm=True)['ok']
    assert 'gentle' in XMLManager(studio._web_config_path).get_all_profiles(ROBOT)
    assert run(studio, 'tuning_profile', op='add', name='gentle', mode=1, confirm=True)['error'] == 'refused'
    # a saved edit lands in that config only
    run(studio, 'tuning_save', mode=1, joint=0, profile='gentle', confirm=True, values={'stiffness': 33.0})
    mgr = XMLManager(studio._web_config_path)
    assert mgr.load_joint_params(ROBOT, 0, 1, 'gentle').stiffness == 33.0
    assert mgr.load_joint_params(ROBOT, 0, 1, 'default').stiffness != 33.0
    assert run(studio, 'tuning_profile', op='delete', name='gentle')['error'] == 'confirm'
    assert run(studio, 'tuning_profile', op='delete', name='gentle', confirm=True)['ok']
    assert 'gentle' not in XMLManager(studio._web_config_path).get_all_profiles(ROBOT)


def test_tuning_profile_protects_default_and_bad_names(studio):
    assert run(studio, 'tuning_profile', op='delete', name='default', confirm=True)['error'] == 'refused'
    assert run(studio, 'tuning_profile', op='delete', name='ghost', confirm=True)['error'] == 'bad_args'
    for bad in ('', ' x', 'a<b', 'a/b', 'x' * 60):
        assert run(studio, 'tuning_profile', op='add', name=bad, confirm=True)['error'] == 'bad_args'


# ── Fleet ────────────────────────────────────────────────────────────────
class _Directory:
    def __init__(self):
        self.cards = {
            1: {'station': 1, 'host': 'master', 'robot': ROBOT, 'robot_display': 'Kinova', 'target': 'simulation',
                'running': True, 'mode': 'Joint Impedance', 'teleop': [], 'urls': [], 'joints': 7},
            2: {'station': 2, 'host': 'robot2', 'robot': ROBOT, 'robot_display': 'Kinova', 'target': 'hardware',
                'running': False, 'mode': '', 'teleop': [], 'urls': ['http://robot2:8080'], 'joints': 7},
        }

    def studios(self):
        return list(self.cards.values())

    def station(self, n):
        return self.cards.get(n)

    def conflicts(self):
        return {}


@pytest.fixture
def fleet(studio):
    studio.sent = []
    studio._fleet_dir = _Directory()
    studio._fleet_session_send = lambda n, op, target=None: studio.sent.append(('session', n, op, target))
    studio._fleet_drive_now = lambda n: studio.sent.append(('drive', n))
    studio._fleet_estop_all = lambda: studio.sent.append(('estop_all',))
    studio._fleet_pause_all = lambda: studio.sent.append(('pause_all',))
    return studio


def test_fleet_state_lists_studios(fleet):
    st = build_snapshot(fleet)['fleet']
    assert st['available'] and [s['station'] for s in st['studios']] == [1, 2]
    assert st['studios'][0]['self'] is True and st['studios'][1]['self'] is False


def test_fleet_state_without_a_fleet(studio):
    assert build_snapshot(studio)['fleet']['available'] is False
    assert run(studio, 'fleet', op='drive', station=2)['error'] == 'unavailable'


def test_fleet_estop_all_needs_no_confirmation(fleet):
    assert run(fleet, 'fleet', op='estop_all')['ok']
    assert fleet.sent == [('estop_all',)]
    assert run(fleet, 'process', id='fic', op='start', confirm=True)['error'] == 'estop_holdoff'


def test_fleet_session_needs_confirmation_and_hardware_needs_two(fleet):
    assert run(fleet, 'fleet', op='session', station=2, session_op='start', target='simulation')['error'] == 'confirm'
    reply = run(fleet, 'fleet', op='session', station=2, session_op='start', target='hardware', confirm=True)
    assert reply['error'] == 'confirm' and reply['hardware'] is True and fleet.sent == []
    assert run(fleet, 'fleet', op='session', station=2, session_op='start', target='hardware',
               confirm=True, hardware=True)['ok']
    assert run(fleet, 'fleet', op='session', station=2, session_op='stop', confirm=True)['ok']
    assert fleet.sent == [('session', 2, 'start', 'hardware'), ('session', 2, 'stop', None)]


def test_fleet_refuses_unknown_and_own_station(fleet):
    assert run(fleet, 'fleet', op='session', station=9, session_op='stop', confirm=True)['error'] == 'bad_args'
    assert run(fleet, 'fleet', op='session', station=1, session_op='stop', confirm=True)['error'] == 'bad_args'
    assert run(fleet, 'fleet', op='session', station='x', session_op='stop', confirm=True)['error'] == 'bad_args'
    assert run(fleet, 'fleet', op='bomb')['error'] == 'bad_args'


def test_fleet_drive_a_hardware_arm_needs_confirmation(fleet):
    assert run(fleet, 'fleet', op='drive', station=2)['error'] == 'confirm'
    assert run(fleet, 'fleet', op='drive', station=2, confirm=True)['ok']
    assert fleet.sent == [('drive', 2)]


# ── Locomotion ───────────────────────────────────────────────────────────
@pytest.fixture
def legged(studio):
    studio.robot_configs[ROBOT]['robot_category'] = 'legged'
    studio.sent = []
    studio._loco_velocity = (0.0, 0.0, 0.0)
    studio._loco_send = lambda c: studio.sent.append(('gait', c))
    studio._loco_set = lambda k, v: studio.sent.append(('set', k, v))
    studio._loco_stop = lambda: setattr(studio, '_loco_velocity', (0.0, 0.0, 0.0))
    return studio


def test_locomotion_state_only_for_legged_robots(studio, legged):
    assert build_snapshot(legged)['locomotion']['available'] is True
    assert build_snapshot(legged)['locomotion']['gaits'] == ['sit', 'stand', 'walk']
    assert run(_PageStudio(), 'locomotion', op='stop')['error'] == 'unavailable'


def test_locomotion_gait_and_set(legged):
    assert run(legged, 'locomotion', op='gait', command='stand')['ok']
    assert run(legged, 'locomotion', op='gait', command='fly')['error'] == 'bad_args'
    assert run(legged, 'locomotion', op='set', key='step_height', value=0.1)['ok']
    assert run(legged, 'locomotion', op='set', key='step_height', value=0.9)['error'] == 'bad_args'
    assert run(legged, 'locomotion', op='set', key='cheat', value=0.1)['error'] == 'bad_args'
    assert legged.sent == [('gait', 'stand'), ('set', 'step_height', 0.1)]


def test_locomotion_velocity_is_bounded_and_held_briefly(legged):
    assert run(legged, 'locomotion', op='velocity', vx=0.3, vy=0.0, yaw=0.5)['ok']
    assert legged._loco_velocity == (0.3, 0.0, 0.5) and legged._loco_hold_until is not None
    assert run(legged, 'locomotion', op='velocity', vx=5.0)['error'] == 'bad_args'
    assert legged._loco_velocity == (0.3, 0.0, 0.5)                        # unchanged
    assert run(legged, 'locomotion', op='velocity', vx=0, vy=0, yaw=0)['ok']
    assert legged._loco_velocity == (0.0, 0.0, 0.0) and legged._loco_hold_until is None


def test_locomotion_refused_after_estop_and_estop_ends_a_held_walk(legged):
    run(legged, 'locomotion', op='velocity', vx=0.3)
    run(legged, 'estop')
    assert legged._loco_velocity == (0.0, 0.0, 0.0)
    assert run(legged, 'locomotion', op='velocity', vx=0.3)['error'] == 'estop_holdoff'


def test_a_held_velocity_lapses_in_the_locomotion_mixin(monkeypatch):
    from fic_studio.mixins import locomotion_mixin as lm
    published = []
    pub = types.SimpleNamespace(publish=published.append)
    obj = types.SimpleNamespace(_loco_velocity=(0.3, 0.0, 0.0), _loco_hold_until=100.0,
                                _loco_pub=lambda key: pub)
    monkeypatch.setattr(lm.time, 'monotonic', lambda: 99.0)
    lm.LocomotionMixin._loco_publish_velocity(obj)
    assert published[-1].linear.x == pytest.approx(0.3)
    monkeypatch.setattr(lm.time, 'monotonic', lambda: 101.0)
    lm.LocomotionMixin._loco_publish_velocity(obj)
    assert published[-1].linear.x == 0.0 and obj._loco_velocity == (0.0, 0.0, 0.0)


# ── Through the HTTP API, as the page calls them ─────────────────────────
import http.client  # noqa: E402
import json  # noqa: E402

from fic_studio.web_remote.lease import ControlLease  # noqa: E402
from fic_studio.web_remote.server import WebRemoteServer  # noqa: E402

TOKEN = 'tok-b'


@pytest.fixture
def api(studio):
    lease = ControlLease(timeout_s=30.0)
    server = WebRemoteServer(
        dispatch=lambda action, args, who: actions.handle_command(studio, action, args, who),
        snapshot=lambda: build_snapshot(studio), lease=lease, host='127.0.0.1', local_token=TOKEN)
    port = server.start(0)

    def call(path, body):
        conn = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
        conn.request('POST', path, body=json.dumps(body).encode(),
                     headers={'Content-Type': 'application/json', 'Cookie': 'fic_local=' + TOKEN})
        res = conn.getresponse()
        out = json.loads(res.read())
        conn.close()
        return res.status, out
    yield studio, call
    server.stop()


def test_safety_and_tuning_edits_over_http_need_the_lease_and_the_confirm(api):
    studio, call = api
    edit = {'client': 'ui', 'action': 'safety_set', 'args': {'cartesian': {'x': {'max': 0.7}}}}
    status, out = call('/api/command', edit)
    assert status == 403 and out['error'] == 'not_in_control'          # no lease: refused by the server
    assert call('/api/control', {'client': 'ui', 'op': 'take', 'force': True})[0] == 200
    status, out = call('/api/command', edit)
    assert out['ok'] is False and out['error'] == 'confirm'            # lease, but no confirm: refused by the Studio
    assert studio.safety_limits == {}
    edit['args']['confirm'] = True
    status, out = call('/api/command', edit)
    assert status == 200 and out['ok'] and studio.safety_limits['cartesian']['x']['max'] == 0.7
    assert out['safety']['cartesian']['x']['max'] == 0.7               # the reply carries the new state

    save = {'client': 'ui', 'action': 'tuning_save',
            'args': {'mode': 1, 'joint': 0, 'values': {'stiffness': 111.0}}}
    assert call('/api/command', save)[1]['error'] == 'confirm'
    save['args']['confirm'] = True
    out = call('/api/command', save)[1]
    assert out['ok'] and studio.param_updates == [('joint', 0)]
    got = call('/api/command', {'client': 'ui', 'action': 'tuning_get', 'args': {'mode': 1, 'joint': 0}})[1]
    assert got['values']['stiffness'] == 111.0
