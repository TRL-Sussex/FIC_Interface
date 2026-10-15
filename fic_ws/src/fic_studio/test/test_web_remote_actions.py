"""Web remote commands and snapshot, against a stand-in Studio.

The stand-in uses real Qt checkboxes, combos and sliders, so the tests see
the same signal behaviour the Studio does (setChecked firing toggled, and so
on); only the handlers those signals reach are recorded instead of publishing.
"""
import json
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from fic_studio.mixins.robot_control_mixin import (  # noqa: E402
    CONTROL_MODE_TABS, RobotControlMixin,
)
from fic_studio.web_remote import actions  # noqa: E402
from fic_studio.web_remote.actions import build_snapshot, handle_command  # noqa: E402
from fic_studio.web_remote.server import encode_json  # noqa: E402
from PyQt5.QtCore import Qt  # noqa: E402
from PyQt5.QtWidgets import QApplication, QCheckBox, QComboBox, QSlider  # noqa: E402
import pytest  # noqa: E402

_app = QApplication.instance() or QApplication([])

ROBOT = 'KinovaGen3_7DOF'
WHO = {'client': 'c1', 'name': 'Phone', 'addr': '10.0.0.9'}


class _Field:
    """CommandedVsActual / MetricField stand-in: clamps like the real one."""

    def __init__(self, lo, hi, value=0.0):
        self.lo, self.hi, self.v = lo, hi, value

    def set_commanded(self, v):
        self.v = max(self.lo, min(self.hi, float(v)))

    def commanded(self):
        return self.v

    set_value = set_commanded
    value = commanded


class _Sequences:
    """Sequences extension stand-in that records what it was asked to do."""

    def __init__(self):
        self.is_playing = False
        self.played = []

    def list_sequences(self):
        return ['pick', 'place']

    def play(self, name, loop=False):
        self.played.append((name, loop))

    def stop(self):
        self.played.append('stop')

    def capture(self, kind, into=None):
        self.played.append(('capture', kind, into))


class _Studio:
    APP_NAME = 'FIC Control Studio'
    APP_VERSION = 'test'

    def __init__(self, cfg=None):
        self.calls = []
        self.logs = []
        self.node = types.SimpleNamespace(station_robot_id='robot1', station_id=1)
        self.robot_dropdown = types.SimpleNamespace(currentText=lambda: ROBOT)
        self.robot_configs = {ROBOT: cfg or {
            'display_name': 'Kinova Gen3 7DOF', 'num_joints': 7, 'has_gripper': True,
            'joint_limits_deg': {'lower': [-10.0] * 7, 'upper': [10.0] * 7},
        }}
        self.control_target = 'simulation'
        self.running = False
        self.driver_cmd = 'cd x && ./driver'
        self._seq = None
        self._rc_active_rid = 'robot1'
        self._rc_selected_mode_idx = 0
        self._rc_selected_kind = 'joint'

        def box(label, key):
            b = QCheckBox(label)
            b.toggled.connect(lambda v: self.calls.append((key, v)))
            return b
        self._rc_admittance_planner_toggle = box('Admittance planner', 'planner')
        self._rc_postural_toggle = box('Postural', 'postural')
        self._rc_elbow_toggle = box('Elbow', 'elbow')
        self._rc_hpmp_toggle = box('Hpmp', 'hpmp')

        self._rc_profile_dropdown = QComboBox()
        self._rc_profile_dropdown.addItem('default', 'default')
        self._rc_profile_dropdown.addItem('Stiff  (fallback → default)', 'Stiff')
        self._rc_profile_dropdown.currentIndexChanged.connect(
            lambda i: self.calls.append(('profile', self._rc_profile_dropdown.itemData(i))))

        self._rc_gripper_slider = QSlider(Qt.Horizontal)
        self._rc_gripper_slider.setRange(0, 100)
        self._rc_gripper_slider.setValue(100)
        self._rc_gripper_slider.valueChanged.connect(self._on_rc_gripper_changed)

        cfg = self.robot_configs[ROBOT]
        self._rc_joint_meta = self._rc_display_joints(ROBOT, cfg)
        self._rc_joint_fields = [_Field(m['lo'], m['hi']) for m in self._rc_joint_meta]
        self._rc_pose_fields = {k: _Field(-1.0, 1.0)
                                for k in ('X', 'Y', 'Z', 'axis X', 'axis Y', 'axis Z')}
        self._rc_pose_fields['angle'] = _Field(-180.0, 180.0)
        self.log_viewer = types.SimpleNamespace(_all_logs=[
            {'timestamp': '12:00:00', 'level': 'INFO', 'source': 'System', 'message': 'started'}])

    # Studio methods the web remote calls.
    def _rc_display_joints(self, robot, cfg):
        return RobotControlMixin._rc_display_joints(self, robot, cfg)

    def _rc_command_rid(self):
        return self._rc_active_rid

    def _on_rc_mode_selected(self, idx, name):
        self.calls.append(('mode', idx, name))
        self._rc_selected_mode_idx = idx
        self._rc_selected_kind = CONTROL_MODE_TABS[idx][1]

    def _rc_publish_joints(self):
        self.calls.append(('joints', [f.v for f in self._rc_joint_fields]))

    def _rc_publish_pose_from_fields(self):
        self.calls.append(('pose', {k: f.v for k, f in self._rc_pose_fields.items()}))

    def _on_rc_gripper_changed(self, v):
        self.calls.append(('gripper', v))

    def _rc_on_home(self):
        self.calls.append(('home',))

    def _publish_desired_wrench(self, values, robot_id=None):
        self.calls.append(('desired_wrench', list(values)))

    def desired_wrench_for(self, rid):
        return [0.0] * 6

    def _cb_sequences(self):
        return self._seq

    def _tb_session_running(self):
        return self.running

    def _tb_set_target(self, target):
        self.calls.append(('target', target))
        self.control_target = target

    def _tb_toggle_start(self):
        self.calls.append(('toggle',))
        self.running = not self.running

    def _resolve_driver_command(self, robot):
        return self.driver_cmd

    def _emergency_stop(self):
        self.calls.append(('estop',))

    def _cb_on_calibrate(self):
        self.calls.append(('calibrate',))

    def log_message(self, level, message, source=''):
        self.logs.append((level, message, source))

    # Telemetry.
    def joint_positions_for(self, rid):
        return [0.01 * i for i in range(7)]

    def joint_velocity_for(self, rid):
        return [0.0] * 7

    def joint_torque_for(self, rid):
        return [1.0] * 7

    def sim_joint_by_name(self, rid):
        return {}

    def ee_pose_for(self, rid):
        ns = types.SimpleNamespace
        return ns(position=ns(x=0.4, y=0.0, z=0.5), orientation=ns(x=0.0, y=0.0, z=0.0, w=1.0))

    def wrench_for(self, rid):
        return (1.0, 2.0, 2.0, 0.0, 0.0, float('nan'))

    def contact_estimate_for(self, rid):
        return None

    def telemetry_health(self, rid):
        return (500.0, 3.0, 'live')


def run(studio, action, **args):
    args.setdefault('robot', ROBOT)
    return handle_command(studio, action, args, WHO)


@pytest.fixture
def studio():
    return _Studio()


# ── Modes, options, config ────────────────────────────────────────────────

def test_set_mode_goes_through_the_studio_handler(studio):
    assert run(studio, 'set_mode', index=6)['ok']
    assert studio.calls[-1] == ('mode', 6, CONTROL_MODE_TABS[6][0])


def test_cartesian_modes_are_refused_for_a_legged_robot():
    studio = _Studio(cfg={'robot_category': 'legged', 'num_joints': 12})
    reply = run(studio, 'set_mode', index=4)
    assert reply['error'] == 'no_cartesian'
    assert not any(c[0] == 'mode' for c in studio.calls)


def test_vr_target_locks_commanding(studio):
    studio.control_target = 'vr'
    assert run(studio, 'set_mode', index=1)['error'] == 'vr_target'
    assert run(studio, 'home')['error'] == 'vr_target'


def test_option_toggles_the_studio_checkbox(studio):
    assert run(studio, 'set_option', name='elbow', value=True)['ok']
    assert studio._rc_elbow_toggle.isChecked()
    assert ('elbow', True) in studio.calls
    assert run(studio, 'set_option', name='nope', value=True)['error'] == 'bad_args'


def test_desired_wrench_goes_to_the_studio(studio):
    w = [1.5, 0.0, -2.0, 0.0, 0.1, 0.0]
    assert run(studio, 'desired_wrench', values=w)['ok']
    assert studio.calls[-1] == ('desired_wrench', w)
    assert run(studio, 'desired_wrench', values=[1, 2, 3])['error'] == 'bad_args'
    assert run(studio, 'desired_wrench', values=[1, 2, 3, 'x', 0, 0])['error'] == 'bad_args'
    assert run(studio, 'desired_wrench', values=[float('nan')] + [0] * 5)['error'] == 'bad_args'


def test_snapshot_carries_the_desired_wrench(studio):
    snap = json.loads(encode_json(build_snapshot(studio)))
    assert snap['desired_wrench'] == [0.0] * 6


def test_profile_selects_by_name_not_label(studio):
    assert run(studio, 'set_profile', name='Stiff')['ok']
    assert studio.calls[-1] == ('profile', 'Stiff')
    assert run(studio, 'set_profile', name='Missing')['error'] == 'bad_args'


# ── Targets ───────────────────────────────────────────────────────────────

def test_send_joints_writes_the_desktop_fields_then_publishes(studio):
    reply = run(studio, 'send_joints', degrees=[1, 2, 3, 4, 5, 6, 99])
    assert reply['ok']
    assert studio.calls[-1] == ('joints', [1, 2, 3, 4, 5, 6, 10.0])   # clamped to the limit
    assert reply['commanded'][-1] == 10.0


def test_send_joints_refused_in_a_cartesian_mode(studio):
    run(studio, 'set_mode', index=4)
    assert run(studio, 'send_joints', degrees=[0] * 7)['error'] == 'wrong_kind'


def test_send_joints_refused_when_the_joint_list_changed(studio):
    assert run(studio, 'send_joints', degrees=[0] * 6)['error'] == 'joints_changed'


def test_send_joints_refused_for_another_robot(studio):
    args = {'robot': 'Franka_FR3', 'degrees': [0] * 7}
    reply = handle_command(studio, 'send_joints', args, WHO)
    assert reply['error'] == 'robot_changed'
    assert not any(c[0] == 'joints' for c in studio.calls)


def test_send_joints_rejects_non_numbers(studio):
    assert run(studio, 'send_joints', degrees=[0, 0, 'x', 0, 0, 0, 0])['error'] == 'bad_args'


def test_send_pose_writes_position_and_axis_angle(studio):
    run(studio, 'set_mode', index=4)
    assert run(studio, 'send_pose', x=0.4, y=0.1, z=0.3, ax=0, ay=0, az=1, angle=90)['ok']
    kind, pose = studio.calls[-1]
    assert kind == 'pose'
    got = (pose['X'], pose['Y'], pose['Z'], pose['axis Z'], pose['angle'])
    assert got == (0.4, 0.1, 0.3, 1.0, 90.0)


def test_send_pose_rejects_a_zero_axis_with_rotation(studio):
    run(studio, 'set_mode', index=4)
    reply = run(studio, 'send_pose', x=0, y=0, z=0, ax=0, ay=0, az=0, angle=30)
    assert reply['error'] == 'bad_args'


def test_gripper_clamps_and_resends_an_unchanged_width(studio):
    assert run(studio, 'gripper', percent=140)['ok']
    assert studio.calls[-1] == ('gripper', 100)             # slider already at 100
    run(studio, 'gripper', percent=25)
    assert studio.calls[-1] == ('gripper', 25)


# ── Home, E-stop, sessions ────────────────────────────────────────────────

def test_home_asks_first_while_a_sequence_plays(studio):
    studio._seq = _Sequences()
    studio._seq.is_playing = True
    reply = run(studio, 'home')
    assert reply['error'] == 'confirm'
    assert ('home',) not in studio.calls
    assert run(studio, 'home', confirm=True)['ok']
    assert studio.calls[-1] == ('home',)


def test_estop_stops_and_blocks_motion_briefly(studio):
    assert run(studio, 'estop')['ok']
    assert studio.calls[-1] == ('estop',)
    assert studio.logs[-1][0] == 'ERROR'
    assert run(studio, 'send_joints', degrees=[0] * 7)['error'] == 'estop_holdoff'
    assert run(studio, 'session', op='start', confirm=True)['error'] == 'estop_holdoff'
    studio._web_estop_time -= actions.ESTOP_HOLDOFF_S + 0.1
    assert run(studio, 'send_joints', degrees=[0] * 7)['ok']


def test_session_needs_confirmation(studio):
    assert run(studio, 'session', op='start')['error'] == 'confirm'
    assert ('toggle',) not in studio.calls
    assert run(studio, 'session', op='start', confirm=True)['ok']
    assert studio.calls[-1] == ('toggle',)


def test_session_start_is_not_a_blind_toggle(studio):
    studio.running = True
    reply = run(studio, 'session', op='start', confirm=True)
    assert reply == {'ok': True, 'already': True}
    assert ('toggle',) not in studio.calls


def test_hardware_start_without_a_driver_is_refused_before_any_dialog(studio):
    studio.control_target = 'hardware'
    studio.driver_cmd = ''
    assert run(studio, 'session', op='start', confirm=True)['error'] == 'no_driver'
    assert ('toggle',) not in studio.calls


def test_target_is_locked_while_a_session_runs(studio):
    assert run(studio, 'set_target', target='hardware')['ok']
    studio.running = True
    assert run(studio, 'set_target', target='simulation')['error'] == 'session_running'


def test_sequences_and_capture(studio):
    assert run(studio, 'sequence', op='play', name='pick')['error'] == 'unavailable'
    studio._seq = _Sequences()
    assert run(studio, 'sequence', op='loop', name='pick')['ok']
    assert run(studio, 'sequence', op='play', name='nope')['error'] == 'bad_args'
    assert run(studio, 'sequence', op='stop')['ok']
    assert run(studio, 'capture', kind='joint', sequence='place')['ok']
    assert studio._seq.played == [('pick', True), 'stop', ('capture', 'joint', 'place')]


def test_unknown_commands_are_refused(studio):
    assert run(studio, 'reboot')['error'] == 'unknown_action'


# ── Snapshot ──────────────────────────────────────────────────────────────

def test_snapshot_describes_what_the_desktop_shows(studio):
    studio._rc_joint_fields[2].set_commanded(4.5)
    snap = build_snapshot(studio)
    assert 'errors' not in snap
    assert snap['studio']['robot'] == ROBOT
    assert snap['mode']['name'] == 'Joint Impedance' and snap['mode']['kind'] == 'joint'
    assert [m['label'] for m in snap['mode']['menus']['cart']['modes']] == \
        ['Impedance', 'Imp. + Planner', 'Admittance', 'HAP-FIC']
    assert snap['options']['admittance_planner']['label'] == 'Admittance planner'
    assert snap['profiles']['items'][1] == {
        'name': 'Stiff', 'label': 'Stiff  (fallback → default)'}
    assert len(snap['joints']) == 7 and snap['joints'][2]['commanded'] == 4.5
    assert snap['joints'][1]['actual'] == pytest.approx(0.5729, abs=1e-3)  # 0.01 rad
    assert snap['pose']['actual']['x'] == 0.4 and snap['pose']['commanded'] is None
    assert snap['telemetry']['health']['state'] == 'live'
    assert snap['gripper']['percent'] == 100
    assert snap['log'][-1]['message'] == 'started'
    assert set(snap['theme']['tokens']) >= {'bg_base', 'accent_red'}
    # NaN in the wrench survives the trip to JSON as null.
    assert json.loads(encode_json(snap))['telemetry']['wrench'][5] is None


def test_snapshot_survives_a_broken_section(studio):
    def broken(rid):
        raise RuntimeError('no pose')
    studio.ee_pose_for = broken
    snap = build_snapshot(studio)
    assert snap['pose'] is None
    assert 'no pose' in snap['errors']['pose']
    assert snap['joints'] and snap['mode']


# ── Accounts that may use the simulation only ────────────────────────────

SIM_ONLY = dict(WHO, limits={'sim_only': True})


def run_limited(studio, action, **args):
    args.setdefault('robot', ROBOT)
    return handle_command(studio, action, args, SIM_ONLY)


def test_a_simulation_only_account_commands_a_simulated_robot(studio):
    studio.control_target = 'simulation'
    assert run_limited(studio, 'home')['ok'] is True


def test_a_simulation_only_account_cannot_command_hardware(studio):
    studio.control_target = 'hardware'
    reply = run_limited(studio, 'home')
    assert reply['ok'] is False and reply['error'] == 'simulation_only'
    assert 'HARDWARE' in reply['message']


def test_a_simulation_only_account_cannot_switch_to_hardware(studio):
    studio.control_target = 'simulation'
    reply = run_limited(studio, 'set_target', target='hardware')
    assert reply['ok'] is False and reply['error'] == 'simulation_only'
    assert run_limited(studio, 'set_target', target='simulation')['ok'] is True


def test_a_simulation_only_account_may_still_stop_everything(studio):
    studio.control_target = 'hardware'
    assert run_limited(studio, 'estop')['ok'] is True


def test_an_unlimited_account_is_not_affected(studio):
    studio.control_target = 'hardware'
    assert run(studio, 'home')['ok'] is True


# ── Teleoperation ────────────────────────────────────────────────────────
class _Pairing:
    """A teleop session, as the web remote needs to see it."""

    def __init__(self, sid, leader, follower, state='pending'):
        self.sid, self.leader, self.follower, self.state = sid, leader, follower, state
        self.scale, self.reason, self.holding, self.rtt = 1.0, '', False, None
        self.from_host = 'robot%d' % leader

    def as_dict(self):
        return {'sid': self.sid, 'leader': self.leader, 'follower': self.follower,
                'state': self.state, 'scale': self.scale, 'reason': self.reason,
                'holding': self.holding, 'rtt': self.rtt}


class _Teleop:
    """TeleopManager stand-in: records what the browser asked it to do."""

    def __init__(self, *pairings):
        self.sessions = {p.sid: p for p in pairings}
        self.accepted, self.rejected, self.stopped = [], [], []
        self.stopped_all = False

    def visible(self):
        return list(self.sessions.values())

    def accept(self, sid):
        self.accepted.append(sid)
        self.sessions[sid].state = 'active'
        return True

    def reject(self, sid, reason):
        self.rejected.append((sid, reason))

    def stop(self, sid, reason='stopped'):
        self.stopped.append((sid, reason))

    def stop_all(self, reason):
        self.stopped_all = True


def test_a_pending_request_is_published_to_the_browser(studio):
    studio._teleop = _Teleop(_Pairing('aa', leader=2, follower=1))
    tel = build_snapshot(studio)['teleop']
    assert tel['available'] and tel['station'] == 1
    assert [p['sid'] for p in tel['pending']] == ['aa']
    assert tel['sessions'][0]['role'] == 'follower'


def test_a_request_for_another_station_is_not_ours_to_answer(studio):
    studio._teleop = _Teleop(_Pairing('bb', leader=1, follower=3))
    tel = build_snapshot(studio)['teleop']
    assert tel['pending'] == []
    assert tel['sessions'][0]['role'] == 'leader'


def test_no_teleop_does_not_break_the_snapshot(studio):
    assert build_snapshot(studio)['teleop']['available'] is False
    assert 'teleop' not in (build_snapshot(studio).get('errors') or {})


def test_accepting_from_the_browser_follows(studio):
    studio._teleop = _Teleop(_Pairing('aa', leader=2, follower=1))
    assert run(studio, 'teleop', op='accept', sid='aa', confirm=True)['ok'] is True
    assert studio._teleop.accepted == ['aa']


def test_accepting_needs_confirmation(studio):
    studio._teleop = _Teleop(_Pairing('aa', leader=2, follower=1))
    reply = run(studio, 'teleop', op='accept', sid='aa')
    assert reply['ok'] is False and reply['error'] == 'confirm'
    assert studio._teleop.accepted == []


def test_a_stale_sid_still_answers_what_is_pending_now(studio):
    # The phone shows what was pending a moment ago; the request may have been
    # remade by the time a thumb lands.
    studio._teleop = _Teleop(_Pairing('new', leader=2, follower=1))
    assert run(studio, 'teleop', op='accept', sid='gone', confirm=True)['ok'] is True
    assert studio._teleop.accepted == ['new']


def test_answering_nothing_is_refused(studio):
    studio._teleop = _Teleop()
    reply = run(studio, 'teleop', op='accept', confirm=True)
    assert reply['ok'] is False and reply['error'] == 'nothing_pending'


def test_refusing_from_the_browser(studio):
    studio._teleop = _Teleop(_Pairing('aa', leader=2, follower=1))
    assert run(studio, 'teleop', op='reject', sid='aa', confirm=True)['ok'] is True
    assert studio._teleop.rejected and studio._teleop.rejected[0][0] == 'aa'
    assert studio._teleop.accepted == []


def test_stopping_an_active_pairing(studio):
    studio._teleop = _Teleop(_Pairing('cc', leader=2, follower=1, state='active'))
    assert run(studio, 'teleop', op='stop', sid='cc', confirm=True)['ok'] is True
    assert studio._teleop.stopped and studio._teleop.stopped[0][0] == 'cc'


def test_accept_is_refused_just_after_an_estop(studio):
    studio._teleop = _Teleop(_Pairing('aa', leader=2, follower=1))
    run(studio, 'estop')
    reply = run(studio, 'teleop', op='accept', sid='aa', confirm=True)
    assert reply['ok'] is False and reply['error'] == 'estop_holdoff'
    assert studio._teleop.accepted == []


# ── The selected robot gets the mode, options and config (not the first one) ─

class _RoutedNode:
    """Records where the node's 'unspecified robot' publishers go."""
    station_robot_id = 'robot1'
    station_id = 1

    def __init__(self):
        self.sent = []
        self.pubs = {'robot2': {'active_profile': 'pub2'}}
        self.active_profile_publisher_ = 'pub1'

    def _resolve_rid(self, robot_id=None):
        return self.station_robot_id if robot_id is None else str(robot_id)

    def publish_mode(self, mode, robot_id=None):
        self.sent.append(('mode', mode, self._resolve_rid(robot_id), self.active_profile_publisher_))


def _routed(active):
    s = _Studio()
    s.node = _RoutedNode()
    s._rc_active_rid = active

    def mode_selected(idx, name):
        s.node.publish_mode(idx + 1)
        s._rc_selected_mode_idx = idx
        s._rc_selected_kind = CONTROL_MODE_TABS[idx][1]
    s._on_rc_mode_selected = mode_selected
    s._publish_admittance_planner = lambda enabled, robot_id=None: s.node.sent.append(
        ('planner', enabled, robot_id or s.node.station_robot_id))
    return s


def test_mode_goes_only_to_the_selected_robot():
    s = _routed('robot2')
    assert run(s, 'set_mode', index=1)['ok']
    assert s.node.sent == [('mode', 2, 'robot2', 'pub2')]
    assert s.node.active_profile_publisher_ == 'pub1'           # restored afterwards
    assert '_resolve_rid' not in s.node.__dict__
    assert s._web_modes == {'robot2': 1}


def test_mode_still_goes_to_the_studios_own_robot_when_it_is_selected():
    s = _routed('robot1')
    assert run(s, 'set_mode', index=2)['ok']
    assert s.node.sent == [('mode', 3, 'robot1', 'pub1')]


def test_scope_is_lifted_even_when_the_handler_raises():
    from fic_studio.web_remote.common import scoped_to_selected
    s = _routed('robot2')
    with pytest.raises(RuntimeError):
        with scoped_to_selected(s):
            raise RuntimeError('boom')
    assert s.node._resolve_rid(None) == 'robot1'
    assert s.node.active_profile_publisher_ == 'pub1'
    assert '_publish_admittance_planner' in s.__dict__
