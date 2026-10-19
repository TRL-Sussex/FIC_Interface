"""Whole-arm safety zones in the Studio: what a zone may be, how it goes to the controllers, the Safety page's
edits (checked, applied, saved with the robot), the controller's clearance read back for the monitor and
Stop on violation, and the warnings in Notices when a robot touches or crosses a limit. No ROS, no window."""
import math
import os
import types

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest  # noqa: E402

from fic_studio import notices, safety_zones  # noqa: E402
from fic_studio.mixins.safety_mixin import STOP_PAST_M, SafetyMixin  # noqa: E402
from fic_studio.web_remote import safety as web_safety  # noqa: E402
from fic_studio.web_remote.common import Refused  # noqa: E402

BOX = {'name': 'Bench', 'shape': 'box', 'keep': 'out', 'enabled': True, 'centre': [0.5, 0.0, 0.2],
       'size': [0.4, 0.6, 0.4], 'yaw_deg': 30}
SPHERE = {'name': 'Head', 'shape': 'sphere', 'keep': 'out', 'centre': [0.0, 0.6, 1.2], 'radius': 0.25}
CYL = {'name': 'Column', 'shape': 'cylinder', 'keep': 'out', 'centre': [-0.4, 0.3], 'radius': 0.1, 'z_min': 0, 'z_max': 2}
CELL = {'name': 'Cell', 'shape': 'prism', 'keep': 'in', 'points': [[-1, -1], [1, -1], [1, 1], [-1, 1]], 'z_min': 0, 'z_max': 1.5}


# ── what a zone may be ──

def test_every_shape_is_accepted_and_defaults_filled():
    out = safety_zones.check_zones([BOX, SPHERE, CYL, CELL])
    assert [z['shape'] for z in out] == ['box', 'sphere', 'cylinder', 'prism']
    assert out[1]['enabled'] is True and out[1]['keep'] == 'out'
    assert out[3]['keep'] == 'in'


@pytest.mark.parametrize('bad, why', [
    (dict(BOX, shape='cone'), 'shape'),
    (dict(BOX, keep='near'), 'keep'),
    (dict(BOX, size=[0.4, 0.0, 0.4]), 'size'),
    (dict(SPHERE, radius=float('nan')), 'radius'),
    (dict(CYL, z_max=0.005), 'top'),
    (dict(CELL, points=[[0, 0], [1, 1], [1, 0], [0, 1]]), 'crosses'),       # a bow tie
    (dict(CELL, points=[[0, 0], [1, 0], [2, 0]]), 'no area'),
    (dict(CELL, points=[[0, 0], [1, 0]]), 'corners'),
    (dict(BOX, centre=[9.0, 0, 0]), 'centre'),
])
def test_a_malformed_zone_is_refused_saying_what(bad, why):
    with pytest.raises(ValueError, match=why):
        safety_zones.check_zones([bad])


def test_at_most_sixteen_zones():
    with pytest.raises(ValueError, match='16'):
        safety_zones.check_zones([SPHERE] * 17)


def test_the_arm_checked_from_a_real_joint():
    assert safety_zones.check_body({'from_joint': 3, 'radius': 0.06}, 7) == {'radius': 0.06, 'tool_radius': 0.03, 'from_joint': 3}
    with pytest.raises(ValueError):
        safety_zones.check_body({'from_joint': 8}, 7)
    with pytest.raises(ValueError):
        safety_zones.check_body({'radius': 0.5}, 7)


# ── to the controllers: the format fic/safety_zones.h reads ──

def test_encoded_as_the_controller_reads_it():
    m = safety_zones.encode(safety_zones.check_zones([BOX, CELL]), {'radius': 0.05, 'tool_radius': 0.03, 'from_joint': 2})
    assert m[:5] == [1.0, 0.05, 0.03, 1.0, 2.0]                       # from_joint 0-based for the controller
    assert m[5:9] == [0.0, 0.0, 1.0, 7.0]                               # box, keep out, on, 7 values
    assert m[9:15] == [0.5, 0.0, 0.2, 0.2, 0.3, 0.2]                    # centre, half sizes
    assert m[15] == pytest.approx(math.radians(30))
    assert m[16:20] == [3.0, 1.0, 1.0, 10.0]                            # prism, keep in, on, 2 + 8 values
    assert m[20:] == [0, 1.5, -1, -1, 1, -1, 1, 1, -1, 1]


def test_no_zones_is_a_message_that_switches_them_off():
    assert safety_zones.encode([], None) == [1.0, 0.05, 0.03, 1.0, 0.0]


# ── the controller's clearance, read back ──

def test_zone_state_from_the_controller():
    zones = safety_zones.check_zones([BOX])
    now = 100.0
    assert safety_zones.zone_state([-0.012, 0, 2, 1, 0.3, 0, 0.4], zones, now, now)['state'] == 'violation'
    st = safety_zones.zone_state([0.02, 0, 2, 0, 0.3, 0, 0.4], zones, now, now)
    assert st['state'] == 'warn' and st['name'] == 'Bench' and st['point'] == [0.3, 0, 0.4]
    assert safety_zones.zone_state([0.2, 0, 2, 0, 0, 0, 0], zones, now, now)['state'] == 'ok'
    assert safety_zones.zone_state([-0.012, 0, 2, 1, 0, 0, 0], zones, now - 2, now) is None          # stale
    assert safety_zones.zone_state([-0.012, 0, 2, 1, 0, 0, 0], [dict(zones[0], enabled=False)], now, now) is None


# ── the monitor: Stop on violation and the warnings ──

class _Pos:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Pose:
    def __init__(self, x, y, z):
        self.position = _Pos(x, y, z)


class _Monitor:
    _safety_robots = SafetyMixin._safety_robots
    _safety_past = SafetyMixin._safety_past
    _enforce_safety_limits = SafetyMixin._enforce_safety_limits
    _enforce_safety_stop = SafetyMixin._enforce_safety_stop
    _warn_safety_contacts = SafetyMixin._warn_safety_contacts
    _on_zone_state = SafetyMixin._on_zone_state
    zone_state_for = SafetyMixin.zone_state_for

    def __init__(self):
        self.safety_enforce = True
        self.safety_limits = {
            'cartesian': {'x': {'min': -1.0, 'max': 0.6, 'enabled': True},
                          'y': {'min': -1.0, 'max': 1.0, 'enabled': True},
                          'z': {'min': 0.1, 'max': 1.5, 'enabled': True}},
            'zones': safety_zones.check_zones([BOX])}
        self.current_ee_pose_by_robot = {'robot1': _Pose(0.0, 0, 0.5)}
        self.current_joint_positions_by_robot = {'robot1': [0.0] * 7}
        self.node = types.SimpleNamespace(real_joint_publishers={})
        self.stopped, self.logs = [], []
        self._web_ui_window = object()            # notices go to the web UI, no Qt boxes

    def stop_for(self, rid):
        self.stopped.append(rid)

    def log_message(self, *a):
        self.logs.append(a)

    def clearance(self, c):
        self._on_zone_state('robot1', types.SimpleNamespace(data=[c, 0, 3, 1 if c < 0 else 0, 0.4, 0, 0.3]))


def test_a_link_clearly_inside_a_zone_stops_that_robot():
    m = _Monitor()
    m.clearance(-0.01)                             # pressed 1 cm in: the controller's wall holds it
    m._enforce_safety_limits()
    assert m.stopped == []
    m.clearance(-(STOP_PAST_M + 0.01))
    m._enforce_safety_limits()
    assert m.stopped == ['robot1']
    assert "safety zone 1 'Bench'" in m.logs[-1][1]


def _passing(m):
    return [n for n in notices.history(m) if n.get('passing')]


def test_touching_then_crossing_a_zone_warns_once_each_in_notices():
    m = _Monitor()
    m.clearance(0.1)
    m._warn_safety_contacts()
    assert _passing(m) == []
    m.clearance(0.003)                             # touching
    m._warn_safety_contacts()
    m._warn_safety_contacts()
    assert [n['title'] for n in _passing(m)] == ["robot1 is touching safety zone 1 'Bench'"]
    m.clearance(-0.008)                            # past it
    m._warn_safety_contacts()
    m._warn_safety_contacts()
    assert len(_passing(m)) == 2 and 'is past' in _passing(m)[-1]['title'] and '8 mm' in _passing(m)[-1]['text']
    assert all(n['level'] == 'warn' for n in _passing(m))
    m.clearance(0.01)                              # just clear: not re-armed yet
    m._warn_safety_contacts()
    m.clearance(0.003)
    m._warn_safety_contacts()
    assert len(_passing(m)) == 2
    m.clearance(0.05)                              # well clear, then touching again: warns again
    m._warn_safety_contacts()
    m.clearance(0.003)
    m._warn_safety_contacts()
    assert len(_passing(m)) == 3


def test_touching_the_workspace_box_warns_too():
    m = _Monitor()
    m.safety_limits['zones'] = []
    m.current_ee_pose_by_robot['robot1'] = _Pose(0.597, 0, 0.5)     # 3 mm from X max
    m._warn_safety_contacts()
    assert [n['title'] for n in _passing(m)] == ['robot1 is touching the workspace X max (0.600 m)']


def test_estop_and_its_reset_go_into_notices(tmp_path, monkeypatch):
    from fic_studio import audit
    monkeypatch.setenv('FIC_STUDIO_AUDIT', str(tmp_path / 'audit.log'))
    audit.reset_for_tests()
    m = _Monitor()
    SafetyMixin._estop_latch_set(m, 'Phone @ 10.0.0.9')
    SafetyMixin._estop_latch_reset(m, 'Desk')
    got = [(n['level'], n['title']) for n in notices.history(m)]
    assert got == [('error', 'E-STOP pressed by Phone @ 10.0.0.9'), ('info', 'E-STOP reset by Desk')]
    audit.reset_for_tests()


# ── the Safety page's edits ──

class _Studio:
    zone_state_for = SafetyMixin.zone_state_for

    def __init__(self):
        self.robot_dropdown = types.SimpleNamespace(currentText=lambda: 'KinovaGen3_7DOF')
        self.robot_configs = {'KinovaGen3_7DOF': {'num_joints': 7, 'safety_zones': {
            'cartesian': {'x': {'min': -1.0, 'max': 1.0, 'enabled': True}}, 'velocity': {'max_joint_vel': 1.0, 'max_cart_vel': 0.5, 'enabled': True,
                                                                                         'sim_max_joint_vel': 2.0}}}}
        self.safety_limits = {}
        self.published = 0
        self.persisted = 0
        self.logs = []
        self.node = types.SimpleNamespace(station_robot_id='robot1')

    def _publish_safety_limits(self):
        self.published += 1

    def _rc_persist(self):
        self.persisted += 1
        return True

    def log_message(self, *a):
        self.logs.append(a)

    def ee_pose_for(self, rid):
        return None


def test_zones_applied_published_and_saved_with_the_robot():
    w = _Studio()
    r = web_safety.set_limits(w, {'confirm': True, 'zones': [BOX, CELL], 'body': {'from_joint': 3}}, None)
    assert r['ok'] and w.published == 1 and w.persisted == 1
    assert [z['name'] for z in w.safety_limits['zones']] == ['Bench', 'Cell']
    saved = w.robot_configs['KinovaGen3_7DOF']['safety_zones']
    assert [z['name'] for z in saved['zones']] == ['Bench', 'Cell'] and saved['body']['from_joint'] == 3
    assert saved['velocity']['sim_max_joint_vel'] == 2.0            # what the page does not show is kept
    assert '2 safety zones' in r['changed']
    st = r['safety']
    assert st['zones'][1]['keep'] == 'in' and st['body']['from_joint'] == 3


def test_zones_removed_is_saved_as_none():
    w = _Studio()
    web_safety.set_limits(w, {'confirm': True, 'zones': [BOX]}, None)
    web_safety.set_limits(w, {'confirm': True, 'zones': []}, None)
    assert w.robot_configs['KinovaGen3_7DOF']['safety_zones']['zones'] == []


def test_a_bad_zone_changes_nothing():
    w = _Studio()
    with pytest.raises(Refused, match='crosses itself'):
        web_safety.set_limits(w, {'confirm': True, 'zones': [dict(CELL, points=[[0, 0], [1, 1], [1, 0], [0, 1]])]}, None)
    assert w.published == 0 and w.persisted == 0 and w.safety_limits == {}


def test_zones_need_confirmation():
    w = _Studio()
    with pytest.raises(Refused):
        web_safety.set_limits(w, {'zones': [BOX]}, None)


def test_the_desktop_panel_keeps_the_zones():
    class _Panel:
        def to_config(self):
            return {'cartesian': {}, 'joint_safety_deg': {}, 'velocity': {}}

    class _W:
        _apply_safety_panel = SafetyMixin._apply_safety_panel

        def __init__(self):
            self._safety_panel = _Panel()
            self.safety_limits = {'zones': [BOX], 'body': {'from_joint': 2}}
            self.published = 0

        def _publish_safety_limits(self):
            self.published += 1

    w = _W()
    w._apply_safety_panel()
    assert w.safety_limits['zones'] == [BOX] and w.published == 1


def test_passing_notices_go_by_themselves_and_show_no_box():
    w = types.SimpleNamespace(log_message=lambda *a: None)       # the classic window: no _web_ui_window
    e = notices.alert(w, 'warn', 'Touching', '', 'Safety', passing=True)
    assert e['passing'] is True
    assert notices.state(w)['recent'][-1]['passing'] is True
