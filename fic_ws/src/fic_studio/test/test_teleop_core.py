"""Built-in teleoperation: the follower's tracker and the session protocol."""
import json
import math

from fic_studio.teleop import session as ts
from fic_studio.teleop.tracker import JointFollower
import pytest


# ── tracker ─────────────────────────────────────────────────────────────
class TestTracker:
    def test_starts_where_the_follower_is(self):
        f = JointFollower(2, alpha=1.0, max_speed=100.0)
        f.start([1.0, 1.0], [0.2, -0.3])
        assert f.step([1.0, 1.0], 0.01) is None       # leader did not move: nothing sent
        assert f.last_target == [0.2, -0.3]

    def test_copies_relative_motion_with_scale(self):
        f = JointFollower(2, alpha=1.0, max_speed=100.0, scale=0.5)
        f.start([0.0, 0.0], [1.0, 1.0])
        assert f.step([0.2, -0.4], 0.01) == pytest.approx([1.1, 0.8])

    def test_speed_cap(self):
        f = JointFollower(1, alpha=1.0, max_speed=0.5)
        f.start([0.0], [0.0])
        out = f.step([1.0], 0.01)                      # wants +1 rad at once
        assert out == pytest.approx([0.005])           # 0.5 rad/s * 10 ms

    def test_joint_limits(self):
        f = JointFollower(1, lower=[-0.1], upper=[0.1], alpha=1.0, max_speed=100.0)
        f.start([0.0], [0.0])
        assert f.step([2.0], 0.01) == pytest.approx([0.1])

    def test_continuous_joint_crossing_pi_is_not_a_jump(self):
        f = JointFollower(1, alpha=1.0, max_speed=100.0)
        f.start([math.pi - 0.01], [0.0])
        out = f.step([-math.pi + 0.01], 0.01)          # moved +0.02 rad across the seam
        assert out == pytest.approx([0.02])

    def test_smoothing(self):
        f = JointFollower(1, alpha=0.5, max_speed=100.0, deadband=0.0)
        f.start([0.0], [0.0])
        assert f.step([1.0], 0.01) == pytest.approx([0.5])
        assert f.step([1.0], 0.01) == pytest.approx([0.75])

    def test_rejects_bad_input(self):
        f = JointFollower(2)
        with pytest.raises(ValueError):
            f.start([0.0], [0.0, 0.0])
        f.start([0.0, 0.0], [0.0, 0.0])
        with pytest.raises(ValueError):
            f.step([float('nan'), 0.0], 0.01)
        with pytest.raises(RuntimeError):
            JointFollower(1).step([0.0], 0.01)

    def test_settles_within_the_deadband_after_the_leader_stops(self):
        f = JointFollower(1, alpha=0.4, max_speed=100.0, deadband=0.002)
        f.start([0.0], [0.0])
        for _ in range(200):
            f.step([0.1], 0.01)
        assert abs(f.last_target[0] - 0.1) <= 0.002
        assert f.step([0.1], 0.01) is None        # and then goes quiet

    def test_alpha_is_unchanged_at_the_tick_it_was_quoted_for(self):
        # The whole point: existing behaviour must not move. alpha is quoted
        # against a 10 ms tick, so at 10 ms the derived fraction is that alpha.
        f = JointFollower(1, alpha=0.5, max_speed=100.0, deadband=0.0)
        f.start([0.0], [0.0])
        assert f.step([1.0], 0.01) == pytest.approx([0.5])
        assert f.step([1.0], 0.01) == pytest.approx([0.75])

    def test_smoothing_does_not_depend_on_how_often_it_is_called(self):
        # Same leader, same wall-clock, two stream rates: the follower must end
        # up in the same place. Before tau this failed badly -- 10x the rate
        # meant 10x less filtering.
        def run(dt, n):
            f = JointFollower(1, alpha=0.4, max_speed=100.0, deadband=0.0)
            f.start([0.0], [0.0])
            out = None
            for _ in range(n):
                out = f.step([1.0], dt) or out
            return out[0]
        slow, fast = run(0.010, 20), run(0.001, 200)      # both 200 ms
        assert slow == pytest.approx(fast, abs=1e-3)

    def test_no_filter_stays_no_filter_at_every_rate(self):
        for dt in (0.01, 0.001):
            # max_speed high so the slew cap cannot bind: this is about the filter.
            f = JointFollower(1, alpha=1.0, max_speed=1e6, deadband=0.0)
            f.start([0.0], [0.0])
            assert f.step([1.0], dt) == pytest.approx([1.0])

    def test_tau_can_be_given_directly(self):
        # The knob for trading lag against noise without touching the rate.
        f = JointFollower(1, tau=0.005, max_speed=1e6, deadband=0.0)
        f.start([0.0], [0.0])
        assert f.step([1.0], 0.005) == pytest.approx([1 - math.exp(-1.0)], abs=1e-9)

    def test_a_stalled_tick_does_not_grant_a_huge_slew(self):
        # dt comes from a GUI timer's wall clock. A 1 s stall used to licence a
        # 1 s worth of travel on the very next tick.
        # 0.5 rad, not 10: a 10 rad "jump" wraps to -2.566 and would test
        # the wrapping rather than the clamp.
        f = JointFollower(1, alpha=1.0, max_speed=1.0, deadband=0.0)
        f.start([0.0], [0.0])
        assert f.step([0.5], 1.0) == pytest.approx([0.050])    # dt clamped to 50 ms

    def test_error(self):
        f = JointFollower(2, alpha=1.0, max_speed=100.0)
        f.start([0.0, 0.0], [0.0, 0.0])
        f.step([0.1, 0.0], 0.01)
        assert f.error([0.0, 0.0]) == pytest.approx(0.1)


# ── sessions: two or three Studios wired through a fake bus ─────────────
class _Bus:
    """Delivers session messages to every Studio (including the sender, like DDS)."""

    def __init__(self):
        self.studios = []
        self.data = {}

    def deliver(self, text):
        for s in list(self.studios):
            s.on_json(text)


class _Pub:
    def __init__(self, bus, topic, owner):
        self.bus, self.topic, self.owner = bus, topic, owner

    def publish(self, msg):
        if self.topic == ts.SESSION_TOPIC:
            self.bus.deliver(msg.data)
        elif self.topic == ts.FOLLOWER_WRENCH_TOPIC:
            for s in self.bus.studios:
                s.on_follower_wrench(msg)
        else:                       # the shared leader stream reaches every Studio
            for s in self.bus.studios:
                s.on_leader_data(msg)


class _Node:
    def __init__(self, bus):
        self.bus = bus

    def create_publisher(self, _type, topic, _qos):
        return _Pub(self.bus, topic, self)

    def create_subscription(self, _type, topic, _cb, _qos):
        return topic

    def destroy_publisher(self, _pub):
        pass

    def destroy_subscription(self, _sub):
        pass


class _Hooks:
    def __init__(self, target='simulation', robot='KinovaGen3_7DOF', joints=7):
        self._target, self._robot, self._joints = target, robot, joints
        self.q = [0.0] * joints
        self.sent, self.began, self.ended, self.asked = [], [], [], []
        self.modes = []
        #: what this arm is touching, and what has been put onto it
        self.w = None
        self.reflected = []

    def robot_type(self):
        return self._robot

    def joint_count(self):
        return self._joints

    def joint_positions(self):
        return list(self.q)

    def joint_limits(self):
        return None, None

    def target(self):
        return self._target

    def set_mode(self, mode):
        self.modes.append(mode)

    def begin_following(self, s):
        self.began.append(s.sid)

    def publish_target(self, q):
        self.sent.append(q)

    def wrench(self):
        return list(self.w) if self.w else None

    def publish_wrench(self, w6):
        self.reflected.append(list(w6))

    def end_following(self, s, reason):
        self.ended.append(reason)

    def confirm_request(self, s):
        self.asked.append(s.sid)

    def on_change(self):
        pass

    def log(self, level, text):
        pass


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def _lab(**follower_kw):
    bus, clock = _Bus(), _Clock()
    hooks = {0: _Hooks(), 1: _Hooks(), 2: _Hooks(**follower_kw)}
    managers = {}
    for station, host in ((0, 'control'), (1, 'robot1'), (2, 'robot2')):
        m = ts.TeleopManager(_Node(bus), station, host, hooks[station], clock=clock)
        managers[station] = m
        bus.studios.append(m)
    return managers, hooks, clock


def _run(managers, clock, seconds, step=0.01, before_tick=None):
    for _ in range(int(round(seconds / step))):
        clock.t += step
        if before_tick:
            before_tick()
        for m in managers.values():
            m.tick()


class TestSessions:
    def test_master_pairs_two_robots_and_the_follower_copies(self):
        m, h, clock = _lab()
        s = m[0].request(1, 2, max_speed=10.0, leader_robot='KinovaGen3_7DOF', leader_joints=7)
        assert m[2].following().sid == s.sid
        assert [x.sid for x in m[1].leading()] == [s.sid]
        assert h[2].began == [s.sid]
        _run(m, clock, 0.05)                     # tracker anchors on the first data
        h[1].q = [0.1] + [0.0] * 6               # the leader arm moves
        _run(m, clock, 0.5)
        assert h[2].sent, 'follower never sent a target'
        assert h[2].sent[-1][0] == pytest.approx(0.1, abs=0.002)   # within the deadband
        assert m[0].sessions[s.sid].state == ts.ACTIVE

    def test_follower_on_hardware_asks_its_operator(self):
        m, h, _ = _lab(target='hardware')
        s = m[1].request(1, 2)
        assert h[2].asked == [s.sid] and m[2].sessions[s.sid].state == ts.PENDING
        assert not h[2].began
        assert m[2].accept(s.sid)
        assert m[1].leading()

    def test_follower_on_vr_accepts_like_simulation(self):
        # VR drives the headset's simulated arm: no real arm moves, so nobody is asked.
        m, h, _ = _lab(target='vr')
        s = m[1].request(1, 2)
        assert not h[2].asked and m[2].following().sid == s.sid

    def test_refused_when_robots_differ(self):
        m, h, _ = _lab(robot='Franka_FR3')
        s = m[0].request(1, 2, leader_robot='KinovaGen3_7DOF', leader_joints=7)
        assert m[0].sessions[s.sid].state == ts.ENDED
        assert 'different robots' in m[0].sessions[s.sid].reason
        assert not h[2].began

    def test_the_follower_holds_where_it_is_before_the_leader_moves(self):
        """Or the arm sags under gravity while the leader stands still."""
        m, h, clock = _lab()
        h[2].q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        m[0].request(1, 2)
        _run(m, clock, 0.2)                       # the leader is not moving
        assert h[2].sent, 'nothing was published: the arm is left to sag'
        assert h[2].sent[0] == pytest.approx(h[2].q)

    def test_an_arm_that_is_still_starting_is_waited_for(self):
        """Start the robot, then ask it to follow: one move, in practice."""
        m, h, clock = _lab()
        h[2].q = []                                   # the simulator is still coming up
        s = m[0].request(1, 2)
        _run(m, clock, 2.0)
        assert m[2].sessions[s.sid].state == ts.PENDING
        assert not h[2].began
        h[2].q = [0.0] * 7                            # it says where it is
        _run(m, clock, 0.5)
        assert m[1].leading() and h[2].began == [s.sid]

    def test_an_arm_that_never_starts_is_refused_in_the_end(self):
        m, h, clock = _lab()
        h[2].q = []
        s = m[0].request(1, 2)
        _run(m, clock, ts.ARM_WAIT_S + 1.0)
        assert m[0].sessions[s.sid].state == ts.ENDED
        assert 'no joint data' in m[0].sessions[s.sid].reason
        assert not h[2].began

    def test_a_hardware_follower_is_asked_only_once_its_arm_is_up(self):
        m, h, clock = _lab(target='hardware')
        h[2].q = []
        s = m[0].request(1, 2)
        _run(m, clock, 1.0)
        assert h[2].asked == []                       # no dialog about a dead arm
        h[2].q = [0.0] * 7
        _run(m, clock, 0.5)
        assert h[2].asked == [s.sid]

    def test_one_leader_at_a_time_per_follower(self):
        m, _, _ = _lab()
        m[0].request(1, 2)
        second = m[0].request(0, 2)
        assert 'already following' in m[0].sessions[second.sid].reason

    def test_stop_from_anywhere_releases_the_follower(self):
        m, h, clock = _lab()
        s = m[0].request(1, 2)
        _run(m, clock, 0.1)
        m[0].stop(s.sid, 'done')
        assert m[2].following() is None and m[1].leading() == []
        assert h[2].ended == ['done']

    def test_follower_holds_then_ends_when_leader_goes_quiet(self):
        m, h, clock = _lab()
        s = m[0].request(1, 2)
        _run(m, clock, 0.1)
        leader = m.pop(1)                        # leader's Studio vanishes from the network
        leader._node.bus.studios.remove(leader)
        _run(m, clock, ts.HOLD_S + 0.1)
        assert m[2].following().holding
        _run(m, clock, ts.LOST_S)
        assert m[2].following() is None
        assert 'lost leader' in h[2].ended[0]
        assert leader.sessions[s.sid].state == ts.ACTIVE     # it never heard

    def test_leader_ends_when_follower_goes_quiet(self):
        m, _, clock = _lab()
        s = m[0].request(1, 2)
        m.pop(2)
        _run(m, clock, ts.LOST_S + 1.5)
        assert m[1].sessions[s.sid].state == ts.ENDED

    def test_stop_all_on_estop_names_the_partners(self):
        m, h, _ = _lab()
        m[0].request(1, 2)
        assert m[2].partner_stations() == [1]
        m[2].stop_all('E-stop on Robot 2')
        assert h[2].ended == ['E-stop on Robot 2'] and m[1].leading() == []

    def test_unanswered_request_fails(self):
        m, h, clock = _lab(target='hardware')
        s = m[0].request(1, 2)
        _run(m, clock, ts.ANSWER_S + 0.5, step=0.5)
        assert m[0].sessions[s.sid].state == ts.ENDED

    def test_malformed_messages_are_ignored(self):
        m, _, _ = _lab()
        for text in ('nope', json.dumps({'type': 'request'}),
                     json.dumps({'type': 'request', 'sid': 'x', 'leader': 'a', 'follower': 3})):
            m[2].on_json(text)
        assert m[2].following() is None

    def test_follower_ignores_other_leaders_streams(self):
        m, h, clock = _lab()
        m[0].request(1, 2)
        h[0].q = [0.5] * 7              # station 0 is not a leader: it must not be copied
        _run(m, clock, 0.1)
        m[2].on_leader_q(0, [0.9] * 7)
        assert m[2]._leader_q != [0.9] * 7

    def test_cannot_lead_itself(self):
        m, _, _ = _lab()
        with pytest.raises(ValueError):
            m[0].request(3, 3)


class TestForceChannel:
    """The follower's contact reaching the leader's arm, and letting go of it."""

    def test_off_by_default(self):
        m, h, clock = _lab()
        m[0].request(1, 2)
        h[2].w = [5.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        _run(m, clock, 0.3)
        assert h[1].reflected == [], 'force reached the leader without being asked for'

    def test_follower_contact_reaches_the_leader_scaled(self):
        m, h, clock = _lab()
        m[0].request(1, 2)
        m[1].force_scale = 0.5
        h[2].w = [4.0, 0.0, -2.0, 0.0, 0.0, 0.0]
        _run(m, clock, 0.3)
        assert h[1].reflected, 'the leader never felt anything'
        assert h[1].reflected[-1] == pytest.approx([2.0, 0.0, -1.0, 0.0, 0.0, 0.0])

    def test_only_the_leader_of_that_follower_feels_it(self):
        m, h, clock = _lab()
        m[0].request(1, 2)
        m[0].force_scale = 1.0          # station 0 leads nobody
        m[1].force_scale = 1.0
        h[2].w = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        _run(m, clock, 0.3)
        assert h[0].reflected == []
        assert h[1].reflected

    def test_released_once_when_the_follower_goes_quiet(self):
        m, h, clock = _lab()
        m[0].request(1, 2)
        m[1].force_scale = 1.0
        h[2].w = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        _run(m, clock, 0.3)
        assert h[1].reflected[-1][0] == pytest.approx(3.0)
        h[2].w = None                   # the follower stops reporting
        _run(m, clock, 1.0)
        assert h[1].reflected[-1] == [0.0] * 6, 'the leader was left holding a force'
        # and the release is sent once, not every tick
        assert h[1].reflected.count([0.0] * 6) == 1

    def test_released_when_the_session_ends(self):
        m, h, clock = _lab()
        s = m[0].request(1, 2)
        m[1].force_scale = 1.0
        h[2].w = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        _run(m, clock, 0.3)
        assert h[1].reflected[-1][0] == pytest.approx(3.0)
        m[0].stop(s.sid, 'done')
        _run(m, clock, 0.1)
        assert h[1].reflected[-1] == [0.0] * 6

    def test_a_follower_reports_nothing_when_not_in_a_session(self):
        m, h, clock = _lab()
        h[2].w = [9.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        m[1].force_scale = 1.0
        _run(m, clock, 0.3)
        assert h[1].reflected == []


class TestRoundTrip:
    """RTT, measured on one clock so the two machines' clocks need not agree."""

    def test_leader_measures_the_round_trip(self):
        m, _, clock = _lab()
        s = m[0].request(1, 2)
        _run(m, clock, 2.5)                     # PING_S is 1 s
        assert m[1].sessions[s.sid].rtt is not None
        # The fake bus is instant, so the trip is one tick's worth at most.
        assert 0.0 <= m[1].sessions[s.sid].rtt <= 0.05

    def test_the_follower_does_not_measure_its_leader(self):
        m, _, clock = _lab()
        s = m[0].request(1, 2)
        _run(m, clock, 2.5)
        assert m[2].sessions[s.sid].rtt is None, 'only the leader pings'

    def test_rtt_is_reported(self):
        m, _, clock = _lab()
        s = m[0].request(1, 2)
        _run(m, clock, 2.5)
        assert 'rtt' in m[1].sessions[s.sid].as_dict()


class TestRemoteCommands:
    """Running a session from the control PC, without opening a Studio.

    The robot PCs' desktops are reached over NoMachine, and connecting that way
    is what kept powering robot1 off on 2026-09-30, so every one of these has to
    work with nobody at the robot's own screen.
    """

    def _cmd(self, m, station, **kw):
        """Send a command the way the control PC does: station 0, host 'control'."""
        m[0]._send(dict({'type': 'command', 'sid': 'new', 'to': station}, **kw))

    def test_control_pc_starts_a_session(self):
        m, h, clock = _lab()
        self._cmd(m, 1, action='request', follower=2, max_speed=10.0)
        assert m[2].following(), 'the follower never took it up'
        assert [s.follower for s in m[1].leading()] == [2]
        _run(m, clock, 0.05)
        h[1].q = [0.1] + [0.0] * 6
        _run(m, clock, 0.5)
        assert h[2].sent[-1][0] == pytest.approx(0.1, abs=0.002)

    def test_control_pc_accepts_for_a_hardware_follower(self):
        m, h, _ = _lab(target='hardware')
        s = m[1].request(1, 2)
        assert h[2].asked == [s.sid] and not h[2].began   # waiting on its operator
        self._cmd(m, 2, action='accept')                  # answered from the control PC
        assert h[2].began == [s.sid]
        assert m[2].sessions[s.sid].state == ts.ACTIVE
        assert m[1].leading(), 'the leader never learned it was accepted'

    def test_accept_finds_the_pending_session_without_being_told_its_sid(self):
        m, h, _ = _lab(target='hardware')
        s = m[1].request(1, 2)
        self._cmd(m, 2, sid='not-a-real-sid', action='accept')
        assert h[2].began == [s.sid]

    def test_only_the_named_station_acts(self):
        m, h, _ = _lab(target='hardware')
        m[1].request(1, 2)
        self._cmd(m, 1, action='accept')       # addressed to the LEADER
        assert not h[2].began, 'the follower acted on a command meant for another station'

    def test_a_remote_request_still_refuses_a_different_robot(self):
        # leader_robot is filled from the leader's own hooks, not the message,
        # so omitting it must not quietly disable the check.
        m, h, _ = _lab(robot='Franka_FR3')
        self._cmd(m, 1, action='request', follower=2)
        assert not m[2].following()
        assert not h[2].began

    def test_accepting_nothing_is_harmless(self):
        m, h, _ = _lab(target='hardware')
        self._cmd(m, 2, action='accept')
        assert not h[2].began

    def test_control_pc_stops_a_session(self):
        m, h, _ = _lab()
        s = m[0].request(1, 2)
        assert m[2].following()
        self._cmd(m, 2, action='stop', reason='stopped from the dashboard')
        assert not m[2].following()
        assert h[2].ended and 'dashboard' in h[2].ended[-1]
        assert m[1].sessions[s.sid].state == ts.ENDED

    def test_an_inconsistent_leader_is_refused(self):
        m, h, _ = _lab()
        self._cmd(m, 1, action='request', leader=3, follower=3)
        assert not m[2].following(), 'acted on a command whose leader was not itself'

    def test_a_request_from_the_master_needs_no_operator(self):
        # The whole point of driving from one place: on hardware a pairing the
        # master asked for runs without anyone answering at the follower.
        m, h, _ = _lab(target='hardware')
        self._cmd(m, 1, action='request', follower=2)
        assert h[2].asked == [], 'the follower still asked its operator'
        assert h[2].began, 'the follower never started following'
        assert m[1].leading()

    def test_a_robot_studio_still_asks_on_hardware(self):
        # Only the master is taken on trust. A robot Studio starting a pairing
        # by itself must still be agreed to at the far end.
        m, h, _ = _lab(target='hardware')
        s = m[1].request(1, 2, leader_robot='KinovaGen3_7DOF', leader_joints=7)
        assert h[2].asked == [s.sid]
        assert not h[2].began

    def test_an_unknown_origin_is_not_trusted(self):
        m, h, _ = _lab(target='hardware')
        m[1].request(1, 2, origin='some-laptop')
        assert not h[2].began, 'accepted a request claiming an origin we do not know'

    def test_control_pc_sets_a_control_mode(self):
        m, h, _ = _lab(target='hardware')
        self._cmd(m, 2, action='mode', mode=3)
        assert h[2].modes == [3]
        assert h[1].modes == [], 'another station took a mode meant for station 3'

    def test_a_mode_outside_1_to_8_is_ignored(self):
        m, h, _ = _lab()
        for bad in (0, 9, 'three', None, True):
            self._cmd(m, 2, action='mode', mode=bad)
        assert h[2].modes == []


def _lab_of(stations_hosts):
    bus, clock = _Bus(), _Clock()
    managers, hooks = {}, {}
    for station, host in stations_hosts:
        hooks[station] = _Hooks()
        managers[station] = ts.TeleopManager(_Node(bus), station, host, hooks[station], clock=clock)
        bus.studios.append(managers[station])
    return managers, hooks, clock


class TestNeighbours:
    """Robot stations stand in a line: a robot pairs with its neighbours (N-1, N+1). A Master, the lab's
    control seat, may pair any two."""

    def test_a_robot_does_not_follow_a_robot_that_is_not_its_neighbour(self):
        m, h, _ = _lab_of(((0, 'control'), (2, 'robot2'), (4, 'robot4')))
        s = m[4].request(4, 2)
        assert m[4].sessions[s.sid].state == ts.ENDED
        assert 'not a neighbour' in m[4].sessions[s.sid].reason
        assert not h[2].began

    def test_neighbours_pair(self):
        m, h, _ = _lab_of(((1, 'robot1'), (2, 'robot2')))
        s = m[1].request(1, 2)
        assert m[2].following().sid == s.sid

    def test_a_master_pairs_any_two(self):
        m, h, _ = _lab_of(((0, 'control'), (2, 'robot2'), (4, 'robot4')))
        s = m[0].request(4, 2)
        assert m[2].following().sid == s.sid
