"""Teleop sessions across Studios: who leads whom, and each Studio's part in it.

Any Studio can ask for "station L leads station F". The messages travel on
``SESSION_TOPIC`` (JSON), and each Studio acts only for its own station:

  request  initiator -> all   {sid, leader, follower, scale, max_speed,
                               leader_robot, leader_joints, from_*}
  accept   follower  -> all   the follower's Studio agreed (automatically in
                              simulation; its operator is asked on hardware)
  reject   follower  -> all   with a reason (busy, different robot, not running…)
  stop     anyone    -> all   ends the session, with a reason
  alive    leader and follower, every ALIVE_S while active
  command  operator  -> one   {to, action: request|accept|stop|mode, ...}: do
                              what an operator would have done AT that Studio.
                              Only the station named in ``to`` acts on it.
                              ``mode`` is not teleoperation, but it is the same
                              question -- drive this Studio from elsewhere -- so
                              it shares the channel rather than inventing a
                              second one.

``command`` exists because the robot PCs' desktops are reached over NoMachine,
and on 2026-09-30 connecting over NoMachine turned out to be what kept powering
robot1 off. Running a session therefore has to be possible without opening a
Studio at all. It is a command rather than a forged ``accept`` because accepting
is not only an announcement: ``accept()`` puts the follower's own arm into
Mode 1 with the planner off through ``begin_following()``, so it has to run on
that machine. Every guard still applies, because the work goes through the same
``request()``/``accept()`` the buttons call.

Once accepted, the leader's Studio streams its arm's joint positions on
``LEADER_TOPIC`` at the tick rate (one stream, however many followers); each
message is ``[leader station, q1, q2, ...]``. Every Studio opens that topic, to
publish and to listen, when it starts: a publisher or subscription created at
session start would first have to be discovered by the other side, which took
6-12 s across the lab's wifi (measured 2026-09-21), longer than a follower waits.
The follower's Studio turns them into targets with a ``JointFollower`` and
sends them to its own arm through the hooks.

Liveness, checked every tick:
  * follower: no leader data for HOLD_S -> stop sending targets (the arm holds
    the last one); none for LOST_S -> end the session
  * leader: no word from a follower for LOST_S + 1 -> end that session
  * initiator: no answer within ANSWER_S -> the request failed

The force channel runs the other way, on ``FOLLOWER_WRENCH_TOPIC``: while
following, a Studio reports what its arm is touching, and the leader's Studio
puts that onto its own arm so the operator feels it. ``force_scale`` starts at
zero, so a pairing behaves exactly as it did before anyone asks for force.

``hooks`` is the Studio: robot_type(), joint_count(), joint_positions(),
joint_limits(), target(), set_mode(mode), begin_following(session), publish_target(q),
end_following(session, reason), confirm_request(session), on_change(),
log(level, text), wrench(), publish_wrench(w6). Qt-free; the clock is injected
for tests.
"""
import json
import os
import time
import uuid

from fic_studio.teleop.tracker import JointFollower

SESSION_TOPIC = '/fic/teleop/session'
ALIVE_S = 0.5
HOLD_S = 0.3
#: How long without word from a partner before a session is torn down.
#: Was 2.0 s, which sat below this lab's own discovery stall: a process JOINING
#: the ROS domain was measured on 2026-10-05 to halt the leader's joint stream
#: for 1.87 s with nothing else happening, and 2.24 s when that process also
#: published, which killed the pairing every time. The arm is never released by
#: this timeout -- HOLD_S stops new targets after 0.3 s and the arm holds the
#: last one under the FIC law -- so the only cost of waiting longer is noticing
#: a genuinely dead partner a few seconds later.
LOST_S = 5.0
#: How often a leader measures the round trip to each follower. Slow on
#: purpose: it is a diagnostic, and it shares the session topic with the
#: protocol itself.
PING_S = 1.0

#: A follower waits this long for the leader's first data.
FIRST_DATA_S = 10.0
#: How long a request may wait for its answer (an operator may be asked).
ANSWER_S = 30.0
#: A station asked to follow, whose arm is not publishing yet, waits this long
#: for it. Starting the robot and asking it to follow are one move in practice,
#: and the arm takes a few seconds to say where it is.
ARM_WAIT_S = 12.0

REQUESTED, PENDING, ACTIVE, ENDED = 'requested', 'pending', 'active', 'ended'


#: Every leader's joint stream: Float64MultiArray [leader station, q...].
LEADER_TOPIC = '/fic/teleop/leader_joints'

#: Every follower's contact, back to its leader: Float64MultiArray
#: [follower station, fx, fy, fz, tx, ty, tz]. This is what makes the pairing
#: bilateral rather than mirroring: the leader's arm answers what the
#: follower's arm is touching, so the operator feels the remote contact.
#:
#: Opened at start-up alongside LEADER_TOPIC, for the same reason: discovering
#: a publisher created at session start took 6-12 s over the lab's wifi.
FOLLOWER_WRENCH_TOPIC = '/fic/teleop/follower_wrench'

#: How much of the follower's measured wrench reaches the leader's arm, before
#: the controller's own per-robot clamp. Starts at zero, i.e. the pairing
#: behaves exactly as it did before anyone turns force feedback on. The
#: operator raises it; nothing raises it automatically.
DEFAULT_FORCE_SCALE = 0.0

#: Studios whose requests a follower takes up without asking its operator.
#: The point of driving the lab from one place is not having to go to the other
#: machine to agree, and on this lab's robot PCs "going to the machine" means
#: NoMachine, which is itself what kept powering robot1 off on 2026-09-30. A
#: request from another ROBOT's Studio is still put to the operator on hardware;
#: only the master's is taken on trust, and it is logged at WARN when it is.
#: Override with FIC_TELEOP_MASTER_HOSTS, comma separated host tags.
MASTER_HOSTS = tuple(h.strip() for h in
                     os.environ.get('FIC_TELEOP_MASTER_HOSTS', 'control').split(',')
                     if h.strip())


class Session:
    """One pairing, as this Studio knows it."""

    def __init__(self, sid, leader, follower, scale=1.0, max_speed=0.6,
                 leader_robot='', leader_joints=0, from_host='', from_station=0,
                 origin=''):
        """Record the request."""
        self.sid = sid
        self.leader = leader
        self.follower = follower
        self.scale = scale
        self.max_speed = max_speed
        self.leader_robot = leader_robot
        self.leader_joints = leader_joints
        self.from_host = from_host
        self.from_station = from_station
        #: Host that asked for this pairing, carried across the station that
        #: leads it. A request the master Studio started is accepted without
        #: asking; see MASTER_HOSTS.
        self.origin = origin
        self.state = REQUESTED
        self.reason = ''
        self.created = 0.0
        self.started = 0.0
        self.last_peer = 0.0
        self.last_data = 0.0
        self.tracker = None
        self.sent = 0
        self.holding = False
        #: Round-trip time to the partner, seconds, or None until measured.
        self.rtt = None
        #: Asked to follow before this station's arm was publishing.
        self.waiting_for_arm = False

    def as_dict(self):
        """Return a JSON-able summary (presence card, UI)."""
        return {'sid': self.sid, 'leader': self.leader, 'follower': self.follower,
                'state': self.state, 'scale': self.scale, 'reason': self.reason,
                'holding': self.holding, 'rtt': self.rtt}


class TeleopManager:
    """This Studio's part in every teleop session on the network."""

    def __init__(self, node, station, host, hooks, clock=time.monotonic):
        """Listen on SESSION_TOPIC for sessions involving ``station``."""
        from std_msgs.msg import Float64MultiArray, String
        self._string = String
        self._array = Float64MultiArray
        self._node = node
        self.station = station
        self.host = host
        self.hooks = hooks
        self.clock = clock
        self.sessions = {}
        self._leader_q = None
        self._last_alive = 0.0
        self._last_ping = 0.0
        self._last_tick = None
        self._pub = node.create_publisher(String, SESSION_TOPIC, 20)
        self._sub = node.create_subscription(String, SESSION_TOPIC, self.on_message, 20)
        from rclpy.qos import qos_profile_sensor_data
        self._leader_pub = node.create_publisher(
            Float64MultiArray, LEADER_TOPIC, qos_profile_sensor_data)
        self._leader_sub = node.create_subscription(
            Float64MultiArray, LEADER_TOPIC, self.on_leader_data, qos_profile_sensor_data)
        self._wrench_pub = node.create_publisher(
            Float64MultiArray, FOLLOWER_WRENCH_TOPIC, qos_profile_sensor_data)
        self._wrench_sub = node.create_subscription(
            Float64MultiArray, FOLLOWER_WRENCH_TOPIC, self.on_follower_wrench,
            qos_profile_sensor_data)
        #: follower station -> (wrench6, when it arrived)
        self._follower_w = {}
        #: Whether this Studio is currently holding a force on its own arm, so
        #: the release is sent once rather than every tick.
        self._reflecting = False
        self.force_scale = DEFAULT_FORCE_SCALE

    # ── what this Studio is doing ────────────────────────────────────────
    def _mine(self, role):
        key = 'leader' if role == 'leader' else 'follower'
        return [s for s in self.sessions.values()
                if s.state == ACTIVE and getattr(s, key) == self.station]

    def leading(self):
        """Return the active sessions this station leads."""
        return self._mine('leader')

    def following(self):
        """Return the active session this station follows in, or None."""
        mine = self._mine('follower')
        return mine[0] if mine else None

    def visible(self):
        """Return the sessions worth showing: not ended, plus recently ended."""
        now = self.clock()
        return [s for s in self.sessions.values()
                if s.state != ENDED or now - s.last_peer < 10.0]

    def partner_stations(self):
        """Return the stations this Studio is paired with right now."""
        out = set()
        for s in self.sessions.values():
            if s.state == ACTIVE and self.station in (s.leader, s.follower):
                out.add(s.follower if s.leader == self.station else s.leader)
        return sorted(out)

    # ── outgoing ─────────────────────────────────────────────────────────
    def _send(self, msg):
        msg.setdefault('from_host', self.host)
        msg.setdefault('from_station', self.station)
        self._pub.publish(self._string(data=json.dumps(msg)))

    def request(self, leader, follower, scale=1.0, max_speed=0.6,
                leader_robot='', leader_joints=0, origin=''):
        """Ask for ``leader`` to lead ``follower``; return the Session."""
        if leader == follower:
            raise ValueError('a station cannot lead itself')
        s = Session(uuid.uuid4().hex[:8], leader, follower, scale, max_speed,
                    leader_robot, leader_joints, self.host, self.station, origin)
        s.created = s.last_peer = self.clock()
        self.sessions[s.sid] = s
        msg = {'type': 'request', 'sid': s.sid, 'leader': leader, 'follower': follower,
               'scale': scale, 'max_speed': max_speed, 'leader_robot': leader_robot,
               'leader_joints': leader_joints, 'origin': origin}
        self._send(msg)
        # A request to ourselves is handled at once, as if it came back.
        if follower == self.station:
            self._on_request(dict(msg, from_host=self.host, from_station=self.station))
        self._changed()
        return s

    def accept(self, sid):
        """Accept a pending request for this station to follow."""
        s = self.sessions.get(sid)
        if s is None or s.state != PENDING or s.follower != self.station:
            return False
        if self.following():
            self.reject(sid, 'already following another station')
            return False
        s.state = ACTIVE
        s.started = s.last_peer = s.last_data = self.clock()
        s.tracker = None
        self._leader_q = None
        self.hooks.begin_following(s)
        self._send({'type': 'accept', 'sid': sid, 'leader': s.leader, 'follower': s.follower})
        self.hooks.log('INFO', 'Teleop: following station %d (session %s)' % (s.leader, sid))
        self._changed()
        return True

    def reject(self, sid, reason):
        """Refuse a request addressed to this station."""
        s = self.sessions.get(sid)
        if s is not None:
            s.state, s.reason = ENDED, reason
        self._send({'type': 'reject', 'sid': sid, 'reason': reason,
                    'leader': s.leader if s else 0, 'follower': self.station})
        self._changed()

    def stop(self, sid, reason='stopped'):
        """End a session, from any Studio."""
        s = self.sessions.get(sid)
        self._send({'type': 'stop', 'sid': sid, 'reason': reason,
                    'leader': s.leader if s else 0, 'follower': s.follower if s else 0})
        self._end(sid, reason)

    def stop_all(self, reason):
        """End every session this station takes part in (e.g. on E-stop)."""
        for s in list(self.sessions.values()):
            if s.state in (ACTIVE, PENDING) and self.station in (s.leader, s.follower):
                self.stop(s.sid, reason)

    # ── incoming ─────────────────────────────────────────────────────────
    def on_message(self, msg):
        """Handle a session message (ROS callback)."""
        self.on_json(msg.data)

    def on_json(self, text):
        """Handle a session message from its JSON text."""
        try:
            msg = json.loads(text)
        except (TypeError, ValueError):
            return
        if not isinstance(msg, dict) or not isinstance(msg.get('sid'), str):
            return
        kind = msg.get('type')
        if msg.get('from_host') == self.host and msg.get('from_station') == self.station:
            return          # our own message, already acted on
        if kind == 'request':
            self._on_request(msg)
        elif kind == 'accept':
            self._on_accept(msg)
        elif kind == 'reject':
            s = self.sessions.get(msg['sid'])
            if s is not None and s.state != ENDED:
                s.state, s.reason = ENDED, 'refused: %s' % msg.get('reason', '')
                s.last_peer = self.clock()
                self.hooks.log('WARN', 'Teleop %s refused by station %s: %s'
                               % (msg['sid'], msg.get('follower'), msg.get('reason', '')))
                self._changed()
        elif kind == 'stop':
            if msg['sid'] in self.sessions:
                self._end(msg['sid'], msg.get('reason') or 'stopped')
        elif kind == 'command':
            self._on_command(msg)
        elif kind == 'alive':
            s = self.sessions.get(msg['sid'])
            if s is not None:
                s.last_peer = self.clock()
        elif kind == 'ping':
            # Echo it straight back, carrying the leader's own clock value.
            # The round trip is measured on one clock, so the two machines'
            # clocks never have to agree: the lab's robots sit up to 0.8 s
            # apart even with timesyncd running, which is far larger than
            # anything being measured here.
            s = self.sessions.get(msg['sid'])
            if s is not None and s.follower == self.station:
                self._send({'type': 'pong', 'sid': msg['sid'], 't': msg.get('t')})
        elif kind == 'pong':
            s = self.sessions.get(msg['sid'])
            t = msg.get('t')
            if s is not None and s.leader == self.station and isinstance(t, (int, float)):
                # Records the round trip and NOTHING else. Deliberately does
                # not touch last_peer: liveness is the `alive` beat's job, and
                # a diagnostic that quietly kept a session alive would be a
                # bad trade. A Studio can answer a ping while its arm has
                # stopped, and the test for a follower going quiet caught
                # exactly that.
                s.rtt = self.clock() - t

    def _on_request(self, msg):
        try:
            leader, follower = int(msg['leader']), int(msg['follower'])
        except (KeyError, TypeError, ValueError):
            return
        if follower != self.station:
            if leader == self.station and msg['sid'] not in self.sessions:
                s = self._session_from(msg)       # remember it: we may be asked to lead
                s.state = PENDING
                self.sessions[s.sid] = s
            return
        s = self.sessions.get(msg['sid']) or self._session_from(msg)
        self.sessions[s.sid] = s
        s.state = PENDING
        s.last_peer = self.clock()
        self._answer_request(s, leader)
        self._changed()

    def _answer_request(self, s, leader):
        """Refuse, wait for the arm, or accept (asking the operator on hardware)."""
        problem = self._cannot_follow(s)
        if problem:
            self.reject(s.sid, problem)
            self.hooks.log('WARN', 'Teleop request from station %d refused: %s'
                           % (leader, problem))
            return
        if not self._arm_ready():
            # The arm was very likely started a moment ago: give it its few
            # seconds rather than making the operator ask twice. The tick
            # answers when it is ready, or gives up after ARM_WAIT_S.
            if not s.waiting_for_arm:
                s.waiting_for_arm = True
                self.hooks.log('INFO', 'Teleop request from station %d: waiting for '
                                       'this arm to start' % leader)
            return
        s.waiting_for_arm = False
        if s.origin and s.origin in MASTER_HOSTS:
            # Asked for from the master Studio, so the operator has already
            # decided; asking again at this end is the step we are removing.
            self.hooks.log('WARN', 'Teleop: following station %d without asking, '
                                   'requested from the master Studio (%s)'
                           % (leader, s.origin))
            self.accept(s.sid)
        elif self.hooks.target() == 'simulation':
            self.accept(s.sid)
        else:
            self.hooks.confirm_request(s)      # the operator answers: accept() / reject()

    def _on_command(self, msg):
        """Do what an operator would have done at this Studio, asked from elsewhere.

        Nothing here bypasses a check. ``request`` and ``accept`` are the same
        calls the buttons make, so a different robot is still refused, a station
        already following still refuses, and an arm that has not said where it
        is still gets its ARM_WAIT_S. What it skips is only the operator being
        physically at that Studio, which is the whole point.

        ``leader_robot`` and ``leader_joints`` are filled in from the leader's
        OWN hooks rather than taken from the message. A caller that omitted them
        would silently disable the follower's different-robot check, because
        ``_cannot_follow`` only compares when they are set.
        """
        if msg.get('to') != self.station:
            return
        action, who = msg.get('action'), msg.get('from_host') or 'the control PC'
        if action == 'accept':
            # The caller has usually never seen the sid, so fall back to
            # whatever is pending here: there can only be one worth accepting.
            s = self.sessions.get(msg.get('sid'))
            if s is None or s.state != PENDING or s.follower != self.station:
                pending = [x for x in self.sessions.values()
                           if x.state == PENDING and x.follower == self.station]
                s = pending[-1] if pending else None
            if s is None:
                self.hooks.log('WARN', 'Teleop: %s asked this station to accept, '
                                       'but nothing is pending here' % who)
                return
            self.hooks.log('WARN', 'Teleop: session %s accepted by %s, not at this Studio'
                           % (s.sid, who))
            self.accept(s.sid)
        elif action == 'request':
            # ``to`` already names the leader, so ``leader`` is optional; when a
            # caller does send it, an inconsistent one is refused rather than
            # quietly ignored.
            follower = msg.get('follower')
            if msg.get('leader', self.station) != self.station or not isinstance(follower, int):
                return
            self.hooks.log('WARN', 'Teleop: %s asked this station to lead station %d'
                           % (who, follower))
            self.request(self.station, follower,
                         scale=float(msg.get('scale', 1.0)),
                         max_speed=float(msg.get('max_speed', 0.6)),
                         leader_robot=self.hooks.robot_type(),
                         leader_joints=self.hooks.joint_count(),
                         # Carried so the follower can tell a pairing the master
                         # asked for from one a robot Studio started by itself.
                         origin=msg.get('from_host') or '')
        elif action == 'mode':
            mode = msg.get('mode')
            if not isinstance(mode, int) or isinstance(mode, bool) or not 1 <= mode <= 8:
                return
            self.hooks.log('WARN', 'Teleop: %s set this station to control mode %d'
                           % (who, mode))
            self.hooks.set_mode(mode)
        elif action == 'stop':
            self.hooks.log('WARN', "Teleop: %s ended this station's sessions" % who)
            self.stop_all(msg.get('reason') or 'stopped from %s' % who)

    def _session_from(self, msg):
        s = Session(msg['sid'], int(msg['leader']), int(msg['follower']),
                    float(msg.get('scale', 1.0)), float(msg.get('max_speed', 0.6)),
                    str(msg.get('leader_robot', '')), int(msg.get('leader_joints', 0) or 0),
                    str(msg.get('from_host', '')), int(msg.get('from_station', 0) or 0),
                    str(msg.get('origin', '')))
        s.created = s.last_peer = self.clock()
        return s

    def _cannot_follow(self, s):
        if self.following():
            return 'already following station %d' % self.following().leader
        if self.leading():
            return 'this station is leading; it cannot follow at the same time'
        if not 0.05 <= s.scale <= 2.0:
            return 'scale %.2f is outside 0.05-2.0' % s.scale
        robot = self.hooks.robot_type()
        if s.leader_robot and s.leader_robot != robot:
            return 'different robots (%s leads, this is %s): not supported yet' % (
                s.leader_robot, robot)
        n = self.hooks.joint_count()
        if s.leader_joints and s.leader_joints != n:
            return 'joint counts differ (%d vs %d)' % (s.leader_joints, n)
        return ''

    def _arm_ready(self):
        """Say whether this station's arm is publishing where it is."""
        return len(self.hooks.joint_positions() or []) >= self.hooks.joint_count()

    def _on_accept(self, msg):
        s = self.sessions.get(msg['sid'])
        if s is None:
            if msg.get('leader') != self.station:
                return
            s = Session(msg['sid'], int(msg['leader']), int(msg['follower']))
            self.sessions[s.sid] = s
        s.state = ACTIVE
        s.started = s.last_peer = self.clock()
        if s.leader == self.station:
            self.hooks.log('INFO', 'Teleop: leading station %d (session %s)'
                           % (s.follower, s.sid))
        self._changed()

    # ── data ─────────────────────────────────────────────────────────────
    def on_leader_data(self, msg):
        """Take a leader's joint positions (ROS callback): [station, q...]."""
        data = list(msg.data)
        if len(data) < 2:
            return
        self.on_leader_q(int(data[0]), data[1:])

    def on_leader_q(self, leader, q):
        """Store station ``leader``'s joints if it is the one this station follows."""
        s = self.following()
        if s is None or s.leader != leader:
            return
        self._leader_q = q
        s.last_data = self.clock()

    # ── the tick ─────────────────────────────────────────────────────────
    def tick(self):
        """Stream, follow and check liveness. Call at the stream rate (~100 Hz)."""
        now = self.clock()
        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now

        if self.leading():
            q = self.hooks.joint_positions()
            if q:
                self._leader_pub.publish(
                    self._array(data=[float(self.station)] + [float(v) for v in q]))
        self._follow(now, dt)
        self._send_wrench()
        self._reflect(now)

        if now - self._last_alive >= ALIVE_S:
            self._last_alive = now
            for s in self.leading() + ([self.following()] if self.following() else []):
                self._send({'type': 'alive', 'sid': s.sid})
        if now - self._last_ping >= PING_S:
            self._last_ping = now
            for s in self.leading():
                self._send({'type': 'ping', 'sid': s.sid, 't': now})
        self._check_liveness(now)

    # ── the force channel, follower -> leader ────────────────────────────
    def on_follower_wrench(self, msg):
        """A follower's contact. Kept only if this station leads that follower."""
        data = list(msg.data)
        if len(data) < 7:
            return
        station = int(data[0])
        if not any(s.follower == station for s in self.leading()):
            return
        self._follower_w[station] = (data[1:7], self.clock())

    def _send_wrench(self):
        """As a follower, report what this arm is touching."""
        if self.following() is None:
            return
        w = self.hooks.wrench()
        if not w or len(w) < 6:
            return
        self._wrench_pub.publish(
            self._array(data=[float(self.station)] + [float(v) for v in w[:6]]))

    def _reflect(self, now):
        """As a leader, put the follower's contact onto this arm.

        Zeroed, once, whenever there is nothing trustworthy to reflect: no
        session, the gain turned down, or the follower gone quiet for HOLD_S.
        The controller fades a stale wrench by itself, but releasing explicitly
        is faster and does not depend on the controller's timer agreeing with
        this one.

        Only ever one follower's force reaches the arm. A leader may drive
        several followers, and summing their contacts would invent a force no
        single arm is feeling. The lowest station number wins, so it is
        predictable rather than whichever packet arrived last.
        """
        sessions = self.leading()
        w6 = None
        if sessions and self.force_scale > 0.0:
            for s in sorted(sessions, key=lambda s: s.follower):
                got = self._follower_w.get(s.follower)
                if got and now - got[1] <= HOLD_S:
                    w6 = [v * self.force_scale for v in got[0]]
                    break
        if w6 is None:
            if self._reflecting:
                self._reflecting = False
                self.hooks.publish_wrench([0.0] * 6)
            return
        self._reflecting = True
        self.hooks.publish_wrench(w6)

    def _follow(self, now, dt):
        s = self.following()
        if s is None or self._leader_q is None:
            return
        fresh = now - s.last_data <= HOLD_S
        if s.holding == fresh:
            s.holding = not fresh
            self._changed()
        if not fresh:
            return
        try:
            if s.tracker is None:
                own = self.hooks.joint_positions()
                n = self.hooks.joint_count()
                lower, upper = self.hooks.joint_limits()
                s.tracker = JointFollower(n, lower, upper, scale=s.scale,
                                          max_speed=s.max_speed)
                s.tracker.start(self._leader_q, own)
                # Hold where the arm is, now. The tracker sends nothing until
                # the leader actually moves, and a controller with no target of
                # its own lets the arm sag in the meantime.
                self.hooks.publish_target(list(own))
                s.sent += 1
                return
            target = s.tracker.step(self._leader_q, dt)
        except ValueError as exc:
            self.stop(s.sid, 'bad joint data: %s' % exc)
            return
        if target is not None:
            self.hooks.publish_target(target)
            s.sent += 1

    def _check_liveness(self, now):
        for s in list(self.sessions.values()):
            if s.state == ACTIVE and s.follower == self.station:
                waited = now - s.last_data
                if s.tracker is None and now - s.started > FIRST_DATA_S and waited > LOST_S:
                    self.stop(s.sid, 'no data from leader station %d' % s.leader)
                elif s.tracker is not None and waited > LOST_S:
                    self.stop(s.sid, 'lost leader station %d' % s.leader)
            elif s.state == ACTIVE and s.leader == self.station:
                if now - s.last_peer > LOST_S + 1.0:
                    self.stop(s.sid, 'lost follower station %d' % s.follower)
            elif s.state == ACTIVE and now - s.last_peer > LOST_S + 3.0:
                # Someone else's pair (we only asked for it): its alive messages stopped.
                s.state, s.reason = ENDED, 'no longer heard'
                self._changed()
            elif s.state == ENDED and now - s.last_peer > 60.0:
                del self.sessions[s.sid]
            elif s.state == REQUESTED and now - s.created > ANSWER_S:
                s.state, s.reason = ENDED, 'no answer from station %d' % s.follower
                self._changed()
            elif s.state == PENDING and s.follower == self.station and s.waiting_for_arm:
                if self._arm_ready():
                    self._answer_request(s, s.leader)
                    self._changed()
                elif now - s.created > ARM_WAIT_S:
                    s.waiting_for_arm = False
                    self.reject(s.sid, 'this station has no joint data '
                                       '(simulation or robot not running)')
                    self.hooks.log('WARN', 'Teleop request from station %d refused: '
                                           'this arm never said where it is' % s.leader)
            elif s.state == PENDING and now - s.created > ANSWER_S:
                if s.follower == self.station:
                    self.reject(s.sid, 'not accepted in time')
                else:
                    s.state = ENDED
                    self._changed()

    # ── ending ───────────────────────────────────────────────────────────
    def _end(self, sid, reason):
        s = self.sessions.get(sid)
        if s is None or s.state == ENDED:
            return
        was_active = s.state == ACTIVE
        s.state, s.reason = ENDED, reason
        s.last_peer = self.clock()
        if s.follower == self.station:
            self._leader_q = None
            if was_active:
                self.hooks.end_following(s, reason)
        if self.station in (s.leader, s.follower):
            self.hooks.log('INFO', 'Teleop %s ended: %s' % (sid, reason))
        self._changed()

    def _changed(self):
        try:
            self.hooks.on_change()
        except Exception:
            pass

    def close(self):
        """End our sessions and stop listening."""
        self.stop_all('Studio closed')
        for destroy, handle in ((self._node.destroy_subscription, self._sub),
                                (self._node.destroy_publisher, self._pub),
                                (self._node.destroy_subscription, self._leader_sub),
                                (self._node.destroy_publisher, self._leader_pub)):
            if handle is None:
                continue
            try:
                destroy(handle)
            except Exception:
                pass
