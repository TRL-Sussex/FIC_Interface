"""Fleet presence, lab-wide commands and stations, without a network."""
import json

from fic_studio.fleet import commands, presence, stations


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


class _Node:
    """Enough of an rclpy node: records publishers, ignores subscriptions."""

    def __init__(self):
        self.pubs = {}

    def create_publisher(self, _type, topic, _qos):
        self.pubs[topic] = _Pub()
        return self.pubs[topic]

    def create_subscription(self, *_args):
        return object()

    def destroy_publisher(self, _pub):
        pass

    def destroy_subscription(self, _sub):
        pass


class _Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _card(studio_id, station, host='h'):
    return json.dumps({'studio_id': studio_id, 'station': station, 'host': host})


class TestStations:
    def test_master_is_zero_and_listed_last(self):
        assert stations.MASTER == 0
        assert stations.all_stations() == [1, 2, 3, 4, 0]

    def test_labels_and_prefixes(self):
        assert stations.station_label(0) == 'Master'
        assert stations.station_label(3) == 'Robot 3'
        assert stations.rid_station('robot4') == 4
        assert stations.rid_station('robot915') == 915
        assert stations.rid_station('robot9') is None
        assert stations.rid_station('robotx') is None

    def test_valid(self):
        assert stations.valid_station(0) and stations.valid_station(4)
        assert not stations.valid_station(5)
        assert not stations.valid_station(True)
        assert not stations.valid_station('1')
        assert stations.valid_station(stations.master_station('control'))

    def test_each_master_gets_its_own_stable_station(self):
        a, b = stations.master_station('control'), stations.master_station('alice-laptop')
        assert a != b and a == stations.master_station('control')
        assert all(stations.MASTER_BASE <= n <= stations.MASTER_TOP for n in (a, b))
        assert stations.resolve_station(0, 'control') == a
        assert stations.resolve_station(3, 'control') == 3
        assert stations.station_label(a) == 'Master' and stations.is_master(a)
        assert stations.picker_value(a) == stations.MASTER and stations.picker_value(2) == 2
        assert not stations.is_master(2) and stations.is_robot(2)


class TestPresence:
    def _dir(self, own_station=0):
        clock = _Clock()
        own = {'studio_id': 'control:1', 'station': own_station, 'host': 'control'}
        return presence.FleetDirectory(_Node(), lambda: own, clock=clock), clock

    def test_own_card_first_then_peers_by_station(self):
        d, _ = self._dir()
        d.on_card_json(_card('r3:9', 3, 'robot3'))
        d.on_card_json(_card('r1:9', 1, 'robot1'))
        assert [c['station'] for c in d.studios()] == [0, 1, 3]
        assert d.studios()[0]['self'] is True

    def test_peer_expires(self):
        d, clock = self._dir()
        d.on_card_json(_card('r1:9', 1))
        clock.t += presence.EXPIRE_S + 0.1
        assert [c['station'] for c in d.studios()] == [0]

    def test_ignores_own_echo_and_garbage(self):
        d, _ = self._dir()
        d.on_card_json(_card('control:1', 0))
        d.on_card_json('not json')
        d.on_card_json(json.dumps({'studio_id': 'x', 'station': 'two', 'host': 'h'}))
        d.on_card_json(json.dumps(['a list']))
        assert len(d.studios()) == 1

    def test_conflict_when_two_studios_claim_one_station(self):
        d, _ = self._dir(own_station=2)
        d.on_card_json(_card('other:5', 2, 'laptop'))
        d.on_card_json(_card('r1:9', 1, 'robot1'))
        assert d.conflicts() == {2: ['control', 'laptop']}

    def test_station_lookup(self):
        d, _ = self._dir()
        d.on_card_json(_card('r4:9', 4, 'robot4'))
        assert d.station(4)['host'] == 'robot4'
        assert d.station(2) is None

    def test_announce_publishes_own_card(self):
        node = _Node()
        own = {'studio_id': 'control:1', 'station': 0, 'host': 'control'}
        d = presence.FleetDirectory(node, lambda: own)
        d.announce()
        assert json.loads(node.pubs[presence.TOPIC].sent[-1]) == own


class TestCommands:
    def _cmds(self, station):
        got = []
        node = _Node()
        c = commands.FleetCommands(node, station, 'host%d' % station,
                                   lambda cmd, info: got.append((cmd, info)))
        return c, got, node

    def test_all_reaches_everyone_once(self):
        c, got, _ = self._cmds(3)
        msg = json.dumps({'id': 'a1', 'cmd': 'estop', 'stations': 'all'})
        c.on_json(msg)
        c.on_json(msg)          # a duplicate is ignored
        assert [g[0] for g in got] == ['estop']

    def test_only_named_stations_act(self):
        c, got, _ = self._cmds(3)
        c.on_json(json.dumps({'id': 'b1', 'cmd': 'estop', 'stations': [1, 2]}))
        assert got == []
        c.on_json(json.dumps({'id': 'b2', 'cmd': 'pause', 'stations': [3]}))
        assert [g[0] for g in got] == ['pause']

    def test_unknown_or_malformed_ignored(self):
        c, got, _ = self._cmds(1)
        c.on_json(json.dumps({'id': 'c1', 'cmd': 'shutdown', 'stations': 'all'}))
        c.on_json('{')
        c.on_json(json.dumps({'id': 'c2', 'cmd': 'estop', 'stations': 'nobody'}))
        assert got == []

    def test_sender_acts_at_once_and_ignores_its_echo(self):
        c, got, node = self._cmds(0)
        c.send('estop', 'all')
        assert [g[0] for g in got] == ['estop']
        c.on_json(node.pubs[commands.TOPIC].sent[-1])
        assert len(got) == 1

    def test_sender_not_named_does_not_act(self):
        c, got, node = self._cmds(0)
        c.send('estop', [2, 3])
        assert got == []
        assert json.loads(node.pubs[commands.TOPIC].sent[-1])['stations'] == [2, 3]

    def test_rejects_unknown_command(self):
        c, _, _ = self._cmds(0)
        try:
            c.send('reboot')
        except ValueError:
            return
        raise AssertionError('send() accepted an unknown command')


class TestSessionCommand:
    def _cmds(self, station):
        got = []
        c = commands.FleetCommands(_Node(), station, 'h', lambda cmd, info: got.append(info))
        return c, got

    def test_start_reaches_only_the_named_station_with_its_target(self):
        c, got = self._cmds(2)
        c.on_json(json.dumps({'id': 's1', 'cmd': 'session', 'stations': [2],
                              'op': 'start', 'target': 'simulation'}))
        c.on_json(json.dumps({'id': 's2', 'cmd': 'session', 'stations': [3],
                              'op': 'start', 'target': 'simulation'}))
        assert [(g['op'], g['target']) for g in got] == [('start', 'simulation')]

    def test_session_for_all_or_bad_op_is_ignored(self):
        c, got = self._cmds(2)
        c.on_json(json.dumps({'id': 's3', 'cmd': 'session', 'stations': 'all', 'op': 'start',
                              'target': 'hardware'}))
        c.on_json(json.dumps({'id': 's4', 'cmd': 'session', 'stations': [2], 'op': 'reboot'}))
        assert got == []

    def test_send_checks_its_arguments(self):
        c, _ = self._cmds(0)
        for kwargs in ({'stations': 'all', 'op': 'stop'},
                       {'stations': [1, 2], 'op': 'stop'},
                       {'stations': [1], 'op': 'start'},
                       {'stations': [1], 'op': 'start', 'target': 'vr'}):
            try:
                c.send('session', **kwargs)
            except ValueError:
                continue
            raise AssertionError('accepted %r' % kwargs)
        assert c.send('session', [1], op='start', target='hardware')['target'] == 'hardware'


class TestClientList:
    def test_clients_by_station_and_expiry(self):
        clock = _Clock()
        cl = presence.ClientList(_Node(), clock=clock)
        cl.on_card_json(json.dumps({'client_id': 'lap:1', 'station': 2, 'host': 'lap',
                                    'script': 'grip.py', 'node': 'alice'}))
        cl.on_card_json(json.dumps({'client_id': 'lap:2', 'station': 3, 'host': 'lap'}))
        cl.on_card_json('junk')
        cl.on_card_json(json.dumps({'client_id': 'x', 'station': 'two'}))
        assert [c['script'] for c in cl.clients(2)] == ['grip.py']
        assert len(cl.clients()) == 2
        clock.t += presence.CLIENT_EXPIRE_S + 0.1
        assert cl.clients() == []
