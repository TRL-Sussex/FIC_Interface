"""MuJoCo Playground: a sandbox for learning to program the robot in simulation.

Pick an arm and a ready-made scene (a table and a few objects), press Start: the Studio's own
simulator and FIC controller start with that scene, as its SIM and FIC buttons would, so
everything else in the Studio works on it (the Joint, Cart and Gripper panels, Snapshot and Quick
Actions, Sequences, Teach Mode, the Script Editor). The page shows where every object is (the
simulator publishes their poses) and walks through the exercises: move the arm, record waypoints
into a sequence, play it, run trajectory templates, and pick an object.

The Studio side is web_state() / web_action(); web/page.js only draws.
"""
import json
import os

from fic_studio.extensions.base import Extension as _Base

from . import pick as pk
from . import scene_builder as sb

HERE = os.path.dirname(os.path.abspath(__file__))
WS_SRC = os.path.abspath(os.path.join(HERE, '..', '..', 'fic_ws', 'src'))
#: Seconds between starting the simulator and starting the FIC controller.
FIC_DELAY_MS = 3000
LOG_LINES = 100


class Extension(_Base):
    ID = 'mujoco_playground'
    NAME = 'MuJoCo Playground'
    DESCRIPTION = ('A sandbox for learning to program the robot: pick an arm and a scene with '
                   'objects, start it in one click, then move, record, sequence and pick.')
    SIDEBAR_SECTION = 'Programming'

    def __init__(self, host):
        super().__init__(host)
        saved = host.get_setting('scene', None) or {}
        preset = saved.get('preset', 'pick')
        _, table, objects = sb.PRESETS.get(preset, sb.PRESETS['pick'])
        self._objects = saved.get('objects', [dict(o) for o in objects])
        self._table = bool(saved.get('table', table))
        self._robot = saved.get('robot')
        # the ideal simulator by default (a responsive sandbox); like the real robot on request
        self._robot_like = bool(saved.get('robot_like', False))
        self._scene_path = None          # the scene this extension started
        self._starting = False
        self._poses = {}                 # object name -> [x, y, z, qw, qx, qy, qz]
        self._poses_sub = None
        self._log = []
        self._robots_cache = None
        if hasattr(host, 'add_quick_action'):
            host.add_quick_action('pick', 'Pick', self._quick_pick, choices=self._pickable, button='Pick',
                                  hint='Playground: builds the pick as a sequence and plays it')

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def teardown(self):
        self._unsubscribe()
        if hasattr(self.host, 'remove_quick_action'):
            self.host.remove_quick_action('pick')

    def on_emergency_stop(self):
        self._starting = False

    # ── Helpers ───────────────────────────────────────────────────────────
    def _say(self, text, level='INFO'):
        self._log.append(text)
        del self._log[:-LOG_LINES]
        self.host.log(level, text, 'Playground')

    def _robots(self):
        if self._robots_cache is None:
            self._robots_cache = sb.robots(self.host.robot_configs, WS_SRC)
        return self._robots_cache

    def _robot_row(self, name):
        return next((r for r in self._robots() if r['name'] == name), None)

    def _selected(self):
        """The robot the page works on: the saved choice, else the Studio's, else the first usable."""
        usable = [r['name'] for r in self._robots() if r['ok']]
        for name in (self._robot, self.host.current_robot_name()):
            if name in usable:
                return name
        return usable[0] if usable else None

    def _save(self, preset=None):
        cur = self.host.get_setting('scene', None) or {}
        self.host.set_setting('scene', {'preset': preset or cur.get('preset', 'custom'),
                                        'objects': self._objects, 'table': self._table,
                                        'robot': self._robot, 'robot_like': self._robot_like})

    def _sid(self):
        return int(getattr(self.host.node, 'station_id', 1) or 1)

    def _active(self):
        """True while the simulator runs the scene this extension started."""
        return bool(self._scene_path and self.host.simulation_running
                    and self.host.simulation_scene(self._robot) == self._scene_path)

    def _subscribe(self):
        if self._poses_sub is not None:
            return
        from std_msgs.msg import String
        from rclpy.qos import qos_profile_sensor_data

        def on_poses(msg):
            try:
                self._poses = json.loads(msg.data)
            except ValueError:
                pass
        topic = '/robot%d_simulation/object_poses' % self._sid()
        self._poses_sub = self.host.node.create_subscription(String, topic, on_poses, qos_profile_sensor_data)

    def _unsubscribe(self):
        if self._poses_sub is not None:
            try:
                self.host.node.destroy_subscription(self._poses_sub)
            except Exception:
                pass
            self._poses_sub = None
        self._poses = {}

    # ── Pick ──────────────────────────────────────────────────────────────
    def _pickable(self):
        """Objects that can be picked now: the running scene's, the table excluded."""
        return sorted(n for n in self._poses if n != 'table') if self._active() else []

    def _quick_pick(self, choice):
        return self._pick(choice)

    def _pick(self, name):
        """Build the pick of ``name`` as the sequence pick_<name> (Sequences page) and play it."""
        if not self._active():
            raise ValueError('Start the Playground first')
        pose = self._poses.get(name)
        if pose is None or name == 'table':
            raise ValueError('No object called %s in the scene' % name)
        seq = self.host.main_window.get_extension('sequences') if hasattr(self.host.main_window, 'get_extension') else None
        if seq is None:
            raise ValueError('Turn on the Sequences extension (Settings, Extensions): the pick is a sequence')
        if seq.target_is_hardware():
            raise ValueError('The pick runs in the Playground\'s simulator: set the Sequences target to Simulation')
        home = (self.host.robot_configs.get(self._robot) or {}).get('q_home_rad')
        wps = pk.pick_waypoints(self._robot, pose[:3], home_rad=home)
        seq_name = seq.add_sequence('pick_%s' % name, wps)
        # Sequences switches to Joint Impedance with Planner for the home waypoint and to Cartesian
        # Impedance with Planner (with H-PMP, the control bar's default) for the Cartesian ones.
        seq.play(seq_name)
        self._say('Picking %s: playing the sequence %s (open, above, down, close, lift); '
                  'the Sequences page shows its waypoints' % (name, seq_name))
        return 'Picking %s with the sequence %s' % (name, seq_name)

    # ── Start / stop ──────────────────────────────────────────────────────
    def _start(self):
        name = self._selected()
        row = self._robot_row(name) if name else None
        if row is None or not row['ok']:
            raise ValueError('No robot that can run in the Playground')
        rid = 'robot%d' % self._sid()
        if self.host.robot_target(rid) == 'hardware' and self.host.controller_running:
            raise ValueError('%s runs the real robot: stop its controller and switch it to Simulation first' % rid)
        path = sb.build(row['scene'], self._objects, table=self._table, name='%s_playground' % name)
        self._robot = name
        self._save()
        if self.host.current_robot_name() != name:
            self.host.select_robot(name)
        if self.host.robot_target(rid) != 'simulation':
            self.host.set_robot_target(rid, 'simulation')
        self.host.stop_fic_simulation()
        self.host.stop_simulation()
        self.host.set_simulation_scene(path, name)
        if hasattr(self.host, 'set_simulation_env'):
            self.host.set_simulation_env({} if self._robot_like else {'FIC_HARDWARE_MODEL': 'off'})
        if hasattr(self.host, 'refresh_profile'):
            self.host.refresh_profile()          # 'sim' tuning in the ideal simulator, 'default' like the arm
        self._scene_path = path
        self.host.start_simulation()
        self._starting = True
        self._subscribe()
        self._say('Started %s with %d object(s)%s, %s' % (
            name, len(self._objects), ' on the table' if self._table else '',
            'simulated like the real robot (its friction, damping and sensor noise)' if self._robot_like
            else 'in the ideal simulator'))
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(FIC_DELAY_MS, self._start_fic)

    def _start_fic(self):
        self._starting = False
        if not self.host.simulation_running:
            self._say('The simulator did not start: see the Log page', 'WARN')
            return
        if not self.host.fic_running:
            self.host.start_fic_simulation()
            self._say('FIC controller started: use the Joint, Cart and Gripper panels below')
        if hasattr(self.host, 'send_hpmp'):
            # the controller starts with H-PMP off and misses the Studio's earlier message: send the
            # control bar's setting once it is up
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(FIC_DELAY_MS, self.host.send_hpmp)

    def _stop(self):
        self._starting = False
        self.host.stop_fic_simulation()
        self.host.stop_simulation()
        if self._robot:
            self.host.set_simulation_scene(None, self._robot)
        if hasattr(self.host, 'set_simulation_env'):
            self.host.set_simulation_env(None)
        if hasattr(self.host, 'refresh_profile'):
            self.host.refresh_profile()
        self._scene_path = None
        self._unsubscribe()
        self._say('Stopped')

    # ── The web page ──────────────────────────────────────────────────────
    def web_state(self):
        name = self._selected()
        row = self._robot_row(name) if name else None
        return {
            'robots': [{'name': r['name'], 'ok': r['ok'], 'gripper': r['gripper'], 'reason': r['reason']}
                       for r in self._robots()],
            'robot': name,
            'gripper': bool(row and row['gripper']),
            'presets': [{'key': k, 'title': v[0]} for k, v in sb.PRESETS.items()],
            'preset': (self.host.get_setting('scene', None) or {}).get('preset', 'pick'),
            'table': self._table,
            'table_height': sb.TABLE['height'],
            'objects': self._objects,
            'shapes': {k: list(v['fields']) for k, v in sb.SHAPES.items()},
            'colors': list(sb.COLORS),
            'robot_like': self._robot_like,
            'running': self._active(),
            'starting': self._starting,
            'sim_running': bool(self.host.simulation_running),
            'fic_running': bool(self.host.fic_running),
            'poses': self._poses if self._active() else {},
            'can_pick': bool(row and row['gripper'] and row['name'] in pk.GRASP),
            'log': self._log[-LOG_LINES:],
        }

    def web_action(self, name, args):
        args = args or {}
        busy = self._active() or self._starting

        def idle():
            if busy:
                raise ValueError('Stop the Playground first, then change the scene')

        if name == 'robot':
            idle()
            row = self._robot_row(args.get('name'))
            if row is None or not row['ok']:
                raise ValueError(row['reason'] if row else 'No such robot')
            self._robot = row['name']
            self._save()
        elif name == 'preset':
            idle()
            key = args.get('key')
            if key not in sb.PRESETS:
                raise ValueError('No such scene')
            _, table, objects = sb.PRESETS[key]
            self._objects, self._table = [dict(o) for o in objects], table
            self._save(preset=key)
        elif name == 'robot_like':
            idle()
            self._robot_like = bool(args.get('value'))
            self._save()
        elif name == 'table':
            idle()
            self._table = bool(args.get('value'))
            self._save()
        elif name in ('add_object', 'update_object'):
            idle()
            obj = sb.check_object(args.get('object') or {})
            others = [o for i, o in enumerate(self._objects)
                      if not (name == 'update_object' and i == int(args.get('index', -1)))]
            if any(o['name'] == obj['name'] for o in others):
                raise ValueError('There is already an object called %s' % obj['name'])
            if name == 'add_object':
                if len(self._objects) >= sb.MAX_OBJECTS:
                    raise ValueError('At most %d objects' % sb.MAX_OBJECTS)
                self._objects.append(obj)
            else:
                i = int(args.get('index', -1))
                if not 0 <= i < len(self._objects):
                    raise ValueError('No such object')
                self._objects[i] = obj
            self._save()
        elif name == 'remove_object':
            idle()
            i = int(args.get('index', -1))
            if not 0 <= i < len(self._objects):
                raise ValueError('No such object')
            self._objects.pop(i)
            self._save()
        elif name == 'start':
            if busy:
                raise ValueError('The Playground is already running')
            self._start()
        elif name == 'stop':
            self._stop()
        elif name == 'pick':
            return {'message': self._pick(str(args.get('name') or ''))}
        else:
            raise ValueError('No such action: %s' % name)
        return {}
