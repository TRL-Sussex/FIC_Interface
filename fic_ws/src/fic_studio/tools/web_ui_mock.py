"""Serve the web UI against a mock Studio: no ROS, no Qt, no robot.

    PYTHONPATH=<pkg root> python3 web_ui_mock.py [port]

For design work and screenshots. The real WebRemoteServer serves the real UI
files; only the state and the command handlers are stand-ins, and the state
moves a little so the live parts look live. Binds to 127.0.0.1 only.
"""
import math
import os
import shutil
import sys
import tempfile
import threading
import time
import types

from fic_studio.web_remote import actions
from fic_studio.web_remote.lease import ControlLease
from fic_studio.web_remote.server import WebRemoteServer
from fic_studio.xml_manager import find_config_path

ROBOT_NAME = 'KinovaGen3_7DOF'
SCENE_XML = os.path.join(os.path.dirname(__file__), '..', '..', 'menagerie', 'model',
                         'KinovaGen3_7DOF', 'xml', 'scene.xml')

LABELS = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7']
LIMITS = [(-180, 180), (-128, 128), (-180, 180), (-147, 147), (-180, 180), (-120, 120), (-180, 180)]
_ROWS = ['Impedance', 'Planner', 'Admittance', 'Adm. Haptic']
MODES = {
    'joint': {'label': 'Joint', 'modes': [{'index': i, 'label': l, 'name': 'Joint ' + l}
                                         for i, l in enumerate(_ROWS)]},
    'cart': {'label': 'Cart', 'modes': [{'index': 4 + i, 'label': l, 'name': 'Cart ' + l}
                                        for i, l in enumerate(_ROWS)]},
}


class _ProfileCombo:
    """The active-config selector, as far as the tuning page's live trial drives it."""

    def __init__(self, mock, w):
        self.mock, self.w = mock, w

    def _names(self):
        from fic_studio.xml_manager import XMLManager
        return XMLManager(self.w._web_config_path).get_all_profiles(self.mock.robot) or ['default']

    def currentData(self):
        return self.mock.profile

    currentText = currentData

    def findData(self, name):
        names = self._names()
        return names.index(name) if name in names else -1

    def currentIndex(self):
        return self.findData(self.mock.profile)

    def setCurrentIndex(self, i):
        self.mock.profile = self._names()[i]
        self.mock.say('INFO', 'Active config -> %s' % self.mock.profile, 'Control')


class _Combo:
    def __init__(self, f):
        self.f = f

    def currentText(self):
        return self.f()


class _Directory:
    """A small lab: this Studio and two robot stations."""

    def __init__(self, mock):
        self.mock = mock

    def studios(self):
        m = self.mock
        return [
            {'station': 1, 'host': 'robot-master', 'robot': m.robot, 'robot_display': 'Kinova Gen3 7DOF',
             'target': m.target, 'running': m.running, 'has_joints': True, 'mode': 'Joint Impedance',
             'teleop': [], 'urls': [], 'joints': 7},
            {'station': 11, 'host': 'robot1', 'robot': 'KinovaGen3_7DOF', 'robot_display': 'Kinova Gen3 7DOF',
             'target': 'hardware', 'running': True, 'has_joints': True, 'mode': 'Cartesian Impedance',
             'teleop': [{'sid': 'ab12', 'leader': 12, 'follower': 11, 'state': 'active'}],
             'urls': ['http://robot1:8080'], 'joints': 7},
            {'station': 12, 'host': 'robot2', 'robot': 'KinovaGen3_7DOF', 'robot_display': 'Kinova Gen3 7DOF',
             'target': 'simulation', 'running': False, 'has_joints': False, 'mode': '',
             'teleop': [{'sid': 'ab12', 'leader': 12, 'follower': 11, 'state': 'active'}],
             'urls': [], 'joints': 7},
        ]

    def station(self, n):
        return next((c for c in self.studios() if c['station'] == n), None)

    def conflicts(self):
        return {}


class _Toggle:
    """A checkbox, as far as the web handlers can tell."""

    def __init__(self, on=False, text=''):
        self.on, self._text = on, text

    def isChecked(self):
        return self.on

    def setChecked(self, v):
        self.on = bool(v)

    def text(self):
        return self._text


class _Led:
    def __init__(self, status='idle'):
        self._status = status


class _Label:
    def __init__(self, text=''):
        self._t = text

    def text(self):
        return self._t


class _GT:
    """The contact panel's ground-truth combo."""

    def __init__(self):
        self.items, self.cur = ['none', 'Shoulder_Link', 'HalfArm1_Link', 'ForeArm_Link', 'Bracelet_Link'], 'none'

    def count(self):
        return len(self.items)

    def itemText(self, i):
        return self.items[i]

    def currentText(self):
        return self.cur

    def setCurrentText(self, t):
        self.cur = t


class _Phase_C(object):
    """The Studio surface the phase-C pages read: chrome, settings, robot list, topics, help."""

    def _init_c(self, mock):
        from fic_studio.mixins.settings_mixin import DEFAULT_SETTINGS
        import copy
        self.settings = copy.deepcopy(DEFAULT_SETTINGS)
        self.settings['general']['default_robot'] = 'KinovaGen3_7DOF'
        self.SETTINGS_FILE = os.path.join(tempfile.mkdtemp(prefix='fic_mock_set_'), 'settings.yaml')
        self.ros_led, self.fic_led, self.sim_led = _Led('connected'), _Led('connected'), _Led('connected')
        self.bags_led, self.il_led, self.teleop_led = _Led('idle'), _Led('connected'), _Led('active')
        self.gamepad_led, self._web_led = _Led('idle'), _Led('connected')
        self.il_label, self.teleop_status_bar_label = _Label('MCU (eth0) · running'), _Label('LEADER')
        self.gamepad_state_label, self._web_status_label = _Label('GAMEPAD'), _Label('WEB 1')
        self.robot_status_label = _Label('Robot: Kinova Gen3 7DOF (7 DOF)')
        self._kbd_on = False
        self.bags_running = self.plotter_running = self.contact_twin_running = False
        self.node = types.SimpleNamespace(station_id=1, station_robot_id='robot1', _active_robots=[1, 2],
                                          active_robots=[1, 2], add_robot=lambda n: self.node._active_robots.append(n),
                                          remove_robot=lambda n: self.node._active_robots.remove(n),
                                          publish_position=lambda p: mock.say('INFO', 'preset pose published', 'Control'),
                                          get_topic_names_and_types=lambda: TOPICS,
                                          count_publishers=lambda n: 1, count_subscribers=lambda n: 2)
        self._rc_active_rid = 'robot1'
        self.presets = [types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=z))
                        for x, y, z in ((0.0, 0.0, 0.0), (0.0, 0.0, 1.31), (1.0, 0.2, 1.0), (0.5, -0.5, 0.5))]
        self._rc_live_publish = _Toggle(False)
        self._cb_gt_combo = _GT()
        self._rc_postural_toggle, self._rc_elbow_toggle = _Toggle(False), _Toggle(False)
        self._rc_hpmp_toggle, self._rc_admittance_planner_toggle = _Toggle(False), _Toggle(True)
        self.robot_configs['KinovaGen3_7DOF'].update({
            'joint_names': ['Actuator%d' % i for i in range(1, 8)], 'q_home_deg': [0, 15, 180, 230, 0, 55, 90],
            'urdf_path': 'menagerie/model/KinovaGen3_7DOF/URDF/gen3.urdf', 'mujoco_xml_path': 'menagerie/model/KinovaGen3_7DOF/xml/scene.xml',
            'ee_frame_name': 'tool_frame', 'elbow_frame_name': 'forearm_link', 'kinematics_solver': 'pinocchio', 'torque_sign': 1,
            'default_ee_target': {'position': [0.45, 0.0, 0.4], 'orientation': [0.0, 1.0, 0.0, 0.0]}, 'is_supported': True,
            'joint_torque_limits': {'max': [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]},
            'gripper_range': [0.0, 0.08], 'admittance_deadbands': {'joint_tau': 1.5, 'cart_force': 4.0, 'cart_torque': 0.2},
            'safety_zones': {'cartesian': {'x': {'min': -0.8, 'max': 0.8, 'enabled': True}, 'y': {'min': -0.6, 'max': 0.6, 'enabled': True},
                                           'z': {'min': 0.05, 'max': 0.9, 'enabled': True}},
                             'joint_safety_deg': {'lower': [-170] * 7, 'upper': [170] * 7, 'enabled': True},
                             'velocity': {'max_joint_vel': 1.0, 'max_cart_vel': 0.5, 'enabled': True}},
            'driver_command': 'ros2 run kortex_driver', 'build_directory': '{robot_lib}/kinova', 'build_command': 'make'})
        self._extensions_manifests = [
            types.SimpleNamespace(id='sequences', name='Sequences', description='Record and play waypoint sequences.', sidebar_icon=''),
            types.SimpleNamespace(id='experiments', name='Experiments', description='Named rosbag sessions with replay.', sidebar_icon=''),
            types.SimpleNamespace(id='gaze', name='Gaze teleop', description='Eye tracking drives the camera.', sidebar_icon='')]
        self._ext_enabled = ['sequences', 'gaze']
        self.settings['extensions']['enabled'] = self._ext_enabled
        self.monitored_topics, self.topic_last_msg, self.topic_msg_count, self.topic_cached_msg = {}, {}, {}, {}
        self._tree_topic_types, self._selected_topic = {}, None
        self.topic_signals = types.SimpleNamespace(message_received=types.SimpleNamespace(connect=lambda f: setattr(self, '_topic_cb', f)))
        self.joint_update_count = lambda rid: 48210
        self.joint_update_time = lambda rid: time.monotonic() - 0.002
        self._web_server = None
        self._web_urls = ['http://127.0.0.1:8088', 'http://192.168.0.20:8088']
        self._web_lease = types.SimpleNamespace(holder=lambda: None)

    def _rc_display_joints(self, robot, cfg=None):
        from fic_studio.mixins.robot_control_mixin import RobotControlMixin
        return RobotControlMixin._rc_display_joints(self, robot, cfg)

    # settings
    def _save_settings(self):
        mock = self.mock
        mock.say('INFO', 'Settings saved', 'Settings')

    def _apply_settings(self):
        pass

    def _save_extensions_enabled(self, ids):
        self._ext_enabled = list(ids)
        self.settings['extensions']['enabled'] = list(ids)
        for m in self._extensions_manifests:
            pass

    def _web_settings(self):
        return {'enabled': True, 'port': 8088}

    def _web_save_setting(self, key, value):
        pass

    def _web_start(self):
        pass

    def _web_stop(self):
        pass

    def _web_revoke_control(self):
        pass

    def _get_workspace_path(self):
        return os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')

    # robots
    def _rc_persist(self):
        self.robots = {k: v.get('num_joints', 7) for k, v in self.robot_configs.items()}
        self.mock.say('INFO', 'Robot configurations saved', 'Config')
        return True

    def _rc_check_sync_state(self):
        return False

    def _rc_publish_hot_config(self, rid):
        pass

    # control-panel odds and ends
    def pause_for(self, rid):
        self.mock.say('WARN', 'Pause sent to %s' % rid, 'Control')

    def stop_for(self, rid):
        self.mock.say('WARN', 'Stop sent to %s' % rid, 'Control')

    def _reset_rc_defaults(self):
        self.mock.say('INFO', 'Control fields reset', 'Control')

    def _rc_rebuild_robot_selector(self):
        pass

    def _rebuild_rc_control_area(self):
        pass

    def set_keyboard_control(self, on):
        self._kbd_on = bool(on)

    def get_extension(self, ext_id):
        return types.SimpleNamespace(state_running=False) if ext_id == 'experiments' else None

    def _toggle_bags(self):
        self.bags_running = not self.bags_running
        self.bags_led._status = 'warning' if self.bags_running else 'idle'

    def _toggle_plotter(self):
        self.plotter_running = not self.plotter_running

    def _cb_on_contact_twin(self):
        self.contact_twin_running = not self.contact_twin_running

    def _quick_start_experiment(self):
        self.mock.say('INFO', 'Quick experiment started', 'Experiment')

    def _toggle_simulation(self):
        self._flip('simulation_running', not self.simulation_running)

    def _toggle_fic_simulation(self):
        self._flip('fic_running', not self.fic_running)

    def _toggle_controller(self):
        self._flip('controller_running', not self.controller_running)

    def _toggle_driver(self):
        self._flip('driver_running', not self.driver_running)

    def _dash_toggle_sim(self):
        self._tb_toggle_start()

    def _open_experiments_folder(self):
        self.mock.say('INFO', 'Opened the experiments folder', 'System')

    def _open_experiment_dir(self, d):
        self.mock.say('INFO', 'Opened %s' % d, 'System')

    def serve_docs(self, page):
        self.mock.say('INFO', 'Documentation opened at %s' % page, 'Help')
        return True

    # topic monitor, as the real mixin does it
    def _start_monitoring(self, name, kind):
        from collections import deque
        self.monitored_topics[name] = object()
        self.topic_last_msg[name] = deque(maxlen=100)
        self.topic_msg_count[name] = 0

    def _stop_monitoring(self, name):
        self.monitored_topics.pop(name, None)

    def _toggle_monitor(self, name, kind):
        if name in self.monitored_topics:
            self._stop_monitoring(name)
        else:
            self._start_monitoring(name, kind)

    def _save_monitored_topics(self):
        pass

    # telemetry the dashboard and tuning pages read
    def joint_commanded_for(self, rid):
        return list(self.mock.traj()[0])

    def ee_target_for(self, rid):
        return self.mock.traj()[3]

    def joint_effort_for(self, rid):
        t = time.time() - self.mock.t0
        return [10.0 * math.tanh(8.0 * self.mock.err(i, t)) for i in range(7)]

    def wrench_for(self, rid):
        t = time.time() - self.mock.t0
        return (0.4 + 0.3 * math.sin(t), -0.2, 2.1 + 0.6 * math.sin(t * 1.3), 0.02, 0.05, -0.01)

    def cartesian_error_for(self, rid):
        t = time.time() - self.mock.t0
        return (0.01 * math.sin(t), 0.006 * math.cos(t * 0.8), 0.012 * math.sin(t * 1.3), 0.02 * math.sin(t))

    def joint_velocity_for(self, rid):
        t = time.time() - self.mock.t0
        return [math.radians(1.4 * math.cos(t * 0.7 + i)) for i in range(7)]

    def joint_torque_for(self, rid):
        t = time.time() - self.mock.t0
        return [3.2 * math.sin(t * 0.4 + i * 0.6) for i in range(7)]

    def sim_joint_by_name(self, rid):
        return {}

    def contact_estimate_for(self, rid):
        return {'in_contact': False}

    def telemetry_health(self, rid):
        return (998.0, 1.2, 'live')


TOPICS = [('/robot1/joint_states', ['sensor_msgs/msg/JointState']), ('/robot1/ee_pose', ['geometry_msgs/msg/Pose']),
          ('/robot1/wrench', ['geometry_msgs/msg/WrenchStamped']), ('/robot1/command_mode', ['std_msgs/msg/String']),
          ('/robot2/joint_states', ['sensor_msgs/msg/JointState']), ('/rosout', ['rcl_interfaces/msg/Log']),
          ('/parameter_events', ['rcl_interfaces/msg/ParameterEvent']), ('/clock', ['rosgraph_msgs/msg/Clock'])]


class _Phase_B_Studio(_Phase_C):
    """Just enough Studio for the real phase-B command handlers to run against."""

    def __init__(self, mock):
        self.mock = mock
        self.robot_dropdown = _Combo(lambda: mock.robot)
        self._web_scene_provider = mock.scene          # the reach sphere for the Cartesian sliders
        arm = {'display_name': 'Kinova Gen3 7DOF', 'num_joints': 7, 'has_gripper': True, 'ee_frame_name': 'ee_link',
               'joint_limits_deg': {'lower': [l for l, _ in LIMITS], 'upper': [h for _, h in LIMITS]}}
        self.robot_configs = {'KinovaGen3_7DOF': arm, 'FrankaPanda': dict(arm, display_name='Franka Panda'),
                              'Unitree_Go2': {'display_name': 'Unitree Go2', 'robot_category': 'legged', 'num_joints': 12}}
        self.robots = {'KinovaGen3_7DOF': 7, 'FrankaPanda': 7, 'Unitree_Go2': 12}
        self.node = types.SimpleNamespace(station_id=1, station_robot_id='robot1')
        self.control_target = mock.target
        self.simulation_running = self.fic_running = True
        self.driver_running = self.controller_running = self.locomotion_running = False
        self.vr_relay_running = self.bags_running = False
        self.safety_enabled, self.safety_enforce, self.warning_threshold = True, True, 0.9
        self.safety_limits = {'cartesian': {'x': {'min': -0.8, 'max': 0.8, 'enabled': True},
                                            'y': {'min': -0.6, 'max': 0.6, 'enabled': True},
                                            'z': {'min': 0.05, 'max': 0.9, 'enabled': True}},
                              'joint_safety_deg': {'lower': [l * 0.9 for l, _ in LIMITS], 'upper': [h * 0.9 for _, h in LIMITS],
                                                   'enabled': True},
                              'velocity': {'max_joint_vel': 1.0, 'max_cart_vel': 0.5, 'enabled': True}}
        self.log_viewer = types.SimpleNamespace(_all_logs=mock.log, clear_logs=mock.log.clear)
        self._web_config_path = os.path.join(tempfile.mkdtemp(prefix='fic_mock_'), 'config.xml')
        shutil.copy(find_config_path(), self._web_config_path)
        self._fleet_dir = _Directory(mock)
        self._loco_velocity = (0.0, 0.0, 0.0)
        self._teleop = types.SimpleNamespace(sessions={}, request=lambda *a, **k: types.SimpleNamespace(sid='new'))
        self.param_updates = []
        self._rc_profile_dropdown = _ProfileCombo(mock, self)
        self._profiles_cache = (None, ['default'])
        self._init_c(mock)

    # state the real handlers read
    def _tb_session_running(self):
        return self.simulation_running or self.fic_running or self.driver_running or self.controller_running

    def _resolve_driver_command(self, robot):
        return 'driver'

    def _rc_command_rid(self):
        return 'robot1'

    def joint_positions_for(self, rid):
        return [math.radians(v) for v in self.mock.actual()]

    def ee_pose_for(self, rid):
        ns = types.SimpleNamespace
        x, y, z = self.mock.traj()[2]
        return ns(position=ns(x=x, y=y, z=z), orientation=ns(x=0, y=0, z=0, w=1))

    def log_message(self, level, message, source=''):
        self.mock.say(level, message, source)

    def _publish_safety_limits(self):
        pass

    def _publish_param_update(self, kind, value=''):
        self.param_updates.append((kind, value))
        self.mock.say('INFO', 'param_update %s %s' % (kind, value), 'Tuning')

    # processes
    def _flip(self, attr, on):
        setattr(self, attr, on)

    def _start_simulation(self): self._flip('simulation_running', True)
    def _stop_simulation(self): self._flip('simulation_running', False)
    def _start_fic_simulation(self): self._flip('fic_running', True)
    def _stop_fic_simulation(self): self._flip('fic_running', False)
    def _start_driver(self): self._flip('driver_running', True)
    def _stop_driver(self): self._flip('driver_running', False)
    def _start_controller(self): self._flip('controller_running', True)
    def _stop_controller(self): self._flip('controller_running', False)
    def _start_locomotion(self): self._flip('locomotion_running', True)
    def _stop_locomotion(self): self._flip('locomotion_running', False)
    def _start_vr(self): self._flip('vr_relay_running', True)
    def _stop_vr(self): self._flip('vr_relay_running', False)
    def _start_bags(self): self._flip('bags_running', True)
    def _stop_bags(self): self._flip('bags_running', False)

    def _tb_toggle_start(self):
        on = not self._tb_session_running()
        for attr in ('simulation_running', 'fic_running'):
            setattr(self, attr, on)

    # fleet and locomotion
    def profile_names(self):
        from fic_studio.xml_manager import XMLManager
        mtime = os.path.getmtime(self._web_config_path)
        if self._profiles_cache[0] != mtime:
            self._profiles_cache = (mtime, XMLManager(self._web_config_path).get_all_profiles(self.mock.robot) or ['default'])
        return self._profiles_cache[1]

    def _fleet_estop_all(self): self.mock.say('ERROR', 'E-STOP ALL sent', 'Fleet')
    def _fleet_pause_all(self): self.mock.say('WARN', 'PAUSE ALL sent', 'Fleet')
    def _fleet_session_send(self, n, op, target=None): self.mock.say('INFO', 'Asked station %d to %s %s' % (n, op, target or ''), 'Fleet')
    def _fleet_drive_now(self, n): self.mock.say('WARN', 'Control area now drives station %d' % n, 'Fleet')
    def _loco_send(self, c): self.mock.say('INFO', 'Gait: %s' % c, 'Locomotion')
    def _loco_set(self, k, v): self.mock.say('INFO', 'set %s %s' % (k, v), 'Locomotion')
    def _loco_stop(self): self._loco_velocity = (0.0, 0.0, 0.0)

    def _emergency_stop(self):
        pass



class _Trajectory:
    """The mock arm tracing a lemniscate, so the Dashboard has moving data to review.

    The target runs along a figure of eight in front of the arm; the commanded
    joints are its inverse kinematics (damped least squares on the Kinova model)
    and the measured arm follows the same path LAG seconds behind, so the
    tracking error is a few centimetres and the 3D overlays, plots and the
    joint tables all agree with one another. Stepped at most every 50 ms.
    """
    LAG, AMP, PERIOD = 0.30, 0.13, 14.0

    def __init__(self, q0_deg):
        import mujoco
        import numpy as np
        from fic_studio.web_remote import scene as scene_mod
        self.mj, self.np = mujoco, np
        self.model = scene_mod.load_model(SCENE_XML)
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'ee_link')
        self.adr = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'Actuator%d' % i)]
                    for i in range(1, 8)]
        self.dof = [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'Actuator%d' % i)]
                    for i in range(1, 8)]
        self.q0 = np.radians(q0_deg)
        self.home = self.fk(self.q0)
        self.q_cmd, self.q_act = self.q0.copy(), self.q0.copy()
        self.lock = threading.Lock()
        self.last = 0.0
        self.out = None
        for _ in range(60):                              # settle onto the path before the first sample
            self.q_cmd = self.ik(self.q_cmd, self.target(0.0))
        self.q_act = self.q_cmd.copy()

    def fk(self, q):
        for a, v in zip(self.adr, q):
            self.data.qpos[a] = v
        self.mj.mj_kinematics(self.model, self.data)
        return self.np.array(self.data.xpos[self.body])

    def target(self, t):
        u = 2 * math.pi * t / self.PERIOD
        d = 1.0 + math.sin(u) ** 2
        return self.home + self.np.array([0.0, self.AMP * math.cos(u) / d, self.AMP * math.sin(u) * math.cos(u) / d * 1.6])

    def ik(self, q, goal, steps=4):
        np = self.np
        for _ in range(steps):
            err = goal - self.fk(q)
            self.mj.mj_comPos(self.model, self.data)
            jac = np.zeros((3, self.model.nv))
            self.mj.mj_jacBody(self.model, self.data, jac, None, self.body)
            j = jac[:, self.dof]
            dq = j.T @ np.linalg.solve(j @ j.T + 1e-3 * np.eye(3), err)
            q = q + np.clip(dq, -0.15, 0.15)
        return q

    def step(self, t):
        with self.lock:
            if self.out is None or t - self.last > 0.05:
                self.last = t
                self.q_cmd = self.ik(self.q_cmd, self.target(t))
                self.q_act = self.ik(self.q_act, self.target(t - self.LAG))
                ee = self.fk(self.q_act)
                tgt = self.target(t)
                self.out = (list(self.q_cmd), list(self.q_act), [float(v) for v in ee], [float(v) for v in tgt])
            return self.out


class MockStudio:
    def __init__(self):
        self.t0 = time.time()
        self.mode = 0
        self.profile = 'default'
        self.target = 'simulation'
        self.robot = os.environ.get('MOCK_ROBOT', 'KinovaGen3_7DOF')
        self.commanded = [10.0, -25.0, 5.0, 70.0, 0.0, 45.0, 0.0]
        self.log = []
        self.calls = []
        self.estops = 0
        self.say('INFO', 'Mock Studio ready', 'System')
        for lv, src, msg in [
                ('INFO', 'Session', 'Starting simulation for KinovaGen3_7DOF'), ('INFO', 'Process', 'mujoco started, pid 48211'),
                ('DEBUG', 'Control', 'Mode select published: 1'), ('INFO', 'Process', 'fic_simulation started, pid 48240'),
                ('WARN', 'Safety', 'Safety limit exceeded: Z min = 0.041 (limit: 0.050)'),
                ('INFO', 'Control', 'Active config -> default (web remote)'),
                ('ERROR', 'Fleet', 'robot2: no answer to session request (timeout 5 s)'),
                ('INFO', 'Tuning', 'Tuning saved from the web UI: joint 0, Joint Impedance / default on KinovaGen3_7DOF')]:
            self.say(lv, msg, src)
        self.w = _Phase_B_Studio(self)

    def say(self, level, message, source='Mock'):
        self._seq = getattr(self, '_seq', 0) + 1
        self.log.append({'seq': self._seq, 'timestamp': time.strftime('%H:%M:%S'), 'time': time.strftime('%H:%M:%S'),
                         'level': level, 'source': source, 'message': message})

    def err(self, i, t):
        return 0.045 * math.sin(t * 0.9 + i * 0.7)

    def traj(self):
        if getattr(self, '_traj', None) is None:
            self._traj = _Trajectory(self.commanded)
        return self._traj.step(time.time() - self.t0)

    def actual(self):
        return [math.degrees(v) for v in self.traj()[1]]

    @property
    def running(self):
        return self.w._tb_session_running()

    @running.setter
    def running(self, _value):
        pass

    def snapshot(self):
        t = time.time() - self.t0
        actual = self.actual()
        kind = 'cart' if self.mode >= 4 else 'joint'
        joints = [{'label': LABELS[i], 'lo': LIMITS[i][0], 'hi': LIMITS[i][1], 'actuated': True,
                   'commanded': self.commanded[i] if kind == 'joint' else None, 'actual': actual[i],
                   'velocity': 1.4 * math.cos(t * 0.7 + i), 'torque': 3.2 * math.sin(t * 0.4 + i * 0.6)}
                  for i in range(7)]
        snap = {
            't': time.time(),
            'studio': {'app': 'FIC Control Studio', 'version': 'mock', 'host': 'robot-master', 'station': 1,
                       'robot': self.robot, 'robot_display': 'Kinova Gen3 7DOF', 'rid': 'robot1'},
            'robots': {'items': [{'name': 'KinovaGen3_7DOF', 'label': 'Kinova Gen3 7DOF'},
                                 {'name': 'FrankaPanda', 'label': 'Franka Panda'},
                                 {'name': 'Unitree_Go2', 'label': 'Unitree Go2'}], 'current': self.robot},
            'session': {'target': self.target, 'targets': ['simulation', 'hardware', 'vr'],
                        'running': self.running,
                        'processes': {'simulation': self.running, 'fic': self.running, 'driver': False,
                                      'controller': self.running, 'vr': False}, 'driver_configured': True},
            'mode': {'index': self.mode, 'name': MODES[kind]['modes'][self.mode % 4]['name'],
                     'kind': kind, 'menus': MODES},
            'options': {
                'admittance_planner': {'value': self.w._rc_admittance_planner_toggle.on, 'kind': 'joint', 'label': 'Admittance planner', 'tip': ''},
                'postural': {'value': self.w._rc_postural_toggle.on, 'kind': 'cart', 'label': 'Postural task', 'tip': ''},
                'elbow': {'value': self.w._rc_elbow_toggle.on, 'kind': 'cart', 'label': 'Elbow control', 'tip': ''},
                'hpmp': {'value': self.w._rc_hpmp_toggle.on, 'kind': 'cart', 'label': 'H-PMP', 'tip': ''}},
            'profiles': {'items': [{'name': n, 'label': n} for n in self.w.profile_names()],
                         'current': self.profile},
            'availability': {'command': True, 'joint': True, 'cart': True, 'gripper': True, 'home': True,
                             'sequences': True, 'reason': '', 'cart_reason': '', 'sequences_reason': ''},
            'joints': joints,
            'pose': {'actual': {'x': self.traj()[2][0], 'y': self.traj()[2][1], 'z': self.traj()[2][2], 'rx': 178.2,
                                'ry': 1.4, 'rz': 92.7, 'ax': 0, 'ay': 0, 'az': 1, 'angle': 0},
                     'commanded': ({'x': 0.45, 'y': 0.0, 'z': 0.4, 'ax': 0.0, 'ay': 1.0, 'az': 0.0, 'angle': 180.0} if kind == 'cart' else None),
                     'position_limit': 1.0, 'reach': self.reach()},
            'telemetry': {'health': {'state': 'live', 'rate': 998.0 + 4 * math.sin(t), 'age_ms': 1.2},
                          'wrench': [0.4 + 0.3 * math.sin(t), -0.2, 2.1 + 0.6 * math.sin(t * 1.3),
                                     0.02, 0.05, -0.01],
                          'contact': {'in_contact': False}},
            'gripper': {'available': True, 'percent': getattr(self, 'gripper', 35), 'max_width': 0.08,
                        'measured': 0.028, 'measured_age': 0.05},
            'status': {'text': 'Tracking commanded joints', 'contact': 'Free motion', 'playing': False},
            'sequences': {'available': True, 'items': ['pick', 'place', 'wave'], 'playing': False},
            'log': self.log[-25:],
            'theme': {'name': 'light', 'tokens': {}},
        }
        ctx = {'robot': self.robot, 'cfg': self.w.robot_configs.get(self.robot, {}), 'rid': 'robot1'}
        self.w.control_target = self.target
        snap['session'] = actions._session_state(self.w, ctx)
        snap['session']['target'] = self.target
        snap['log'] = actions._log_state(self.w, ctx)
        snap['safety'] = actions._safety_state(self.w, ctx)
        snap['fleet'] = actions._fleet_state(self.w, ctx)
        snap['locomotion'] = actions._locomotion_state(self.w, ctx)
        snap['teleop'] = {'available': True, 'station': 1, 'pending': [], 'sessions': [
            {'sid': 'ab12', 'leader': 12, 'follower': 11, 'state': 'active', 'scale': 1.0, 'reason': '',
             'holding': False, 'rtt': 0.004, 'role': '', 'from_host': 'robot2'}]}
        for key, build in actions._chrome.SECTIONS:
            snap[key] = build(self.w, ctx)
        snap['appearance'] = actions._appearance_state(self.w, ctx)
        snap['stations'] = actions._chrome.stations_state(self.w, ctx)
        snap['mode']['kind'] = kind
        snap['inner_loop'] = {'path': 'standard', 'state': '', 'detail': '', 'live': False, 'where': '', 'adapters': []}
        return snap

    DELEGATED = ('process', 'log_read', 'log_clear', 'safety_set', 'tuning_info', 'tuning_get', 'tuning_plot',
                 'tuning_save', 'tuning_trial', 'tuning_profile', 'tuning_xml', 'fleet', 'locomotion', 'teleop', 'toolbar', 'station', 'menu',
                 'robot_pause', 'robot_stop', 'live_publish', 'reset_defaults', 'preset', 'contact_truth', 'export',
                 'import_snapshot', 'recent_experiments', 'build', 'settings_get', 'settings_set', 'settings_reset',
                 'settings_extensions', 'settings_gamepad', 'settings_web', 'robots_list', 'robots_get', 'robots_save',
                 'robots_add', 'robots_duplicate', 'robots_delete', 'robots_import', 'robots_upload', 'robots_yaml', 'robots_report',
                 'robots_target_from_home', 'robots_import_yaml', 'robots_compile_driver', 'topics', 'help')

    def dispatch(self, action, args, client):
        self.calls.append((action, args))
        if action in self.DELEGATED:           # the mock is loopback-only, so it is the trusted desktop session
            return actions.handle_command(self.w, action, args, dict(client or {}, local=True))
        if action == 'estop':
            self.estops += 1
            self.say('ERROR', 'E-STOP from web remote')
        elif action == 'set_mode':
            self.mode = int(args['index'])
        elif action == 'set_profile':
            self.profile = args['name']
        elif action == 'set_target':
            self.target = args['target']
        elif action == 'set_robot':
            self.robot = args['name']
        elif action == 'gripper':
            self.gripper = float(args['percent'])
        elif action == 'set_option':
            self.w.__dict__['_rc_%s_toggle' % {'admittance_planner': 'admittance_planner'}.get(args['name'], args['name'])].setChecked(args['value'])
        elif action == 'send_pose':
            self.say('INFO', 'Pose target sent', 'Control')
        elif action == 'home':
            self.commanded = [0.0, 15.0, 180.0 - 180.0, 70.0, 0.0, 55.0, 0.0]
            self.say('INFO', 'Home sent', 'Control')
        elif action == 'send_joints':
            self.commanded = [float(v) for v in args['degrees']]
            self.say('INFO', 'Joint targets sent')
        elif action == 'session':
            self.w._tb_toggle_start()
            self.say('INFO', 'Session %s' % args.get('op'), 'Web')
        elif action == 'panel':
            return {'ok': True, 'panel': self.panel_act(args)}
        return {'ok': True}

    # ── Extensions: two made-up Qt pages, described as qtmirror would ──
    def panels(self, ext_id=None):
        listing = [{'id': 'sequences', 'label': 'Sequences', 'section': 'Extensions',
                    'description': 'Record waypoints and replay them.'},
                   {'id': 'gaze', 'label': 'Gaze teleop', 'section': 'Extensions',
                    'description': 'Eye tracking drives the camera.'}]
        if ext_id is None:
            return listing
        if ext_id not in ('sequences', 'gaze'):
            return None
        v = self.pvals
        nodes = {
            'sequences': [
                {'path': '0', 'kind': 'group', 'label': 'Playback', 'enabled': True, 'nodes': [
                    {'path': '0.0', 'kind': 'choice', 'label': 'Sequence', 'enabled': True,
                     'options': ['pick', 'place', 'wave'], 'value': v['seq']},
                    {'path': '0.1', 'kind': 'number', 'label': 'Speed', 'enabled': True, 'value': v['speed'],
                     'min': 0.1, 'max': 2.0, 'step': 0.1, 'suffix': ' x', 'decimals': 1},
                    {'path': '0.2', 'kind': 'toggle', 'label': 'Loop', 'enabled': True, 'value': v['loop']},
                    {'path': '0.3', 'kind': 'button', 'label': '\u25b6 PLAY', 'enabled': True},
                    {'path': '0.4', 'kind': 'button', 'label': '\u23f9 STOP', 'enabled': True}]},
                {'path': '1', 'kind': 'group', 'label': 'Capture', 'enabled': True, 'nodes': [
                    {'path': '1.0', 'kind': 'text', 'label': 'Name', 'enabled': True, 'value': v['name']},
                    {'path': '1.1', 'kind': 'button', 'label': 'Add waypoint', 'enabled': True},
                    {'path': '1.2', 'kind': 'label', 'label': '3 waypoints recorded', 'enabled': True}]},
                {'path': '2', 'kind': 'log', 'label': 'Output', 'enabled': True, 'value': 'loaded pick (3 waypoints)\nready'}],
            'gaze': [
                {'path': '0', 'kind': 'label', 'label': 'Tracker connected, 120 Hz', 'enabled': True},
                {'path': '1', 'kind': 'slider', 'label': 'Gain', 'enabled': True, 'value': 40, 'min': 0, 'max': 100},
                {'path': '3', 'kind': 'button', 'label': '\u25b6 START FOLLOWING \u2192', 'enabled': True},
                {'path': '2', 'kind': 'bar', 'label': 'Confidence %p%', 'enabled': True, 'value': 82, 'min': 0, 'max': 100}],
        }[ext_id]
        return {'id': ext_id, 'label': 'Sequences' if ext_id == 'sequences' else 'Gaze teleop',
                'sig': ext_id + str(len(nodes)), 'nodes': nodes}

    pvals = {'seq': 0, 'speed': 1.0, 'loop': False, 'name': 'pick'}

    def panel_act(self, args):
        path, kind, value = args.get('path'), args.get('kind'), args.get('value')
        key = {'0.0': 'seq', '0.1': 'speed', '0.2': 'loop', '1.0': 'name'}.get(path)
        if key:
            self.pvals[key] = value
        self.say('INFO', 'panel %s %s %r' % (path, kind, value), 'Extension')
        return self.panels(args.get('extension'))

    _scene = None
    _reach = None

    def reach(self):
        from fic_studio.web_remote import scene as scene_mod
        if MockStudio._reach is None:
            MockStudio._reach = scene_mod.reach(self.scene(None), 'ee_link') or {}
        return MockStudio._reach or None

    def scene(self, mesh_index):
        from fic_studio.web_remote import scene as scene_mod
        if MockStudio._scene is None:
            model = scene_mod.load_model(SCENE_XML)
            MockStudio._scene = (model, scene_mod.scene_dict(
                model, robot=self.robot, joint_names=['Actuator%d' % i for i in range(1, 8)]))
        if mesh_index is None:
            return MockStudio._scene[1]
        return scene_mod.mesh_bytes(MockStudio._scene[0], int(mesh_index))


def stub_import_pipeline():
    """The real importer needs pinocchio / MuJoCo and writes into the workspace; the mock reads the staged
    URDF's name and joints with a regex and invents the rest, so the drop, review and Create flow can be seen."""
    import re
    from fic_studio.robot_import import importer

    def find_urdf(source):
        if os.path.isfile(source):
            return source
        for root, _dirs, files in os.walk(source):
            for f in sorted(files):
                if f.lower().endswith(('.urdf', '.xacro')):
                    return os.path.join(root, f)
        return None

    def analyse(source, existing_ids=(), search_roots=(), urdf_path=None, mjcf_path=None):
        if os.path.isfile(source) and source.lower().endswith('.zip'):
            import zipfile
            tmp = tempfile.mkdtemp(prefix='mock_zip_')
            zipfile.ZipFile(source).extractall(tmp)
            source = tmp
        urdf = find_urdf(source)
        if not urdf:
            raise importer.ImportError_(importer.URDF_REQUIRED)
        text = open(urdf, errors='ignore').read()
        name = (re.search(r'<robot[^>]*name="([^"]+)"', text) or [None, os.path.basename(urdf)])[1]
        joints = re.findall(r'<joint[^>]*name="([^"]+)"[^>]*type="(?:revolute|continuous)"', text) or ['joint1']
        rid = importer.make_robot_id(name, existing_ids)
        return types.SimpleNamespace(
            robot_id=rid, display_name=name, urdf_path=urdf, ee_link='tool', elbow_link='link4',
            messages=[('ok', 'Mock analysis of %s' % os.path.basename(urdf))], errors=[], mesh_counts=[], closed_loop=False,
            urdf_candidates=[urdf], mjcf=mjcf_path, cleanup=lambda: None,
            joints=[types.SimpleNamespace(name=j, lower_deg=-180.0, upper_deg=180.0, effort=50.0, home_deg=0.0) for j in joints])

    def create_robot(plan, model_root, overwrite=False, progress=None):
        time.sleep(1.0)
        cfg = {'display_name': plan.display_name, 'num_joints': len(plan.joints), 'joint_names': [j.name for j in plan.joints]}
        return types.SimpleNamespace(robot_id=plan.robot_id, config=cfg, model_dir=os.path.join(model_root, plan.robot_id),
                                     messages=[('ok', 'Mock create')])
    importer.analyse, importer.create_robot = analyse, create_robot
    importer._what_was_found = lambda plan: [('Joints', str(len(plan.joints)), 'URDF'), ('End effector', plan.ee_link, 'guessed')]


def main(port=0, token=None, auth=None):
    stub_import_pipeline()
    mock = MockStudio()
    server = WebRemoteServer(dispatch=mock.dispatch, snapshot=mock.snapshot, lease=ControlLease(),
                             scene=mock.scene, panels=mock.panels,
                             host='127.0.0.1', local_token=token, auth=auth)
    bound = server.start(port)
    mock.w._web_event_sinks = [server]

    def pump():
        n = 0
        while True:
            n += 1
            if n % 40 == 0:
                mock.say('INFO', 'Heartbeat %d: telemetry %.0f Hz' % (n // 40, 998 + 4 * math.sin(n)), 'Telemetry')
            server.publish(mock.snapshot())
            time.sleep(0.1)
    threading.Thread(target=pump, daemon=True).start()
    return mock, server, bound


if __name__ == '__main__':
    _auth = None
    if os.environ.get('MOCK_AUTH'):        # MOCK_AUTH=user:password turns the remote login on (no MOCK_TOKEN: a remote client)
        from fic_studio.web_remote.accounts import Accounts
        from fic_studio.web_remote.auth import Auth
        _user, _pw = os.environ['MOCK_AUTH'].split(':', 1)
        _acc = Accounts(os.path.join(tempfile.mkdtemp(prefix='mockacc'), 'users.json'))
        _acc.set_user(_user, password=_pw, role='admin', pages=['control', 'view3d', 'dashboard', 'session', 'log', 'extensions'], sim_only=False)
        _auth = Auth(_acc)
    _, _, bound = main(int(sys.argv[1]) if len(sys.argv) > 1 else 0, os.environ.get('MOCK_TOKEN') or None, _auth)
    print('http://127.0.0.1:%d/ui/' % bound, flush=True)
    threading.Event().wait()
