"""A simulated robot's joints come from the simulator's joint_states by name, not from trajectory_data.

trajectory_data's layout depends on the mode the controller is in: H-PMP writes the joint layout in a
Cartesian mode, and a mode changed elsewhere than the Studio's selector leaves the Studio parsing with
the wrong one. The 3D views drew the arm from that: turned over in H-PMP, jumping at a mode change."""
import types

from fic_studio.mixins.joint_state_mixin import JointStateMixin

NAMES = ['Actuator%d' % k for k in range(1, 8)]


class _Host:
    _on_sim_joint_states = JointStateMixin._on_sim_joint_states
    _sim_joint_order = JointStateMixin._sim_joint_order
    _on_trajectory_data = JointStateMixin._on_trajectory_data
    joint_positions_for = JointStateMixin.joint_positions_for
    _normalize_rid = JointStateMixin._normalize_rid

    def __init__(self):
        self.node = types.SimpleNamespace(station_robot_id='robot1')
        self.robot_configs = {'Franka': {'joint_names': ['fr3_joint%d' % k for k in range(1, 8)]},
                              'KinovaGen3_7DOF': {'joint_names': NAMES}}
        self.robot_dropdown = types.SimpleNamespace(currentText=lambda: 'KinovaGen3_7DOF')
        self.robots = {'KinovaGen3_7DOF': 7}
        self.current_sim_joint_by_name_by_robot = {}
        self.current_joint_positions_by_robot = {}
        self.current_joint_commanded_by_robot = {}
        self.current_joint_efforts_by_robot = {}
        self.current_cartesian_error_by_robot = {}
        self.current_joint_update_count_by_robot = {}
        self.current_joint_update_time_by_robot = {}
        self.current_joint_positions = []
        self._q_current_received = set()
        self._sim_q_time = {}
        self._sim_order_cache = {}
        self._rc_selected_mode_idx = 5             # the Studio shows mode 6


def _js(names, pos):
    return types.SimpleNamespace(name=list(names), position=list(pos), velocity=[0.0] * len(pos), effort=[0.0] * len(pos))


def test_joints_from_the_simulator_in_controller_order():
    h = _Host()
    names = ['finger'] + NAMES[::-1]                      # another order, and a gripper joint
    pos = [0.5] + [0.1 * k for k in range(7, 0, -1)]
    h._on_sim_joint_states('robot1', _js(names, pos))
    assert h.joint_positions_for('robot1') == [0.1 * k for k in range(1, 8)]
    assert h.current_joint_positions == h.joint_positions_for('robot1')


def test_a_prefixed_multi_robot_scene_matches():
    h = _Host()
    h._on_sim_joint_states('robot2', _js(['r2_' + n for n in NAMES], [0.2] * 7))
    assert h.joint_positions_for('robot2') == [0.2] * 7


def test_hpmp_trajectory_data_no_longer_moves_the_arm():
    h = _Host()
    h._on_sim_joint_states('robot1', _js(NAMES, [0.3] * 7))
    # H-PMP in mode 6: the joint layout (blocks of n) then the task block; parsed as Cartesian it was garbage
    traj = [9.0] * 7 + [0.3] * 7 + [0.01] * 7 + [0.3] * 7 + [0.0] * 28 + [0.0] * 16
    h._on_trajectory_data('robot1', types.SimpleNamespace(data=traj))
    assert h.joint_positions_for('robot1') == [0.3] * 7


def test_without_the_simulator_trajectory_data_still_serves():
    h = _Host()
    h._rc_selected_mode_idx = 0
    traj = [0.0] * 7 + [0.4] * 7 + [0.0] * 42
    h._on_trajectory_data('robot1', types.SimpleNamespace(data=traj))
    assert h.joint_positions_for('robot1') == [0.4] * 7


def test_a_real_arms_q_current_wins():
    h = _Host()
    h._q_current_received.add('robot1')
    h.current_joint_positions_by_robot['robot1'] = [0.7] * 7
    h._on_sim_joint_states('robot1', _js(NAMES, [0.3] * 7))
    assert h.joint_positions_for('robot1') == [0.7] * 7


def test_an_unknown_robot_is_left_to_trajectory_data():
    h = _Host()
    h._on_sim_joint_states('robot1', _js(['a', 'b'], [1.0, 2.0]))
    assert 'robot1' not in h.current_joint_positions_by_robot
