import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='Franka_FR3',
        description='Name of the robot to control (e.g., Franka_FR3, KinovaGen3_7DOF)'
    )

    station_id_arg = DeclareLaunchArgument(
        'station_id',
        default_value='1',
        description='Station ID (1-4): determines which robotN topics to use'
    )

    # Contact localization rides along with the hardware stack: it consumes
    # {rid}_joint_torques, which only fic_control publishes, and nothing
    # else starts it. Without it {rid}_contact_estimate has no publisher at
    # all, so the MuJoCo twin mirrors the pose and never highlights.
    contact_arg = DeclareLaunchArgument(
        'contact',
        default_value='true',
        description='Start the contact localization node alongside the arm'
    )

    # The joint inner loop as a process on this PC (fic_inner_loop, on the vendor driver's shared memory):
    # fic_control drives the arm only through an inner loop, this one or an MCU board. false: an MCU drives
    # every robot here, or another fic_inner_loop already runs (one per PC: the shared memory is one segment).
    inner_loop_arg = DeclareLaunchArgument(
        'inner_loop',
        default_value='true',
        description='Start fic_inner_loop (the PC joint inner loop) with the controller'
    )
    robot_name = LaunchConfiguration('robot_name')
    station_id = LaunchConfiguration('station_id')

    contact_launch = os.path.join(
        get_package_share_directory('fic_contact'), 'launch',
        'contact_launch.py')

    return LaunchDescription([
        robot_name_arg,
        station_id_arg,
        contact_arg,
        inner_loop_arg,

        ExecuteProcess(
            cmd=[PathJoinSubstitution([FindPackagePrefix('fic_control'), 'lib', 'fic_control', 'fic_inner_loop']),
                 '--rt'],
            output='both',
            condition=IfCondition(LaunchConfiguration('inner_loop')),
        ),

        Node(
            package='fic_control',
            executable='fic_control',
            # Per station, the same way contact_launch.py names
            # contact_localization_N and the simulation launches name
            # simulation_mujoco_native_N. The whole lab shares one ROS domain
            # (LAB_ROS_DOMAIN_ID=10), so a fixed name put two identically
            # named nodes on one graph whenever two arms ran at once, and
            # every /fic_hardware_node/*_parameters service then existed
            # twice. Topics were never affected: they are all robotN_.
            name=['fic_hardware_node_', station_id],
            # 'both', not 'screen': FIC Studio starts this launch with
            # stdout and stderr on DEVNULL (process_mixin
            # _start_controller), so 'screen' discards every log line this
            # node emits. That is how a silently-dropped Cartesian target
            # stayed undiagnosable -- targetPositionCallback's first line
            # is `if (!is_robot_supported_) return;`, and its IK progress
            # and 'Cartesian target ignored' warning all went nowhere.
            # 'both' also writes ~/.ros/log. Same fix, same reason, as
            # contact_launch.py.
            output='both',
            parameters=[
                {'selected_robot': robot_name},
                {'station_id': station_id},
            ]
        ),

        # Both arguments must be forwarded: contact_launch.py defaults to
        # station 1 and KinovaGen3_7DOF, which would silently estimate for
        # the wrong arm on any other bench.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(contact_launch),
            condition=IfCondition(LaunchConfiguration('contact')),
            launch_arguments={
                'station_id': station_id,
                'robot_name': robot_name,
            }.items(),
        ),
    ])
