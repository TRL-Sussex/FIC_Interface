"""
Launch the locomotion controller for a legged robot.

Usage:
    ros2 launch fic_locomotion locomotion_launch.py robot_name:=Spot station_id:=1
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _macos_dyld_env():
    """
    Restore DYLD_LIBRARY_PATH on macOS so rosidl typesupport libs load.

    `ros2 launch` does NOT propagate DYLD_LIBRARY_PATH to the process it
    spawns, so the rosidl Python typesupport libraries fail to dlopen and the
    node dies at its first subscription. AMENT_PREFIX_PATH survives, so
    rebuild the loader path from it and inject it explicitly. No-op on Linux
    (which uses LD_LIBRARY_PATH and already works).

    Deliberately duplicated from fic_control/launch/sim_control_launch.py rather
    than shared: sourcing it across ROS packages would mean a package
    dependency for fifteen lines, and every launch file in this repo already
    carries its own copy. Self-guarding on sys.platform so a caller cannot
    forget the check.
    """
    if sys.platform != "darwin":
        return {}
    parts = []
    existing = os.environ.get('DYLD_LIBRARY_PATH', '')
    if existing:
        parts.append(existing)
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        libdir = os.path.join(prefix, 'lib')
        if prefix and os.path.isdir(libdir) and libdir not in parts:
            parts.append(libdir)
    conda = os.environ.get('CONDA_PREFIX')
    if conda:
        parts.append(os.path.join(conda, 'lib'))
    return {'DYLD_LIBRARY_PATH': os.pathsep.join(parts)} if parts else {}


def _configure(context, *_, **__):
    sid = LaunchConfiguration('station_id').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    source_prefix = LaunchConfiguration('source_prefix').perform(context)
    base_state_topic = LaunchConfiguration('base_state_topic').perform(context)
    return [Node(
        package='fic_locomotion',
        executable='locomotion',
        name='locomotion_node_%s' % sid,
        output='screen',
        emulate_tty=True,
        additional_env=_macos_dyld_env(),  # macOS: restore DYLD for typesupport
        parameters=[{
            'station_id': int(sid),
            'robot_name': robot_name,
            'source_prefix': source_prefix,
            'base_state_topic': base_state_topic,
        }],
    )]


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name', default_value='Spot',
            description='Legged robot to control'),
        DeclareLaunchArgument(
            'station_id', default_value='1',
            description='Station ID (1-4): scopes the robot{N}_* topics'),
        DeclareLaunchArgument(
            'source_prefix', default_value='simulation',
            description=('Where measured state comes from: "simulation" '
                         '(default) or "hardware". "hardware" has no '
                         'base-state source, so the node will refuse to '
                         'command unless base_state_topic is also set -- see '
                         'docs/spot-locomotion-hardware.md')),
        DeclareLaunchArgument(
            'base_state_topic', default_value='',
            description=('nav_msgs/Odometry topic carrying trunk pose and '
                         'twist. Empty means "derive from source_prefix", '
                         'which yields nothing on the hardware path')),
        OpaqueFunction(function=_configure),
    ])
