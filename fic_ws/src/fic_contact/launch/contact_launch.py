"""Launch the contact localization node alongside the hardware stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start contact_node for one station."""
    return LaunchDescription([
        DeclareLaunchArgument('station_id', default_value='1'),
        DeclareLaunchArgument('robot_name', default_value='KinovaGen3_7DOF'),
        Node(
            package='fic_contact',
            executable='contact_node',
            name=['contact_localization_', LaunchConfiguration('station_id')],
            # 'both', not 'screen': FIC Studio starts the hardware launch
            # with stdout and stderr on DEVNULL (process_mixin
            # _start_controller), so a screen-only node leaves no trace of
            # why estimation is gated. 'both' also writes ~/.ros/log.
            output='both',
            parameters=[{
                'station_id': LaunchConfiguration('station_id'),
                'robot_name': LaunchConfiguration('robot_name'),
                # No 'workspace' override here: ContactNode resolves its own
                # default from this module's location (see contact_node.py's
                # _resolve_repo_root / _DEFAULT_WORKSPACE), not the cwd of
                # whatever launched this file -- see FIX 1's history.
            }],
        ),
    ])
