"""
fic_launch.py — Launch file for FIC Controller Node only

This launches just the FIC controller node without MuJoCo simulation.
Use simulation_launch.py separately to start MuJoCo.

Usage:
    ros2 launch fic_simulation fic_launch.py robot_name:=Franka_FR3
    ros2 launch fic_simulation fic_launch.py robot_name:=Franka_FR3 num_joints:=7
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def _macos_dyld_env():
    """macOS: `ros2 launch` does NOT propagate DYLD_LIBRARY_PATH to the process
    it spawns, so the C++ fic_simulation node fails to dlopen its ROS message
    (`communication`) + pinocchio dylibs and crashes at startup (= "no control").
    AMENT_PREFIX_PATH survives, so rebuild the loader path from it and inject it
    explicitly. No-op on Linux (which uses LD_LIBRARY_PATH and already works)."""
    if sys.platform != "darwin":
        return {}
    parts = []
    existing = os.environ.get("DYLD_LIBRARY_PATH", "")
    if existing:
        parts.append(existing)
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        libdir = os.path.join(prefix, "lib")
        if prefix and os.path.isdir(libdir) and libdir not in parts:
            parts.append(libdir)
    conda = os.environ.get("CONDA_PREFIX")
    if conda:
        parts.append(os.path.join(conda, "lib"))
    return {"DYLD_LIBRARY_PATH": os.pathsep.join(parts)} if parts else {}


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="KinovaGen3_7DOF",
        description="Name of the robot to control (KinovaGen3_7DOF, Franka_FR3, IIWA_14, Pendulum)"
    )
    
    num_joints_arg = DeclareLaunchArgument(
        "num_joints",
        default_value="7",
        description="Number of joints for the robot"
    )

    station_id_arg = DeclareLaunchArgument(
        "station_id",
        default_value="1",
        description="Station ID (1-4): determines which robotN topics to use"
    )

    joint_name_prefix_arg = DeclareLaunchArgument(
        "joint_name_prefix",
        default_value="",
        description="Prefix for MuJoCo joint/actuator names in multi-robot scenes"
    )

    source_prefix_arg = DeclareLaunchArgument(
        "source_prefix",
        default_value="simulation",
        description="Topic prefix for data source: 'simulation' (MuJoCo desktop) or 'vr' (VR headset)"
    )

    # FIC Controller Node — unique name per station so multiple instances coexist.
    node_name = PythonExpression([
        "'fic_simulation_node_' + str(", LaunchConfiguration("station_id"), ")"
    ])
    fic_node = Node(
        package='fic_simulation',
        executable='fic_simulation',
        name=node_name,
        output='screen',
        parameters=[{
            "selected_robot": LaunchConfiguration("robot_name"),
            "number_of_joints": LaunchConfiguration("num_joints"),
            "station_id": LaunchConfiguration("station_id"),
            "joint_name_prefix": LaunchConfiguration("joint_name_prefix"),
            "source_prefix": LaunchConfiguration("source_prefix"),
        }],
        emulate_tty=True,
        additional_env=_macos_dyld_env(),  # macOS: restore DYLD so the C++ node loads
        arguments=["--ros-args", "--log-level", "info"]
    )

    return LaunchDescription([
        robot_name_arg,
        num_joints_arg,
        station_id_arg,
        joint_name_prefix_arg,
        source_prefix_arg,
        fic_node
    ])