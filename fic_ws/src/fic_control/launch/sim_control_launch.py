"""
sim_control_launch.py — the FIC controller of a simulated arm (no simulator: start mujoco_native's
simulation_launch.py for MuJoCo). robot_control_launch.py is the same controller for a real robot.

Starts the shared FIC node (fic_control's executable with target:=simulation, named fic_simulation_node_<N>)
and the station's joint inner loop (fic_inner_loop --adapter sim on 127.0.0.1:simInnerLoopPort(station), fic_link.h), which
mujoco_native steps in lockstep. For a simulator without lockstep (plant: topics) the node feeds the inner loop
from its joint_states instead.

Usage:
    ros2 launch fic_control sim_control_launch.py robot_name:=Franka_FR3
    ros2 launch fic_control sim_control_launch.py robot_name:=KinovaGen3_7DOF station_id:=2
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def _macos_dyld_env():
    """macOS: `ros2 launch` does NOT propagate DYLD_LIBRARY_PATH to the process it spawns, so the C++ node fails
    to dlopen its ROS message and pinocchio dylibs. Rebuild the loader path from AMENT_PREFIX_PATH. No-op on Linux."""
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
    args = [
        DeclareLaunchArgument("robot_name", default_value="KinovaGen3_7DOF",
                              description="Robot to control (KinovaGen3_7DOF, Franka_FR3, IIWA_14, Pendulum, ...)"),
        DeclareLaunchArgument("num_joints", default_value="7",
                              description="Unused: the joint count comes from the robot's configuration"),
        DeclareLaunchArgument("station_id", default_value="1",
                              description="Station ID: the robotN topics and the inner loop's port (fic_link.h simInnerLoopPort)"),
        DeclareLaunchArgument("joint_name_prefix", default_value="",
                              description="Prefix of the simulator's joint/actuator names (multi-robot scenes)"),
        DeclareLaunchArgument("source_prefix", default_value="simulation",
                              description="The simulator's topic prefix (robotN_simulation)"),
        DeclareLaunchArgument("plant", default_value="auto",
                              description="lockstep (mujoco_native) | topics (Newton) | auto"),
        DeclareLaunchArgument("inner_loop", default_value="true",
                              description="Start the station's fic_inner_loop --adapter sim"),
    ]
    sid = LaunchConfiguration("station_id")
    inner_loop = ExecuteProcess(
        cmd=[PathJoinSubstitution([FindPackagePrefix("fic_control"), "lib", "fic_control", "fic_inner_loop"]),
             "--adapter", "sim", "--station", sid],
        output="screen",
        condition=IfCondition(LaunchConfiguration("inner_loop")),
    )
    node = Node(
        package="fic_control",
        executable="fic_control",
        name=PythonExpression(["'fic_simulation_node_' + str(", sid, ")"]),
        output="screen",
        parameters=[{
            "target": "simulation",
            "selected_robot": LaunchConfiguration("robot_name"),
            "station_id": sid,
            "joint_name_prefix": LaunchConfiguration("joint_name_prefix"),
            "source_prefix": LaunchConfiguration("source_prefix"),
            "plant": LaunchConfiguration("plant"),
        }],
        emulate_tty=True,
        additional_env=_macos_dyld_env(),
        arguments=["--ros-args", "--log-level", "info"],
    )
    return LaunchDescription(args + [inner_loop, node])
