"""simulation_launch.py — Launch the Newton-backed MuJoCo simulator.

Mirrors the interface of `mujoco_native/launch/simulation_launch.py` so the FIC
controller can swap backends with a single setting.

Usage:
    ros2 launch mujoco_newton simulation_launch.py robot_name:=Franka_FR3
    ros2 launch mujoco_newton simulation_launch.py robot_name:=MyArm model_file:=/abs/scene.xml

model_file overrides the built-in map; FIC Studio always passes it.
"""

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Same MJCF mapping mujoco_native uses (a subset) — keep these in sync.
_ROBOT_XML = {
    "KinovaGen3_7DOF": "model/KinovaGen3_7DOF/xml/scene.xml",
    "KinovaGen3_CSPM": "model/KinovaGen3_CSPM/xml/scene.xml",
    "Franka_FR3":      "model/Franka_FR3/xml/scene.xml",
    "IIWA_14":         "model/IIWA_14/XML/scene.xml",
    "Pendulum":        "model/Pendulum/xml/pendulum.xml",
    "CSPM":            "model/CSPM/xml/coaxial_spm.xml",
}


def _resolve_xml(context) -> str:
    robot_name = LaunchConfiguration("robot_name").perform(context)
    model_file = LaunchConfiguration("model_file").perform(context)
    if model_file:
        if not os.path.isfile(model_file):
            raise ValueError(f"model_file not found: {model_file}")
        return model_file
    if robot_name not in _ROBOT_XML:
        raise ValueError(
            f"Unknown robot name: {robot_name}. "
            f"Supported: {list(_ROBOT_XML.keys())}; for any other robot pass "
            f"model_file:=/path/to/scene.xml")
    return os.path.join(get_package_share_path("menagerie"), _ROBOT_XML[robot_name])


def _configure(context, *_, **__):
    xml_file = _resolve_xml(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true", "yes")
    device = LaunchConfiguration("device").perform(context)
    sid = LaunchConfiguration("station_id").perform(context)
    topic_prefix = f"robot{sid}_simulation"

    sim_node = Node(
        package="mujoco_newton",
        executable="simulation",
        name=f"simulation_mujoco_newton_{sid}",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "simulation/model_file": xml_file,
            "headless": headless,
            "device": device,
            "topic_prefix": topic_prefix,
            "initial_joint_positions":
                LaunchConfiguration("initial_joint_positions").perform(context).strip(),
        }],
    )
    return [sim_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value="Franka_FR3",
            description=f"One of {list(_ROBOT_XML.keys())}"),
        DeclareLaunchArgument(
            "station_id",
            default_value="1",
            description="Station ID (1-4): scopes simulation topics"),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Disable the Newton OpenGL viewer (useful over SSH)."),
        DeclareLaunchArgument(
            "device",
            default_value="cuda:0",
            description="Warp device for simulation (e.g. cuda:0, cpu)."),
        DeclareLaunchArgument(
            "model_file",
            default_value="",
            description="Absolute path to a MuJoCo scene; overrides robot_name's "
                        "built-in model"),
        DeclareLaunchArgument(
            "initial_joint_positions",
            default_value="",
            description="Start pose of a fixed-base arm: comma-separated angles "
                        "(rad) of its first joints; empty = all joints at 0"),
        OpaqueFunction(function=_configure),
    ])
