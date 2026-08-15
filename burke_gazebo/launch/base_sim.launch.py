"""Launch the empty world with one CAD-visualized Burk-e base."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = Path(get_package_share_directory("burke_gazebo"))
    description_share = Path(get_package_share_directory("burke_description"))
    # ros_gz_sim resolves URDF package:// meshes as model:// URIs. Gazebo must
    # therefore search the installed share parent for model/burke_description.
    gazebo_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_roots = [str(description_share.parent)]
    if gazebo_resource_path:
        resource_roots.append(gazebo_resource_path)
    world_launch = gazebo_share / "launch" / "empty_world.launch.py"
    bridge_config = gazebo_share / "config" / "bridge.yaml"
    foxglove_config = gazebo_share / "config" / "foxglove_bridge.yaml"
    xacro_file = description_share / "urdf" / "burke_base.urdf.xacro"

    robot_description = ParameterValue(Command(["xacro ", str(xacro_file)]), value_type=str)

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.pathsep.join(resource_roots),
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo's graphical client.",
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="true",
            description="Start the read-only Foxglove WebSocket bridge.",
        ),
        DeclareLaunchArgument(
            "foxglove_address",
            default_value="0.0.0.0",
            description="Foxglove WebSocket bind address.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove WebSocket TCP port.",
        ),
        DeclareLaunchArgument(
            "foxglove_config",
            default_value=str(foxglove_config),
            description="Foxglove bridge ROS parameter file.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(world_launch)),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": True,
                }
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_burke_base",
            arguments=["-topic", "robot_description", "-name", "burke_base", "-x", "0", "-y", "0", "-z", "0"],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            parameters=[{"config_file": str(bridge_config)}],
            output="screen",
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            condition=IfCondition(LaunchConfiguration("foxglove")),
            parameters=[
                LaunchConfiguration("foxglove_config"),
                {
                    "address": LaunchConfiguration("foxglove_address"),
                    "port": ParameterValue(
                        LaunchConfiguration("foxglove_port"),
                        value_type=int,
                    ),
                },
            ],
            output="screen",
        ),
    ])
