"""Launch the Burk-e simulation with its default read-only Foxglove bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = Path(get_package_share_directory("burke_gazebo"))
    base_launch = package_share / "launch" / "base_sim.launch.py"
    return LaunchDescription([
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
            default_value=str(package_share / "config" / "foxglove_bridge.yaml"),
            description="Foxglove bridge ROS parameter file.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base_launch)),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "foxglove": LaunchConfiguration("foxglove"),
                "foxglove_address": LaunchConfiguration("foxglove_address"),
                "foxglove_port": LaunchConfiguration("foxglove_port"),
                "foxglove_config": LaunchConfiguration("foxglove_config"),
            }.items(),
        ),
    ])
