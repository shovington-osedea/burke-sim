"""Launch the empty world with one primitive Burk-e base."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = Path(get_package_share_directory("burke_gazebo"))
    description_share = Path(get_package_share_directory("burke_description"))
    world_launch = gazebo_share / "launch" / "empty_world.launch.py"
    bridge_config = gazebo_share / "config" / "bridge.yaml"
    xacro_file = description_share / "urdf" / "burke_base.urdf.xacro"

    robot_description = ParameterValue(Command(["xacro ", str(xacro_file)]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo's graphical client.",
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
    ])
