"""Launch the deterministic empty Burk-e Gazebo world."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = str(
        Path(get_package_share_directory("burke_gazebo")) / "worlds" / "empty.sdf"
    )
    gz_launch = Path(
        get_package_share_directory("ros_gz_sim")
    ) / "launch" / "gz_sim.launch.py"
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")

    return LaunchDescription(
        [
            # Simulation default: software OpenGL avoids renderer startup
            # failures on VMs and systems without a usable GPU context.
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "true"),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start Gazebo's graphical client (set false for headless startup).",
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value="ogre",
                description="Gazebo render engine (ogre is the supported project default).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_launch)),
                launch_arguments={
                    "gz_args": ["-r --render-engine ", render_engine, " ", world]
                }.items(),
                condition=IfCondition(gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gz_launch)),
                launch_arguments={
                    "gz_args": ["-r -s --render-engine ", render_engine, " ", world]
                }.items(),
                condition=UnlessCondition(gui),
            ),
        ]
    )
