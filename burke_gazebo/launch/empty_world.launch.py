"""Launch the deterministic empty Burk-e Gazebo world."""

import os
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
    description_share = Path(get_package_share_directory("burke_description"))
    gazebo_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_roots = [str(description_share.parent)]
    if gazebo_resource_path:
        resource_roots.append(gazebo_resource_path)
    gz_launch = Path(
        get_package_share_directory("ros_gz_sim")
    ) / "launch" / "gz_sim.launch.py"
    gui = LaunchConfiguration("gui")
    render_engine = LaunchConfiguration("render_engine")
    software_rendering = LaunchConfiguration("software_rendering")

    return LaunchDescription(
        [
            # The world uses model://burke_description/... for the aircraft
            # mesh, so make the installed package parent discoverable for both
            # direct empty-world launches and nested base-simulation launches.
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.pathsep.join(resource_roots),
            ),
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
            DeclareLaunchArgument(
                "software_rendering",
                default_value="true",
                description="Use software OpenGL for reliable rendering in VMs.",
            ),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                "true",
                condition=IfCondition(software_rendering),
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
