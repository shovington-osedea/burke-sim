"""Launch the Burk-e simulation and its opt-in fixed perimeter follower."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_paths() -> tuple[Path, Path, Path]:
    navigation_share = Path(get_package_share_directory('aircraft_navigation'))
    return (navigation_share / 'config' / 'aircraft_pose.yaml',
            navigation_share / 'config' / 'perimeter_path.yaml',
            navigation_share / 'config' / 'controller.yaml')


def _navigation_poses(
        pose_file: Path,
        path_file: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return the world spawn and spawn-relative odom aircraft poses."""
    pose_yaml = yaml.safe_load(pose_file.read_text())[
        'aircraft_frame_publisher']['ros__parameters']
    path_yaml = yaml.safe_load(path_file.read_text())['perimeter_path']
    points = path_yaml['waypoints']
    p0, p1 = points[0], points[1]
    aircraft_x = float(pose_yaml['x'])
    aircraft_y = float(pose_yaml['y'])
    aircraft_yaw = float(pose_yaml['yaw'])
    tangent = math.atan2(float(p1[1]) - float(p0[1]),
                         float(p1[0]) - float(p0[0]))
    cosine, sine = math.cos(aircraft_yaw), math.sin(aircraft_yaw)
    spawn_x = aircraft_x + cosine * float(p0[0]) - sine * float(p0[1])
    spawn_y = aircraft_y + sine * float(p0[0]) + cosine * float(p0[1])
    spawn_yaw = _normalise_angle(aircraft_yaw + tangent)

    # Gazebo DiffDrive initializes odom at the robot spawn rather than at the
    # Gazebo world origin.  Express the fixed world aircraft pose in that
    # spawn-relative odom frame so the ROS path remains on the physical model.
    delta_x = aircraft_x - spawn_x
    delta_y = aircraft_y - spawn_y
    spawn_cosine = math.cos(spawn_yaw)
    spawn_sine = math.sin(spawn_yaw)
    odom_aircraft_x = spawn_cosine * delta_x + spawn_sine * delta_y
    odom_aircraft_y = -spawn_sine * delta_x + spawn_cosine * delta_y
    odom_aircraft_yaw = _normalise_angle(aircraft_yaw - spawn_yaw)
    return ((spawn_x, spawn_y, spawn_yaw),
            (odom_aircraft_x, odom_aircraft_y, odom_aircraft_yaw))


def _normalise_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def generate_launch_description():
    """Build the dedicated navigation launch description."""
    pose_file, path_file, controller_file = _default_paths()
    spawn_pose, odom_aircraft_pose = _navigation_poses(pose_file, path_file)
    spawn_x, spawn_y, spawn_yaw = spawn_pose
    odom_aircraft_x, odom_aircraft_y, odom_aircraft_yaw = odom_aircraft_pose
    gazebo_share = Path(get_package_share_directory('burke_gazebo'))
    base_launch = gazebo_share / 'launch' / 'base_sim.launch.py'

    gui = LaunchConfiguration('gui')
    foxglove = LaunchConfiguration('foxglove')
    autostart = LaunchConfiguration('autostart')
    aircraft_frame = Node(
        package='aircraft_navigation', executable='aircraft_frame_publisher',
        name='aircraft_frame_publisher',
        parameters=[LaunchConfiguration('aircraft_pose_config'),
                    {'use_sim_time': True,
                     'x': odom_aircraft_x,
                     'y': odom_aircraft_y,
                     'yaw': odom_aircraft_yaw}],
        output='screen')
    path_publisher = Node(
        package='aircraft_navigation', executable='perimeter_path_publisher',
        name='perimeter_path_publisher',
        parameters=[{'use_sim_time': True},
                    {'path_file': LaunchConfiguration('path_config')}],
        output='screen')
    follower = Node(
        package='aircraft_navigation', executable='pure_pursuit_follower',
        name='pure_pursuit_follower',
        parameters=[LaunchConfiguration('controller_config'),
                    {'use_sim_time': True,
                     'enabled': ParameterValue(autostart, value_type=bool)}],
        output='screen')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description=(
                'Begin following automatically. If false, the follower waits '
                'for its set_enabled service.')),
        DeclareLaunchArgument('aircraft_pose_config', default_value=str(pose_file)),
        DeclareLaunchArgument('path_config', default_value=str(path_file)),
        DeclareLaunchArgument('controller_config', default_value=str(controller_file)),
        DeclareLaunchArgument('spawn_x', default_value=str(spawn_x)),
        DeclareLaunchArgument('spawn_y', default_value=str(spawn_y)),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value=str(spawn_yaw)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base_launch)),
            launch_arguments={
                'gui': gui, 'foxglove': foxglove, 'aircraft_frame': 'false',
                'spawn_x': LaunchConfiguration('spawn_x'),
                'spawn_y': LaunchConfiguration('spawn_y'),
                'spawn_z': LaunchConfiguration('spawn_z'),
                'spawn_yaw': LaunchConfiguration('spawn_yaw'),
            }.items()),
        aircraft_frame,
        path_publisher,
        RegisterEventHandler(OnProcessStart(
            target_action=path_publisher,
            on_start=[follower, rviz])),
    ])
