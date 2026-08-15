"""Bounded interface checks for the dedicated perimeter-follow launch."""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


TIMEOUT = 30.0
DEBUG_TOPICS = {
    '/path_follower/lookahead': 'geometry_msgs/msg/PointStamped',
    '/path_follower/closest_point': 'geometry_msgs/msg/PointStamped',
    '/path_follower/progress': 'std_msgs/msg/Float64',
    '/path_follower/cross_track_error': 'std_msgs/msg/Float64',
}


class InterfaceNode(Node):
    """Collect only public navigation interfaces used by the follower."""

    def __init__(self) -> None:
        super().__init__('navigation_interface_test')
        self.path: PathMessage | None = None
        self.odom: Odometry | None = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PathMessage, '/aircraft_path', self._path, qos)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _path(self, message: PathMessage) -> None:
        self.path = message

    def _odom(self, message: Odometry) -> None:
        self.odom = message


def _spin_until(node: InterfaceNode, predicate, description: str) -> None:
    deadline = time.monotonic() + TIMEOUT
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError(f'timed out waiting for {description}')


@pytest.fixture(scope='module')
def navigation_launch():
    package_root = Path(__file__).resolve().parents[2]
    log_directory = Path(tempfile.mkdtemp(prefix='burke_navigation_interfaces_', dir='/tmp'))
    log_file = (log_directory / 'launch.log').open('w')
    environment = os.environ.copy()
    environment['ROS_LOG_DIR'] = str(log_directory / 'ros')
    environment['GZ_SIM_LOG_DIR'] = str(log_directory / 'gazebo')
    environment['LIBGL_ALWAYS_SOFTWARE'] = 'true'
    process = subprocess.Popen(
        ['ros2', 'launch', 'aircraft_navigation', 'fixed_perimeter_follow.launch.py',
         'gui:=false', 'rviz:=false', 'foxglove:=false'],
        cwd=package_root, env=environment, stdout=log_file,
        stderr=subprocess.STDOUT, start_new_session=True, text=True)
    try:
        yield process
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5.0)
        log_file.close()


def test_navigation_interfaces(navigation_launch) -> None:
    assert navigation_launch.poll() is None
    rclpy.init()
    node = InterfaceNode()
    try:
        _spin_until(node, lambda: node.path is not None, '/aircraft_path')
        _spin_until(node, lambda: node.odom is not None, '/odom')
        _spin_until(node, lambda: _has_transform(node), 'odom TFs')
        assert node.path.header.frame_id == 'aircraft'
        assert len(node.path.poses) >= 3
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in node.path.poses]
        aircraft_tf = node.tf_buffer.lookup_transform('odom', 'aircraft', Time())
        aircraft_yaw = _yaw(aircraft_tf.transform.rotation)
        cosine, sine = math.cos(aircraft_yaw), math.sin(aircraft_yaw)
        p0_odom = (
            aircraft_tf.transform.translation.x
            + cosine * points[0][0] - sine * points[0][1],
            aircraft_tf.transform.translation.y
            + sine * points[0][0] + cosine * points[0][1])
        assert p0_odom == pytest.approx((0.0, 0.0), abs=1e-6)
        tangent = math.atan2(
            points[1][1] - points[0][1], points[1][0] - points[0][0])
        assert _angle_difference(aircraft_yaw + tangent, 0.0) == pytest.approx(
            0.0, abs=1e-9)
        area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1)
                   in zip(points, points[1:] + points[:1])) / 2.0
        assert area < 0.0, 'path is not clockwise'
        assert points[0][0] == pytest.approx(max(x for x, _ in points), abs=0.2)
        assert _publisher_count(node, '/cmd_vel', 'geometry_msgs/msg/Twist') == 1
        for topic, message_type in DEBUG_TOPICS.items():
            assert _publisher_count(node, topic, message_type) == 1
        assert not _ground_truth_reference_exists()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _has_transform(node: InterfaceNode) -> bool:
    try:
        node.tf_buffer.lookup_transform('odom', 'base_footprint', Time())
        node.tf_buffer.lookup_transform('odom', 'aircraft', Time())
        return True
    except TransformException:
        return False


def _yaw(orientation) -> float:
    return math.atan2(2.0 * orientation.w * orientation.z,
                      1.0 - 2.0 * orientation.z * orientation.z)


def _angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _publisher_count(node: Node, topic: str, message_type: str) -> int:
    return sum(info.topic_type == message_type
               for info in node.get_publishers_info_by_topic(topic))


def _ground_truth_reference_exists() -> bool:
    source_root = Path(__file__).parents[1] / 'aircraft_navigation'
    source = '\n'.join(path.read_text() for path in source_root.rglob('*.py'))
    return 'model_states' in source or 'gazebo/model' in source
