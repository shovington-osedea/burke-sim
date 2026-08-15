"""Publish the validated fixed aircraft-relative perimeter path."""

import math
from pathlib import Path

from aircraft_navigation.path_geometry import (
    PolygonBoundary, rounded_offset_polygon, validate_path)
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


class PerimeterPathPublisher(Node):
    """Load, validate, and publish a transient-local fixed path."""

    def __init__(self) -> None:
        super().__init__('perimeter_path_publisher')
        default_path = (
            Path(get_package_share_directory('aircraft_navigation'))
            / 'config' / 'perimeter_path.yaml')
        self.declare_parameter('path_file', str(default_path))
        path_file = Path(str(self.get_parameter('path_file').value))
        config = yaml.safe_load(path_file.read_text())['perimeter_path']
        self._frame_id = str(config['frame_id'])
        footprint = tuple(
            tuple(float(value) for value in point) for point in config['footprint'])
        boundary = PolygonBoundary(footprint)
        points = [
            tuple(float(value) for value in point)
            for point in config.get('waypoints', [])]
        if not points:
            points = rounded_offset_polygon(
                footprint,
                radius_m=float(config['construction_radius_m']),
                maximum_segment_length=float(config['maximum_pose_spacing_m']))
        metrics = validate_path(
            points,
            boundary,
            float(config['clearance_m']),
            maximum_clearance_m=float(config['maximum_clearance_m']),
            maximum_segment_length_m=float(config['maximum_pose_spacing_m']))
        self._publisher = self.create_publisher(
            PathMessage,
            '/aircraft_path',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._message = self._make_message(points)
        self._publisher.publish(self._message)
        self.get_logger().info(
            f'Published {len(points)} clockwise waypoints in {self._frame_id}; '
            f'length {metrics["total_length_m"]:.3f} m, '
            f'clearance range '
            f'{metrics["minimum_clearance_m"]:.3f}..'
            f'{metrics["maximum_clearance_m"]:.3f} m, '
            f'maximum pose spacing {metrics["maximum_pose_spacing_m"]:.3f} m.')

    def _make_message(self, points: list[tuple[float, float]]) -> PathMessage:
        """Build a planar Path message from aircraft-frame points."""
        message = PathMessage()
        message.header.frame_id = self._frame_id
        stamp = self.get_clock().now().to_msg()
        message.header.stamp = stamp
        for index, (x, y) in enumerate(points):
            next_x, next_y = points[(index + 1) % len(points)]
            yaw = planar_yaw((x, y), (next_x, next_y))
            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)
        return message


def planar_yaw(start: tuple[float, float], end: tuple[float, float]) -> float:
    """Return the planar heading from one waypoint to the next."""
    if start == end:
        raise ValueError('waypoint orientation requires distinct consecutive points')
    return math.atan2(end[1] - start[1], end[0] - start[0])


def main(args=None) -> None:
    """Run the fixed path publisher until shutdown."""
    rclpy.init(args=args)
    node = PerimeterPathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
