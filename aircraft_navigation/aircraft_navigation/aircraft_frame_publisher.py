"""Publish the configured aircraft frame relative to wheel odometry."""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class AircraftFramePublisher(Node):
    """Publish a configurable static ``parent_frame -> aircraft`` transform."""

    def __init__(self) -> None:
        super().__init__('aircraft_frame_publisher')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'aircraft')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 12.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('yaw', math.pi)

        transform = make_aircraft_transform(
            parent_frame=str(self.get_parameter('parent_frame').value),
            child_frame=str(self.get_parameter('child_frame').value),
            x=float(self.get_parameter('x').value),
            y=float(self.get_parameter('y').value),
            z=float(self.get_parameter('z').value),
            yaw=float(self.get_parameter('yaw').value),
        )
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transform)
        self.get_logger().info(
            f'Published {transform.header.frame_id} -> '
            f'{transform.child_frame_id} at '
            f'({transform.transform.translation.x:.3f}, '
            f'{transform.transform.translation.y:.3f}, '
            f'{transform.transform.translation.z:.3f}), '
            f'yaw {float(self.get_parameter("yaw").value):.6f} rad.')


def make_aircraft_transform(
        parent_frame: str,
        child_frame: str,
        x: float,
        y: float,
        z: float,
        yaw: float) -> TransformStamped:
    """Build a planar transform from validated aircraft pose parameters."""
    if not parent_frame or not child_frame:
        raise ValueError('parent_frame and child_frame must not be empty')
    if parent_frame == child_frame:
        raise ValueError('parent_frame and child_frame must be different')
    values = (x, y, z, yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('aircraft pose values must be finite')

    half_yaw = yaw / 2.0
    transform = TransformStamped()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = z
    transform.transform.rotation.z = math.sin(half_yaw)
    transform.transform.rotation.w = math.cos(half_yaw)
    return transform


def main(args=None) -> None:
    """Run the static aircraft frame publisher until shutdown."""
    rclpy.init(args=args)
    node = AircraftFramePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
