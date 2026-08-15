"""Bounded, read-only verification of the wheel odometry TF contract."""

import math
from typing import Optional

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class OdometryBaselineCheck(Node):
    """Compare recent odometry messages with the odom-to-base TF transform."""

    def __init__(self) -> None:
        super().__init__('odometry_baseline_check')
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._latest_odom: Optional[Odometry] = None
        self._odom_subscription = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

    def _odom_callback(self, message: Odometry) -> None:
        self._latest_odom = message

    def check(self, timeout_s: float = 5.0) -> bool:
        """Wait briefly for matching odometry and TF, then report the result."""
        deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_odom is None:
                continue
            odom = self._latest_odom
            if (odom.header.frame_id != 'odom'
                    or odom.child_frame_id != 'base_footprint'):
                self.get_logger().error(
                    f'Expected /odom frame contract odom -> base_footprint, '
                    f'got {odom.header.frame_id} -> {odom.child_frame_id}')
                return False
            try:
                transform = self._buffer.lookup_transform(
                    'odom', 'base_footprint', rclpy.time.Time(),
                    timeout=Duration(seconds=0.2))
            except TransformException:
                continue
            if self._matches(odom, transform):
                self.get_logger().info(
                    'Verified /odom odom -> base_footprint matches TF '
                    'translation and yaw.')
                return True
            self.get_logger().error(
                'The latest /odom pose does not match odom -> base_footprint TF.')
            return False
        self.get_logger().error(
            f'Timed out after {timeout_s:.1f} s waiting for /odom and '
            'odom -> base_footprint TF.')
        return False

    @staticmethod
    def _matches(odom: Odometry, transform: TransformStamped, tolerance: float = 1e-3) -> bool:
        """Return whether translation and planar yaw agree within tolerance."""
        odom_translation = odom.pose.pose.position
        tf_translation = transform.transform.translation
        if any(abs(a - b) > tolerance for a, b in (
                (odom_translation.x, tf_translation.x),
                (odom_translation.y, tf_translation.y),
                (odom_translation.z, tf_translation.z))):
            return False
        odom_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        tf_yaw = _yaw_from_quaternion(transform.transform.rotation)
        return abs(_normalize_angle(odom_yaw - tf_yaw)) <= tolerance


def _yaw_from_quaternion(quaternion) -> float:
    """Extract planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> int:
    """Run the bounded baseline check and return a process status."""
    rclpy.init(args=args)
    node = OdometryBaselineCheck()
    try:
        return 0 if node.check() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
