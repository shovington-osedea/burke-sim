from math import isclose, pi, sin, cos

from aircraft_navigation.odometry_baseline_check import (
    OdometryBaselineCheck,
    _normalize_angle,
    _yaw_from_quaternion,
)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def _pose(x, y, yaw):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = sin(yaw / 2.0)
    message.pose.pose.orientation.w = cos(yaw / 2.0)
    return message


def test_frame_contract_and_matching_pose():
    odom = _pose(1.0, -2.0, 0.4)
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    transform = TransformStamped()
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = -2.0
    transform.transform.rotation.z = odom.pose.pose.orientation.z
    transform.transform.rotation.w = odom.pose.pose.orientation.w
    assert OdometryBaselineCheck._matches(odom, transform)


def test_yaw_wraps_at_pi():
    assert isclose(abs(_normalize_angle(-pi - (pi - 0.1))), 0.1)
    assert abs(_yaw_from_quaternion(_pose(0.0, 0.0, -0.7).pose.pose.orientation) + 0.7) < 1e-9
