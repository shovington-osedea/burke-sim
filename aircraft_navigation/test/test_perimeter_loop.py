"""Bounded headless one-loop validation using wheel odometry and TF only."""

from __future__ import annotations

import math
from pathlib import Path
import time

import pytest
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rclpy.node import Node

from test_navigation_interfaces import navigation_launch


LOOP_TIMEOUT = 360.0
STOP_OBSERVATION_S = 5.0
MAX_CROSS_TRACK_M = 1.5
RMS_CROSS_TRACK_M = 0.75
MAX_HEADING_ERROR_RAD = math.pi
STOW_POSE = (0.0, -2.0944, 2.0944, -1.5708, 1.5708, 0.0)


class LoopNode(Node):
    def __init__(self) -> None:
        super().__init__('perimeter_loop_test')
        self.commands: list[tuple[float, float, float]] = []
        self.progress: list[float] = []
        self.errors: list[float] = []
        self.heading_errors: list[float] = []
        self.lookahead: PointStamped | None = None
        self.odom: Odometry | None = None
        self.joints: JointState | None = None
        self.create_subscription(Twist, '/cmd_vel', self._command, 10)
        self.create_subscription(Float64, '/path_follower/progress', self._progress, 10)
        self.create_subscription(Float64, '/path_follower/cross_track_error', self._error, 10)
        self.create_subscription(PointStamped, '/path_follower/lookahead', self._target, 10)
        self.create_subscription(Odometry, '/odom', self._odom, 10)
        self.create_subscription(JointState, '/joint_states', self._joints, 10)

    def _command(self, message: Twist) -> None:
        self.commands.append((time.monotonic(), message.linear.x, message.angular.z))

    def _progress(self, message: Float64) -> None:
        self.progress.append(message.data)

    def _error(self, message: Float64) -> None:
        self.errors.append(message.data)

    def _target(self, message: PointStamped) -> None:
        self.lookahead = message
        if self.odom is not None:
            yaw = _yaw(self.odom.pose.pose.orientation)
            target_yaw = math.atan2(
                message.point.y - self.odom.pose.pose.position.y,
                message.point.x - self.odom.pose.pose.position.x)
            self.heading_errors.append(abs(_angle_difference(target_yaw, yaw)))

    def _odom(self, message: Odometry) -> None:
        self.odom = message

    def _joints(self, message: JointState) -> None:
        self.joints = message


def _spin_until(node: LoopNode, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError(f'timed out waiting for {description}')


def _joint_values(message: JointState | None) -> dict[str, float]:
    if message is None:
        return {}
    return {name: message.position[index] for index, name in enumerate(message.name)
            if index < len(message.position)}


def _yaw(orientation) -> float:
    return math.atan2(2.0 * orientation.w * orientation.z,
                      1.0 - 2.0 * orientation.z * orientation.z)


def _angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _frozen_path_metrics() -> None:
    """Ensure this test scores the validated path, not Gazebo ground truth."""
    source = (Path(__file__).parents[1] / 'config' / 'perimeter_path.yaml').read_text()
    assert 'frame_id: aircraft' in source
    assert 'clearance_m: 1.0' in source


def test_one_clockwise_loop_stops_and_does_not_restart(navigation_launch) -> None:
    assert navigation_launch.poll() is None
    _frozen_path_metrics()
    rclpy.init()
    node = LoopNode()
    try:
        _spin_until(node, lambda: node.joints is not None, 30.0, 'joint states')
        joints = _joint_values(node.joints)
        for index, name in enumerate(
                ('arm_joint_1', 'arm_joint_2', 'arm_joint_3',
                 'arm_joint_4', 'arm_joint_5', 'arm_joint_6')):
            assert joints[name] == pytest.approx(STOW_POSE[index], abs=0.08)
        assert joints['lift_stage_2_joint'] == pytest.approx(0.0, abs=0.03)
        assert joints['lift_stage_3_joint'] == pytest.approx(0.0, abs=0.03)
        _spin_until(node, lambda: any(abs(x) > 1e-4 for _, x, _ in node.commands),
                    45.0, 'first non-zero base command')
        _spin_until(node, lambda: any(value >= 1.0 - 1e-9 for value in node.progress),
                    LOOP_TIMEOUT, 'one full clockwise loop')
        completion_index = next(index for index, value in enumerate(node.progress)
                                if value >= 1.0 - 1e-9)
        progress_after_completion = node.progress[completion_index:]
        assert all(b + 1e-6 >= a for a, b in zip(progress_after_completion,
                                                  progress_after_completion[1:]))
        observation_end = time.monotonic() + STOP_OBSERVATION_S
        while time.monotonic() < observation_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert not any(abs(x) > 1e-4 or abs(z) > 1e-4
                       for _, x, z in node.commands[-50:])
        assert max(node.errors, default=0.0) <= MAX_CROSS_TRACK_M
        rms = math.sqrt(sum(error * error for error in node.errors)
                        / max(1, len(node.errors)))
        assert rms <= RMS_CROSS_TRACK_M
        assert max(node.heading_errors, default=0.0) <= MAX_HEADING_ERROR_RAD
        assert all(abs(x) <= 0.5 + 1e-6 and abs(z) <= 0.5 + 1e-6
                   for _, x, z in node.commands)
        accelerations = [
            ((x - previous_x) / max(1e-6, stamp - previous_stamp),
             (z - previous_z) / max(1e-6, stamp - previous_stamp))
            for (previous_stamp, previous_x, previous_z), (stamp, x, z)
            in zip(node.commands, node.commands[1:])]
        assert all(abs(linear) <= 0.4 + 0.05 and abs(angular) <= 0.8 + 0.05
                   for linear, angular in accelerations)
        assert all(b + 1e-6 >= a for a, b in zip(node.progress, node.progress[1:]))
    finally:
        node.destroy_node()
        rclpy.shutdown()
