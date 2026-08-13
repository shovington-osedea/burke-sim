"""Headless smoke test for the primitive arm and mobile base interfaces."""

from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


ARM_JOINTS = tuple(f"arm_joint_{index}" for index in range(1, 7))
COMMAND_TOPICS = tuple(f"/arm/joint_{index}/command" for index in range(1, 7))
JOINT_LIMITS = {
    "arm_joint_1": (-3.14, 3.14),
    "arm_joint_2": (-1.57, 1.57),
    "arm_joint_3": (-2.35, 2.35),
    "arm_joint_4": (-3.14, 3.14),
    "arm_joint_5": (-1.57, 1.57),
    "arm_joint_6": (-3.14, 3.14),
}
TARGETS = (0.20, -0.20, 0.18, -0.18, 0.16, -0.16)
READY_TIMEOUT = 20.0
MOTION_TIMEOUT = 20.0
TARGET_TOLERANCE = 0.07


class ArmSmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_smoke_test")
        self.joint_state: JointState | None = None
        self.odom: Odometry | None = None
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )
        self.command_publishers = [
            self.create_publisher(Float64, topic, 10) for topic in COMMAND_TOPICS
        ]
        self.base_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def _joint_state_callback(self, message: JointState) -> None:
        self.joint_state = message

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message

    def publish_joint_target(self, index: int, target: float) -> None:
        self.command_publishers[index].publish(Float64(data=target))

    def publish_base_command(self, linear_x: float) -> None:
        message = Twist()
        message.linear.x = linear_x
        self.base_publisher.publish(message)

    def stop_base(self) -> None:
        self.publish_base_command(0.0)


def _spin_until(node: ArmSmokeNode, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError(f"Timed out after {timeout:.1f}s waiting for {description}")


def _arm_positions(message: JointState | None) -> dict[str, float]:
    if message is None:
        return {}
    return {
        name: message.position[index]
        for index, name in enumerate(message.name)
        if name in ARM_JOINTS and index < len(message.position)
    }


@pytest.fixture(scope="module")
def simulation():
    ros2 = shutil.which("ros2")
    if ros2 is None:
        pytest.fail("The ros2 executable is required for the headless simulation test")

    package_root = Path(__file__).resolve().parents[1]
    log_directory = Path(tempfile.mkdtemp(prefix="burke_gazebo_test_", dir="/tmp"))
    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(log_directory)
    os.environ["ROS_LOG_DIR"] = str(log_directory)
    process = subprocess.Popen(
        [ros2, "launch", "burke_gazebo", "base_sim.launch.py", "gui:=false"],
        cwd=package_root,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
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


def test_arm_topics_and_base_motion(simulation) -> None:
    del simulation  # The fixture keeps the launch process alive for this test.
    rclpy.init()
    node = ArmSmokeNode()
    try:
        _spin_until(
            node,
            lambda: node.joint_state is not None
            and set(ARM_JOINTS).issubset(_arm_positions(node.joint_state)),
            READY_TIMEOUT,
            "/joint_states with all six arm joints",
        )
        _spin_until(
            node,
            lambda: all(
                any(topic == name for topic, _ in node.get_topic_names_and_types())
                for name in COMMAND_TOPICS
            ),
            READY_TIMEOUT,
            "all six arm command topics",
        )
        _spin_until(
            node,
            lambda: all(
                publisher.get_subscription_count() > 0
                for publisher in node.command_publishers
            ),
            READY_TIMEOUT,
            "all six arm bridge subscriptions",
        )
        _spin_until(
            node,
            lambda: node.odom is not None,
            READY_TIMEOUT,
            "/odom feedback",
        )

        starting_positions = _arm_positions(node.joint_state)
        assert set(starting_positions) == set(ARM_JOINTS)
        for name, position in starting_positions.items():
            lower, upper = JOINT_LIMITS[name]
            assert lower - 0.02 <= position <= upper + 0.02, (
                f"{name} started outside its declared limits: {position}"
            )

        for index, (name, target) in enumerate(zip(ARM_JOINTS, TARGETS)):
            def reached_target(name=name, target=target) -> bool:
                positions = _arm_positions(node.joint_state)
                return name in positions and math.isclose(
                    positions[name], target, abs_tol=TARGET_TOLERANCE
                )

            deadline = time.monotonic() + MOTION_TIMEOUT
            while time.monotonic() < deadline and not reached_target():
                # Repeating during the bounded wait avoids losing a one-shot
                # message while the ROS-Gazebo bridge subscriber connects.
                node.publish_joint_target(index, target)
                rclpy.spin_once(node, timeout_sec=0.1)
            assert reached_target(), (
                f"Timed out after {MOTION_TIMEOUT:.1f}s waiting for "
                f"{name} to {target:.2f} rad; current positions: "
                f"{_arm_positions(node.joint_state)}"
            )
            positions = _arm_positions(node.joint_state)
            for joint_name, position in positions.items():
                lower, upper = JOINT_LIMITS[joint_name]
                assert lower - 0.02 <= position <= upper + 0.02, (
                    f"{joint_name} exceeded its declared limits: {position}"
                )

        initial_x = node.odom.pose.pose.position.x
        movement_deadline = time.monotonic() + 2.0
        while time.monotonic() < movement_deadline:
            node.publish_base_command(0.12)
            rclpy.spin_once(node, timeout_sec=0.1)
        node.stop_base()
        _spin_until(
            node,
            lambda: node.odom is not None
            and abs(node.odom.pose.pose.position.x - initial_x) > 0.005,
            5.0,
            "odometry change after a base command",
        )
    finally:
        node.stop_base()
        for index in range(6):
            node.publish_joint_target(index, 0.0)
        node.destroy_node()
        rclpy.shutdown()
