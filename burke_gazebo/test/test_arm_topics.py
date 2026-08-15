"""Headless integration tests for the Burk-e arm, lift, and base interfaces."""

from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


ARM_JOINTS = tuple(f"arm_joint_{index}" for index in range(1, 7))
COMMAND_TOPICS = tuple(f"/arm/joint_{index}/command" for index in range(1, 7))
JOINT_LIMITS = {
    "arm_joint_1": (-2.0 * math.pi, 2.0 * math.pi),
    "arm_joint_2": (-2.0 * math.pi, 2.0 * math.pi),
    "arm_joint_3": (-math.pi, math.pi),
    "arm_joint_4": (-2.0 * math.pi, 2.0 * math.pi),
    "arm_joint_5": (-2.0 * math.pi, 2.0 * math.pi),
    "arm_joint_6": (-2.0 * math.pi, 2.0 * math.pi),
}
TARGETS = (0.20, -0.20, 0.18, -0.18, 0.16, -0.16)
READY_TIMEOUT = 20.0
MOTION_TIMEOUT = 20.0
TARGET_TOLERANCE = 0.07
TARGET_HOLD_DURATION = 0.5
LIFT_JOINTS = ("lift_stage_2_joint", "lift_stage_3_joint")
LIFT_TOPICS = ("/lift/stage_2/command", "/lift/stage_3/command")
LIFT_LIMITS = (0.275, 0.225)
LIFT_TARGET_TOLERANCE = 0.025
LIFT_BASE_HEIGHT = 0.555
ARM_MOUNT_HEIGHT = 0.876230
STOW_POSE = (0.0, -2.0944, 2.0944, -1.5708, 1.5708, 0.0)


class ArmSmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_smoke_test")
        self.joint_state: JointState | None = None
        self.odom: Odometry | None = None
        self.robot_description: String | None = None
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )
        self.description_sub = self.create_subscription(
            String,
            "/robot_description",
            self._description_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.command_publishers = [
            self.create_publisher(Float64, topic, 10) for topic in COMMAND_TOPICS
        ]
        self.lift_publishers = [
            self.create_publisher(Float64, topic, 10) for topic in LIFT_TOPICS
        ]
        self.base_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def _joint_state_callback(self, message: JointState) -> None:
        self.joint_state = message

    def _odom_callback(self, message: Odometry) -> None:
        self.odom = message

    def _description_callback(self, message: String) -> None:
        self.robot_description = message

    def publish_joint_target(self, index: int, target: float) -> None:
        self.command_publishers[index].publish(Float64(data=target))

    def publish_base_command(self, linear_x: float) -> None:
        message = Twist()
        message.linear.x = linear_x
        self.base_publisher.publish(message)

    def publish_lift_target(self, index: int, target: float) -> None:
        self.lift_publishers[index].publish(Float64(data=target))

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


def _lift_positions(message: JointState | None) -> dict[str, float]:
    if message is None:
        return {}
    return {
        name: message.position[index]
        for index, name in enumerate(message.name)
        if name in LIFT_JOINTS and index < len(message.position)
    }


def _wait_for_lift_target(node: ArmSmokeNode, targets: tuple[float, float]) -> None:
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        for index, target in enumerate(targets):
            node.publish_lift_target(index, target)
        rclpy.spin_once(node, timeout_sec=0.1)
        positions = _lift_positions(node.joint_state)
        if all(
            joint in positions
            and math.isclose(positions[joint], target, abs_tol=LIFT_TARGET_TOLERANCE)
            for joint, target in zip(LIFT_JOINTS, targets)
        ):
            return
    raise AssertionError(
        f"Timed out waiting for lift target {targets}; current positions: "
        f"{_lift_positions(node.joint_state)}"
    )


def _assert_lift_limits(node: ArmSmokeNode) -> None:
    positions = _lift_positions(node.joint_state)
    assert set(positions) == set(LIFT_JOINTS)
    for joint, upper in zip(LIFT_JOINTS, LIFT_LIMITS):
        assert -0.02 <= positions[joint] <= upper + 0.02, (
            f"{joint} exceeded its declared limit: {positions[joint]}"
        )


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
            hold_deadline = time.monotonic() + TARGET_HOLD_DURATION
            while time.monotonic() < hold_deadline:
                node.publish_joint_target(index, target)
                rclpy.spin_once(node, timeout_sec=0.05)
                position = _arm_positions(node.joint_state).get(name)
                assert position is not None
                lower, upper = JOINT_LIMITS[name]
                assert lower - 0.02 <= position <= upper + 0.02, (
                    f"{name} exceeded its declared limits while holding: {position}"
                )
            positions = _arm_positions(node.joint_state)
            for joint_name, position in positions.items():
                lower, upper = JOINT_LIMITS[joint_name]
                assert lower - 0.02 <= position <= upper + 0.02, (
                    f"{joint_name} exceeded its declared limits: {position}"
                )

    finally:
        node.stop_base()
        for index in range(6):
            node.publish_joint_target(index, 0.0)
        node.destroy_node()
        rclpy.shutdown()


def test_lift_description_motion_and_safe_sequence(simulation) -> None:
    del simulation
    rclpy.init()
    node = ArmSmokeNode()
    try:
        _spin_until(
            node,
            lambda: node.joint_state is not None
            and set(ARM_JOINTS + LIFT_JOINTS).issubset(
                set(node.joint_state.name)
            ),
            READY_TIMEOUT,
            "joint-state feedback for all arm and lift joints",
        )
        _spin_until(
            node,
            lambda: node.robot_description is not None,
            READY_TIMEOUT,
            "/robot_description",
        )
        _spin_until(
            node,
            lambda: all(
                publisher.get_subscription_count() > 0
                for publisher in node.lift_publishers
            ),
            READY_TIMEOUT,
            "both lift bridge subscriptions",
        )
        _spin_until(node, lambda: node.odom is not None, READY_TIMEOUT, "/odom feedback")

        description = ET.fromstring(node.robot_description.data)
        links = {link.attrib["name"]: link for link in description.findall("link")}
        assert {"lift_stage_1_link", "lift_stage_2_link", "lift_stage_3_link"} <= set(links)
        lift_joints = {
            joint.attrib["name"]: joint
            for joint in description.findall("joint")
            if joint.attrib["name"] in LIFT_JOINTS
        }
        assert set(lift_joints) == set(LIFT_JOINTS)
        assert all(joint.attrib["type"] == "prismatic" for joint in lift_joints.values())
        assert all(joint.find("limit") is not None for joint in lift_joints.values())
        arm_mount_joint = next(
            joint for joint in description.findall("joint")
            if joint.attrib["name"] == "arm_mount_joint"
        )
        assert arm_mount_joint.find("parent").attrib["link"] == "lift_stage_3_link"
        assert arm_mount_joint.find("origin").attrib["xyz"].split()[2] == "0.555"

        expected_meshes = {
            "LIFTKIT_1.stl",
            "LIFTKIT_2.stl",
            "LIFTKIT_3.stl",
        }
        description_share = Path(get_package_share_directory("burke_description"))
        for mesh_name in expected_meshes:
            assert (description_share / "cad" / "stl" / mesh_name).is_file(), (
                f"Installed LiftKit mesh is missing: {mesh_name}"
            )
        for link_name in ("lift_stage_1_link", "lift_stage_2_link", "lift_stage_3_link"):
            visual_mesh = links[link_name].find("visual/geometry/mesh")
            assert visual_mesh is not None
            assert "LIFTKIT" in visual_mesh.attrib["filename"]
            scale = visual_mesh.attrib["scale"].split()
            assert scale == ["0.001", "0.001", "0.001"]
            assert not any(
                "LIFTKIT" in (element.attrib.get("filename", ""))
                for element in links[link_name].findall(".//collision//mesh")
            ), f"LiftKit CAD mesh used as collision on {link_name}"

        # The arm must be in the documented stow pose before the mast moves.
        for index, target in enumerate(STOW_POSE):
            node.publish_joint_target(index, target)
        _spin_until(
            node,
            lambda: all(
                math.isclose(_arm_positions(node.joint_state).get(joint, 99.0), target, abs_tol=0.10)
                for joint, target in zip(ARM_JOINTS, STOW_POSE)
            ),
            MOTION_TIMEOUT,
            "arm stow pose before lift motion",
        )

        _wait_for_lift_target(node, (0.0, 0.0))
        _assert_lift_limits(node)
        assert math.isclose(
            LIFT_BASE_HEIGHT, 0.555, abs_tol=1e-6
        ), "Collapsed mast height constant changed unexpectedly"

        half_target = (LIFT_LIMITS[0] / 2.0, LIFT_LIMITS[1] / 2.0)
        _wait_for_lift_target(node, half_target)
        _assert_lift_limits(node)
        half_positions = _lift_positions(node.joint_state)
        assert math.isclose(
            LIFT_BASE_HEIGHT + sum(half_target), 0.805, abs_tol=0.002
        )
        assert all(
            math.isclose(half_positions[joint], target, abs_tol=LIFT_TARGET_TOLERANCE)
            for joint, target in zip(LIFT_JOINTS, half_target)
        )
        assert math.isclose(
            LIFT_BASE_HEIGHT + sum(half_positions[joint] for joint in LIFT_JOINTS),
            0.805,
            abs_tol=0.04,
        ), "Arm mount height did not track the combined half-lift displacement"

        full_target = LIFT_LIMITS
        _wait_for_lift_target(node, full_target)
        _assert_lift_limits(node)
        assert math.isclose(LIFT_BASE_HEIGHT + sum(full_target), 1.055, abs_tol=0.002)
        full_positions = _lift_positions(node.joint_state)
        assert math.isclose(
            LIFT_BASE_HEIGHT + sum(full_positions[joint] for joint in LIFT_JOINTS),
            1.055,
            abs_tol=0.04,
        ), "Arm mount height did not track the combined full-lift displacement"

        # Return the mast to zero before issuing the first base command.
        _wait_for_lift_target(node, (0.0, 0.0))
        _assert_lift_limits(node)
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
            "odometry change after lift returned to zero and base command",
        )
    finally:
        node.stop_base()
        for index, target in enumerate((0.0, 0.0)):
            node.publish_lift_target(index, target)
        for index, target in enumerate(STOW_POSE):
            node.publish_joint_target(index, target)
        node.destroy_node()
        rclpy.shutdown()
