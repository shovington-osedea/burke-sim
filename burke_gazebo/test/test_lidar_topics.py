"""Bounded headless integration checks for the fixed 3D lidar."""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


READY_TIMEOUT = 25.0
MESSAGE_TIMEOUT = 15.0
EXPECTED_FRAME = "mir_3d_lidar_frame"
EXPECTED_TOPIC = "/lidar/points"


class LidarInspectionNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_integration_test")
        self.robot_description: String | None = None
        self.clouds: list[PointCloud2] = []
        description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            "/robot_description",
            self._description_callback,
            description_qos,
        )
        self.create_subscription(
            PointCloud2,
            EXPECTED_TOPIC,
            self._cloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    def _description_callback(self, message: String) -> None:
        self.robot_description = message

    def _cloud_callback(self, message: PointCloud2) -> None:
        self.clouds.append(message)
        del self.clouds[:-4]


def _spin_until(node: LidarInspectionNode, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError(f"Timed out after {timeout:.1f}s waiting for {description}")


def _stamp_seconds(message: PointCloud2) -> float:
    return float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9


def _finite_points(message: PointCloud2) -> list[tuple[float, float, float]]:
    fields = {field.name: field for field in message.fields}
    assert {"x", "y", "z"} <= set(fields), "PointCloud2 is missing x/y/z fields"
    assert message.point_step > 0
    endian = ">" if message.is_bigendian else "<"
    points: list[tuple[float, float, float]] = []
    count = message.width * message.height
    for index in range(count):
        base = index * message.point_step
        try:
            values = tuple(
                struct.unpack_from(endian + "f", message.data, base + fields[name].offset)[0]
                for name in ("x", "y", "z")
            )
        except struct.error:
            continue
        if all(math.isfinite(value) for value in values):
            points.append(values)
    return points


def _float_element(parent: ET.Element, path: str) -> float:
    element = parent.find(path)
    assert element is not None and element.text is not None, f"Missing XML value: {path}"
    return float(element.text)


@pytest.fixture(scope="module")
def simulation():
    package_root = Path(__file__).resolve().parents[1]
    log_directory = Path(tempfile.mkdtemp(prefix="burke_lidar_test_", dir="/tmp"))
    launch_log = (log_directory / "launch.log").open("w")
    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(log_directory / "ros")
    environment["GZ_SIM_LOG_DIR"] = str(log_directory / "gazebo")
    environment["LIBGL_ALWAYS_SOFTWARE"] = "true"
    process = subprocess.Popen(
        ["ros2", "launch", "burke_gazebo", "base_sim.launch.py", "gui:=false"],
        cwd=package_root,
        env=environment,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
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
        launch_log.close()


def test_lidar_description_bridge_and_cloud(simulation) -> None:
    assert simulation.poll() is None, "Headless simulation exited before lidar inspection"
    rclpy.init()
    node = LidarInspectionNode()
    try:
        _spin_until(node, lambda: node.robot_description is not None, READY_TIMEOUT, "/robot_description")
        _spin_until(
            node,
            lambda: any(
                info.topic_type == "sensor_msgs/msg/PointCloud2"
                for info in node.get_publishers_info_by_topic(EXPECTED_TOPIC)
            ),
            READY_TIMEOUT,
            "a PointCloud2 publisher on /lidar/points",
        )

        description = ET.fromstring(node.robot_description.data)
        links = {link.attrib["name"] for link in description.findall("link")}
        assert {"mir_3d_lidar_link", EXPECTED_FRAME} <= links
        joints = {joint.attrib["name"]: joint for joint in description.findall("joint")}
        mount = joints["mir_3d_lidar_mount_joint"]
        frame_joint = joints["mir_3d_lidar_frame_joint"]
        assert mount.attrib["type"] == "fixed"
        assert mount.find("parent").attrib["link"] == "base_link"
        assert mount.find("child").attrib["link"] == "mir_3d_lidar_link"
        assert frame_joint.attrib["type"] == "fixed"
        assert frame_joint.find("parent").attrib["link"] == "mir_3d_lidar_link"
        assert frame_joint.find("child").attrib["link"] == EXPECTED_FRAME

        mount_xyz = [float(value) for value in mount.find("origin").attrib["xyz"].split()]
        mount_rpy = [float(value) for value in mount.find("origin").attrib["rpy"].split()]
        assert mount_xyz == pytest.approx([0.400, 0.000, 0.381230], abs=1e-6)
        assert mount_rpy == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)
        assert 0.0 < mount_xyz[0] < 0.670 - 0.060
        assert abs(mount_xyz[1]) + 0.060 < 0.450
        assert mount_xyz[2] - 0.050 > 0.321230
        assert mount_xyz[0] - 0.060 > 0.161450

        housing = description.find("link[@name='mir_3d_lidar_link']")
        assert housing is not None
        cylinder = housing.find("collision/geometry/cylinder")
        assert cylinder is not None
        assert float(cylinder.attrib["radius"]) == pytest.approx(0.060)
        assert float(cylinder.attrib["length"]) == pytest.approx(0.100)

        sensor = description.find("gazebo/sensor")
        assert sensor is not None and sensor.attrib["type"] == "gpu_lidar"
        assert sensor.findtext("topic") == "/model/burke_base/lidar/points"
        lidar = sensor.find("lidar")
        assert lidar is not None
        horizontal = lidar.find("scan/horizontal")
        vertical = lidar.find("scan/vertical")
        assert horizontal is not None and vertical is not None
        assert int(horizontal.findtext("samples")) == 640
        assert int(vertical.findtext("samples")) == 16
        assert _float_element(horizontal, "min_angle") == pytest.approx(-math.pi)
        assert _float_element(horizontal, "max_angle") == pytest.approx(math.pi)
        assert _float_element(vertical, "min_angle") == pytest.approx(-math.radians(15))
        assert _float_element(vertical, "max_angle") == pytest.approx(math.radians(15))
        assert _float_element(sensor, "update_rate") == pytest.approx(10.0)
        assert _float_element(lidar, "range/min") == pytest.approx(0.20)
        assert _float_element(lidar, "range/max") == pytest.approx(50.0)
        assert _float_element(lidar, "range/resolution") == pytest.approx(0.010)
        noise = lidar.find("noise")
        assert noise is not None and noise.findtext("type") == "gaussian"
        assert _float_element(noise, "mean") == pytest.approx(0.0)
        assert _float_element(noise, "stddev") == pytest.approx(0.010)

        _spin_until(node, lambda: len(node.clouds) >= 2, MESSAGE_TIMEOUT, "two lidar point clouds")
        first, second = node.clouds[-2:]
        assert first.header.frame_id == EXPECTED_FRAME
        assert second.header.frame_id == EXPECTED_FRAME
        assert _stamp_seconds(second) > _stamp_seconds(first), "Point-cloud stamps did not advance"
        assert first.width == 640 and first.height == 16
        assert second.width == 640 and second.height == 16
        field_names = {field.name for field in second.fields}
        assert {"x", "y", "z"} <= field_names
        finite = _finite_points(second)
        assert finite, "Point cloud contains no finite x/y/z points"
        ranges = [math.sqrt(x * x + y * y + z * z) for x, y, z in finite]
        assert any(0.20 <= distance <= 50.0 for distance in ranges), (
            "No finite in-range return was found; the deterministic ground plane "
            "did not produce a lidar target"
        )
        assert len({round(z, 3) for _, _, z in finite}) > 1, (
            "All finite returns share one vertical layer; lidar appears planar"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
