"""Bounded local tests for the default Burk-e Foxglove Bridge lifecycle."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml


TEST_ADDRESS = "127.0.0.1"
TEST_PORT = 8766
READY_TIMEOUT = 30.0
RELEASE_TIMEOUT = 5.0
REQUIRED_TOPICS = {
    "/clock",
    "/tf",
    "/tf_static",
    "/robot_description",
    "/odom",
    "/joint_states",
    "/lidar/points",
    "/foxglove_bridge/client_count",
}


def _port_is_open(port: int) -> bool:
    try:
        with socket.create_connection((TEST_ADDRESS, port), timeout=0.5):
            return True
    except OSError:
        return False


def _wait_for_port(port: int, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _port_is_open(port):
            return
        time.sleep(0.25)
    raise AssertionError(
        f"Foxglove listener did not open {TEST_ADDRESS}:{port} within {timeout:.1f}s; "
        "this is a local bridge failure, not an external firewall or Parallels issue"
    )


def _ros_topics(environment: dict[str, str]) -> set[str]:
    result = subprocess.run(
        ["ros2", "topic", "list"],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5.0,
        check=True,
    )
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def _stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)


def _assert_port_released(port: int) -> None:
    deadline = time.monotonic() + RELEASE_TIMEOUT
    while time.monotonic() < deadline:
        if not _port_is_open(port):
            with socket.socket() as listener:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((TEST_ADDRESS, port))
            return
        time.sleep(0.25)
    raise AssertionError(f"Foxglove test port {port} was not released after cleanup")


@pytest.fixture(scope="module")
def foxglove_simulation():
    ros2 = shutil.which("ros2")
    if ros2 is None:
        pytest.fail("The ros2 executable is required for the Foxglove bridge test")

    log_directory = Path(tempfile.mkdtemp(prefix="burke_foxglove_test_", dir="/tmp"))
    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(log_directory / "ros")
    environment["GZ_SIM_LOG_DIR"] = str(log_directory / "gazebo")
    environment["LIBGL_ALWAYS_SOFTWARE"] = "true"
    (log_directory / "ros").mkdir()
    (log_directory / "gazebo").mkdir()
    launch_log = (log_directory / "launch.log").open("w")
    process = subprocess.Popen(
        [
            ros2,
            "launch",
            "burke_gazebo",
            "foxglove_sim.launch.py",
            "gui:=false",
            f"foxglove_address:={TEST_ADDRESS}",
            f"foxglove_port:={TEST_PORT}",
        ],
        env=environment,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    try:
        _wait_for_port(TEST_PORT, READY_TIMEOUT)
        yield process, environment, launch_log.name
    finally:
        _stop_process(process)
        launch_log.close()
        _assert_port_released(TEST_PORT)


def test_foxglove_package_resolves() -> None:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", "foxglove_bridge"],
        capture_output=True,
        text=True,
        timeout=10.0,
        check=True,
    )
    assert Path(result.stdout.strip()).is_dir()


def test_base_launch_starts_bridge_by_default() -> None:
    """The normal base launch owns the default Foxglove listener."""
    test_port = TEST_PORT + 1
    log_directory = Path(tempfile.mkdtemp(prefix="burke_foxglove_default_", dir="/tmp"))
    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(log_directory / "ros")
    environment["GZ_SIM_LOG_DIR"] = str(log_directory / "gazebo")
    environment["LIBGL_ALWAYS_SOFTWARE"] = "true"
    (log_directory / "ros").mkdir()
    (log_directory / "gazebo").mkdir()
    launch_log = (log_directory / "launch.log").open("w")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "burke_gazebo",
            "base_sim.launch.py",
            "gui:=false",
            f"foxglove_address:={TEST_ADDRESS}",
            f"foxglove_port:={test_port}",
        ],
        env=environment,
        stdout=launch_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    try:
        _wait_for_port(test_port, READY_TIMEOUT)
        assert process.poll() is None
    finally:
        _stop_process(process)
        launch_log.close()
        _assert_port_released(test_port)


def test_installed_bridge_policy() -> None:
    config_path = (
        Path(get_package_share_directory("burke_gazebo"))
        / "config"
        / "foxglove_bridge.yaml"
    )
    with config_path.open() as stream:
        document = yaml.safe_load(stream)
    parameters = document["/foxglove_bridge"]["ros__parameters"]

    assert parameters["use_sim_time"] is True
    assert parameters["topic_whitelist"] == [
        "^/(?:clock|tf|tf_static|robot_description|odom|joint_states|lidar/points|foxglove_bridge/client_count)$"
    ]
    assert parameters["client_topic_whitelist"] == ["^$"]
    assert parameters["service_whitelist"] == ["^$"]
    assert parameters["param_whitelist"] == ["^$"]
    assert parameters["capabilities"] == ["connectionGraph", "assets"]
    assert parameters["best_effort_qos_topic_whitelist"] == ["^/lidar/points$"]
    assert parameters["asset_uri_allowlist"] == [
        "^package://burke_description/cad/stl/[A-Za-z0-9_.%-]+[.]stl$"
    ]
    assert parameters["publish_client_count"] is True
    assert parameters["sysinfo"] is False


def test_bridge_listener_and_required_topics(foxglove_simulation) -> None:
    process, environment, launch_log = foxglove_simulation
    assert process.poll() is None, f"Foxglove launch exited early; see {launch_log}"
    _wait_for_port(TEST_PORT, READY_TIMEOUT)

    deadline = time.monotonic() + READY_TIMEOUT
    topics: set[str] = set()
    while time.monotonic() < deadline:
        topics = _ros_topics(environment)
        if REQUIRED_TOPICS <= topics:
            break
        time.sleep(0.5)
    assert REQUIRED_TOPICS <= topics, (
        f"Missing local visualization topics: {sorted(REQUIRED_TOPICS - topics)}; "
        f"see {launch_log}. Remote reachability is not tested here."
    )


def test_local_tcp_client_connects(foxglove_simulation) -> None:
    process, _, launch_log = foxglove_simulation
    assert process.poll() is None, f"Foxglove launch exited early; see {launch_log}"
    with socket.create_connection((TEST_ADDRESS, TEST_PORT), timeout=2.0) as client:
        assert client.getpeername()[1] == TEST_PORT
