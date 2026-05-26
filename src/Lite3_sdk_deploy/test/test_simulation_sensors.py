import atexit
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2


TIMEOUT_SEC = 60.0
STOP_TIMEOUT_SEC = 10.0
KILL_TIMEOUT_SEC = 5.0
_PROCESS_GROUPS: set[int] = set()


@dataclass(frozen=True)
class ExpectedTopic:
    name: str
    msg_type: type
    is_valid: object


def _sensor_config(simulator: str, sensor: str, topics: dict[str, str]) -> dict:
    config = {
        "simulator": simulator,
        "headless": True,
        "sensors": {},
    }
    if sensor == "lidar_2d":
        config["sensors"]["lidar_2d"] = {"enabled": True, "topic": topics["scan"]}
    elif sensor == "mid360":
        config["sensors"]["mid360"] = {"enabled": True, "topic": topics["mid360"]}
    elif sensor == "realsense":
        config["sensors"]["realsense"] = {
            "enable_depth": True,
            "enable_color": True,
            "enable_pointcloud": True,
            "depth_image_topic": topics["depth_image"],
            "depth_info_topic": topics["depth_info"],
            "color_image_topic": topics["color_image"],
            "color_info_topic": topics["color_info"],
            "pointcloud_topic": topics["realsense_points"],
        }
    else:
        raise ValueError(f"unsupported sensor: {sensor}")
    return config


def _topics(simulator: str, sensor: str) -> dict[str, str]:
    base = f"/lite3_sensor_test/{simulator}/{sensor}"
    return {
        "scan": f"{base}/scan",
        "mid360": f"{base}/mid360/points",
        "depth_image": f"{base}/camera/depth/image_rect_raw",
        "depth_info": f"{base}/camera/depth/camera_info",
        "color_image": f"{base}/camera/color/image_raw",
        "color_info": f"{base}/camera/color/camera_info",
        "realsense_points": f"{base}/camera/depth/color/points",
    }


def _expected_topics(sensor: str, topics: dict[str, str]) -> list[ExpectedTopic]:
    if sensor == "lidar_2d":
        return [
            ExpectedTopic(topics["scan"], LaserScan, lambda msg: msg.header.frame_id == "lidar" and len(msg.ranges) > 0),
        ]
    if sensor == "mid360":
        return [
            ExpectedTopic(topics["mid360"], PointCloud2, lambda msg: msg.header.frame_id == "mid360" and _has_points(msg)),
        ]
    if sensor == "realsense":
        return [
            ExpectedTopic(topics["depth_image"], Image, lambda msg: msg.header.frame_id == "camera_depth_optical_frame" and _has_image_data(msg, "16UC1")),
            ExpectedTopic(topics["depth_info"], CameraInfo, lambda msg: msg.header.frame_id == "camera_depth_optical_frame" and _has_camera_info(msg)),
            ExpectedTopic(topics["color_image"], Image, lambda msg: msg.header.frame_id == "camera_color_optical_frame" and _has_image_data(msg, "rgb8")),
            ExpectedTopic(topics["color_info"], CameraInfo, lambda msg: msg.header.frame_id == "camera_color_optical_frame" and _has_camera_info(msg)),
            ExpectedTopic(topics["realsense_points"], PointCloud2, lambda msg: msg.header.frame_id == "camera_depth_optical_frame" and _has_points(msg)),
        ]
    raise ValueError(f"unsupported sensor: {sensor}")

def _has_image_data(msg: Image, encoding: str) -> bool:
    return msg.encoding == encoding and msg.width > 0 and msg.height > 0 and msg.step > 0 and len(msg.data) > 0


def _has_camera_info(msg: CameraInfo) -> bool:
    return msg.width > 0 and msg.height > 0 and msg.k[0] > 0.0 and msg.k[4] > 0.0


def _has_points(msg: PointCloud2) -> bool:
    return msg.width * msg.height > 0 and msg.point_step > 0 and len(msg.data) > 0


def _write_config(tmp_path: Path, simulator: str, sensor: str, topics: dict[str, str]) -> Path:
    path = tmp_path / f"{simulator}_{sensor}.yaml"
    path.write_text(yaml.safe_dump(_sensor_config(simulator, sensor, topics), sort_keys=False), encoding="utf-8")
    return path


def _start_simulation(config_path: Path, log_path: Path, domain_id: str):
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = domain_id
    env["PYTHONUNBUFFERED"] = "1"
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "run", "lite3_sdk_deploy", "start_simulation.py", "--config", str(config_path)],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )
    process_group_id = os.getpgid(process.pid)
    process._lite3_process_group_id = process_group_id
    _PROCESS_GROUPS.add(process_group_id)
    return process, log_file


def _wait_for_topics(node, executor, process, log_path: Path, expected: list[ExpectedTopic]):
    received = set()
    subscriptions = []

    for topic in expected:
        def callback(msg, topic=topic):
            if topic.is_valid(msg):
                received.add(topic.name)

        subscriptions.append(node.create_subscription(topic.msg_type, topic.name, callback, 10))

    deadline = time.monotonic() + TIMEOUT_SEC
    try:
        while time.monotonic() < deadline and len(received) < len(expected):
            if process.poll() is not None:
                missing = sorted(topic.name for topic in expected if topic.name not in received)
                raise AssertionError(f"simulation exited with code {process.returncode}; missing {missing}\n{_tail(log_path)}")
            executor.spin_once(timeout_sec=0.1)

        missing = sorted(topic.name for topic in expected if topic.name not in received)
        assert not missing, f"timed out waiting for sensor topics: {missing}\n{_tail(log_path)}"
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)


def _stop_simulation(process, log_file):
    try:
        process_group_id = _process_group_id(process)
        if process_group_id is not None:
            _signal_process_group_id(process_group_id, signal.SIGINT)
        if process.poll() is None:
            try:
                process.wait(timeout=STOP_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                if process_group_id is not None:
                    _signal_process_group_id(process_group_id, signal.SIGTERM)
                try:
                    process.wait(timeout=KILL_TIMEOUT_SEC)
                except subprocess.TimeoutExpired:
                    if process_group_id is not None:
                        _signal_process_group_id(process_group_id, signal.SIGKILL)
                    process.wait(timeout=KILL_TIMEOUT_SEC)
        if process_group_id is not None:
            _signal_process_group_id(process_group_id, signal.SIGKILL)
    finally:
        if process_group_id is not None and process.poll() is not None:
            _PROCESS_GROUPS.discard(process_group_id)
        log_file.close()


def _process_group_id(process) -> int | None:
    process_group_id = getattr(process, "_lite3_process_group_id", None)
    if process_group_id is not None:
        return process_group_id
    try:
        return os.getpgid(process.pid)
    except ProcessLookupError:
        return None


def _signal_process_group_id(process_group_id: int, signum: int):
    try:
        os.killpg(process_group_id, signum)
    except ProcessLookupError:
        pass


def _cleanup_process_groups():
    for process_group_id in list(_PROCESS_GROUPS):
        _signal_process_group_id(process_group_id, signal.SIGKILL)


atexit.register(_cleanup_process_groups)


def _tail(path: Path, lines: int = 80) -> str:
    if not path.exists():
        return ""
    return "".join(path.read_text(encoding="utf-8", errors="replace").splitlines(keepends=True)[-lines:])


def _run_sensor_test(tmp_path, simulator: str, sensor: str):
    topics = _topics(simulator, sensor)
    config_path = _write_config(tmp_path, simulator, sensor, topics)
    log_path = tmp_path / f"{simulator}_{sensor}.log"
    domain_id = str(200 + ((os.getpid() + (0 if simulator == "mujoco" else 1)) % 30))
    previous_domain_id = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = domain_id

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(f"test_{simulator}_simulation_sensors", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    process, log_file = _start_simulation(config_path, log_path, domain_id)

    try:
        _wait_for_topics(node, executor, process, log_path, _expected_topics(sensor, topics))
    finally:
        _stop_simulation(process, log_file)
        executor.shutdown(timeout_sec=0.0)
        node.destroy_node()
        rclpy.shutdown(context=context)
        if previous_domain_id is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain_id


@pytest.mark.parametrize("simulator", ["mujoco", "newton"])
def test_simulation_publishes_lidar_2d(tmp_path, simulator):
    _run_sensor_test(tmp_path, simulator, "lidar_2d")


@pytest.mark.parametrize("simulator", ["mujoco", "newton"])
def test_simulation_publishes_mid360(tmp_path, simulator):
    _run_sensor_test(tmp_path, simulator, "mid360")


@pytest.mark.parametrize("simulator", ["mujoco", "newton"])
def test_simulation_publishes_realsense_outputs(tmp_path, simulator):
    _run_sensor_test(tmp_path, simulator, "realsense")