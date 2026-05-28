import math
import os
from pathlib import Path
import time
import unittest
import yaml

from ament_index_python.packages import get_package_share_directory
from artefacts_toolkit.config import get_artefacts_params
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


TEST_TIMEOUT_SEC = 60.0
MAX_ODOM_STEP_M = 1.0
OUTPUT_FOLDER = Path(os.getenv("ARTEFACTS_SCENARIO_UPLOAD_DIR", "./"))
TEST_VIDEO_PATH = OUTPUT_FOLDER / 'lite3_rail_target_follow_distance.mp4'
TEST_CONFIG_PATH = OUTPUT_FOLDER / 'lite3_rail_target_follow_distance.yaml'

try:
    artefacts_params = get_artefacts_params()
except Exception:
    artefacts_params = {}


min_distance_to_travel = float(artefacts_params.get('min_distance_to_travel', 1.5))


def _write_simulation_config() -> None:
    OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
    TEST_CONFIG_PATH.write_text(
        yaml.safe_dump({
            'simulator': 'mujoco',
            'scene': 'procedural://railroad',
            'headless': False,
            'procedural_env_seed': 123,
            'sensors': {
                'mid360': {
                    'enabled': True,
                },
                'follow_camera': {
                    'enabled': True,
                    'video_path': str(TEST_VIDEO_PATH),
                },
            },
        }, sort_keys=False),
        encoding='utf-8',
    )


@pytest.mark.launch_test
def generate_test_description():
    if TEST_VIDEO_PATH.exists():
        TEST_VIDEO_PATH.unlink()
    _write_simulation_config()

    launch_file = (
        get_package_share_directory('lite3_sdk_deploy')
        + '/launch/sim_rail_target_follow.launch.py'
    )

    rail_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'simulation_config': str(TEST_CONFIG_PATH),
            'use_rviz': 'false',
            'use_keyboard_teleop': 'false',
            'use_joy_teleop': 'false',
            'enable_heightmap': 'true',
            'follow_distance': '1.5',
            'min_linear_x': '0.35',
            'max_linear_x': '0.45',
            'stale_timeout_sec': '0.75',
            **artefacts_params,
        }.items(),
    )

    return LaunchDescription([
        rail_follow_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestRailTargetFollowDistance(unittest.TestCase):

    def test_robot_travels_minimum_distance(self):
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('test_sim_rail_target_follow_distance', context=context)
        executor = SingleThreadedExecutor(context=context)
        state = {
            'distance_m': 0.0,
            'message_count': 0,
            'previous_xy': None,
            'last_xy': None,
            'first_stamp_sec': None,
            'last_stamp_sec': None,
        }

        def odom_callback(msg: Odometry):
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            xy = (x, y)
            stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            state['message_count'] += 1
            state['last_xy'] = xy
            # TODO: use /clock once we migrate to use simulation time 
            state['last_stamp_sec'] = stamp_sec
            if state['first_stamp_sec'] is None:
                state['first_stamp_sec'] = stamp_sec

            previous_xy = state['previous_xy']
            state['previous_xy'] = xy
            if previous_xy is None:
                return

            step_m = math.hypot(x - previous_xy[0], y - previous_xy[1])
            if step_m <= MAX_ODOM_STEP_M:
                state['distance_m'] += step_m

        subscription = node.create_subscription(Odometry, '/odom', odom_callback, 20)
        wall_deadline = time.monotonic() + (5.0 * TEST_TIMEOUT_SEC)

        try:
            while time.monotonic() < wall_deadline and state['distance_m'] < min_distance_to_travel:
                first_stamp_sec = state['first_stamp_sec']
                last_stamp_sec = state['last_stamp_sec']
                if first_stamp_sec is not None and last_stamp_sec is not None:
                    if (last_stamp_sec - first_stamp_sec) >= TEST_TIMEOUT_SEC:
                        break
                rclpy.spin_once(node, executor=executor, timeout_sec=0.1)

            elapsed_sec = 0.0
            if state['first_stamp_sec'] is not None and state['last_stamp_sec'] is not None:
                elapsed_sec = state['last_stamp_sec'] - state['first_stamp_sec']
            self.assertGreaterEqual(
                state['distance_m'],
                min_distance_to_travel,
                msg=(
                    f"robot travelled {state['distance_m']:.3f} m in {elapsed_sec:.1f} sim s; "
                    f"expected at least {min_distance_to_travel:.3f} m "
                    f"from {state['message_count']} odom messages, last_xy={state['last_xy']}"
                ),
            )
        finally:
            executor.shutdown(timeout_sec=0.0)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown(context=context)


@launch_testing.post_shutdown_test()
class TestRailTargetFollowVideo(unittest.TestCase):

    def test_follow_camera_video_created(self):
        self.assertTrue(
            os.path.exists(TEST_VIDEO_PATH),
            msg=f"expected follow-camera video at {TEST_VIDEO_PATH}",
        )
        self.assertGreater(
            os.path.getsize(TEST_VIDEO_PATH),
            1024,
            msg=f"follow-camera video at {TEST_VIDEO_PATH} is empty or too small",
        )
