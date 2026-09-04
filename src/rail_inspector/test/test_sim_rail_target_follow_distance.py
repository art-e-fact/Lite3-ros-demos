import os
from pathlib import Path

import pytest
from artefacts_toolkit.config import get_artefacts_params

from sim_control_harness import SimControlHarness, StopReason


TEST_TIMEOUT_SEC = 60.0

OUTPUT_FOLDER = Path(os.getenv('ARTEFACTS_SCENARIO_UPLOAD_DIR', './'))

REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_PACKAGE_ROOT = REPO_ROOT / 'src' / 'simulation_package'

try:
    artefacts_params = get_artefacts_params()
except Exception:
    artefacts_params = {}

min_distance_to_travel = float(artefacts_params.get('min_distance_to_travel', 1.5))

# Only these keys from artefacts_params are forwarded to the logic launch file.
_LOGIC_LAUNCH_PARAMS = {
    'follow_distance', 'min_linear_x', 'max_linear_x',
    'distance_error_for_max_speed', 'max_linear_y', 'max_angular_z',
    'k_center', 'k_heading', 'stale_timeout_sec',
}


def test_robot_travels_minimum_distance(tmp_path, simulator, headless, follow_camera_budget):

    OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
    test_video_path = OUTPUT_FOLDER / f'lite3_{simulator}_rail_target_follow_distance.mp4'
    test_rrd_path = OUTPUT_FOLDER / f'lite3_{simulator}_rail_target_follow_distance.rrd'
    test_config_path = OUTPUT_FOLDER / f'lite3_{simulator}_rail_target_follow_distance.yaml'
    if test_video_path.exists():
        test_video_path.unlink()
    if test_rrd_path.exists():
        test_rrd_path.unlink()

    sim_config = {
        'simulator': simulator,
        'scene': 'procedural://railroad',
        'headless': headless,
        'procedural_env_seed': 123,
        'sensors': {
            'mid360': {
                'enabled': True,
            },
            'follow_camera': {
                'enabled': True,
                'video_path': str(test_video_path),
                **follow_camera_budget,
            },
        },
        'rerun': {
            'enabled': True,
            'spawn': False,
            'save_path': str(test_rrd_path),
            'close_viewer_on_exit': True,
        },
    }

    logic_args = {
        'enable_heightmap': 'true',
        'cloud_topic': '/mid360/points',
        'follow_distance': '1.5',
        'min_linear_x': '0.35',
        'max_linear_x': '0.45',
        'stale_timeout_sec': '0.75',
        'use_sim_time': 'true',
        **{k: str(v) for k, v in artefacts_params.items() if k in _LOGIC_LAUNCH_PARAMS},
    }

    with SimControlHarness(
        sim_config,
        config_path=test_config_path,
        log_dir=tmp_path,
        repo_root=REPO_ROOT,
        sim_package_root=SIM_PACKAGE_ROOT,
        control_launch_args=logic_args,
        max_runtime_sec=5.0 * TEST_TIMEOUT_SEC,
        sim_timeout_sec=TEST_TIMEOUT_SEC,
    ) as harness:
        reason = harness.wait(
            predicate=lambda: harness.total_distance_m >= min_distance_to_travel,
        )

    if reason is StopReason.SIM_EXITED:
        pytest.fail(
            f'simulation exited unexpectedly; '
            f'distance so far {harness.total_distance_m:.3f} m\n'
            f'{harness.sim_log_tail()}'
        )
    if reason is StopReason.CONTROL_EXITED:
        pytest.fail(
            f'nav logic exited unexpectedly; '
            f'distance so far {harness.total_distance_m:.3f} m\n'
            f'{harness.control_log_tail()}'
        )

    assert harness.total_distance_m >= min_distance_to_travel, (
        f"robot travelled {harness.total_distance_m:.3f} m "
        f"in {harness.elapsed_sim_sec:.1f} sim s; "
        f"expected at least {min_distance_to_travel:.3f} m "
        f"from {harness.message_count} odom messages, last_xy={harness.last_xy}"
    )

    assert test_video_path.exists(), f'expected follow-camera video at {test_video_path}'
    assert test_video_path.stat().st_size > 1024, (
        f'follow-camera video at {test_video_path} is empty or too small'
    )

    assert test_rrd_path.exists(), f'expected Rerun recording at {test_rrd_path}'
    assert test_rrd_path.stat().st_size > 1024, (
        f'Rerun recording at {test_rrd_path} is empty or too small'
    )