import atexit
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from artefacts_toolkit.config import get_artefacts_params
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor


TEST_TIMEOUT_SEC = 60.0
MAX_ODOM_STEP_M = 1.0
STOP_TIMEOUT_SEC = 10.0
KILL_TIMEOUT_SEC = 5.0

OUTPUT_FOLDER = Path(os.getenv("ARTEFACTS_SCENARIO_UPLOAD_DIR", "./"))
TEST_VIDEO_PATH = OUTPUT_FOLDER / 'lite3_rail_target_follow_distance.mp4'
TEST_CONFIG_PATH = OUTPUT_FOLDER / 'lite3_rail_target_follow_distance.yaml'

# Simulation package source root — lets pixi run the sim with -m simulation_package.start_simulation
# without depending on a ROS install space, mirroring the approach in test_simulation_sensors.py.
REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_PACKAGE_ROOT = REPO_ROOT / "src" / "simulation_package"

_PROCESS_GROUPS: set[int] = set()
_PROCESS_GROUP_IDS: dict[int, int] = {}

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


def _write_simulation_config() -> None:
    headless_str = str(artefacts_params.get('headless', 'false')).strip().lower()
    headless = headless_str in ('true', '1', 'yes', 'on')
    OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
    if TEST_VIDEO_PATH.exists():
        TEST_VIDEO_PATH.unlink()
    TEST_CONFIG_PATH.write_text(
        yaml.safe_dump({
            'simulator': 'mujoco',
            'scene': 'procedural://railroad',
            'headless': headless,
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


def _start_process(cmd: list[str], log_path: Path, domain_id: str, extra_env: dict | None = None):
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = domain_id
    env['PYTHONUNBUFFERED'] = '1'
    if extra_env:
        env.update(extra_env)
    log_file = log_path.open('w', encoding='utf-8')
    process = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        cwd=str(REPO_ROOT),
        start_new_session=True,
    )
    pgid = os.getpgid(process.pid)
    _PROCESS_GROUPS.add(pgid)
    _PROCESS_GROUP_IDS[process.pid] = pgid
    return process, log_file


def _stop_process(process, log_file) -> None:
    pgid = _PROCESS_GROUP_IDS.get(process.pid)
    if pgid is None:
        try:
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            pgid = None
    try:
        if pgid is not None:
            _killpg(pgid, signal.SIGINT)
        if process.poll() is None:
            try:
                process.wait(timeout=STOP_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                if pgid is not None:
                    _killpg(pgid, signal.SIGTERM)
                try:
                    process.wait(timeout=KILL_TIMEOUT_SEC)
                except subprocess.TimeoutExpired:
                    if pgid is not None:
                        _killpg(pgid, signal.SIGKILL)
                    process.wait(timeout=KILL_TIMEOUT_SEC)
        if pgid is not None:
            _killpg(pgid, signal.SIGKILL)
    finally:
        if pgid is not None and process.poll() is not None:
            _PROCESS_GROUPS.discard(pgid)
            _PROCESS_GROUP_IDS.pop(process.pid, None)
        log_file.close()


def _killpg(pgid: int, signum: int) -> None:
    try:
        os.killpg(pgid, signum)
    except ProcessLookupError:
        pass


def _cleanup_process_groups() -> None:
    for pgid in list(_PROCESS_GROUPS):
        _killpg(pgid, signal.SIGKILL)


atexit.register(_cleanup_process_groups)


def _tail(path: Path, lines: int = 80) -> str:
    if not path.exists():
        return ''
    return ''.join(path.read_text(encoding='utf-8', errors='replace').splitlines(keepends=True)[-lines:])


def test_rail_target_follow(tmp_path):
    _write_simulation_config()

    domain_id = str(200 + (os.getpid() % 30))
    previous_domain_id = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = domain_id

    sim_log = tmp_path / 'sim.log'
    nav_log = tmp_path / 'nav.log'

    # Simulation runs in the "sim" pixi environment (ROS Kilted / MuJoCo).
    # We use PIXI_EXE (set by pixi when invoking a task) so the binary is found
    # even when "pixi" is not explicitly on PATH.
    pixi_exe = os.environ.get('PIXI_EXE', 'pixi')
    sim_pythonpath = os.pathsep.join(
        part for part in [str(SIM_PACKAGE_ROOT), os.environ.get('PYTHONPATH', '')] if part
    )
    sim_proc, sim_log_file = _start_process(
        cmd=[
            pixi_exe, 'run', '-e', 'sim',
            'python', '-m', 'simulation_package.start_simulation',
            '--config', str(TEST_CONFIG_PATH),
        ],
        log_path=sim_log,
        domain_id=domain_id,
        extra_env={'PYTHONPATH': sim_pythonpath},
    )

    # Nav logic runs in the current environment (ROS Jazzy, nav-test), which already
    # has the nav install space on PATH via the global activation script.
    logic_args = {
        'enable_heightmap': 'true',
        'cloud_topic': '/mid360/points',
        'follow_distance': '1.5',
        'min_linear_x': '0.35',
        'max_linear_x': '0.45',
        'stale_timeout_sec': '0.75',
        **{k: str(v) for k, v in artefacts_params.items() if k in _LOGIC_LAUNCH_PARAMS},
    }
    nav_proc, nav_log_file = _start_process(
        cmd=(
            ['ros2', 'launch', 'rail_inspector', 'rail_target_follow.launch.py']
            + [f'{k}:={v}' for k, v in logic_args.items()]
        ),
        log_path=nav_log,
        domain_id=domain_id,
    )

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_rail_target_follow', context=context)
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

    failure_msg = None
    try:
        while time.monotonic() < wall_deadline and state['distance_m'] < min_distance_to_travel:
            if sim_proc.poll() is not None:
                failure_msg = (
                    f'simulation exited unexpectedly with code {sim_proc.returncode}; '
                    f'distance so far {state["distance_m"]:.3f} m\n{_tail(sim_log)}'
                )
                break
            if nav_proc.poll() is not None:
                failure_msg = (
                    f'nav logic exited unexpectedly with code {nav_proc.returncode}; '
                    f'distance so far {state["distance_m"]:.3f} m\n{_tail(nav_log)}'
                )
                break
            first_stamp_sec = state['first_stamp_sec']
            last_stamp_sec = state['last_stamp_sec']
            if first_stamp_sec is not None and last_stamp_sec is not None:
                if (last_stamp_sec - first_stamp_sec) >= TEST_TIMEOUT_SEC:
                    break
            rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
    finally:
        executor.shutdown(timeout_sec=0.0)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown(context=context)
        _stop_process(nav_proc, nav_log_file)
        _stop_process(sim_proc, sim_log_file)
        if previous_domain_id is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous_domain_id

    if failure_msg is not None:
        pytest.fail(failure_msg)

    elapsed_sec = 0.0
    if state['first_stamp_sec'] is not None and state['last_stamp_sec'] is not None:
        elapsed_sec = state['last_stamp_sec'] - state['first_stamp_sec']

    assert state['distance_m'] >= min_distance_to_travel, (
        f"robot travelled {state['distance_m']:.3f} m in {elapsed_sec:.1f} sim s; "
        f"expected at least {min_distance_to_travel:.3f} m "
        f"from {state['message_count']} odom messages, last_xy={state['last_xy']}"
    )

    assert TEST_VIDEO_PATH.exists(), f"expected follow-camera video at {TEST_VIDEO_PATH}"
    assert TEST_VIDEO_PATH.stat().st_size > 1024, (
        f"follow-camera video at {TEST_VIDEO_PATH} is empty or too small"
    )
