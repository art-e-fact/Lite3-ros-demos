"""Drive a recorded route with Nav2 on the Lite3, in a world extruded from the
route's map, and check that the robot got round.

    pixi run test-nav2-route            # or: pixi run -e nav-test pytest src/nav2_demo/test -s

The simulator (Newton by default) runs in its own pixi environment with the
robot spawned at the route's start; Nav2 and rl_deploy run from this
environment via nav2_route.launch.py; the test itself drives the route with
artefacts_toolkit_navigation.follow_route(). Params, also settable from
Artefacts scenarios: headless, simulator. Environment: ROUTE picks a route file
(default: the route recorded last, else the bundled depot route); RVIZ=true
opens Nav2's RViz view alongside (the simulator's own viewer follows headless).
"""

import os
from pathlib import Path

import pytest
from artefacts_toolkit_navigation import follow_route, load_route, map_to_world, route_launch_args

from sim_control_harness import SimControlHarness, StopReason

# Where the world, the simulator config and the route result go: Artefacts' upload
# directory when it sets one, else results/ (gitignored).
OUTPUT_FOLDER = Path(os.getenv('ARTEFACTS_SCENARIO_UPLOAD_DIR', 'results'))
REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SIM_PACKAGE_ROOT = REPO_ROOT / 'src' / 'simulation_package'
# The route to drive: ROUTE=... if given, else the route recorded last (artefacts-route
# record saves into ./routes), else the bundled depot route.
ROUTE = (os.environ.get('ROUTE') or (None if Path('routes').is_dir() else PACKAGE_ROOT / 'routes' / 'depot.yaml'))
RVIZ = os.environ.get('RVIZ', 'false').lower() in ('1', 'true', 'yes')

ROUTE_TIMEOUT_SEC = 300.0
NAV2_STARTUP_TIMEOUT_SEC = 180.0


def test_route_is_completed(tmp_path, simulator, headless):
    OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
    route = load_route(ROUTE)

    # The world: the route's map with its occupied cells raised into walls.
    world = map_to_world(REPO_ROOT / route.map.file, OUTPUT_FOLDER / f'{route.name}_world.xml',
                         format='mjcf')

    sim_config = {
        'simulator': simulator,
        'scene': world,
        'headless': headless,
        'robot': {'model': 'lite3'},
        'sensors': {'lidar_2d': {'enabled': True, 'range_max': 12.0}},
        'rerun': {'enabled': False},
    }
    # Spawn the robot where the route starts: the simulator's config fields,
    # named the way its --set overrides want them.
    start = route_launch_args(
        route, names=('robot.start_pose.x', 'robot.start_pose.y', 'robot.start_pose.yaw'),
        map_arg=None)
    sim_extra_args = [arg for key, value in start.items() for arg in ('--set', f'{key}={value}')]

    with SimControlHarness(
        sim_config,
        config_path=OUTPUT_FOLDER / f'{route.name}_{simulator}_sim.yaml',
        log_dir=tmp_path,
        repo_root=REPO_ROOT,
        sim_package_root=SIM_PACKAGE_ROOT,
        control_package='nav2_demo',
        control_launch_file='nav2_route.launch.py',
        control_launch_args={'use_rviz': str(RVIZ).lower(), **({'route': str(ROUTE)} if ROUTE else {})},
        sim_extra_args=sim_extra_args,
        max_runtime_sec=NAV2_STARTUP_TIMEOUT_SEC + ROUTE_TIMEOUT_SEC + 120.0,
    ) as harness:
        result = follow_route(
            route, timeout_s=ROUTE_TIMEOUT_SEC, startup_timeout_s=NAV2_STARTUP_TIMEOUT_SEC,
            label=f'{route.name}_{simulator}', results_dir=str(OUTPUT_FOLDER))
        reason = harness.stop_reason

    if reason is StopReason.SIM_EXITED:
        pytest.fail(f'simulation exited unexpectedly:\n{harness.sim_log_tail()}')
    if reason is StopReason.CONTROL_EXITED:
        pytest.fail(f'Nav2 launch exited unexpectedly:\n{harness.control_log_tail()}')

    assert result.succeeded, (
        f'{result.outcome}: {result.error}\n--- nav log ---\n{harness.control_log_tail(40)}')
    assert result.final_pos_error_m is not None and result.final_pos_error_m < 0.5, (
        f'ended {result.final_pos_error_m:.2f} m from the last waypoint')
