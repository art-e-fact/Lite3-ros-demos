from pathlib import Path
import os
import rerun as rr
import numpy as np
import pyarrow as pa
import pytest

from sim_control_harness import SimControlHarness, StopReason

OUTPUT_FOLDER = Path(os.getenv('ARTEFACTS_SCENARIO_UPLOAD_DIR', './test_outputs'))
OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)

try:
    from artefacts_toolkit.config import get_artefacts_params
    artefacts_params = get_artefacts_params()
except Exception:
    artefacts_params = {}

# Only these keys from artefacts_params are forwarded to the logic launch file.
_LOGIC_LAUNCH_PARAMS = {
    'follow_distance', 'min_linear_x', 'max_linear_x',
    'distance_error_for_max_speed', 'max_linear_y', 'max_angular_z',
    'k_center', 'k_heading', 'stale_timeout_sec',
}


@pytest.fixture(scope="module")
def follow_distance():
    return float(artefacts_params.get('follow_distance', 1.0))

# Set this to a specific path to reuse an existing recording instead of generating a new one each time. 
# (Useful for development/debugging to avoid long test runtimes, but should be None for CI runs to ensure fresh recordings.)
USE_RECORDING_PATH = None
# USE_RECORDING_PATH = OUTPUT_FOLDER / "lite3_recording_test.rrd"


@pytest.fixture(scope="module")
def recording_path(tmp_path_factory, follow_distance, headless, robot_profile):
    if USE_RECORDING_PATH is not None:
        return Path(USE_RECORDING_PATH)

    robot = robot_profile.name
    rrd_path = OUTPUT_FOLDER / f"{robot}_recording_test.rrd"
    video_path = OUTPUT_FOLDER / f"{robot}_recording_test.mp4"
    config_path = OUTPUT_FOLDER / f"{robot}_recording_test.yaml"

    for p in (rrd_path, video_path):
        if p.exists():
            p.unlink()

    sim_config = {
        'simulator': 'mujoco',
        'scene': 'procedural://railroad',
        'headless': headless,
        'procedural_env_seed': 123,
        'sensors': {
            **robot_profile.sensors,
            'follow_camera': {
                'enabled': True,
                'video_path': str(video_path),
                'target_height_m': robot_profile.follow_camera_target_height_m,
            },
        },
        'rerun': {
            'enabled': True,
            'spawn': not headless,
            'save_path': str(rrd_path),
        },
    }
    if robot_profile.sim_robot:
        sim_config['robot'] = robot_profile.sim_robot

    logic_args = {
        'enable_heightmap': 'true',
        'params_file': robot_profile.resolve_params_file(),
        'deploy_package': robot_profile.deploy_package,
        'deploy_args': '--twist',
        'follow_distance': str(follow_distance),
        'min_linear_x': '0.35',
        'max_linear_x': '0.45',
        'stale_timeout_sec': '0.75',
        **{k: str(v) for k, v in artefacts_params.items() if k in _LOGIC_LAUNCH_PARAMS},
    }

    repo_root = Path(__file__).resolve().parents[3]
    sim_package_root = repo_root / 'src' / 'simulation_package'
    min_distance = float(artefacts_params.get('min_distance_to_travel', 10.0))

    with SimControlHarness(
        sim_config,
        config_path=config_path,
        log_dir=tmp_path_factory.mktemp("harness_logs"),
        repo_root=repo_root,
        sim_package_root=sim_package_root,
        control_launch_args=logic_args,
        max_runtime_sec=300.0,
        sim_timeout_sec=120.0,
    ) as harness:
        reason = harness.wait(
            predicate=lambda: harness.total_distance_m >= min_distance,
        )
        if reason in (StopReason.SIM_EXITED, StopReason.CONTROL_EXITED):
            pytest.fail(
                f"Simulation/control exited unexpectedly (reason={reason}).\n"
                f"--- SIM LOG ---\n{harness.sim_log_tail()}\n"
                f"--- CONTROL LOG ---\n{harness.control_log_tail()}"
            )

    assert rrd_path.exists(), f"Recording not found at {rrd_path}"
    return rrd_path


@pytest.fixture(scope="module")
def dataset(recording_path):
    assert recording_path.exists(), f"Recording not found at {recording_path}"
    with rr.server.Server(datasets={"recording": [str(recording_path)]}) as server:
        yield server.client().get_dataset("recording")


def query_dataset(dataset, contents, step_ns=20_000_000):
    df_ranges = dataset.get_index_ranges().to_pandas()
    row = df_ranges.iloc[0]
    segment_id = row["rerun_segment_id"]

    start_ns = int(row["sim_time:start"].total_seconds() * 1e9)
    end_ns = int(row["sim_time:end"].total_seconds() * 1e9)

    times_ns = pa.array(range(start_ns, end_ns + step_ns, step_ns), type=pa.int64())

    return dataset.filter_contents(contents).reader(
        index="sim_time",
        using_index_values={segment_id: times_ns},
        fill_latest_at=True
    ).to_pandas()


def test_robot_is_travelling(dataset, robot_profile):
    body_col = robot_profile.body_translation_col()
    df = query_dataset(dataset, [robot_profile.body_path()])

    assert body_col in df.columns, f"Could not find {body_col} in recording"

    # Stack the 3D coordinates into a numpy array
    body_pts = np.vstack([t[0] for t in df[body_col]])

    # Calculate step-by-step distances in 2D (x, y)
    diffs = np.diff(body_pts[:, :2], axis=0)
    steps = np.linalg.norm(diffs, axis=1)

    # Filter out initial spawn teleport jump (> 1.0 m)
    MAX_STEP_M = 1.0
    total_distance = steps[steps <= MAX_STEP_M].sum()

    # Use baseline or configured threshold distance
    min_distance_to_travel = 8.5

    print(f"Total distance calculated: {total_distance:.4f} m (threshold: {min_distance_to_travel:.4f} m)")
    assert total_distance >= min_distance_to_travel, (
        f"Robot only travelled {total_distance:.3f} m in the recording; expected at least {min_distance_to_travel:.3f} m"
    )


def test_robot_keeps_max_distance_from_target(dataset, follow_distance, robot_profile):
    body_col = robot_profile.body_translation_col()
    df = query_dataset(dataset, [robot_profile.body_path(), "/bodies/uwb_tag"])

    uwb_col = "/bodies/uwb_tag:Transform3D:translation"
    assert body_col in df.columns, f"Could not find {body_col} in recording"
    assert uwb_col in df.columns, f"Could not find {uwb_col} in recording"

    # Stack resampled coordinates directly into numpy arrays (all elements align perfectly!)
    body_pts = np.vstack([t[0] for t in df[body_col]])
    uwb_pts = np.vstack([u[0] for u in df[uwb_col]])

    # Exclude initial uninitialized frames (spawn coordinates)
    valid_mask = (body_pts[:, 0] != 0.0) & (uwb_pts[:, 0] != 0.0)
    body_pts = body_pts[valid_mask]
    uwb_pts = uwb_pts[valid_mask]

    # Calculate distance between robot body and target UWB tag over time
    distances = np.linalg.norm(body_pts[:, :2] - uwb_pts[:, :2], axis=1)

    # Collect time-difference pairs: (sim_time as float seconds, distance as float)
    time_sec = df.iloc[np.flatnonzero(valid_mask)]["sim_time"].dt.total_seconds().tolist()

    # Check that maximum distance limit (default 2.5m) was never exceeded
    max_distance_limit = float(artefacts_params.get("max_distance_limit", 2.5))
    max_measured_distance = distances.max() if len(distances) > 0 else 0.0

    # Save distance over time plot using Plotly
    if len(distances) > 0:
        import plotly.graph_objects as go
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=time_sec,
            y=distances,
            mode='lines',
            name='Distance',
            line=dict(color='royalblue', width=2)
        ))
        fig.add_hline(
            y=max_distance_limit,
            line_dash="dash",
            line_color="red",
            line_width=2,
            annotation_text=f"Max Limit ({max_distance_limit}m)",
            annotation_position="top left"
        )
        fig.add_hline(
            y=follow_distance,
            line_dash="dash",
            line_color="green",
            line_width=2,
            annotation_text=f"Target Follow Distance ({follow_distance}m)",
            annotation_position="bottom left"
        )
        fig.update_layout(
            title="Robot-to-Target Distance Over Time",
            xaxis_title="Time (seconds)",
            yaxis_title="Distance (meters)",
            template="plotly_white"
        )
        OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
        fig.write_html(OUTPUT_FOLDER / f"distance_to_target_{robot_profile.name}.html")

    print(f"Max measured distance from target: {max_measured_distance:.4f} m (limit: {max_distance_limit:.4f} m)")
    assert max_measured_distance <= max_distance_limit, (
        f"Robot exceeded max distance limit of {max_distance_limit} m (measured {max_measured_distance:.3f} m)"
    )


def test_robot_keeps_close_to_rail_center(dataset, robot_profile):
    body_col = robot_profile.body_translation_col()
    df = query_dataset(dataset, [robot_profile.body_path(), "/network/mission_waypoints"])

    wp_col = "/network/mission_waypoints:Points3D:positions"
    assert body_col in df.columns, f"Could not find {body_col} in recording"
    assert wp_col in df.columns, f"Could not find {wp_col} in recording"

    # Extract the static waypoints from the last valid row
    valid_wp_rows = df[df[wp_col].notna()]
    if len(valid_wp_rows) == 0:
        pytest.fail("No waypoints found in `/network/mission_waypoints`")

    wps_raw = valid_wp_rows[wp_col].iloc[-1]
    track = np.vstack(wps_raw)[:, :2]  # N x 2

    # Extract robot body positions
    body_pts = np.vstack([t[0] for t in df[body_col]])

    # Exclude initial uninitialized frames (spawn coordinates)
    valid_mask = (body_pts[:, 0] != 0.0)
    body_pts = body_pts[valid_mask]

    # Collect time-difference pairs: (sim_time as float seconds)
    time_sec = df.iloc[np.flatnonzero(valid_mask)]["sim_time"].dt.total_seconds().tolist()

    deviations = []
    for p in body_pts[:, :2]:
        # Compute distances to all points in track
        dist_sq = np.sum((track - p)**2, axis=1)
        # Find the indices of the two closest points (smallest distances)
        closest_idxs = np.argsort(dist_sq)[:2]
        A = track[closest_idxs[0]]
        B = track[closest_idxs[1]]

        # Calculate distance from p to segment AB
        ab = B - A
        ab_len_sq = np.sum(ab**2)
        if ab_len_sq < 1e-9:
            # A and B are essentially the same point
            dev = float(np.sqrt(dist_sq[closest_idxs[0]]))
        else:
            # Projection factor t
            t = np.dot(p - A, ab) / ab_len_sq
            # Clamp to segment
            t_clamped = np.clip(t, 0.0, 1.0)
            # Closest point on segment
            closest_point = A + t_clamped * ab
            dev = float(np.linalg.norm(p - closest_point))

        deviations.append(dev)

    deviations = np.array(deviations)

    max_deviation_limit = 0.20  # 20 cm
    max_measured_deviation = deviations.max() if len(deviations) > 0 else 0.0

    # Save deviation over time plot using Plotly
    if len(deviations) > 0:
        import plotly.graph_objects as go
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=time_sec,
            y=deviations,
            mode='lines',
            name='Deviation',
            line=dict(color='firebrick', width=2)
        ))
        fig.add_hline(
            y=max_deviation_limit,
            line_dash="dash",
            line_color="red",
            line_width=2,
            annotation_text=f"Max Limit ({max_deviation_limit}m)",
            annotation_position="top left"
        )
        fig.update_layout(
            title="Robot Deviation from Rail Center Over Time",
            xaxis_title="Time (seconds)",
            yaxis_title="Deviation (meters)",
            template="plotly_white"
        )
        OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
        fig.write_html(OUTPUT_FOLDER / f"deviation_from_rail_{robot_profile.name}.html")

    print(f"Max measured deviation from rail center: {max_measured_deviation:.4f} m (limit: {max_deviation_limit:.4f} m)")
    assert max_measured_deviation <= max_deviation_limit, (
        f"Robot exceeded max rail deviation limit of {max_deviation_limit} m (measured {max_measured_deviation:.3f} m)"
    )
