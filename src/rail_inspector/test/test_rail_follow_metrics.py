from pathlib import Path
import os
import rerun as rr
import numpy as np
import pyarrow as pa
import pytest

OUTPUT_FOLDER = Path(os.getenv('ARTEFACTS_SCENARIO_UPLOAD_DIR', './'))

try:
    from artefacts_toolkit.config import get_artefacts_params
    artefacts_params = get_artefacts_params()
except Exception:
    artefacts_params = {}

USE_RECORDING_PATH = None
# USE_RECORDING_PATH = OUTPUT_FOLDER / "lite3_rail_target_follow_distance_test.rrd"


@pytest.fixture(scope="module")
def dataset():
    assert RECORDING_PATH.exists(), f"Recording not found at {RECORDING_PATH}"
    with rr.server.Server(datasets={"recording": [str(RECORDING_PATH)]}) as server:
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


def test_recording_rail_target_follow(dataset):
    df = query_dataset(dataset, ["/bodies/TORSO"])
    
    translations_col = "/bodies/TORSO:Transform3D:translation"
    assert translations_col in df.columns, f"Could not find {translations_col} in recording"
    
    # Stack the 3D coordinates into a numpy array
    torso_pts = np.vstack([t[0] for t in df[translations_col]])
    
    # Calculate step-by-step distances in 2D (x, y)
    diffs = np.diff(torso_pts[:, :2], axis=0)
    steps = np.linalg.norm(diffs, axis=1)
    
    # Filter out initial spawn teleport jump (> 1.0 m)
    MAX_STEP_M = 1.0
    total_distance = steps[steps <= MAX_STEP_M].sum()
    
    # Use baseline or configured threshold distance
    min_distance_to_travel = float(artefacts_params.get("min_distance_to_travel", 1.5))
    
    print(f"Total distance calculated: {total_distance:.4f} m (threshold: {min_distance_to_travel:.4f} m)")
    assert total_distance >= min_distance_to_travel, (
        f"Robot only travelled {total_distance:.3f} m in the recording; expected at least {min_distance_to_travel:.3f} m"
    )


def test_robot_keeps_max_distance_from_target(dataset):
    df = query_dataset(dataset, ["/bodies/TORSO", "/bodies/uwb_tag"])
    
    torso_col = "/bodies/TORSO:Transform3D:translation"
    uwb_col = "/bodies/uwb_tag:Transform3D:translation"
    assert torso_col in df.columns, f"Could not find {torso_col} in recording"
    assert uwb_col in df.columns, f"Could not find {uwb_col} in recording"
    
    # Stack resampled coordinates directly into numpy arrays (all elements align perfectly!)
    torso_pts = np.vstack([t[0] for t in df[torso_col]])
    uwb_pts = np.vstack([u[0] for u in df[uwb_col]])
    
    # Exclude initial uninitialized frames (spawn coordinates)
    valid_mask = (torso_pts[:, 0] != 0.0) & (uwb_pts[:, 0] != 0.0)
    torso_pts = torso_pts[valid_mask]
    uwb_pts = uwb_pts[valid_mask]
    
    # Calculate distance between robot TORSO and target UWB tag over time
    distances = np.linalg.norm(torso_pts[:, :2] - uwb_pts[:, :2], axis=1)
    
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
        fig.update_layout(
            title="Robot-to-Target Distance Over Time",
            xaxis_title="Time (seconds)",
            yaxis_title="Distance (meters)",
            template="plotly_white"
        )
        OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
        fig.write_html(OUTPUT_FOLDER / "distance_to_target.html")
    
    print(f"Max measured distance from target: {max_measured_distance:.4f} m (limit: {max_distance_limit:.4f} m)")
    assert max_measured_distance <= max_distance_limit, (
        f"Robot exceeded max distance limit of {max_distance_limit} m (measured {max_measured_distance:.3f} m)"
    )
