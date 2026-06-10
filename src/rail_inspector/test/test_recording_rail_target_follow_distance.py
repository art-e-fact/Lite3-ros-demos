import math
from pathlib import Path
import pytest

# Gracefully skip this test if rerun is not installed in the current environment
rr = pytest.importorskip("rerun")

try:
    from artefacts_toolkit.config import get_artefacts_params
    artefacts_params = get_artefacts_params()
except Exception:
    artefacts_params = {}

RECORDING_PATH = Path("/home/azazdeaz/repos/art/Lite3-ros-demos/lite3_rail_target_follow_distance.rrd")


def test_recording_rail_target_follow():
    # Use the recording's expected minimum distance (1.5 m) as the default
    min_distance_to_travel = float(artefacts_params.get("min_distance_to_travel", 1.5))
    
    assert RECORDING_PATH.exists(), f"Recording not found at {RECORDING_PATH}"

    # Load recording and query TORSO translations
    with rr.server.Server(datasets={"recording": [str(RECORDING_PATH)]}) as server:
        dataset = server.client().get_dataset("recording")
        df = dataset.filter_contents(["/bodies/TORSO"]).reader(index="sim_time").to_pandas()
    
    translations_col = "/bodies/TORSO:Transform3D:translation"
    assert translations_col in df.columns, f"Could not find {translations_col} in recording"
    
    translations = df[translations_col].dropna().tolist()
    
    total_distance = 0.0
    prev_xy = None
    MAX_STEP_M = 1.0  # Filter out initial teleport/jump
    
    for t in translations:
        if len(t) == 0:
            continue
        coord = t[0]
        x, y = float(coord[0]), float(coord[1])
        
        if prev_xy is None:
            prev_xy = (x, y)
            continue
            
        step = math.hypot(x - prev_xy[0], y - prev_xy[1])
        if step <= MAX_STEP_M:
            total_distance += step
        prev_xy = (x, y)
        
    print(f"Total distance calculated: {total_distance:.4f} m (threshold: {min_distance_to_travel:.4f} m)")
    assert total_distance >= min_distance_to_travel, (
        f"Robot only travelled {total_distance:.3f} m in the recording; expected at least {min_distance_to_travel:.3f} m"
    )
