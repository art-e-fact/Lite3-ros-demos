"""Newton IMU readings come from the physics, not from the base orientation alone.

An accelerometer measures specific force, so it reads ~0 in free fall, far above g on
impact, and exactly g at rest. A stand-in that only projects gravity into the body frame
reports a constant g through all three, which is why the magnitude is checked at each.
"""

import sys
from pathlib import Path

import numpy as np
from newton._src.utils.selection import match_labels

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "simulation_package"))
sys.path.insert(0, str(PACKAGE_ROOT / "simulation_package" / "newton_backend"))

from simulation import (  # noqa: E402
    BASE_DOF_COUNT,
    ROBOT_PROFILES,
    NewtonSimulation,
    rotate_world_to_body,
    warp_to_numpy,
)

MODEL_PATH = PACKAGE_ROOT / "assets" / "m20_mjcf" / "mjcf" / "M20.xml"
GRAVITY = 9.81


def _imu_site_quat(model) -> np.ndarray:
    """Orientation of imu_site within its body frame, xyzw."""
    (site_index,) = match_labels(model.shape_label, "*imu_site")
    return model.shape_transform.numpy()[site_index][3:7].astype(np.float32)


def test_newton_imu_reports_specific_force():
    sim = NewtonSimulation(model_path=str(MODEL_PATH), headless=True, profile=ROBOT_PROFILES["M20"])
    site_quat = _imu_site_quat(sim.model)

    samples = []
    for _ in range(750):  # 3 s at DT = 0.004: spawn, free fall, touchdown, settle
        sim.step()
        state = sim.state_snapshot()
        joint_qd = warp_to_numpy(sim.state_0.joint_qd, BASE_DOF_COUNT + sim.num_dofs)
        samples.append((state, rotate_world_to_body(state.quat_xyzw, joint_qd[3:6])))

    # The sensor reports in the imu_site frame, angvel_body in the base frame. A rotated site
    # describes the same turn on different axes, so rotate the base value into the site frame
    # first.
    #
    # Compare where the robot spins fastest, not at the end of the run: standing still it turns
    # at ~1e-5 rad/s and so does the expectation, so allclose(atol=1e-3) would pass for any
    # reading under 0.001 -- a broken sensor included. Settling peaks near 0.33 rad/s, 300x the
    # tolerance. The first assert is a tripwire: if the robot ever stops tumbling this hard, fail
    # loudly instead of silently testing nothing.
    state, angvel_body = max(samples, key=lambda sample: np.linalg.norm(sample[1]))
    assert np.linalg.norm(angvel_body) > 0.2, np.linalg.norm(angvel_body)
    assert np.allclose(state.imu_gyro, rotate_world_to_body(site_quat, angvel_body), atol=1e-3), (
        state.imu_gyro,
        angvel_body,
    )

    acc_norms = [float(np.linalg.norm(state.imu_acc)) for state, _ in samples]
    assert min(acc_norms) < 0.1 * GRAVITY, min(acc_norms)  # free fall after spawn
    assert max(acc_norms) > 1.5 * GRAVITY, max(acc_norms)  # touchdown

    # Resting on the ground the only specific force left is gravity, seen from the site frame.
    resting, _ = samples[-1]
    gravity_world = np.array([0.0, 0.0, GRAVITY], dtype=np.float32)
    gravity_in_site = rotate_world_to_body(site_quat, rotate_world_to_body(resting.quat_xyzw, gravity_world))
    assert np.allclose(resting.imu_acc, gravity_in_site, atol=0.05), (resting.imu_acc, gravity_in_site)
