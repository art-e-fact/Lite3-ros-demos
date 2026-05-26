"""Newton model/site helpers for sensor placement and raytracing."""

from __future__ import annotations

import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation as R_scipy

try:
    from newton._src.utils.selection import match_labels
except Exception:  # Newton's helper location is stable in 1.2, but keep imports soft.
    match_labels = None


def _label_text(value) -> str:
    return value.decode() if isinstance(value, bytes) else str(value)


def _find_label_index(labels, label: str, kind: str) -> int:
    if match_labels is not None:
        matches = match_labels(labels, label)
        if matches:
            return int(matches[0])

    for index, value in enumerate(labels):
        text = _label_text(value)
        if text == label or text.endswith(label):
            return index
    raise ValueError(f"Newton {kind} '{label}' was not found")


def find_site_index(model, label: str) -> int:
    labels = list(model.shape_label) if getattr(model, "shape_label", None) is not None else []
    return _find_label_index(labels, label, "site")


def find_builder_shape_index(builder, label: str) -> int:
    return _find_label_index(builder.shape_label, label, "shape")


def find_builder_body_index(builder, label: str) -> int:
    return _find_label_index(builder.body_label, label, "body")


def transform_expand(transform_value) -> tuple[np.ndarray, np.ndarray]:
    values = np.asarray(transform_value, dtype=np.float64)
    return values[:3].copy(), values[3:7].copy()


def transform_matrix(transform_value) -> np.ndarray:
    _, quat_xyzw = transform_expand(transform_value)
    return R_scipy.from_quat(quat_xyzw).as_matrix()


def site_world_pose(model, state, site_index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    body_index = int(model.shape_body.numpy()[site_index])
    shape_transform = model.shape_transform.numpy()[site_index]
    site_local_pos, site_local_quat = transform_expand(shape_transform)

    if body_index >= 0:
        body_pos, body_quat = transform_expand(state.body_q.numpy()[body_index])
        body_rot = R_scipy.from_quat(body_quat).as_matrix()
        site_pos = body_pos + body_rot @ site_local_pos
        site_rot = body_rot @ R_scipy.from_quat(site_local_quat).as_matrix()
    else:
        site_pos = site_local_pos
        site_rot = R_scipy.from_quat(site_local_quat).as_matrix()

    return site_pos, site_rot, site_local_pos


def site_local_pose(model, site_index: int) -> tuple[np.ndarray, np.ndarray]:
    site_local_pos, site_local_quat = transform_expand(model.shape_transform.numpy()[site_index])
    return site_local_pos, R_scipy.from_quat(site_local_quat).as_matrix()


def builder_shape_local_pose(builder, shape_index: int) -> tuple[np.ndarray, np.ndarray]:
    shape_local_pos, shape_local_quat = transform_expand(builder.shape_transform[shape_index])
    return shape_local_pos, R_scipy.from_quat(shape_local_quat).as_matrix()


def warp_transform(position: np.ndarray, rotation_matrix: np.ndarray):
    quat_xyzw = R_scipy.from_matrix(rotation_matrix).as_quat()
    return wp.transformf(
        wp.vec3f(float(position[0]), float(position[1]), float(position[2])),
        wp.quat(float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2]), float(quat_xyzw[3])),
    )


def camera_transforms(position: np.ndarray, rotation_matrix: np.ndarray, world_count: int):
    transform = warp_transform(position, rotation_matrix)
    # SensorTiledCamera expects (camera_count, world_count).
    return wp.array([[transform for _ in range(world_count)]], dtype=wp.transformf)
