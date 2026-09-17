"""PointCloud2 packing helpers for simulated depth and LiDAR sensors."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

XYZ_DTYPE = np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32)])
XYZ_POINT_STEP = 12

ROBOSENSE_POINT_STEP = 26  # x,y,z(f32) + intensity(f32) + ring(u16) + timestamp(f64)

def make_robosense_pointcloud(
    points: np.ndarray,      # (N,3) float32 xyz
    intensity: np.ndarray,   # (N,) float32
    ring: np.ndarray,        # (N,) uint16 — channel/row index
    point_time: np.ndarray,  # (N,) float64 — per-point timestamp (seconds)
    stamp,
    frame_id: str,
) -> PointCloud2:
    n = len(points)
    
    # [claude] NOTE: default struct packing would pad ring to align timestamp to 8 bytes,
    # producing point_step=32, not 26. Force the driver's tight layout explicitly:
    buf = np.empty(n, dtype=np.dtype({
        "names": ["x", "y", "z", "intensity", "ring", "timestamp"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
        "offsets": [0, 4, 8, 12, 16, 18],
        "itemsize": ROBOSENSE_POINT_STEP,
    }))
    buf["x"], buf["y"], buf["z"] = points[:, 0], points[:, 1], points[:, 2]
    buf["intensity"] = intensity
    buf["ring"] = ring
    buf["timestamp"] = point_time

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = ROBOSENSE_POINT_STEP
    msg.row_step = ROBOSENSE_POINT_STEP * n
    msg.data = buf.tobytes()
    msg.is_dense = False
    return msg

def make_xyz_pointcloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = XYZ_POINT_STEP
    msg.row_step = XYZ_POINT_STEP * msg.width
    msg.data = points.astype(np.float32, copy=False).tobytes()
    msg.is_dense = True
    return msg


def make_structured_xyz_pointcloud(x_coords, y_coords, z_coords, stamp, frame_id: str) -> PointCloud2:
    data = np.empty(len(z_coords), dtype=XYZ_DTYPE)
    data["x"] = x_coords
    data["y"] = y_coords
    data["z"] = z_coords

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(z_coords)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = XYZ_POINT_STEP
    msg.row_step = XYZ_POINT_STEP * msg.width
    msg.data = data.tobytes()
    msg.is_dense = True
    return msg
