"""PointCloud2 packing helpers for simulated depth and LiDAR sensors."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

XYZ_DTYPE = np.dtype([("x", np.float32), ("y", np.float32), ("z", np.float32)])
XYZ_POINT_STEP = 12


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
