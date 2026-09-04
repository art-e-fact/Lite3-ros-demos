"""Small transform helpers used by sensor backends."""

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from scipy.spatial.transform import Rotation as R_scipy

OPTICAL_QUAT_XYZW = (-0.5, 0.5, -0.5, 0.5)

# camera_link (X forward, Y left, Z up) -> Newton tiled camera
# (X right, Y up, -Z forward). Columns are tiled-camera axes in camera_link.
CAMERA_LINK_FROM_TILED_CAMERA = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
], dtype=np.float64)


def sim_time_stamp(timestamp: float) -> Time:
    """Build a ROS stamp from simulated seconds.

    Every message a simulator publishes must be stamped on the same clock it puts on
    ``/clock``; the navigation stack runs with ``use_sim_time`` and will not match a
    sensor stamped from the wall clock against a TF stamped from simulated time.
    """
    stamp = Time()
    seconds = int(timestamp)
    stamp.sec = seconds
    stamp.nanosec = int((timestamp - seconds) * 1e9)
    return stamp


def make_transform(stamp, parent: str, child: str, translation, quat_xyzw) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation = Vector3(
        x=float(translation[0]), y=float(translation[1]), z=float(translation[2])
    )
    transform.transform.rotation = Quaternion(
        x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
        z=float(quat_xyzw[2]), w=float(quat_xyzw[3])
    )
    return transform


def quat_from_matrix(matrix: np.ndarray) -> np.ndarray:
    return R_scipy.from_matrix(matrix).as_quat()


def matrix_from_quat(quat_xyzw: np.ndarray) -> np.ndarray:
    return R_scipy.from_quat(quat_xyzw).as_matrix()
