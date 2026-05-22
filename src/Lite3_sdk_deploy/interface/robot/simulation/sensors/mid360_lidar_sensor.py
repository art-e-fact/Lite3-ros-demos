"""
CPU-only Livox Mid360 simulation using MuJoCo ray casting.
Publishes sensor_msgs/msg/PointCloud2 from a precomputed mid360.npy angle table.
"""

from pathlib import Path

import mujoco
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import PointCloud2, PointField

MID360_ATTACHMENT_PREFIX = "mid360-"
MID360_MOUNT_SITE_NAME = "mid360_mount"
MID360_SENSOR_SITE_NAME = "lidar_frame"
MID360_SITE_NAME = f"{MID360_ATTACHMENT_PREFIX}{MID360_SENSOR_SITE_NAME}"
MID360_FRAME_ID = "mid360"
MID360_TOPIC = "/mid360/points"
MID360_FREQUENCY_HZ = 10.0
MID360_PATTERN_FILE = "mid360.npy"

SAMPLES_PER_SCAN = 24000
RANGE_MIN = 0.1
RANGE_MAX = 200.0


class Mid360LidarSensor:
    @staticmethod
    def configure_spec(spec, mid360_xml_path):
        """Attach the Mid360 body to the TORSO mid360_mount site."""
        mid360_spec = mujoco.MjSpec.from_file(mid360_xml_path)
        torso = spec.worldbody.first_body()
        mount_site = next(site for site in torso.sites if site.name == MID360_MOUNT_SITE_NAME)
        spec.attach(mid360_spec, prefix=MID360_ATTACHMENT_PREFIX, site=mount_site)

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node: Node,
        viewer=None,
        enabled: bool = False,
        pattern_path: str | Path | None = None,
        samples_per_scan: int = SAMPLES_PER_SCAN,
        downsample: int = 1,
    ):
        self.model = model
        self.data = data
        self.node = node
        self.viewer = viewer
        self.enabled = enabled
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, MID360_SITE_NAME)

        if not self.enabled:
            node.get_logger().info("[INFO] Mid360 LiDAR disabled")
            return

        if self.site_id < 0:
            raise ValueError(f"Site '{MID360_SITE_NAME}' not found in model")

        self.body_id = model.site_bodyid[self.site_id]
        self.geomgroup = np.ones(mujoco.mjNGROUP, dtype=np.uint8)
        self.samples_per_scan = samples_per_scan
        self.downsample = max(1, downsample)
        self._pattern_index = 0

        path = (
            Path(pattern_path)
            if pattern_path is not None
            else Path(__file__).with_name(MID360_PATTERN_FILE)
        )
        self.ray_angles = np.load(path).astype(np.float32)
        if self.ray_angles.ndim != 2 or self.ray_angles.shape[1] != 2:
            raise ValueError("Mid360 pattern must have shape (N, 2) with theta, phi columns")

        n_rays = (self.samples_per_scan + self.downsample - 1) // self.downsample
        self.distances = np.empty(n_rays, dtype=np.float64)
        self.geom_ids = np.empty(n_rays, dtype=np.int32)
        self.pub = node.create_publisher(PointCloud2, MID360_TOPIC, 10)

        node.get_logger().info(
            f"[INFO] Mid360 LiDAR initialized ({n_rays} rays @ {MID360_FREQUENCY_HZ} Hz, "
            f"pattern: {path})"
        )

    def update(self, timestamp: float):
        """Cast one Mid360 scan chunk and publish valid hit points."""
        if not self.enabled:
            return

        angles = self._sample_angles()
        local_dirs = self._angles_to_dirs(angles[:, 0], angles[:, 1])

        site_pos = self.data.site_xpos[self.site_id]
        site_rot = self.data.site_xmat[self.site_id].reshape(3, 3)
        world_dirs = (site_rot @ local_dirs.T).T

        self.distances.fill(RANGE_MAX)
        self.geom_ids.fill(-1)
        mujoco.mj_multiRay(
            self.model,
            self.data,
            pnt=site_pos,
            vec=world_dirs.ravel(),
            geomgroup=self.geomgroup,
            flg_static=1,
            bodyexclude=self.body_id,
            geomid=self.geom_ids,
            dist=self.distances,
            normal=None,
            nray=len(local_dirs),
            cutoff=RANGE_MAX,
        )

        valid = (
            (self.geom_ids != -1)
            & (self.distances >= RANGE_MIN)
            & (self.distances <= RANGE_MAX)
        )
        points = (local_dirs[valid] * self.distances[valid, None]).astype(np.float32)
        self._publish_pointcloud(points)

    def _sample_angles(self) -> np.ndarray:
        start = self._pattern_index
        stop = start + self.samples_per_scan
        indices = np.arange(start, stop, dtype=np.int64) % len(self.ray_angles)
        self._pattern_index = stop % len(self.ray_angles)
        return self.ray_angles[indices][:: self.downsample]

    @staticmethod
    def _angles_to_dirs(theta: np.ndarray, phi: np.ndarray) -> np.ndarray:
        cos_phi = np.cos(phi)
        return np.column_stack((
            cos_phi * np.cos(theta),
            cos_phi * np.sin(theta),
            np.sin(phi),
        )).astype(np.float64)

    def _publish_pointcloud(self, points: np.ndarray):
        msg = PointCloud2()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = MID360_FRAME_ID
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        msg.is_dense = True
        self.pub.publish(msg)

    def get_static_transforms(self, stamp):
        """Return base_link -> mid360 TF, read from MuJoCo model."""
        transforms = []
        if not self.enabled or self.site_id < 0:
            return transforms

        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "TORSO")
        torso_pos = self.data.xpos[torso_id]
        torso_rot = self.data.xmat[torso_id].reshape(3, 3)

        site_pos_w = self.data.site_xpos[self.site_id]
        site_rot_w = self.data.site_xmat[self.site_id].reshape(3, 3)
        pos_local = torso_rot.T @ (site_pos_w - torso_pos)
        rot_local = torso_rot.T @ site_rot_w
        quat_xyzw = R_scipy.from_matrix(rot_local).as_quat()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "base_link"
        transform.child_frame_id = MID360_FRAME_ID
        transform.transform.translation = Vector3(
            x=float(pos_local[0]), y=float(pos_local[1]), z=float(pos_local[2])
        )
        transform.transform.rotation = Quaternion(
            x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
            z=float(quat_xyzw[2]), w=float(quat_xyzw[3])
        )
        transforms.append(transform)
        return transforms