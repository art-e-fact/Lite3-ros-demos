"""
CPU-only Livox Mid360 simulation using MuJoCo ray casting.
Publishes sensor_msgs/msg/PointCloud2 from a precomputed angle table.
"""

from pathlib import Path

import mujoco
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import PointCloud2

from sensors.common.pointcloud import make_xyz_pointcloud
from sensors.common.resources import MID360_XML_PATH
from simulation_config import Mid360Config, resolve_path

MID360_FREQUENCY_HZ = 10.0


class Mid360LidarSensor:
    @staticmethod
    def init_visuals(spec, config: Mid360Config | None = None):
        config = config or Mid360Config()
        if not MID360_XML_PATH.is_file():
            raise FileNotFoundError(f"Mid360 XML not found: {MID360_XML_PATH}")
        mid360_spec = mujoco.MjSpec.from_file(str(MID360_XML_PATH))
        torso = spec.worldbody.first_body()
        mount_site = next(site for site in torso.sites if site.name == config.mount_site_name)
        spec.attach(mid360_spec, prefix=config.attachment_prefix, site=mount_site)

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node: Node,
        viewer=None,
        enabled: bool = False,
        pattern_path: str | Path | None = None,
        samples_per_scan: int = 24000,
        downsample: int = 1,
        config: Mid360Config | None = None,
    ):
        self.config = config or Mid360Config(
            enabled=enabled,
            pattern_file=str(pattern_path or "mid360.npy"),
            samples_per_scan=samples_per_scan,
            downsample=downsample,
        )
        self.model = model
        self.data = data
        self.node = node
        self.viewer = viewer
        self.enabled = self.config.enabled
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, self.config.attached_site_name)

        if not self.enabled:
            node.get_logger().info("[INFO] Mid360 LiDAR disabled")
            return

        if self.site_id < 0:
            raise ValueError(f"Site '{self.config.attached_site_name}' not found in model")

        self.body_id = model.site_bodyid[self.site_id]
        self.geomgroup = np.ones(mujoco.mjNGROUP, dtype=np.uint8)
        self.samples_per_scan = int(self.config.samples_per_scan)
        self.downsample = max(1, int(self.config.downsample))
        self._pattern_index = 0

        path = resolve_path(self.config.pattern_file, must_exist=False)
        if not path.exists():
            path = Path(__file__).resolve().parents[1] / self.config.pattern_file
        self.ray_angles = np.load(path).astype(np.float32)
        if self.ray_angles.ndim != 2 or self.ray_angles.shape[1] != 2:
            raise ValueError("Mid360 pattern must have shape (N, 2) with theta, phi columns")

        ray_count = (self.samples_per_scan + self.downsample - 1) // self.downsample
        self.distances = np.empty(ray_count, dtype=np.float64)
        self.geom_ids = np.empty(ray_count, dtype=np.int32)
        self.pub = node.create_publisher(PointCloud2, self.config.topic, 10)
        node.get_logger().info(
            f"[INFO] Mid360 LiDAR initialized ({ray_count} rays @ {self.config.frequency_hz} Hz, pattern: {path})"
        )

    def update(self, timestamp: float):
        if not self.enabled:
            return

        angles = self._sample_angles()
        local_dirs = self._angles_to_dirs(angles[:, 0], angles[:, 1])
        site_pos = self.data.site_xpos[self.site_id]
        site_rot = self.data.site_xmat[self.site_id].reshape(3, 3)
        world_dirs = (site_rot @ local_dirs.T).T

        self.distances.fill(self.config.range_max)
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
            cutoff=self.config.range_max,
        )

        valid = (
            (self.geom_ids != -1)
            & (self.distances >= self.config.range_min)
            & (self.distances <= self.config.range_max)
        )
        points = (local_dirs[valid] * self.distances[valid, None]).astype(np.float32)
        self.pub.publish(make_xyz_pointcloud(points, self.node.get_clock().now().to_msg(), self.config.frame_id))

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

    def get_static_transforms(self, stamp):
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
        transform.child_frame_id = self.config.frame_id
        transform.transform.translation = Vector3(
            x=float(pos_local[0]), y=float(pos_local[1]), z=float(pos_local[2])
        )
        transform.transform.rotation = Quaternion(
            x=float(quat_xyzw[0]), y=float(quat_xyzw[1]), z=float(quat_xyzw[2]), w=float(quat_xyzw[3])
        )
        transforms.append(transform)
        return transforms
