"""
2D 360-degree LiDAR sensor simulation using MuJoCo ray casting.
Publishes sensor_msgs/msg/LaserScan on the configured scan topic.
"""

import math

import mujoco
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import LaserScan

from simulation_config import Lidar2DConfig

LIDAR_FREQUENCY_HZ = 10.0
VISUALIZE_RAYS = False
RAY_VIS_HIT_RGBA = np.array([0.0, 1.0, 0.0, 0.3], dtype=np.float32)
RAY_VIS_MISS_RGBA = np.array([0.0, 1.0, 0.0, 0.1], dtype=np.float32)
RAY_VIS_WIDTH = 0.002


class LidarSensor:
    @staticmethod
    def init_visuals(spec, config: Lidar2DConfig | None = None):
        config = config or Lidar2DConfig()
        torso = spec.worldbody.first_body()
        site = next(s for s in torso.sites if s.name == config.site_name)
        site.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        site.size = [0.025, 0.01, 0.0]
        site.rgba = [0.1, 0.1, 0.1, 1.0]
        site.group = 2

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node: Node,
        viewer=None,
        enabled: bool = False,
        config: Lidar2DConfig | None = None,
    ):
        self.config = config or Lidar2DConfig(enabled=enabled)
        self.model = model
        self.data = data
        self.node = node
        self.viewer = viewer
        self.enabled = self.config.enabled
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, self.config.site_name)

        if not self.enabled:
            node.get_logger().info("[INFO] LiDAR disabled")
            return

        if self.site_id < 0:
            raise ValueError(f"Site '{self.config.site_name}' not found in model")

        self.body_id = model.site_bodyid[self.site_id]
        angles = np.linspace(-math.pi, math.pi, self.config.num_rays, endpoint=False)
        self.local_dirs = np.column_stack([
            np.cos(angles),
            np.sin(angles),
            np.zeros(self.config.num_rays),
        ])
        self.geomgroup = np.ones(mujoco.mjNGROUP, dtype=np.uint8)
        self.distances = np.zeros(self.config.num_rays, dtype=np.float64)
        self.geom_ids = np.full(self.config.num_rays, -1, dtype=np.int32)
        self.pub = node.create_publisher(LaserScan, self.config.topic, 10)
        self.angle_increment = 2.0 * math.pi / self.config.num_rays
        self.scan_time = 1.0 / self.config.frequency_hz
        self.time_increment = self.scan_time / max(1, self.config.num_rays - 1)

    def update(self, timestamp: float):
        if not self.enabled:
            return

        site_pos = self.data.site_xpos[self.site_id]
        site_rot = self.data.site_xmat[self.site_id].reshape(3, 3)
        world_dirs = (site_rot @ self.local_dirs.T).T

        mujoco.mj_multiRay(
            self.model,
            self.data,
            pnt=site_pos,
            vec=world_dirs.flatten(),
            geomgroup=self.geomgroup,
            flg_static=1,
            bodyexclude=self.body_id,
            geomid=self.geom_ids,
            dist=self.distances,
            normal=None,
            nray=self.config.num_rays,
            cutoff=self.config.range_max,
        )

        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.config.frame_id
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = self.angle_increment
        msg.time_increment = self.time_increment
        msg.scan_time = self.scan_time
        msg.range_min = self.config.range_min
        msg.range_max = self.config.range_max

        ranges = self.distances.copy()
        ranges[ranges < 0] = float("inf")
        ranges[(ranges < self.config.range_min) | (ranges > self.config.range_max)] = float("inf")
        msg.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(msg)

        self._last_site_pos = site_pos.copy()
        self._last_world_dirs = world_dirs
        self._last_distances = self.distances.copy()

    def visualize(self):
        if not self.enabled or not self.config.visualize_rays or self.viewer is None:
            return
        if not hasattr(self, "_last_site_pos"):
            return

        scn = self.viewer.user_scn
        scn.ngeom = 0
        origin = self._last_site_pos
        for index in range(self.config.num_rays):
            if scn.ngeom >= scn.maxgeom:
                break
            distance = self._last_distances[index]
            if distance < 0:
                draw_dist = self.config.range_max
                rgba = RAY_VIS_MISS_RGBA
            else:
                draw_dist = distance
                rgba = RAY_VIS_HIT_RGBA

            end = origin + self._last_world_dirs[index] * draw_dist
            geom = scn.geoms[scn.ngeom]
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, RAY_VIS_WIDTH, origin, end)
            geom.rgba[:] = rgba
            scn.ngeom += 1

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
