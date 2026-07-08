"""
CPU-only RoboSense 96-line LiDAR simulation using MuJoCo ray casting.

The Lynx M20 carries two 96-line units (360 deg x 90 deg FOV). Each simulated
unit casts a mechanical-spinner ray grid from its mount site and publishes
sensor_msgs/msg/PointCloud2 in the sensor frame.
"""

import mujoco
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from sensors.common.pointcloud import make_xyz_pointcloud
from simulation_config import RobosenseConfig, RobosenseUnitConfig


class RobosenseLidarSensor:
    """One spinning 96-line LiDAR attached to a MuJoCo site."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node: Node,
        unit: RobosenseUnitConfig,
        config: RobosenseConfig,
    ):
        self.model = model
        self.data = data
        self.node = node
        self.unit = unit
        self.config = config
        self.frame_id = unit.frame_id
        self.range_min = float(config.range_min)
        self.range_max = float(config.range_max)

        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, unit.site_name)
        if self.site_id < 0:
            raise ValueError(f"Site '{unit.site_name}' not found in model")
        self.body_id = model.site_bodyid[self.site_id]
        self.geomgroup = np.ones(mujoco.mjNGROUP, dtype=np.uint8)

        cols = max(1, config.columns // max(1, int(config.column_downsample)))
        theta = np.linspace(-np.pi, np.pi, cols, endpoint=False, dtype=np.float64)
        phi = np.deg2rad(
            np.linspace(config.v_fov_deg[0], config.v_fov_deg[1], config.channels, dtype=np.float64)
        )
        theta_grid, phi_grid = np.meshgrid(theta, phi)
        theta_flat = theta_grid.ravel()
        phi_flat = phi_grid.ravel()
        cos_phi = np.cos(phi_flat)
        self.local_dirs = np.column_stack((
            cos_phi * np.cos(theta_flat),
            cos_phi * np.sin(theta_flat),
            np.sin(phi_flat),
        ))

        nray = len(self.local_dirs)
        self.distances = np.empty(nray, dtype=np.float64)
        self.geom_ids = np.empty(nray, dtype=np.int32)
        self.pub = node.create_publisher(PointCloud2, unit.topic, 10)
        node.get_logger().info(
            f"[INFO] RoboSense LiDAR '{unit.frame_id}' initialized "
            f"({config.channels}x{cols} = {nray} rays @ {config.frequency_hz} Hz -> {unit.topic})"
        )

    def update(self, stamp):
        site_pos = self.data.site_xpos[self.site_id]
        site_rot = self.data.site_xmat[self.site_id].reshape(3, 3)
        world_dirs = self.local_dirs @ site_rot.T

        self.distances.fill(self.range_max)
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
            nray=len(self.local_dirs),
            cutoff=self.range_max,
        )

        valid = (
            (self.geom_ids != -1)
            & (self.distances >= self.range_min)
            & (self.distances <= self.range_max)
        )
        points = (self.local_dirs[valid] * self.distances[valid, None]).astype(np.float32)
        self.pub.publish(make_xyz_pointcloud(points, stamp, self.frame_id))

    def get_static_transform(self, stamp, parent_frame: str = "base_link") -> TransformStamped:
        pos = self.model.site_pos[self.site_id]
        quat_wxyz = self.model.site_quat[self.site_id]

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = self.frame_id
        transform.transform.translation = Vector3(
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
        )
        transform.transform.rotation = Quaternion(
            x=float(quat_wxyz[1]),
            y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]),
            w=float(quat_wxyz[0]),
        )
        return transform


class RobosenseLidarSuite:
    """Front/rear RoboSense pair with staggered scan timing."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node: Node,
        config: RobosenseConfig | None = None,
        dt: float = 0.001,
    ):
        self.config = config or RobosenseConfig()
        self.enabled = self.config.enabled
        self.lidars: list[RobosenseLidarSensor] = []
        self.step_interval = max(1, int(round(1.0 / (self.config.frequency_hz * dt))))

        if not self.enabled:
            node.get_logger().info("[INFO] RoboSense LiDAR suite disabled")
            return

        units = [self.config.front]
        if self.config.enable_rear:
            units.append(self.config.rear)

        for unit in units:
            self.lidars.append(RobosenseLidarSensor(model, data, node, unit, self.config))

    def update(self, stamp, step: int):
        if not self.enabled:
            return
        for index, lidar in enumerate(self.lidars):
            if (step + index * self.step_interval // 2) % self.step_interval == 0:
                lidar.update(stamp)

    def get_static_transforms(self, stamp):
        if not self.enabled:
            return []
        return [lidar.get_static_transform(stamp) for lidar in self.lidars]
