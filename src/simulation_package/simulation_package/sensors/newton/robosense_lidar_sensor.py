"""RoboSense 96-line spinner LiDAR for Newton using SensorTiledCamera ray batches.

Mirrors sensors/mujoco/robosense_lidar_sensor.py: the M20 carries two units
(360 deg x 90 deg FOV) publishing sensor_msgs/msg/PointCloud2 in the sensor frame.
"""

import numpy as np
from newton._src.sensors.sensor_tiled_camera import SensorTiledCamera
from sensor_msgs.msg import PointCloud2

from sensors.common.pointcloud import make_xyz_pointcloud
from sensors.common.transforms import make_transform, quat_from_matrix
from sensors.newton.geometry import (
    camera_transforms,
    find_site_index,
    site_local_pose,
    site_world_pose,
)
from sensors.newton.ray_buffers import rays_from_dirs
from simulation_config import RobosenseConfig, RobosenseUnitConfig


def _spinner_dirs(config: RobosenseConfig) -> np.ndarray:
    cols = max(1, config.columns // max(1, int(config.column_downsample)))
    theta = np.linspace(-np.pi, np.pi, cols, endpoint=False, dtype=np.float64)
    phi = np.deg2rad(
        np.linspace(config.v_fov_deg[0], config.v_fov_deg[1], config.channels, dtype=np.float64)
    )
    theta_grid, phi_grid = np.meshgrid(theta, phi)
    theta_flat = theta_grid.ravel()
    phi_flat = phi_grid.ravel()
    cos_phi = np.cos(phi_flat)
    return np.column_stack((
        cos_phi * np.cos(theta_flat),
        cos_phi * np.sin(theta_flat),
        np.sin(phi_flat),
    )).astype(np.float32)


class NewtonRobosenseLidarSensor:
    """One spinning 96-line LiDAR attached to a Newton site shape."""

    def __init__(self, model, node, unit: RobosenseUnitConfig, config: RobosenseConfig):
        self.model = model
        self.node = node
        self.unit = unit
        self.config = config
        self.frame_id = unit.frame_id

        self.site_index = find_site_index(model, unit.site_name)
        self.shape_body = model.shape_body.numpy()
        self.body_id = int(self.shape_body[self.site_index])

        self.local_dirs = _spinner_dirs(config)
        self.rays = rays_from_dirs(self.local_dirs)
        self.ray_count = len(self.local_dirs)
        self.sensor = SensorTiledCamera(model, load_textures=False)
        self.depth_image = self.sensor.utils.create_depth_image_output(self.ray_count, 1)
        self.shape_index_image = self.sensor.utils.create_shape_index_image_output(self.ray_count, 1)
        self.pub = node.create_publisher(PointCloud2, unit.topic, 10)
        cols = self.ray_count // config.channels
        node.get_logger().info(
            f"[INFO] Newton RoboSense LiDAR '{unit.frame_id}' initialized "
            f"({config.channels}x{cols} = {self.ray_count} rays @ {config.frequency_hz} Hz -> {unit.topic})"
        )

    def update(self, state, timestamp: float):
        site_pos, site_rot, _ = site_world_pose(self.model, state, self.site_index)
        transforms = camera_transforms(site_pos, site_rot, self.model.world_count)
        self.sensor.update(
            state,
            transforms,
            self.rays,
            depth_image=self.depth_image,
            shape_index_image=self.shape_index_image,
        )

        ranges = self.depth_image.numpy()[0, 0, 0].astype(np.float32)
        shape_indices = self.shape_index_image.numpy()[0, 0, 0]
        valid = (ranges >= self.config.range_min) & (ranges <= self.config.range_max)

        valid_shape = shape_indices < len(self.shape_body)
        hit_bodies = np.full(shape_indices.shape, -9999, dtype=np.int32)
        hit_bodies[valid_shape] = self.shape_body[shape_indices[valid_shape].astype(np.int64)]
        valid &= hit_bodies != self.body_id

        points = (self.local_dirs[valid] * ranges[valid, None]).astype(np.float32)
        self.pub.publish(make_xyz_pointcloud(points, self.node.get_clock().now().to_msg(), self.frame_id))

    def get_static_transform(self, stamp, parent_frame: str = "base_link"):
        site_pos_local, site_rot = site_local_pose(self.model, self.site_index)
        return make_transform(stamp, parent_frame, self.frame_id, site_pos_local, quat_from_matrix(site_rot))


class NewtonRobosenseLidarSuite:
    """Front/rear RoboSense pair with staggered scan timing."""

    def __init__(self, model, node, dt: float, config: RobosenseConfig | None = None):
        self.config = config or RobosenseConfig()
        self.enabled = self.config.enabled
        self.lidars: list[NewtonRobosenseLidarSensor] = []
        self.step_interval = max(1, int(round(1.0 / (self.config.frequency_hz * dt))))

        if not self.enabled:
            node.get_logger().info("[INFO] Newton RoboSense LiDAR suite disabled")
            return

        units = [self.config.front]
        if self.config.enable_rear:
            units.append(self.config.rear)
        for unit in units:
            self.lidars.append(NewtonRobosenseLidarSensor(model, node, unit, self.config))

    def update(self, state, step_count: int, timestamp: float):
        if not self.enabled:
            return
        for index, lidar in enumerate(self.lidars):
            if (step_count + index * self.step_interval // 2) % self.step_interval == 0:
                lidar.update(state, timestamp)

    def any_due(self, step_count: int) -> bool:
        if not self.enabled:
            return False
        return any(
            (step_count + index * self.step_interval // 2) % self.step_interval == 0
            for index in range(len(self.lidars))
        )

    def get_static_transforms(self, stamp):
        if not self.enabled:
            return []
        return [lidar.get_static_transform(stamp) for lidar in self.lidars]
