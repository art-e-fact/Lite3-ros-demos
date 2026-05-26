"""Livox Mid360-style point cloud for Newton using custom ray batches."""

from pathlib import Path

import numpy as np
import warp as wp
from newton._src.sensors.sensor_tiled_camera import SensorTiledCamera
from sensor_msgs.msg import PointCloud2

from sensors.common.lidar import (
    MID360_FRAME_ID,
    MID360_FREQUENCY_HZ,
    MID360_MOUNT_SITE_NAME,
    MID360_PATTERN_FILE,
    MID360_RANGE_MAX,
    MID360_RANGE_MIN,
    MID360_SAMPLES_PER_SCAN,
    MID360_TOPIC,
    mid360_angles_to_dirs,
)
from sensors.common.pointcloud import make_xyz_pointcloud
from sensors.common.resources import MID360_XML_PATH
from sensors.common.transforms import make_transform, quat_from_matrix
from sensors.newton.geometry import (
    builder_shape_local_pose,
    camera_transforms,
    find_builder_body_index,
    find_builder_shape_index,
    find_site_index,
    site_local_pose,
    site_world_pose,
)
from sensors.newton.ray_buffers import rays_from_dirs
from simulation_config import Mid360Config, resolve_path


class NewtonMid360LidarSensor:
    @staticmethod
    def init_visuals(builder, config: Mid360Config | None = None):
        config = config or Mid360Config()
        if not MID360_XML_PATH.is_file():
            raise FileNotFoundError(f"Mid360 XML not found: {MID360_XML_PATH}")

        torso_body = find_builder_body_index(builder, "TORSO")
        mount_shape = find_builder_shape_index(builder, config.mount_site_name)
        mount_pos, mount_rot = builder_shape_local_pose(builder, mount_shape)
        builder.add_mjcf(
            str(MID360_XML_PATH),
            xform=wp.transform(mount_pos, quat_from_matrix(mount_rot)),
            parent_body=torso_body,
            floating=False,
            parse_sites=True,
            parse_visuals=True,
            parse_meshes=True,
        )

    def __init__(self, model, node, enabled: bool = False, pattern_path=None, samples_per_scan=MID360_SAMPLES_PER_SCAN, downsample=1, config: Mid360Config | None = None):
        self.config = config or Mid360Config(
            enabled=enabled,
            pattern_file=str(pattern_path or MID360_PATTERN_FILE),
            samples_per_scan=samples_per_scan,
            downsample=downsample,
        )
        self.model = model
        self.node = node
        self.enabled = self.config.enabled
        self.site_index = -1

        if not self.enabled:
            node.get_logger().info("[INFO] Newton Mid360 LiDAR disabled")
            return

        self.site_index = find_site_index(model, self.config.mount_site_name)
        self.body_id = int(model.shape_body.numpy()[self.site_index])
        self.shape_body = model.shape_body.numpy()
        self.samples_per_scan = int(self.config.samples_per_scan)
        self.downsample = max(1, int(self.config.downsample))
        self._pattern_index = 0

        path = resolve_path(self.config.pattern_file, must_exist=False)
        if not path.exists():
            path = Path(__file__).resolve().parents[1] / self.config.pattern_file
        self.ray_angles = np.load(path).astype(np.float32)
        if self.ray_angles.ndim != 2 or self.ray_angles.shape[1] != 2:
            raise ValueError("Mid360 pattern must have shape (N, 2) with theta, phi columns")

        self.ray_count = (self.samples_per_scan + self.downsample - 1) // self.downsample
        self.sensor = SensorTiledCamera(model, load_textures=False)
        self.depth_image = self.sensor.utils.create_depth_image_output(self.ray_count, 1)
        self.shape_index_image = self.sensor.utils.create_shape_index_image_output(self.ray_count, 1)
        self.pub = node.create_publisher(PointCloud2, self.config.topic, 10)
        node.get_logger().info(
            f"[INFO] Newton Mid360 initialized ({self.ray_count} rays @ {self.config.frequency_hz} Hz, pattern: {path})"
        )

    def update(self, state, timestamp: float):
        if not self.enabled:
            return

        local_dirs = self._sample_dirs()
        rays = rays_from_dirs(local_dirs)
        site_pos, site_rot, _ = site_world_pose(self.model, state, self.site_index)
        transforms = camera_transforms(site_pos, site_rot, self.model.world_count)
        self.sensor.update(
            state,
            transforms,
            rays,
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

        points = (local_dirs[valid] * ranges[valid, None]).astype(np.float32)
        self.pub.publish(make_xyz_pointcloud(points, self.node.get_clock().now().to_msg(), self.config.frame_id))

    def _sample_dirs(self) -> np.ndarray:
        start = self._pattern_index
        stop = start + self.samples_per_scan
        indices = np.arange(start, stop, dtype=np.int64) % len(self.ray_angles)
        self._pattern_index = stop % len(self.ray_angles)
        angles = self.ray_angles[indices][:: self.downsample]
        return mid360_angles_to_dirs(angles[:, 0], angles[:, 1])

    def get_static_transforms(self, stamp):
        if not self.enabled or self.site_index < 0:
            return []
        site_pos_local, site_rot = site_local_pose(self.model, self.site_index)
        return [make_transform(stamp, "base_link", self.config.frame_id, site_pos_local, quat_from_matrix(site_rot))]
