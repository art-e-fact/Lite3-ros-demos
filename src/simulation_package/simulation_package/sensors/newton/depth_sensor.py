"""RealSense-style D435i output for Newton using SensorTiledCamera."""

import newton
import numpy as np
import warp as wp
from newton._src.sensors.sensor_tiled_camera import SensorTiledCamera
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import Image, PointCloud2

from sensors.common.camera import (
    COLOR_CX,
    COLOR_CY,
    COLOR_FX,
    COLOR_FY,
    COLOR_IMAGE_TOPIC,
    COLOR_INFO_TOPIC,
    COLOR_OPTICAL_FRAME,
    DEPTH_CX,
    DEPTH_CY,
    DEPTH_FREQUENCY_HZ,
    DEPTH_FX,
    DEPTH_FY,
    DEPTH_IMAGE_TOPIC,
    DEPTH_INFO_TOPIC,
    DEPTH_OPTICAL_FRAME,
    DEPTH_RANGE_MAX,
    DEPTH_RANGE_MIN,
    D435I_FORWARD_OFFSET,
    D435I_MOUNT_SITE_NAME,
    HEIGHT,
    POINTCLOUD_TOPIC,
    WIDTH,
    make_camera_info,
)
from sensors.common.pointcloud import make_structured_xyz_pointcloud
from sensors.common.resources import D435I_XML_PATH
from sensors.common.transforms import CAMERA_LINK_FROM_TILED_CAMERA, OPTICAL_QUAT_XYZW, make_transform, quat_from_matrix, sim_time_stamp
from sensors.newton.geometry import (
    builder_shape_local_pose,
    camera_transforms,
    find_builder_body_index,
    find_builder_shape_index,
    find_site_index,
    site_local_pose,
    site_world_pose,
)
from simulation_config import RealsenseConfig

D435I_FORWARD_OFFSET_VEC = np.array(D435I_FORWARD_OFFSET, dtype=np.float64)
D435I_VISUAL_QUAT_XYZW = (0.5, 0.5, 0.5, 0.5)
D435I_VISUAL_MESHES = (
    ("d435i_0.obj", (0.035601, 0.035601, 0.035601)),
    ("d435i_1.obj", (0.287440, 0.665387, 0.327778)),
    ("d435i_2.obj", (0.799102, 0.806952, 0.799103)),
    ("d435i_3.obj", (0.035601, 0.035601, 0.035601)),
    ("d435i_4.obj", (0.296138, 0.296138, 0.296138)),
    ("d435i_5.obj", (0.070360, 0.070360, 0.070360)),
    ("d435i_6.obj", (0.070360, 0.070360, 0.070360)),
    ("d435i_7.obj", (0.087140, 0.002866, 0.009346)),
    ("d435i_8.obj", (1.0, 1.0, 1.0)),
)


def _vertical_fov_from_fy(fy: float, height: int) -> float:
    return float(2.0 * np.arctan((height * 0.5) / fy))


class NewtonDepthSensor:
    @staticmethod
    def init_visuals(builder, config: RealsenseConfig | None = None):
        config = config or RealsenseConfig()
        if not D435I_XML_PATH.is_file():
            raise FileNotFoundError(f"D435i XML not found: {D435I_XML_PATH}")

        try:
            import trimesh
        except ImportError as exc:
            raise ImportError("Newton D435i visuals require trimesh to load OBJ meshes") from exc

        torso_body = find_builder_body_index(builder, "TORSO")
        mount_shape = find_builder_shape_index(builder, config.mount_site_name)
        mount_pos, mount_rot = builder_shape_local_pose(builder, mount_shape)
        mount_pos = mount_pos + mount_rot @ np.array(config.forward_offset, dtype=np.float64)
        visual_rot = mount_rot @ R_scipy.from_quat(D435I_VISUAL_QUAT_XYZW).as_matrix()
        visual_xform = wp.transform(mount_pos, quat_from_matrix(visual_rot))

        visual_cfg = newton.ModelBuilder.ShapeConfig(
            density=0.0,
            has_shape_collision=False,
            has_particle_collision=False,
            collision_group=0,
            is_visible=True,
        )
        mesh_dir = D435I_XML_PATH.parent / "assets"
        for index, (mesh_name, color) in enumerate(D435I_VISUAL_MESHES):
            mesh_path = mesh_dir / mesh_name
            if not mesh_path.is_file():
                raise FileNotFoundError(f"D435i mesh not found: {mesh_path}")
            loaded_mesh = trimesh.load_mesh(mesh_path, process=False)
            mesh = newton.Mesh(
                loaded_mesh.vertices,
                loaded_mesh.faces.reshape(-1),
                compute_inertia=False,
                color=color,
            )
            builder.add_shape_mesh(
                body=torso_body,
                xform=visual_xform,
                mesh=mesh,
                cfg=visual_cfg,
                color=color,
                label=f"d435i_visual_{index}",
            )

    def __init__(self, model, node, enable_depth=False, enable_color=False, enable_pointcloud=False, config: RealsenseConfig | None = None):
        self.config = config or RealsenseConfig(
            enable_depth=enable_depth,
            enable_color=enable_color,
            enable_pointcloud=enable_pointcloud,
        )
        enable_depth = self.config.enable_depth
        enable_color = self.config.enable_color
        enable_pointcloud = self.config.enable_pointcloud
        if enable_pointcloud and not enable_depth:
            raise ValueError("enable_pointcloud requires enable_depth")

        self.model = model
        self.node = node
        self.enable_depth = enable_depth
        self.enable_color = enable_color
        self.enable_pointcloud = enable_pointcloud
        self.enabled = enable_depth or enable_color

        if not self.enabled:
            self.site_index = -1
            node.get_logger().info("[INFO] Newton D435i disabled")
            return

        self.site_index = find_site_index(model, self.config.mount_site_name)
        self.sensor = SensorTiledCamera(model, load_textures=enable_color)
        if enable_color:
            # Color rendering needs scene lighting; depth does not.
            self.sensor.utils.create_default_light(enable_shadows=True)
        self.depth_rays = self.sensor.utils.compute_pinhole_camera_rays(
            self.config.width, self.config.height, _vertical_fov_from_fy(self.config.depth_fy, self.config.height)
        )
        self.color_rays = self.sensor.utils.compute_pinhole_camera_rays(
            self.config.width, self.config.height, _vertical_fov_from_fy(self.config.color_fy, self.config.height)
        )
        self.depth_image = self.sensor.utils.create_depth_image_output(self.config.width, self.config.height)
        self.forward_depth_image = self.sensor.utils.create_depth_image_output(self.config.width, self.config.height)
        self.color_image = self.sensor.utils.create_color_image_output(self.config.width, self.config.height)

        self._depth_info = make_camera_info(
            self.config.depth_fx,
            self.config.depth_fy,
            self.config.depth_cx,
            self.config.depth_cy,
            self.config.width,
            self.config.height,
            self.config.depth_optical_frame,
        )
        self._color_info = make_camera_info(
            self.config.color_fx,
            self.config.color_fy,
            self.config.color_cx,
            self.config.color_cy,
            self.config.width,
            self.config.height,
            self.config.color_optical_frame,
        )

        pixel_x = np.arange(self.config.width, dtype=np.float32)
        pixel_y = np.arange(self.config.height, dtype=np.float32)
        self._u_grid, self._v_grid = np.meshgrid(pixel_x, pixel_y)
        self._x_factor = (self._u_grid - self.config.depth_cx) / self.config.depth_fx
        self._y_factor = (self._v_grid - self.config.depth_cy) / self.config.depth_fy

        if enable_depth:
            self.depth_image_pub = node.create_publisher(Image, self.config.depth_image_topic, 10)
            self.depth_info_pub = node.create_publisher(type(self._depth_info), self.config.depth_info_topic, 10)
        if enable_color:
            self.color_image_pub = node.create_publisher(Image, self.config.color_image_topic, 10)
            self.color_info_pub = node.create_publisher(type(self._color_info), self.config.color_info_topic, 10)
        if enable_pointcloud:
            self.pointcloud_pub = node.create_publisher(PointCloud2, self.config.pointcloud_topic, 10)

        enabled = [name for name, active in (("depth", enable_depth), ("color", enable_color), ("pointcloud", enable_pointcloud)) if active]
        node.get_logger().info(f"[INFO] Newton D435i initialized ({', '.join(enabled)})")

    def update(self, state, timestamp: float):
        if not self.enabled:
            return

        stamp = sim_time_stamp(timestamp)
        site_pos, site_rot, _ = site_world_pose(self.model, state, self.site_index)
        site_pos = site_pos + site_rot @ D435I_FORWARD_OFFSET_VEC
        tiled_rot = site_rot @ CAMERA_LINK_FROM_TILED_CAMERA
        transforms = camera_transforms(site_pos, tiled_rot, self.model.world_count)
        depth_m = None

        if self.enable_depth:
            self.sensor.update(state, transforms, self.depth_rays, depth_image=self.depth_image)
            self.forward_depth_image = self.sensor.utils.convert_ray_depth_to_forward_depth(
                self.depth_image, transforms, self.depth_rays, self.forward_depth_image
            )
            depth_m = self.forward_depth_image.numpy()[0, 0]
            valid = (depth_m >= self.config.depth_range_min) & (depth_m <= self.config.depth_range_max)
            depth_mm = np.zeros_like(depth_m, dtype=np.uint16)
            depth_mm[valid] = (depth_m[valid] * 1000.0).astype(np.uint16)

            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.config.depth_optical_frame
            msg.height = self.config.height
            msg.width = self.config.width
            msg.encoding = "16UC1"
            msg.is_bigendian = False
            msg.step = self.config.width * 2
            msg.data = depth_mm.tobytes()
            self.depth_image_pub.publish(msg)

            self._depth_info.header.stamp = stamp
            self.depth_info_pub.publish(self._depth_info)

        if self.enable_color:
            self.sensor.update(state, transforms, self.color_rays, color_image=self.color_image)
            rgba = self.color_image.numpy()[0, 0].view(np.uint8).reshape(self.config.height, self.config.width, 4)
            rgb = np.ascontiguousarray(rgba[:, :, :3])

            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.config.color_optical_frame
            msg.height = self.config.height
            msg.width = self.config.width
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = self.config.width * 3
            msg.data = rgb.tobytes()
            self.color_image_pub.publish(msg)

            self._color_info.header.stamp = stamp
            self.color_info_pub.publish(self._color_info)

        if self.enable_pointcloud and depth_m is not None:
            self._publish_pointcloud(depth_m, stamp)

    def _publish_pointcloud(self, depth_m, stamp):
        valid = (depth_m >= self.config.depth_range_min) & (depth_m <= self.config.depth_range_max)
        z_coords = depth_m[valid].astype(np.float32)
        x_coords = (self._x_factor[valid] * z_coords).astype(np.float32)
        y_coords = (self._y_factor[valid] * z_coords).astype(np.float32)
        self.pointcloud_pub.publish(
            make_structured_xyz_pointcloud(x_coords, y_coords, z_coords, stamp, self.config.depth_optical_frame)
        )

    def get_static_transforms(self, stamp):
        if not self.enabled or self.site_index < 0:
            return []

        mount_pos, mount_rot = site_local_pose(self.model, self.site_index)
        mount_pos = mount_pos + mount_rot @ np.array(self.config.forward_offset, dtype=np.float64)
        transforms = [
            make_transform(stamp, "base_link", self.config.camera_link_frame, mount_pos, quat_from_matrix(mount_rot)),
            make_transform(stamp, self.config.camera_link_frame, self.config.depth_frame, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            make_transform(stamp, self.config.depth_frame, self.config.depth_optical_frame, [0.0, 0.0, 0.0], OPTICAL_QUAT_XYZW),
            make_transform(stamp, self.config.camera_link_frame, self.config.color_frame, [0.0, 0.0, -0.015], [0.0, 0.0, 0.0, 1.0]),
            make_transform(stamp, self.config.color_frame, self.config.color_optical_frame, [0.0, 0.0, 0.0], OPTICAL_QUAT_XYZW),
        ]
        return transforms
