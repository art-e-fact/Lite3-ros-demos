"""
Simulated Intel RealSense D435i depth camera using MuJoCo offscreen rendering.
Publishes depth image, color image, CameraInfo, and PointCloud2 on configured topics.
"""

import mujoco
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from sensors.common.camera import make_camera_info
from sensors.common.resources import D435I_XML_PATH
from simulation_config import RealsenseConfig

DEPTH_FREQUENCY_HZ = 15.0
DEPTH_CAMERA_NAME = "d435i-depth"
COLOR_CAMERA_NAME = "d435i-color"

_MJ_CAM_TO_CL = np.array([
    [0, -1, 0],
    [0, 0, 1],
    [-1, 0, 0],
], dtype=np.float64)
_OPT_QUAT_XYZW = (-0.5, 0.5, -0.5, 0.5)


class DepthSensor:
    @staticmethod
    def init_visuals(spec, config: RealsenseConfig | None = None):
        config = config or RealsenseConfig()
        if not D435I_XML_PATH.is_file():
            raise FileNotFoundError(f"D435i XML not found: {D435I_XML_PATH}")
        d435i_spec = mujoco.MjSpec.from_file(str(D435I_XML_PATH))
        torso = spec.worldbody.first_body()
        mount_site = next(site for site in torso.sites if site.name == config.mount_site_name)
        mount_site.pos = [
            mount_site.pos[0] + config.forward_offset[0],
            mount_site.pos[1] + config.forward_offset[1],
            mount_site.pos[2] + config.forward_offset[2],
        ]
        spec.attach(d435i_spec, prefix="d435i-", site=mount_site)

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node: Node,
        viewer=None,
        enable_depth: bool = False,
        enable_color: bool = False,
        enable_pointcloud: bool = False,
        config: RealsenseConfig | None = None,
    ):
        self.config = config or RealsenseConfig(
            enable_depth=enable_depth,
            enable_color=enable_color,
            enable_pointcloud=enable_pointcloud,
        )
        self.model = model
        self.data = data
        self.node = node
        self.enable_depth = self.config.enable_depth
        self.enable_color = self.config.enable_color
        self.enable_pointcloud = self.config.enable_pointcloud

        if self.enable_pointcloud and not self.enable_depth:
            raise ValueError("enable_pointcloud requires enable_depth")

        self.enabled = self.enable_depth or self.enable_color
        if not self.enabled:
            self.depth_cam_id = -1
            self.color_cam_id = -1
            node.get_logger().info("[INFO] D435i depth sensor disabled")
            return

        self.depth_cam_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, self.config.depth_camera_name
        )
        self.color_cam_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, self.config.color_camera_name
        )
        if self.enable_depth and self.depth_cam_id < 0:
            raise ValueError(f"Camera '{self.config.depth_camera_name}' not found in model")
        if self.enable_color and self.color_cam_id < 0:
            raise ValueError(f"Camera '{self.config.color_camera_name}' not found in model")

        self.depth_renderer = (
            mujoco.Renderer(model, height=self.config.height, width=self.config.width)
            if self.enable_depth else None
        )
        self.color_renderer = (
            mujoco.Renderer(model, height=self.config.height, width=self.config.width)
            if self.enable_color else None
        )

        if self.enable_depth:
            self._depth_info = make_camera_info(
                self.config.depth_fx,
                self.config.depth_fy,
                self.config.depth_cx,
                self.config.depth_cy,
                self.config.width,
                self.config.height,
                self.config.depth_optical_frame,
            )
        if self.enable_color:
            self._color_info = make_camera_info(
                self.config.color_fx,
                self.config.color_fy,
                self.config.color_cx,
                self.config.color_cy,
                self.config.width,
                self.config.height,
                self.config.color_optical_frame,
            )

        if self.enable_pointcloud:
            pixel_x = np.arange(self.config.width, dtype=np.float32)
            pixel_y = np.arange(self.config.height, dtype=np.float32)
            self._u_grid, self._v_grid = np.meshgrid(pixel_x, pixel_y)
            self._x_factor = (self._u_grid - self.config.depth_cx) / self.config.depth_fx
            self._y_factor = (self._v_grid - self.config.depth_cy) / self.config.depth_fy

        if self.enable_depth:
            self.depth_image_pub = node.create_publisher(Image, self.config.depth_image_topic, 10)
            self.depth_info_pub = node.create_publisher(CameraInfo, self.config.depth_info_topic, 10)
        if self.enable_color:
            self.color_image_pub = node.create_publisher(Image, self.config.color_image_topic, 10)
            self.color_info_pub = node.create_publisher(CameraInfo, self.config.color_info_topic, 10)
        if self.enable_pointcloud:
            self.pointcloud_pub = node.create_publisher(PointCloud2, self.config.pointcloud_topic, 10)

        enabled = [
            name for name, active in (
                ("depth", self.enable_depth),
                ("color", self.enable_color),
                ("pointcloud", self.enable_pointcloud),
            ) if active
        ]
        node.get_logger().info(
            f"[INFO] D435i depth sensor initialized "
            f"({self.config.width}x{self.config.height} @ {self.config.frequency_hz} Hz, "
            f"enabled: {', '.join(enabled) or 'none'})"
        )

    def update(self, timestamp: float):
        if not self.enabled:
            return
        stamp = self.node.get_clock().now().to_msg()
        depth_m = None

        if self.enable_depth:
            self.depth_renderer.update_scene(self.data, camera=self.config.depth_camera_name)
            self.depth_renderer.enable_depth_rendering()
            depth_m = self.depth_renderer.render().copy()
            self.depth_renderer.disable_depth_rendering()

            invalid = (depth_m < self.config.depth_range_min) | (depth_m > self.config.depth_range_max)
            depth_mm = (depth_m * 1000.0).astype(np.uint16)
            depth_mm[invalid] = 0

            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.config.depth_optical_frame
            depth_msg.height = self.config.height
            depth_msg.width = self.config.width
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = False
            depth_msg.step = self.config.width * 2
            depth_msg.data = depth_mm.tobytes()
            self.depth_image_pub.publish(depth_msg)

            self._depth_info.header.stamp = stamp
            self.depth_info_pub.publish(self._depth_info)

        if self.enable_color:
            self.color_renderer.update_scene(self.data, camera=self.config.color_camera_name)
            rgb_buf = self.color_renderer.render()

            color_msg = Image()
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = self.config.color_optical_frame
            color_msg.height = self.config.height
            color_msg.width = self.config.width
            color_msg.encoding = "rgb8"
            color_msg.is_bigendian = False
            color_msg.step = self.config.width * 3
            color_msg.data = rgb_buf.tobytes()
            self.color_image_pub.publish(color_msg)

            self._color_info.header.stamp = stamp
            self.color_info_pub.publish(self._color_info)

        if self.enable_pointcloud and depth_m is not None:
            self._publish_pointcloud(depth_m, stamp)

    def _publish_pointcloud(self, depth_m, stamp):
        valid = (depth_m >= self.config.depth_range_min) & (depth_m <= self.config.depth_range_max)
        z_coords = depth_m[valid].astype(np.float32)
        x_coords = (self._x_factor[valid] * z_coords).astype(np.float32)
        y_coords = (self._y_factor[valid] * z_coords).astype(np.float32)

        point_count = len(z_coords)
        point_step = 12
        data = np.empty(point_count, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
        data["x"] = x_coords
        data["y"] = y_coords
        data["z"] = z_coords

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.config.depth_optical_frame
        msg.height = 1
        msg.width = point_count
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * point_count
        msg.data = data.tobytes()
        msg.is_dense = True
        self.pointcloud_pub.publish(msg)

    def get_static_transforms(self, stamp):
        transforms = []
        if self.depth_cam_id < 0:
            return transforms

        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "TORSO")
        torso_pos = self.data.xpos[torso_id]
        torso_rot = self.data.xmat[torso_id].reshape(3, 3)
        depth_pos_w = self.data.cam_xpos[self.depth_cam_id]
        depth_rot_w = self.data.cam_xmat[self.depth_cam_id].reshape(3, 3)
        depth_pos_local = torso_rot.T @ (depth_pos_w - torso_pos)
        depth_rot_local = torso_rot.T @ depth_rot_w
        camera_link_rot = depth_rot_local @ _MJ_CAM_TO_CL
        camera_link_quat = R_scipy.from_matrix(camera_link_rot).as_quat()

        camera_link_tf = TransformStamped()
        camera_link_tf.header.stamp = stamp
        camera_link_tf.header.frame_id = "base_link"
        camera_link_tf.child_frame_id = self.config.camera_link_frame
        camera_link_tf.transform.translation = Vector3(
            x=float(depth_pos_local[0]), y=float(depth_pos_local[1]), z=float(depth_pos_local[2])
        )
        camera_link_tf.transform.rotation = Quaternion(
            x=float(camera_link_quat[0]), y=float(camera_link_quat[1]),
            z=float(camera_link_quat[2]), w=float(camera_link_quat[3])
        )
        transforms.append(camera_link_tf)

        depth_frame_tf = TransformStamped()
        depth_frame_tf.header.stamp = stamp
        depth_frame_tf.header.frame_id = self.config.camera_link_frame
        depth_frame_tf.child_frame_id = self.config.depth_frame
        depth_frame_tf.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        transforms.append(depth_frame_tf)

        depth_optical_tf = TransformStamped()
        depth_optical_tf.header.stamp = stamp
        depth_optical_tf.header.frame_id = self.config.depth_frame
        depth_optical_tf.child_frame_id = self.config.depth_optical_frame
        depth_optical_tf.transform.rotation = Quaternion(
            x=_OPT_QUAT_XYZW[0], y=_OPT_QUAT_XYZW[1], z=_OPT_QUAT_XYZW[2], w=_OPT_QUAT_XYZW[3]
        )
        transforms.append(depth_optical_tf)

        if self.color_cam_id >= 0:
            color_pos_w = self.data.cam_xpos[self.color_cam_id]
            color_pos_local = torso_rot.T @ (color_pos_w - torso_pos)
            offset_torso = color_pos_local - depth_pos_local
            offset_camera_link = camera_link_rot.T @ offset_torso

            color_frame_tf = TransformStamped()
            color_frame_tf.header.stamp = stamp
            color_frame_tf.header.frame_id = self.config.camera_link_frame
            color_frame_tf.child_frame_id = self.config.color_frame
            color_frame_tf.transform.translation = Vector3(
                x=float(offset_camera_link[0]), y=float(offset_camera_link[1]), z=float(offset_camera_link[2])
            )
            color_frame_tf.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            transforms.append(color_frame_tf)

            color_optical_tf = TransformStamped()
            color_optical_tf.header.stamp = stamp
            color_optical_tf.header.frame_id = self.config.color_frame
            color_optical_tf.child_frame_id = self.config.color_optical_frame
            color_optical_tf.transform.rotation = Quaternion(
                x=_OPT_QUAT_XYZW[0], y=_OPT_QUAT_XYZW[1], z=_OPT_QUAT_XYZW[2], w=_OPT_QUAT_XYZW[3]
            )
            transforms.append(color_optical_tf)

        return transforms
