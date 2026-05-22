"""
Simulated Intel RealSense D435i depth camera using MuJoCo offscreen rendering.
Publishes depth image, color image, CameraInfo, and PointCloud2 on standard
realsense-ros topic names.
"""

import math
import numpy as np
import mujoco

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from scipy.spatial.transform import Rotation as R_scipy

# -- D435i configuration --
DEPTH_CAMERA_NAME = "d435i-depth"
COLOR_CAMERA_NAME = "d435i-color"
DEPTH_FREQUENCY_HZ = 15.0

WIDTH = 640
HEIGHT = 480

# D435i depth specs
DEPTH_RANGE_MIN = 0.105  # metres
DEPTH_RANGE_MAX = 10.0   # metres

# D435i intrinsics (640x480)
# Depth: HFOV=87deg => fx = 320 / tan(87/2 * pi/180) = 382.68
DEPTH_FX = 382.68
DEPTH_FY = 382.68
DEPTH_CX = 320.0
DEPTH_CY = 240.0

# Color: HFOV=69deg => fx = 320 / tan(69/2 * pi/180) = 615.69
COLOR_FX = 615.69
COLOR_FY = 615.69
COLOR_CX = 320.0
COLOR_CY = 240.0

# Frame IDs (match realsense-ros defaults)
DEPTH_OPTICAL_FRAME = "camera_depth_optical_frame"
COLOR_OPTICAL_FRAME = "camera_color_optical_frame"

# Topic names (match realsense-ros defaults)
DEPTH_IMAGE_TOPIC = "/camera/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
COLOR_IMAGE_TOPIC = "/camera/color/image_raw"
COLOR_INFO_TOPIC = "/camera/color/camera_info"
POINTCLOUD_TOPIC = "/camera/depth/color/points"

# MuJoCo camera frame to ROS camera_link frame rotation matrix.
# MuJoCo camera: X=right, Y=up, Z=backward (-Z is forward).
# ROS camera_link (REP-103): X=forward, Y=left, Z=up.
_MJ_CAM_TO_CL = np.array([[0, -1, 0],
                            [0,  0, 1],
                            [-1, 0, 0]], dtype=np.float64)

# Standard optical rotation quaternion (camera_link -> optical frame).
# RPY(-pi/2, 0, -pi/2): maps X-fwd/Y-left/Z-up to X-right/Y-down/Z-fwd.
_OPT_QUAT_XYZW = (-0.5, 0.5, -0.5, 0.5)


def _make_camera_info(fx, fy, cx, cy, width, height, frame_id):
    """Build a CameraInfo message with pinhole intrinsics (no distortion)."""
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


class DepthSensor:
    @staticmethod
    def configure_spec(spec, d435i_xml_path):
        """Attach the D435i body to the TORSO d435i_mount site.

        Call on the MjSpec before compile() when any RealSense output
        (depth/color/pointcloud) is enabled.
        """
        d435i_spec = mujoco.MjSpec.from_file(d435i_xml_path)
        torso = spec.worldbody.first_body()
        mount_site = next(s for s in torso.sites if s.name == 'd435i_mount')
        # Move realsense a bit further forward to avoid capturing robot body
        mount_site.pos = [0.25 + 0.01, 0.0, 0.08]
        spec.attach(d435i_spec, prefix='d435i-', site=mount_site)

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 node: Node, viewer=None,
                 enable_depth: bool = False,
                 enable_color: bool = False,
                 enable_pointcloud: bool = False):
        self.model = model
        self.data = data
        self.node = node

        if enable_pointcloud and not enable_depth:
            raise ValueError("enable_pointcloud requires enable_depth")

        self.enable_depth = enable_depth
        self.enable_color = enable_color
        self.enable_pointcloud = enable_pointcloud
        self.enabled = enable_depth or enable_color

        if not self.enabled:
            self.depth_cam_id = -1
            self.color_cam_id = -1
            node.get_logger().info("[INFO] D435i depth sensor disabled")
            return

        # Look up camera IDs
        self.depth_cam_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, DEPTH_CAMERA_NAME)
        self.color_cam_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, COLOR_CAMERA_NAME)
        if self.enable_depth and self.depth_cam_id < 0:
            raise ValueError(f"Camera '{DEPTH_CAMERA_NAME}' not found in model")
        if self.enable_color and self.color_cam_id < 0:
            raise ValueError(f"Camera '{COLOR_CAMERA_NAME}' not found in model")

        # Offscreen renderers (only allocate what's needed)
        self.depth_renderer = (
            mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
            if self.enable_depth else None)
        self.color_renderer = (
            mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
            if self.enable_color else None)

        # Precompute constant CameraInfo messages
        if self.enable_depth:
            self._depth_info = _make_camera_info(
                DEPTH_FX, DEPTH_FY, DEPTH_CX, DEPTH_CY,
                WIDTH, HEIGHT, DEPTH_OPTICAL_FRAME)
        if self.enable_color:
            self._color_info = _make_camera_info(
                COLOR_FX, COLOR_FY, COLOR_CX, COLOR_CY,
                WIDTH, HEIGHT, COLOR_OPTICAL_FRAME)

        # Precompute pixel grid for pointcloud back-projection
        if self.enable_pointcloud:
            u = np.arange(WIDTH, dtype=np.float32)
            v = np.arange(HEIGHT, dtype=np.float32)
            self._u_grid, self._v_grid = np.meshgrid(u, v)
            self._x_factor = (self._u_grid - DEPTH_CX) / DEPTH_FX
            self._y_factor = (self._v_grid - DEPTH_CY) / DEPTH_FY

        # ROS publishers (only create what's needed)
        if self.enable_depth:
            self.depth_image_pub = node.create_publisher(Image, DEPTH_IMAGE_TOPIC, 10)
            self.depth_info_pub = node.create_publisher(CameraInfo, DEPTH_INFO_TOPIC, 10)
        if self.enable_color:
            self.color_image_pub = node.create_publisher(Image, COLOR_IMAGE_TOPIC, 10)
            self.color_info_pub = node.create_publisher(CameraInfo, COLOR_INFO_TOPIC, 10)
        if self.enable_pointcloud:
            self.pointcloud_pub = node.create_publisher(PointCloud2, POINTCLOUD_TOPIC, 10)

        enabled = [s for s, e in [('depth', self.enable_depth),
                                   ('color', self.enable_color),
                                   ('pointcloud', self.enable_pointcloud)] if e]
        node.get_logger().info(
            f"[INFO] D435i depth sensor initialized "
            f"({WIDTH}x{HEIGHT} @ {DEPTH_FREQUENCY_HZ} Hz, "
            f"enabled: {', '.join(enabled) or 'none'})")

    def update(self, timestamp: float):
        """Render depth + color and publish enabled topics."""
        if not self.enabled:
            return
        stamp = self.node.get_clock().now().to_msg()
        depth_m = None
        rgb_buf = None

        # --- Depth ---
        if self.enable_depth:
            self.depth_renderer.update_scene(self.data, camera=DEPTH_CAMERA_NAME)
            self.depth_renderer.enable_depth_rendering()
            depth_buf = self.depth_renderer.render()  # float32 (H, W), metres
            self.depth_renderer.disable_depth_rendering()

            depth_m = depth_buf.copy()
            invalid = (depth_m < DEPTH_RANGE_MIN) | (depth_m > DEPTH_RANGE_MAX)
            depth_mm = (depth_m * 1000.0).astype(np.uint16)
            depth_mm[invalid] = 0

            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = DEPTH_OPTICAL_FRAME
            depth_msg.height = HEIGHT
            depth_msg.width = WIDTH
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = False
            depth_msg.step = WIDTH * 2
            depth_msg.data = depth_mm.tobytes()
            self.depth_image_pub.publish(depth_msg)

            self._depth_info.header.stamp = stamp
            self.depth_info_pub.publish(self._depth_info)

        # --- Color ---
        if self.enable_color:
            self.color_renderer.update_scene(self.data, camera=COLOR_CAMERA_NAME)
            rgb_buf = self.color_renderer.render()  # uint8 (H, W, 3)

            color_msg = Image()
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = COLOR_OPTICAL_FRAME
            color_msg.height = HEIGHT
            color_msg.width = WIDTH
            color_msg.encoding = "rgb8"
            color_msg.is_bigendian = False
            color_msg.step = WIDTH * 3
            color_msg.data = rgb_buf.tobytes()
            self.color_image_pub.publish(color_msg)

            self._color_info.header.stamp = stamp
            self.color_info_pub.publish(self._color_info)

        # --- PointCloud2 ---
        if self.enable_pointcloud and depth_m is not None:
            self._publish_pointcloud(depth_m, stamp)

    def _publish_pointcloud(self, depth_m, stamp):
        """Back-project depth to 3D and publish PointCloud2 (XYZ only)."""
        # Mask valid depths
        valid = (depth_m >= DEPTH_RANGE_MIN) & (depth_m <= DEPTH_RANGE_MAX)
        z = depth_m[valid].astype(np.float32)
        x = (self._x_factor[valid] * z).astype(np.float32)
        y = (self._y_factor[valid] * z).astype(np.float32)

        # Build structured array: x, y, z (each float32)
        n_points = len(z)
        point_step = 12  # 3 fields * 4 bytes
        data = np.empty(n_points, dtype=[
            ('x', np.float32), ('y', np.float32),
            ('z', np.float32)])
        data['x'] = x
        data['y'] = y
        data['z'] = z

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = DEPTH_OPTICAL_FRAME
        msg.height = 1
        msg.width = n_points
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = point_step
        msg.row_step = point_step * n_points
        msg.data = data.tobytes()
        msg.is_dense = True

        self.pointcloud_pub.publish(msg)

    def get_static_transforms(self, stamp):
        """Return static TFs for camera frames, read from MuJoCo model."""
        transforms = []
        if self.depth_cam_id < 0:
            return transforms

        # TORSO = base_link
        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'TORSO')
        torso_pos = self.data.xpos[torso_id]
        torso_rot = self.data.xmat[torso_id].reshape(3, 3)

        # Depth camera in world frame
        depth_pos_w = self.data.cam_xpos[self.depth_cam_id]
        depth_rot_w = self.data.cam_xmat[self.depth_cam_id].reshape(3, 3)

        # Depth camera relative to TORSO
        depth_pos_local = torso_rot.T @ (depth_pos_w - torso_pos)
        depth_rot_local = torso_rot.T @ depth_rot_w

        # Convert MuJoCo camera frame to ROS camera_link frame
        cl_rot = depth_rot_local @ _MJ_CAM_TO_CL
        cl_quat = R_scipy.from_matrix(cl_rot).as_quat()  # [x, y, z, w]

        # base_link -> camera_link (at depth camera position)
        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'camera_link'
        t1.transform.translation = Vector3(
            x=float(depth_pos_local[0]),
            y=float(depth_pos_local[1]),
            z=float(depth_pos_local[2]))
        t1.transform.rotation = Quaternion(
            x=float(cl_quat[0]), y=float(cl_quat[1]),
            z=float(cl_quat[2]), w=float(cl_quat[3]))
        transforms.append(t1)

        # camera_link -> camera_depth_frame (identity)
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = 'camera_link'
        t2.child_frame_id = 'camera_depth_frame'
        t2.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        transforms.append(t2)

        # camera_depth_frame -> camera_depth_optical_frame
        t3 = TransformStamped()
        t3.header.stamp = stamp
        t3.header.frame_id = 'camera_depth_frame'
        t3.child_frame_id = DEPTH_OPTICAL_FRAME
        t3.transform.rotation = Quaternion(
            x=_OPT_QUAT_XYZW[0], y=_OPT_QUAT_XYZW[1],
            z=_OPT_QUAT_XYZW[2], w=_OPT_QUAT_XYZW[3])
        transforms.append(t3)

        # Color camera frames
        if self.color_cam_id >= 0:
            color_pos_w = self.data.cam_xpos[self.color_cam_id]
            color_pos_local = torso_rot.T @ (color_pos_w - torso_pos)
            offset_torso = color_pos_local - depth_pos_local
            offset_cl = cl_rot.T @ offset_torso

            # camera_link -> camera_color_frame
            t4 = TransformStamped()
            t4.header.stamp = stamp
            t4.header.frame_id = 'camera_link'
            t4.child_frame_id = 'camera_color_frame'
            t4.transform.translation = Vector3(
                x=float(offset_cl[0]),
                y=float(offset_cl[1]),
                z=float(offset_cl[2]))
            t4.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            transforms.append(t4)

            # camera_color_frame -> camera_color_optical_frame
            t5 = TransformStamped()
            t5.header.stamp = stamp
            t5.header.frame_id = 'camera_color_frame'
            t5.child_frame_id = COLOR_OPTICAL_FRAME
            t5.transform.rotation = Quaternion(
                x=_OPT_QUAT_XYZW[0], y=_OPT_QUAT_XYZW[1],
                z=_OPT_QUAT_XYZW[2], w=_OPT_QUAT_XYZW[3])
            transforms.append(t5)

        return transforms
