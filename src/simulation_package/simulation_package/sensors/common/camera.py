"""Shared camera constants and ROS message helpers."""

from sensor_msgs.msg import CameraInfo

WIDTH = 640
HEIGHT = 480

DEPTH_RANGE_MIN = 0.105
DEPTH_RANGE_MAX = 10.0

DEPTH_FX = 382.68
DEPTH_FY = 382.68
DEPTH_CX = 320.0
DEPTH_CY = 240.0

COLOR_FX = 615.69
COLOR_FY = 615.69
COLOR_CX = 320.0
COLOR_CY = 240.0

DEPTH_OPTICAL_FRAME = "camera_depth_optical_frame"
COLOR_OPTICAL_FRAME = "camera_color_optical_frame"

DEPTH_IMAGE_TOPIC = "/camera/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
COLOR_IMAGE_TOPIC = "/camera/color/image_raw"
COLOR_INFO_TOPIC = "/camera/color/camera_info"
POINTCLOUD_TOPIC = "/camera/depth/color/points"
DEPTH_FREQUENCY_HZ = 15.0

D435I_MOUNT_SITE_NAME = "d435i_mount"
D435I_FORWARD_OFFSET = (0.01, 0.0, 0.0)


def make_camera_info(fx, fy, cx, cy, width, height, frame_id):
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
