"""Shared LiDAR constants and ray pattern helpers."""

import math
import numpy as np

LIDAR_SITE_NAME = "lidar_site"
LIDAR_FRAME_ID = "lidar"
LIDAR_TOPIC = "/scan"
LIDAR_FREQUENCY_HZ = 10.0
NUM_RAYS = 360
RANGE_MIN = 0.15
RANGE_MAX = 8.0

MID360_MOUNT_SITE_NAME = "mid360_mount"
MID360_FRAME_ID = "mid360"
MID360_TOPIC = "/mid360/points"
MID360_FREQUENCY_HZ = 10.0
MID360_PATTERN_FILE = "mid360.npy"
MID360_SAMPLES_PER_SCAN = 24000
MID360_RANGE_MIN = 0.1
MID360_RANGE_MAX = 200.0


def planar_lidar_dirs(num_rays: int = NUM_RAYS) -> np.ndarray:
    angles = np.linspace(-math.pi, math.pi, num_rays, endpoint=False)
    return np.column_stack((
        np.cos(angles),
        np.sin(angles),
        np.zeros(num_rays),
    )).astype(np.float32)


def mid360_angles_to_dirs(theta: np.ndarray, phi: np.ndarray) -> np.ndarray:
    cos_phi = np.cos(phi)
    return np.column_stack((
        cos_phi * np.cos(theta),
        cos_phi * np.sin(theta),
        np.sin(phi),
    )).astype(np.float32)
