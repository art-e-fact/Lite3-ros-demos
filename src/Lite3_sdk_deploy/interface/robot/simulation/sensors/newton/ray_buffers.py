"""Warp ray-buffer construction for Newton SensorTiledCamera."""

import numpy as np
import warp as wp


def rays_from_dirs(local_dirs: np.ndarray, height: int = 1):
    dirs = np.asarray(local_dirs, dtype=np.float32).reshape(height, -1, 3)
    origins = np.zeros_like(dirs, dtype=np.float32)
    rays = np.stack((origins, dirs), axis=2)
    rays = rays.reshape(1, height, dirs.shape[1], 2, 3)
    return wp.array(rays, dtype=wp.vec3f)
