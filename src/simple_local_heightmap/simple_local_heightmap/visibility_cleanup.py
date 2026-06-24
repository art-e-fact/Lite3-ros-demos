"""Line-of-sight cleanup for 2.5D local heightmaps."""

import math

import numpy as np
from numba import njit


@njit(cache=True)
def _clear_ray(
    elevation,
    valid,
    scan_observed,
    row0,
    col0,
    sx,
    sy,
    sz,
    row1,
    col1,
    hz,
    x_min,
    y_min,
    res,
    width,
    height,
    tolerance,
):
    """Walk a 2D Bresenham line and drop cells that block the ray to the hit."""
    hx = x_min + (col1 + 0.5) * res
    hy = y_min + (row1 + 0.5) * res
    dx = hx - sx
    dy = hy - sy
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 1e-4:
        return

    dz = hz - sz
    dr = abs(row1 - row0)
    dc = abs(col1 - col0)
    sr = 1 if row0 < row1 else -1
    sc = 1 if col0 < col1 else -1
    err = dr - dc
    row, col = row0, col0

    while not (row == row1 and col == col1):
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            row += sr
        if e2 < dr:
            err += dr
            col += sc
        if row == row1 and col == col1:
            break
        if row < 0 or row >= height or col < 0 or col >= width:
            continue
        if scan_observed[row, col] or not valid[row, col]:
            continue

        cx = x_min + (col + 0.5) * res
        cy = y_min + (row + 0.5) * res
        along = math.sqrt((cx - sx) ** 2 + (cy - sy) ** 2) / dist
        z_ray = sz + along * dz
        if elevation[row, col] > z_ray + tolerance:
            elevation[row, col] = np.nan
            valid[row, col] = False


@njit(cache=True)
def _visibility_cleanup_core(
    elevation,
    valid,
    scan_observed,
    target_rows,
    target_cols,
    target_zs,
    row0,
    col0,
    sx,
    sy,
    sz,
    x_min,
    y_min,
    res,
    width,
    height,
    tolerance,
):
    for i in range(target_rows.shape[0]):
        _clear_ray(
            elevation,
            valid,
            scan_observed,
            row0,
            col0,
            sx,
            sy,
            sz,
            target_rows[i],
            target_cols[i],
            target_zs[i],
            x_min,
            y_min,
            res,
            width,
            height,
            tolerance,
        )


def apply_visibility_cleanup(
    elevation,
    valid,
    scan_heightmap,
    sensor_xyz,
    grid_origin_xy,
    resolution,
    tolerance,
):
    """Remove stale cells that float above rays from the sensor to this scan's hits."""
    scan_observed = np.isfinite(scan_heightmap)
    if not np.any(scan_observed):
        return

    rows, cols = np.nonzero(scan_observed)
    sx, sy, sz = sensor_xyz
    x_min, y_min = grid_origin_xy
    map_height, map_width = elevation.shape

    _visibility_cleanup_core(
        elevation,
        valid,
        scan_observed,
        rows.astype(np.int32),
        cols.astype(np.int32),
        scan_heightmap[scan_observed].astype(np.float32),
        int(math.floor((sy - y_min) / resolution)),
        int(math.floor((sx - x_min) / resolution)),
        np.float32(sx),
        np.float32(sy),
        np.float32(sz),
        np.float32(x_min),
        np.float32(y_min),
        np.float32(resolution),
        map_width,
        map_height,
        np.float32(tolerance),
    )
