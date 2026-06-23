"""Rerun logging for local_heightmap GridMap and front-clear markers."""

from __future__ import annotations

import numpy as np
import rerun as rr
from grid_map_msgs.msg import GridMap
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

_FRONT_CLEAR_PATHS = ('heightmap/front_clear/fill', 'heightmap/front_clear/outline')


def log_local_heightmap(msg: GridMap, *, static: bool = False) -> None:
    """
    Convert a ``local_heightmap_node`` GridMap into a ``Boxes3D`` Rerun entity.

    One cube is logged per valid cell, coloured by elevation height.  Pass
    ``static=True`` to avoid recording the entity to the timeline (useful for
    live-only visualisation to keep recording sizes small).

    Args:
        msg: The incoming ``grid_map_msgs/GridMap`` message.
        static: Forward directly to ``rr.log`` as the ``static`` argument.
    """
    if "elevation" not in msg.layers:
        return

    time = Time.from_msg(msg.header.stamp)
    rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))

    rr.log("heightmap", rr.CoordinateFrame(frame=msg.header.frame_id), static=True)

    elevation = _decode_elevation(msg)
    if elevation is None:
        rr.log("heightmap", [], static=static)
        return

    h, w = elevation.shape
    resolution = msg.info.resolution
    x_min = msg.info.pose.position.x - 0.5 * msg.info.length_x
    y_min = msg.info.pose.position.y - 0.5 * msg.info.length_y

    xs = x_min + (np.arange(w, dtype=np.float32) + 0.5) * resolution
    ys = y_min + (np.arange(h, dtype=np.float32) + 0.5) * resolution
    xx, yy = np.meshgrid(xs, ys)  # (h, w)

    valid = np.isfinite(elevation)
    if not valid.any():
        rr.log("heightmap", [], static=static)
        return

    cx = xx[valid].ravel()
    cy = yy[valid].ravel()
    cz = elevation[valid].ravel()

    n = len(cx)
    half = np.float32(resolution * 0.5)
    half_sizes = np.full((n, 3), [half, half, half], dtype=np.float32)
    centers = np.stack([cx, cy, cz], axis=1)
    colors = _elevation_colors(cz, alpha=180)

    rr.log(
        "heightmap",
        rr.Boxes3D(
            half_sizes=half_sizes,
            centers=centers,
            colors=colors,
            fill_mode="solid",
        ),
        static=static,
    )


def log_front_clear_markers(msg: MarkerArray, front_clear_frame: str | None) -> str | None:
    """
    Convert ``/local_heightmap/front_clear_markers`` into Rerun entities.

    The producer publishes a semi-transparent CUBE plus a LINE_STRIP outline
    when front fast-clear is enabled, or only a DELETEALL sentinel otherwise.

    Args:
        msg: The incoming MarkerArray.
        front_clear_frame: The coordinate frame last pinned to the front-clear
            entities, or ``None`` if not yet initialised.

    Returns:
        The (possibly updated) ``front_clear_frame`` value.
    """
    ns_markers: dict[str, list[Marker]] = {}
    frame_id: str | None = None
    stamp = None

    for marker in msg.markers:
        if marker.action == Marker.DELETEALL:
            continue
        ns_markers.setdefault(marker.ns, []).append(marker)
        if stamp is None:
            frame_id = marker.header.frame_id
            stamp = marker.header.stamp

    if stamp is None:
        for path in _FRONT_CLEAR_PATHS:
            rr.log(path, [])
        return front_clear_frame

    time = Time.from_msg(stamp)
    rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))

    if frame_id and frame_id != front_clear_frame:
        front_clear_frame = frame_id
        for path in _FRONT_CLEAR_PATHS:
            rr.log(path, rr.CoordinateFrame(frame=frame_id), static=True)

    area_markers = ns_markers.get('front_clear_area', [])
    if area_markers:
        marker = area_markers[0]
        rr.log(
            'heightmap/front_clear/fill',
            rr.Boxes3D(
                half_sizes=[[marker.scale.x / 2, marker.scale.y / 2, marker.scale.z / 2]],
                centers=[[
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z,
                ]],
                rotations=rr.Quaternion(xyzw=[[
                    marker.pose.orientation.x,
                    marker.pose.orientation.y,
                    marker.pose.orientation.z,
                    marker.pose.orientation.w,
                ]]),
                colors=[[
                    round(marker.color.r * 255),
                    round(marker.color.g * 255),
                    round(marker.color.b * 255),
                    round(marker.color.a * 255),
                ]],
                fill_mode='solid',
            ),
        )
    else:
        rr.log('heightmap/front_clear/fill', [])

    outline_markers = ns_markers.get('front_clear_outline', [])
    if outline_markers and outline_markers[0].points:
        marker = outline_markers[0]
        pts = [[point.x, point.y, point.z] for point in marker.points]
        rr.log(
            'heightmap/front_clear/outline',
            rr.LineStrips3D(
                [pts],
                radii=marker.scale.x,
                colors=[[
                    round(marker.color.r * 255),
                    round(marker.color.g * 255),
                    round(marker.color.b * 255),
                    round(marker.color.a * 255),
                ]],
            ),
        )
    else:
        rr.log('heightmap/front_clear/outline', [])

    return front_clear_frame


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _decode_elevation(msg: GridMap) -> np.ndarray | None:
    """Return the elevation layer as a (height, width) float32 array."""
    layer_idx = msg.layers.index("elevation")
    layer = msg.data[layer_idx]

    w = round(msg.info.length_x / msg.info.resolution)
    h = round(msg.info.length_y / msg.info.resolution)

    if len(layer.data) != w * h:
        return None

    # The producer stores: np.flip(elevation, axis=(0,1)).reshape(-1)
    flat = np.array(layer.data, dtype=np.float32)
    return np.flip(flat.reshape(h, w), axis=(0, 1))


def _elevation_colors(elev: np.ndarray, alpha: int = 180) -> np.ndarray:
    """
    Blue→green→tan colour ramp over a flat elevation array.

    Returns an (N, 4) uint8 RGBA array.
    """
    z_min, z_max = float(elev.min()), float(elev.max())
    z_range = z_max - z_min if z_max > z_min else 1.0
    t = np.clip((elev - z_min) / z_range, 0.0, 1.0).astype(np.float32)

    r = np.where(t < 0.5, t * 2 * 80, 80 + (t - 0.5) * 2 * (175 - 80)).astype(np.uint8)
    g = np.where(t < 0.5, 80 + t * 2 * (160 - 80), 160 - (t - 0.5) * 2 * 40).astype(np.uint8)
    b = np.where(t < 0.5, 160 - t * 2 * 80, 80 - (t - 0.5) * 2 * 60).astype(np.uint8)
    a = np.full(len(elev), alpha, dtype=np.uint8)

    return np.stack([r, g, b, a], axis=1)
