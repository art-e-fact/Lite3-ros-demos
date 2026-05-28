import math

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def build_base_markers(frame_id, stamp, front_clear_area=None):
    msg = MarkerArray()
    msg.markers.append(_delete_all_marker(frame_id, stamp))
    if front_clear_area is not None:
        msg.markers.extend(_front_clear_markers(frame_id, stamp, front_clear_area))
    return msg


def _delete_all_marker(frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    return marker


def _front_clear_markers(frame_id, stamp, front_clear_area):
    origin = np.asarray(front_clear_area['origin'], dtype=np.float32)
    orientation = np.asarray(front_clear_area['orientation'], dtype=np.float32)
    offset_x = float(front_clear_area['offset_x'])
    length = float(front_clear_area['length'])
    width = float(front_clear_area['width'])

    center_local = np.array([[offset_x + 0.5 * length, 0.0, 0.0]], dtype=np.float32)
    corners_local = np.array([
        [offset_x, -0.5 * width, 0.0],
        [offset_x + length, -0.5 * width, 0.0],
        [offset_x + length, 0.5 * width, 0.0],
        [offset_x, 0.5 * width, 0.0],
        [offset_x, -0.5 * width, 0.0],
    ], dtype=np.float32)

    rotation = _quaternion_matrix(*orientation)
    world_center = (rotation @ center_local.T).T[0] + origin
    world_corners = (rotation @ corners_local.T).T + origin

    fill = Marker()
    fill.header.frame_id = frame_id
    fill.header.stamp = stamp
    fill.ns = 'front_clear_area'
    fill.id = 1
    fill.type = Marker.CUBE
    fill.action = Marker.ADD
    fill.scale.x = length
    fill.scale.y = width
    fill.scale.z = 0.02
    fill.color.r = 1.0
    fill.color.g = 0.45
    fill.color.b = 0.1
    fill.color.a = 0.18
    fill.pose.position.x = float(world_center[0])
    fill.pose.position.y = float(world_center[1])
    fill.pose.position.z = float(world_center[2]) + 0.01
    fill.pose.orientation.x = float(orientation[0])
    fill.pose.orientation.y = float(orientation[1])
    fill.pose.orientation.z = float(orientation[2])
    fill.pose.orientation.w = float(orientation[3])

    outline = Marker()
    outline.header.frame_id = frame_id
    outline.header.stamp = stamp
    outline.ns = 'front_clear_outline'
    outline.id = 2
    outline.type = Marker.LINE_STRIP
    outline.action = Marker.ADD
    outline.scale.x = 0.03
    outline.color.r = 1.0
    outline.color.g = 0.6
    outline.color.b = 0.2
    outline.color.a = 0.9
    outline.points = [_point(x, y, z + 0.03) for x, y, z in world_corners]
    return [fill, outline]


def _point(x, y, z):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)