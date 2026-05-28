import math

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from simple_local_heightmap.heightmap_visualization import build_base_markers


class LocalHeightmapNode(Node):
    def __init__(self):
        super().__init__('local_heightmap_node')

        self.cloud_topic = self.declare_parameter('cloud_topic', '/mid360/points').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.output_topic = self.declare_parameter('heightmap_topic', '/local_heightmap').value
        self.debug_topic = self.declare_parameter(
            'debug_cloud_topic', '/local_heightmap/debug_points'
        ).value
        self.front_clear_marker_topic = '/local_heightmap/front_clear_markers'
        self.map_frame = self.declare_parameter('map_frame', 'odom').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.resolution = float(self.declare_parameter('resolution', 0.05).value)
        self.length_x = float(self.declare_parameter('length_x', 3.0).value)
        self.length_y = float(self.declare_parameter('length_y', 3.0).value)
        self.min_z = float(self.declare_parameter('min_z', -1.0).value)
        self.max_z = float(self.declare_parameter('max_z', 2.0).value)
        self.min_range = float(self.declare_parameter('min_range', 0.1).value)
        self.max_range = float(self.declare_parameter('max_range', 12.0).value)
        self.stale_time_sec = float(self.declare_parameter('stale_time_sec', 100.0).value)
        self.front_clear_enabled = bool(
            self.declare_parameter('front_clear_enabled', False).value
        )
        self.front_clear_length = float(
            self.declare_parameter('front_clear_length', 2.5).value
        )
        self.front_clear_width = float(
            self.declare_parameter('front_clear_width', 1.0).value
        )
        self.front_clear_offset_x = float(
            self.declare_parameter('front_clear_offset_x', 0.75).value
        )
        self.front_stale_time_sec = float(
            self.declare_parameter('front_stale_time_sec', 0.35).value
        )
        self.max_pose_variance = float(
            self.declare_parameter('max_pose_variance', 0.0).value
        )

        self.width = max(1, int(round(self.length_x / self.resolution)))
        self.height = max(1, int(round(self.length_y / self.resolution)))
        self.length_x = self.width * self.resolution
        self.length_y = self.height * self.resolution
        self.latest_pose_variance = 0.0
        self.center_x = None
        self.center_y = None
        self.elevation = np.full((self.height, self.width), np.nan, dtype=np.float32)
        self.valid = np.zeros((self.height, self.width), dtype=bool)
        self.last_seen = np.full((self.height, self.width), -np.inf, dtype=np.float64)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_pub = self.create_publisher(GridMap, self.output_topic, 10)
        self.debug_pub = self.create_publisher(PointCloud2, self.debug_topic, 10)
        self.front_clear_marker_pub = self.create_publisher(
            MarkerArray, self.front_clear_marker_topic, 10
        )
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            f'Publishing {self.length_x:.2f} x {self.length_y:.2f} m height maps at '
            f'{self.resolution:.3f} m/cell in {self.map_frame} from {self.cloud_topic}'
        )
        if self.front_clear_enabled:
            self.get_logger().info(
                'Front fast-clear enabled with '
                f'{self.front_clear_length:.2f} x {self.front_clear_width:.2f} m '
                f'rectangle, offset {self.front_clear_offset_x:.2f} m, '
                f'timeout {self.front_stale_time_sec:.2f} s on '
                f'{self.front_clear_marker_topic}'
            )

    def odom_callback(self, msg):
        cov = msg.pose.covariance
        self.latest_pose_variance = max(cov[0], cov[7], cov[35])

    def cloud_callback(self, msg):
        if self.max_pose_variance > 0.0:
            if self.latest_pose_variance > self.max_pose_variance:
                return

        stamp = msg.header.stamp
        scan_time = self._stamp_to_sec(stamp)

        points = self._cloud_to_array(msg)

        try:
            cloud_transform = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, stamp
            )
            robot_transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, stamp
            )
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=2.0)
            return

        self._shift_grid_if_needed(
            robot_transform.transform.translation.x,
            robot_transform.transform.translation.y,
        )

        if len(points) != 0:
            points = self._filter_points_by_range(points)
            if len(points) != 0:
                points = self._transform_points(points, cloud_transform)
                points = self._filter_points_by_height(points)
            if len(points) != 0:
                self._fuse_scan(self._rasterize(points), scan_time)

        self._expire_stale_cells(scan_time, robot_transform)
        self.map_pub.publish(self._to_grid_map(stamp))
        self.debug_pub.publish(self._to_debug_cloud(stamp))
        self.front_clear_marker_pub.publish(
            build_base_markers(
                self.map_frame,
                stamp,
                front_clear_area=self._front_clear_area(robot_transform),
            )
        )

    def _cloud_to_array(self, msg):
        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if isinstance(points, np.ndarray) and points.dtype.names:
            return np.column_stack((points['x'], points['y'], points['z'])).astype(np.float32)
        return np.asarray(list(points), dtype=np.float32).reshape(-1, 3)

    def _filter_points_by_range(self, points):
        ranges = np.linalg.norm(points, axis=1)
        valid = (
            (ranges >= self.min_range)
            & (ranges <= self.max_range)
        )
        return points[valid]

    def _filter_points_by_height(self, points):
        valid = (points[:, 2] >= self.min_z) & (points[:, 2] <= self.max_z)
        return points[valid]

    def _rasterize(self, points):
        x_min, y_min = self._grid_min_corner()
        cols = np.floor((points[:, 0] - x_min) / self.resolution).astype(np.int32)
        rows = np.floor((points[:, 1] - y_min) / self.resolution).astype(np.int32)
        inside = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)

        heightmap = np.full((self.height, self.width), np.nan, dtype=np.float32)
        rows = rows[inside]
        cols = cols[inside]
        zs = points[inside, 2]
        if len(zs) == 0:
            return heightmap

        # Median per cell keeps one robust surface estimate from each scan.
        linear = rows * self.width + cols
        unique_cells, inverse = np.unique(linear, return_inverse=True)
        for index, cell in enumerate(unique_cells):
            heightmap.flat[cell] = np.median(zs[inverse == index])
        return heightmap

    def _fuse_scan(self, scan_heightmap, scan_time):
        observed = np.isfinite(scan_heightmap)
        self.elevation[observed] = scan_heightmap[observed]
        self.valid[observed] = True
        self.last_seen[observed] = scan_time

    def _expire_stale_cells(self, scan_time, robot_transform):
        front_mask = self._front_clear_mask(robot_transform)
        if self.stale_time_sec <= 0.0 and front_mask is None:
            return

        age = scan_time - self.last_seen
        stale = np.zeros_like(self.valid, dtype=bool)
        if self.stale_time_sec > 0.0:
            if front_mask is None:
                stale = self.valid & (age > self.stale_time_sec)
            else:
                stale = self.valid & (~front_mask) & (age > self.stale_time_sec)
        if front_mask is not None:
            stale |= self.valid & front_mask & (age > self.front_stale_time_sec)
        self.elevation[stale] = np.nan
        self.valid[stale] = False

    def _front_clear_active(self):
        return (
            self.front_clear_enabled
            and self.front_stale_time_sec > 0.0
            and self.front_clear_length > 0.0
            and self.front_clear_width > 0.0
        )

    def _front_clear_mask(self, robot_transform):
        if not self._front_clear_active():
            return None

        x_min, y_min = self._grid_min_corner()
        cols = x_min + (np.arange(self.width, dtype=np.float32) + 0.5) * self.resolution
        rows = y_min + (np.arange(self.height, dtype=np.float32) + 0.5) * self.resolution
        xs, ys = np.meshgrid(cols, rows)

        translation = robot_transform.transform.translation
        rotation = robot_transform.transform.rotation
        rot = self._quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        relative = np.stack(
            (
                xs - np.float32(translation.x),
                ys - np.float32(translation.y),
                np.zeros_like(xs),
            ),
            axis=-1,
        )
        robot_frame = relative @ rot

        x_local = robot_frame[..., 0]
        y_local = robot_frame[..., 1]
        return (
            (x_local >= self.front_clear_offset_x)
            & (x_local <= self.front_clear_offset_x + self.front_clear_length)
            & (np.abs(y_local) <= 0.5 * self.front_clear_width)
        )

    def _front_clear_area(self, robot_transform):
        if not self._front_clear_active():
            return None

        translation = robot_transform.transform.translation
        rotation = robot_transform.transform.rotation
        return {
            'origin': np.array([translation.x, translation.y, translation.z], dtype=np.float32),
            'orientation': np.array(
                [rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float32
            ),
            'offset_x': self.front_clear_offset_x,
            'length': self.front_clear_length,
            'width': self.front_clear_width,
        }

    def _shift_grid_if_needed(self, robot_x, robot_y):
        if self.center_x is None or self.center_y is None:
            self.center_x = robot_x
            self.center_y = robot_y
            return

        shift_cols = int((robot_x - self.center_x) / self.resolution)
        shift_rows = int((robot_y - self.center_y) / self.resolution)
        if shift_cols == 0 and shift_rows == 0:
            return

        self.center_x += shift_cols * self.resolution
        self.center_y += shift_rows * self.resolution
        self.elevation = np.roll(self.elevation, shift=(-shift_rows, -shift_cols), axis=(0, 1))
        self.valid = np.roll(self.valid, shift=(-shift_rows, -shift_cols), axis=(0, 1))
        self.last_seen = np.roll(self.last_seen, shift=(-shift_rows, -shift_cols), axis=(0, 1))
        self._clear_shifted_regions(shift_rows, shift_cols)

    def _clear_shifted_regions(self, shift_rows, shift_cols):
        if shift_rows > 0:
            self._invalidate_slice(slice(-shift_rows, None), slice(None))
        elif shift_rows < 0:
            self._invalidate_slice(slice(None, -shift_rows), slice(None))

        if shift_cols > 0:
            self._invalidate_slice(slice(None), slice(-shift_cols, None))
        elif shift_cols < 0:
            self._invalidate_slice(slice(None), slice(None, -shift_cols))

    def _invalidate_slice(self, row_slice, col_slice):
        self.elevation[row_slice, col_slice] = np.nan
        self.valid[row_slice, col_slice] = False
        self.last_seen[row_slice, col_slice] = -np.inf

    def _grid_min_corner(self):
        return self.center_x - 0.5 * self.length_x, self.center_y - 0.5 * self.length_y

    def _to_grid_map(self, stamp):
        msg = GridMap()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.length_x = self.length_x
        msg.info.length_y = self.length_y
        msg.info.pose.position.x = 0.0 if self.center_x is None else self.center_x
        msg.info.pose.position.y = 0.0 if self.center_y is None else self.center_y
        msg.info.pose.orientation.w = 1.0
        msg.layers = ['elevation']
        msg.basic_layers = ['elevation']
        msg.data = [self._to_multi_array()]
        return msg

    def _to_multi_array(self):
        data = Float32MultiArray()
        data.layout.dim = [
            MultiArrayDimension(
                label='column_index', size=self.width, stride=self.width * self.height
            ),
            MultiArrayDimension(label='row_index', size=self.height, stride=self.height),
        ]
        data.data = np.flip(self.elevation, axis=(0, 1)).reshape(-1).astype(np.float32).tolist()
        return data

    def _to_debug_cloud(self, stamp):
        rows, cols = np.nonzero(self.valid)
        x_min, y_min = self._grid_min_corner()
        x = x_min + (cols + 0.5) * self.resolution
        y = y_min + (rows + 0.5) * self.resolution
        points = np.column_stack((x, y, self.elevation[rows, cols])).astype(np.float32)
        header = Header(stamp=stamp, frame_id=self.map_frame)
        return pc2.create_cloud_xyz32(header, points.tolist())

    @staticmethod
    def _stamp_to_sec(stamp):
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    @staticmethod
    def _transform_points(points, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot = LocalHeightmapNode._quaternion_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return (rot @ points.T).T + offset

    @staticmethod
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


def main(args=None):
    rclpy.init(args=args)
    node = LocalHeightmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
