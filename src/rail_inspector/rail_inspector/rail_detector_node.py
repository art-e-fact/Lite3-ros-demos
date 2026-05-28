import math

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

from rail_inspector.rail_detector_visualization import build_markers


class RailDetectorNode(Node):
    def __init__(self):
        super().__init__('rail_detector_node')

        def declare(name, default, description):
            return self.declare_parameter(
                name,
                default,
                ParameterDescriptor(description=description),
            ).value

        self.heightmap_topic = declare(
            'heightmap_topic',
            '/local_heightmap',
            'Input GridMap topic with the local elevation layer.',
        )
        self.odom_topic = declare(
            'odom_topic',
            '/odom',
            'Odometry topic used for robot position and heading.',
        )
        self.marker_topic = declare(
            'marker_topic',
            '/rail_detector/markers',
            'Output MarkerArray topic for RViz debug markers.',
        )
        self.center_offset_topic = declare(
            'center_offset_topic',
            '/rail_detector/center_offset',
            'Output topic for the signed rail center offset in meters.',
        )
        self.tangent_yaw_topic = declare(
            'tangent_yaw_topic',
            '/rail_detector/tangent_yaw',
            'Output topic for the detected rail tangent yaw in radians.',
        )
        self.target_distance_topic = declare(
            'target_distance_topic',
            '/rail_detector/target_distance',
            'Output topic for the detected target distance in meters; negative means invalid.',
        )
        self.track_gauge = float(declare(
            'track_gauge',
            1.067,
            'Expected distance between the two rails in meters.',
        ))
        self.rail_width = float(declare(
            'rail_width',
            0.15,
            'Expected lateral width of one rail in meters.',
        ))
        self.gauge_tolerance = float(declare(
            'gauge_tolerance',
            0.40,
            'Maximum rail-pair gauge error allowed in one slice.',
        ))
        self.angle_sweep_deg = float(declare(
            'angle_sweep_deg',
            40.0,
            'Half-range of the center-slice heading search in degrees.',
        ))
        self.angle_step_deg = float(declare(
            'angle_step_deg',
            5.0,
            'Heading increment used during the center-slice search.',
        ))
        self.min_rail_height = float(declare(
            'min_rail_height',
            0.05,
            'Minimum height above the local baseline to accept a rail hit.',
        ))
        self.max_rail_height = float(declare(
            'max_rail_height',
            0.30,
            'Maximum height above the local baseline to accept a rail hit.',
        ))
        self.max_rail_height_difference = float(
            declare(
                'max_rail_height_difference',
                0.08,
                'Maximum height mismatch allowed between the left and right rail.',
            )
        )
        self.forward_span = float(declare(
            'forward_span',
            2.6,
            'Total forward span covered by the sampled rail slices.',
        ))
        self.num_slices = max(3, int(declare(
            'num_slices',
            15,
            'Number of cross-sections sampled across the forward span.',
        )))
        self.lateral_search_width = float(
            declare(
                'lateral_search_width',
                1.8,
                'Half-width of each sampled cross-section in meters.',
            )
        )
        self.follow_target_lookahead = float(declare(
            'follow_target_lookahead',
            8.0,
            'How far ahead along the detected rail center to search for a follow target.',
        ))
        self.follow_target_kernel_size = float(declare(
            'follow_target_kernel_size',
            0.35,
            'Width of the center sample window used to measure the follow target.',
        ))
        self.follow_target_sample_step = float(declare(
            'follow_target_sample_step',
            0.10,
            'Distance between follow-target samples along the rail center.',
        ))
        self.follow_target_min_height = float(declare(
            'follow_target_min_height',
            0.1,
            'Minimum rise above the detected rail height to count as a follow target.',
        ))
        self.follow_target_max_height = float(declare(
            'follow_target_max_height',
            2.2,
            'Maximum rise above the detected rail height to keep the follow target plausible.',
        ))
        self.latest_odom = None

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.center_offset_pub = self.create_publisher(Float32, self.center_offset_topic, 10)
        self.tangent_yaw_pub = self.create_publisher(Float32, self.tangent_yaw_topic, 10)
        self.target_distance_pub = self.create_publisher(Float32, self.target_distance_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(GridMap, self.heightmap_topic, self.heightmap_callback, 10)

        self.get_logger().info(
            f'Parsing rails from {self.heightmap_topic} with {self.num_slices} slices '
            f'and publishing markers on {self.marker_topic}'
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def heightmap_callback(self, msg):
        if self.latest_odom is None:
            return

        grid = self._decode_grid_map(msg)
        if grid is None:
            return

        detection = self._detect_rails(grid, self.latest_odom)
        self.marker_pub.publish(
            build_markers(
                msg.header.frame_id,
                msg.header.stamp,
                detection,
                max(self.forward_span, self.follow_target_lookahead),
            )
        )
        self._publish_detection_scalars(detection)

    def _publish_detection_scalars(self, detection):
        line = detection['line']
        if line is None:
            center_offset = float('nan')
            tangent_yaw = float('nan')
        else:
            center_offset = float(line['signed_offset'])
            tangent_yaw = float(line['yaw'])

        follow_target = detection['follow_target']
        target_distance = -1.0 if follow_target is None else float(follow_target['distance'])

        self.center_offset_pub.publish(Float32(data=center_offset))
        self.tangent_yaw_pub.publish(Float32(data=tangent_yaw))
        self.target_distance_pub.publish(Float32(data=target_distance))

    def _decode_grid_map(self, msg):
        """Decode the published GridMap message back into a 2D elevation array."""
        if 'elevation' not in msg.layers:
            self.get_logger().warn('GridMap does not contain an elevation layer', throttle_duration_sec=2.0)
            return None

        layer = msg.data[msg.layers.index('elevation')]
        if len(layer.layout.dim) < 2:
            self.get_logger().warn('GridMap elevation layout is missing dimensions', throttle_duration_sec=2.0)
            return None

        width = int(layer.layout.dim[0].size)
        height = int(layer.layout.dim[1].size)
        flat = np.asarray(layer.data, dtype=np.float32)
        if flat.size != width * height:
            self.get_logger().warn('GridMap elevation data size does not match its layout', throttle_duration_sec=2.0)
            return None

        elevation = np.flip(flat.reshape(height, width), axis=(0, 1))
        resolution = float(msg.info.resolution)
        center = np.array([
            float(msg.info.pose.position.x),
            float(msg.info.pose.position.y),
        ], dtype=np.float32)
        return {
            'elevation': elevation,
            'resolution': resolution,
            'center': center,
            'length_x': float(msg.info.length_x),
            'length_y': float(msg.info.length_y),
            'width': width,
            'height': height,
        }

    def _detect_rails(self, grid, odom):
        """Extract rail hits from several slices after aligning the center slice orientation."""
        robot_xy = np.array([
            float(odom.pose.pose.position.x),
            float(odom.pose.pose.position.y),
        ], dtype=np.float32)
        yaw = self._yaw_from_quaternion(odom.pose.pose.orientation)

        forward_offsets = np.linspace(
            -0.5 * self.forward_span, 0.5 * self.forward_span, self.num_slices, dtype=np.float32
        )
        sample_count = max(31, int(round(2.0 * self.lateral_search_width / grid['resolution'])) + 1)
        lateral_offsets = np.linspace(
            -self.lateral_search_width, self.lateral_search_width, sample_count, dtype=np.float32
        )
        forward, lateral = self._find_best_slice_axes(robot_xy, yaw, lateral_offsets, grid)

        slices = []
        midpoints = []
        hits = []

        for forward_offset in forward_offsets:
            xy, z, baseline = self._sample_slice(
                robot_xy,
                forward,
                lateral,
                forward_offset,
                lateral_offsets,
                grid,
            )

            pair = self._find_rail_pair(
                xy,
                lateral_offsets,
                z,
                baseline=baseline,
                resolution=grid['resolution'],
            )

            slice_result = {
                'xy': xy,
                'z': z,
                'left': None,
                'right': None,
                'midpoint': None,
                'follow_target': None,
            }
            if pair is not None:
                left_point, right_point = pair
                midpoint = 0.5 * (left_point + right_point)
                slice_result['left'] = left_point
                slice_result['right'] = right_point
                slice_result['midpoint'] = midpoint
                midpoints.append(midpoint)
                hits.extend([left_point, right_point])

            slices.append(slice_result)

        line = self._fit_centerline(robot_xy, forward, midpoints)
        rail_baseline = self._rail_height_baseline(hits)
        follow_target_candidates, follow_target = self._detect_follow_target(
            grid,
            robot_xy,
            line,
            rail_baseline,
        )
        return {
            'robot_xy': robot_xy,
            'forward': forward,
            'slices': slices,
            'midpoints': np.asarray(midpoints, dtype=np.float32),
            'hits': np.asarray(hits, dtype=np.float32),
            'line': line,
            'rail_baseline': rail_baseline,
            'follow_target_points': np.asarray(
                [candidate['point'] for candidate in follow_target_candidates],
                dtype=np.float32,
            ),
            'follow_target': follow_target,
        }

    def _find_best_slice_axes(self, robot_xy, yaw, lateral_offsets, grid):
        """Sweep the center slice angle and keep the valid orientation with the smallest gauge error."""
        best = None
        if self.angle_step_deg > 0.0 and self.angle_sweep_deg > 0.0:
            angle_offsets_deg = np.arange(
                -self.angle_sweep_deg,
                self.angle_sweep_deg + 0.5 * self.angle_step_deg,
                self.angle_step_deg,
                dtype=np.float32,
            )
        else:
            angle_offsets_deg = np.array([0.0], dtype=np.float32)

        for angle_offset_deg in angle_offsets_deg:
            forward, lateral = self._slice_axes(yaw + math.radians(float(angle_offset_deg)))
            xy, z, baseline = self._sample_slice(
                robot_xy,
                forward,
                lateral,
                0.0,
                lateral_offsets,
                grid,
            )
            pair = self._find_rail_pair(
                xy,
                lateral_offsets,
                z,
                baseline=baseline,
                resolution=grid['resolution'],
            )
            if pair is None:
                continue

            left_point, right_point = pair
            gauge_error = abs(float(np.linalg.norm(left_point[:2] - right_point[:2])) - self.track_gauge)
            candidate = (gauge_error, abs(float(angle_offset_deg)), forward, lateral)
            if best is None or candidate[:2] < best[:2]:
                best = candidate

        if best is not None:
            return best[2], best[3]
        return self._slice_axes(yaw)

    def _sample_slice(self, robot_xy, forward, lateral, forward_offset, lateral_offsets, grid):
        """Sample one cross-section and estimate its local center baseline."""
        slice_center = robot_xy + forward_offset * forward
        xy = slice_center + lateral_offsets[:, None] * lateral[None, :]
        z = self._sample_grid(grid, xy)
        center_mask = np.abs(lateral_offsets) <= min(0.25, 0.25 * self.track_gauge)
        baseline = self._safe_nanmedian(z[center_mask])
        if not math.isfinite(baseline):
            baseline = self._safe_nanmedian(z)
        return xy, z, baseline

    def _sample_grid(self, grid, xy):
        """Sample the elevation grid at world XY points with nearest-neighbor lookup."""
        x_min = grid['center'][0] - 0.5 * grid['length_x']
        y_min = grid['center'][1] - 0.5 * grid['length_y']
        coords = np.empty_like(xy)
        coords[:, 0] = (xy[:, 0] - x_min) / grid['resolution'] - 0.5
        coords[:, 1] = (xy[:, 1] - y_min) / grid['resolution'] - 0.5
        cols = np.rint(coords[:, 0]).astype(np.int32)
        rows = np.rint(coords[:, 1]).astype(np.int32)
        inside = (
            (cols >= 0)
            & (cols < grid['width'])
            & (rows >= 0)
            & (rows < grid['height'])
        )

        z = np.full(len(xy), np.nan, dtype=np.float32)
        z[inside] = grid['elevation'][rows[inside], cols[inside]]
        return z

    def _find_rail_pair(self, xy, lateral_offsets, z, baseline, resolution):
        """Group one slice into rail-like blobs and return the best left/right pair."""
        if not math.isfinite(baseline):
            return None

        smooth = self._smooth_profile(z)
        in_height_band = (
            np.isfinite(smooth)
            & (smooth >= baseline + self.min_rail_height)
            & (smooth <= baseline + self.max_rail_height)
        )
        groups = self._extract_slice_groups(
            xy,
            lateral_offsets,
            z,
            smooth,
            in_height_band,
            resolution,
        )
        if not groups:
            return None

        width_tolerance = max(2.0 * resolution, 0.5 * self.rail_width)
        left_groups = [
            group for group in groups
            if group['offset'] > 0.0 and abs(group['width'] - self.rail_width) <= width_tolerance
        ]
        right_groups = [
            group for group in groups
            if group['offset'] < 0.0 and abs(group['width'] - self.rail_width) <= width_tolerance
        ]
        if not left_groups or not right_groups:
            return None

        best_pair = None
        best_score = float('inf')
        for left_group in left_groups:
            for right_group in right_groups:
                measured_gauge = float(left_group['offset'] - right_group['offset'])
                gauge_error = abs(measured_gauge - self.track_gauge)
                height_error = abs(left_group['point'][2] - right_group['point'][2])
                width_error = (
                    abs(left_group['width'] - self.rail_width)
                    + abs(right_group['width'] - self.rail_width)
                )
                if (
                    gauge_error > self.gauge_tolerance
                    or height_error > self.max_rail_height_difference
                ):
                    continue

                score = gauge_error + height_error + 0.5 * width_error
                if score < best_score:
                    best_score = score
                    best_pair = (left_group['point'], right_group['point'])

        return best_pair

    def _rail_height_baseline(self, hits):
        return self._safe_nanmedian(np.asarray(hits, dtype=np.float32)[:, 2]) if hits else float('nan')

    def _detect_follow_target(self, grid, robot_xy, line, rail_baseline):
        if line is None or not math.isfinite(rail_baseline):
            return [], None

        step = max(float(grid['resolution']), self.follow_target_sample_step)
        if self.follow_target_lookahead <= 0.0 or self.follow_target_kernel_size <= 0.0:
            return [], None

        tangent = line['tangent']
        normal = np.array([-tangent[1], tangent[0]], dtype=np.float32)
        start = robot_xy - line['signed_offset'] * normal
        kernel_half_width = 0.5 * self.follow_target_kernel_size
        sample_count = max(3, int(round(self.follow_target_kernel_size / grid['resolution'])) + 1)
        lateral_offsets = np.linspace(
            -kernel_half_width,
            kernel_half_width,
            sample_count,
            dtype=np.float32,
        )

        candidates = []
        distances = np.arange(0.0, self.follow_target_lookahead + 0.5 * step, step, dtype=np.float32)
        for distance in distances:
            center = start + float(distance) * tangent
            xy = center + lateral_offsets[:, None] * normal[None, :]
            z = self._sample_grid(grid, xy)
            peak = self._safe_nanpercentile(z, 90.0)
            if not math.isfinite(peak):
                continue

            height = float(peak - rail_baseline)
            if height < self.follow_target_min_height or height > self.follow_target_max_height:
                continue

            target = {
                'point': np.array([center[0], center[1], peak], dtype=np.float32),
                'distance': float(distance),
                'height': height,
            }
            candidates.append(target)
            return candidates, target

        return candidates, None

    def _extract_slice_groups(self, xy, lateral_offsets, z, smooth, mask, resolution):
        """Split one slice into contiguous height-consistent groups."""
        groups = []
        group_height_step = max(resolution, 0.5 * self.min_rail_height)
        start = None
        prev = None

        for index in range(len(smooth)):
            if not mask[index]:
                if start is not None:
                    groups.append(self._build_slice_group(xy, lateral_offsets, z, start, prev, resolution))
                    start = None
                    prev = None
                continue

            if start is None:
                start = prev = index
                continue

            if abs(float(smooth[index] - smooth[prev])) <= group_height_step:
                prev = index
                continue

            groups.append(self._build_slice_group(xy, lateral_offsets, z, start, prev, resolution))
            start = prev = index

        if start is not None:
            groups.append(self._build_slice_group(xy, lateral_offsets, z, start, prev, resolution))

        return [group for group in groups if group is not None]

    def _build_slice_group(self, xy, lateral_offsets, z, start, end, resolution):
        """Summarize one contiguous group by its center, width, and representative height."""
        segment = slice(start, end + 1)
        segment_xy = xy[segment]
        segment_z = z[segment]
        valid = np.isfinite(segment_z)
        if not np.any(valid):
            return None

        point = np.empty(3, dtype=np.float32)
        point[:2] = np.mean(segment_xy[valid], axis=0)
        point[2] = float(np.nanmedian(segment_z[valid]))
        return {
            'point': point,
            'offset': float(np.mean(lateral_offsets[segment])),
            'width': float(abs(lateral_offsets[end] - lateral_offsets[start]) + resolution),
        }

    def _smooth_profile(self, z):
        """Apply a tiny 1D smoothing kernel while preserving missing samples."""
        finite = np.isfinite(z)
        weights = finite.astype(np.float32)
        values = np.where(finite, z, 0.0).astype(np.float32)
        kernel = np.array([1.0, 2.0, 1.0], dtype=np.float32)
        numer = np.convolve(values, kernel, mode='same')
        denom = np.convolve(weights, kernel, mode='same')
        smooth = np.full(len(z), np.nan, dtype=np.float32)
        valid = denom > 0.0
        smooth[valid] = numer[valid] / denom[valid]
        return smooth

    @staticmethod
    def _slice_axes(yaw):
        """Convert a slice yaw into forward and lateral unit vectors."""
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float32)
        lateral = np.array([-math.sin(yaw), math.cos(yaw)], dtype=np.float32)
        return forward, lateral

    def _fit_centerline(self, robot_xy, forward, midpoints):
        """Fit a short centerline and derive the robot's signed lateral offset."""
        if len(midpoints) == 0:
            return None

        xy = np.asarray(midpoints, dtype=np.float32)[:, :2]
        center = np.mean(xy, axis=0)
        tangent = np.asarray(forward, dtype=np.float32)
        if len(xy) >= 2:
            _, _, vh = np.linalg.svd(xy - center[None, :], full_matrices=False)
            tangent = vh[0].astype(np.float32)
        if np.dot(tangent, forward) < 0.0:
            tangent = -tangent

        normal = np.array([-tangent[1], tangent[0]], dtype=np.float32)
        signed_offset = float(np.dot(robot_xy - center, normal))
        return {
            'center': center,
            'tangent': tangent,
            'signed_offset': signed_offset,
            'yaw': math.atan2(float(tangent[1]), float(tangent[0])),
        }

    @staticmethod
    def _safe_nanmedian(values):
        finite = np.asarray(values, dtype=np.float32)
        finite = finite[np.isfinite(finite)]
        if len(finite) == 0:
            return float('nan')
        return float(np.median(finite))

    @staticmethod
    def _safe_nanpercentile(values, percentile):
        finite = np.asarray(values, dtype=np.float32)
        finite = finite[np.isfinite(finite)]
        if len(finite) == 0:
            return float('nan')
        return float(np.percentile(finite, percentile))

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = RailDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()