import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float32


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RailTargetFollowerNode(Node):
    def __init__(self):
        super().__init__('rail_target_follower_node')

        def declare(name, default, description):
            return self.declare_parameter(
                name,
                default,
                ParameterDescriptor(description=description),
            ).value

        self.cmd_vel_topic = declare(
            'cmd_vel_topic',
            '/cmd_vel',
            'Output Twist topic consumed by the RL deploy twist interface.',
        )
        self.odom_topic = declare(
            'odom_topic',
            '/odom',
            'Odometry topic used for robot yaw and body-frame conversion.',
        )
        self.center_offset_topic = declare(
            'center_offset_topic',
            '/rail_detector/center_offset',
            'Input topic with the signed rail center offset in meters.',
        )
        self.tangent_yaw_topic = declare(
            'tangent_yaw_topic',
            '/rail_detector/tangent_yaw',
            'Input topic with the detected rail tangent yaw in radians.',
        )
        self.target_distance_topic = declare(
            'target_distance_topic',
            '/rail_detector/target_distance',
            'Input topic with the target distance in meters; negative means invalid.',
        )
        self.control_rate_hz = float(declare(
            'control_rate_hz',
            15.0,
            'Control loop rate used to publish cmd_vel.',
        ))
        self.stale_timeout_sec = float(declare(
            'stale_timeout_sec',
            0.5,
            'Maximum wall-time age accepted for detector and odometry inputs.',
        ))
        self.follow_distance = float(declare(
            'follow_distance',
            1.5,
            'Desired stopping distance to keep from the detected target.',
        ))
        self.target_distance_deadband = float(declare(
            'target_distance_deadband',
            0.1,
            'Additional distance margin before forward motion resumes.',
        ))
        self.min_linear_x = float(declare(
            'min_linear_x',
            0.4,
            'Minimum forward command that reliably starts locomotion.',
        ))
        self.max_linear_x = float(declare(
            'max_linear_x',
            0.55,
            'Maximum forward body-frame speed command in meters per second.',
        ))
        self.distance_error_for_max_speed = float(declare(
            'distance_error_for_max_speed',
            1.5,
            'Distance error at which forward speed reaches max_linear_x.',
        ))
        self.max_linear_y = float(declare(
            'max_linear_y',
            0.4,
            'Maximum lateral body-frame speed command in meters per second.',
        ))
        self.max_angular_z = float(declare(
            'max_angular_z',
            0.5,
            'Maximum yaw-rate command in radians per second.',
        ))
        self.k_center = float(declare(
            'k_center',
            1.0,
            'Gain that converts rail center offset into lateral correction speed.',
        ))
        self.k_heading = float(declare(
            'k_heading',
            1.2,
            'Gain that converts rail tangent yaw error into angular speed.',
        ))

        self.latest_center_offset = float('nan')
        self.latest_tangent_yaw = float('nan')
        self.latest_target_distance = -1.0
        self.latest_odom_yaw = float('nan')

        self.center_offset_walltime = float('-inf')
        self.tangent_yaw_walltime = float('-inf')
        self.target_distance_walltime = float('-inf')
        self.odom_walltime = float('-inf')

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 20)
        self.create_subscription(Float32, self.center_offset_topic, self.center_offset_callback, 10)
        self.create_subscription(Float32, self.tangent_yaw_topic, self.tangent_yaw_callback, 10)
        self.create_subscription(Float32, self.target_distance_topic, self.target_distance_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.control_timer = self.create_timer(
            1.0 / max(1e-6, self.control_rate_hz),
            self.control_callback,
        )

        self.get_logger().info(
            f'Following rails from {self.center_offset_topic}, {self.tangent_yaw_topic}, '
            f'and {self.target_distance_topic}; publishing Twist on {self.cmd_vel_topic}'
        )

    def center_offset_callback(self, msg):
        self.latest_center_offset = float(msg.data)
        self.center_offset_walltime = time.monotonic()

    def tangent_yaw_callback(self, msg):
        self.latest_tangent_yaw = float(msg.data)
        self.tangent_yaw_walltime = time.monotonic()

    def target_distance_callback(self, msg):
        self.latest_target_distance = float(msg.data)
        self.target_distance_walltime = time.monotonic()

    def odom_callback(self, msg):
        self.latest_odom_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_walltime = time.monotonic()

    def control_callback(self):
        now = time.monotonic()
        if self._is_stale(now):
            self.publish_stop()
            self.get_logger().warn('Follower inputs are stale; stopping.', throttle_duration_sec=2.0)
            return

        if not math.isfinite(self.latest_center_offset) or not math.isfinite(self.latest_tangent_yaw):
            self.publish_stop()
            self.get_logger().warn('Rail line is invalid; stopping.', throttle_duration_sec=2.0)
            return

        if not math.isfinite(self.latest_target_distance) or self.latest_target_distance < 0.0:
            self.publish_stop()
            self.get_logger().warn('Follow target is unavailable; stopping.', throttle_duration_sec=2.0)
            return

        distance_error = self.latest_target_distance - self.follow_distance
        if distance_error <= 0.0:
            self.publish_stop()
            return

        tangent = (
            math.cos(self.latest_tangent_yaw),
            math.sin(self.latest_tangent_yaw),
        )
        normal = (-tangent[1], tangent[0])

        along_speed = self._compute_along_speed(distance_error)
        if along_speed <= 0.0:
            self.publish_stop()
            return

        lateral_speed = _clamp(
            -self.k_center * self.latest_center_offset,
            -self.max_linear_y,
            self.max_linear_y,
        )
        yaw_error = _normalize_angle(self.latest_tangent_yaw - self.latest_odom_yaw)

        world_vx = along_speed * tangent[0] + lateral_speed * normal[0]
        world_vy = along_speed * tangent[1] + lateral_speed * normal[1]
        body_vx, body_vy = self._world_to_body(world_vx, world_vy, self.latest_odom_yaw)

        cmd = Twist()
        cmd.linear.x = _clamp(body_vx, -self.max_linear_x, self.max_linear_x)
        cmd.linear.y = _clamp(body_vy, -self.max_linear_y, self.max_linear_y)
        cmd.angular.z = _clamp(
            self.k_heading * yaw_error,
            -self.max_angular_z,
            self.max_angular_z,
        )
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _compute_along_speed(self, distance_error):
        effective_error = distance_error - self.target_distance_deadband
        if effective_error <= 0.0:
            return 0.0

        min_linear_x = _clamp(self.min_linear_x, 0.0, self.max_linear_x)
        ramp_distance = max(1e-6, self.distance_error_for_max_speed)
        ramp = _clamp(effective_error / ramp_distance, 0.0, 1.0)
        return min_linear_x + ramp * (self.max_linear_x - min_linear_x)

    def _is_stale(self, now):
        return (
            now - self.center_offset_walltime > self.stale_timeout_sec
            or now - self.tangent_yaw_walltime > self.stale_timeout_sec
            or now - self.target_distance_walltime > self.stale_timeout_sec
            or now - self.odom_walltime > self.stale_timeout_sec
        )

    @staticmethod
    def _world_to_body(world_vx, world_vy, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_vx = cos_yaw * world_vx + sin_yaw * world_vy
        body_vy = -sin_yaw * world_vx + cos_yaw * world_vy
        return body_vx, body_vy

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = RailTargetFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()