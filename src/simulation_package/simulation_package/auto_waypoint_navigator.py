#!/usr/bin/env python3
"""
Autonomous waypoint traversal for procedurally generated MuJoCo scenes.

Subscribes:
  - /procedural_waypoints (geometry_msgs/PoseArray)
  - /odom (nav_msgs/Odometry)

Publishes:
  - /cmd_vel (geometry_msgs/Twist)
"""

from __future__ import annotations

import math
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _quat_wxyz_to_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AutoWaypointNavigator(Node):
    def __init__(self) -> None:
        super().__init__("auto_waypoint_navigator")

        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("k_linear", 0.7)
        self.declare_parameter("k_angular", 1.3)
        self.declare_parameter("yaw_align_threshold", 0.50)
        self.declare_parameter("max_linear_x", 0.7)
        self.declare_parameter("max_linear_y", 0.5)
        self.declare_parameter("max_angular_z", 0.7)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.k_linear = float(self.get_parameter("k_linear").value)
        self.k_angular = float(self.get_parameter("k_angular").value)
        self.yaw_align_threshold = float(self.get_parameter("yaw_align_threshold").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.waypoint_sub = self.create_subscription(PoseArray, "/procedural_waypoints", self._waypoints_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 50)
        self.control_timer = self.create_timer(1.0 / max(1e-6, control_rate_hz), self._control_tick)

        self.odom_xy: Tuple[float, float] | None = None
        self.odom_yaw: float = 0.0
        self.last_odom_walltime: float = 0.0

        self.mission: List[Tuple[float, float]] = []
        self.mission_ready: bool = False
        self.mission_completed: bool = False
        self.current_idx: int = 0
        self.waypoint_signature: Tuple[Tuple[float, float], ...] | None = None
        self.last_log_time: float = 0.0

        self.get_logger().info("AutoWaypointNavigator started. Waiting for mission on /procedural_waypoints and /odom.")

    def _waypoints_cb(self, msg: PoseArray) -> None:
        points: List[Tuple[float, float]] = []
        for pose in msg.poses:
            points.append((float(pose.position.x), float(pose.position.y)))
        signature = tuple((round(p[0], 3), round(p[1], 3)) for p in points)
        if signature == self.waypoint_signature:
            return

        self.waypoint_signature = signature
        self.mission = points
        self.mission_ready = len(self.mission) > 0
        self.mission_completed = False
        self.current_idx = 0
        self.get_logger().info(
            f"Received updated mission with {len(self.mission)} waypoints from /procedural_waypoints."
        )

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_xy = (float(pos.x), float(pos.y))
        self.odom_yaw = _quat_wxyz_to_yaw(float(q.w), float(q.x), float(q.y), float(q.z))
        self.last_odom_walltime = time.monotonic()

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _is_odom_stale(self) -> bool:
        if self.last_odom_walltime <= 0.0:
            return True
        return (time.monotonic() - self.last_odom_walltime) > self.odom_timeout_sec

    def _control_tick(self) -> None:
        if self._is_odom_stale():
            self._publish_stop()
            now = time.monotonic()
            if now - self.last_log_time > 2.0:
                self.get_logger().warn("Odometry is stale or unavailable; publishing stop command.")
                self.last_log_time = now
            return

        if self.mission_completed:
            self._publish_stop()
            return

        if not self.mission_ready or self.odom_xy is None:
            self._publish_stop()
            return

        if self.current_idx >= len(self.mission):
            self.mission_completed = True
            self._publish_stop()
            self.get_logger().info("Completed mission waypoints. Robot stopped.")
            return

        target_x, target_y = self.mission[self.current_idx]
        cur_x, cur_y = self.odom_xy
        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        if dist <= self.goal_tolerance:
            self.current_idx += 1
            self.get_logger().info(f"Reached mission waypoint {self.current_idx}/{len(self.mission)}.")
            self._publish_stop()
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_err = _normalize_angle(desired_yaw - self.odom_yaw)
        cmd = Twist()
        cmd.angular.z = _clamp(self.k_angular * yaw_err, -self.max_angular_z, self.max_angular_z)

        if abs(yaw_err) > self.yaw_align_threshold:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
        else:
            speed = min(self.max_linear_x, self.k_linear * dist)
            ux = dx / max(1e-6, dist)
            uy = dy / max(1e-6, dist)
            body_x = math.cos(self.odom_yaw) * ux + math.sin(self.odom_yaw) * uy
            body_y = -math.sin(self.odom_yaw) * ux + math.cos(self.odom_yaw) * uy
            cmd.linear.x = _clamp(speed * body_x, -self.max_linear_x, self.max_linear_x)
            cmd.linear.y = _clamp(speed * body_y, -self.max_linear_y, self.max_linear_y)

        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = AutoWaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
