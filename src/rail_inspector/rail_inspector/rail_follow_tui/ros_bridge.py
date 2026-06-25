"""ROS interface for the rail-follow TUI."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


class RailFollowRosBridge(Node):
    """Publishes teleop speed and sets follow_mode on the follower node."""

    def __init__(self) -> None:
        super().__init__('rail_follow_tui')

        self.declare_parameter('follow_rail_speed_topic', '/follow_rail_speed')
        self.declare_parameter('follower_node_name', 'rail_target_follower_node')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('default_speed', 0.4)
        self.declare_parameter('max_speed', 0.75)

        speed_topic = self.get_parameter('follow_rail_speed_topic').value
        follower_name = self.get_parameter('follower_node_name').value

        self._speed_pub = self.create_publisher(Twist, speed_topic, 10)
        self._param_client = self.create_client(
            SetParameters, f'/{follower_name}/set_parameters'
        )

        self.get_logger().info(
            f'Publishing teleop speed on {speed_topic}; '
            f'follow_mode target: /{follower_name}'
        )

    @property
    def default_speed(self) -> float:
        return float(self.get_parameter('default_speed').value)

    @property
    def max_speed(self) -> float:
        return float(self.get_parameter('max_speed').value)

    @property
    def publish_rate_hz(self) -> float:
        return float(self.get_parameter('publish_rate_hz').value)

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0)

    def publish_speed(self, linear_x: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        self._speed_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_speed(0.0)

    def set_follow_mode(self, mode: str) -> bool:
        """Set follow_mode to 'auto' or 'teleop' on the follower node."""
        if not self._param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Follower set_parameters service unavailable')
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter('follow_mode', Parameter.Type.STRING, mode).to_parameter_msg()
        ]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().warning('Failed to set follow_mode')
            return False

        results = future.result().results
        if not results or not results[0].successful:
            reason = results[0].reason if results else 'unknown error'
            self.get_logger().warning(f'follow_mode rejected: {reason}')
            return False
        return True
