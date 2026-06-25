"""ROS state for the rail-follow TUI: watched params, events, teleop publish."""

from __future__ import annotations

import threading
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEventHandler

from rail_inspector.rail_follow_tui.editable_param import EditableParam

NODE_NAME = 'rail_follow_tui'
KEY_SPEED = f'{NODE_NAME}/speed'
KEY_GOING = f'{NODE_NAME}/going'


def _follow_mode_key(follower_node: str) -> str:
    return f'{follower_node}/follow_mode'


# Params shown on the Control tab — excluded from the Parameters tab.
def params_tab_blacklist(follower_node: str) -> frozenset[str]:
    return frozenset({KEY_SPEED, KEY_GOING, _follow_mode_key(follower_node)})


def params_by_node(
    params: dict[str, EditableParam],
    blacklist: frozenset[str],
) -> dict[str, list[EditableParam]]:
    grouped: dict[str, list[EditableParam]] = {}
    for key, param in params.items():
        if key in blacklist:
            continue
        grouped.setdefault(param.node_name, []).append(param)
    for node_params in grouped.values():
        node_params.sort(key=lambda p: p.param_name)
    return grouped


def _tui_params() -> dict[str, EditableParam]:
    return {
        KEY_SPEED: EditableParam(
            key=KEY_SPEED,
            param_type=Parameter.Type.DOUBLE,
            default=0.4,
            min=-0.55,
            max=0.55,
            step=0.05,
            description='Teleop speed command in m/s.',
        ),
        KEY_GOING: EditableParam(
            key=KEY_GOING,
            param_type=Parameter.Type.BOOL,
            default=False,
            kind='bool',
            description='When true, publish speed in teleop mode.',
        ),
    }


# Dynamic follower params (declare_dynamic in rail_target_follower_node).
_FOLLOWER_DOUBLES: tuple[tuple[str, float, str], ...] = (
    ('stale_timeout_sec', 0.5, 'Max age accepted for detector and odometry inputs.'),
    ('follow_distance', 1.5, 'Desired stopping distance from the follow target.'),
    ('target_distance_deadband', 0.1, 'Extra no-motion margin beyond follow_distance.'),
    ('min_linear_x', 0.4, 'Minimum forward command that starts locomotion.'),
    ('max_linear_x', 0.55, 'Maximum forward body-frame speed (m/s).'),
    ('distance_error_for_max_speed', 1.5, 'Distance error where speed reaches max_linear_x.'),
    ('max_linear_y', 0.4, 'Maximum lateral centering speed (m/s).'),
    ('max_angular_z', 0.5, 'Maximum yaw-rate command (rad/s).'),
    ('k_center', 1.0, 'Gain: rail center offset → lateral speed.'),
    ('k_heading', 1.2, 'Gain: tangent yaw error → angular speed.'),
)


def _follower_params(follower_node: str) -> dict[str, EditableParam]:
    params = {
        _follow_mode_key(follower_node): EditableParam(
            key=_follow_mode_key(follower_node),
            param_type=Parameter.Type.STRING,
            default='teleop',
            local=False,
            kind='enum',
            options=('auto', 'teleop'),
            description='Follower control mode.',
        ),
    }
    for name, default, description in _FOLLOWER_DOUBLES:
        key = f'{follower_node}/{name}'
        params[key] = EditableParam(
            key=key,
            param_type=Parameter.Type.DOUBLE,
            default=default,
            local=False,
            description=description,
        )
    return params


def _build_params(follower_node: str) -> dict[str, EditableParam]:
    return {**_tui_params(), **_follower_params(follower_node)}


class RosState(Node):
    """Owns watched parameters, syncs via /parameter_events, publishes teleop Twist."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.declare_parameter('follow_rail_speed_topic', '/follow_rail_speed')
        self.declare_parameter('follower_node_name', 'rail_target_follower_node')
        self.declare_parameter('publish_rate_hz', 10.0)

        self._follower_node = self.get_parameter('follower_node_name').value
        speed_topic = self.get_parameter('follow_rail_speed_topic').value

        self.params = _build_params(self._follower_node)
        for param in self.params.values():
            param.bind(self)
            if param.local:
                self.declare_parameter(param.param_name, param.default)

        self._dirty = False
        self._dirty_lock = threading.Lock()
        self._set_clients: dict[str, rclpy.client.Client] = {}
        self._get_clients: dict[str, rclpy.client.Client] = {}

        self._speed_pub = self.create_publisher(Twist, speed_topic, 10)
        self._param_handler = ParameterEventHandler(self)
        self._param_handles = []
        for key, param in self.params.items():
            if param.local:
                continue  # local writes don't need event echo sync
            handle = self._param_handler.add_parameter_callback(
                param.param_name,
                param.node_name,
                lambda p, watched_key=key: self._on_param_changed(watched_key, p),
            )
            self._param_handles.append(handle)

        self._sync_remote_params()

        self.get_logger().info(
            f'Publishing teleop speed on {speed_topic}; '
            f'follow_mode target: /{self._follower_node}'
        )

    @property
    def publish_rate_hz(self) -> float:
        return float(self.get_parameter('publish_rate_hz').value)

    @property
    def follower_node(self) -> str:
        return self._follower_node

    @property
    def follow_mode_key(self) -> str:
        return _follow_mode_key(self._follower_node)

    def mark_dirty(self) -> None:
        with self._dirty_lock:
            self._dirty = True

    def is_dirty(self) -> bool:
        with self._dirty_lock:
            return self._dirty

    def clear_dirty(self) -> None:
        with self._dirty_lock:
            self._dirty = False

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0)

    def publish_tick(self) -> None:
        """Publish teleop speed when going in teleop mode; otherwise stop."""
        going = self.params[KEY_GOING].get()
        mode = self.params[self.follow_mode_key].get()
        if going and mode == 'teleop':
            self._publish_speed(self.params[KEY_SPEED].get())
        else:
            self.publish_stop()

    def publish_stop(self) -> None:
        self._publish_speed(0.0)

    def set_remote_param(self, node_name: str, param_name: str, value: Any) -> bool:
        """Blocking remote write; call from a worker thread."""
        param = self.params[f'{node_name}/{param_name}']
        client = self._set_client(node_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'/{node_name}/set_parameters unavailable')
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(param_name, param.param_type, value).to_parameter_msg()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().warning(f'Failed to set {node_name}/{param_name}')
            return False

        results = future.result().results
        if not results or not results[0].successful:
            reason = results[0].reason if results else 'unknown error'
            self.get_logger().warning(f'{param_name} rejected: {reason}')
            return False

        param.assign(value)
        return True

    def _on_param_changed(self, key: str, param_msg) -> None:
        value = parameter_value_to_python(param_msg.value)
        self.params[key].syncup(value)

    def _sync_remote_params(self) -> None:
        for key, param in self.params.items():
            if param.local:
                param.assign(self.get_parameter(param.param_name).value)
            else:
                value = self._fetch_remote_param(param.node_name, param.param_name)
                if value is not None:
                    param.syncup(value)

    def fetch_remote_param(self, node_name: str, param_name: str) -> Any | None:
        return self._fetch_remote_param(node_name, param_name)

    def _fetch_remote_param(self, node_name: str, param_name: str) -> Any | None:
        client = self._get_client(node_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'/{node_name}/get_parameters unavailable')
            return None

        request = GetParameters.Request()
        request.names = [param_name]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            return None

        values = future.result().values
        if not values:
            return None
        return parameter_value_to_python(values[0])

    def _publish_speed(self, linear_x: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        self._speed_pub.publish(msg)

    def _set_client(self, node_name: str) -> rclpy.client.Client:
        if node_name not in self._set_clients:
            self._set_clients[node_name] = self.create_client(
                SetParameters, f'/{node_name}/set_parameters'
            )
        return self._set_clients[node_name]

    def _get_client(self, node_name: str) -> rclpy.client.Client:
        if node_name not in self._get_clients:
            self._get_clients[node_name] = self.create_client(
                GetParameters, f'/{node_name}/get_parameters'
            )
        return self._get_clients[node_name]
