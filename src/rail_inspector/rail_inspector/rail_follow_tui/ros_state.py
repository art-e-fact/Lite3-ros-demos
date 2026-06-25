"""ROS state for the rail-follow TUI: watched params, events, teleop publish."""

from __future__ import annotations

import threading
from typing import Any, NamedTuple

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


class RemoteSpec(NamedTuple):
    """One remote ROS parameter exposed in the Parameters tab."""

    name: str
    default: Any
    description: str = ''
    kind: str | None = None
    options: tuple[str, ...] = ()


S = RemoteSpec


def _remote_param(node: str, spec: RemoteSpec) -> EditableParam:
    default = spec.default
    if spec.kind == 'enum':
        param_type, kind = Parameter.Type.STRING, 'enum'
    elif isinstance(default, bool):
        param_type, kind = Parameter.Type.BOOL, spec.kind or 'bool'
    elif isinstance(default, int):
        param_type, kind = Parameter.Type.INTEGER, spec.kind or 'number'
    elif isinstance(default, str):
        param_type, kind = Parameter.Type.STRING, spec.kind or 'string'
    else:
        param_type, kind = Parameter.Type.DOUBLE, spec.kind or 'number'
    return EditableParam(
        key=f'{node}/{spec.name}',
        param_type=param_type,
        default=default,
        local=False,
        kind=kind,
        options=spec.options,
        description=spec.description,
    )


def _remote_params(node: str, specs: tuple[RemoteSpec, ...]) -> dict[str, EditableParam]:
    params: dict[str, EditableParam] = {}
    for spec in specs:
        param = _remote_param(node, spec)
        params[param.key] = param
    return params


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


# declare_dynamic params per node — keep in sync with the node implementations.
FOLLOWER_REMOTE: tuple[RemoteSpec, ...] = (
    S('follow_mode', 'teleop', 'Follower control mode.', kind='enum', options=('auto', 'teleop')),
    S('stale_timeout_sec', 0.5, 'Max age accepted for detector and odometry inputs.'),
    S('follow_distance', 1.5, 'Desired stopping distance from the follow target.'),
    S('target_distance_deadband', 0.1, 'Extra no-motion margin beyond follow_distance.'),
    S('min_linear_x', 0.4, 'Minimum forward command that starts locomotion.'),
    S('max_linear_x', 0.55, 'Maximum forward body-frame speed (m/s).'),
    S('distance_error_for_max_speed', 1.5, 'Distance error where speed reaches max_linear_x.'),
    S('max_linear_y', 0.4, 'Maximum lateral centering speed (m/s).'),
    S('max_angular_z', 0.5, 'Maximum yaw-rate command (rad/s).'),
    S('k_center', 1.0, 'Gain: rail center offset → lateral speed.'),
    S('k_heading', 1.2, 'Gain: tangent yaw error → angular speed.'),
)

DETECTOR_REMOTE: tuple[RemoteSpec, ...] = (
    S('track_gauge', 1.067, 'Expected distance between the two rails in meters.'),
    S('rail_width', 0.15, 'Expected lateral width of one rail in meters.'),
    S('gauge_tolerance', 0.40, 'Maximum rail-pair gauge error allowed in one slice.'),
    S('angle_sweep_deg', 40.0, 'Half-range of the center-slice heading search in degrees.'),
    S('angle_step_deg', 5.0, 'Heading increment used during the center-slice search.'),
    S('min_rail_height', 0.05, 'Minimum height above the local baseline to accept a rail hit.'),
    S('max_rail_height', 0.30, 'Maximum height above the local baseline to accept a rail hit.'),
    S('max_rail_height_difference', 0.08, 'Maximum height mismatch between left and right rail.'),
    S('baseline_auto', True, 'Estimate the ground baseline per slice from the height scan.'),
    S('baseline_z', 0.0, 'Fixed ground baseline Z when baseline_auto is false.'),
    S('forward_span', 2.6, 'Forward distance ahead covered by sampled rail slices.'),
    S('backward_span', 0.0, 'Backward distance behind covered by sampled rail slices.'),
    S('num_slices', 15, 'Number of cross-sections sampled along the rail.'),
    S('lateral_search_width', 1.8, 'Half-width of each sampled cross-section in meters.'),
    S('follow_target_lookahead', 8.0, 'Distance ahead to search for a follow target.'),
    S('follow_target_kernel_size', 0.35, 'Width of the center sample window for the follow target.'),
    S('follow_target_sample_step', 0.10, 'Distance between follow-target samples along the rail.'),
    S('follow_target_min_height', 0.1, 'Minimum rise above rail height to count as a follow target.'),
    S('follow_target_max_height', 2.2, 'Maximum rise above rail height for a plausible follow target.'),
)

HEIGHTMAP_REMOTE: tuple[RemoteSpec, ...] = (
    S('min_z', -1.0, 'Minimum point height in the map frame.'),
    S('max_z', 2.0, 'Maximum point height in the map frame.'),
    S('min_range', 0.1, 'Minimum sensor range in meters.'),
    S('max_range', 12.0, 'Maximum sensor range in meters.'),
    S('stale_time_sec', 100.0, 'Age after which non-front cells are cleared.'),
    S('front_clear_enabled', False, 'Enable faster expiry in the front-clear rectangle.'),
    S('front_clear_length', 2.5, 'Front-clear rectangle length in meters.'),
    S('front_clear_width', 1.0, 'Front-clear rectangle width in meters.'),
    S('front_clear_offset_x', 0.75, 'Front-clear rectangle start offset along robot X.'),
    S('front_stale_time_sec', 0.35, 'Age after which front-clear cells are cleared.'),
    S('max_pose_variance', 0.0, 'Skip scans when pose variance exceeds this; 0 disables.'),
    S('visibility_cleanup_enabled', False, 'Remove cells occluded by newly observed geometry.'),
    S('visibility_cleanup_tolerance', 0.05, 'Height tolerance used by visibility cleanup.'),
)

REMOTE_NODE_SPECS: tuple[tuple[str, tuple[RemoteSpec, ...]], ...] = (
    ('follower_node_name', FOLLOWER_REMOTE),
    ('detector_node_name', DETECTOR_REMOTE),
    ('heightmap_node_name', HEIGHTMAP_REMOTE),
)


def _build_params(node_names: dict[str, str]) -> dict[str, EditableParam]:
    params = _tui_params()
    for node_param, specs in REMOTE_NODE_SPECS:
        params.update(_remote_params(node_names[node_param], specs))
    return params


class RosState(Node):
    """Owns watched parameters, syncs via /parameter_events, publishes teleop Twist."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.declare_parameter('follow_rail_speed_topic', '/follow_rail_speed')
        self.declare_parameter('follower_node_name', 'rail_target_follower_node')
        self.declare_parameter('detector_node_name', 'rail_detector_node')
        self.declare_parameter('heightmap_node_name', 'local_heightmap_node')
        self.declare_parameter('publish_rate_hz', 10.0)

        node_names = {
            name: self.get_parameter(name).value
            for name, _ in REMOTE_NODE_SPECS
        }
        self._follower_node = node_names['follower_node_name']
        speed_topic = self.get_parameter('follow_rail_speed_topic').value

        self.params = _build_params(node_names)
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
                continue
            handle = self._param_handler.add_parameter_callback(
                param.param_name,
                param.node_name,
                lambda p, watched_key=key: self._on_param_changed(watched_key, p),
            )
            self._param_handles.append(handle)

        self._sync_remote_params()

        self.get_logger().info(
            f'Publishing teleop speed on {speed_topic}; '
            f'watching params on {", ".join(f"/{n}" for n in node_names.values())}'
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
