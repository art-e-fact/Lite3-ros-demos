"""ROS state for the rail-follow TUI: watched params, events, teleop publish."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
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
            min=-0.75,
            max=0.75,
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


@dataclass
class _PendingSet:
    param_key: str
    value: Any
    future: Any
    on_done: Callable[[bool], None] | None
    generation: int


@dataclass
class _PendingGet:
    param_key: str
    future: Any
    on_done: Callable[[bool], None] | None


@dataclass
class _RefreshRequest:
    remaining: int
    all_succeeded: bool
    on_done: Callable[[bool], None] | None


@dataclass
class _PendingGetBatch:
    param_keys: list[str]
    future: Any
    refresh: _RefreshRequest


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
        self._pending_sets: dict[str, _PendingSet] = {}
        self._pending_gets: list[_PendingGet] = []
        self._pending_get_batches: list[_PendingGetBatch] = []
        self._set_generation: dict[str, int] = {}

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

    def process_pending(self) -> None:
        """Finish in-flight remote param service calls (call after spin_once)."""
        self._process_pending_sets()
        self._process_pending_gets()
        self._process_pending_get_batches()

    def refresh_from_server(
        self,
        on_done: Callable[[bool], None] | None = None,
    ) -> None:
        """Re-read all watched params from the parameter server (async)."""
        for param in self.params.values():
            if param.local:
                param.assign(self.get_parameter(param.param_name).value)

        remote_by_node: dict[str, list[str]] = {}
        for key, param in self.params.items():
            if not param.local:
                remote_by_node.setdefault(param.node_name, []).append(key)

        if not remote_by_node:
            self.mark_dirty()
            if on_done:
                on_done(True)
            return

        refresh = _RefreshRequest(
            remaining=len(remote_by_node),
            all_succeeded=True,
            on_done=on_done,
        )
        for node_name, param_keys in remote_by_node.items():
            self._request_get_batch(node_name, param_keys, refresh)

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

    def request_set(
        self,
        param_key: str,
        value: Any,
        on_done: Callable[[bool], None] | None = None,
    ) -> bool:
        """Queue an async remote set; completion runs in process_pending()."""
        param = self.params[param_key]
        client = self._set_client(param.node_name)
        if not client.service_is_ready():
            self.get_logger().warning(
                f'/{param.node_name}/set_parameters unavailable'
            )
            if on_done:
                on_done(False)
            else:
                # Revert optimistic UI updates (e.g. follow-mode toggle).
                self._request_get(param_key, None)
            return False

        request = SetParameters.Request()
        request.parameters = [
            Parameter(param.param_name, param.param_type, value).to_parameter_msg()
        ]
        future = client.call_async(request)
        # Latest write per param wins if the user toggles quickly.
        generation = self._set_generation.get(param_key, 0) + 1
        self._set_generation[param_key] = generation
        self._pending_sets[param_key] = _PendingSet(
            param_key, value, future, on_done, generation
        )
        return True

    def _process_pending_sets(self) -> None:
        for param_key in list(self._pending_sets):
            pending = self._pending_sets[param_key]
            if not pending.future.done():
                continue
            if pending.generation != self._set_generation.get(param_key):
                del self._pending_sets[param_key]
                continue

            del self._pending_sets[param_key]
            if self._complete_set(pending):
                if pending.on_done:
                    pending.on_done(True)
            else:
                # Pull the authoritative value back after a rejected write.
                self._request_get(param_key, pending.on_done)

    def _complete_set(self, pending: _PendingSet) -> bool:
        param = self.params[pending.param_key]
        try:
            response = pending.future.result()
        except Exception:
            self.get_logger().warning(f'Failed to set {pending.param_key}')
            return False

        if response is None:
            self.get_logger().warning(f'Failed to set {pending.param_key}')
            return False

        results = response.results
        if not results or not results[0].successful:
            reason = results[0].reason if results else 'unknown error'
            self.get_logger().warning(f'{param.param_name} rejected: {reason}')
            return False

        param.assign(pending.value)
        return True

    def _request_get(
        self,
        param_key: str,
        on_done: Callable[[bool], None] | None,
    ) -> None:
        param = self.params[param_key]
        client = self._get_client(param.node_name)
        if not client.service_is_ready():
            self.get_logger().warning(
                f'/{param.node_name}/get_parameters unavailable'
            )
            if on_done:
                on_done(False)
            return

        request = GetParameters.Request()
        request.names = [param.param_name]
        future = client.call_async(request)
        self._pending_gets.append(_PendingGet(param_key, future, on_done))

    def _process_pending_gets(self) -> None:
        remaining: list[_PendingGet] = []
        for pending in self._pending_gets:
            if not pending.future.done():
                remaining.append(pending)
                continue

            value = self._complete_get(pending)
            if value is not None:
                self.params[pending.param_key].syncup(value)
            if pending.on_done:
                pending.on_done(value is not None)

        self._pending_gets = remaining

    def _request_get_batch(
        self,
        node_name: str,
        param_keys: list[str],
        refresh: _RefreshRequest,
    ) -> None:
        client = self._get_client(node_name)
        if not client.service_is_ready():
            self.get_logger().warning(
                f'/{node_name}/get_parameters unavailable'
            )
            self._finish_refresh_batch(refresh, False)
            return

        request = GetParameters.Request()
        request.names = [self.params[key].param_name for key in param_keys]
        future = client.call_async(request)
        self._pending_get_batches.append(
            _PendingGetBatch(param_keys, future, refresh)
        )

    def _process_pending_get_batches(self) -> None:
        remaining: list[_PendingGetBatch] = []
        for pending in self._pending_get_batches:
            if not pending.future.done():
                remaining.append(pending)
                continue
            self._finish_refresh_batch(
                pending.refresh,
                self._complete_get_batch(pending),
            )
        self._pending_get_batches = remaining

    def _complete_get_batch(self, pending: _PendingGetBatch) -> bool:
        try:
            response = pending.future.result()
        except Exception:
            return False

        if response is None:
            return False

        values = response.values
        if not values or len(values) != len(pending.param_keys):
            return False

        for key, value_msg in zip(pending.param_keys, values, strict=True):
            self.params[key].assign(parameter_value_to_python(value_msg))
        return True

    def _finish_refresh_batch(
        self,
        refresh: _RefreshRequest,
        success: bool,
    ) -> None:
        if not success:
            refresh.all_succeeded = False
        refresh.remaining -= 1
        if refresh.remaining > 0:
            return
        self.mark_dirty()
        if refresh.on_done:
            refresh.on_done(refresh.all_succeeded)

    def _complete_get(self, pending: _PendingGet) -> Any | None:
        try:
            response = pending.future.result()
        except Exception:
            return None

        if response is None:
            return None

        values = response.values
        if not values:
            return None
        return parameter_value_to_python(values[0])

    def _on_param_changed(self, key: str, param_msg) -> None:
        value = parameter_value_to_python(param_msg.value)
        self.params[key].syncup(value)

    def _sync_remote_params(self) -> None:
        remote_by_node: dict[str, list[str]] = {}
        for key, param in self.params.items():
            if param.local:
                param.assign(self.get_parameter(param.param_name).value)
            else:
                remote_by_node.setdefault(param.node_name, []).append(key)

        for node_name, param_keys in remote_by_node.items():
            param_names = [self.params[key].param_name for key in param_keys]
            values = self._fetch_remote_params_blocking(node_name, param_names)
            if values is None:
                continue
            for key, value in zip(param_keys, values, strict=True):
                self.params[key].syncup(value)

    def _fetch_remote_params_blocking(
        self, node_name: str, param_names: list[str]
    ) -> list[Any] | None:
        """Blocking fetch used only at startup before the UI tick loop runs."""
        if not param_names:
            return []

        client = self._get_client(node_name)
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'/{node_name}/get_parameters unavailable')
            return None

        request = GetParameters.Request()
        request.names = param_names
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            return None

        values = future.result().values
        if not values or len(values) != len(param_names):
            return None
        return [parameter_value_to_python(v) for v in values]

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
