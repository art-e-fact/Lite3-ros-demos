"""Editable ROS parameter with UI metadata."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from rclpy.parameter import Parameter

if TYPE_CHECKING:
    from rail_inspector.rail_follow_tui.ros_state import RosState


@dataclass
class EditableParam:
    """One watched parameter: local value, ROS I/O, and UI constraints."""

    key: str
    param_type: Parameter.Type
    default: Any
    local: bool = True
    min: float | None = None
    max: float | None = None
    step: float | None = None
    kind: str = 'number'
    options: tuple[str, ...] = ()
    description: str = ''
    _value: Any = field(default=None, init=False)
    _owner: RosState = field(default=None, repr=False, init=False)  # type: ignore[assignment]

    @property
    def node_name(self) -> str:
        return self.key.split('/', 1)[0]

    @property
    def param_name(self) -> str:
        return self.key.split('/', 1)[1]

    def bind(self, owner: RosState) -> None:
        self._owner = owner
        self._value = self.default

    def get(self) -> Any:
        return self._value

    def set(self, value: Any) -> bool:
        """Write to the parameter server and update the local copy."""
        value = self._coerce(value)
        if self.local:
            results = self._owner.set_parameters(
                [Parameter(self.param_name, self.param_type, value)]
            )
            if not all(result.successful for result in results):
                return False
            self._value = value
            return True
        return self._owner.set_remote_param(self.node_name, self.param_name, value)

    def assign(self, value: Any) -> None:
        """Set local copy without writing ROS (startup / confirmed remote write)."""
        self._value = self._coerce(value)

    def syncup(self, value: Any) -> None:
        """Update local copy only (from parameter events; avoids write loops)."""
        value = self._coerce(value)
        if value == self._value:
            return
        self._value = value
        self._owner.mark_dirty()

    def _coerce(self, value: Any) -> Any:
        if self.param_type == Parameter.Type.DOUBLE:
            value = float(value)
            if self.min is not None and self.max is not None:
                value = max(self.min, min(self.max, value))
        elif self.param_type == Parameter.Type.INTEGER:
            value = int(round(float(value)))
        elif self.param_type == Parameter.Type.BOOL:
            if isinstance(value, str):
                value = value.strip().lower() in ('1', 'true', 'yes', 'on')
            else:
                value = bool(value)
        elif self.param_type == Parameter.Type.STRING:
            value = str(value)
            if self.options and value not in self.options:
                raise ValueError(f'{value!r} not in {self.options}')
        return value
