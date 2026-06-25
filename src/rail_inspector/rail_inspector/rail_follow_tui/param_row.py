"""Single parameter input row with an inline Set button."""

from __future__ import annotations

from typing import Any

from textual.containers import Horizontal
from textual.validation import Number
from textual.widgets import Button, Input, Label

from rail_inspector.rail_follow_tui.editable_param import EditableParam
from rail_inspector.rail_follow_tui.sync_input import sync_input


def dom_id(key: str) -> str:
    return f'param-row--{key.replace("/", "--")}'


class ParamRow(Horizontal):
    """Label + input + Set; Set enables when the typed value differs from ROS."""

    def __init__(self, param: EditableParam, **kwargs) -> None:
        self.param = param
        super().__init__(classes='param-row', **kwargs)

    def compose(self):
        description = self.param.description or None
        yield Label(self.param.param_name, classes='param-name')
        validators = []
        if self.param.min is not None and self.param.max is not None:
            validators = [Number(minimum=self.param.min, maximum=self.param.max)]
        yield Input(
            _format_value(self.param.get()),
            type='number' if self.param.kind == 'number' else 'text',
            classes='param-input',
            validators=validators,
            compact=True,
            tooltip=description,
        )
        yield Button(
            'Set',
            classes='param-set',
            disabled=True,
            compact=True,
            tooltip=description,
        )

    def on_mount(self) -> None:
        if self.param.description:
            self.query_one('.param-name', Label).tooltip = self.param.description

    def sync_from_param(self, *, force: bool = False) -> None:
        """Refresh input from ROS; skip while the user is editing."""
        inp = self.query_one('.param-input', Input)
        if force or not inp.has_focus:
            sync_input(self.app, inp, _format_value(self.param.get()))
        self._refresh_set_button()

    def pending_value(self) -> Any | None:
        inp = self.query_one('.param-input', Input)
        if not inp.is_valid:
            return None
        try:
            return self.param._coerce(inp.value)
        except ValueError:
            return None

    def is_dirty(self) -> bool:
        pending = self.pending_value()
        return pending is not None and pending != self.param.get()

    def _refresh_set_button(self) -> None:
        self.query_one('.param-set', Button).disabled = not self.is_dirty()


def _format_value(value: Any) -> str:
    if isinstance(value, float):
        return f'{value:.4g}'
    return str(value)
