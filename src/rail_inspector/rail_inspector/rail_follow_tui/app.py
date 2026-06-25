"""Textual TUI for rail-follow teleop control."""

from __future__ import annotations

from textual import on, work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.validation import Number
from textual.widgets import Button, Footer, Header, Input, Label, Rule, Static, Switch

from rail_inspector.rail_follow_tui.ros_state import (
    KEY_GOING,
    KEY_SPEED,
    RosState,
)


class RailFollowTuiApp(App):
    """Terminal UI: mode switch, GO/STOP, and speed controls."""

    TITLE = 'Rail Follow'

    CSS = """
    Screen {
        background: #1a1625;
        overflow: hidden;
    }

    Header {
        background: #5b4ca9;
        color: #f0ecff;
    }

    Footer {
        background: #2d2640;
    }

    #content {
        height: auto;
        margin: 0 2;
    }

    .mode-row {
        height: auto;
        align: center middle;
        margin: 1 0;
    }

    .mode-label {
        width: 16;
        text-align: center;
        color: #c8bfe8;
    }

    Switch {
        margin: 0 2;
    }

    .go-row {
        height: auto;
        align: center middle;
        margin: 1 0;
    }

    #go-button {
        width: 60%;
        height: 3;
        text-style: bold;
    }

    #go-button.-go {
        background: #2d8a4e;
        color: #ffffff;
        border: tall #3cb371;
    }

    #go-button.-stop {
        background: #b83232;
        color: #ffffff;
        border: tall #e05050;
    }

    .speed-panel {
        height: auto;
        margin: 1 0;
    }

    .speed-row {
        height: auto;
        align: center middle;
    }

    .speed-label {
        color: #c8bfe8;
        margin-bottom: 1;
    }

    #speed-input {
        width: 1fr;
        margin-right: 1;
    }

    .step-button {
        min-width: 5;
        margin-left: 1;
        background: #5b4ca9;
    }

    #reverse-button {
        min-width: 10;
        margin-left: 2;
        background: #4a3d8a;
    }

    #status-line {
        height: auto;
        margin: 1 0 0 0;
        color: #a89fd4;
        text-align: center;
    }
    """

    BINDINGS = [
        Binding('q', 'quit', 'Quit'),
        Binding('space', 'toggle_go', 'GO/STOP', show=True),
        Binding('r', 'toggle_reverse', 'Reverse', show=True),
    ]

    def __init__(self, ros: RosState) -> None:
        super().__init__()
        self.ros = ros

    def compose(self) -> ComposeResult:
        speed = self.ros.params[KEY_SPEED]
        follow_mode = self.ros.params[self.ros.follow_mode_key]
        # Switch OFF = Autonomous (teleop); ON = Follow (auto).
        auto_mode = follow_mode.get() == 'auto'

        yield Header(show_clock=False)
        with Vertical(id='content'):
            with Horizontal(classes='mode-row'):
                yield Label('Autonomous', classes='mode-label')
                yield Switch(value=auto_mode, id='mode-switch')
                yield Label('Follow', classes='mode-label')
            yield Rule(line_style='heavy')
            with Horizontal(classes='go-row'):
                yield Button('GO', id='go-button', classes='-go', disabled=auto_mode)
            with Vertical(classes='speed-panel'):
                yield Label('Speed (m/s)', classes='speed-label')
                with Horizontal(classes='speed-row'):
                    yield Input(
                        f'{speed.get():.2f}',
                        type='number',
                        id='speed-input',
                        validators=[Number(minimum=speed.min, maximum=speed.max)],
                    )
                    yield Button('-', classes='step-button', id='speed-down')
                    yield Button('+', classes='step-button', id='speed-up')
                    yield Button('⟲ Reverse', id='reverse-button', variant='default')
            yield Static(self._status_text(), id='status-line')
        yield Footer()

    def on_mount(self) -> None:
        interval = 1.0 / max(self.ros.publish_rate_hz, 1.0)
        self.set_interval(interval, self._tick)
        self._update_from_ros()

    def _status_text(self) -> str:
        speed = self.ros.params[KEY_SPEED].get()
        going = self.ros.params[KEY_GOING].get()
        auto_mode = self.ros.params[self.ros.follow_mode_key].get() == 'auto'
        mode = 'Follow' if auto_mode else 'Autonomous'
        motion = 'GO' if going else 'STOP'
        direction = 'reverse' if speed < 0 else 'forward'
        return f'{mode} · {motion} · {direction} · {speed:+.2f} m/s'

    def _update_from_ros(self) -> None:
        """Refresh all widgets from self.ros.params."""
        speed = self.ros.params[KEY_SPEED]
        going = self.ros.params[KEY_GOING]
        follow_mode = self.ros.params[self.ros.follow_mode_key]
        auto_mode = follow_mode.get() == 'auto'

        switch = self.query_one('#mode-switch', Switch)
        if switch.value != auto_mode:
            switch.value = auto_mode

        button = self.query_one('#go-button', Button)
        button.disabled = auto_mode
        if going.get():
            button.label = 'STOP'
            button.remove_class('-go')
            button.add_class('-stop')
        else:
            button.label = 'GO'
            button.remove_class('-stop')
            button.add_class('-go')

        speed_input = self.query_one('#speed-input', Input)
        if not speed_input.has_focus:
            text = f'{speed.get():.2f}'
            if speed_input.value != text:
                speed_input.value = text

        self.query_one('#status-line', Static).update(self._status_text())

    def _tick(self) -> None:
        self.ros.spin_once()
        self.ros.publish_tick()
        if self.ros.is_dirty():
            self._update_from_ros()
            self.ros.clear_dirty()

    @work(thread=True)
    def _apply_follow_mode(self, mode: str) -> None:
        param = self.ros.params[self.ros.follow_mode_key]
        if param.set(mode):
            return
        value = self.ros.fetch_remote_param(param.node_name, param.param_name)
        if value is not None:
            param.syncup(value)
        self.call_from_thread(self._update_from_ros)

    @on(Switch.Changed, '#mode-switch')
    def on_mode_changed(self, event: Switch.Changed) -> None:
        mode = 'auto' if event.value else 'teleop'
        follow_mode = self.ros.params[self.ros.follow_mode_key]
        # Ignore programmatic switch updates from _update_from_ros.
        if follow_mode.get() == mode:
            return
        if event.value:
            self.ros.params[KEY_GOING].set(False)
        follow_mode.assign(mode)  # optimistic until remote set confirms
        self._apply_follow_mode(mode)
        self._update_from_ros()

    @on(Button.Pressed, '#go-button')
    def on_go_pressed(self) -> None:
        if self.ros.params[self.ros.follow_mode_key].get() == 'auto':
            return
        going = self.ros.params[KEY_GOING]
        going.set(not going.get())
        self._update_from_ros()

    @on(Button.Pressed, '#speed-down')
    def on_speed_down(self) -> None:
        speed = self.ros.params[KEY_SPEED]
        speed.set(speed.get() - speed.step)
        self._update_from_ros()

    @on(Button.Pressed, '#speed-up')
    def on_speed_up(self) -> None:
        speed = self.ros.params[KEY_SPEED]
        speed.set(speed.get() + speed.step)
        self._update_from_ros()

    @on(Button.Pressed, '#reverse-button')
    def on_reverse_pressed(self) -> None:
        speed = self.ros.params[KEY_SPEED]
        speed.set(speed.get() * -1)
        self._update_from_ros()

    @on(Input.Changed, '#speed-input')
    def on_speed_input_changed(self, event: Input.Changed) -> None:
        if not event.validation_result or not event.validation_result.is_valid:
            return
        try:
            self.ros.params[KEY_SPEED].set(float(event.value))
            self._update_from_ros()
        except ValueError:
            pass

    def action_toggle_go(self) -> None:
        if self.ros.params[self.ros.follow_mode_key].get() == 'teleop':
            self.on_go_pressed()

    def action_toggle_reverse(self) -> None:
        self.on_reverse_pressed()

    def on_unmount(self) -> None:
        self.ros.publish_stop()
