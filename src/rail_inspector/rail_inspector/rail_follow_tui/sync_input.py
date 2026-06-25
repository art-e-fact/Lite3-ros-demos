"""Set Input/Switch values without posting change events."""

from __future__ import annotations

from textual.message_pump import MessagePump
from textual.widgets import Input, Switch


def sync_input(pump: MessagePump, inp: Input, text: str) -> None:
    if inp.value == text:
        return
    with pump.prevent(Input.Changed):
        inp.value = text


def sync_switch(pump: MessagePump, switch: Switch, value: bool) -> None:
    if switch.value == value:
        return
    with pump.prevent(Switch.Changed):
        switch.value = value
