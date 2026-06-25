"""Set Input values without posting Input.Changed."""

from __future__ import annotations

from textual.message_pump import MessagePump
from textual.widgets import Input


def sync_input(pump: MessagePump, inp: Input, text: str) -> None:
    if inp.value == text:
        return
    with pump.prevent(Input.Changed):
        inp.value = text
