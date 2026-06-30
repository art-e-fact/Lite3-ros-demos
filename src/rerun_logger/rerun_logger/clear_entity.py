"""Helpers for clearing Rerun entities."""

from __future__ import annotations

import rerun as rr


def clear_entity(path: str, *, static: bool = False) -> None:
    rr.log(path, rr.Clear(recursive=False), static=static)
