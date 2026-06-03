"""Shared Lite3 sensor resource resolution helpers."""

from __future__ import annotations

from simulation_config import resolve_path


def resolve_lite3_resource(*parts: str):
    return resolve_path(f"package://simulation_package/assets/{'/'.join(parts)}", must_exist=False)


D435I_XML_PATH = resolve_lite3_resource("lite3_mjcf", "realsense_d435i", "d435i.xml")
MID360_XML_PATH = resolve_lite3_resource("lite3_mjcf", "mid360", "mid360.xml")