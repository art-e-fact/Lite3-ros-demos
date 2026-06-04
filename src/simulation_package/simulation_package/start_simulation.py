#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import yaml

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))
NEWTON_DIR = CURRENT_DIR / "newton"
if str(NEWTON_DIR) not in sys.path:
    sys.path.insert(0, str(NEWTON_DIR))

from simulation_config import SimulationConfig


def _load_newton_runner():
    module_path = NEWTON_DIR / "newton_simulation_ros2.py"
    spec = importlib.util.spec_from_file_location("lite3_newton_simulation_ros2", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.run_newton


def _load_mujoco_runner():
    module_path = CURRENT_DIR / "mujoco_simulation_ros2.py"
    spec = importlib.util.spec_from_file_location("lite3_mujoco_simulation_ros2", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.run_mujoco

def _parse_args():
    import argparse

    parser = argparse.ArgumentParser(description="Start the configured Lite3 simulator")
    parser.add_argument("--config", default=None, help="Path to simulation YAML config")
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="Override a simulation config field using dotted syntax, for example sensors.lidar_2d.enabled=true",
    )
    return parser.parse_known_args()


def _parse_overrides(items: list[str]) -> dict[str, object]:
    overrides: dict[str, object] = {}
    for item in items:
        key, separator, raw_value = item.partition("=")
        if not separator:
            raise SystemExit(f"Invalid simulation override '{item}', expected PATH=VALUE")

        dotted_key = key.strip()
        if not dotted_key:
            raise SystemExit(f"Invalid simulation override '{item}', PATH cannot be empty")

        overrides[dotted_key] = yaml.safe_load(raw_value)
    return overrides


def main():
    args, ros_args = _parse_args()
    config = SimulationConfig.load(args.config).with_overrides(_parse_overrides(args.set))
    errors = config.validate()
    if errors:
        raise SystemExit("Invalid simulation config:\n- " + "\n- ".join(errors))

    if config.simulator.lower() == "newton":
        _load_newton_runner()(config, ros_args)
    elif config.simulator.lower() == "mujoco":
        _load_mujoco_runner()(config, ros_args)
    else:
        raise SystemExit(f"Unsupported simulator: {config.simulator}")


if __name__ == "__main__":
    main()
