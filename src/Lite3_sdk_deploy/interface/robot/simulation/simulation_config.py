from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, get_type_hints

try:
	import yaml
except ImportError:  # pragma: no cover - reported when config loading is used
	yaml = None

try:
	from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - source-tree fallback handles this
	get_package_share_directory = None


PACKAGE_NAME = "lite3_sdk_deploy"
DEFAULT_SCENE_URI = "package://lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf/stairs_floors.xml"
DEFAULT_ROBOT_DESCRIPTION_URI = "package://lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf/Lite3.xml"
DEFAULT_USD_URI = "package://lite3_sdk_deploy/Lite3_description/Lite3_usd/Lite3.usd"


@dataclass
class RealsenseConfig:
	enable_depth: bool = False
	enable_color: bool = False
	enable_pointcloud: bool = False
	width: int = 640
	height: int = 480
	frequency_hz: float = 15.0
	depth_range_min: float = 0.105
	depth_range_max: float = 10.0
	depth_fx: float = 382.68
	depth_fy: float = 382.68
	depth_cx: float = 320.0
	depth_cy: float = 240.0
	color_fx: float = 615.69
	color_fy: float = 615.69
	color_cx: float = 320.0
	color_cy: float = 240.0
	depth_image_topic: str = "/camera/depth/image_rect_raw"
	depth_info_topic: str = "/camera/depth/camera_info"
	color_image_topic: str = "/camera/color/image_raw"
	color_info_topic: str = "/camera/color/camera_info"
	pointcloud_topic: str = "/camera/depth/color/points"
	depth_optical_frame: str = "camera_depth_optical_frame"
	color_optical_frame: str = "camera_color_optical_frame"
	camera_link_frame: str = "camera_link"
	depth_frame: str = "camera_depth_frame"
	color_frame: str = "camera_color_frame"
	mount_site_name: str = "d435i_mount"
	depth_camera_name: str = "d435i-depth"
	color_camera_name: str = "d435i-color"
	forward_offset: list[float] = field(default_factory=lambda: [0.01, 0.0, 0.0])

	@property
	def enabled(self) -> bool:
		return self.enable_depth or self.enable_color


@dataclass
class Lidar2DConfig:
	enabled: bool = False
	site_name: str = "lidar_site"
	frame_id: str = "lidar"
	topic: str = "/scan"
	frequency_hz: float = 10.0
	num_rays: int = 360
	range_min: float = 0.15
	range_max: float = 8.0
	visualize_rays: bool = False


@dataclass
class Mid360Config:
	enabled: bool = False
	mount_site_name: str = "mid360_mount"
	attachment_prefix: str = "mid360-"
	sensor_site_name: str = "lidar_frame"
	frame_id: str = "mid360"
	topic: str = "/mid360/points"
	frequency_hz: float = 10.0
	pattern_file: str = "mid360.npy"
	samples_per_scan: int = 24000
	range_min: float = 0.1
	range_max: float = 200.0
	downsample: int = 1

	@property
	def attached_site_name(self) -> str:
		return f"{self.attachment_prefix}{self.sensor_site_name}"


@dataclass
class SensorsConfig:
	realsense: RealsenseConfig = field(default_factory=RealsenseConfig)
	lidar_2d: Lidar2DConfig = field(default_factory=Lidar2DConfig)
	mid360: Mid360Config = field(default_factory=Mid360Config)


@dataclass
class SimulationConfig:
	simulator: str = "newton"
	scene: str = DEFAULT_SCENE_URI
	robot_description: str = DEFAULT_ROBOT_DESCRIPTION_URI
	headless: bool = True
	use_procedural_scene: bool = False
	procedural_env_seed: int = -1
	sensors: SensorsConfig = field(default_factory=SensorsConfig)

	@classmethod
	def load(cls, config_path: str | None = None) -> "SimulationConfig":
		data = _dataclass_to_dict(cls())
		if config_path:
			data = _deep_merge(data, _read_yaml(resolve_path(config_path)))
		return cls.from_dict(data)

	@classmethod
	def from_dict(cls, data: dict[str, Any] | None) -> "SimulationConfig":
		return _dataclass_from_dict(cls, data or {})

	def with_overrides(self, overrides: dict[str, Any]) -> "SimulationConfig":
		data = _dataclass_to_dict(self)
		data = _deep_merge(data, _unflatten(overrides))
		return self.from_dict(data)

	def resolved_scene(self) -> str | None:
		raw = str(self.scene).strip()
		if not raw:
			return None
		return str(resolve_path(raw))

	def resolved_robot_description(self) -> str:
		return str(resolve_path(self.robot_description))

	def validate(self) -> list[str]:
		errors: list[str] = []
		simulator = self.simulator.lower()
		if simulator not in {"newton", "mujoco"}:
			errors.append("simulator must be 'newton' or 'mujoco'")

		scene_value = str(self.scene).strip()
		scene_path = resolve_path(scene_value, must_exist=False) if scene_value else None
		robot_path = resolve_path(self.robot_description, must_exist=False)
		if scene_path is not None and scene_path.suffix.lower() not in {".xml", ".mjcf"}:
			errors.append("scene must be an MJCF/XML file")
		if robot_path.suffix.lower() == ".usd" and simulator != "newton":
			errors.append("USD robot_description is only supported by the Newton simulator")
		if robot_path.suffix.lower() not in {".xml", ".mjcf", ".usd"}:
			errors.append("robot_description must be an MJCF/XML or USD file")
		if scene_path is not None and not scene_path.exists():
			errors.append(f"scene does not exist: {scene_path}")
		if not robot_path.exists():
			errors.append(f"robot_description does not exist: {robot_path}")

		sensors = self.sensors
		if sensors.realsense.enable_pointcloud and not sensors.realsense.enable_depth:
			errors.append("sensors.realsense.enable_pointcloud requires sensors.realsense.enable_depth")
		_validate_positive(errors, "sensors.realsense.width", sensors.realsense.width)
		_validate_positive(errors, "sensors.realsense.height", sensors.realsense.height)
		_validate_positive(errors, "sensors.realsense.frequency_hz", sensors.realsense.frequency_hz)
		_validate_range(errors, "sensors.realsense.depth", sensors.realsense.depth_range_min, sensors.realsense.depth_range_max)
		_validate_positive(errors, "sensors.lidar_2d.frequency_hz", sensors.lidar_2d.frequency_hz)
		_validate_positive(errors, "sensors.lidar_2d.num_rays", sensors.lidar_2d.num_rays)
		_validate_range(errors, "sensors.lidar_2d", sensors.lidar_2d.range_min, sensors.lidar_2d.range_max)
		_validate_positive(errors, "sensors.mid360.frequency_hz", sensors.mid360.frequency_hz)
		_validate_positive(errors, "sensors.mid360.samples_per_scan", sensors.mid360.samples_per_scan)
		_validate_positive(errors, "sensors.mid360.downsample", sensors.mid360.downsample)
		_validate_range(errors, "sensors.mid360", sensors.mid360.range_min, sensors.mid360.range_max)
		return errors

def candidate_package_roots() -> list[Path]:
	roots: list[Path] = []
	seen: set[Path] = set()

	if get_package_share_directory is not None:
		try:
			package_root = Path(get_package_share_directory(PACKAGE_NAME)).resolve()
			roots.append(package_root)
			seen.add(package_root)
		except Exception:
			pass

	for parent in Path(__file__).resolve().parents:
		candidates = (parent, parent / "share" / PACKAGE_NAME)
		for candidate in candidates:
			resolved = candidate.resolve()
			if resolved in seen:
				continue
			if (resolved / "Lite3_description").exists() or (resolved / "config").exists():
				roots.append(resolved)
				seen.add(resolved)
	if not roots:
		roots.append(Path(__file__).resolve().parents[3])
	return roots


def resolve_path(path_value: str | os.PathLike[str], must_exist: bool = False) -> Path:
	raw = str(path_value).strip()
	if raw.startswith("package://"):
		package_and_path = raw[len("package://") :]
		package, _, rel = package_and_path.partition("/")
		if package.lower() != PACKAGE_NAME:
			raise ValueError(f"Unsupported package URI '{raw}', expected package://{PACKAGE_NAME}/...")
		rel_path = Path(rel)
		fallback = None
		for root in candidate_package_roots():
			candidate = (root / rel_path).resolve()
			if candidate.exists():
				return candidate
			if fallback is None:
				fallback = candidate
		if fallback is not None and not must_exist:
			return fallback

	candidate = Path(raw).expanduser()
	if candidate.is_absolute():
		return candidate.resolve()
	if candidate.exists():
		return candidate.resolve()
	fallback = None
	for root in candidate_package_roots():
		for base in (
			root,
			root / "Lite3_description" / "lite3_mjcf" / "mjcf",
			root / "Lite3_description" / "Lite3_usd",
			root / "config",
		):
			resolved = (base / candidate).resolve()
			if resolved.exists():
				return resolved
			if fallback is None:
				fallback = resolved
	if fallback is not None and not must_exist:
		return fallback
	return candidate.resolve()


def _read_yaml(path: Path) -> dict[str, Any]:
	if yaml is None:
		raise RuntimeError("PyYAML is required to load simulation configuration")
	if not path.exists():
		return {}
	with path.open("r", encoding="utf-8") as config_file:
		data = yaml.safe_load(config_file) or {}
	if not isinstance(data, dict):
		raise ValueError(f"Simulation config must be a YAML mapping: {path}")
	return data


def _dataclass_from_dict(cls, data: dict[str, Any]):
	kwargs = {}
	field_map = {field.name: field for field in fields(cls)}
	type_hints = get_type_hints(cls)
	for name, field_info in field_map.items():
		value = data.get(name)
		if value is None:
			continue
		field_type = type_hints.get(name, field_info.type)
		if hasattr(field_type, "__dataclass_fields__"):
			kwargs[name] = _dataclass_from_dict(field_type, value)
		else:
			kwargs[name] = value
	return cls(**kwargs)


def _dataclass_to_dict(value):
	if is_dataclass(value):
		return {field_info.name: _dataclass_to_dict(getattr(value, field_info.name)) for field_info in fields(value)}
	if isinstance(value, list):
		return [_dataclass_to_dict(item) for item in value]
	return copy.deepcopy(value)


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
	merged = copy.deepcopy(base)
	for key, value in override.items():
		if isinstance(value, dict) and isinstance(merged.get(key), dict):
			merged[key] = _deep_merge(merged[key], value)
		else:
			merged[key] = copy.deepcopy(value)
	return merged


def _unflatten(overrides: dict[str, Any]) -> dict[str, Any]:
	result: dict[str, Any] = {}
	for dotted_key, value in overrides.items():
		current = result
		parts = dotted_key.split(".")
		for part in parts[:-1]:
			current = current.setdefault(part, {})
		current[parts[-1]] = value
	return result


def _validate_positive(errors: list[str], name: str, value: int | float):
	if value <= 0:
		errors.append(f"{name} must be positive")


def _validate_range(errors: list[str], name: str, min_value: float, max_value: float):
	if min_value < 0:
		errors.append(f"{name}_range_min must be non-negative")
	if max_value <= min_value:
		errors.append(f"{name}_range_max must be greater than range_min")
