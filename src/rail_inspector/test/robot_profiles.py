from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


@dataclass(frozen=True)
class RobotProfile:
    name: str
    rerun_body: str
    params_file_name: str
    deploy_package: str
    sim_robot: dict[str, Any]
    sensors: dict[str, Any]
    follow_camera_target_height_m: float = 0.55

    def body_path(self) -> str:
        return f"/bodies/{self.rerun_body}"

    def body_translation_col(self) -> str:
        return f"{self.body_path()}:Transform3D:translation"

    def resolve_params_file(self) -> str:
        if get_package_share_directory is None:
            raise RuntimeError(
                "ament_index_python is required to resolve rail_inspector config paths"
            )
        pkg_share = get_package_share_directory("rail_inspector")
        return str(Path(pkg_share) / "config" / self.params_file_name)


ROBOT_PROFILES: dict[str, RobotProfile] = {
    "lite3": RobotProfile(
        name="lite3",
        rerun_body="TORSO",
        params_file_name="rail_follow_sim.yaml",
        deploy_package="lite3_sdk_deploy",
        sim_robot={},
        sensors={
            "mid360": {"enabled": True},
            "robosense": {"enabled": False},
        },
        follow_camera_target_height_m=0.55,
    ),
    "m20": RobotProfile(
        name="m20",
        rerun_body="base_link",
        params_file_name="rail_follow_sim_m20.yaml",
        deploy_package="m20_sdk_deploy",
        sim_robot={"model": "m20"},
        sensors={
            "mid360": {"enabled": False},
            "robosense": {
                "enabled": True,
                "channels": 48,
                "columns": 224,
                "column_downsample": 1,
            },
        },
        follow_camera_target_height_m=0.9,
    ),
}


def get_robot_profile(name: str) -> RobotProfile:
    try:
        return ROBOT_PROFILES[name]
    except KeyError as exc:
        supported = ", ".join(sorted(ROBOT_PROFILES))
        raise ValueError(f"Unknown robot profile {name!r}; choose from: {supported}") from exc
