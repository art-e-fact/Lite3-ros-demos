"""Instantiate the procedural scenes (``railroad``, ``blocks``) in Newton.

The scene modules under ``scenes/`` describe each world in a backend-neutral
form (numpy meshes, boxes, an elevation grid, a follow-target path). This module
is the Newton consumer of that description; ``scenes/`` stays free of any
physics-engine import.

Build order matters and is enforced by the two-phase API:

1. :meth:`NewtonProceduralScene.add_world` — static world geometry, added to the
   builder *before* the robot MJCF so ``joint_q[0:7+ndof]`` stays the robot's
   free joint and the convex-hull pass can target the robot shape range.
2. :meth:`NewtonProceduralScene.add_follow_target` — the kinematic marker body,
   added *after* the robot for the same reason.
"""

from __future__ import annotations

import math

import numpy as np
import warp as wp

import newton

from scenes.procedural_railroad_scene import (
    RailroadSceneGeometry,
    build_railroad_scene,
    railroad_scene_meta,
)
from scenes.procedural_scene_generator import (
    BlocksSceneGeometry,
    blocks_scene_meta,
    build_blocks_geometry,
)

# Reserves XY as the blocks-scene
# free-space seed node and spawns the robot there.
BLOCKS_ROBOT_START_XY = (-5.0, 0.0)

FREE_JOINT_Q_COUNT = 7
FREE_JOINT_QD_COUNT = 6

# The marker is kinematic, so its mass never enters the dynamics
_MARKER_MASS = 1.0
_MARKER_INERTIA = 0.1


# Box corner order: bit 0 = +x, bit 1 = +y, bit 2 = +z.
_BOX_CORNERS = np.array(
    [(sx, sy, sz) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)], dtype=np.float64
)[[0, 1, 3, 2, 4, 5, 7, 6]]
_BOX_FACES = np.array(
    [
        (0, 3, 2), (0, 2, 1),  # -z
        (4, 5, 6), (4, 6, 7),  # +z
        (0, 1, 5), (0, 5, 4),  # -y
        (3, 7, 6), (3, 6, 2),  # +y
        (0, 4, 7), (0, 7, 3),  # -x
        (1, 2, 6), (1, 6, 5),  # +x
    ],
    dtype=np.int32,
)


def _boxes_to_mesh(boxes) -> tuple[np.ndarray, np.ndarray]:
    """Merge yaw-rotated boxes into one CCW triangle mesh in world coordinates."""
    vertices = []
    faces = []
    for index, box in enumerate(boxes):
        cos_y, sin_y = math.cos(box.yaw), math.sin(box.yaw)
        local = _BOX_CORNERS * np.asarray(box.half_size, dtype=np.float64)
        rotated = np.column_stack((
            cos_y * local[:, 0] - sin_y * local[:, 1],
            sin_y * local[:, 0] + cos_y * local[:, 1],
            local[:, 2],
        ))
        vertices.append(rotated + np.asarray(box.pos, dtype=np.float64))
        faces.append(_BOX_FACES + 8 * index)
    return np.concatenate(vertices), np.concatenate(faces)


def _yaw_quat(yaw: float) -> wp.quat:
    return wp.quat(0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def _rgb(rgba) -> tuple[float, float, float]:
    return float(rgba[0]), float(rgba[1]), float(rgba[2])


def _static_cfg(builder: newton.ModelBuilder) -> newton.ModelBuilder.ShapeConfig:
    """Shape config for world-static scene geometry (``default_shape_cfg`` is shared)."""
    cfg = builder.default_shape_cfg.copy()
    cfg.density = 0.0
    return cfg


def _visual_only_cfg(builder: newton.ModelBuilder) -> newton.ModelBuilder.ShapeConfig:
    """Shape config for a shape lidar rays hit but nothing collides with.

    ``SensorTiledCamera`` keys on ``ShapeFlags.VISIBLE``, so a visual-only shape is
    still returned by the lidar; the robot walks straight through it.
    """
    cfg = builder.default_shape_cfg.copy()
    cfg.density = 0.0
    cfg.has_shape_collision = False
    cfg.has_particle_collision = False
    cfg.is_visible = True
    return cfg


class FollowTargetDriver:
    """Drives a kinematic marker body by writing its free-joint coordinates.

    Follows ``newton/examples/basic/example_basic_conveyor.py``: the host writes
    ``state.joint_q`` outside any captured CUDA graph, then ``eval_fk`` with
    ``body_flag_filter=BodyFlags.KINEMATIC`` (inside the graph is fine) refreshes
    ``body_q`` for collision and the ray sensors.
    """

    def __init__(self, model, joint_index: int, pose_at):
        self.pose_at = pose_at
        self.q_start = int(model.joint_q_start.numpy()[joint_index])
        self.qd_start = int(model.joint_qd_start.numpy()[joint_index])
        self._host = np.zeros(FREE_JOINT_Q_COUNT, dtype=np.float32)
        self._pose = wp.zeros(FREE_JOINT_Q_COUNT, dtype=wp.float32, device=model.device)
        self._zero_qd = wp.zeros(FREE_JOINT_QD_COUNT, dtype=wp.float32, device=model.device)

    def update(self, state, sim_time_s: float):
        x, y, z, yaw = self.pose_at(sim_time_s)
        half = 0.5 * yaw
        self._host[:] = (x, y, z, 0.0, 0.0, math.sin(half), math.cos(half))
        self._pose.assign(self._host)
        wp.copy(state.joint_q, self._pose, dest_offset=self.q_start, count=FREE_JOINT_Q_COUNT)
        wp.copy(state.joint_qd, self._zero_qd, dest_offset=self.qd_start, count=FREE_JOINT_QD_COUNT)


class NewtonProceduralScene:
    """Base class: a procedural world plus the metadata the ROS runner republishes."""

    name = ""

    def __init__(self):
        self.meta: dict = {}
        self.robot_start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.marker_joint: int = -1

    def add_world(self, builder: newton.ModelBuilder) -> bool:
        """Add the static world. Returns True when it provides its own ground."""
        raise NotImplementedError

    def add_follow_target(self, builder: newton.ModelBuilder) -> None:
        """Add the kinematic follow-target body. Called after the robot is added."""

    def target_pose_at(self, sim_time_s: float):
        """``(x, y, z, yaw)`` of the follow target, or None when the scene has none."""
        return None

    def make_driver(self, model) -> FollowTargetDriver | None:
        if self.marker_joint < 0:
            return None
        return FollowTargetDriver(model, self.marker_joint, self.target_pose_at)

    def describe(self) -> str:
        return self.name


class NewtonRailroadScene(NewtonProceduralScene):
    """``procedural://railroad``: rails, sleepers, terrain and the moving UWB tag."""

    name = "railroad"

    def __init__(self, seed: int, **scene_kwargs):
        super().__init__()
        self.scene = build_railroad_scene(seed, **scene_kwargs)
        self.geometry: RailroadSceneGeometry = self.scene.geometry
        self.meta = railroad_scene_meta(self.scene)
        self.robot_start_pose = self.geometry.robot_start_pose

    def add_world(self, builder: newton.ModelBuilder) -> bool:
        cfg = _static_cfg(builder)
        geometry = self.geometry

        rail_vertices, rail_faces = _boxes_to_mesh(geometry.rail_colliders)
        if len(rail_faces):
            mesh = newton.Mesh(
                vertices=np.ascontiguousarray(rail_vertices, dtype=np.float32),
                indices=np.ascontiguousarray(rail_faces, dtype=np.int32).ravel(),
                compute_inertia=False,
            )
            builder.add_shape_mesh(
                -1,
                mesh=mesh,
                cfg=cfg,
                color=_rgb(geometry.rail_meshes[0].rgba if geometry.rail_meshes else (0.55, 0.55, 0.6, 1.0)),
                label="procedural_railroad/rails",
            )

        for box in geometry.sleepers:
            builder.add_shape_box(
                -1,
                xform=wp.transform(wp.vec3(*box.pos), _yaw_quat(box.yaw)),
                hx=box.half_size[0],
                hy=box.half_size[1],
                hz=box.half_size[2],
                cfg=cfg,
                color=_rgb(box.rgba),
                label=f"procedural_railroad/{box.name}",
            )

        grid = geometry.terrain
        if grid is None:
            builder.add_ground_plane()
            return True

        # Heightfield rows run along +Y and columns along +X, matching TerrainGrid.
        # Passing min_z/max_z explicitly makes the internal [0, 1] normalisation
        # round-trip back to the exact elevations the MJCF hfield uses.
        half_x, half_y = grid.half_extent_xy
        center_x, center_y = grid.center_xy
        heightfield = newton.Heightfield(
            data=np.ascontiguousarray(grid.elevation, dtype=np.float32),
            nrow=grid.nrow,
            ncol=grid.ncol,
            hx=half_x,
            hy=half_y,
            min_z=grid.min_z,
            max_z=grid.max_z,
        )
        builder.add_shape_heightfield(
            xform=wp.transform(wp.vec3(center_x, center_y, 0.0), wp.quat_identity()),
            heightfield=heightfield,
            cfg=cfg,
            color=_rgb(grid.rgba),
            label="procedural_railroad/terrain",
        )
        return True

    def add_follow_target(self, builder: newton.ModelBuilder) -> None:
        target = self.geometry.follow_target
        if not target.path.valid:
            return

        x, y, z, yaw = target.pose_at(0.0)
        body = builder.add_body(
            xform=wp.transform(wp.vec3(x, y, z), _yaw_quat(yaw)),
            mass=_MARKER_MASS,
            inertia=wp.mat33(
                _MARKER_INERTIA, 0.0, 0.0,
                0.0, _MARKER_INERTIA, 0.0,
                0.0, 0.0, _MARKER_INERTIA,
            ),
            is_kinematic=True,
            label=target.name,
        )  # fmt: skip
        builder.add_shape_cylinder(
            body,
            radius=target.radius,
            half_height=target.half_height,
            cfg=_visual_only_cfg(builder),
            color=_rgb(target.rgba),
            label=f"{target.name}/marker",
        )
        self.marker_joint = builder.joint_count - 1

    def target_pose_at(self, sim_time_s: float):
        return self.geometry.target_pose_at(sim_time_s)

    def describe(self) -> str:
        geometry = self.geometry
        terrain = "none" if geometry.terrain is None else f"{geometry.terrain.nrow}x{geometry.terrain.ncol}"
        return (
            f"railroad seed={self.meta['seed']}, roads={geometry.road_count}, "
            f"rail_meshes={len(geometry.rail_meshes)}, sleepers={len(geometry.sleepers)}, "
            f"terrain={terrain}, mainline_waypoints={len(geometry.mission_xy)}"
        )


class NewtonBlocksScene(NewtonProceduralScene):
    """``procedural://blocks``: a ground plane with random primitive obstacles."""

    name = "blocks"

    def __init__(self, seed: int, robot_start_xy=BLOCKS_ROBOT_START_XY):
        super().__init__()
        self.geometry: BlocksSceneGeometry = build_blocks_geometry(robot_start_xy, seed)
        self.meta = blocks_scene_meta(self.geometry)
        self.meta["blocks_geometry"] = self.geometry
        self.robot_start_pose = self.geometry.robot_start_pose

    def add_world(self, builder: newton.ModelBuilder) -> bool:
        cfg = _static_cfg(builder)
        for obstacle in self.geometry.obstacles:
            xform = wp.transform(wp.vec3(*obstacle.pos), _yaw_quat(obstacle.yaw))
            color = _rgb(obstacle.rgba)
            label = f"procedural_blocks/{obstacle.name}"
            size = obstacle.size
            if obstacle.shape == "box":
                builder.add_shape_box(
                    -1, xform=xform, hx=size[0], hy=size[1], hz=size[2], cfg=cfg, color=color, label=label
                )
            elif obstacle.shape == "cylinder":
                builder.add_shape_cylinder(
                    -1, xform=xform, radius=size[0], half_height=size[1], cfg=cfg, color=color, label=label
                )
            elif obstacle.shape == "capsule":
                builder.add_shape_capsule(
                    -1, xform=xform, radius=size[0], half_height=size[1], cfg=cfg, color=color, label=label
                )
            else:
                raise ValueError(f"Unsupported blocks obstacle shape: {obstacle.shape}")

        builder.add_ground_plane()
        return True

    def describe(self) -> str:
        return (
            f"blocks seed={self.meta['seed']}, nodes={self.meta['nodes']}, "
            f"edges={self.meta['edges']}, obstacles={self.meta['obstacles']}, "
            f"mission_waypoints={len(self.meta['mission_xy'])}"
        )


SCENE_FACTORIES = {
    NewtonRailroadScene.name: NewtonRailroadScene,
    NewtonBlocksScene.name: NewtonBlocksScene,
}


def build_newton_procedural_scene(name: str, seed: int, **kwargs) -> NewtonProceduralScene:
    """Create the Newton-side procedural scene called *name* with *seed*."""
    factory = SCENE_FACTORIES.get(str(name).strip().lower())
    if factory is None:
        supported = ", ".join(sorted(SCENE_FACTORIES))
        raise ValueError(f"Unsupported procedural scene '{name}' for Newton, expected one of: {supported}")
    return factory(seed, **kwargs)
