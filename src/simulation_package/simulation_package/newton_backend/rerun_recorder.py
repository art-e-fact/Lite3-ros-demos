"""Rerun recording for the Newton backend.

Mirrors the entity-path scheme produced by ``rerun_loader_mjcf`` so that
recordings made with either backend can be queried with the same columns:

* ``bodies/<body_name>``                        -> ``rr.Transform3D``
* ``visual_geometries/<body_name>/<shape_name>`` -> geometry attached to the
  body frame via ``rr.CoordinateFrame("tf#/bodies/<body_name>")`` plus an
  ``rr.InstancePoses3D`` local offset.

Newton has no MuJoCo model to hand to ``rerun_loader_mjcf.MJCFLogger``, so the
geometry logging is reimplemented here directly against the Rerun SDK.
"""

from __future__ import annotations

import os
import signal

import numpy as np
import rerun as rr

import newton

# Rerun implicit-frame prefix; see rerun_loader_mjcf.__init__ for the rationale.
_TF = "tf#"

BODIES_ROOT = "bodies"
VISUAL_ROOT = "visual_geometries"
TIMELINE_NAME = "sim_time"

# Newton uses -1 for "attached to the world"; MuJoCo names that body "world".
WORLD_BODY_NAME = "world"

# Full extent used when a plane is stored with a non-positive (infinite) scale.
INFINITE_PLANE_EXTENT_M = 200.0

# gRPC port for a viewer this process spawns. Matches rerun's own default.
RERUN_VIEWER_PORT = 9876
RERUN_FLUSH_TIMEOUT_SEC = 10.0

_RGBA_MAX = 255
_DEFAULT_COLOR = (0.6, 0.6, 0.6)


def _last_segment(label: str) -> str:
    text = label.decode() if isinstance(label, bytes) else str(label)
    return text.rsplit("/", 1)[-1] or text


def _deduplicate(names: list[str]) -> list[str]:
    """Make names unique by appending _1, _2, ... to repeats."""
    seen: dict[str, int] = {}
    unique: list[str] = []
    for name in names:
        if name not in seen:
            seen[name] = 0
            unique.append(name)
            continue
        seen[name] += 1
        candidate = f"{name}_{seen[name]}"
        while candidate in seen:
            seen[name] += 1
            candidate = f"{name}_{seen[name]}"
        seen[candidate] = 0
        unique.append(candidate)
    return unique


def _rgba_uint8(color) -> np.ndarray:
    rgb = np.asarray(color, dtype=np.float64).reshape(-1)[:3]
    if not np.all(np.isfinite(rgb)):
        rgb = np.asarray(_DEFAULT_COLOR, dtype=np.float64)
    rgba = np.concatenate([np.clip(rgb, 0.0, 1.0), [1.0]])
    return (rgba * _RGBA_MAX).astype(np.uint8)


def _box_mesh(hx: float, hy: float, hz: float):
    """24-vertex box with hard-edge normals (same layout as the MJCF logger)."""
    vertices = np.array(
        [
            [hx, -hy, -hz], [hx, hy, -hz], [hx, hy, hz], [hx, -hy, hz],
            [-hx, hy, -hz], [-hx, -hy, -hz], [-hx, -hy, hz], [-hx, hy, hz],
            [hx, hy, -hz], [-hx, hy, -hz], [-hx, hy, hz], [hx, hy, hz],
            [-hx, -hy, -hz], [hx, -hy, -hz], [hx, -hy, hz], [-hx, -hy, hz],
            [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
            [-hx, hy, -hz], [hx, hy, -hz], [hx, -hy, -hz], [-hx, -hy, -hz],
        ],
        dtype=np.float32,
    )
    face_normals = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
    normals = np.repeat(face_normals, 4, axis=0).astype(np.float32)
    faces = np.array(
        [[i * 4 + 0, i * 4 + 1, i * 4 + 2, i * 4 + 0, i * 4 + 2, i * 4 + 3] for i in range(6)],
        dtype=np.int32,
    ).reshape(-1, 3)
    return vertices, faces, normals


def _plane_mesh(width: float, length: float):
    half_x = 0.5 * (width if width > 0.0 else INFINITE_PLANE_EXTENT_M)
    half_y = 0.5 * (length if length > 0.0 else INFINITE_PLANE_EXTENT_M)
    vertices = np.array(
        [
            [-half_x, -half_y, 0.0],
            [half_x, -half_y, 0.0],
            [half_x, half_y, 0.0],
            [-half_x, half_y, 0.0],
        ],
        dtype=np.float32,
    )
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    normals = np.tile(np.array([0.0, 0.0, 1.0], dtype=np.float32), (4, 1))
    return vertices, faces, normals


class NewtonRerunRecorder:
    """Log a Newton model + body trajectories to Rerun.

    Args:
        model: finalized ``newton.Model``.
        robot_name: used for the Rerun application id (``f"{robot_name}_simulation"``).
        config: ``simulation_config.RerunConfig``.
        headless: when True the viewer is never spawned regardless of ``config.spawn``.
        logger: optional object with ``.info()``/``.warn()`` (e.g. a ROS logger).
    """

    def __init__(self, model, robot_name: str, config, headless: bool, logger=None):
        self.model = model
        self.robot_name = robot_name
        self.config = config
        self.headless = bool(headless)
        self.logger = logger
        self.closed = False

        self.body_names = _deduplicate([_last_segment(label) for label in list(model.body_label)])
        self._body_index = {name: index for index, name in enumerate(self.body_names)}
        self._world_name = WORLD_BODY_NAME
        while self._world_name in self._body_index:
            self._world_name = f"{self._world_name}_"

        self._durations: list[float] = []
        self._body_q: list[np.ndarray] = []

        spawn = bool(config.spawn) and not self.headless
        save_path = str(getattr(config, "save_path", "") or "").strip()
        self._log_info(
            f"Initializing Rerun (spawn={spawn}, save_path={save_path or 'None'})"
        )
        # Spawn by hand rather than via rr.init(spawn=True): the viewer is detached into
        # its own session, so a harness that kills our process group cannot reach it and
        # would leak a window. _spawn_viewer hands back the pid so close() can end it.
        rr.init(f"{robot_name}_simulation", spawn=False)
        self.viewer_pid = None
        if spawn:
            self.viewer_pid = self._spawn_viewer()
        if save_path:
            absolute = os.path.abspath(os.path.expanduser(save_path))
            os.makedirs(os.path.dirname(absolute) or ".", exist_ok=True)
            rr.save(absolute)
            self.save_path = absolute
        else:
            self.save_path = ""
        rr.set_time(TIMELINE_NAME, duration=0.0)

    def _spawn_viewer(self):
        """Start a viewer process and connect to it, returning its pid (None on failure)."""
        try:
            from rerun._spawn import _spawn_viewer
        except ImportError as exc:  # pragma: no cover - depends on the rerun-sdk build
            self._log_warn(f"Cannot spawn the Rerun viewer ({exc}); logging without one")
            return None
        try:
            pid = _spawn_viewer(port=RERUN_VIEWER_PORT)
            rr.connect_grpc(f"rerun+http://127.0.0.1:{RERUN_VIEWER_PORT}/proxy")
        except Exception as exc:
            self._log_warn(f"Failed to spawn the Rerun viewer: {exc}")
            return None
        self._log_info(f"Rerun viewer spawned (pid={pid}, port={RERUN_VIEWER_PORT})")
        return pid

    def _terminate_viewer(self) -> None:
        """Stop the viewer this recorder spawned, if the config asks for it."""
        if not getattr(self.config, "close_viewer_on_exit", False) or not self.viewer_pid:
            return
        pid = self.viewer_pid
        self.viewer_pid = None
        # _spawn_viewer starts a python wrapper whose child is the real rerun binary in
        # the wrapper's own process group, so kill the group when we lead it.
        try:
            if os.getpgid(pid) == pid:
                os.killpg(pid, signal.SIGTERM)
            else:
                os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError) as exc:
            self._log_warn(f"Could not stop the Rerun viewer (pid={pid}): {exc}")
            return
        self._log_info(f"Rerun viewer stopped (pid={pid})")

    # -- public API ---------------------------------------------------------

    def body_index(self, name: str) -> int:
        """Index into ``model.body_label`` / ``state.body_q`` for a short body name."""
        if name in self._body_index:
            return self._body_index[name]
        raise ValueError(f"Unknown Newton body '{name}' (known: {', '.join(self.body_names)})")

    def body_path(self, body_name: str) -> str:
        return f"{BODIES_ROOT}/{body_name}"

    def body_frame(self, body_name: str) -> str:
        return f"{_TF}/{self.body_path(body_name)}"

    def log_model(self, state) -> None:
        """Log static geometry for every body plus the initial body transforms."""
        rr.set_time(TIMELINE_NAME, duration=0.0)
        self._log_shapes()
        self._log_body_transforms(state)

    def record(self, state, timestamp: float) -> None:
        """Buffer one frame of body transforms."""
        if self.closed:
            return
        self._durations.append(float(timestamp))
        self._body_q.append(np.asarray(state.body_q.numpy(), dtype=np.float64).copy())

    def flush(self) -> None:
        """Send the buffered frames with Rerun's columnar API."""
        if not self._body_q:
            return

        body_q = np.stack(self._body_q)  # (frames, body_count, 7)
        indexes = [rr.TimeColumn(TIMELINE_NAME, duration=self._durations)]
        for index, name in enumerate(self.body_names):
            rr.send_columns(
                self.body_path(name),
                indexes=indexes,
                columns=rr.Transform3D.columns(
                    translation=body_q[:, index, 0:3],
                    # Newton stores body_q quaternions as xyzw, same as Rerun.
                    quaternion=body_q[:, index, 3:7],
                ),
            )
        self._durations.clear()
        self._body_q.clear()

    def close(self) -> None:
        if self.closed:
            return
        try:
            self.flush()
        finally:
            self.closed = True
        if self.save_path:
            self._log_info(f"Rerun recording saved: {self.save_path}")
        # Drain the sink before the viewer goes away, or the tail of the run is lost.
        try:
            rr.get_global_data_recording().flush(timeout_sec=RERUN_FLUSH_TIMEOUT_SEC)
        except Exception as exc:  # pragma: no cover - best effort
            self._log_warn(f"Rerun flush failed: {exc}")
        self._terminate_viewer()

    # -- geometry -----------------------------------------------------------

    def _log_body_transforms(self, state) -> None:
        body_q = np.asarray(state.body_q.numpy(), dtype=np.float64)
        for index, name in enumerate(self.body_names):
            rr.log(
                self.body_path(name),
                rr.Transform3D(translation=body_q[index, 0:3], quaternion=body_q[index, 3:7]),
            )

    def _log_shapes(self) -> None:
        model = self.model
        shape_count = int(model.shape_count)
        if shape_count == 0:
            return

        shape_body = model.shape_body.numpy()
        shape_flags = model.shape_flags.numpy()
        shape_type = model.shape_type.numpy()
        shape_scale = model.shape_scale.numpy()
        shape_transform = model.shape_transform.numpy()
        shape_color = model.shape_color.numpy() if model.shape_color is not None else None
        shape_source = list(model.shape_source)
        shape_labels = list(model.shape_label)

        visible = int(newton.ShapeFlags.VISIBLE)
        is_site = int(newton.ShapeFlags.SITE)
        collides = int(newton.ShapeFlags.COLLIDE_SHAPES)

        # Group shapes per body, splitting visual-only from collision shapes so we
        # can prefer visuals and fall back to collision geometry (as MJCFLogger does).
        visual: dict[int, list[int]] = {}
        collision: dict[int, list[int]] = {}
        for shape in range(shape_count):
            flags = int(shape_flags[shape])
            if not flags & visible or flags & is_site:
                continue
            body = int(shape_body[shape])
            bucket = collision if flags & collides else visual
            bucket.setdefault(body, []).append(shape)

        for body in sorted(set(visual) | set(collision)):
            body_name = self._world_name if body < 0 else self.body_names[body]
            frame = self.body_frame(body_name)
            shapes = visual.get(body) or collision.get(body) or []
            names = _deduplicate([_last_segment(shape_labels[s]) for s in shapes])
            for shape, shape_name in zip(shapes, names):
                path = f"{VISUAL_ROOT}/{body_name}/{shape_name}"
                color = _rgba_uint8(shape_color[shape] if shape_color is not None else _DEFAULT_COLOR)
                try:
                    self._log_geom_with_frame(
                        path,
                        frame,
                        int(shape_type[shape]),
                        shape_scale[shape],
                        shape_transform[shape],
                        shape_source[shape] if shape < len(shape_source) else None,
                        color,
                    )
                except Exception as exc:  # a single odd shape must not kill the recording
                    self._log_warn(f"Skipping Rerun geometry for shape '{shape_name}': {exc}")

    def _log_geom_with_frame(self, path, frame, geo_type, scale, transform, source, color) -> None:
        rr.log(path, rr.CoordinateFrame(frame), static=True)
        rr.log(
            path,
            rr.InstancePoses3D(
                translations=[np.asarray(transform[0:3], dtype=np.float64)],
                # Newton shape transforms are xyzw, matching Rerun.
                quaternions=[np.asarray(transform[3:7], dtype=np.float64)],
            ),
            static=True,
        )
        self._log_geom(path, geo_type, scale, source, color)

    def _log_geom(self, path, geo_type, scale, source, color) -> None:
        geo_type = newton.GeoType(geo_type)
        scale = np.asarray(scale, dtype=np.float64).reshape(-1)

        if geo_type in (newton.GeoType.MESH, newton.GeoType.CONVEX_MESH):
            if source is None:
                raise ValueError("mesh shape has no source geometry")
            vertices = np.asarray(source.vertices, dtype=np.float32) * scale[:3].astype(np.float32)
            faces = np.asarray(source.indices, dtype=np.int32).reshape(-1, 3)
            normals = getattr(source, "normals", None)
            normals = np.asarray(normals, dtype=np.float32) if normals is not None else None
            if normals is not None and normals.shape != vertices.shape:
                normals = None
            self._log_mesh(path, vertices, faces, normals, color)
            return

        if geo_type == newton.GeoType.HFIELD:
            if source is None:
                raise ValueError("heightfield shape has no source geometry")
            # Heightfield stores row-major (row=Y, col=X); create_heightfield wants ij (i=X).
            heights = source.min_z + source.data * (source.max_z - source.min_z)
            mesh = newton.Mesh.create_heightfield(
                heightfield=heights.T,
                extent_x=source.hx * 2.0,
                extent_y=source.hy * 2.0,
                ground_z=source.min_z,
                compute_inertia=False,
            )
            vertices = np.asarray(mesh.vertices, dtype=np.float32) * scale[:3].astype(np.float32)
            faces = np.asarray(mesh.indices, dtype=np.int32).reshape(-1, 3)
            self._log_mesh(path, vertices, faces, None, color)
            return

        if geo_type == newton.GeoType.PLANE:
            # shape_scale = (width, length, 0); non-positive means "infinite".
            vertices, faces, normals = _plane_mesh(float(scale[0]), float(scale[1]))
            self._log_mesh(path, vertices, faces, normals, color)
            return

        if geo_type == newton.GeoType.BOX:
            # shape_scale = (hx, hy, hz) half-extents.
            vertices, faces, normals = _box_mesh(float(scale[0]), float(scale[1]), float(scale[2]))
            self._log_mesh(path, vertices, faces, normals, color)
            return

        if geo_type == newton.GeoType.SPHERE:
            radius = float(scale[0])
            rr.log(
                path,
                rr.Ellipsoids3D(
                    half_sizes=[radius, radius, radius],
                    colors=color,
                    fill_mode=rr.components.FillMode.Solid,
                ),
                static=True,
            )
            return

        if geo_type == newton.GeoType.ELLIPSOID:
            rr.log(
                path,
                rr.Ellipsoids3D(
                    half_sizes=[float(scale[0]), float(scale[1]), float(scale[2])],
                    colors=color,
                    fill_mode=rr.components.FillMode.Solid,
                ),
                static=True,
            )
            return

        if geo_type == newton.GeoType.CAPSULE:
            # shape_scale = (radius, half_height, radius); Newton capsules run along Z.
            radius, half_height = float(scale[0]), float(scale[1])
            rr.log(
                path,
                rr.Capsules3D(
                    lengths=2.0 * half_height,
                    radii=radius,
                    translations=[[0.0, 0.0, -half_height]],
                    colors=color,
                    fill_mode=rr.components.FillMode.Solid,
                ),
                static=True,
            )
            return

        if geo_type in (newton.GeoType.CYLINDER, newton.GeoType.CONE):
            # Rerun has no cone archetype; a cylinder is a reasonable stand-in.
            radius, half_height = float(scale[0]), float(scale[1])
            rr.log(
                path,
                rr.Cylinders3D(
                    lengths=2.0 * half_height,
                    radii=radius,
                    centers=[[0.0, 0.0, 0.0]],
                    colors=color,
                    fill_mode=rr.components.FillMode.Solid,
                ),
                static=True,
            )
            return

        raise NotImplementedError(f"Unsupported Newton geometry type: {geo_type!r}")

    @staticmethod
    def _log_mesh(path, vertices, faces, normals, color) -> None:
        vertex_colors = np.tile(color, (len(vertices), 1))
        rr.log(
            path,
            rr.Mesh3D(
                vertex_positions=vertices,
                triangle_indices=faces,
                vertex_normals=normals,
                vertex_colors=vertex_colors,
            ),
            static=True,
        )

    # -- logging ------------------------------------------------------------

    def _log_info(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(f"[INFO] {message}")
        else:
            print(f"[INFO] {message}")

    def _log_warn(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warn(f"[WARN] {message}")
        else:
            print(f"[WARN] {message}")
