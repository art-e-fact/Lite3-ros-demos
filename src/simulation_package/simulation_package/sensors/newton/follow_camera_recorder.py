"""Offscreen third-person follow camera video recorder for the Newton backend.

Mirrors ``sensors/mujoco/follow_camera_recorder.FollowCameraRecorder``: an
offscreen RGB camera that trails the robot base and writes an mp4.

Frames come from Newton's GL viewer via ``ViewerGL.get_frame`` so the video looks
like the Newton screen, headless or not (like MuJoCo's EGL recorder):

* ``headless: false``: the interactive window, at its own resolution; its camera
  is driven to the follow pose while recording.
* ``headless: true``: a private EGL-backed ``ViewerGL`` at ``width`` x ``height``
  that needs no display server.  The physics loop never sees it, so the CUDA
  graph stays enabled.

If no GL context can be created at all, it falls back to ``SensorTiledCamera``
(pure Warp raytracer, works on the CPU device too, but slow there).

Camera placement replicates MuJoCo's free-camera semantics so both backends
frame the robot the same way:

    lookat  = smoothed(robot position) + target_height_m
    azimuth = robot yaw + azimuth_offset_deg
    forward = (cos(el)cos(az), cos(el)sin(az), sin(el))
    position = lookat - distance_m * forward

``SensorTiledCamera`` uses the OpenGL camera convention (+X right, +Y up,
-Z forward), which is also what ``sensors/common/transforms`` encodes.
"""

from __future__ import annotations

import math
from pathlib import Path

import imageio.v2 as imageio
from newton._src.sensors.sensor_tiled_camera import SensorTiledCamera
import numpy as np
import warp as wp

from sensors.newton.geometry import camera_transforms
from simulation_config import FollowCameraConfig

WORLD_UP = np.array([0.0, 0.0, 1.0], dtype=np.float64)


def _yaw_from_quat_xyzw(quat_xyzw) -> float:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _look_rotation(forward: np.ndarray) -> np.ndarray:
    """Camera rotation matrix (columns = camera X/Y/Z in world) looking along *forward*."""
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, WORLD_UP)
    norm = np.linalg.norm(right)
    if norm < 1e-6:  # looking straight up/down: pick an arbitrary stable right vector
        right = np.array([0.0, -1.0, 0.0], dtype=np.float64)
    else:
        right = right / norm
    z_axis = -forward
    up = np.cross(z_axis, right)
    return np.column_stack([right, up, z_axis])


class NewtonFollowCameraRecorder:
    """Render a follow camera into an mp4 while the Newton simulation runs.

    Args:
        model: finalized ``newton.Model``.
        node_or_logger: ROS node (``.get_logger()``), a bare logger, or ``None``.
        config: ``simulation_config.FollowCameraConfig``.
        robot_body_index: index into ``state.body_q`` of the robot base body.
        bvh: shared ``sensors.newton.bvh.NewtonBvh`` (or ``None`` to own one).
        viewer: open ``newton.viewer.ViewerGL`` to grab frames from, or ``None``
            to raytrace offscreen.
    """

    def __init__(
        self,
        model,
        node_or_logger,
        config: FollowCameraConfig | None = None,
        robot_body_index: int = 0,
        bvh=None,
        viewer=None,
    ):
        self.model = model
        self.logger = _resolve_logger(node_or_logger)
        self.config = config or FollowCameraConfig()
        self.robot_body_index = int(robot_body_index)
        self.bvh = bvh
        self.viewer = viewer if hasattr(viewer, "get_frame") else None
        self._owns_viewer = False
        self.enabled = bool(self.config.enabled)
        self.video_path = str(self.config.video_path).strip()
        self.fps = float(self.config.fps)
        # FollowCameraConfig.fov_deg defaults to MuJoCo's free-camera fovy (45 deg) so
        # both backends frame the robot the same way.
        self.fov_deg = float(self.config.fov_deg)
        self.closed = True
        self.frame_count = 0
        self._lookat = None

        if not self.enabled:
            self._log_info("Newton follow camera recorder disabled")
            self.sensor = None
            self.writer = None
            return

        if not self.video_path:
            raise ValueError(
                "follow_camera_video_path must be set when enable_follow_camera is true"
            )

        output_path = Path(self.video_path).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.video_path = str(output_path)

        width = int(self.config.width)
        height = int(self.config.height)
        self.sensor = None
        if self.viewer is None:
            self.viewer = _create_headless_viewer(model, width, height, self.logger)
            self._owns_viewer = self.viewer is not None
        if self.viewer is not None:
            # The mp4 takes the viewer's window size; width/height size the raytracer.
            width, height = self.viewer.renderer.window.get_framebuffer_size()
            width, height = width - width % 8, height - height % 8  # see render()
            source = "Newton viewer (EGL)" if self._owns_viewer else "Newton viewer"
        else:
            self.sensor = SensorTiledCamera(model, load_textures=False)
            self.sensor.default_render_config.enable_shadows = True
            # Textures are a CPU-device liability and the scene has none worth showing.
            self.sensor.default_render_config.enable_textures = False
            self.sensor.utils.create_default_light(enable_shadows=True)
            self.rays = self.sensor.utils.compute_camera_rays_pinhole(
                width, height, camera_fovs=math.radians(self.fov_deg)
            )
            self.color_image = self.sensor.utils.create_color_image_output(
                width, height, camera_count=1
            )
            source = "Warp raytracer"

        self.writer = imageio.get_writer(
            self.video_path,
            fps=self.fps,
            codec="libx264",
            quality=self.config.quality,
            macro_block_size=8,  # 800x600 is not a multiple of 16
        )
        self.closed = False
        self._log_info(
            f"Newton follow camera recording enabled: {self.video_path} "
            f"({width}x{height} @ {self.fps:.1f} Hz, fov {self.fov_deg:.0f} deg, {source})"
        )

    def render(self, state, refit: bool = True, timestamp: float = 0.0) -> np.ndarray | None:
        """Render one RGB frame (H, W, 3) uint8 without writing it to the video."""
        if not self.enabled or self.closed:
            return None
        position, forward, azimuth, elevation = self._camera_pose(state)

        if self.viewer is not None:
            # Drive the interactive camera to the follow pose and grab the GL frame.
            self.viewer.set_camera(
                wp.vec3(*position), math.degrees(elevation), math.degrees(azimuth)
            )
            self.viewer.begin_frame(timestamp)
            self.viewer.log_state(state)
            self.viewer.end_frame()
            frame = self.viewer.get_frame().numpy()
            # Crop to the writer's macro block so odd window sizes are not resampled.
            h, w = frame.shape[:2]
            return np.ascontiguousarray(frame[: h - h % 8, : w - w % 8])

        if refit and self.bvh is not None:
            self.bvh.refit(state)
        elif refit:
            self.model.bvh_refit_shapes(state)

        transforms = camera_transforms(position, _look_rotation(forward), self.model.world_count)
        self.sensor.update(
            state,
            transforms,
            self.rays,
            color_image=self.color_image,
            clear_data=SensorTiledCamera.GRAY_CLEAR_DATA,
        )
        rgba = self.sensor.utils.to_rgba_from_color(self.color_image).numpy()
        return np.ascontiguousarray(rgba[0, :, :, :3])

    def update(self, state, refit: bool = True, timestamp: float = 0.0) -> None:
        """Render one frame and append it to the video."""
        frame = self.render(state, refit=refit, timestamp=timestamp)
        if frame is None:
            return
        self.writer.append_data(frame)
        self.frame_count += 1

    def reset(self) -> None:
        """Forget the smoothed look-at point (used after a warm-up rewind)."""
        self._lookat = None

    def close(self) -> None:
        if not self.enabled or self.closed:
            return
        self.closed = True
        self.writer.close()
        if self._owns_viewer:
            self.viewer.close()
        self._log_info(
            f"Newton follow camera video saved: {self.video_path} ({self.frame_count} frames)"
        )

    # -- camera placement ---------------------------------------------------

    def _camera_pose(self, state) -> tuple[np.ndarray, np.ndarray, float, float]:
        """Camera position, forward unit vector, azimuth and elevation (radians)."""
        body_q = state.body_q.numpy()[self.robot_body_index]
        target = np.asarray(body_q[0:3], dtype=np.float64).copy()
        target[2] += float(self.config.target_height_m)
        if self._lookat is None:
            self._lookat = target
        else:
            alpha = float(np.clip(self.config.smoothing, 0.0, 1.0))
            self._lookat = (1.0 - alpha) * self._lookat + alpha * target

        azimuth = _yaw_from_quat_xyzw(body_q[3:7]) + math.radians(self.config.azimuth_offset_deg)
        elevation = math.radians(self.config.elevation_deg)
        forward = np.array(
            [
                math.cos(elevation) * math.cos(azimuth),
                math.cos(elevation) * math.sin(azimuth),
                math.sin(elevation),
            ],
            dtype=np.float64,
        )
        position = self._lookat - float(self.config.distance_m) * forward
        return position, forward, azimuth, elevation

    # -- logging ------------------------------------------------------------

    def _log_info(self, message: str) -> None:
        if self.logger is not None:
            self.logger.info(f"[INFO] {message}")
        else:
            print(f"[INFO] {message}")


def _create_headless_viewer(model, width: int, height: int, logger):
    """Invisible EGL-backed ``ViewerGL`` (no display server needed), or ``None``."""
    try:
        import pyglet

        pyglet.options["headless"] = True
        import newton.viewer

        viewer = newton.viewer.ViewerGL(width=width, height=height, headless=True)
        viewer.set_model(model)
        return viewer
    except Exception as exc:  # no EGL/GL at all: fall back to the raytracer
        message = f"[WARN] No headless GL viewer for the follow camera, raytracing instead: {exc}"
        if logger is not None:
            logger.warn(message)
        else:
            print(message)
        return None


def _resolve_logger(node_or_logger):
    if node_or_logger is None:
        return None
    if hasattr(node_or_logger, "get_logger"):
        return node_or_logger.get_logger()
    return node_or_logger
