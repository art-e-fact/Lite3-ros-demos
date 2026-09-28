"""Orchestrates Newton sensors at their own publish rates."""

from dataclasses import dataclass, field

from tf2_ros import StaticTransformBroadcaster

from sensors.common.transforms import sim_time_stamp
from simulation_config import FollowCameraConfig, Lidar2DConfig, Mid360Config, RealsenseConfig, RobosenseConfig
from sensors.newton.bvh import NewtonBvh
from sensors.newton.depth_sensor import NewtonDepthSensor
from sensors.newton.follow_camera_recorder import NewtonFollowCameraRecorder
from sensors.newton.lidar_sensor import NewtonLidarSensor
from sensors.newton.mid360_lidar_sensor import NewtonMid360LidarSensor
from sensors.newton.robosense_lidar_sensor import NewtonRobosenseLidarSuite


@dataclass
class NewtonSensorOptions:
    lidar_2d: Lidar2DConfig = field(default_factory=Lidar2DConfig)
    mid360: Mid360Config = field(default_factory=Mid360Config)
    realsense: RealsenseConfig = field(default_factory=RealsenseConfig)
    robosense: RobosenseConfig = field(default_factory=RobosenseConfig)
    follow_camera: FollowCameraConfig = field(default_factory=FollowCameraConfig)

    @property
    def enable_lidar(self) -> bool:
        return self.lidar_2d.enabled

    @property
    def enable_mid360(self) -> bool:
        return self.mid360.enabled

    @property
    def enable_robosense(self) -> bool:
        return self.robosense.enabled

    @property
    def enable_depth(self) -> bool:
        return self.realsense.enable_depth

    @property
    def enable_color(self) -> bool:
        return self.realsense.enable_color

    @property
    def enable_follow_camera(self) -> bool:
        return self.follow_camera.enabled


class NewtonSensorManager:
    """Runs every Newton ray sensor plus the follow camera at their own rates.

    All of them raytrace against the same shape BVH, so the manager refits it once
    per step for whichever sensors are due and then renders them all from that fit.
    """

    def __init__(
        self,
        model,
        state,
        node,
        dt: float,
        options: NewtonSensorOptions,
        robot_body_index: int = 0,
        viewer=None,
    ):
        self.model = model
        self.node = node
        self.static_tf_broadcaster = StaticTransformBroadcaster(node)

        self.lidar = NewtonLidarSensor(model, node, config=options.lidar_2d)
        self.mid360 = NewtonMid360LidarSensor(
            model,
            node,
            config=options.mid360,
        )
        self.depth = NewtonDepthSensor(
            model,
            node,
            config=options.realsense,
        )
        self.robosense = NewtonRobosenseLidarSuite(model, node, dt, config=options.robosense)
        # The follow camera raytraces the same BVH as the lidars, so it belongs in the
        # same due-list rather than owning a second refit.
        self.follow_camera = NewtonFollowCameraRecorder(
            model,
            node,
            config=options.follow_camera,
            robot_body_index=robot_body_index,
            viewer=viewer,
            bvh=None,  # assigned below once the shared BVH exists
        )

        self.lidar_step_interval = max(1, int(1.0 / (options.lidar_2d.frequency_hz * dt)))
        self.mid360_step_interval = max(1, int(1.0 / (options.mid360.frequency_hz * dt)))
        self.depth_step_interval = max(1, int(1.0 / (options.realsense.frequency_hz * dt)))
        self.follow_camera_step_interval = max(1, int(round(1.0 / (options.follow_camera.fps * dt))))
        self.bvh = NewtonBvh(model, state) if self.enabled else None
        self.follow_camera.bvh = self.bvh
        self._publish_static_transforms()

    @property
    def enabled(self) -> bool:
        return (
            self.lidar.enabled
            or self.mid360.enabled
            or self.depth.enabled
            or self.robosense.enabled
            or self.follow_camera.enabled
        )

    def update(self, state, step_count: int, timestamp: float):
        if not self.enabled:
            return

        due_lidar = self.lidar.enabled and step_count % self.lidar_step_interval == 0
        due_mid360 = self.mid360.enabled and step_count % self.mid360_step_interval == 0
        due_depth = self.depth.enabled and step_count % self.depth_step_interval == 0
        due_robosense = self.robosense.any_due(step_count)
        due_camera = self.follow_camera.enabled and step_count % self.follow_camera_step_interval == 0
        if not (due_lidar or due_mid360 or due_depth or due_robosense or due_camera):
            return

        # Refit once for all due sensors; sensor rendering then sees the same world pose.
        self.bvh.refit(state)
        if due_lidar:
            self.lidar.update(state, timestamp)
        if due_mid360:
            self.mid360.update(state, timestamp)
        if due_depth:
            self.depth.update(state, timestamp)
        if due_robosense:
            self.robosense.update(state, step_count, timestamp)
        if due_camera:
            self.follow_camera.update(state, refit=False, timestamp=timestamp)

    def warmup(self, state, timestamp: float = 0.0):
        """Render every enabled sensor once so their Warp kernels are compiled.

        Publishes one message per sensor topic; the follow-camera frame is rendered but
        deliberately not written to the video.
        """
        if not self.enabled:
            return

        self.bvh.refit(state)
        if self.lidar.enabled:
            self.lidar.update(state, timestamp)
        if self.mid360.enabled:
            self.mid360.update(state, timestamp)
        if self.depth.enabled:
            self.depth.update(state, timestamp)
        if self.robosense.enabled:
            self.robosense.warmup(state, timestamp)
        if self.follow_camera.enabled:
            self.follow_camera.render(state, refit=False, timestamp=timestamp)

    def close(self):
        self.follow_camera.close()

    def _publish_static_transforms(self):
        # Sim time, like every other message the simulator publishes: the nav stack runs
        # with use_sim_time and cannot match a wall-clock TF against sim-clock data.
        stamp = sim_time_stamp(0.0)
        transforms = []
        transforms.extend(self.lidar.get_static_transforms(stamp))
        transforms.extend(self.mid360.get_static_transforms(stamp))
        transforms.extend(self.depth.get_static_transforms(stamp))
        transforms.extend(self.robosense.get_static_transforms(stamp))
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)
