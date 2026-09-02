"""Orchestrates Newton sensors at their own publish rates."""

from dataclasses import dataclass, field

from tf2_ros import StaticTransformBroadcaster

from simulation_config import Lidar2DConfig, Mid360Config, RealsenseConfig, RobosenseConfig
from sensors.newton.bvh import NewtonBvh
from sensors.newton.depth_sensor import NewtonDepthSensor
from sensors.newton.lidar_sensor import NewtonLidarSensor
from sensors.newton.mid360_lidar_sensor import NewtonMid360LidarSensor
from sensors.newton.robosense_lidar_sensor import NewtonRobosenseLidarSuite


@dataclass
class NewtonSensorOptions:
    lidar_2d: Lidar2DConfig = field(default_factory=Lidar2DConfig)
    mid360: Mid360Config = field(default_factory=Mid360Config)
    realsense: RealsenseConfig = field(default_factory=RealsenseConfig)
    robosense: RobosenseConfig = field(default_factory=RobosenseConfig)

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


class NewtonSensorManager:
    def __init__(self, model, state, node, dt: float, options: NewtonSensorOptions):
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

        self.lidar_step_interval = max(1, int(1.0 / (options.lidar_2d.frequency_hz * dt)))
        self.mid360_step_interval = max(1, int(1.0 / (options.mid360.frequency_hz * dt)))
        self.depth_step_interval = max(1, int(1.0 / (options.realsense.frequency_hz * dt)))
        self.bvh = NewtonBvh(model, state) if self.enabled else None
        self._publish_static_transforms()

    @property
    def enabled(self) -> bool:
        return self.lidar.enabled or self.mid360.enabled or self.depth.enabled or self.robosense.enabled

    def update(self, state, step_count: int, timestamp: float):
        if not self.enabled:
            return

        due_lidar = self.lidar.enabled and step_count % self.lidar_step_interval == 0
        due_mid360 = self.mid360.enabled and step_count % self.mid360_step_interval == 0
        due_depth = self.depth.enabled and step_count % self.depth_step_interval == 0
        due_robosense = self.robosense.any_due(step_count)
        if not (due_lidar or due_mid360 or due_depth or due_robosense):
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

    def _publish_static_transforms(self):
        stamp = self.node.get_clock().now().to_msg()
        transforms = []
        transforms.extend(self.lidar.get_static_transforms(stamp))
        transforms.extend(self.mid360.get_static_transforms(stamp))
        transforms.extend(self.depth.get_static_transforms(stamp))
        transforms.extend(self.robosense.get_static_transforms(stamp))
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)
