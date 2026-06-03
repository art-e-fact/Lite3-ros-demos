#!/usr/bin/env python3
"""
 * @file mujoco_simulation.py
 * @brief simulation in mujoco
 * @author Haokai Dai
 * @version 1.0
 * @date 2025-12-08
 *
 * @copyright Copyright (c) 2025  DeepRobotics
"""

import os
import time
import argparse
import math
import random
import tempfile
from pathlib import Path
from typing import Callable
from xml.sax.saxutils import quoteattr
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco
import mujoco.viewer

from sensors.mujoco.lidar_sensor import LidarSensor
from sensors.mujoco.depth_sensor import DepthSensor
from sensors.mujoco.follow_camera_recorder import FollowCameraRecorder
from sensors.mujoco.mid360_lidar_sensor import Mid360LidarSensor
from scenes.procedural_railroad_scene import build_railroad_spec
from scenes.procedural_scene_generator import build_procedural_spec
from simulation_config import SimulationConfig, resolve_path

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Pose, PoseArray
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy



MODEL_NAME = "Lite3"
# Get the directory of the current Python file
CURRENT_DIR = Path(__file__).resolve().parent


def _resolve_resource_path(*parts: str) -> str:
    return str(resolve_path(Path(*parts), must_exist=False))

# Define the default XML path relative to the Python file
XML_PATH = _resolve_resource_path("lite3_mjcf", "mjcf", "Lite3_stair.xml")

USE_VIEWER = True


def _add_ground_plane(spec: mujoco.MjSpec) -> None:
    worldbody = spec.worldbody
    floor = next((geom for geom in worldbody.geoms if geom.name == "floor"), None)
    if floor is None:
        floor = worldbody.add_geom()
    floor.name = "floor"
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    floor.size = np.array([20.0, 20.0, 0.1], dtype=np.float64)
    floor.contype = 1
    floor.conaffinity = 1
    floor.material = "checker_mat"


def _build_static_spec(scene_path: str | None, robot_xml_path: str) -> mujoco.MjSpec:
    if not scene_path:
        spec = mujoco.MjSpec.from_file(robot_xml_path)
        _add_ground_plane(spec)
        return spec

    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False, encoding="utf-8") as merged_file:
        merged_file.write(
            "<mujoco model=\"Lite3_static_scene\">\n"
            f"  <include file={quoteattr(scene_path)}/>\n"
            f"  <include file={quoteattr(robot_xml_path)}/>\n"
            "</mujoco>\n"
        )
        merged_path = merged_file.name

    try:
        return mujoco.MjSpec.from_file(merged_path)
    finally:
        try:
            os.unlink(merged_path)
        except OSError:
            pass
DT = 0.001
RENDER_INTERVAL = 50

JOINT_INIT = {
    "Lite3": np.array([0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,], dtype=np.float32),
}


SceneUpdater = Callable[[mujoco.MjModel, mujoco.MjData, float], bool]
def _as_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class MuJoCoSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str | None = None,
                 config: SimulationConfig | None = None):

        super().__init__('mujoco_simulation')

        config = config or SimulationConfig.load()
        if xml_path:
            config = config.with_overrides({"scene": xml_path})

        self.declare_parameter('scene', config.scene)
        self.declare_parameter('procedural_env_seed', config.procedural_env_seed)
        self.declare_parameter('headless', config.headless)
        self.declare_parameter('enable_lidar', config.sensors.lidar_2d.enabled)
        self.declare_parameter('enable_mid360', config.sensors.mid360.enabled)
        self.declare_parameter('enable_depth', config.sensors.realsense.enable_depth)
        self.declare_parameter('enable_color', config.sensors.realsense.enable_color)
        self.declare_parameter('enable_pointcloud', config.sensors.realsense.enable_pointcloud)
        self.declare_parameter('enable_follow_camera', config.sensors.follow_camera.enabled)
        self.declare_parameter('follow_camera_video_path', config.sensors.follow_camera.video_path)

        config = config.with_overrides({
            "simulator": "mujoco",
            "scene": str(self.get_parameter('scene').value).strip(),
            "procedural_env_seed": int(self.get_parameter('procedural_env_seed').value),
            "headless": _as_bool(self.get_parameter('headless').value, config.headless),
            "sensors.lidar_2d.enabled": _as_bool(self.get_parameter('enable_lidar').value, config.sensors.lidar_2d.enabled),
            "sensors.mid360.enabled": _as_bool(self.get_parameter('enable_mid360').value, config.sensors.mid360.enabled),
            "sensors.realsense.enable_depth": _as_bool(self.get_parameter('enable_depth').value, config.sensors.realsense.enable_depth),
            "sensors.realsense.enable_color": _as_bool(self.get_parameter('enable_color').value, config.sensors.realsense.enable_color),
            "sensors.realsense.enable_pointcloud": _as_bool(self.get_parameter('enable_pointcloud').value, config.sensors.realsense.enable_pointcloud),
            "sensors.follow_camera.enabled": _as_bool(self.get_parameter('enable_follow_camera').value, config.sensors.follow_camera.enabled),
            "sensors.follow_camera.video_path": str(self.get_parameter('follow_camera_video_path').value).strip(),
        })
        errors = config.validate()
        if errors:
            raise ValueError("Invalid simulation config: " + "; ".join(errors))

        procedural_scene = config.procedural_scene_name()
        configured_seed = config.procedural_env_seed
        headless = config.headless
        enable_lidar = config.sensors.lidar_2d.enabled
        enable_mid360 = config.sensors.mid360.enabled
        enable_depth = config.sensors.realsense.enable_depth
        enable_color = config.sensors.realsense.enable_color
        use_viewer = USE_VIEWER and (not headless)
        self.scene_meta: dict = {}
        self.scene_update: SceneUpdater | None = None
        self.procedural_waypoints_msg = None
        self.scene_start_pose = (-5.0, 0.0, 0.0)
        scene_meta: dict = {}
        robot_xml_path = config.resolved_robot_description()
        if not os.path.isfile(robot_xml_path):
            raise FileNotFoundError(f"Cannot find Lite3 robot MJCF: {robot_xml_path}")

        if procedural_scene == "blocks":
            scene_seed = configured_seed if configured_seed >= 0 else random.randint(0, 2**31 - 1)
            spec, scene_meta = build_procedural_spec(
                robot_xml_path,
                robot_start_xy=(-5.0, 0.0),
                seed=scene_seed,
            )
            self.scene_start_pose = tuple(scene_meta.get("robot_start_pose", self.scene_start_pose))
            self.get_logger().info(
                f"[INFO] Shapes scene enabled (robot=Lite3.xml, world built in Python): "
                f"seed={scene_meta['seed']}, nodes={scene_meta['nodes']}, "
                f"edges={scene_meta['edges']}, obstacles={scene_meta['obstacles']}"
            )
            self.procedural_waypoints_msg = self._build_waypoint_pose_array(scene_meta)
            if self.procedural_waypoints_msg is not None:
                self.get_logger().info(
                    f"[INFO] Publishing mission with {len(self.procedural_waypoints_msg.poses)} ordered waypoints on /procedural_waypoints"
                )
        elif procedural_scene == "railroad":
            scene_seed = configured_seed if configured_seed >= 0 else random.randint(0, 2**31 - 1)
            spec, scene_meta = build_railroad_spec(
                robot_xml_path,
                seed=scene_seed,
            )
            self.scene_start_pose = tuple(scene_meta.get("robot_start_pose", self.scene_start_pose))
            self.get_logger().info(
                f"[INFO] Railroad scene enabled (robot=Lite3.xml, world built in Python): "
                f"seed={scene_meta['seed']}, roads={scene_meta['roads']}, "
                f"mainline_waypoints={len(scene_meta.get('mission_xy', []))}"
            )
            self.procedural_waypoints_msg = self._build_waypoint_pose_array(scene_meta)
            if self.procedural_waypoints_msg is not None:
                self.get_logger().info(
                    f"[INFO] Publishing mission with {len(self.procedural_waypoints_msg.poses)} ordered waypoints on /procedural_waypoints"
                )
        else:
            scene_path = config.resolved_scene()
            spec = _build_static_spec(scene_path, robot_xml_path)
            if scene_path:
                self.get_logger().info(f"[INFO] Using static environment scene: {scene_path}")
            else:
                self.get_logger().info("[INFO] No static scene configured; using the robot model with a generated floor")

        self._set_scene_meta(scene_meta)

        # Sensor-driven MjSpec mutations (must happen before compile)
        if enable_depth or enable_color:
            DepthSensor.init_visuals(spec, config.sensors.realsense)
            self.get_logger().info("[INFO] D435i model attached via mjSpec")
        if enable_lidar:
            LidarSensor.init_visuals(spec, config.sensors.lidar_2d)
        if enable_mid360:
            Mid360LidarSensor.init_visuals(spec, config.sensors.mid360)
            self.get_logger().info("[INFO] Mid360 model attached via mjSpec")

        self.model = spec.compile()
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)
        self.timestamp = 0.0

        # 机器人自由度列表
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..11
        self.dof_num = len(self.actuator_ids)
        assert self.dof_num == 12, "Expected 12 DOF for Lite3"

        # 初始化站立姿态
        self._set_initial_pose(model_key, self.scene_start_pose)
        self._update_scene(self.timestamp)

        # 缓存
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)

        self.get_logger().info(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # ROS Publishers
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        waypoint_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.waypoint_pub = self.create_publisher(PoseArray, '/procedural_waypoints', waypoint_qos)

        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # ROS Subscriber
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            '/JOINTS_CMD',
            self._cmd_callback,
            50
        )

        # 可视化
        self.viewer = None
        if use_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            # Point viewer camera at the robot's initial position
            self.viewer.cam.lookat[:] = [self.scene_start_pose[0], self.scene_start_pose[1], 0.3]
            self.viewer.cam.distance = 3.0
            self.viewer.cam.azimuth = 90.0
            self.viewer.cam.elevation = -20.0
        else:
            self.get_logger().info("[INFO] Running MuJoCo in headless mode (viewer disabled)")

        # LiDAR sensor
        self.lidar = LidarSensor(self.model, self.data, self, self.viewer, config=config.sensors.lidar_2d)
        self.lidar_step_interval = max(1, int(1.0 / (config.sensors.lidar_2d.frequency_hz * DT)))

        self.mid360 = Mid360LidarSensor(self.model, self.data, self, self.viewer, config=config.sensors.mid360)
        self.mid360_step_interval = max(1, int(1.0 / (config.sensors.mid360.frequency_hz * DT)))

        # Depth camera sensor (RealSense D435i)
        self.depth = DepthSensor(
            self.model, self.data, self, self.viewer,
            config=config.sensors.realsense,
        )
        self.depth_step_interval = max(1, int(1.0 / (config.sensors.realsense.frequency_hz * DT)))

        self.follow_camera = FollowCameraRecorder(
            self.model,
            self.data,
            self,
            config=config.sensors.follow_camera,
        )
        self.follow_camera_step_interval = max(1, int(round(1.0 / (config.sensors.follow_camera.fps * DT))))

        self._publish_clock(self._make_sim_stamp())
        # Publish all static transforms in one call
        self._publish_static_transforms()
        self._publish_procedural_waypoints()
        self.waypoint_timer = self.create_timer(1.0, self._publish_procedural_waypoints)

    def _set_initial_pose(self, key: str, base_pose: tuple[float, float, float]):
        """关节位置设置为与 PyBullet 脚本一致的初始角度"""
        x, y, yaw = base_pose
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]  # ,3-6 basequat，0-2 basepos
        qpos0[:3] = np.array([x, y, 0.43])
        qpos0[3:7] = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _set_scene_meta(self, scene_meta: dict | None):
        self.scene_meta = scene_meta or {}
        update_scene = self.scene_meta.get("update_scene")
        if update_scene is None:
            self.scene_update = None
            return
        if not callable(update_scene):
            raise TypeError("scene_meta['update_scene'] must be callable")
        self.scene_update = update_scene

    def _update_scene(self, sim_time_s: float) -> bool:
        if self.scene_update is None:
            return False
        changed = bool(self.scene_update(self.model, self.data, sim_time_s))
        if changed:
            mujoco.mj_forward(self.model, self.data)
        return changed

    def _publish_static_transforms(self):
        """Collect static TFs from sensors and publish in one call."""
        stamp = self._make_sim_stamp()
        transforms = []
        transforms.extend(self.lidar.get_static_transforms(stamp))
        transforms.extend(self.mid360.get_static_transforms(stamp))
        transforms.extend(self.depth.get_static_transforms(stamp))
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    @staticmethod
    def _stamp_from_seconds(timestamp: float) -> Time:
        stamp = Time()
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def _make_sim_stamp(self, timestamp: float | None = None) -> Time:
        if timestamp is None:
            timestamp = self.timestamp
        return self._stamp_from_seconds(timestamp)

    def _publish_clock(self, stamp: Time):
        clock_msg = Clock()
        clock_msg.clock = stamp
        self.clock_pub.publish(clock_msg)

    def _build_waypoint_pose_array(self, scene_meta: dict) -> PoseArray | None:
        mission_xy = scene_meta.get("mission_xy", [])
        if not mission_xy:
            return None

        msg = PoseArray()
        msg.header.frame_id = "odom"
        msg.poses = []
        for xy in mission_xy:
            if len(xy) != 2:
                continue
            pose = Pose()
            pose.position.x = float(xy[0])
            pose.position.y = float(xy[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg if msg.poses else None

    def _publish_procedural_waypoints(self):
        if self.procedural_waypoints_msg is None:
            return
        self.procedural_waypoints_msg.header.stamp = self._make_sim_stamp()
        self.waypoint_pub.publish(self.procedural_waypoints_msg)

    def _publish_odom_and_tf(self, stamp: Time, publish_odom: bool = True):
        pos = self.data.qpos[0:3]
        # MuJoCo quaternion is [w, x, y, z]
        quat_wxyz = self.data.qpos[3:7]
        linvel = self.data.qvel[0:3]
        angvel = self.data.qvel[3:6]

        # Rotate world-frame velocities into body frame
        # (nav_msgs/Odometry twist must be in child_frame_id = base_link)
        R = Rotation.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        body_linvel = R.inv().apply(linvel)
        body_angvel = R.inv().apply(angvel)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation = Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        t.transform.rotation = Quaternion(
            x=float(quat_wxyz[1]), y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]), w=float(quat_wxyz[0])
        )
        self.tf_broadcaster.sendTransform(t)

        if not publish_odom:
            return

        # Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        # odom_msg.child_frame_id = 'base_footprint' #'base_link'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])
        odom_msg.pose.pose.orientation.x = float(quat_wxyz[1])
        odom_msg.pose.pose.orientation.y = float(quat_wxyz[2])
        odom_msg.pose.pose.orientation.z = float(quat_wxyz[3])
        odom_msg.pose.pose.orientation.w = float(quat_wxyz[0])
        odom_msg.twist.twist.linear.x = float(body_linvel[0])
        odom_msg.twist.twist.linear.y = float(body_linvel[1])
        odom_msg.twist.twist.linear.z = float(body_linvel[2])
        odom_msg.twist.twist.angular.x = float(body_angvel[0])
        odom_msg.twist.twist.angular.y = float(body_angvel[1])
        odom_msg.twist.twist.angular.z = float(body_angvel[2])
        self.odom_pub.publish(odom_msg)

    def _cmd_callback(self, msg: JointsDataCmd):
        """Convert received (published) positions/velocities to internal (raw)"""
        if len(msg.data.joints_data) not in (12, 16):
            self.get_logger().warn("Received JointsDataCmd with incorrect number of joints")
            return

        pub_pos = np.zeros(self.dof_num, dtype=np.float32)
        pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        # 兼容长度为16的命令，仅读取前12个关节
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp
            self.kd_cmd[i] = joint_cmd.kd
            # pub_pos[i] = joint_cmd.position
            # pub_vel[i] = joint_cmd.velocity
            self.pos_cmd[i] = joint_cmd.position
            self.vel_cmd[i] = joint_cmd.velocity
            self.tau_ff[i] = joint_cmd.torque  # tau_ff no processing

    def start(self):
        # 主模拟循环
        step = 0
        last_time = time.time()
        try:
            while rclpy.ok():
                if time.time() - last_time >= DT:
                    last_time = time.time()
                    step += 1
                    # 控制律
                    self._apply_joint_torque()
                    # 模拟一步
                    mujoco.mj_step(self.model, self.data)

                    self.timestamp = step * DT
                    self._update_scene(self.timestamp)
                    stamp = self._make_sim_stamp(self.timestamp)
                    self._publish_clock(stamp)

                    # Keep TF current for exact sensor timestamp lookups.
                    self._publish_odom_and_tf(stamp, publish_odom=False)

                    # 采样 & 发送观测
                    self._publish_robot_state(stamp)
                    self._publish_odom_and_tf(stamp)

                    # LiDAR scan
                    if step % self.lidar_step_interval == 0:
                        self.lidar.update(stamp)

                    if step % self.mid360_step_interval == 0:
                        self.mid360.update(stamp)

                    # Depth camera
                    if step % self.depth_step_interval == 0:
                        self.depth.update(stamp)

                    if self.follow_camera.enabled and step % self.follow_camera_step_interval == 0:
                        self.follow_camera.update()

                    # Viewer
                    if self.viewer and step % RENDER_INTERVAL == 0:
                        self.lidar.visualize()
                        self.viewer.sync()

                # Handle ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            self.follow_camera.close()

    def _apply_joint_torque(self):
        # 当前关节状态
        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
        self.input_tq = (
                self.kp_cmd * (self.pos_cmd - q) +
                self.kd_cmd * (self.vel_cmd - dq) +
                self.tau_ff
        )

        # 写入 control 缓冲区
        self.data.ctrl[:] = self.input_tq.flatten()

    # --------------------------------------------------------
    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q

        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # 防止数值漂移导致 |t2|>1
        pitch = np.arcsin(t2)

        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    # --------------------------------------------------------

    def _publish_robot_state(self, stamp: Time):
        # ----- IMU -----
        # q_world = self.data.sensordata[:4]  # quaternion
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        # ImuDataValue expects degrees for roll/pitch/yaw
        rpy_deg = np.rad2deg(rpy)
        # body_acc = self.data.sensordata[4:7]
        body_acc = self.data.sensordata[16:19]
        angvel_b = self.data.qvel[3:6]  # body frame

        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(angvel_b[0])
        imu_msg.data.omega_y = float(angvel_b[1])
        imu_msg.data.omega_z = float(angvel_b[2])
        imu_msg.data.acc_x = float(body_acc[0])
        imu_msg.data.acc_y = float(body_acc[1])
        imu_msg.data.acc_z = float(body_acc[2])
        self.imu_pub.publish(imu_msg)

        # ----- 关节 -----
        q = self.data.qpos[7:7 + self.dof_num]
        dq = self.data.qvel[6:6 + self.dof_num]
        tau = self.input_tq.flatten()

        pub_pos = q
        pub_vel = dq
        pub_tau = tau
        
        joints_msg = JointsData()
        joints_msg.header = MetaType()
        joints_msg.header.frame_id = 0
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue()
        # drdds 消息要求长度 16，这里补足 4 个占位
        joints_msg.data.joints_data = [JointData() for _ in range(16)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]
            joint.name = [32, 32, 32, 32]  # Dummy name (four spaces)
            # joint.name = "    "
            joint.data_id = 0  # Dummy
            joint.status_word = 1  # Normal
            joint.position = float(pub_pos[i])
            joint.torque = float(pub_tau[i])
            joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0  # Dummy normal temp
            joint.driver_temp = 45.0  # Dummy normal temp
        # 余下的 4 个占位保持默认（0），status_word 设为正常
        for i in range(self.dof_num, 16):
            joint = joints_msg.data.joints_data[i]
            joint.status_word = 1
        self.joints_pub.publish(joints_msg)


def run_mujoco(config: SimulationConfig, ros_args: list[str] | None = None):
    rclpy.init(args=ros_args)
    sim_node = MuJoCoSimulationNode(config=config)
    try:
        sim_node.start()
    finally:
        sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    np.set_printoptions(precision=4, suppress=True)
    parser = argparse.ArgumentParser(description="Run Lite3 MuJoCo ROS2 simulation")
    parser.add_argument("--config", default=None, help="Path to simulation YAML config")
    args, ros_args = parser.parse_known_args()

    config = SimulationConfig.load(args.config).with_overrides({"simulator": "mujoco"})
    errors = config.validate()
    if errors:
        raise SystemExit("Invalid simulation config:\n- " + "\n- ".join(errors))
    run_mujoco(config, ros_args)

if __name__ == "__main__":
    main()
