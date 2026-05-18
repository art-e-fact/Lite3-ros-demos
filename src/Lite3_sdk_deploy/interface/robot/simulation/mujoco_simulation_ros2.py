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
import socket
import struct
import threading
import argparse
import random
from pathlib import Path
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco
import mujoco.viewer

from lidar_sensor import LidarSensor, LIDAR_FREQUENCY_HZ
from depth_sensor import DepthSensor, DEPTH_FREQUENCY_HZ
from procedural_scene_generator import build_procedural_spec

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Pose, PoseArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy



MODEL_NAME = "Lite3"
# Get the directory of the current Python file
CURRENT_DIR = Path(__file__).resolve().parent

# Define the default XML path relative to the Python file
XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "Lite3_description" / "lite3_mjcf" / "mjcf" / "Lite3_stair.xml"

# Convert to absolute path as string
XML_PATH = str(XML_PATH.resolve())

# Robot-only MJCF used when the full scene is generated procedurally in Python.
LITE3_ROBOT_XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "Lite3_description" / "lite3_mjcf" / "mjcf" / "Lite3.xml"
LITE3_ROBOT_XML_PATH = str(LITE3_ROBOT_XML_PATH.resolve())

D435I_XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "Lite3_description" / "lite3_mjcf" / "realsense_d435i" / "d435i.xml"
D435I_XML_PATH = str(D435I_XML_PATH.resolve())


USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 50

JOINT_INIT = {
    "Lite3": np.array([0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,], dtype=np.float32),
}


class MuJoCoSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = XML_PATH,
                 d435i_xml_path: str = D435I_XML_PATH):

        super().__init__('mujoco_simulation')

        self.declare_parameter('use_procedural_scene', False)
        self.declare_parameter('procedural_env_seed', -1)
        self.declare_parameter('headless', False)
        self.declare_parameter('enable_lidar', False)
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('enable_color', False)
        self.declare_parameter('enable_pointcloud', False)
        use_procedural_scene = bool(self.get_parameter('use_procedural_scene').value)
        configured_seed = int(self.get_parameter('procedural_env_seed').value)
        headless = bool(self.get_parameter('headless').value)
        enable_lidar = bool(self.get_parameter('enable_lidar').value)
        enable_depth = bool(self.get_parameter('enable_depth').value)
        enable_color = bool(self.get_parameter('enable_color').value)
        enable_pointcloud = bool(self.get_parameter('enable_pointcloud').value)
        use_viewer = USE_VIEWER and (not headless)
        self.procedural_waypoints_msg = None

        if use_procedural_scene:
            if not os.path.isfile(LITE3_ROBOT_XML_PATH):
                raise FileNotFoundError(f"Cannot find Lite3 robot MJCF: {LITE3_ROBOT_XML_PATH}")
            scene_seed = configured_seed if configured_seed >= 0 else random.randint(0, 2**31 - 1)
            spec, scene_meta = build_procedural_spec(
                LITE3_ROBOT_XML_PATH,
                robot_start_xy=(-5.0, 0.0),
                seed=scene_seed,
            )
            self.get_logger().info(
                f"[INFO] Procedural scene enabled (robot=Lite3.xml, world built in Python): "
                f"seed={scene_meta['seed']}, nodes={scene_meta['nodes']}, "
                f"edges={scene_meta['edges']}, obstacles={scene_meta['obstacles']}"
            )
            self.procedural_waypoints_msg = self._build_waypoint_pose_array(scene_meta)
            if self.procedural_waypoints_msg is not None:
                self.get_logger().info(
                    f"[INFO] Publishing mission with {len(self.procedural_waypoints_msg.poses)} ordered waypoints on /procedural_waypoints"
                )
        else:
            if not os.path.isfile(xml_path):
                raise FileNotFoundError(f"Cannot find MJCF: {xml_path}")
            spec = mujoco.MjSpec.from_file(xml_path)
            self.get_logger().info("[INFO] Using default static scene")

        # Sensor-driven MjSpec mutations (must happen before compile)
        if enable_depth or enable_color:
            if not os.path.isfile(d435i_xml_path):
                raise FileNotFoundError(f"D435i XML not found: {d435i_xml_path}")
            DepthSensor.configure_spec(spec, d435i_xml_path)
            self.get_logger().info("[INFO] D435i model attached via mjSpec")
        if enable_lidar:
            LidarSensor.configure_spec(spec)

        self.model = spec.compile()
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)

        # 机器人自由度列表
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..11
        self.dof_num = len(self.actuator_ids)
        assert self.dof_num == 12, "Expected 12 DOF for Lite3"

        # 初始化站立姿态
        self._set_initial_pose(model_key)

        # 缓存
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0

        self.get_logger().info(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # ROS Publishers
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
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
            self.viewer.cam.lookat[:] = [-5.0, 0.0, 0.3]
            self.viewer.cam.distance = 3.0
            self.viewer.cam.azimuth = 90.0
            self.viewer.cam.elevation = -20.0
        else:
            self.get_logger().info("[INFO] Running MuJoCo in headless mode (viewer disabled)")

        # LiDAR sensor
        self.lidar = LidarSensor(self.model, self.data, self, self.viewer, enabled=enable_lidar)
        self.lidar_step_interval = int(1.0 / (LIDAR_FREQUENCY_HZ * DT))

        # Depth camera sensor (RealSense D435i)
        self.depth = DepthSensor(
            self.model, self.data, self, self.viewer,
            enable_depth=enable_depth,
            enable_color=enable_color,
            enable_pointcloud=enable_pointcloud,
        )
        self.depth_step_interval = int(1.0 / (DEPTH_FREQUENCY_HZ * DT))

        # Publish all static transforms in one call
        self._publish_static_transforms()
        self._publish_procedural_waypoints()
        self.waypoint_timer = self.create_timer(1.0, self._publish_procedural_waypoints)

    def _set_initial_pose(self, key: str):
        """关节位置设置为与 PyBullet 脚本一致的初始角度"""
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]  # ,3-6 basequat，0-2 basepos
        qpos0[:3] = np.array([-5, 0, 0.43])
        qpos0[3:7] = np.array([1, 0, 0, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _publish_static_transforms(self):
        """Collect static TFs from sensors and publish in one call."""
        stamp = self._make_sim_stamp()
        transforms = []
        transforms.extend(self.lidar.get_static_transforms(stamp))
        transforms.extend(self.depth.get_static_transforms(stamp))
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    def _make_sim_stamp(self):
        return self.get_clock().now().to_msg()

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

    def _publish_odom_and_tf(self):
        stamp = self._make_sim_stamp()

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

        # Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
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
        while rclpy.ok():
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1
                # 控制律
                self._apply_joint_torque()
                # 模拟一步
                mujoco.mj_step(self.model, self.data)

                self.timestamp = step * DT

                # 采样 & 发送观测
                self._publish_robot_state(step)
                self._publish_odom_and_tf()

                # LiDAR scan
                if step % self.lidar_step_interval == 0:
                    self.lidar.update(self.timestamp)

                # Depth camera
                if step % self.depth_step_interval == 0:
                    self.depth.update(self.timestamp)

                # Viewer
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.lidar.visualize()
                    self.viewer.sync()

            # Handle ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.0)

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

    def _publish_robot_state(self, step: int):
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
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
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
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
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


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    parser = argparse.ArgumentParser(description="Run Lite3 MuJoCo ROS2 simulation")
    parser.add_argument(
        "--xml",
        dest="xml_path",
        default=XML_PATH,
        help="Path to top-level MuJoCo XML (default: Lite3_stair.xml)",
    )
    args, ros_args = parser.parse_known_args()

    xml_path = str(Path(args.xml_path).expanduser().resolve())

    rclpy.init(args=ros_args)
    sim_node = MuJoCoSimulationNode(xml_path=xml_path)
    sim_node.start()
    sim_node.destroy_node()
    rclpy.shutdown()