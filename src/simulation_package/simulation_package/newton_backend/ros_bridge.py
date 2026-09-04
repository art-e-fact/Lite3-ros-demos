import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from drdds.msg import ImuData, ImuDataValue, JointData, JointsData, JointsDataCmd, JointsDataValue, MetaType
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from simulation import DEFAULT_DAMPING, DEFAULT_STIFFNESS, ROBOT_PROFILES, JointCommand, RobotProfile, quat_xyzw_to_rpy


class NewtonRosBridge:
    def __init__(self, headless: bool, model_path: str, profile: RobotProfile | None = None):
        self.node = Node("newton_simulation")
        self.node.declare_parameter("headless", headless)
        self.node.declare_parameter("model_path", model_path)
        self._shutdown_requested = False

        self.profile = profile if profile is not None else ROBOT_PROFILES["Lite3"]
        self.num_dofs = self.profile.num_dofs
        self.kp_cmd = np.full(self.num_dofs, DEFAULT_STIFFNESS, dtype=np.float32)
        self.kd_cmd = np.full(self.num_dofs, DEFAULT_DAMPING, dtype=np.float32)
        self.pos_cmd = self.profile.default_joint_pos.copy()
        self.vel_cmd = np.zeros(self.num_dofs, dtype=np.float32)
        self.tau_ff = np.zeros(self.num_dofs, dtype=np.float32)

        self.imu_pub = self.node.create_publisher(ImuData, "/IMU_DATA", 200)
        self.joints_pub = self.node.create_publisher(JointsData, "/JOINTS_DATA", 200)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 50)
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.cmd_sub = self.node.create_subscription(JointsDataCmd, "/JOINTS_CMD", self._cmd_callback, 10)

    def get_logger(self):
        return self.node.get_logger()

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def should_exit(self) -> bool:
        return self._shutdown_requested or not rclpy.ok()

    def destroy(self):
        self.node.destroy_node()

    def read_latest_action(self) -> JointCommand:
        return JointCommand(
            kp=self.kp_cmd.copy(),
            kd=self.kd_cmd.copy(),
            position=self.pos_cmd.copy(),
            velocity=self.vel_cmd.copy(),
            torque=self.tau_ff.copy(),
        )

    def publish_state(self, timestamp: float, state, last_tau: np.ndarray):
        stamp = self._stamp(timestamp)
        rpy_deg = np.degrees(quat_xyzw_to_rpy(state.quat_xyzw))

        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(state.imu_gyro[0])
        imu_msg.data.omega_y = float(state.imu_gyro[1])
        imu_msg.data.omega_z = float(state.imu_gyro[2])
        imu_msg.data.acc_x = float(state.imu_acc[0])
        imu_msg.data.acc_y = float(state.imu_acc[1])
        imu_msg.data.acc_z = float(state.imu_acc[2])
        self.imu_pub.publish(imu_msg)

        joints_msg = JointsData()
        joints_msg.header = MetaType()
        joints_msg.header.frame_id = 0
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(16)]
        pub_pos, pub_vel, pub_tau = self._raw_to_pub(state.joint_position, state.joint_velocity, last_tau)
        for index in range(self.num_dofs):
            joint = joints_msg.data.joints_data[index]
            joint.name = [32, 32, 32, 32]
            joint.data_id = 0
            joint.status_word = 1
            joint.position = float(pub_pos[index])
            joint.velocity = float(pub_vel[index])
            joint.torque = float(pub_tau[index])
            joint.motion_temp = 40.0
            joint.driver_temp = 45.0
        for index in range(self.num_dofs, 16):
            joints_msg.data.joints_data[index].status_word = 1
        self.joints_pub.publish(joints_msg)

    def publish_odom_and_tf(self, timestamp: float, state):
        stamp = self._stamp(timestamp)
        linvel_body = state.linvel_body

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(state.position[0])
        transform.transform.translation.y = float(state.position[1])
        transform.transform.translation.z = float(state.position[2])
        transform.transform.rotation = Quaternion(
            x=float(state.quat_xyzw[0]),
            y=float(state.quat_xyzw[1]),
            z=float(state.quat_xyzw[2]),
            w=float(state.quat_xyzw[3]),
        )
        self.tf_broadcaster.sendTransform(transform)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = float(state.position[0])
        odom_msg.pose.pose.position.y = float(state.position[1])
        odom_msg.pose.pose.position.z = float(state.position[2])
        odom_msg.pose.pose.orientation = transform.transform.rotation
        odom_msg.twist.twist.linear.x = float(linvel_body[0])
        odom_msg.twist.twist.linear.y = float(linvel_body[1])
        odom_msg.twist.twist.linear.z = float(linvel_body[2])
        odom_msg.twist.twist.angular.x = float(state.angvel_body[0])
        odom_msg.twist.twist.angular.y = float(state.angvel_body[1])
        odom_msg.twist.twist.angular.z = float(state.angvel_body[2])
        self.odom_pub.publish(odom_msg)

    def _cmd_callback(self, msg: JointsDataCmd):
        if len(msg.data.joints_data) not in (self.num_dofs, 16):
            self.node.get_logger().warn("Received JointsDataCmd with incorrect number of joints")
            return

        pub_pos = np.zeros(self.num_dofs, dtype=np.float32)
        pub_vel = np.zeros(self.num_dofs, dtype=np.float32)
        for index in range(self.num_dofs):
            joint_cmd = msg.data.joints_data[index]
            self.kp_cmd[index] = joint_cmd.kp
            self.kd_cmd[index] = joint_cmd.kd
            pub_pos[index] = joint_cmd.position
            pub_vel[index] = joint_cmd.velocity
            self.tau_ff[index] = joint_cmd.torque  # tau_ff no processing, matches MuJoCo bridge

        if self.profile.joint_dir is not None:
            self.pos_cmd[:] = pub_pos * self.profile.joint_dir + self.profile.pos_offset_rad
            self.vel_cmd[:] = pub_vel * self.profile.joint_dir
        else:
            self.pos_cmd[:] = pub_pos
            self.vel_cmd[:] = pub_vel

    def _raw_to_pub(self, q: np.ndarray, dq: np.ndarray, tau: np.ndarray):
        if self.profile.joint_dir is None:
            return q, dq, tau
        direction = self.profile.joint_dir
        return (q - self.profile.pos_offset_rad) * direction, dq * direction, tau * direction

    @staticmethod
    def _stamp(timestamp: float) -> Time:
        stamp = Time()
        sec = int(timestamp)
        stamp.sec = sec
        stamp.nanosec = int((timestamp - sec) * 1e9)
        return stamp
