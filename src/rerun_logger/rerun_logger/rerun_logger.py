#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections.abc import Callable
import subprocess
import warnings

import numpy as np

import rerun as rr  # pip install rerun-sdk

from ament_index_python.packages import get_package_share_directory

ROBOT_PATH = f"{get_package_share_directory('assets_package')}/deep_robotics_model/Lite3/Lite3_urdf/urdf/Lite3.urdf"

import rclpy
from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Float32
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray
from drdds.msg import JointsData

from rerun_logger.local_heightmap_rerun import log_front_clear_markers, log_local_heightmap
from rerun_logger.rail_detector_rerun import log_rail_detector_markers

warnings.filterwarnings(
    "ignore",
    message=r"Joint .* angle .* is outside limits .* Clamping\.",
    category=UserWarning,
)

# Real-robot /joint_states names (LF/RF/LB/RB) -> Lite3 URDF joint names (FL/FR/HL/HR).
JOINT_STATE_NAME_TO_URDF = {
    "LF_Joint": "FL_HipX_joint",
    "LF_Joint_1": "FL_HipY_joint",
    "LF_Joint_2": "FL_Knee_joint",
    "RF_Joint": "FR_HipX_joint",
    "RF_Joint_1": "FR_HipY_joint",
    "RF_Joint_2": "FR_Knee_joint",
    "LB_Joint": "HL_HipX_joint",
    "LB_Joint_1": "HL_HipY_joint",
    "LB_Joint_2": "HL_Knee_joint",
    "RB_Joint": "HR_HipX_joint",
    "RB_Joint_1": "HR_HipY_joint",
    "RB_Joint_2": "HR_Knee_joint",
}


class RerunSubscriber(Node):  # type: ignore[misc]
    def __init__(self, *, log_heightmap: bool = False, static_heightmap: bool = False) -> None:
        super().__init__("rr_turtlebot")
        self._static_heightmap = static_heightmap

        # Assorted helpers for data conversions
        self.subscribers: list[rclpy.Subscription] = []

        # Subscribe to the topics we want to republish to Rerun.
        # See the callback methods below for how each message type is handled.
        self.subscribe("/tf", TFMessage, self.tf_callback)
        self.subscribe("/tf_static", TFMessage, self.tf_callback, latching=True)
        self.subscribe("/mid360/points", PointCloud2, self.scan_callback)
        self.subscribe("/livox/lidar", PointCloud2, self.scan_callback)
        rr.log_file_from_path(
            file_path=ROBOT_PATH,
            entity_path_prefix="urdf",
            static=True,
        )
        rr.log("/urdf", rr.CoordinateFrame("base_link"), static=True)
        rr.log(
            "transforms",
            rr.Transform3D(
                child_frame="TORSO",
                parent_frame="base_link",
            ),
            static=True,
        )
        rr.log("/", rr.CoordinateFrame("odom"), static=True)
        self.urdf_tree = rr.urdf.UrdfTree.from_file_path(ROBOT_PATH, entity_path_prefix="urdf")
        self.joint_name_to_index = {
            'FL_HipX_joint': 0, 'FL_HipY_joint': 1, 'FL_Knee_joint': 2,
            'FR_HipX_joint': 3, 'FR_HipY_joint': 4, 'FR_Knee_joint': 5,
            'HL_HipX_joint': 6, 'HL_HipY_joint': 7, 'HL_Knee_joint': 8,
            'HR_HipX_joint': 9, 'HR_HipY_joint': 10, 'HR_Knee_joint': 11
        }

        self.log_frame_box(
            frame_name="odom",
            box_entity_path="odom_box",
            color=(255, 0, 0),
            size=(0.2, 0.2, 0.03),
        )
        self.log_frame_box(
            frame_name="base_link",
            box_entity_path="base_link_box",
            color=(0, 128, 255),
            size=(0.1, 0.1, 0.1)
        )

        self.subscribe("/JOINTS_DATA", JointsData, self.joints_callback)
        self.subscribe("/joint_states", JointState, self.joint_states_callback)
        self.subscribe("/rail_detector/markers", MarkerArray, self.rail_detector_markers_callback)
        self.subscribe("/perf/height_scan", Float32, self.height_scan_perf_callback)
        self._detector_frame: str | None = None
        self._front_clear_frame: str | None = None
        self._heartbeat_count = 0
        self.create_timer(1.0, self._heartbeat_callback)
        if log_heightmap:
            self.subscribe("/local_heightmap", GridMap, self.local_heightmap_callback)
            self.subscribe(
                "/local_heightmap/front_clear_markers",
                MarkerArray,
                self.front_clear_markers_callback,
            )

    def subscribe(
        self, topic: str, msg_type: type, callback: Callable[[rclpy.MsgT], None], latching: bool = False
    ) -> None:
        """Adds a subscriber to a topic with the given message type and callback."""
        # `qos_profile` can either be an int (history depth) or a QoSProfile.
        # See: https://docs.ros.org/en/rolling/p/rclpy/rclpy.node.html#rclpy.node.Node.create_subscription
        qos_profile = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL) if latching else 10
        sub = self.create_subscription(
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos_profile=qos_profile,
            callback_group=ReentrantCallbackGroup(),  # allow concurrent callbacks
        )
        self.subscribers.append(sub)

    def log_frame_box(self, frame_name: str, box_entity_path: str, color: tuple[int, int, int], size: tuple[float, float, float]) -> None:
        """
        Logs a small static box at the origin of a named frame.
        """
        rr.log(box_entity_path, rr.CoordinateFrame(frame=frame_name), static=True)
        rr.log(
            box_entity_path,
            rr.Boxes3D(
                centers=[[0.0, 0.0, 0.0]],
                half_sizes=[[size[0] / 2, size[1] / 2, size[2] / 2]],
                radii=0.01,
                colors=[color],
                fill_mode="solid",
            ),
            static=True,
        )

    def scan_callback(self, cloud: PointCloud2) -> None:
        """
        Logs a PointCloud2 message to Rerun.
        """
        time = Time.from_msg(cloud.header.stamp)
        rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))

        # Read fields x, y, z from PointCloud2
        pts = point_cloud2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
        pts = structured_to_unstructured(pts)

        # Log to Rerun as Points3D
        rr.log("scan", rr.Points3D(positions=pts, colors=[255, 165, 0], radii=0.01))
        rr.log("scan", rr.CoordinateFrame(frame=cloud.header.frame_id))

    def tf_callback(self, tf_msg: TFMessage) -> None:
        """
        Logs TF transforms to Rerun as Transform3D messages,
        with `parent_frame` and `child_frame` fields set.

        Documentation about transforms in Rerun can be found here:
        https://rerun.io/docs/concepts/transforms
        """
        for transform in tf_msg.transforms:
            time = Time.from_msg(transform.header.stamp)
            rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))
            rr.log(
                "transforms",
                # f"transforms/{transform.child_frame_id}",
                rr.Transform3D(
                    translation=[
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ],
                    rotation=rr.Quaternion(
                        xyzw=[
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w,
                        ]
                    ),
                    parent_frame=transform.header.frame_id,
                    child_frame=transform.child_frame_id,
                ),
                # static=True,  # Uncomment this if the transform is static
            )

    def rail_detector_markers_callback(self, msg: MarkerArray) -> None:
        self._detector_frame = log_rail_detector_markers(msg, self._detector_frame)

    def local_heightmap_callback(self, msg: GridMap) -> None:
        log_local_heightmap(msg, static=self._static_heightmap)

    def front_clear_markers_callback(self, msg: MarkerArray) -> None:
        self._front_clear_frame = log_front_clear_markers(msg, self._front_clear_frame)

    def height_scan_perf_callback(self, msg: Float32) -> None:
        time = self.get_clock().now()
        rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))
        rr.log("perf/height-scan", rr.Scalars(msg.data))

    def _heartbeat_callback(self) -> None:
        time = self.get_clock().now()
        rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))
        rr.log("logger/alive", rr.Scalars(float(self._heartbeat_count)))
        self._heartbeat_count += 1

    def _log_joint_angles(self, angles_by_name: dict[str, float]) -> None:
        """Logs joint transforms to Rerun given a mapping of joint name -> angle (radians)."""
        for joint in self.urdf_tree.joints():
            if joint.joint_type == "revolute" and joint.name in angles_by_name:
                transform = joint.compute_transform(angles_by_name[joint.name], clamp=True)
                rr.log("transforms", transform)

    def joints_callback(self, msg: JointsData) -> None:
        """
        Logs actual joint motions from JointsData to Rerun.
        """
        time = Time.from_msg(msg.header.stamp)
        rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))

        angles_by_name = {
            name: msg.data.joints_data[idx].position
            for name, idx in self.joint_name_to_index.items()
            if idx < len(msg.data.joints_data)
        }
        self._log_joint_angles(angles_by_name)

    def joint_states_callback(self, msg: JointState) -> None:
        """
        Logs joint motions from a standard sensor_msgs/JointState message to Rerun.
        """
        time = Time.from_msg(msg.header.stamp)
        rr.set_time("ros_time", timestamp=np.datetime64(time.nanoseconds, "ns"))

        # The joint states message uses different names on real hardware than the URDF.
        angles_by_name: dict[str, float] = {}
        for name, pos in zip(msg.name, msg.position):
            urdf_name = JOINT_STATE_NAME_TO_URDF.get(name, name)
            if urdf_name in self.joint_name_to_index:
                angles_by_name[urdf_name] = pos
        self._log_joint_angles(angles_by_name)

def main() -> None:
    parser = argparse.ArgumentParser(description="Rerun logger for the Lite3 rail demo.")
    rr.script_add_args(parser)
    parser.add_argument(
        "--log_heightmap",
        action="store_true",
        help="Subscribe to /local_heightmap and visualise it as Boxes3D (off by default).",
    )
    parser.add_argument(
        "--use_static_heightmap",
        action="store_true",
        help="Log the heightmap as a static entity so it is not recorded to the timeline.",
    )
    parser.add_argument(
        "--onboard-fix",
        action="store_true",
        help="Start a gRPC server for remote viewing and kill rerun on exit.",
    )
    args, unknownargs = parser.parse_known_args()
    rr.script_setup(args, "lite3_rail")

    if args.onboard_fix:
        rr.serve_grpc(grpc_port=9876)

    rclpy.init(args=unknownargs)

    rerun_subscriber = RerunSubscriber(
        log_heightmap=args.log_heightmap,
        static_heightmap=args.use_static_heightmap,
    )

    try:
        rclpy.spin(rerun_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        rerun_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
