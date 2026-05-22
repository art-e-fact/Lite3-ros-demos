"""
2D 360-degree LiDAR sensor simulation using MuJoCo ray casting.
Publishes sensor_msgs/msg/LaserScan on /scan.
"""

import math
import numpy as np
import mujoco

from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from scipy.spatial.transform import Rotation as R_scipy

# -- LiDAR configuration --
LIDAR_SITE_NAME = "lidar_site"
LIDAR_FRAME_ID = "lidar"
LIDAR_TOPIC = "/scan"
LIDAR_FREQUENCY_HZ = 10.0
NUM_RAYS = 360
RANGE_MIN = 0.15  # RPLiDAR A2M8 spec
RANGE_MAX = 8.0   # RPLiDAR A2M8 spec
VISUALIZE_RAYS = False  # set to True to draw rays in the MuJoCo viewer (for debugging)

RAY_VIS_HIT_RGBA = np.array([0.0, 1.0, 0.0, 0.3], dtype=np.float32)
RAY_VIS_MISS_RGBA = np.array([0.0, 1.0, 0.0, 0.1], dtype=np.float32)
RAY_VIS_WIDTH = 0.002  # line width in meters


class LidarSensor:
    @staticmethod
    def configure_spec(spec):
        """Make the lidar_site visible as a small cylinder marker.

        Call on the MjSpec before compile() when LiDAR is enabled.
        """
        torso = spec.worldbody.first_body()
        site = next(s for s in torso.sites if s.name == LIDAR_SITE_NAME)
        site.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        site.size = [0.025, 0.01, 0.0]
        site.rgba = [0.1, 0.1, 0.1, 1.0]
        site.group = 2

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 node: Node, viewer=None, enabled: bool = False):
        self.model = model
        self.data = data
        self.node = node
        self.viewer = viewer
        self.enabled = enabled
        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, LIDAR_SITE_NAME)

        if not self.enabled:
            node.get_logger().info("[INFO] LiDAR disabled")
            return

        if self.site_id < 0:
            raise ValueError(f"Site '{LIDAR_SITE_NAME}' not found in model")

        # Body that owns the site (rays will exclude it)
        self.body_id = model.site_bodyid[self.site_id]

        # Precompute local ray directions in the site's XY plane
        # RPLiDAR publishes from -pi to +pi (via M_PI - angle transform)
        angles = np.linspace(-math.pi, math.pi, NUM_RAYS, endpoint=False)
        self.local_dirs = np.column_stack([
            np.cos(angles), np.sin(angles), np.zeros(NUM_RAYS)
        ])  # (NUM_RAYS, 3)

        # Geom group filter: include all groups
        self.geomgroup = np.ones(mujoco.mjNGROUP, dtype=np.uint8)

        # Allocate output buffers
        self.distances = np.zeros(NUM_RAYS, dtype=np.float64)
        self.geom_ids = np.full(NUM_RAYS, -1, dtype=np.int32)

        # ROS publisher
        self.pub = node.create_publisher(LaserScan, LIDAR_TOPIC, 10)

        # Precompute constant LaserScan fields
        self.angle_increment = 2.0 * math.pi / NUM_RAYS
        self.scan_time = 1.0 / LIDAR_FREQUENCY_HZ
        self.time_increment = self.scan_time / (NUM_RAYS - 1)  # match RPLiDAR

    def update(self, timestamp: float):
        """Cast rays and publish LaserScan."""
        if not self.enabled:
            return
        # Site position and rotation in world frame
        site_pos = self.data.site_xpos[self.site_id]      # (3,)
        site_rot = self.data.site_xmat[self.site_id].reshape(3, 3)  # columns = site X,Y,Z in world

        # Transform local ray directions to world frame
        world_dirs = (site_rot @ self.local_dirs.T).T  # (NUM_RAYS, 3)

        # Batch ray cast
        mujoco.mj_multiRay(
            self.model, self.data,
            pnt=site_pos,
            vec=world_dirs.flatten(),
            geomgroup=self.geomgroup,
            flg_static=1,
            bodyexclude=self.body_id,
            geomid=self.geom_ids,
            dist=self.distances,
            normal=None,
            nray=NUM_RAYS,
            cutoff=RANGE_MAX,
        )

        # Build LaserScan message
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = LIDAR_FRAME_ID
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = self.angle_increment
        msg.time_increment = self.time_increment
        msg.scan_time = self.scan_time
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX

        # Convert distances: -1 (no hit) → inf
        ranges = self.distances.copy()
        ranges[ranges < 0] = float('inf')

        # Filter ground hits caused by body tilt while walking.
        # Any ray whose world-space direction points downward is hitting
        # the ground, not a real obstacle. Also filter hits close to ground.
        GROUND_Z_THRESHOLD = 0.15  # metres above Z=0
        downward_mask = world_dirs[:, 2] < -0.05  # ray pointing down
        ranges[downward_mask] = float('inf')

        # Also filter remaining hits whose world-Z is near ground
        hit_mask = ranges < float('inf')
        if np.any(hit_mask):
            hit_points_z = site_pos[2] + ranges[hit_mask] * world_dirs[hit_mask, 2]
            ground_hits = hit_points_z < GROUND_Z_THRESHOLD
            idx = np.where(hit_mask)[0][ground_hits]
            ranges[idx] = float('inf')

        msg.ranges = ranges.astype(np.float32).tolist()

        self.pub.publish(msg)

        # Cache for visualization
        self._last_site_pos = site_pos.copy()
        self._last_world_dirs = world_dirs
        self._last_distances = self.distances.copy()

    def visualize(self):
        """Draw LiDAR rays in the MuJoCo viewer."""
        if not self.enabled or not VISUALIZE_RAYS or self.viewer is None:
            return
        if not hasattr(self, '_last_site_pos'):
            return

        scn = self.viewer.user_scn
        scn.ngeom = 0  # clear previous frame

        origin = self._last_site_pos
        for i in range(NUM_RAYS):
            if scn.ngeom >= scn.maxgeom:
                break
            dist = self._last_distances[i]
            if dist < 0:
                # No hit: draw ray at max range using the configured miss color
                draw_dist = RANGE_MAX
                rgba = RAY_VIS_MISS_RGBA
            else:
                draw_dist = dist
                rgba = RAY_VIS_HIT_RGBA

            end = origin + self._last_world_dirs[i] * draw_dist
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_connector(
                g, mujoco.mjtGeom.mjGEOM_LINE, RAY_VIS_WIDTH,
                origin, end,
            )
            g.rgba[:] = rgba
            scn.ngeom += 1

    def get_static_transforms(self, stamp):
        """Return base_link -> lidar TF, read from MuJoCo model."""
        transforms = []
        if not self.enabled or self.site_id < 0:
            return transforms

        # Compute lidar pose relative to TORSO (base_link) via world-frame data
        torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'TORSO')
        torso_pos = self.data.xpos[torso_id]
        torso_rot = self.data.xmat[torso_id].reshape(3, 3)

        site_pos_w = self.data.site_xpos[self.site_id]
        site_rot_w = self.data.site_xmat[self.site_id].reshape(3, 3)

        pos_local = torso_rot.T @ (site_pos_w - torso_pos)
        rot_local = torso_rot.T @ site_rot_w
        quat_xyzw = R_scipy.from_matrix(rot_local).as_quat()  # [x, y, z, w]

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = LIDAR_FRAME_ID
        t.transform.translation = Vector3(
            x=float(pos_local[0]), y=float(pos_local[1]), z=float(pos_local[2]))
        t.transform.rotation = Quaternion(
            x=float(quat_xyzw[0]), y=float(quat_xyzw[1]),
            z=float(quat_xyzw[2]), w=float(quat_xyzw[3]))
        transforms.append(t)
        return transforms
