import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import newton
import newton.examples
import warp as wp
from newton import JointTargetMode

from sensors.newton.depth_sensor import NewtonDepthSensor
from sensors.newton.lidar_sensor import NewtonLidarSensor
from sensors.newton.mid360_lidar_sensor import NewtonMid360LidarSensor


NUM_DOFS = 12
BASE_DOF_COUNT = 6
FLOATING_BASE_Q_SIZE = 7
DT = 0.004
ROS_SPIN_EVERY_STEPS = 1
PUBLISH_EVERY_STEPS = 5
ODOM_EVERY_STEPS = 20
DEFAULT_JOINT_POS = np.array(
    [0.0, -1.35453, 2.54948, 0.0, -1.35453, 2.54948, 0.0, -1.35453, 2.54948, 0.0, -1.35453, 2.54948],
    dtype=np.float32,
)
DEFAULT_STIFFNESS = 30.0
DEFAULT_DAMPING = 1.0
ARMATURE = 0.0

# M20 sim-to-policy frame, mirrors mujoco_simulation_ros2.py (DdsInterface::Handler).
M20_JOINT_DIR = np.array(
    [1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1],
    dtype=np.float32,
)
M20_POS_OFFSET_RAD = (
    np.array(
        [-25, -131, 160, 0, 25, -131, 160, 0, -25, 131, -160, 0, 25, 131, -160, 0],
        dtype=np.float32,
    )
    / 180.0
    * np.pi
)


@dataclass(frozen=True)
class RobotProfile:
    name: str
    num_dofs: int
    base_height: float
    default_joint_pos: np.ndarray
    effort_limit: np.ndarray  # per-DOF |tau| clamp, mirrors MJCF actuator ctrlrange
    joint_dir: np.ndarray | None = None  # pub<->raw joint calibration, None = identity
    pos_offset_rad: np.ndarray | None = None


ROBOT_PROFILES: dict[str, RobotProfile] = {
    "Lite3": RobotProfile(
        name="Lite3",
        num_dofs=12,
        base_height=0.43,
        default_joint_pos=DEFAULT_JOINT_POS,
        effort_limit=np.full(12, 30.0, dtype=np.float32),
    ),
    "M20": RobotProfile(
        name="M20",
        num_dofs=16,
        base_height=1.0,
        default_joint_pos=np.array(
            [-0.438, -1.16, 2.76, 0, 0.438, -1.16, 2.76, 0, -0.438, 1.16, -2.76, 0, 0.438, 1.16, -2.76, 0],
            dtype=np.float32,
        ),
        effort_limit=np.array([76.4, 76.4, 76.4, 21.6] * 4, dtype=np.float32),
        joint_dir=M20_JOINT_DIR,
        pos_offset_rad=M20_POS_OFFSET_RAD,
    ),
}


@dataclass
class JointCommand:
    kp: np.ndarray
    kd: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray


@dataclass
class SimulationState:
    position: np.ndarray
    quat_xyzw: np.ndarray
    linvel_body: np.ndarray
    angvel_body: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray

def create_newton_viewer():
    parser = newton.examples.create_parser()
    saved_argv = sys.argv
    try:
        sys.argv = [saved_argv[0]]
        viewer, _ = newton.examples.init(parser)
        return viewer
    finally:
        sys.argv = saved_argv


def quat_xyzw_to_rpy(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=np.float32)


def rotate_world_to_body(quat_xyzw: np.ndarray, vec_world: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_xyzw
    q_vec = np.array([x, y, z], dtype=np.float32)
    return vec_world * (2.0 * w * w - 1.0) - np.cross(q_vec, vec_world) * (2.0 * w) + q_vec * (2.0 * np.dot(q_vec, vec_world))


def warp_to_numpy(array, fallback_size: int) -> np.ndarray:
    if array is None:
        return np.zeros(fallback_size, dtype=np.float32)
    try:
        return np.asarray(array.numpy(), dtype=np.float32).copy()
    except Exception:
        try:
            return np.asarray(array, dtype=np.float32).copy()
        except Exception:
            return np.zeros(fallback_size, dtype=np.float32)


class NewtonSimulation:
    def __init__(self, model_path: str, scene_path: str | None = None, headless: bool = True, viewer=None, logger=None, sensor_options=None, profile: RobotProfile | None = None):
        self.model_path = model_path
        self.scene_path = scene_path
        self.headless = headless
        self.viewer = None if headless else viewer
        self.logger = logger
        self.sensor_options = sensor_options
        self.profile = profile if profile is not None else ROBOT_PROFILES["Lite3"]
        self.num_dofs = self.profile.num_dofs
        self.device = wp.get_device()
        self.timestamp = 0.0
        self.step_count = 0

        self.kp_cmd = np.full(self.num_dofs, DEFAULT_STIFFNESS, dtype=np.float32)
        self.kd_cmd = np.full(self.num_dofs, DEFAULT_DAMPING, dtype=np.float32)
        self.pos_cmd = self.profile.default_joint_pos.copy()
        self.vel_cmd = np.zeros(self.num_dofs, dtype=np.float32)
        self.tau_ff = np.zeros(self.num_dofs, dtype=np.float32)
        self.last_tau = np.zeros(self.num_dofs, dtype=np.float32)

        self.model, self.solver, self.state_0, self.state_1, self.control = self._build_newton_model(model_path, scene_path)
        self.contacts = self._create_contacts()
        self.graph = None
        self.use_cuda_graph = False
        self._joint_f_buffer = np.zeros(BASE_DOF_COUNT + self.num_dofs, dtype=np.float32)

        if self.viewer is not None:
            self.viewer.set_model(self.model)
            self.viewer.vsync = True
            # newton.examples.init() shows a splash that examples.run() would normally hide.
            if hasattr(self.viewer, "hide_loading_splash"):
                self.viewer.hide_loading_splash()

        self._setup_cuda_graph()
        self._log_info(f"Newton {self.profile.name} simulation loaded: {self.model_path}")
        if self.scene_path is not None:
            self._log_info(f"Newton environment scene loaded: {self.scene_path}")
        if self.viewer is not None:
            self._log_info("Newton viewer enabled because headless is false.")
        self._log_info("Optional sensors are managed by the ROS runner.")

    def set_command(self, command: JointCommand):
        self.kp_cmd[:] = command.kp
        self.kd_cmd[:] = command.kd
        self.pos_cmd[:] = command.position
        self.vel_cmd[:] = command.velocity
        self.tau_ff[:] = command.torque

    def step(self):
        self._apply_control()
        if self.graph is not None:
            wp.capture_launch(self.graph)
        else:
            self._simulate_physics_step(apply_viewer_forces=True, keep_state_buffers=False)
        self.timestamp += DT
        self.step_count += 1

    def render(self):
        if self.viewer is None:
            return
        self.viewer.begin_frame(self.timestamp)
        self.viewer.log_state(self.state_0)
        if self.contacts is not None and hasattr(self.viewer, "log_contacts"):
            self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.end_frame()

    def state_snapshot(self) -> SimulationState:
        joint_q = warp_to_numpy(self.state_0.joint_q, FLOATING_BASE_Q_SIZE + self.num_dofs)
        joint_qd = warp_to_numpy(self.state_0.joint_qd, BASE_DOF_COUNT + self.num_dofs)
        quat_xyzw = joint_q[3:7]
        return SimulationState(
            position=joint_q[:3],
            quat_xyzw=quat_xyzw,
            linvel_body=rotate_world_to_body(quat_xyzw, joint_qd[:3]),
            angvel_body=rotate_world_to_body(quat_xyzw, joint_qd[3:6]),
            joint_position=joint_q[FLOATING_BASE_Q_SIZE : FLOATING_BASE_Q_SIZE + self.num_dofs],
            joint_velocity=joint_qd[BASE_DOF_COUNT : BASE_DOF_COUNT + self.num_dofs],
        )

    def _setup_cuda_graph(self):
        if not self.device.is_cuda:
            self._log_info("CUDA graph disabled because Newton is not running on a CUDA device.")
            return
        if not wp.is_mempool_enabled(self.device):
            self._log_info("CUDA graph disabled because the Warp memory pool is not enabled.")
            return
        if self.viewer is not None:
            self._log_info("CUDA graph disabled while the interactive Newton viewer is enabled.")
            return

        try:
            with wp.ScopedCapture() as capture:
                self._simulate_physics_step(apply_viewer_forces=False, keep_state_buffers=True)
            self.graph = capture.graph
            self.use_cuda_graph = True
            self._log_info("Using CUDA graph for Newton physics stepping.")
        except Exception as exc:
            self.graph = None
            self.use_cuda_graph = False
            self._log_warn(f"CUDA graph capture failed; falling back to uncaptured Newton stepping: {exc}")

    def _create_contacts(self):
        try:
            return newton.Contacts(self.solver.get_max_contact_count(), 0)
        except Exception as exc:
            self._log_debug(f"Newton contacts unavailable for viewer logging: {exc}")
            return None

    def _build_newton_model(self, model_path: str, scene_path: str | None = None):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Lite3 model description not found: {model_path}")

        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
        builder.default_joint_cfg = newton.ModelBuilder.JointDofConfig(armature=ARMATURE, limit_ke=1.0e2, limit_kd=1.0e0)
        builder.default_shape_cfg.ke = 5.0e4
        builder.default_shape_cfg.kd = 5.0e2
        builder.default_shape_cfg.kf = 1.0e3
        builder.default_shape_cfg.mu = 0.75

        scene_loaded = self._add_environment_scene(builder, scene_path, model_path)

        if Path(model_path).suffix.lower() == ".xml":
            builder.add_mjcf(
                model_path,
                xform=wp.transform(wp.vec3(0.0, 0.0, 0.0)),
                collapse_fixed_joints=True,
                enable_self_collisions=False,
                parse_visuals=True,
                parse_meshes=True,
                parse_sites=True,
                ignore_names=("floor",),
            )
        else:
            builder.add_usd(
                model_path,
                xform=wp.transform(wp.vec3(0.0, 0.0, 0.0)),
                collapse_fixed_joints=True,
                enable_self_collisions=False,
                joint_ordering="dfs",
                hide_collision_shapes=True,
            )

        self._init_sensor_visuals(builder)
        builder.approximate_meshes("convex_hull")
        if not scene_loaded:
            builder.add_ground_plane()

        builder.joint_q[:3] = [-5.0, 0.0, self.profile.base_height]
        builder.joint_q[3:7] = [0.0, 0.0, 0.0, 1.0]
        builder.joint_q[FLOATING_BASE_Q_SIZE : FLOATING_BASE_Q_SIZE + self.num_dofs] = self.profile.default_joint_pos.tolist()

        for dof_index in range(self.num_dofs):
            target_index = BASE_DOF_COUNT + dof_index
            # PD runs on CPU and enters via control.joint_f; solver-side servo stays off.
            builder.joint_target_ke[target_index] = 0.0
            builder.joint_target_kd[target_index] = 0.0
            builder.joint_armature[target_index] = ARMATURE
            builder.joint_target_mode[target_index] = int(JointTargetMode.NONE)

        model = builder.finalize()
        model.set_gravity((0.0, 0.0, -9.81))
        solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False, solver="newton", nconmax=300, njmax=1000)
        state_0 = model.state()
        state_1 = model.state()
        control = model.control()
        if control.joint_f is None:
            raise RuntimeError("Newton Control.joint_f is unavailable; cannot apply joint torques")
        newton.eval_fk(model, state_0.joint_q, state_0.joint_qd, state_0)
        newton.eval_fk(model, state_1.joint_q, state_1.joint_qd, state_1)
        return model, solver, state_0, state_1, control

    def _add_environment_scene(self, builder, scene_path: str | None, model_path: str) -> bool:
        if not scene_path:
            return False

        scene_file = Path(scene_path)
        if scene_file.resolve() == Path(model_path).resolve():
            return False
        if scene_file.suffix.lower() not in {".xml", ".mjcf"}:
            return False

        builder.add_mjcf(
            str(scene_file),
            xform=wp.transform(wp.vec3(0.0, 0.0, 0.0)),
            collapse_fixed_joints=True,
            enable_self_collisions=False,
            parse_visuals=True,
            parse_meshes=True,
            parse_sites=False,
        )
        return True

    def _init_sensor_visuals(self, builder):
        options = self.sensor_options
        if options is None:
            return

        if getattr(options, "enable_lidar", False):
            NewtonLidarSensor.init_visuals(builder, options.lidar_2d)
        if getattr(options, "enable_depth", False) or getattr(options, "enable_color", False):
            NewtonDepthSensor.init_visuals(builder, options.realsense)
        if getattr(options, "enable_mid360", False):
            NewtonMid360LidarSensor.init_visuals(builder, options.mid360)

    def _simulate_physics_step(self, apply_viewer_forces: bool, keep_state_buffers: bool):
        self.state_0.clear_forces()
        if apply_viewer_forces and self.viewer is not None and hasattr(self.viewer, "apply_forces"):
            self.viewer.apply_forces(self.state_0)
        self.solver.step(self.state_0, self.state_1, self.control, None, DT)
        if keep_state_buffers:
            self.state_0.assign(self.state_1)
        else:
            self.state_0, self.state_1 = self.state_1, self.state_0
        if self.contacts is not None:
            self.solver.update_contacts(self.contacts, self.state_0)

    def _apply_control(self):
        joint_q = warp_to_numpy(self.state_0.joint_q, FLOATING_BASE_Q_SIZE + self.num_dofs)
        joint_qd = warp_to_numpy(self.state_0.joint_qd, BASE_DOF_COUNT + self.num_dofs)
        q = joint_q[FLOATING_BASE_Q_SIZE : FLOATING_BASE_Q_SIZE + self.num_dofs]
        dq = joint_qd[BASE_DOF_COUNT : BASE_DOF_COUNT + self.num_dofs]
        tau = self.kp_cmd * (self.pos_cmd - q) + self.kd_cmd * (self.vel_cmd - dq) + self.tau_ff
        limit = self.profile.effort_limit
        self.last_tau = np.clip(tau, -limit, limit)

        self._joint_f_buffer[BASE_DOF_COUNT:] = self.last_tau
        self.control.joint_f.assign(self._joint_f_buffer)

    def _log_info(self, message: str):
        self.logger.info(message) if self.logger is not None else print(f"[INFO] {message}")

    def _log_warn(self, message: str):
        self.logger.warn(message) if self.logger is not None else print(f"[WARN] {message}")

    def _log_debug(self, message: str):
        if self.logger is not None:
            self.logger.debug(message)
