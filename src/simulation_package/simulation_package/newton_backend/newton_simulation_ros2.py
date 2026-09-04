#!/usr/bin/env python3

import argparse
import sys
import time
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))
SIMULATION_DIR = CURRENT_DIR.parent
if str(SIMULATION_DIR) not in sys.path:
    sys.path.insert(0, str(SIMULATION_DIR))

import rclpy

from ros_bridge import NewtonRosBridge
from simulation import DT, ROS_SPIN_EVERY_STEPS, ROBOT_PROFILES, NewtonSimulation, create_newton_viewer
from sensors.newton.sensor_manager import NewtonSensorManager, NewtonSensorOptions
from simulation_config import SimulationConfig

RENDER_EVERY_STEPS = 5

def run_loop(sim: NewtonSimulation, ros: NewtonRosBridge, state_every_steps: int, sensors: NewtonSensorManager | None = None):
    next_step_time = time.perf_counter()
    while rclpy.ok() and not ros.should_exit():
        sleep_time = next_step_time - time.perf_counter()
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        else:
            next_step_time = time.perf_counter()
        next_step_time += DT

        if sim.step_count % ROS_SPIN_EVERY_STEPS == 0:
            ros.spin_once()
        sim.set_command(ros.read_latest_action())
        sim.step()  

        if sensors is not None:
            sensors.update(sim.state_0, sim.step_count, sim.timestamp)

        state = None
        if sim.step_count % state_every_steps == 0:
            state = sim.state_snapshot()
            ros.publish_state(sim.timestamp, state, sim.last_tau)
            ros.publish_odom_and_tf(sim.timestamp, state)
        
        if sim.step_count % RENDER_EVERY_STEPS == 0:
            sim.render()


def run_newton(config: SimulationConfig, ros_args: list[str] | None = None):
    rclpy.init(args=ros_args)
    model_path = config.resolved_robot_description()
    scene_path = config.resolved_scene()
    profile = ROBOT_PROFILES.get(config.robot.model_name)
    if profile is None:
        raise SystemExit(f"No Newton robot profile for '{config.robot.model_name}'")
    viewer = None if config.headless else create_newton_viewer()
    ros = NewtonRosBridge(headless=config.headless, model_path=model_path, profile=profile)
    sensor_options = NewtonSensorOptions(
        lidar_2d=config.sensors.lidar_2d,
        mid360=config.sensors.mid360,
        realsense=config.sensors.realsense,
        robosense=config.sensors.robosense,
    )
    sim = NewtonSimulation(
        model_path=model_path,
        scene_path=scene_path,
        headless=config.headless,
        viewer=viewer,
        logger=ros.get_logger(),
        sensor_options=sensor_options,
        profile=profile,
    )
    sensors = NewtonSensorManager(sim.model, sim.state_0, ros.node, DT, sensor_options)

    state_every_steps = max(1, int(round(1.0 / (config.robot.state_frequency_hz * DT))))

    try:
        run_loop(sim, ros, state_every_steps, sensors)
    finally:
        ros.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Run Lite3 Newton ROS2 simulation")
    parser.add_argument("--config", default=None, help="Path to simulation YAML config")
    args, ros_args = parser.parse_known_args()

    config = SimulationConfig.load(args.config).with_overrides({"simulator": "newton"})
    errors = config.validate()
    if errors:
        raise SystemExit("Invalid simulation config:\n- " + "\n- ".join(errors))
    run_newton(config, ros_args)


if __name__ == "__main__":
    main()
