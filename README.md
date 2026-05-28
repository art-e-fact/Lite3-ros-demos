# SDK DEPLOY

## Installation

### Prerequisites

- **ROS 2 Humble** installed and sourced ([install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))

> **Tip:** A VS Code Dev Container config is included in `.devcontainer/`. Open the repo in VS Code and select **Reopen in Container** to get a pre-configured ROS 2 Humble environment with all dependencies (including GPU passthrough). This is optional — you can also install everything natively.
>
> **X11 note (for MuJoCo/GLFW in devcontainer):** If simulation fails with errors like `X11: Failed to open display :0`, allow the container/root user to access your host X server before launching:
> ```bash
> xhost +si:localuser:root
> ```
> Run this on the host (outside the container) each new login/session.


### System dependencies:
  ```bash
  sudo apt install libevdev-dev
  rosdep install --from-paths src --ignore-src -r -y
  ```

### Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_PLATFORM=x86
source install/setup.bash
```

Run `colcon build` from a shell with ROS sourced but without the simulation venv activated.

### Python venv (for simulation only)

The MuJoCo simulation requires Python packages not available via rosdep. Set up a venv once:

```bash
python3 -m venv venv
touch venv/COLCON_IGNORE
source venv/bin/activate
pip install -r requirements.txt
```

Activate the venv before running simulation commands.

Inside the devcontainer, the first post-create setup creates this `venv/` automatically and marks it with `COLCON_IGNORE` so ROS builds do not traverse it.


## RTAB-Map SLAM + Nav2 Autonomous Navigation

Uses RTAB-Map (2D lidar ICP mode) for SLAM, then Nav2 for autonomous goal navigation.

### 1. Build a Map (SLAM)

- Using launch file:
```bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py mode:=2 control_type:=0
```
- Arguments:
  - `mode`: RTAB-Map mode (0 - lidar, 1 - rgbd, 2 - lidar+rgbd). Default: 2.
  - `control_type`: Joints controller type for RL policy (0 - twist, 1 - keyboard, 2 - gamepad). Default: 0.

### 2. Navigate with Nav2

After building a map, use RTAB-Map in localization mode with Nav2 for autonomous navigation:

```bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py mode:=2 control_type:=0 localization:=true
```

In RViz2: Set **2D Goal Pose** to send a navigation goal

### Optional: Procedural Scene (MuJoCo)

By default, `mujoco_simulation_ros2.launch.py` uses the authored static-scene presets. To switch the full launch stack to a procedural environment, pass a procedural simulation config:

```bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py \
  mode:=2 control_type:=0 \
  simulation_config:=src/Lite3_sdk_deploy/config/simulations/mujoco_procedural_rgbd_lidar.yaml
```

For a reproducible procedural seed, create a small custom YAML from that preset and set `procedural_env_seed` there.

```bash
cp src/Lite3_sdk_deploy/config/simulations/mujoco_procedural_rgbd_lidar.yaml /tmp/my_proc_scene.yaml
```

Stand the robot with "z" then put into RL control mode with "c". Drive the robot around with keyboard controls (wasd) to build the map.

The map is saved automatically to `~/.ros/rtabmap.db`. RTAB-Map will reload it in localization mode.

### Optional: Autonomous Waypoint Traversal (Procedural Scene)

You can automate exploration for map-building by launching the full stack in one command (MuJoCo + RL twist + waypoint navigator + RTAB-Map + RViz mapping config):

```bash
source install/setup.bash
source venv/bin/activate
ros2 launch lite3_sdk_deploy autonomous_mapping.launch.py mode:=2 control_type:=0
```

Headless mode (MuJoCo viewer off and RViz disabled):

```bash
ros2 launch lite3_sdk_deploy autonomous_mapping.launch.py headless:=true mode:=2 control_type:=0
```


Notes:
- The waypoint mission is generated from the scene graph to traverse all free-space edges (some waypoints may be revisited by design).
- This mode is a coverage helper and does not perform reactive obstacle avoidance beyond following the free-space corridors generated in the procedural map.
- Waypoints are published on `/procedural_waypoints` (`geometry_msgs/PoseArray`, `odom` frame) and velocity commands are published on `/cmd_vel`.


## Twist Control (Simulation)

### Run

```bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py mode:=2 control_type:=0
```

This launches MuJoCo, the RL twist controller, Nav2, RTAB-Map, and RViz together. The controller automatically stands the robot up and enters RL mode after ~5 seconds.

### Autonomous Rail Target Following

To launch the full railroad-following stack in one command (MuJoCo + local heightmap + rail detector + rail target follower + RL twist controller + RViz), use:

```bash
source install/setup.bash
source venv/bin/activate
ros2 launch lite3_sdk_deploy sim_rail_target_follow.launch.py
```

This launch file defaults to the railroad scene with Mid360 enabled, local heightmap enabled, and `procedural_env_seed:=123`.

Useful tuning arguments:
- `follow_distance`: stop distance to keep from the detected target. Default: `1.5`
- `max_linear_x`: max forward speed. Default: `0.35`
- `max_linear_y`: max lateral centering speed. Default: `0.2`
- `max_angular_z`: max yaw-rate command. Default: `0.5`

Example with custom follow distance and speed limits:

```bash
ros2 launch lite3_sdk_deploy sim_rail_target_follow.launch.py \
  follow_distance:=2.0 \
  max_linear_x:=0.25 \
  max_linear_y:=0.15 \
  max_angular_z:=0.4
```

Record a headless follow-camera video while the stack runs:

```bash
ros2 launch lite3_sdk_deploy sim_rail_target_follow.launch.py \
  headless:=true \
  enable_follow_camera:=true \
  follow_camera_video_path:=/tmp/lite3_rail_follow.mp4
```

#### Running tests
```bash
# Run with Artefacts CLI
artefacts run rail_target_follow

# Run with pytest
pytest -s src/Lite3_sdk_deploy/test/test_sim_rail_target_follow_distance.py
```

The recorder is disabled by default and writes MP4 files through the simulation venv's `imageio` and `imageio-ffmpeg` packages.

The follower publishes to `/cmd_vel` and will stop if the rail line is invalid, the target is not detected, or the target is already within the configured follow distance.

### Manual Velocity Commands

After the robot stands and enters RL mode (~5 seconds), send velocity commands:

```bash
# Move forward (0.3 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Strafe left (0.3 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# Turn left (0.3 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

| Twist field    | Robot motion          | Max value |
|----------------|-----------------------|-----------|
| `linear.x`    | Forward / backward    | 0.7 m/s   |
| `linear.y`    | Strafe left / right   | 0.5 m/s   |
| `angular.z`   | Turn left / right     | 0.7 rad/s |

Or use a teleop tool like `teleop_twist_keyboard` for manual control:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Simulation Config

Simulation startup uses the dataclass defaults from `interface/robot/simulation/simulation_config.py` when no config file is provided. You can launch the simulator stack without passing any YAML:

```bash
ros2 launch lite3_sdk_deploy simulation.launch.py
```

For a full reference of all available fields and their default values, see `src/Lite3_sdk_deploy/config/simulations/reference.yaml`.

MuJoCo and the common launchfile modes now use small preset configs under `src/Lite3_sdk_deploy/config/simulations/`. For example:

```bash
ros2 launch lite3_sdk_deploy simulation.launch.py \
  config:=src/Lite3_sdk_deploy/config/simulations/mujoco_rgbd_lidar.yaml
```

The higher-level launchfiles select these presets automatically and expose an optional `simulation_config:=...` override when a custom YAML is needed.

Only `simulation.launch.py` exposes generic `simulation_overrides:=...` for targeted one-off changes without creating a new YAML. Use semicolon-separated dotted paths:

```bash
ros2 launch lite3_sdk_deploy simulation.launch.py \
  config:=src/Lite3_sdk_deploy/config/simulations/mujoco_rgbd_lidar.yaml \
  simulation_overrides:="headless=true;sensors.lidar_2d.visualize_rays=true"
```

Sensor enable flags and sensor tuning live under `sensors` in the YAML file:

```bash
ros2 launch lite3_sdk_deploy simulation.launch.py \
  config:=/absolute/path/to/my_simulation.yaml
```

The default schema includes advanced topic, frame, camera, and mount-site fields with values matching the current simulated interfaces. Most users should only need the enable flags, paths, and rates.

> **Note:** Currently, the color aligned depth topic is not published. In simulation, the two cameras are co-located so there is no need for alignment. We can publish the aligned depth topic to match how we interface with the real robot.

## Adding Custom Scenes

Scenes live alongside the existing terrain files in:
```
src/Lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf/
```

**Step 1 – Add your scene assets**

Copy your scene XML and its `meshes/` folder into that directory:
```
mjcf/
  your_scene.xml
  meshes/
    visual/   ← .obj / texture files
    collision/ ← collision .obj files
```
The XML's `meshdir` and `texturedir` should be relative to its own location, e.g.:
```xml
<compiler meshdir="meshes" texturedir="meshes/visual" .../>
```

**Step 2 – Use unique default class names**

MuJoCo does not allow duplicate `default class` names across merged XMLs. Prefix yours so they don't clash with the Lite3 robot classes (`visual`, `collision`):
```xml
<default>
  <default class="warehouse_visual">
    <geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/>
  </default>
  <default class="warehouse_collision">
    <geom type="mesh" contype="1" conaffinity="1" group="3" condim="3"/>
  </default>
</default>
```
Update every `class="visual"` / `class="collision"` reference in the file to match.

**Step 3 – Use The Scene XML Directly**

Scene XMLs are now environment-only. Do not include `Lite3.xml` in the scene file; the simulator loads the robot separately from `robot_description`. Update or create a simulation YAML that sets `scene` to your custom scene XML path.

Example `my_scene_simulation.yaml`:
```yaml
simulator: mujoco
scene: package://lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf/stairs_floors.xml
```

**Step 4 – Build and run**

```bash
colcon build --packages-select lite3_sdk_deploy
source install/setup.bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py \
  mode:=2 control_type:=0 \
  simulation_config:=/absolute/path/to/my_scene_simulation.yaml
```

where `my_scene_simulation.yaml` contains:

```yaml
scene: your_scene.xml
```


## DDDMR for Lite3 with Mid360 lidar (3D Mapping + 3D Navigation)

### 0. Installation

- Current support: ROS Humble
- Please follow guide at: https://github.com/art-e-fact/dddmr_navigation

### 1. Mapping mode

- Launch:
```
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2_dddrm.launch.py mode:=0
```

- (From another terminal) Save local map file:
```
ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

Note: The map will be saved into `/tmp/2026_xxx`, use this path when update navigation config file.

### 2. Navigation mode

- Modify config file: `src/Lite3_sdk_deploy/config/navigation_mid360s.yaml`:
```
...
sub_maps:
  ros__parameters:
    pose_graph_dir: "/tmp/2026_xxx" 
...
```
- Launch:
```
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2_dddrm.launch.py mode:=1
```
- In Rviz, click on "3D pose estimate" -> set initial pose, then click on "3D goal pose" -> set target goal for navigation

### Tunable config files:

- `src/Lite3_sdk_deploy/config/mapping_mid360s.yaml`
- `src/Lite3_sdk_deploy/config/navigation_mid360s.yaml`


---


## SDK Overview
This repository contains the robotics control SDK, currently supporting Lite3 and M20.

> [!NOTE]
> Damage caused by using SDK is not covered under warranty!

 The repository is structured as follows:
- `src/drdds`: The drdds communication format used by the SDK.
### Lite3
- `src/Lite3_sdk_deploy`: The source code for the Lite3 SDK deploy.
- `src/lite3_sdk_service`: The source code for Lite3 SDK mode switching and frequency modification services.
- `src/lite3_transfer`: The source code for Lite3 UDP-ROS2 message transfer.  

Before using the Lite3 SDK, please refer to the [Lite3 SDK Service Guide](README_lite3_sdk_service.md). For the Lite3 SDK deployment process, please refer to the [Lite3 SDK Deployment Guide](src/Lite3_sdk_deploy/README.md).
### M20
- `src/M20_sdk_deploy`: The source code for the M20 SDK deploy.  

For the M20 SDK deployment process, please refer to the [M20 SDK Deployment Guide](/src/M20_sdk_deploy/README.md).  
## Contributors
See the [Contributors](Contributors.md) page for a list of contributors.

## SDK总览
本仓库包含机器人控制SDK，当前支持Lite3和M20平台。

> [!NOTE]
> 因使用 SDK 造成的设备损坏不在保修范围内！

仓库结构如下：
- `src/drdds`：SDK使用的drdds通信格式。
### Lite3
- `src/Lite3_sdk_deploy`：Lite3 SDK部署的源代码。
- `src/lite3_sdk_service`：Lite3 SDK模式切换、频率修改服务的源代码。
- `src/lite3_transfer`：Lite3 UDP-ROS2消息转换的源代码。

使用Lite3 SDK前请查看[Lite3 SDK服务说明](README_lite3_sdk_service.md)，Lite3 SDK部署流程请查看[Lite3 SDK部署说明](src/Lite3_sdk_deploy/README.md)。
### M20
- `src/M20_sdk_deploy`：M20 SDK部署的源代码。

M20 SDK部署流程请查看[M20 SDK部署说明](/src/M20_sdk_deploy/README.md)。
## 贡献者
请参阅[贡献者](Contributors.md)页面查看贡献者列表。
