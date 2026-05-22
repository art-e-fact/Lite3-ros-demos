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

### Python venv (for simulation only)

The MuJoCo simulation requires Python packages not available via rosdep. Set up a venv once:

```bash
python3 -m venv venv
touch venv/COLCON_IGNORE
source venv/bin/activate
pip install -r requirements.txt
```

Activate the venv before running simulation commands.


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

By default, simulation uses the authored static scene. To generate the environment procedurally at runtime with the full launch stack, use:

```bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py \
  mode:=2 control_type:=0 use_procedural_scene:=true
```

Optional seed for reproducible layouts:

```bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py \
  mode:=2 control_type:=0 use_procedural_scene:=true procedural_env_seed:=1234
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


## Simulated Sensors

Sensor modules live in `src/Lite3_sdk_deploy/interface/robot/simulation/`:

| Sensor | File | Key topics |
|--------|------|------------|
| 2D LiDAR (RPLiDAR A2M8) | `lidar_sensor.py` | `/scan` |
| Depth camera (RealSense D435i) | `depth_sensor.py` | `/camera/depth/image_rect_raw`, `/camera/color/image_raw`, `/camera/depth/color/points` |

Each module has **enable/disable flags** at the top of the file to skip expensive computation:

```python
# lidar_sensor.py
ENABLE_LIDAR = True

# depth_sensor.py
ENABLE_DEPTH = True
ENABLE_COLOR = False
ENABLE_POINTCLOUD = False  # requires ENABLE_DEPTH
```

Resolution (`WIDTH`/`HEIGHT`) and publish rate (`*_FREQUENCY_HZ`) are also configurable there.

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

**Step 3 – Create a top-level entry XML**

Create a new `Lite3_<yourscene>.xml` in `src/Lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf`:
```xml
<mujoco model="Lite3_yourscene">
    <include file="./Lite3.xml"/>
    <include file="./your_scene.xml"/>
</mujoco>
```

**Step 4 – Build and run**

```bash
colcon build --packages-select lite3_sdk_deploy
source install/setup.bash
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2.launch.py \
  mode:=2 control_type:=0 \
  xml:=Lite3_yourscene.xml
```


## DDDMR for Lite3 with Mid360 lidar (3D Mapping + 3D Navigation)

### 0. Installation

- Current support: ROS Humble
- Please follow guide at: https://github.com/art-e-fact/dddmr_navigation

### 1. Mapping mode

- Launch:
```
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2_dddrm.launch.py mode:=0 xml:=stairs_floors.xml
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
ros2 launch lite3_sdk_deploy mujoco_simulation_ros2_dddrm.launch.py mode:=1 xml:=stairs_floors.xml
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


