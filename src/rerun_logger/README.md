# rerun_logger

A single ROS 2 node that subscribes to topics from the Lite3 rail demo and
forwards them to [Rerun](https://rerun.io) for live 3-D visualisation.

## Node: `rerun_logger`

### Subscribed topics

| Topic | Type | Description |
|-------|------|-------------|
| `/tf` | `tf2_msgs/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms (latching) |
| `/mid360/points` | `sensor_msgs/PointCloud2` | LiDAR point cloud |
| `/JOINTS_DATA` | `drdds/JointsData` | Joint positions for URDF animation |
| `/rail_detector/markers` | `visualization_msgs/MarkerArray` | Rail detector debug markers |
| `/local_heightmap` | `grid_map_msgs/GridMap` | Local elevation map *(only when `--log_heightmap` is set)* |

### CLI flags

All standard [Rerun script flags](https://rerun.io/docs/getting-started/data-in/rerun-sdk/logging#connecting-to-the-viewer)
(e.g. `--connect`, `--save`) are supported in addition to:

| Flag | Default | Description |
|------|---------|-------------|
| `--log_heightmap` | off | Subscribe to `/local_heightmap` and visualise it as `Boxes3D` |
| `--use_static_heightmap` | off | Log the heightmap as a static (non-timeline) entity to keep recording sizes small; only meaningful together with `--log_heightmap` |

### Usage

```bash
# Live viewer — basic
ros2 run rerun_logger rerun_logger -- --connect

# With heightmap visualisation
ros2 run rerun_logger rerun_logger -- --connect --log_heightmap

# With heightmap, but don't record it to the timeline
ros2 run rerun_logger rerun_logger -- --connect --log_heightmap --use_static_heightmap

# Save a recording (heightmap excluded by default)
ros2 run rerun_logger rerun_logger -- --save my_recording.rrd
```
