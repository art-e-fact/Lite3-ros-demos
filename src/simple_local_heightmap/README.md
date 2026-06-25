# simple_local_heightmap

This package contains `local_heightmap_node`, which builds a local
`grid_map_msgs/GridMap` elevation map for RViz debugging and downstream consumers.

Rail-specific detector and follower nodes live in the `rail_inspector` package.

## local_heightmap_node

The local heightmap keeps one elevation value per grid cell and ages out cells using the
time since they were last observed. The grid recenters on the robot as it moves. Each scan
uses the median height per cell. Point clouds are transformed into `map_frame` via TF;
range filtering uses the cloud's sensor frame. To clear transient hits faster in front of
the robot, the node can apply a shorter timeout inside a rectangle defined in the robot
frame. Optional visibility cleanup can remove ghost cells along the line of sight
from the sensor to each observed cell in the current scan.

### Subscribed topics

| Topic | Type | Description |
|-------|------|-------------|
| `cloud_topic` (default `/mid360/points`) | `sensor_msgs/PointCloud2` | Input point cloud |
| `odom_topic` (default `/odom`) | `nav_msgs/Odometry` | Used for pose-covariance gating |

### Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `heightmap_topic` (default `/local_heightmap`) | `grid_map_msgs/GridMap` | Elevation map output |
| `/local_heightmap/front_clear_markers` | `visualization_msgs/MarkerArray` | Fast-clear rectangle (only when `front_clear_enabled` is true) |
| `/perf/height_scan` | `std_msgs/Float32` | Per-scan processing time in milliseconds |

### Parameters

#### Input / output

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cloud_topic` | `/mid360/points` | Input `PointCloud2` topic |
| `odom_topic` | `/odom` | Odometry topic for pose-covariance gating |
| `heightmap_topic` | `/local_heightmap` | Output `GridMap` topic |

#### Coordinate frames

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_frame` | `odom` | Fixed frame for the output map |
| `robot_frame` | `base_link` | Robot body frame (used for grid centering and fast-clear) |

#### Map geometry

| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | `0.05` | Cell size in metres |
| `length_x` | `3.0` | Map extent along X (metres); snapped to whole cells |
| `length_y` | `3.0` | Map extent along Y (metres); snapped to whole cells |

#### Point-cloud filtering

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_z` | `-1.0` | Reject points below this height (metres, map frame) |
| `max_z` | `2.0` | Reject points above this height (metres, map frame) |
| `min_range` | `0.1` | Reject points closer than this range (metres, sensor frame) |
| `max_range` | `12.0` | Reject points farther than this range (metres, sensor frame) |

#### Cell ageing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stale_time_sec` | `100.0` | Seconds before a cell is expired outside the fast-clear zone; `0` disables |
| `max_pose_variance` | `0.0` | Drop scans when the maximum diagonal covariance from odometry exceeds this value; `0` disables |

#### Front fast-clear rectangle

A robot-frame rectangle immediately ahead of the robot can be given a much shorter
expiry so transient obstacles (e.g. the robot's own legs) are cleared quickly.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `front_clear_enabled` | `false` | Enable the fast-clear zone |
| `front_clear_length` | `2.5` | Length of the rectangle (metres, robot X axis) |
| `front_clear_width` | `1.0` | Width of the rectangle (metres, robot Y axis) |
| `front_clear_offset_x` | `0.75` | Forward offset from the robot origin to the near edge of the rectangle (metres) |
| `front_stale_time_sec` | `0.35` | Expiry timeout inside the fast-clear rectangle |

#### Visibility cleanup

Ray-based cleanup runs after rasterizing each scan and before fusing it into the map.
For every observed cell in the current scan, a 2D line is traced from the sensor to
that cell; older map cells above the ray (not touched by this scan) are invalidated.
This clears phantom obstacles once the sensor can see past them. Rays start at the
cloud/sensor frame origin (`cloud_topic` header frame), not `robot_frame`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `visibility_cleanup_enabled` | `false` | Enable line-of-sight cleanup |
| `visibility_cleanup_tolerance` | `0.05` | Keep map cells whose height is within this margin above the ray (metres) |
