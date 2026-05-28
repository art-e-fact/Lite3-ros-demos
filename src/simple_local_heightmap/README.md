# simple_local_heightmap

This package contains `local_heightmap_node`, which builds a local
`grid_map_msgs/GridMap` elevation map for RViz debugging and downstream consumers.

Rail-specific detector and follower nodes live in the `rail_inspector` package.

## local_heightmap_node

The local heightmap keeps one elevation value per grid cell and ages out cells using the
time since they were last observed. To clear transient hits faster in front of the robot,
the node can apply a shorter timeout inside a rectangle defined in the robot frame.

### Parameters

- `cloud_topic` (`/mid360/points`): `sensor_msgs/PointCloud2` topic used as the heightmap input cloud
- `odom_topic` (`/odom`): `nav_msgs/Odometry` topic used for pose covariance gating
- `stale_time_sec` (`1.0`): default timeout used outside the front fast-clear rectangle
- `front_clear_enabled` (`false`): enables a robot-frame rectangle with a shorter timeout
- `front_clear_length` (`3.5`): length of the fast-clear rectangle in meters
- `front_clear_width` (`1.0`): width of the fast-clear rectangle in meters
- `front_clear_offset_x` (`0.25`): forward offset from the robot origin to the start of the rectangle
- `front_stale_time_sec` (`0.75`): timeout used for cells inside the fast-clear rectangle

The front-clear overlay is published on `/local_heightmap/front_clear_markers` as a
`visualization_msgs/MarkerArray` for RViz debugging.