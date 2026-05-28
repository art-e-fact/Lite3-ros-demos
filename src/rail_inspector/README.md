# rail_inspector

This package contains rail-specific perception and navigation nodes that consume a
local height map and publish rail-following control signals.

## rail_detector_node

The detector consumes a `grid_map_msgs/GridMap` elevation map and odometry. It samples
multiple lateral slices around the robot, pairs rail-like height groups using the
configured track gauge, fits a short rail centerline, and publishes RViz markers plus
scalar outputs for the follower.

### Parameters

- `heightmap_topic` (`/local_heightmap`): input `grid_map_msgs/GridMap`
- `odom_topic` (`/odom`): input `nav_msgs/Odometry` used for robot position and heading
- `marker_topic` (`/rail_detector/markers`): output `visualization_msgs/MarkerArray`
- `center_offset_topic` (`/rail_detector/center_offset`): output signed rail-center offset in meters
- `tangent_yaw_topic` (`/rail_detector/tangent_yaw`): output detected rail tangent yaw in radians
- `target_distance_topic` (`/rail_detector/target_distance`): output target distance in meters, or `-1.0` when invalid
- `track_gauge` (`1.067`): expected distance between rails in meters
- `rail_width` (`0.15`): expected lateral width of one rail in meters
- `gauge_tolerance` (`0.40`): maximum rail-pair gauge error allowed in one slice
- `angle_sweep_deg` (`40.0`): half-range of the center-slice heading search in degrees
- `angle_step_deg` (`5.0`): heading increment used during the center-slice search
- `min_rail_height` (`0.05`): minimum height above the local baseline to accept a rail hit
- `max_rail_height` (`0.30`): maximum height above the local baseline to accept a rail hit
- `max_rail_height_difference` (`0.08`): maximum height mismatch between left and right rails
- `forward_span` (`2.6`): total span covered by the sampled rail slices
- `num_slices` (`15`): number of cross-sections sampled across the forward span
- `lateral_search_width` (`1.8`): half-width of each sampled cross-section in meters
- `follow_target_lookahead` (`8.0`): forward distance checked for a follow target
- `follow_target_kernel_size` (`0.35`): width of the center sample window used to measure the target
- `follow_target_sample_step` (`0.10`): distance between follow-target samples
- `follow_target_min_height` (`0.10`): minimum rise above the detected rail height to count as a target
- `follow_target_max_height` (`2.2`): maximum plausible rise above the detected rail height

### Example

```bash
ros2 run rail_inspector rail_detector_node --ros-args \
  -p heightmap_topic:=/local_heightmap \
  -p odom_topic:=/odom \
  -p marker_topic:=/rail_detector/markers
```

## rail_target_follower_node

The follower consumes detector outputs and odometry, then publishes
`geometry_msgs/Twist` commands that keep the robot near the rail center while stopping
at a configured distance from the detected follow target.

If the rail line becomes invalid, the target disappears, or any input becomes stale,
the follower publishes a zero `Twist`.

### Parameters

- `cmd_vel_topic` (`/cmd_vel`): output `geometry_msgs/Twist`
- `odom_topic` (`/odom`): input `nav_msgs/Odometry` used for robot yaw and body-frame conversion
- `center_offset_topic` (`/rail_detector/center_offset`): input signed rail-center offset topic
- `tangent_yaw_topic` (`/rail_detector/tangent_yaw`): input rail tangent yaw topic
- `target_distance_topic` (`/rail_detector/target_distance`): input follow-target distance topic
- `control_rate_hz` (`15.0`): control loop rate used to publish `cmd_vel`
- `stale_timeout_sec` (`0.5`): maximum wall-time age accepted for detector and odometry inputs
- `follow_distance` (`1.5`): desired stop distance to the follow target
- `target_distance_deadband` (`0.1`): extra no-motion margin beyond `follow_distance`
- `min_linear_x` (`0.4`): minimum forward command that reliably starts locomotion
- `max_linear_x` (`0.55`): maximum forward body-frame speed command
- `distance_error_for_max_speed` (`1.5`): distance error where forward speed reaches `max_linear_x`
- `max_linear_y` (`0.4`): maximum lateral centering speed command
- `max_angular_z` (`0.5`): maximum yaw-rate command
- `k_center` (`1.0`): gain that converts rail-center offset into lateral correction speed
- `k_heading` (`1.2`): gain that converts rail tangent yaw error into angular speed

### Example

```bash
ros2 run rail_inspector rail_target_follower_node --ros-args \
  -p center_offset_topic:=/rail_detector/center_offset \
  -p tangent_yaw_topic:=/rail_detector/tangent_yaw \
  -p target_distance_topic:=/rail_detector/target_distance \
  -p follow_distance:=1.5 \
  -p min_linear_x:=0.4 \
  -p max_linear_x:=0.55 \
  -p distance_error_for_max_speed:=1.5
```