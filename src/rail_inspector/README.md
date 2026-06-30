# rail_inspector

This package contains rail-specific perception and navigation nodes that consume a
local height map and publish rail-following control signals.

## rail_detector_node

The detector consumes a `grid_map_msgs/GridMap` elevation map and odometry. It samples
multiple lateral slices around the robot, pairs rail-like height groups using the
configured track gauge, fits a short rail centerline, and always publishes RViz
`MarkerArray` debug markers as well as scalar outputs for the follower. When no rail
line is found, `center_offset` and `tangent_yaw` are published as `NaN`; `target_distance`
is `-1.0`. Rerun visualization is handled by the separate `rerun_logger` node, which
subscribes to `/rail_detector/markers` and converts them into Rerun entities.

### Parameters

- `heightmap_topic` (`/local_heightmap`): input `grid_map_msgs/GridMap`
- `odom_topic` (`/odom`): input `nav_msgs/Odometry` used for robot position and heading
- `marker_topic` (`/rail_detector/markers`): output `visualization_msgs/MarkerArray`
- `center_offset_topic` (`/rail_detector/center_offset`): output `std_msgs/Float32` signed rail-center offset in meters, or `NaN` when invalid
- `tangent_yaw_topic` (`/rail_detector/tangent_yaw`): output `std_msgs/Float32` detected rail tangent yaw in radians, or `NaN` when invalid
- `target_distance_topic` (`/rail_detector/target_distance`): output `std_msgs/Float32` target distance in meters, or `-1.0` when invalid
- `track_gauge` (`1.067`): expected distance between rails in meters
- `rail_width` (`0.15`): expected lateral width of one rail in meters
- `gauge_tolerance` (`0.40`): maximum rail-pair gauge error allowed in one slice
- `angle_sweep_deg` (`40.0`): half-range of the center-slice heading search in degrees
- `angle_step_deg` (`5.0`): heading increment used during the center-slice search
- `min_rail_height` (`0.05`): minimum height above the baseline (sleeper elevation) to accept a rail hit
- `max_rail_height` (`0.30`): maximum height above the baseline (sleeper elevation) to accept a rail hit
- `max_rail_height_difference` (`0.08`): maximum height mismatch between left and right rails
- `baseline_auto` (`true`): estimate ground height per slice from the height scan
- `baseline_z` (`0.0`): fixed ground baseline Z in the **odom** frame (same as `/local_heightmap`); used when `baseline_auto` is `false`. `min_rail_height` and `max_rail_height` are offsets above this baseline.
- `forward_span` (`2.6`): forward distance ahead of the robot covered by the sampled rail slices
- `backward_span` (`0.0`): backward distance behind the robot covered by the sampled rail slices
- `num_slices` (`15`): number of cross-sections sampled from `backward_span` behind to `forward_span` ahead
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
`geometry_msgs/Twist` commands that keep the robot aligned and centered on the rail.

It supports two control modes, configurable via the `follow_mode` parameter:
- **`auto`**: The robot automatically follows the rail forward and stops at a configured distance from the detected follow target. If the rail line becomes invalid, the target disappears, or any input becomes stale, the follower publishes a zero `Twist`.
- **`teleop`**: The robot aligns and centers itself on the rail, but its forward/backward speed along the rail is controlled by publishing to the `follow_rail_speed` topic. A valid follow target is not required. If the teleop speed input, the rail line, or any other input becomes stale, the follower publishes a zero `Twist`.

### Parameters

- `cmd_vel_topic` (`/cmd_vel`): output `geometry_msgs/Twist`
- `odom_topic` (`/odom`): input `nav_msgs/Odometry` used for robot yaw and body-frame conversion
- `center_offset_topic` (`/rail_detector/center_offset`): input `std_msgs/Float32` signed rail-center offset
- `tangent_yaw_topic` (`/rail_detector/tangent_yaw`): input `std_msgs/Float32` rail tangent yaw
- `target_distance_topic` (`/rail_detector/target_distance`): input `std_msgs/Float32` follow-target distance; negative means invalid
- `follow_mode` (`"auto"`): control mode, `"auto"` (follows target automatically) or `"teleop"` (uses `follow_rail_speed`)
- `follow_rail_speed_topic` (`/follow_rail_speed`): input `geometry_msgs/Twist` topic for teleop forward speed command (only `linear.x` is read)
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

### Examples

#### Automatic Mode (Default)

```bash
ros2 run rail_inspector rail_target_follower_node --ros-args \
  -p follow_mode:=auto \
  -p center_offset_topic:=/rail_detector/center_offset \
  -p tangent_yaw_topic:=/rail_detector/tangent_yaw \
  -p target_distance_topic:=/rail_detector/target_distance \
  -p follow_distance:=1.5 \
  -p min_linear_x:=0.4 \
  -p max_linear_x:=0.55 \
  -p distance_error_for_max_speed:=1.5
```

#### Teleop Mode

```bash
ros2 run rail_inspector rail_target_follower_node --ros-args \
  -p follow_mode:=teleop \
  -p center_offset_topic:=/rail_detector/center_offset \
  -p tangent_yaw_topic:=/rail_detector/tangent_yaw \
  -p follow_rail_speed_topic:=/follow_rail_speed \
  -p max_linear_x:=0.55
```