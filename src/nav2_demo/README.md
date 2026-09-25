# nav2_demo

A route, driven by Nav2 on the Lite3 quadruped, in a world made from the
route's map. Uses
[artefacts-toolkit-navigation](https://github.com/art-e-fact/artefacts-toolkit-navigation):

- `artefacts-route record` records a route by clicking in RViz on a map.
  `routes/sandbox.yaml` was recorded that way on Nav2's TurtleBot3 sandbox
  map; `routes/depot.yaml`, the default here, was written by hand on Nav2's
  depot map (the sandbox's 1.1 m pillar grid is a tight fit for a 0.56 m
  quadruped; the depot has an open floor).
- `map_to_world()` raises the map's occupied cells into walls, giving a
  simulator world at the map's coordinates (`format="mjcf"` for this
  simulator). It takes milliseconds, so the test makes it fresh each run.
- `follow_route()` sends the route to Nav2 and reports whether it was
  completed.

## Run it

```bash
pixi run test-nav2-route --headless  # Newton (GPU if there is one), no windows
pixi run test-nav2-route             # with the Newton viewer
RVIZ=true pixi run test-nav2-route   # ... and Nav2's RViz view (map, costmaps, path, robot)
pixi run test-nav2-route-mujoco      # the same on MuJoCo
ROUTE=src/nav2_demo/routes/sandbox.yaml pixi run test-nav2-route   # a particular route
```

The test drives the route recorded last (`artefacts-route record` saves into
`routes/`), or the bundled depot route when none has been recorded.

## Record your own route

From the repository root, on one of the maps here (or your own):

```bash
pixi run -e nav artefacts-route record --name lab --map src/nav2_demo/maps/depot.yaml
```

This opens RViz with the map. Click **2D Pose Estimate** where the robot
should start, then **Publish Point** for each waypoint in order; press enter
in the terminal (or close RViz) to save `routes/lab.yaml`. Keep the points on
open floor: the Lite3 is 0.56 x 0.30 m and walks at 0.3 m/s. Then
`pixi run test-nav2-route`.

## Pieces

- `launch/nav2_route.launch.py`: rl_deploy and Nav2's `bringup_launch.py` on
  the route's map with `config/nav2_lite3.yaml`. Nav2 is started a few
  seconds after rl_deploy (`nav2_delay`), once the robot is standing, so the
  lidar has stopped sweeping the ground.
- `config/nav2_lite3.yaml`: Nav2's default parameters with the Lite3's frames,
  footprint and speeds, and Regulated Pure Pursuit as the controller; the
  header lists every change.
- `test/test_route.py`: the pytest. It extrudes the world from the route's
  map, starts the simulator (Newton, the Lite3 with its 2D lidar) with
  `--set robot.start_pose.*` from the route, launches the nav side, and
  asserts on `follow_route()`'s result. The world, the simulator config and
  the route result are written to `results/` (or Artefacts' upload directory).
