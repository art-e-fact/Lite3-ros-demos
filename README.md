
Tested on Linux(x64) and MacOS (arm64).

* The project is currently unsupported on Windows (Due to lack of dependencies available on robostack for Windows).
* The project does run on WSL2, but when rendering Mujoco is extremely slow (OpenGL is not passed through by Nvidia GPUs to WSL).

## Requirements

* Either Linux (Tested Ubuntu24.04) or MacOS (Tested MacOS 15)
* [pixi](https://pixi.prefix.dev/latest/installation/)


### Build the project
```bash
pixi run build
```
Note: All the ROS environments are build under the `rosbuild` folder. You can delete the folder if you need a clean build.

### Run rail-following demo
_Note: The simulation and the robot controller are requiring different ROS 2 versions, so we need to start them as separate processes._

**Lite3:**
```bash
pixi run sim-rail-follow
```
In a separate terminal, run before the target gets too far in the simulation:
```bash
pixi run nav-rail-follow
```

**M20:**
```bash
pixi run sim-rail-follow-m20
```
In a separate terminal:
```bash
pixi run nav-rail-follow-m20
```

### Running tests with pytest
```bash
pixi run test-sim-sensors
pixi run test-rail-follow
pixi run test-rerun-recording
pixi run test-rerun-recording-m20   # M20 with Robosense lidar
```

Pass `--robot m20` (or `--robot lite3`) to `test_rerun_recording.py` directly for other pytest invocations.


### Running tests with Artefacts


To avoid dependency conflicts `pixi` will setup an artefacts environment for you. `artefacts` commands can be ran by prefacing with:
```
pixi run -e artefacts artefacts ...
```

_Alternatively, you may source the artefacts environment with `pixi shell -e artefacts`. Then you may use artefacts commands with out `pixi run -e artefacts`, e.g. `artefacts run test-rail-follow`_

#### Creating a new project
1. In the [Dashboard](https://app.artefacts.com) create a new project.
2. Rename L1 of `artefacts.yaml` with the project you just created in the format `myorg/myproject`
3. Login to artefacts with `pixi run -e artefacts artefacts login` (if not already)

#### Joining an existing project
1. Ask to be invited to the project (if not already) as "developer" or "administrator"
2. Rename L1 of `artefacts.yaml` in the format `myorg/myproject`
3. Login to artefacts with `pixi run -e artefacts artefacts login` (if not already)

Then run the tests with 
```bash
pixi run -e artefacts artefacts run test-rail-follow
pixi run -e artefacts artefacts run test-rerun-recording
pixi run -e artefacts artefacts run test-sim-sensors
```

## Logging to Rerun
To start logging the sensor, tf, and joint state data to Rerun:
```bash
pixi run rerun-logger
```
Or without spawning the viewer window:
```bash
pixi run rerun-logger --headless
```
To visualize heightmaps
```bash
pixi run rerun-logger --log_heightmap --use_static_heightmap
```

## TUI Interface
To launch the terminal user interface run:
```
pixi run rail-follow-tui
```
The `Control` tab gives access to the main follow settings while the `Parameters` tab allows dynamically updating node parameters.

The state of the UI is kept in the ROS parameter server so it's safe to relaunch it or run multiple instances simultaneously.

Notes:
 - To sync the UI when nodes restart, click the `refresh` button
