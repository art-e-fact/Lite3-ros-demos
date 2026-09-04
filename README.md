
Tested on Linux(x64) and MacOS (arm64).

* The project is currently unsupported on Windows (Due to lack of dependencies available on robostack for Windows).

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

The simulator defaults to **Newton**. `sim-*` tasks auto-detect an NVIDIA GPU (via `scripts/sim_pixi_env.sh`, which runs `nvidia-smi`) and run in the `sim-gpu` pixi environment with the CUDA build of Warp when one is found, or `sim` (CPU Warp) otherwise.

Append `-cpu` to a task name (e.g. `sim-rail-follow-cpu`) to force the CPU `sim` environment even on a GPU machine.

**Lite3:**
```bash
pixi run sim-rail-follow
# or, to force CPU:
pixi run sim-rail-follow-cpu
```
In a separate terminal, run:
```bash
pixi run nav-rail-follow
```
(The follow target only starts moving once the simulator sees this controller's first command, so there's no rush to start it.)

**M20:**
```bash
pixi run sim-rail-follow-m20
# or, to force CPU:
pixi run sim-rail-follow-m20-cpu
```
In a separate terminal:
```bash
pixi run nav-rail-follow-m20
```

MuJoCo is still available as an explicit alternative; see the [MuJoCo](#mujoco) section below for the `-mujoco` task variants.

### Running tests with pytest
```bash
pixi run test-sim-sensors
pixi run test-rail-follow
pixi run test-rerun-recording
pixi run test-rerun-recording-m20   # M20 with Robosense lidar
```
These run against Newton by default and auto-detect an NVIDIA GPU (via `scripts/sim_pixi_env.sh`), launching the Newton simulator in the `sim-gpu` environment when one is found, or `sim` (CPU Warp) otherwise. Each also has a `-mujoco` variant (e.g. `pixi run test-rerun-recording-mujoco`) that runs the same test against MuJoCo instead, and a `-cpu` variant (e.g. `pixi run test-rerun-recording-cpu`) that forces the CPU `sim` environment even on a GPU machine.



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
MuJoCo variants of the integration jobs (`test-rail-follow-mujoco`, `test-rerun-recording-mujoco`, `test-rerun-recording-m20-mujoco`) are also defined in `artefacts.yaml`.

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
Select the robot with
```bash
pixi run rerun-logger --robot m20
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


## MuJoCo

MuJoCo remains available as an explicit alternative to Newton via the `-mujoco` task variants (e.g. `pixi run sim-rail-follow-mujoco`, `pixi run test-rerun-recording-mujoco`).

Notes:
* macOS requires `mjpython` instead of `python` to launch the simulator; the `-mujoco` pixi tasks (and `SimControlHarness`) already handle this automatically on `osx-arm64`.
* The project does run on WSL2, but when rendering MuJoCo is extremely slow (OpenGL is not passed through by Nvidia GPUs to WSL).


## Newton

The same robots also run in Newton, including on the Karuizawa world.

**Lite3:**
```bash
pixi run sim-newton
```
In separate terminals:
```bash
pixi run -e nav ros2 run lite3_sdk_deploy rl_deploy --twist
pixi run -e nav ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**M20:**
```bash
pixi run sim-newton-m20
```
In separate terminals:
```bash
pixi run -e nav ros2 run m20_sdk_deploy rl_deploy --twist
pixi run -e nav ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To use the Karuizawa world:
```bash
# part1, part2, part3 are available
pixi run sim-newton-m20-karuizawa part1
```
Large assets are loaded on-demand. See [docs/remote_assets.md](docs/remote_assets.md) for more details.

Notes:
* GPU acceleration uses the `sim-gpu` environment (see `[feature.sim-cuda]` in `pixi.toml`); it requires an NVIDIA driver with CUDA 12. `sim-*` and `test-*` tasks auto-detect it via `scripts/sim_pixi_env.sh` (`nvidia-smi`); append `-cpu` to a task name to force CPU.
* On the default CPU (`sim`) environment, Newton is significantly slower than real-time; performance is limited until GPU acceleration is used.
