
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
```bash
pixi run sim-rail-follow
```
In a separate terminal, run before the target gets too far in the simulation:
```bash
pixi run nav-rail-follow
```


### Running tests with pytest
```bash
pixi run test-sim-sensors
pixi run test-rail-follow
pixi run test-rerun-recording
```

### Running tests with Artefacts
```bash
pixi run -e artefacts artefacts run test-sim-sensors
pixi run -e artefacts artefacts run test-rail-follow
pixi run -e artefacts artefacts run test-rerun-recording
```

