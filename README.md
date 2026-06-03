
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
```

### Running tests with Artefacts
```bash
pixi run -e artefacts artefacts run test-sim-sensors
pixi run -e artefacts artefacts run test-rail-follow
```
