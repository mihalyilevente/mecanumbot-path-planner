# mecanumbot_path_planner

C++17 `ament_cmake` global planner plugin for **ROS 2 Humble / Nav2**.
This is a development skeleton: every active planning request throws
`nav2_core::PlannerException` with an explicit "not implemented" message.
It does not produce paths, publish velocity commands, or implement A*.

## Interface and configuration

`mecanumbot_path_planner::AStarPlanner` implements `configure`, `cleanup`,
`activate`, `deactivate`, and Humble's two-argument `createPlan(start, goal)`.
The pluginlib lookup name is `mecanumbot_path_planner/AStarPlanner`.

`<planner_id>.optimization_mode` accepts `shortest` (default), `safest`, or
`smoothest`. Invalid values fail configuration. These are placeholders without
cost functions; parameters are read at configuration time. To change a mode,
deactivate and clean up Planner Server, set its parameter, then configure and
activate again. Dynamic mode switching is not implemented.

`config/planner_params.yaml` is a **Planner Server fragment**, not a full Nav2
bringup configuration. It retains Mecanumbot's existing `GridBased` planner ID.
For later integration, merge this fragment into a separate copy of the robot's
complete Nav2 parameters, retaining its global costmap, footprint, frames,
transforms, controller, and other server settings. Source this package's overlay
before starting Planner Server. Do not replace the complete robot parameter
file with this fragment. No robot configuration has been changed.

## Build and test

From the repository root in Ubuntu 22.04:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mecanumbot_path_planner
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=91 colcon test \
  --packages-select mecanumbot_path_planner --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg prefix mecanumbot_path_planner
cat install/mecanumbot_path_planner/share/mecanumbot_path_planner/plugin.xml
cat install/mecanumbot_path_planner/share/ament_index/resource_index/nav2_core__pluginlib__plugin/mecanumbot_path_planner
```

The smoke tests load the installed shared library via pluginlib (without linking
to it), exercise configuration/lifecycle and the three placeholders, and check
that unimplemented planning fails explicitly. They do not launch navigation or
validate a pathfinding algorithm. Rebuild and source the overlay before testing.

On a fresh Ubuntu 22.04 system with the official ROS 2 Humble apt repository
already configured, dependencies can be installed with:

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions \
  ros-humble-ament-cmake ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-ament-cmake-gtest
```

The package can also be cloned as a separate repository under another colcon
workspace's `src/`; build from that workspace root. No dependency on robot-specific
packages is needed to build this plugin.

## Files

| File | Purpose |
| --- | --- |
| `include/mecanumbot_path_planner/a_star_planner.hpp` | Humble interface and lifecycle state |
| `src/a_star_planner.cpp` | Lifecycle, mode validation, explicit unimplemented response, registration |
| `config/planner_params.yaml` | Future Planner Server integration fragment |
| `plugin.xml` | Plugin lookup name, C++ type, base class and shared library |
| `CMakeLists.txt` | C++17 build, installation, ament/pluginlib exports and smoke tests |
| `package.xml` | ROS package metadata and dependencies |
| `test/test_plugin_loading.cpp` | Installed-plugin and lifecycle smoke tests |

## Next development step

Implement and test shortest-path grid A* first: world/map coordinate conversion,
start/goal bounds, consistent costmap access, occupied and unknown cell handling,
8-connected movement with diagonal corner checks, admissible heuristic, path
reconstruction, and correct path frames/stamps. Define criterion costs and metrics
before adding safest/smoothest modes. Mecanum motion permits lateral translation;
avoid adding a car-like nonholonomic constraint by assumption.

## References inspected before implementation

- [Humble GlobalPlanner API](https://github.com/ros-navigation/navigation2/blob/humble/nav2_core/include/nav2_core/global_planner.hpp)
- [Mecanumbot documentation](https://fortuz.github.io/docs/mecanumbot_main/)
- [Robot CMake convention](https://github.com/Fortuz/mecanumbot/blob/5818369f8571942a3b21b3ed4861d5dae3752ab0/mecanumbot_bringup/CMakeLists.txt)
- [Existing Planner Server configuration](https://github.com/Fortuz/mecanumbot/blob/5818369f8571942a3b21b3ed4861d5dae3752ab0/mecanumbot_description/param/mecanumbot_custom_nav2.yaml)
- [Robot Nav2 launch](https://github.com/Fortuz/mecanumbot/blob/5818369f8571942a3b21b3ed4861d5dae3752ab0/mecanumbot_bringup/launch/nav2.launch.py)
- [Current simulator](https://github.com/Fortuz/mecanumbot_remote/blob/0e696edd96f2c43a93bfd0168cfb13a73ca89009/mecanumbot_sim/README.md)

The live repositories place `mecanumbot_sim` in `mecanumbot_remote`; the project
overview still describes its older location. The simulator's Nav2 shim does not
load global planner plugins, so future integration testing needs real Planner
Server and costmaps.

License metadata is `Proprietary` pending the owner's choice of a distribution
license; no third-party source files were copied into this repository.
