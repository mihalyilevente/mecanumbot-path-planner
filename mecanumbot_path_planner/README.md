# mecanumbot_path_planner

C++17 `ament_cmake` global planner plugin for **ROS 2 Humble / Nav2**.
All four modes implement grid A*. Each returns a `nav_msgs::msg::Path` or
throws `nav2_core::PlannerException` if the request is invalid or no route exists.
Their objectives are defined below. The plugin does not publish velocity commands.

## Interface and configuration

`mecanumbot_path_planner::AStarPlanner` implements `configure`, `cleanup`,
`activate`, `deactivate`, and Humble's two-argument `createPlan(start, goal)`.
The pluginlib lookup name is `mecanumbot_path_planner/AStarPlanner`.

`<planner_id>.optimization_mode` accepts `shortest` (default), `safest`,
`smoothest`, or `combined`. Invalid values fail configuration. `<planner_id>.allow_unknown`
defaults to `false`. Parameters are read at configuration time. To change them,
deactivate and clean up Planner Server, set its parameter, then configure and
activate again. Dynamic parameter updates are not implemented.

## Criteria definitions

These are explicit engineering objectives, not universal definitions of safety
or smoothness. Let `L` be planar length in meters, `E` costmap exposure, and `T`
the sum of squared changes in travel direction in radians squared:

| Mode | Minimized objective | Default weight |
| --- | --- | --- |
| `shortest` | `L` | None |
| `safest` | `L + safety_weight * E` | `2.0` (dimensionless) |
| `smoothest` | `L + smoothness_weight * T` | `0.5` meters/radian squared |
| `combined` | `L + safety_weight * E + smoothness_weight * T` | `2.0`, `0.5` respectively |

For each grid edge of length `d`, exposure is `d * (r(a) + r(b)) / 2`, where
`r = cost / 252` for traversable known cells. Allowed unknown cells have `r = 1`,
not zero. Lethal and inscribed cells remain forbidden in every mode, regardless
of weights. A costmap with only free and lethal cells provides no clearance
gradient, so safest then behaves like shortest in known free space. Configure
an appropriate inflation layer before interpreting this as obstacle clearance:
[Humble inflation cost computation](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp).
Exposure is a cost proxy, not collision probability or a maximum-minimum
clearance guarantee; other costmap layers may contribute to it too.

For smoothest, each turn is the wrapped angle in `[-pi, pi]` between consecutive
nonzero translation segments. Squaring penalizes sharp corners more strongly.
The search state includes the incoming direction (eight states per cell).
It also accounts for turns at the exact start/goal connectors, using a virtual
terminal state so the first goal-cell arrival is not prematurely accepted.
Body yaw is not part of this objective: a mecanum robot can translate laterally.
Requested endpoint orientations are still preserved in the output.

The objectives apply to cell-center routes with fixed endpoint connectors.
Connector length/exposure is constant across these candidate routes and need
not be added during search; connector turns do affect smoothest. Same-cell
requests retain the direct-segment behavior. The distance-only octile heuristic
is admissible because all additional penalties are nonnegative.

Weights must be finite and nonnegative. In the individual modes, zero restores
shortest-path behavior for that mode's extra term. Combined mode always counts
path length and exposes safety and smoothness weights. Zero disables that extra
criterion; setting both weights to zero runs the shortest planner exactly.
Increasing a weight permits longer detours to reduce its penalty. Defaults are
starting values, not tuned Mecanumbot parameters. Smoothest remains a polyline:
it does not provide continuous curvature, acceleration, or jerk limits, and
large turn weights can favor long detours or revisiting a cell with a different
arrival direction. Safest does not optimize turns; smoothest does not penalize
traversable inflation costs. All modes use the same hard collision rules.

## Shortest-path behavior

- Search uses an 8-connected cell-center graph, unit cardinal costs and sqrt(2)
  diagonal costs, and an admissible octile heuristic. Equal-cost alternatives
  use deterministic ordering. `shortest` means minimum geometric graph length;
  it is not a continuous-space shortest-path or smoothness guarantee.
- Cells with costs 253 (inscribed) and 254 (lethal) are blocked. Unknown cells
  (255) are blocked unless `allow_unknown: true`. Costs 0 through 252 are
  traversable and do not add a safety penalty in this mode.
- Diagonal moves require both adjacent cardinal cells to be traversable.
- Start and goal must already be in the costmap's global frame, as normally
  supplied by Planner Server. Mismatched frames, nonfinite coordinates, zero
  quaternions, out-of-bounds endpoints, and blocked endpoints fail explicitly.
  Valid nonunit quaternions are normalized. No cell is cleared to force a route.
- The live costmap is copied under its mutex. Search uses the immutable copy,
  releasing the live map before search and leaving its data unchanged.
- The path preserves exact start and goal positions and their headings, adding
  cell centers between them. Endpoint-to-center connectors stay within their
  respective cells. Intermediate headings face the next segment. All headers
  share the costmap frame and planning timestamp. Same-cell requests use a
  direct segment; identical positions return one pose with the goal heading.
- This is a planar, holonomic search. It relies on a correctly inflated costmap;
  it does not perform orientation-dependent polygon footprint checks, dynamic
  obstacle prediction, continuous curve smoothing, or velocity planning. Shortest optimizes graph
  length with fixed endpoint connectors rather than heading changes.

The costmap snapshot may become stale during execution; normal Nav2 replanning
and controller collision checks remain necessary. No full robot or simulation
navigation validation has been performed yet.

## Code flow

The implementation follows one short pipeline:

1. `AStarPlanner` reads parameters and copies the live Nav2 costmap safely.
2. `GridProblem` validates inputs and provides neighbors, risks, and geometry.
3. `grid_a_star.cpp` selects shortest, safest, smoothest, or combined.
4. The selected criterion file runs A* using the shared queue utilities.
5. `path_builder.cpp` converts the selected cells into `nav_msgs::msg::Path`.

Shortest and safest share the ordinary cell-state search and supply different
edge-cost functions. Smoothest uses direction-aware search because turn cost
depends on how the robot entered a cell. Combined uses ordinary search when its
turn weight is zero and direction-aware search otherwise.

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

The plugin tests load the installed library via pluginlib without directly
linking it, exercise lifecycle behavior, and plan on a synthetic Costmap2DROS
with no sensor layers or activation. They verify unknown-space configuration,
unchanged map data, planning in all four modes, and invalid parameter rejection.

Algorithm tests cover analytical path lengths, wall detours, no-route failures,
diagonal corner checks, unknown/inscribed/lethal cells, exact endpoints, origin
and resolution, frames/stamps, quaternion validity, and same-cell requests.
An independent dense Dijkstra oracle checks optimality and reachability across
100 seeded maps; repeated requests check deterministic output. These are
synthetic map tests, not evidence of robot or simulator navigation performance.
Criterion tests additionally compare both weighted objectives against an independent
direction-expanded Dijkstra oracle on 40 seeded maps per mode, including endpoint
turns and different resolutions. Fixtures demonstrate a safer detour and a longer
route with fewer turns; zero-weight, unknown-space, and collision rules are checked.
Rebuild and source the overlay before testing.

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
| `src/plugin/a_star_planner.cpp` | Lifecycle, parameters, costmap snapshot, plugin registration |
| `include/mecanumbot_path_planner/planner_options.hpp`, `src/core/planner_options.cpp` | Mode, weights, parsing and validation |
| `src/plugin/` | Nav2 lifecycle and costmap adapter |
| `src/core/` | Options, validation, dispatch, shared map geometry and path output |
| `src/planners/` | One focused implementation per planning objective, including combined weights |
| `src/search/` | Reusable A* queue and cell-search mechanics |
| `config/planner_params.yaml` | Future Planner Server integration fragment |
| `plugin.xml` | Plugin lookup name, C++ type, base class and shared library |
| `CMakeLists.txt` | C++17 build, installation, ament/pluginlib exports and smoke tests |
| `package.xml` | ROS package metadata and dependencies |
| `test/integration/` | Installed-plugin and lifecycle smoke tests |
| `test/unit/` | Grid behavior, optimality oracles, tradeoffs and parameter checks |

## Next development step

Validate the baseline with the real Nav2 Planner Server in a separate simulation
configuration, including Mecanumbot's footprint, inflation, TF, and maps. Compare
path length and planning time against NavFn. Evaluate cost exposure and squared
direction changes separately, then tune weights against measured clearance and
controller behavior. No robot
configuration should be replaced by this package's parameter fragment.

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
