# Environment inspection and setup

Inspected on 2026-10-23.

## Workspace and source control

- Windows path: `D:\Uni\3.semester\EmbodiedInt\CustomCriteriaPlanning_PathFinding`
- Ubuntu path: `/mnt/d/Uni/3.semester/EmbodiedInt/CustomCriteriaPlanning_PathFinding`
- The directory was empty, with no enclosing Git repository or applicable
  ancestor `AGENTS.md`. Adjacent coursework contains `j8rfm5` and `midterm_sensor_5`.
- Existing Ubuntu workspace: `/home/mihal/ros2_ws`, with build/install/log and
  example/coursework packages; its `src/examples` is a Git repository.
  No Mecanumbot packages were found in the inspected coursework or Ubuntu workspace.
- Windows Git: 2.41.0; GitHub CLI: 2.97.0; authenticated account: `mihalyilevente`.
  Existing Windows commit identity: `Levente <mihalyilevi2@gmail.com>`.
- Ubuntu Git: 2.34.1. No Ubuntu-specific Git identity was configured during inspection.
- New private repository: <https://github.com/mihalyilevente/mecanumbot-path-planner>.
- Default branch `main` contains the initial repository scaffold; package
  development is committed to `develop`, which is the local checked-out branch.
- Only this repository's origin is configured; no Fortuz repository was changed.

## Build environment

| Component | Status |
| --- | --- |
| OS | Ubuntu 22.04.5 LTS, amd64, WSL2 distribution `Ubuntu-22.04` |
| ROS | Humble, `/opt/ros/humble`, already installed; explicitly source setup.bash |
| CMake | 3.22.1, already installed |
| C++ compiler | GCC/G++ 11.4.0, already installed; package uses C++17 |
| colcon common extensions | 0.3.0-100, already installed |
| ament_cmake / ament_cmake_gtest | 1.3.14, already installed |
| Navigation2 / nav2_core / nav2_bringup | Missing initially; installed 1.1.20 |

The Windows shell itself did not expose ROS, colcon, CMake or a C++ compiler.
All compilation and ROS validation use the existing Ubuntu-22.04 distribution;
the default WSL distribution was not changed.

Missing Nav2 dependencies were reported before installation. Installed with:

```powershell
wsl -d Ubuntu-22.04 -u root -- bash -lc 'apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup'
```

Apt also installed required transitive dependencies and updated required Kerberos
libraries. No general system upgrade, ROS reinstall, simulator install, or robot
bringup was performed. No remaining dependency required by this package was
identified after the build.

## Upstream inspection

The package follows Mecanumbot's C++17 / CMake 3.8 minimum convention and retains
`GridBased` as the example Planner Server ID. The existing robot uses
`nav2_navfn_planner/NavfnPlanner`. The new plugin is only in this repository's
example parameter fragment; existing launch and YAML files are untouched.

Sources inspected read-only:

- `Fortuz/mecanumbot` at `5818369f8571942a3b21b3ed4861d5dae3752ab0`:
  bringup CMake, Nav2 launch, and planner parameters.
- `Fortuz/mecanumbot_remote` at `0e696edd96f2c43a93bfd0168cfb13a73ca89009`:
  package layout and `mecanumbot_sim/README.md`.
- Mecanumbot documentation and both the upstream Humble and locally installed
  `nav2_core/global_planner.hpp`.

Links and integration details are in the [package README](../mecanumbot_path_planner/README.md).

## Initial scaffold validation (before shortest-path A*)

- `colcon build --packages-select mecanumbot_path_planner`: **passed**, one package,
  about 78 seconds, no compilation errors or warnings reported.
- `colcon test --packages-select mecanumbot_path_planner --event-handlers console_direct+`:
  **passed**, two GoogleTest cases in one CTest target. Tests used localhost-only
  DDS and ROS domain 91.
- `colcon test-result --verbose`: **3 tests, 0 errors, 0 failures, 0 skipped**
  (colcon includes the CTest result as well as the two GoogleTest cases).
- Pluginlib discovered and loaded the installed shared library through
  `nav2_core::GlobalPlanner`, without a direct link to the plugin in the test.
- Tested configure/activate/deactivate/cleanup, all three mode placeholders,
  invalid mode rejection, and explicit failure for unimplemented path search.
- Installed plugin XML and `nav2_core__pluginlib__plugin` ament resource match
  the plugin name used in the installed YAML.
- `build/`, `install/`, and `log/` are ignored by Git.

Full Planner Server bringup, simulation navigation, robot execution, and path
quality evaluation are outside this scaffold's validation.

## Local shortest-path implementation validation

The next development step implements `shortest` grid A* and leaves `safest` and
`smoothest` explicitly unimplemented. No dependencies were installed for this
step. Work remains uncommitted on `develop`; nothing was pushed.

- `colcon build --packages-select mecanumbot_path_planner`: passed with no
  compiler warnings after correcting explicit Time construction in the tests.
- 12 GoogleTest cases passed: 9 algorithm tests and 3 installed-plugin tests.
  Both CTest targets passed. Colcon's combined summary counts 14 results
  (12 cases plus 2 CTest targets), with 0 errors, failures, or skipped tests.
- Algorithm tests include an independent Dijkstra comparison over 100 seeded
  maps, deterministic paths, obstacle detours, corner blocking, bounds, unknown
  cells, inscribed costs, exact endpoints, frames/stamps, and invalid poses.
- The plugin test used a configured synthetic Costmap2DROS with no sensor layers
  and no activation. It checked actual shortest-path output, lifecycle errors,
  uninitialized costmap handling, the unknown-space parameter, and unchanged
  costmap data.
- Tests ran with `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=91`. No robot or simulation
  navigation was started. Full footprint and Planner Server integration remain
  unvalidated; synthetic-map results do not establish robot navigation safety.

Rebuild/test commands remain in the root and package READMEs. The package README
documents exact search semantics and limits.

## Safest and smoothest implementation validation

Confirmed on 2026-10-24 after resuming the interrupted session:

- Build succeeded; 19 GoogleTest cases and all 3 CTest targets passed.
  Colcon reports 22 results, 0 errors, 0 failures, 0 skipped.
- Both new objectives match an independent direction-expanded Dijkstra oracle
  on 40 seeded maps each, including offset endpoints, connector turns, varied
  preferences and map resolutions. The original 100-map shortest-path check passes.
- Tests demonstrate the distance/exposure and distance/turn tradeoffs, zero-preference
  equivalence, unknown-space penalties, common collision rules, and invalid preferences.
- Installed-plugin tests exercise shortest, safest, smoothest, and combined modes.
- No additional dependencies, robot runs, commits, or pushes were performed.
  Changes remain local on develop. Synthetic tests do not establish robot safety
  or dynamic trajectory smoothness; definitions and limits are in the package README.

## Readability refactor validation

Confirmed on 2026-10-25: the single large search implementation was separated
into criterion planners, shared grid rules, reusable queue/search mechanics,
option parsing, and path construction. Public behavior and objectives are
unchanged. The package rebuilt successfully and all 22 reported test results
passed with 0 errors, failures, or skipped tests. Changes remain local and
uncommitted on `develop`; nothing was pushed.

Source files are grouped under `src/plugin`, `src/core`, `src/planners`, and
`src/search`. Tests are grouped under `test/unit` and `test/integration`.

## Combined weighted planner

The local implementation also provides `optimization_mode: combined`, minimizing
`length + (2 * safety_preference) * exposure + (0.5 m/rad^2 *
smoothness_preference) * turns`. Both exposed preferences are
finite ROS parameters in `[0, 1]`; one means the defined maximum penalty. When
both are zero, combined mode delegates exactly to shortest. It reuses the shared
search code and switches to direction-aware states only when smoothness is positive.
No commit or push was
performed. The current build reports 23 tests, 0 errors, 0 failures, and 0 skipped.
