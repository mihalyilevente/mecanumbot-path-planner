# Phase 2: Nav2 config for the simulated robot

This is a summary of what I set up for Nav2 in `mecanumbot_sim_env` and why.
The config lives in `mecanumbot_sim_env/config/nav2_params.yaml`.

## Starting point

I didn't write the config from zero. I took the one the real robot uses
(`mecanumbot_custom_nav2_no_keepout.yaml` from Fortuz/mecanumbot, commit 5818369)
so that the simulation behaves as close to the real thing as possible. The
footprint, inflation (radius 0.66 m, scaling 5.0), the holonomic DWB controller
with the rotation shim, the velocity limits and the AMCL omni motion model are
all unchanged. Frame and topic names stay the same too (`mecanumbot/base_link`,
`/mecanumbot/scan`, `/mecanumbot/cmd_vel` and so on), which should make it
easier to move to the real robot later.

## What I changed

The robot's file targets a newer Nav2 than Humble, which is what we use, so a
few things had to go or be renamed:

- Removed `docking_server`, `error_code_names` and `costmap_update_timeout`.
  Humble doesn't have these.
- Removed the keepout filter. It needs a separate mask server and we don't
  need it in simulation.
- `behavior_server` uses Humble's parameter names (`costmap_topic`,
  `footprint_topic`, `global_frame`) instead of the newer
  `local_costmap_topic` style ones.
- The waypoint follower plugin is `nav2_waypoint_follower::WaitAtWaypoint` in
  Humble (with `::`). With the `/` version it crashed on startup.
- The lifecycle manager list no longer contains `global_costmap` and
  `local_costmap`. In Humble those are managed by the planner and controller
  servers themselves.
- `use_sim_time` is in every section and the launch files set it.

## Planners

The biggest change is in `planner_server`. Instead of one `GridBased` planner
there are five:

| planner_id | what it is |
| --- | --- |
| `Shortest` | our AStarPlanner, shortest mode |
| `Safest` | our AStarPlanner, safest mode, safety_preference 1.0 |
| `Smoothest` | our AStarPlanner, smoothest mode, smoothness_preference 1.0 |
| `Combined` | our AStarPlanner, combined mode, both preferences 1.0 |
| `NavFn` | the default Nav2 planner, kept as a baseline to compare against |

The project description asks for the criterion to be switchable at runtime
without restarting the stack. The plugin only reads its parameters on
configure, but because every criterion is its own planner instance we can just
pick one per request by `planner_id`. Nothing has to be reconfigured.

For `NavigateToPose` the planner id is set inside the behavior tree, so there
is one tree per planner in `behavior_trees/` (copies of Humble's default
`navigate_to_pose_w_replanning_and_recovery.xml` with a different
`planner_id`). The UI can pick the planner by sending the right tree file in
the goal. The default tree uses `Shortest`.

`allow_unknown` is false for all of them, including NavFn, so the comparison
is fair. Note that the real robot runs NavFn with `allow_unknown: true`.

## Launch files

- `planner_harness.launch.py` starts only map_server and planner_server, plus
  fixed TF frames standing in for the robot. The global costmap is built from
  the static map and inflation only (`harness_overrides.yaml`), since there is
  no lidar. This is for comparing planners quickly without Gazebo.
- `navigation.launch.py` starts the full stack (map server, optional AMCL,
  controller, smoother, planner, behaviors, BT navigator, waypoint follower,
  velocity smoother, collision monitor). Velocity goes controller ->
  `/mecanumbot/cmd_vel_nav` -> velocity smoother -> `/mecanumbot/cmd_vel_smoothed`
  -> collision monitor -> `/mecanumbot/cmd_vel`, same as on the robot.
  `localization:=none` (default) publishes map -> odom as identity, which is
  fine in simulation because odometry is exact there. `localization:=amcl`
  runs AMCL instead.

## Maps and scenarios

The `AI_dept` and `LED_exp` maps are copied from the robot repo. For start/goal
pairs I first used the waypoint files that come with them. That worked for
AI_dept, but the LED_exp waypoints don't match the LED_exp map (two are
outside the map, two are in unknown space), so every plan failed. For LED_exp I
generated new points with `make_scenarios`, which picks points that are at
least 0.5 m away from walls and unknown space and spreads them out.

## What I checked

- All four of our planners and NavFn load into one planner server.
- Harness: 50/50 plans succeed on AI_dept and 50/50 on LED_exp
  (10 start/goal pairs x 5 planners each).
- Full stack: all 9 Nav2 servers come up active without errors, and a
  `Safest` request goes through it fine.
- Driving, once the Gazebo robot was in (`sim.launch.py`): on LED_exp the
  robot reached its goal with both the `Safest` and the `Shortest` tree, and on
  AI_dept with AMCL running it crossed the map with `Combined` and stopped
  0.15 m from the goal (tolerance is 0.30 m). AMCL's estimate was about 0.11 m
  off the true position at the end.

## Things I noticed about the planners (for Levente)

From the AI_dept run:

- `Shortest` is longer than NavFn on 8 of the 10 pairs (for example 15.23 m vs
  14.97 m for wp1 to wp4). Our A* moves on an 8-connected grid, so straight diagonal
  lines turn into staircases, while NavFn interpolates. If we want "shortest"
  to really be the shortest, a line-of-sight shortcut on the result or an
  any-angle search (Theta*) would fix it.
- `Safest` clearly works. The mean costmap cost along the path drops a lot
  compared to `Shortest` (wp4 to wp5: 53.0 -> 15.2).
- `Smoothest` cuts the total turning a lot (wp1 to wp4: 720 -> 360 degrees),
  but it's the slowest: up to about 77 ms on AI_dept. `Combined` is up to
  about 157 ms.
- With both preferences at 1.0, `Combined` ends up on exactly the same path as
  `Smoothest` in 5 of the 10 pairs, so the smoothness term seems to dominate. The weights
  probably need tuning.

From the generated test maps:

- Our paths start and end with a tiny step backwards. The path goes from the
  exact start pose to the centre of its grid cell, which can be behind it, and
  only then forward (same at the goal). In `test_open_room` a straight 5 m path
  starts (1.5, 4.0) -> (1.475, 3.975) -> (1.525, 3.975). That's why even
  straight paths show 180 degrees of turning in the harness table, and the
  robot gets a small hook at both ends (at most half a cell, 2.5-3.5 cm here).
  It comes from the start/goal connectors in the search, which are costed on
  purpose, and `PreferenceObjectivesMatchIndependentOracleIncludingEndpointTurns`
  checks exactly that model. Just dropping the cell centres in `buildPath`
  breaks that test, so a real fix would mean costing the first and last step
  from the actual pose in the search (and in the oracle). I left it as it is.
- `test_gap_vs_detour` works as intended: `Shortest` takes the narrow
  passage (7.07 m), `Safest` the wide detour (8.94 m, mean cost 0).

## How to run it

See `mecanumbot_sim_env/README.md`. In short, in the dev container:

```bash
cb
ros2 launch mecanumbot_sim_env planner_harness.launch.py map:=AI_dept rviz:=true
# second terminal
cd /ws/src/mecanumbot-path-planner
ros2 run mecanumbot_sim_env plan_scenarios AI_dept
```
