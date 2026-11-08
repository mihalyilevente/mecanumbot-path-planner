# mecanumbot_planner_ui

Browser UI for Part 4: select planning criteria, visualise paths, and drive the
robot. Pattern matches the onboard `mecanumbot_web` stack (Flask inside an
rclpy node).

## Run (Docker sim)

With the simulation already up (`sim.launch.py`):

```bash
ros2 launch mecanumbot_planner_ui ui.launch.py
```

Open <http://localhost:8090>.

## What it does

| Control | Behaviour |
| --- | --- |
| Criteria radios | Shortest / Safest / Smoothest / Combined / NavFn |
| Map click-drag | Sets goal pose (press = position, drag = yaw) |
| Scenario + Load | Fills start/goal from `mecanumbot_sim_env` scenario YAML |
| Compare all | `ComputePathToPose` for every planner, coloured overlays |
| Drive | `NavigateToPose` with `navigate_to_pose_<planner>.xml` |
| Cancel | Cancels the active NavigateToPose goal |

Harness colours: Shortest blue, Safest green, Smoothest red, Combined purple,
NavFn orange.

## Parameters

Defaults are in `config/ui_params.yaml` (Docker sim / `localization:=none`):

| Parameter | Default |
| --- | --- |
| `host` / `port` | `0.0.0.0` / `8090` |
| `map_topic` | `map` |
| `odom_topic` | `/mecanumbot/odom` |
| `plan_topic` | `plan` |
| `navigate_action` | `navigate_to_pose` |
| `compute_path_action` | `compute_path_to_pose` |
| `pose_source` | `odom` (use `tf` with AMCL on the real robot) |
| `map_frame` / `base_frame` | `map` / `mecanumbot/base_link` |
| `behavior_trees_dir` / `scenarios_dir` | `mecanumbot_sim_env` share paths |

Compose binds `127.0.0.1:8090` (no login; LAN exposure is intentional risk, same
policy as `mecanumbot_web`).

## Lab / Orin notes

Use the same `ROS_DOMAIN_ID` / RMW as the robot. Override topic and action names
if the live stack differs, and set `pose_source:=tf` when AMCL is running.
Initial pose is still set with RViz / `/initialpose` in v1.

## Tests

```bash
cd /ws/src/mecanumbot-path-planner
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=mecanumbot_planner_ui:$PYTHONPATH \
  python3 -m pytest mecanumbot_planner_ui/test -q
```
