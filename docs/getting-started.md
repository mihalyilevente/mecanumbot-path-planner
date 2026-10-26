# Getting started from scratch

This walks through everything from a fresh machine to the robot driving in
Gazebo. You don't need ROS installed locally, everything runs in Docker. It
works on macOS (Intel and Apple Silicon), Windows and Linux.

It takes about half an hour the first time, most of it is the Docker build.

## 1. Install the tools

- **Git**
- **Docker Desktop** (macOS/Windows) or Docker Engine with the compose plugin
  (Linux). In Docker Desktop go to Settings → Resources and give it at least
  **8 GB memory and 4 CPUs**. Gazebo renders in software and needs it.
- A web browser, for the desktop that shows RViz and Gazebo.

Check that Docker runs:

```bash
docker --version
docker compose version
```

## 2. Get the code

```bash
git clone https://github.com/mihalyilevente/mecanumbot-path-planner.git
cd mecanumbot-path-planner
git switch Simulation-develop      # until it is merged into main
```

## 3. Build and start the container

From the repository folder:

```bash
docker compose up -d --build
```

The first build downloads ROS 2 Humble, Nav2 and Gazebo, which takes 10-20
minutes. After that, `docker compose up -d` starts in a few seconds.

## 4. Open the desktop

Open <http://localhost:6080/vnc.html> in the browser and click **Connect**.
You get an empty dark desktop. RViz and Gazebo windows show up here later.

## 5. Build the workspace

Open a shell inside the container:

```bash
docker compose exec dev bash
```

Your prompt changes to something like `dev@mecanumbot-dev`. Everything below
runs in this shell. Build:

```bash
cb
```

(`cb` is a shortcut for `colcon build --symlink-install`.) It should end with
`2 packages finished`. Run `cb` again whenever you pull new code or change C++.

Optionally run the tests with `ct`. Everything should pass.

## 6. Drive the robot in Gazebo

```bash
ros2 launch mecanumbot_sim_env sim.launch.py map:=AI_dept
```

Wait about 20 seconds. In the terminal you should see `Managed nodes are
active` twice. After that the log goes quiet and **the terminal doesn't give
you a prompt back**. That's normal: the simulation runs as long as this
command runs. Leave it open and switch to the browser.

These messages in the log look scary but are harmless:

- `GLSL link result : active samplers with a different type...` (RViz with
  software rendering, the map still shows fine)
- `Timed out waiting for transform from mecanumbot/base_link to
  mecanumbot/odom` (only during the first second of startup)
- `Message Filter dropping message: frame 'mecanumbot/base_scan'` (once, at
  startup)

In the browser desktop RViz shows the map with the costmaps (purple/blue
around walls). The robot is the small green outline, at first in the lower
middle of the AI_dept map. Scroll to zoom, drag with the middle mouse button
(or Shift + left) to move the view. Left-drag with the **Move Camera** tool
rotates the view; **Reset** at the bottom left puts it back.

Some shimmer along the walls and a square patch that moves with the robot
are normal. The lidar has a bit of simulated noise, so wall edges keep
changing in the costmap, and the patch is the local costmap, the 3x3 m area
the controller uses for nearby obstacles.

### What the colours mean

RViz draws several layers on top of each other. From the bottom up:

**The map** (what the robot knows about the room, it never changes):

| Colour | Meaning |
| --- | --- |
| White | Free space |
| Black | Wall or obstacle |
| Grey-green | Unknown, never seen when the map was recorded. Only on the lab maps; goals there are rejected. |

**The costmaps**, drawn see-through over the map. Nav2 gives every cell a
cost: how risky it is for the robot's centre to be there. The global costmap
covers the whole map and is what the planners use. The local costmap is the
3x3 m square around the robot that the controller uses.

| Colour | Meaning |
| --- | --- |
| No colour (plain white) | Cost 0, far enough from everything |
| Light blue / lavender, turning pink and red closer to walls | Safety margin around obstacles (inflation). The redder, the higher the cost. Paths may go here, but "Safest" avoids it. |
| Cyan (bright light blue) line | So close to an obstacle that the robot's body would touch it. The robot's centre may never be here. |
| Magenta / dark purple | The obstacle itself |

Both costmaps also contain what the lidar sees right now, so obstacles that
are not on the map would show up here too.

**The robot and its plans:**

| Colour | Meaning |
| --- | --- |
| Green outline | The robot's footprint, where it is right now |
| Red dots | Lidar points: what the laser scanner hits at this moment |
| Blue line | The global plan, the whole path from the robot to the goal (the chosen planner's result) |
| Orange line | The local plan, the short piece the controller is about to drive |
| Green arrow (while clicking) | The goal you are setting with 2D Goal Pose and the heading the robot will have at the end |

To send the robot somewhere: in RViz click **2D Goal Pose** in the toolbar,
then **press the mouse button on the spot where the robot should go**, drag a
little in the direction it should face when it arrives, and release. The
point where you press is the goal; the drag only sets the final heading. So
don't start the drag on the robot itself: a goal closer than 0.3 m counts as
already reached, and the robot only turns on the spot.

A blue line (the planned path) appears and the robot starts driving. In the
terminal you see `Begin navigating from current location ...` and, when it
arrives, `Goal succeeded`.

Other things to try:

```bash
ros2 launch mecanumbot_sim_env sim.launch.py map:=LED_exp              # other map
ros2 launch mecanumbot_sim_env sim.launch.py x:=-1.2 y:=-2.6 yaw:=1.57  # start pose
ros2 launch mecanumbot_sim_env sim.launch.py localization:=amcl         # real localization
ros2 launch mecanumbot_sim_env sim.launch.py gui:=true                  # also the Gazebo window
```

`gui:=true` is slow without a GPU, RViz alone is enough most of the time.

### Maps to try

`map:=` (and the scenario name for `plan_scenarios` / `drive_scenarios`) can
be any of these:

| Map | What it is |
| --- | --- |
| `AI_dept`, `LED_exp` | the real lab maps |
| `test_gap_vs_detour` | narrow passage vs wide detour, Shortest and Safest split here |
| `test_slalom` | corridor with alternating baffles, shows turning |
| `test_cluttered` | room full of pillars |
| `test_dead_end` | U-shaped trap in front of the goal |
| `test_open_room` | empty room, the baseline |

## 7. Test the planners (part 2)

The four planners are `Shortest`, `Safest`, `Smoothest` and `Combined`, with
Nav2's own `NavFn` as a baseline. Only one launch can run at a time, so stop
the simulation (Ctrl+C) first.

**Compare all planners at once (no Gazebo, fast):**

```bash
# shell 1
ros2 launch mecanumbot_sim_env planner_harness.launch.py map:=test_slalom rviz:=true
# shell 2 (docker compose exec dev bash)
cd /ws/src/mecanumbot-path-planner
ros2 run mecanumbot_sim_env plan_scenarios test_slalom
```

Every route of the map is planned with every planner. The table should end
with `15/15 plans succeeded`. RViz shows the last route, and `results/` (in
the repo folder on your machine) gets a CSV and one picture per route.

**Watch the robot drive a planner's path (Gazebo):**

```bash
# shell 1
ros2 launch mecanumbot_sim_env sim.launch.py map:=test_gap_vs_detour
# shell 2
cd /ws/src/mecanumbot-path-planner
ros2 run mecanumbot_sim_env drive_scenarios test_gap_vs_detour --only through_or_around --planners Shortest Safest
```

The robot drives the route once per planner (about a minute each), and
`results/drive_.../` gets the planned and driven paths.

**Another map:** change the map name in both commands, `map:=` in shell 1 and
the argument of `plan_scenarios` / `drive_scenarios` in shell 2. The two must
match. All map names are in "Maps to try" in section 6. To run a single route
add `--only <route>`; the route names are in
`mecanumbot_sim_env/scenarios/<map>.yaml`.

**Your own map** (a `.pgm` + `.yaml` from map_server or SLAM): generate
start/goal points for it, then pass file paths instead of names:

```bash
ros2 run mecanumbot_sim_env make_scenarios /path/to/my_map.yaml > /path/to/my_routes.yaml
ros2 launch mecanumbot_sim_env planner_harness.launch.py map:=/path/to/my_map.yaml rviz:=true
ros2 run mecanumbot_sim_env plan_scenarios /path/to/my_routes.yaml --no-images
```

`--no-images` is needed because the pictures only work for maps inside the
package. The Gazebo simulation also takes `map:=/path/to/my_map.yaml` and
builds the world from it.

**Colours** in the harness RViz and in the pictures:

| Colour | Planner |
| --- | --- |
| Blue | Shortest |
| Green | Safest |
| Red | Smoothest |
| Purple | Combined |
| Orange | NavFn (Nav2's default, for comparison) |

In the pictures the black circle is the start, the red circle the goal, and
"(failed)" in the legend means that planner found no path or didn't arrive.
Drive pictures show the driven path thick and the planned path thin. (In the
simulation's RViz the colours mean something else: blue is the current plan,
whatever the planner, see section 6.)

**What to look for:**

- `test_gap_vs_detour`: Shortest goes through the narrow passage, Safest
  around the top.
- `test_slalom`: Smoothest cuts across with few turns, Shortest hugs the
  corners, Safest stays in the middle of the openings.
- `test_open_room`: all planners nearly agree.

**Known limitations** (not bugs, see `docs/phase2-nav2-config.md`):

- Shortest moves on an 8-direction grid, so it is often a few percent longer
  than NavFn.
- Combined often gives the same path as Smoothest with the default weights.
- Paths start and end with a tiny step back (at most 3.5 cm), so even
  straight paths show some turning in the table.
- An energy/time criterion and per-path metric reporting (from the task page)
  are not implemented yet.

## 8. Shutting down

```bash
exit                    # leave the container shell
docker compose down     # stop the container
```
