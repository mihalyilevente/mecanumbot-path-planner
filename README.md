# Mecanumbot path planner

Independent ROS 2 Humble / Nav2 global planner development for the university
project's Part 2: Path Planning Algorithms.

Development branch: **develop**. The **main** branch contains only the initial
repository scaffold. Make future changes and pushes on develop.

The [mecanumbot_path_planner](mecanumbot_path_planner/README.md) package provides a
C++17 plugin implementing **shortest**, **safest**, **smoothest**, and
**combined weighted** grid A* with explicit cost functions. This repository
does not modify the robot or simulation repositories. Current development is
local only; do not push until explicitly requested.

## Reopen this workspace

In PowerShell:

```powershell
Set-Location 'D:\Uni\3.semester\EmbodiedInt\CustomCriteriaPlanning_PathFinding'
git switch develop
wsl -d Ubuntu-22.04
```

In the resulting Ubuntu shell:

```bash
cd /mnt/d/Uni/3.semester/EmbodiedInt/CustomCriteriaPlanning_PathFinding
source /opt/ros/humble/setup.bash
colcon build --packages-select mecanumbot_path_planner
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=91 colcon test \
  --packages-select mecanumbot_path_planner --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg prefix mecanumbot_path_planner
```

Source `install/setup.bash` in each new shell after building. This folder is both
an independent Git repository and a standalone colcon workspace; colcon discovers
the package directly beneath it. Build artifacts remain in ignored `build/`,
`install/`, and `log/`. The existing `/home/mihal/ros2_ws` is not used or changed.

Windows Git/GitHub CLI handle commits and authenticated pushes. Ubuntu Git can
read this checkout, but its authentication is not separately configured.

## Development workflow

Run Git commands from PowerShell in the repository root:

```powershell
git switch develop
git status
# After editing, building, testing, and reviewing:
git add mecanumbot_path_planner README.md docs
git diff --cached --check
git diff --cached
git commit -m 'Describe the planner change'
git push origin develop
```

Do not push development work to `main`. No upstream robot remote is configured.
See [environment and validation notes](docs/environment.md) for the inspected
tools, installed dependencies, upstream references, and validation boundaries.
