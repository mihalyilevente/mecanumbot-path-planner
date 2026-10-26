# Docker development environment

One image for the whole team: ROS 2 Humble, Nav2, Gazebo Fortress (`ros_gz`),
slam_toolbox, RViz, Foxglove bridge, and a browser desktop for GUIs. It builds
natively on amd64 (Linux, Windows/WSL) and arm64 (Apple Silicon). A native
Ubuntu 22.04 + Humble install still works; this is the zero-setup option.

## Requirements

- Docker Desktop (macOS/Windows) or Docker Engine with the compose plugin (Linux).
- On Docker Desktop, give the VM at least 8 GB memory and 4 CPUs
  (Settings → Resources). Gazebo uses software rendering.

## Daily use

From the repository root:

```bash
docker compose up -d --build     # first build takes ~10-20 minutes
docker compose exec dev bash     # shell inside the container
```

Inside the container:

```bash
cb    # colcon build --symlink-install (from /ws)
ct    # colcon test + colcon test-result --verbose
sws   # re-source /ws/install/setup.bash after building in another shell
```

Stop with `docker compose down`. Build outputs survive in named volumes; add
`-v` to `down` to wipe them for a clean build.

## Layout

| Path in container | What it is |
| --- | --- |
| `/ws/src/mecanumbot-path-planner` | This repository, bind-mounted (edit on the host) |
| `/ws/build`, `/ws/install`, `/ws/log` | colcon outputs in Docker volumes, not in the repo |

## GUIs

- **Desktop:** open <http://localhost:6080/vnc.html> and click *Connect*.
  Anything started from `docker compose exec dev bash` with GUI output
  (Gazebo, RViz) appears there, because `DISPLAY=:1` is preset.
- **Foxglove:** run `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
  in the container, then connect Foxglove Studio to `ws://localhost:8765`.

Both ports are bound to `127.0.0.1` only. The VNC desktop has no password, so
do not expose it on a network.

## VS Code

With the *Dev Containers* extension, run **Dev Containers: Reopen in Container**.
It uses the same `compose.yaml`, so the terminal and C++/Python tooling run in
the container.

## Headless use (CI)

Set `HEADLESS=1` to skip the desktop:

```bash
docker compose run --rm -e HEADLESS=1 dev bash -lc 'cd /ws && colcon build && colcon test'
```

## ROS networking

The container uses `ROS_DOMAIN_ID=42` on Docker's private network, so it does
not see other ROS graphs. Talking to the real robot needs host networking and a
matching domain ID; that is out of scope for the simulation setup.
