"""Checks shared by the launch files."""

import subprocess

STOP_HINT = "  pkill -f 'ros2 launch'; pkill -f 'ign gazebo'; pkill -f /opt/ros/humble/lib/"


def already_running():
    """Gazebo or Nav2 processes left from another launch, as 'pid command' lines."""
    try:
        found = subprocess.run(
            ['pgrep', '-af', r'ign gazebo|__node:=planner_server'],
            capture_output=True, text=True, check=False).stdout
    except FileNotFoundError:
        return []
    return [line[:100] for line in found.splitlines() if line.strip()]


def refuse_if_running():
    """Stop the launch if a simulation or harness is already up.

    Two at once fight over /clock, /map and the Nav2 node names: time jumps
    back, the map flickers and the robot does not move.
    """
    running = already_running()
    if running:
        raise RuntimeError(
            'Another simulation or planner harness is still running:\n  '
            + '\n  '.join(running)
            + '\nStop it with Ctrl+C in its terminal, or run:\n' + STOP_HINT)
