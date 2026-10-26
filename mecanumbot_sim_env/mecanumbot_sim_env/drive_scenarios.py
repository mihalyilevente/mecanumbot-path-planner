"""Drive every start/goal pair with every planner in Gazebo and record it.

Needs a running simulation (sim.launch.py) on the same map. For each pair and
planner the robot is put on the start pose, NavigateToPose is sent with that
planner's behavior tree, and the planned path, the driven path and the
velocities are recorded in simulation time.

Output goes to results/drive_<map>_<timestamp>/:
  summary.csv         one row per run: success, time, planned/driven length, ...
  runs/<pair>_<planner>.csv
                      the driven trajectory: t, x, y, yaw, odometry velocities
                      (robot frame) and the velocity command sent to the base
  paths.json          planned and driven paths of every run
  <pair>.png          driven paths (thick) over planned paths (thin)

    ros2 run mecanumbot_sim_env drive_scenarios test_slalom
    ros2 run mecanumbot_sim_env drive_scenarios AI_dept --planners Shortest Safest --only wp1_to_wp3
"""

import argparse
import csv
import json
import math
import os
import subprocess
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from mecanumbot_sim_env import path_image
from mecanumbot_sim_env.plan_scenarios import (
    DEFAULT_PLANNERS, load_scenarios, make_pose, path_length, path_points,
)

PACKAGE = 'mecanumbot_sim_env'
WORLD = 'sim'
ROBOT = 'mecanumbot'
SUMMARY_FIELDS = [
    'pair', 'planner', 'success', 'duration_s', 'planned_length_m', 'driven_length_m',
    'final_error_m', 'replans',
]
RUN_FIELDS = ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'cmd_vx', 'cmd_vy', 'cmd_wz']


def yaw_of(orientation):
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


def teleport(x, y, yaw):
    """Put the robot on a pose through Gazebo's set_pose service."""
    request = (f'name: "{ROBOT}" position {{x: {x} y: {y} z: 0}} '
               f'orientation {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}')
    result = subprocess.run(
        ['ign', 'service', '-s', f'/world/{WORLD}/set_pose',
         '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean',
         '--timeout', '3000', '--req', request],
        capture_output=True, text=True, check=False)
    if 'true' not in result.stdout:
        raise RuntimeError('Could not move the robot in Gazebo; is sim.launch.py running? '
                           + (result.stdout + result.stderr).strip())


class Driver(Node):

    def __init__(self):
        super().__init__('drive_scenarios',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, name) for name in (
                'global_costmap/clear_entirely_global_costmap',
                'local_costmap/clear_entirely_local_costmap')]
        self.initial_pose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.create_subscription(Odometry, '/mecanumbot/odom', self.on_odom, 50)
        self.create_subscription(Twist, '/mecanumbot/cmd_vel', self.on_cmd, 50)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.odom = None
        self.cmd = Twist()
        self.recording = None
        self.plans = []

    def on_odom(self, msg):
        self.odom = msg
        if self.recording is not None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pose, twist = msg.pose.pose, msg.twist.twist
            self.recording.append([
                round(stamp, 3), round(pose.position.x, 4), round(pose.position.y, 4),
                round(yaw_of(pose.orientation), 4),
                round(twist.linear.x, 4), round(twist.linear.y, 4), round(twist.angular.z, 4),
                round(self.cmd.linear.x, 4), round(self.cmd.linear.y, 4), round(self.cmd.angular.z, 4),
            ])

    def on_cmd(self, msg):
        self.cmd = msg

    def on_plan(self, msg):
        if self.recording is not None:
            self.plans.append(path_points(msg))

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_ready(self, timeout):
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('navigate_to_pose is not available; is sim.launch.py running?')
        deadline = time.monotonic() + timeout
        while self.odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.odom is None:
            raise RuntimeError('No odometry on /mecanumbot/odom')

    def reset_to(self, start):
        teleport(*start)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose = make_pose(start, 'map').pose
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.01
        pose.pose.covariance[35] = 0.01
        self.initial_pose.publish(pose)  # only matters with localization:=amcl
        self.spin_for(1.0)
        for client in self.clear_clients:
            if client.wait_for_service(timeout_sec=2.0):
                rclpy.spin_until_future_complete(
                    self, client.call_async(ClearEntireCostmap.Request()), timeout_sec=2.0)
        self.spin_for(1.0)

    def drive(self, start, goal, tree, timeout):
        """Drive one run; returns (status, trajectory rows, plans)."""
        self.reset_to(start)
        request = NavigateToPose.Goal()
        request.pose = make_pose(goal, 'map')
        request.behavior_tree = tree

        self.recording, self.plans = [], []
        sent = self.client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            self.recording = None
            return GoalStatus.STATUS_ABORTED, [], []

        done = handle.get_result_async()
        started = self.get_clock().now()
        while not done.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.get_clock().now() - started).nanoseconds * 1e-9 > timeout:
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
                break
        self.spin_for(0.5)
        rows, plans = self.recording, self.plans
        self.recording = None
        status = done.result().status if done.done() else GoalStatus.STATUS_CANCELED
        return status, rows, plans


def summarize(pair, planner, status, rows, plans):
    driven = [(row[1], row[2]) for row in rows]
    planned = plans[0] if plans else []
    goal = pair['goal']
    return {
        'pair': pair['name'],
        'planner': planner,
        'success': status == GoalStatus.STATUS_SUCCEEDED,
        'duration_s': round(rows[-1][0] - rows[0][0], 2) if len(rows) > 1 else '',
        'planned_length_m': round(path_length(planned), 3) if planned else '',
        'driven_length_m': round(path_length(driven), 3) if driven else '',
        'final_error_m': round(math.dist(driven[-1], goal[:2]), 3) if driven else '',
        'replans': max(len(plans) - 1, 0),
    }, planned, driven


def print_table(rows):
    header = (f"{'pair':<18}{'planner':<11}{'ok':<4}{'time s':>8}"
              f"{'planned m':>11}{'driven m':>10}{'error m':>9}")
    print(header)
    print('-' * len(header))
    for row in rows:
        print(f"{row['pair']:<18}{row['planner']:<11}{'yes' if row['success'] else 'NO':<4}"
              f"{row['duration_s']:>8}{row['planned_length_m']:>11}{row['driven_length_m']:>10}"
              f"{row['final_error_m']:>9}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('scenarios', help='Scenario name in this package or a yaml path')
    parser.add_argument('--planners', nargs='+', default=DEFAULT_PLANNERS)
    parser.add_argument('--only', nargs='+', help='Run only these pair names')
    parser.add_argument('--out', default='results', help='Parent directory for the output folder')
    parser.add_argument('--timeout', type=float, default=180.0,
                        help='Simulated seconds allowed per run before it is cancelled')
    parser.add_argument('--no-images', action='store_true', help='Skip the PNG overlays')
    args = parser.parse_args()

    scenarios = load_scenarios(args.scenarios)
    pairs = [p for p in scenarios['pairs'] if not args.only or p['name'] in args.only]
    if not pairs:
        parser.error('no matching pairs')
    trees = os.path.join(get_package_share_directory(PACKAGE), 'behavior_trees')

    out_dir = os.path.join(args.out, f"drive_{scenarios['map']}_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(os.path.join(out_dir, 'runs'), exist_ok=True)

    rclpy.init()
    driver = Driver()
    summary, paths = [], {}
    try:
        driver.wait_until_ready(30.0)
        for pair in pairs:
            paths[pair['name']] = {}
            for planner in args.planners:
                tree = os.path.join(trees, f'navigate_to_pose_{planner.lower()}.xml')
                print(f"{pair['name']} / {planner} ...", flush=True)
                status, rows, plans = driver.drive(pair['start'], pair['goal'], tree, args.timeout)
                row, planned, driven = summarize(pair, planner, status, rows, plans)
                summary.append(row)
                paths[pair['name']][planner] = {'planned': planned, 'driven': driven}
                with open(os.path.join(out_dir, 'runs', f"{pair['name']}_{planner}.csv"),
                          'w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(RUN_FIELDS)
                    writer.writerows(rows)
    finally:
        driver.destroy_node()
        rclpy.shutdown()

    with open(os.path.join(out_dir, 'summary.csv'), 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(summary)
    with open(os.path.join(out_dir, 'paths.json'), 'w') as stream:
        json.dump({'map': scenarios['map'], 'pairs': scenarios['pairs'], 'paths': paths}, stream)

    if not args.no_images:
        map_yaml = os.path.join(
            get_package_share_directory(PACKAGE), 'maps', scenarios['map'], scenarios['map'] + '.yaml')
        for pair in pairs:
            runs = paths[pair['name']]
            path_image.draw(
                map_yaml, pair,
                {planner: run['driven'] for planner, run in runs.items() if run['driven']},
                os.path.join(out_dir, pair['name'] + '.png'),
                thin_paths={planner: run['planned'] for planner, run in runs.items() if run['planned']},
                planners=args.planners)

    print()
    print_table(summary)
    failed = sum(not row['success'] for row in summary)
    print(f'\n{len(summary) - failed}/{len(summary)} runs reached the goal. '
          f'Results in {os.path.abspath(out_dir)}')


if __name__ == '__main__':
    main()
