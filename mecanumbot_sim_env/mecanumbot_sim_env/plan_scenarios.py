"""Request a plan for every start/goal pair and planner, then save the results.

Needs a running Planner Server, e.g. planner_harness.launch.py. For each pair
and planner it records whether planning succeeded, the path length, planning
time, total turning and the costmap cost along the path. Output goes to
results/<map>_<timestamp>/: results.csv, paths.json and one PNG per pair with
all planners drawn on the map.

    ros2 run mecanumbot_sim_env plan_scenarios LED_exp
    ros2 run mecanumbot_sim_env plan_scenarios AI_dept --planners Shortest Safest --only wp1_to_wp4
"""

import argparse
import csv
import json
import math
import os
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from mecanumbot_sim_env import path_image

PACKAGE = 'mecanumbot_sim_env'
DEFAULT_PLANNERS = ['Shortest', 'Safest', 'Smoothest', 'Combined', 'NavFn']
CSV_FIELDS = [
    'pair', 'planner', 'success', 'length_m', 'poses', 'planning_time_ms',
    'total_turn_deg', 'mean_cost', 'max_cost',
]


def load_scenarios(value):
    if not value.endswith('.yaml'):
        value = os.path.join(get_package_share_directory(PACKAGE), 'scenarios', value + '.yaml')
    with open(value) as stream:
        return yaml.safe_load(stream)


def make_pose(values, frame):
    x, y, yaw = values
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def path_points(path):
    return [(p.pose.position.x, p.pose.position.y) for p in path.poses]


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def total_turn_deg(points):
    """Sum of absolute heading changes between consecutive non-zero segments."""
    headings = [
        math.atan2(b[1] - a[1], b[0] - a[0])
        for a, b in zip(points, points[1:]) if math.dist(a, b) > 1e-6
    ]
    turn = 0.0
    for before, after in zip(headings, headings[1:]):
        turn += abs(math.atan2(math.sin(after - before), math.cos(after - before)))
    return math.degrees(turn)


def costs_along(points, grid):
    """Costmap values (0-100 as published) under each path point; unknown cells are skipped."""
    info = grid.info
    values = []
    for x, y in points:
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if 0 <= col < info.width and 0 <= row < info.height:
            cost = grid.data[row * info.width + col]
            if cost >= 0:
                values.append(cost)
    return values


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('plan_scenarios')
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.costmap = None
        self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap', self.on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.path_publishers = {}

    def on_costmap(self, msg):
        self.costmap = msg

    def wait_until_ready(self, timeout):
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('compute_path_to_pose is not available; is the harness running?')
        deadline = time.monotonic() + timeout
        while self.costmap is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.costmap is None:
            raise RuntimeError('No message on global_costmap/costmap')

    def plan(self, start, goal, planner, timeout):
        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal
        request.planner_id = planner
        request.use_start = True

        sent = self.client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=timeout)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return None
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done, timeout_sec=timeout)
        response = done.result()
        if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
            return None
        return response.result

    def publish_path(self, planner, path):
        """Latched, so RViz shows the most recent path of every planner."""
        if planner not in self.path_publishers:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.path_publishers[planner] = self.create_publisher(
                Path, 'harness/path/' + planner.lower(), qos)
        self.path_publishers[planner].publish(path)


def evaluate(result, grid):
    points = path_points(result.path)
    costs = costs_along(points, grid)
    elapsed = result.planning_time
    return {
        'success': True,
        'length_m': round(path_length(points), 3),
        'poses': len(points),
        'planning_time_ms': round(elapsed.sec * 1e3 + elapsed.nanosec / 1e6, 2),
        'total_turn_deg': round(total_turn_deg(points), 1),
        'mean_cost': round(sum(costs) / len(costs), 2) if costs else '',
        'max_cost': max(costs) if costs else '',
    }, points


def print_table(rows):
    header = f"{'pair':<16}{'planner':<11}{'ok':<4}{'length':>8}{'time ms':>9}{'turn deg':>10}{'mean cost':>11}"
    print(header)
    print('-' * len(header))
    for row in rows:
        if row['success']:
            print(f"{row['pair']:<16}{row['planner']:<11}{'yes':<4}{row['length_m']:>8}"
                  f"{row['planning_time_ms']:>9}{row['total_turn_deg']:>10}{row['mean_cost']:>11}")
        else:
            print(f"{row['pair']:<16}{row['planner']:<11}{'NO':<4}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('scenarios', help='Scenario name in this package (AI_dept, LED_exp) or a yaml path')
    parser.add_argument('--planners', nargs='+', default=DEFAULT_PLANNERS)
    parser.add_argument('--only', nargs='+', help='Run only these pair names')
    parser.add_argument('--out', default='results', help='Parent directory for the output folder')
    parser.add_argument('--timeout', type=float, default=30.0, help='Seconds per request')
    parser.add_argument('--no-images', action='store_true', help='Skip the PNG overlays')
    args = parser.parse_args()

    scenarios = load_scenarios(args.scenarios)
    pairs = [p for p in scenarios['pairs'] if not args.only or p['name'] in args.only]
    if not pairs:
        parser.error('no matching pairs')

    out_dir = os.path.join(args.out, f"{scenarios['map']}_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    runner = ScenarioRunner()
    rows, paths = [], {}
    try:
        runner.wait_until_ready(args.timeout)
        frame = runner.costmap.header.frame_id or 'map'
        for pair in pairs:
            start = make_pose(pair['start'], frame)
            goal = make_pose(pair['goal'], frame)
            paths[pair['name']] = {}
            for planner in args.planners:
                result = runner.plan(start, goal, planner, args.timeout)
                row = {'pair': pair['name'], 'planner': planner, 'success': False}
                if result is not None:
                    metrics, points = evaluate(result, runner.costmap)
                    row.update(metrics)
                    paths[pair['name']][planner] = points
                    runner.publish_path(planner, result.path)
                rows.append(row)
    finally:
        runner.destroy_node()
        rclpy.shutdown()

    with open(os.path.join(out_dir, 'results.csv'), 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, restval='')
        writer.writeheader()
        writer.writerows(rows)
    with open(os.path.join(out_dir, 'paths.json'), 'w') as stream:
        json.dump({'map': scenarios['map'], 'pairs': scenarios['pairs'], 'paths': paths}, stream)

    if not args.no_images:
        map_yaml = os.path.join(
            get_package_share_directory(PACKAGE), 'maps', scenarios['map'], scenarios['map'] + '.yaml')
        for pair in pairs:
            path_image.draw(
                map_yaml, pair, paths[pair['name']],
                os.path.join(out_dir, pair['name'] + '.png'), planners=args.planners)

    print_table(rows)
    failed = sum(not row['success'] for row in rows)
    print(f'\n{len(rows) - failed}/{len(rows)} plans succeeded. Results in {os.path.abspath(out_dir)}')


if __name__ == '__main__':
    main()
