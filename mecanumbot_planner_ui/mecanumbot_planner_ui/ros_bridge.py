"""ROS 2 bridge: map/odom/plan caches and Navigate / ComputePath clients."""

from __future__ import annotations

import base64
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

from mecanumbot_planner_ui.planner_catalog import (
    PLANNER_IDS,
    behavior_tree_path,
    planner_list,
)
from mecanumbot_planner_ui.pose_math import (
    downsample_path,
    grid_to_png_bytes,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from mecanumbot_planner_ui.scenario_catalog import find_pair, load_scenario, list_scenarios


def _default_share_subdir(subdir: str) -> str:
    return os.path.join(get_package_share_directory('mecanumbot_sim_env'), subdir)


class RosBridge(Node):
    """Thread-safe caches and action helpers for the Flask UI."""

    def __init__(self) -> None:
        super().__init__('planner_ui')
        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8090)
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('odom_topic', '/mecanumbot/odom')
        self.declare_parameter('plan_topic', 'plan')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('compute_path_action', 'compute_path_to_pose')
        self.declare_parameter('pose_source', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'mecanumbot/base_link')
        self.declare_parameter('behavior_trees_dir', '')
        self.declare_parameter('scenarios_dir', '')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = int(self.get_parameter('port').value)
        self.pose_source = self.get_parameter('pose_source').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        trees = self.get_parameter('behavior_trees_dir').get_parameter_value().string_value
        scenarios = self.get_parameter('scenarios_dir').get_parameter_value().string_value
        self.behavior_trees_dir = trees or _default_share_subdir('behavior_trees')
        self.scenarios_dir = scenarios or _default_share_subdir('scenarios')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        plan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value
        nav_action = self.get_parameter('navigate_action').get_parameter_value().string_value
        compute_action = self.get_parameter(
            'compute_path_action').get_parameter_value().string_value

        self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, map_qos, callback_group=self._cb_group)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, odom_qos, callback_group=self._cb_group)
        self.create_subscription(
            Path, plan_topic, self._on_plan, plan_qos, callback_group=self._cb_group)

        self._nav_client = ActionClient(
            self, NavigateToPose, nav_action, callback_group=self._cb_group)
        self._compute_client = ActionClient(
            self, ComputePathToPose, compute_action, callback_group=self._cb_group)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map_msg: Optional[OccupancyGrid] = None
        self._map_png_b64: Optional[str] = None
        self._map_version = 0
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._active_path: List[Tuple[float, float]] = []
        self._compare_paths: Dict[str, List[Tuple[float, float]]] = {}
        self._selected_planner = 'Shortest'
        self._goal: Optional[Tuple[float, float, float]] = None
        self._scenario_start: Optional[Tuple[float, float, float]] = None
        self._goal_source = 'free'  # free | scenario
        self._scenario_name: Optional[str] = None
        self._pair_name: Optional[str] = None
        self._nav_handle = None
        self._nav_status = 'idle'
        self._last_error = ''

    # --- subscriptions -------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            png = grid_to_png_bytes(msg.info.width, msg.info.height, msg.data)
            b64 = base64.b64encode(png).decode('ascii')
        except Exception as exc:  # noqa: BLE001 — keep UI up if map encode fails
            self.get_logger().error(f'Map PNG encode failed: {exc}')
            return
        with self._lock:
            self._map_msg = msg
            self._map_png_b64 = b64
            self._map_version += 1

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(pose.orientation.z, pose.orientation.w)
        with self._lock:
            self._odom_pose = (pose.position.x, pose.position.y, yaw)

    def _on_plan(self, msg: Path) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._active_path = downsample_path(points)

    # --- pose / status -------------------------------------------------

    def _tf_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        return (t.x, t.y, quaternion_to_yaw(q.z, q.w))

    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            source = self.pose_source
            odom = self._odom_pose
        if source == 'tf':
            return self._tf_pose()
        return odom

    def select_planner(self, planner_id: str) -> Dict[str, Any]:
        if planner_id not in PLANNER_IDS:
            return {'ok': False, 'error': f'Unknown planner: {planner_id}'}
        with self._lock:
            self._selected_planner = planner_id
        return {'ok': True, 'planner_id': planner_id}

    def set_goal(self, x: float, y: float, yaw: float) -> Dict[str, Any]:
        with self._lock:
            self._goal = (float(x), float(y), float(yaw))
            self._goal_source = 'free'
            self._scenario_start = None
            self._scenario_name = None
            self._pair_name = None
            self._compare_paths = {}
        return {'ok': True, 'goal': {'x': x, 'y': y, 'yaw': yaw}}

    def set_goal_from_scenario(self, scenario: str, pair_name: str) -> Dict[str, Any]:
        try:
            data = load_scenario(self.scenarios_dir, scenario)
        except (OSError, ValueError) as exc:
            return {'ok': False, 'error': str(exc)}
        pair = find_pair(data, pair_name)
        if pair is None:
            return {'ok': False, 'error': f'Unknown pair: {pair_name}'}
        start = tuple(float(v) for v in pair['start'])
        goal = tuple(float(v) for v in pair['goal'])
        with self._lock:
            self._scenario_start = start  # type: ignore[assignment]
            self._goal = goal  # type: ignore[assignment]
            self._goal_source = 'scenario'
            self._scenario_name = scenario
            self._pair_name = pair_name
            self._compare_paths = {}
        return {
            'ok': True,
            'start': {'x': start[0], 'y': start[1], 'yaw': start[2]},
            'goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2]},
            'map': data.get('map'),
        }

    def _make_pose(self, values: Tuple[float, float, float]) -> PoseStamped:
        x, y, yaw = values
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _wait_future(self, future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def drive(self) -> Dict[str, Any]:
        with self._lock:
            goal = self._goal
            planner = self._selected_planner
            if self._nav_status == 'active':
                return {'ok': False, 'error': 'Navigation already active'}
        if goal is None:
            return {'ok': False, 'error': 'No goal set'}
        try:
            tree = behavior_tree_path(self.behavior_trees_dir, planner)
        except ValueError as exc:
            return {'ok': False, 'error': str(exc)}
        if not os.path.isfile(tree):
            return {'ok': False, 'error': f'Behavior tree missing: {tree}'}

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'error': 'navigate_to_pose action server not available'}

        request = NavigateToPose.Goal()
        request.pose = self._make_pose(goal)
        request.behavior_tree = tree

        with self._lock:
            self._nav_status = 'sending'
            self._last_error = ''
            self._compare_paths = {}

        sent = self._nav_client.send_goal_async(request)
        if not self._wait_future(sent, 10.0):
            with self._lock:
                self._nav_status = 'idle'
            return {'ok': False, 'error': 'Timed out waiting for goal acceptance'}
        handle = sent.result()
        if handle is None or not handle.accepted:
            with self._lock:
                self._nav_status = 'idle'
            return {'ok': False, 'error': 'NavigateToPose goal rejected'}

        with self._lock:
            self._nav_handle = handle
            self._nav_status = 'active'

        result_future = handle.get_result_async()

        def _on_done(fut) -> None:
            try:
                response = fut.result()
                status = response.status if response is not None else GoalStatus.STATUS_UNKNOWN
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._nav_status = 'failed'
                    self._last_error = str(exc)
                    self._nav_handle = None
                return
            label = {
                GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                GoalStatus.STATUS_CANCELED: 'canceled',
                GoalStatus.STATUS_ABORTED: 'aborted',
            }.get(status, 'finished')
            with self._lock:
                self._nav_status = label
                self._nav_handle = None

        result_future.add_done_callback(_on_done)
        return {'ok': True, 'planner_id': planner, 'behavior_tree': tree}

    def cancel(self) -> Dict[str, Any]:
        with self._lock:
            handle = self._nav_handle
            status = self._nav_status
        if handle is None:
            return {'ok': False, 'error': f'Nothing to cancel (status={status})'}
        cancel_future = handle.cancel_goal_async()
        if not self._wait_future(cancel_future, 5.0):
            return {'ok': False, 'error': 'Cancel timed out'}
        with self._lock:
            self._nav_status = 'canceling'
        return {'ok': True}

    def compare(self) -> Dict[str, Any]:
        with self._lock:
            goal = self._goal
            goal_source = self._goal_source
            scenario_start = self._scenario_start
        if goal is None:
            return {'ok': False, 'error': 'No goal set'}

        if goal_source == 'scenario' and scenario_start is not None:
            start = scenario_start
        else:
            start = self.robot_pose()
            if start is None:
                return {'ok': False, 'error': 'Robot pose unknown'}

        if not self._compute_client.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'error': 'compute_path_to_pose action server not available'}

        start_pose = self._make_pose(start)
        goal_pose = self._make_pose(goal)
        paths: Dict[str, Any] = {}
        for planner_id in PLANNER_IDS:
            request = ComputePathToPose.Goal()
            request.start = start_pose
            request.goal = goal_pose
            request.planner_id = planner_id
            request.use_start = True
            sent = self._compute_client.send_goal_async(request)
            if not self._wait_future(sent, 15.0):
                paths[planner_id] = {'ok': False, 'error': 'send timeout', 'points': []}
                continue
            handle = sent.result()
            if handle is None or not handle.accepted:
                paths[planner_id] = {'ok': False, 'error': 'rejected', 'points': []}
                continue
            done = handle.get_result_async()
            if not self._wait_future(done, 30.0):
                paths[planner_id] = {'ok': False, 'error': 'result timeout', 'points': []}
                continue
            response = done.result()
            if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
                paths[planner_id] = {'ok': False, 'error': 'failed', 'points': []}
                continue
            points = [
                (p.pose.position.x, p.pose.position.y)
                for p in response.result.path.poses
            ]
            thin = downsample_path(points)
            paths[planner_id] = {
                'ok': True,
                'points': [{'x': x, 'y': y} for x, y in thin],
            }

        with self._lock:
            self._compare_paths = {
                pid: [(p['x'], p['y']) for p in entry['points']]
                for pid, entry in paths.items()
                if entry.get('ok')
            }
        return {'ok': True, 'paths': paths}

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'selected_planner': self._selected_planner,
                'nav_status': self._nav_status,
                'last_error': self._last_error,
                'goal_source': self._goal_source,
                'scenario': self._scenario_name,
                'pair': self._pair_name,
                'map_ready': self._map_msg is not None,
                'pose_ready': self.robot_pose() is not None,
                'navigate_ready': self._nav_client.server_is_ready(),
                'compute_ready': self._compute_client.server_is_ready(),
                'pose_source': self.pose_source,
            }

    def state(self) -> Dict[str, Any]:
        pose = self.robot_pose()
        with self._lock:
            map_version = self._map_version
            png = self._map_png_b64
            map_msg = self._map_msg
            goal = self._goal
            active = list(self._active_path)
            compare = {
                pid: [{'x': x, 'y': y} for x, y in pts]
                for pid, pts in self._compare_paths.items()
            }
            scenario_start = self._scenario_start

        map_info = None
        if map_msg is not None:
            info = map_msg.info
            map_info = {
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin': {
                    'x': info.origin.position.x,
                    'y': info.origin.position.y,
                    'yaw': quaternion_to_yaw(
                        info.origin.orientation.z, info.origin.orientation.w),
                },
                'version': map_version,
                'png_base64': png,
            }

        return {
            'map': map_info,
            'robot': (
                {'x': pose[0], 'y': pose[1], 'yaw': pose[2]} if pose else None),
            'goal': (
                {'x': goal[0], 'y': goal[1], 'yaw': goal[2]} if goal else None),
            'scenario_start': (
                {
                    'x': scenario_start[0],
                    'y': scenario_start[1],
                    'yaw': scenario_start[2],
                }
                if scenario_start else None),
            'active_path': [{'x': x, 'y': y} for x, y in active],
            'compare_paths': compare,
            'planners': planner_list(),
        }

    def scenarios(self) -> List[str]:
        return list_scenarios(self.scenarios_dir)

    def scenario_detail(self, name: str) -> Dict[str, Any]:
        try:
            data = load_scenario(self.scenarios_dir, name)
        except (OSError, ValueError) as exc:
            return {'ok': False, 'error': str(exc)}
        pairs = [
            {
                'name': pair['name'],
                'start': pair['start'],
                'goal': pair['goal'],
            }
            for pair in data.get('pairs', [])
        ]
        return {'ok': True, 'map': data.get('map'), 'pairs': pairs}
