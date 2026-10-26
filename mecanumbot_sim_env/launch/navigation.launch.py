"""Full Nav2 stack for Mecanumbot: map, localization and navigation servers.

Expects something else (the simulator or the real robot) to publish
/mecanumbot/scan, /mecanumbot/odom and the mecanumbot/odom -> base_footprint
transform, and to listen on /mecanumbot/cmd_vel.

Velocity chain: controller -> /mecanumbot/cmd_vel_nav -> velocity_smoother
-> /mecanumbot/cmd_vel_smoothed -> collision_monitor -> /mecanumbot/cmd_vel.

    ros2 launch mecanumbot_sim_env navigation.launch.py map:=LED_exp localization:=amcl
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'mecanumbot_sim_env'
NAVIGATION_NODES = [
    'controller_server', 'smoother_server', 'planner_server', 'behavior_server',
    'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor',
]


def resolve_map(value):
    if value.endswith('.yaml'):
        return value
    return os.path.join(get_package_share_directory(PACKAGE), 'maps', value, value + '.yaml')


def launch_setup(context):
    share = get_package_share_directory(PACKAGE)
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    localization = LaunchConfiguration('localization').perform(context)
    map_yaml = resolve_map(LaunchConfiguration('map').perform(context))
    if localization not in ('amcl', 'none'):
        raise RuntimeError("localization must be 'amcl' or 'none'")
    if not os.path.isfile(map_yaml):
        raise RuntimeError('Map file not found: ' + map_yaml)

    common = {'use_sim_time': use_sim_time}
    trees = os.path.join(share, 'behavior_trees')

    def nav_node(package, executable, remappings=(), extra=None):
        return Node(
            package=package, executable=executable, name=executable,
            output='screen', respawn=False,
            parameters=[params_file, common, extra or {}],
            remappings=list(remappings),
        )

    actions = [
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[params_file, common, {'yaml_filename': map_yaml}],
        ),
    ]
    localization_nodes = ['map_server']
    if localization == 'amcl':
        actions.append(Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen', parameters=[params_file, common, {
                'set_initial_pose': True,
                'initial_pose.x': float(LaunchConfiguration('x').perform(context)),
                'initial_pose.y': float(LaunchConfiguration('y').perform(context)),
                'initial_pose.yaw': float(LaunchConfiguration('yaw').perform(context)),
            }],
        ))
        localization_nodes.append('amcl')
    else:
        # Odometry is exact in simulation, so map and odom can coincide.
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='tf_map_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'mecanumbot/odom'],
            parameters=[common],
        ))

    actions += [
        nav_node('nav2_controller', 'controller_server',
                 [('cmd_vel', '/mecanumbot/cmd_vel_nav')]),
        nav_node('nav2_smoother', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server',
                 [('cmd_vel', '/mecanumbot/cmd_vel_smoothed')]),
        nav_node('nav2_bt_navigator', 'bt_navigator', extra={
            'default_nav_to_pose_bt_xml': os.path.join(trees, 'navigate_to_pose_shortest.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(trees, 'navigate_through_poses.xml'),
        }),
        nav_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother',
                 [('cmd_vel', '/mecanumbot/cmd_vel_nav'),
                  ('cmd_vel_smoothed', '/mecanumbot/cmd_vel_smoothed')]),
        nav_node('nav2_collision_monitor', 'collision_monitor'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[common, {'autostart': True, 'node_names': localization_nodes}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[common, {'autostart': True, 'node_names': NAVIGATION_NODES}],
        ),
    ]
    return actions


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='AI_dept',
                              description='Map name in this package or a path to a map yaml'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization', default_value='none',
                              description="'none' (map = odom, exact sim odometry) or 'amcl'"),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial pose for AMCL'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])
