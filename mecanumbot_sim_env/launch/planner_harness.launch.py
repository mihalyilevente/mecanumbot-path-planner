"""Planner Server on a static map, without a simulator.

Starts map_server, planner_server (with its global costmap) and fixed TF frames
standing in for the robot, so global plans can be requested and compared
without any physics. Use with `ros2 run mecanumbot_sim_env plan_scenarios`.

    ros2 launch mecanumbot_sim_env planner_harness.launch.py map:=LED_exp rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mecanumbot_sim_env.launch_checks import refuse_if_running

PACKAGE = 'mecanumbot_sim_env'


def resolve_map(value):
    """Accept either a map name from this package (e.g. LED_exp) or a path to a map yaml."""
    if value.endswith('.yaml'):
        return value
    return os.path.join(get_package_share_directory(PACKAGE), 'maps', value, value + '.yaml')


def static_tf(parent, child, z='0.0'):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_' + child.replace('/', '_'),
        arguments=['--frame-id', parent, '--child-frame-id', child, '--z', z],
        output='log',
    )


def launch_setup(context):
    refuse_if_running()
    share = get_package_share_directory(PACKAGE)
    map_yaml = resolve_map(LaunchConfiguration('map').perform(context))
    if not os.path.isfile(map_yaml):
        raise RuntimeError('Map file not found: ' + map_yaml)

    params = [
        LaunchConfiguration('params_file').perform(context),
        os.path.join(share, 'config', 'harness_overrides.yaml'),
    ]

    return [
        # The robot never moves here; plans are requested with an explicit start pose.
        static_tf('map', 'mecanumbot/odom'),
        static_tf('mecanumbot/odom', 'mecanumbot/base_footprint'),
        static_tf('mecanumbot/base_footprint', 'mecanumbot/base_link', z='0.01'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params[0], {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=params,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_harness',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server']}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'harness.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='log',
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='AI_dept',
            description='Map name in this package (AI_dept, LED_exp) or a path to a map yaml'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml'),
            description='Full Nav2 parameter file'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        OpaqueFunction(function=launch_setup),
    ])
