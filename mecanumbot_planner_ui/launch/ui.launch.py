"""Launch the planner criteria browser UI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('mecanumbot_planner_ui')
    params = os.path.join(share, 'config', 'ui_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8090'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('pose_source', default_value='odom'),
        Node(
            package='mecanumbot_planner_ui',
            executable='planner_ui',
            name='planner_ui',
            output='screen',
            parameters=[
                params,
                {
                    'host': LaunchConfiguration('host'),
                    'port': LaunchConfiguration('port'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'pose_source': LaunchConfiguration('pose_source'),
                },
            ],
        ),
    ])
