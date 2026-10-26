"""Mecanumbot in Gazebo Fortress with the full Nav2 stack.

The Gazebo world is built from the map at launch time (see make_world.py), so
the walls the lidar sees are exactly the obstacles in the map. The robot is
spawned at x/y/yaw; if those are not given it starts at the start pose of the
first pair in the map's scenario file.

Gazebo publishes /mecanumbot/scan, /mecanumbot/odom and the
mecanumbot/odom -> base_footprint transform, and listens on
/mecanumbot/cmd_vel. Everything else is navigation.launch.py.

    ros2 launch mecanumbot_sim_env sim.launch.py map:=LED_exp
    ros2 launch mecanumbot_sim_env sim.launch.py map:=AI_dept x:=1.0 y:=2.0 yaw:=1.57 gui:=true
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from mecanumbot_sim_env.launch_checks import refuse_if_running
from mecanumbot_sim_env.make_world import world_sdf

PACKAGE = 'mecanumbot_sim_env'


def resolve_map(value):
    if value.endswith('.yaml'):
        return value
    return os.path.join(get_package_share_directory(PACKAGE), 'maps', value, value + '.yaml')


def default_spawn(share, map_name):
    """Start pose of the first scenario pair as (x, y, yaw), or the origin if there is none."""
    scenario = os.path.join(share, 'scenarios', map_name + '.yaml')
    if os.path.isfile(scenario):
        with open(scenario) as stream:
            pairs = yaml.safe_load(stream).get('pairs') or []
        if pairs:
            x, y, yaw = (list(pairs[0]['start']) + [0.0])[:3]
            return str(x), str(y), str(yaw)
    return '0.0', '0.0', '0.0'


def static_tf(parent, child, z):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_' + child.replace('/', '_'),
        arguments=['--frame-id', parent, '--child-frame-id', child, '--z', z],
        parameters=[{'use_sim_time': True}],
        output='log',
    )


def launch_setup(context):
    refuse_if_running()
    share = get_package_share_directory(PACKAGE)
    map_yaml = resolve_map(LaunchConfiguration('map').perform(context))
    if not os.path.isfile(map_yaml):
        raise RuntimeError('Map file not found: ' + map_yaml)
    map_name = os.path.splitext(os.path.basename(map_yaml))[0]

    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    if not x or not y:
        x, y, default_yaw = default_spawn(share, map_name)
        yaw = yaw or default_yaw
    yaw = yaw or '0.0'

    world_dir = os.path.join(tempfile.gettempdir(), PACKAGE)
    os.makedirs(world_dir, exist_ok=True)
    world_file = os.path.join(world_dir, map_name + '.sdf')
    with open(world_file, 'w') as stream:
        stream.write(world_sdf(map_yaml, world_name='sim'))

    gazebo_cmd = ['ign', 'gazebo', '-r', world_file]
    if LaunchConfiguration('gui').perform(context).lower() != 'true':
        gazebo_cmd.insert(2, '-s')

    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_mecanumbot',
        output='screen',
        arguments=['-world', 'sim', '-name', 'mecanumbot',
                   '-file', os.path.join(share, 'models', 'mecanumbot', 'model.sdf'),
                   '-x', x, '-y', y, '-z', '0.0', '-Y', yaw],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': LaunchConfiguration('params_file').perform(context),
            'use_sim_time': 'true',
            'localization': LaunchConfiguration('localization').perform(context),
            'x': x, 'y': y, 'yaw': yaw,
        }.items(),
    )

    actions = [
        ExecuteProcess(cmd=gazebo_cmd, output='screen'),
        spawn,
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/mecanumbot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/mecanumbot/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/mecanumbot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/mecanumbot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            ],
            remappings=[('/mecanumbot/tf', '/tf')],
        ),
        # The rest of the robot's tree, same offsets as the URDF.
        static_tf('mecanumbot/base_footprint', 'mecanumbot/base_link', '0.01'),
        static_tf('mecanumbot/base_link', 'mecanumbot/base_scan', '0.122'),
        # Nav2 starts once the robot exists, so /clock is already running and
        # the first transforms are there when the servers configure.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[navigation])),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')],
            parameters=[{'use_sim_time': True}],
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='AI_dept',
                              description='Map name in this package or a path to a map yaml'),
        DeclareLaunchArgument('x', default_value='',
                              description='Spawn x in the map frame (default: first scenario start)'),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('localization', default_value='none',
                              description="'none' (map = odom) or 'amcl'"),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Open the Gazebo window (slow without a GPU)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
