

import os
import string
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = get_package_share_directory('amr_navigation')
CONFIG = os.path.join(PKG, 'config')
MAPPING = os.path.join(CONFIG, 'mapping.yaml')
SIM_TIME = {'use_sim_time': True}


def fill_template(template, robot, settings):
    """Copy a *_template.yaml with ${robot}, ${max_vel_x}, ... replaced (values: fleet.yaml)."""
    values = {k: str(v).lower() if isinstance(v, bool) else v for k, v in settings.items()}
    with open(os.path.join(CONFIG, template), 'r') as f:
        text = string.Template(f.read()).substitute(values, robot=robot)
    path = os.path.join(tempfile.gettempdir(), f'{robot}_{template}')
    with open(path, 'w') as f:
        f.write(text)
    return path


def robot_nodes(robot, settings):
    slam_params = fill_template('slam_template.yaml', robot, settings)
    nav2_params = fill_template('nav2_template.yaml', robot, settings)
    # Nav2's velocity output (normally /<robot>/cmd_vel). Section 3 renames it to cmd_vel_nav
    # and puts the motion smoother + safety node between Nav2 and the wheels.
    cmd_vel = [('cmd_vel', LaunchConfiguration('nav_cmd_vel'))]
    nav2 = [Node(package=f'nav2_{pkg}', executable=exe, namespace=robot,
                 parameters=[nav2_params], remappings=cmd_vel, output='screen')
            for pkg, exe in [('controller', 'controller_server'),
                             ('planner', 'planner_server'),
                             ('behaviors', 'behavior_server'),
                             ('bt_navigator', 'bt_navigator'),
                             ('waypoint_follower', 'waypoint_follower')]]
    nav2.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_navigation', namespace=robot,
                     parameters=[nav2_params], output='screen'))
    return [
        # The robot's SLAM map frame sits exactly on the shared 'map' frame: both robots start
        # at known poses and their odometry is measured in the world frame.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', f'{robot}/map'],
             parameters=[SIM_TIME]),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             namespace=robot, parameters=[slam_params], output='screen'),
        TimerAction(period=8.0, actions=nav2),   # Nav2 once Gazebo and SLAM are running
    ]


def generate_launch_description():
    with open(os.path.join(CONFIG, 'fleet.yaml'), 'r') as f:
        fleet = yaml.safe_load(f)['robots']

    actions = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav_cmd_vel', default_value='cmd_vel'),
        # Section 1 simulation (without its temporary world -> odom transforms: SLAM does that)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('amr_gazebo'), 'launch', 'sim.launch.py')),
            launch_arguments={'headless': LaunchConfiguration('headless'),
                              'rviz': 'false', 'world_tf': 'false'}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'map'],
             parameters=[SIM_TIME]),
        # our nodes
        Node(package='amr_navigation', executable='selective_map_filter.py',
             namespace='amr1', parameters=[MAPPING], output='screen'),
        Node(package='amr_navigation', executable='map_fusion.py',
             parameters=[MAPPING], output='screen'),
        Node(package='amr_navigation', executable='terrain_cost.py',
             parameters=[MAPPING, {'terrain_file': os.path.join(CONFIG, 'terrain.yaml')}],
             output='screen'),
        Node(package='amr_navigation', executable='fleet_scan.py',
             parameters=[SIM_TIME], output='screen'),
        Node(package='rviz2', executable='rviz2',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(PKG, 'rviz', 'navigation.rviz')],
             parameters=[SIM_TIME]),
    ]
    for robot, settings in fleet.items():
        actions += robot_nodes(robot, settings)
    return LaunchDescription(actions)
