import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOTS = ['amr1', 'amr2']


def generate_launch_description():
    params = os.path.join(get_package_share_directory('amr_control'), 'config', 'control.yaml')
    navigation = os.path.join(get_package_share_directory('amr_navigation'), 'launch',
                              'navigation.launch.py')
    actions = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Section 2, but Nav2 publishes on cmd_vel_nav instead of driving the wheels directly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation),
            launch_arguments={'headless': LaunchConfiguration('headless'),
                              'rviz': LaunchConfiguration('rviz'),
                              'nav_cmd_vel': 'cmd_vel_nav'}.items()),
        Node(package='amr_control', executable='traffic_control.py', parameters=[params],
             output='screen'),
    ]
    for robot in ROBOTS:
        actions += [
            Node(package='amr_control', executable='motion_smoother.py', namespace=robot,
                 parameters=[params], output='screen'),
            Node(package='amr_control', executable='safety_stop.py', namespace=robot,
                 parameters=[params], output='screen'),
        ]
    return LaunchDescription(actions)
