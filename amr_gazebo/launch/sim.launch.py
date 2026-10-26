

import os
import shutil
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FLOOR_Z = 0.0342  # the AWS floor surface is at z = 0.0342, not 0

# The fleet. Add a robot = add one line here.
ROBOTS = [
    # AMR-1: heavy lead / mapper (Waffle Pi body) at the Base, facing north
    {'name': 'amr1', 'sdf': 'amr1.sdf.xacro', 'urdf': 'turtlebot3_waffle_pi.urdf',
     'x': -3.70, 'y': -9.40, 'yaw': 1.5708},
    # AMR-2: light scout (Burger body) at the Scout Dock, facing south
    {'name': 'amr2', 'sdf': 'amr2.sdf.xacro', 'urdf': 'turtlebot3_burger.urdf',
     'x': 1.70, 'y': 9.20, 'yaw': -1.5708},
]


def gazebo_media_dir():
    """Gazebo's own resource folder: /usr/share/gazebo-11 on Ubuntu."""
    gzserver = shutil.which('gzserver') or '/usr/bin/gzserver'
    return os.path.join(os.path.dirname(os.path.dirname(gzserver)), 'share', 'gazebo-11')


def robot_nodes(robot):
    """Everything one robot needs: spawn into Gazebo + TF."""
    name = robot['name']

    # Gazebo model: our SDF with every frame prefixed by the robot name,
    # written to a temporary file because spawn_entity.py reads a file
    sdf_path = os.path.join(get_package_share_directory('amr_description'), 'sdf', robot['sdf'])
    sdf_xml = xacro.process_file(sdf_path, mappings={'robot_name': name}).toxml()
    spawn_file = os.path.join(tempfile.gettempdir(), f'{name}.sdf')
    with open(spawn_file, 'w') as f:
        f.write(sdf_xml)

    # URDF for robot_state_publisher (TF of the fixed parts: base_link, base_scan, ...)
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    with open(os.path.join(tb3_share, 'urdf', robot['urdf']), 'r') as f:
        urdf_xml = f.read()

    return [
        # -robot_namespace puts all of this robot's Gazebo plugins under /amrX
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', name, '-file', spawn_file, '-robot_namespace', name,
                        '-x', str(robot['x']), '-y', str(robot['y']),
                        '-z', str(FLOOR_Z + 0.01), '-Y', str(robot['yaw']),
                        '-timeout', '120']),
        # BSP layer: the Gazebo sensor plugins publish scan_raw / imu_raw; only data that passes
        # this check reaches /amrX/scan and /amrX/imu (and with that SLAM, Nav2, ...)
        Node(package='amr_bsp', executable='sensor_validator.py', namespace=name,
             parameters=[os.path.join(get_package_share_directory('amr_bsp'), 'config',
                                      'sensor_limits.yaml')],
             output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=name,
             parameters=[{'robot_description': urdf_xml,
                          'frame_prefix': f'{name}/',        # amr1/base_link, amr1/base_scan, ...
                          'use_sim_time': True}]),
        # TurtleBot3's Gazebo odometry is measured in the world frame, so world -> amrX/odom
        # is the identity. Only without SLAM (world_tf:=false when SLAM provides map -> odom).
        Node(package='tf2_ros', executable='static_transform_publisher',
             name=f'{name}_world_to_odom',
             arguments=['--frame-id', 'world', '--child-frame-id', f'{name}/odom'],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('world_tf'))),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('amr_gazebo')
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    world = os.path.join(pkg, 'worlds', 'amr_warehouse.world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')

    actions = [
        DeclareLaunchArgument('headless', default_value='false', description='no Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world_tf', default_value='true',
                              description='static world -> amrX/odom (set false when SLAM runs)'),

        # gazebo_ros puts only the packages' media folders on GAZEBO_RESOURCE_PATH, which
        # hides Gazebo's own media folder: gzclient then crashes ("Assertion `px != 0'").
        # Same effect as `source /usr/share/gazebo/setup.sh`.
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_media_dir()),

        # Gazebo server (physics + sensors) and client (the window).
        # gazebo_params.yaml: /clock at 100 Hz instead of 10 Hz, so sim-time timers run smoothly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzserver.launch.py')),
            launch_arguments={'world': world, 'server_required': 'true',
                              'params_file': os.path.join(pkg, 'config', 'gazebo_params.yaml')
                              }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzclient.launch.py')),
            condition=UnlessCondition(headless)),

        Node(package='amr_gazebo', executable='moving_obstacles.py', output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'obstacles.yaml'),
                          'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', os.path.join(pkg, 'rviz', 'sim.rviz')],
             parameters=[{'use_sim_time': True}]),
    ]
    for robot in ROBOTS:
        actions += robot_nodes(robot)
    return LaunchDescription(actions)
