#!/usr/bin/env python3
"""
TadeoeCar SLAM
Launches Gazebo, Xbox control, and SLAM Toolbox with RViz2
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_navigation = get_package_share_directory('tadeocar_navigation')

    slam_params_file = os.path.join(pkg_navigation, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_navigation, 'config', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui
        }.items()
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )

    xbox_control = Node(
        package='tadeocar_control',
        executable='xbox_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    fourws_kinematics = Node(
        package='tadeocar_control',
        executable='fourws_kinematics',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo_launch,
        joy_node,
        xbox_control,
        fourws_kinematics,
        slam_toolbox,
        rviz
    ])
