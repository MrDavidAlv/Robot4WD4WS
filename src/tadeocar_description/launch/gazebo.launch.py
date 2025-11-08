#!/usr/bin/env python3
"""
TadeoeCar Gazebo Simulation
Launches Gazebo with TadeoeCar robot and ros2_control
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    pkg_description = get_package_share_directory('tadeocar_description')
    pkg_control = get_package_share_directory('tadeocar_control')

    model_sdf = os.path.join(pkg_description, 'models', 'tadeocar_v1', 'model.sdf')
    urdf_xacro = os.path.join(pkg_description, 'urdf', 'tadeocar_control.urdf.xacro')
    controllers_file = os.path.join(pkg_control, 'config', 'ros2_controllers.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_description, 'models')

    robot_description = xacro.process_file(
        urdf_xacro,
        mappings={'controllers_file': controllers_file}
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             PathJoinSubstitution([pkg_description, 'worlds', world]),
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    spawn_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-file', model_sdf,
             '-entity', 'tadeocar_v1',
             '-x', '0.0',
             '-y', '0.0',
             '-z', '0.2'],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    controller_spawners = []
    wheels = ['front_left', 'front_right', 'rear_left', 'rear_right']

    for wheel in wheels:
        controller_spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=[f'{wheel}_steering_controller'],
            output='screen'
        ))
        controller_spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=[f'{wheel}_wheel_controller'],
            output='screen'
        ))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value='slam_world.world',
            description='World file name (tadeocar.world or slam_world.world)'
        ),
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_robot,
        joint_state_broadcaster
    ] + controller_spawners)
