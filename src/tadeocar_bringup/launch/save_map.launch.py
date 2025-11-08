#!/usr/bin/env python3
"""
Save SLAM Map
Saves the current map to tadeocar_navigation/maps/mapa
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_navigation = get_package_share_directory('tadeocar_navigation')
    maps_dir = os.path.join(pkg_navigation, 'maps')
    map_path = os.path.join(maps_dir, 'mapa')

    save_map = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_path],
        output='screen'
    )

    return LaunchDescription([
        save_map
    ])
