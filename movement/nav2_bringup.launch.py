#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map_path = os.path.join(
        get_package_share_directory('my_robot_launch'),
        'maps',
        'test_map.yaml'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map file to load'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map': LaunchConfiguration('map')
            }.items()
        ),
    ])
 

 # ros2 launch lunabot nav2_bringup.launch.py
