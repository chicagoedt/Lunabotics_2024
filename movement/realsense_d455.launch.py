#jetson path: !/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # install this on the jetson, we are using this node
    #   https://github.com/IntelRealSense/realsense-ros 
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_d455',
            output='screen',
            parameters=[
                {
                    'enable_sync': True,
                    'enable_color': True,
                    'enable_depth': True,
                    'enable_pointcloud': True,
                    'depth_module.profile': '640x480x30',
                    'rgb_camera.profile': '640x480x30',
                }
            ]
        ),
    ])


# ros2 launch lunabot_launch realsense_d455.launch.py
