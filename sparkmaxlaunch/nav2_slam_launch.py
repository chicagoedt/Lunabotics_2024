from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='nav2_bringup',
            output='screen',
            parameters=['path/to/nav2_params.yaml'],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            parameters=['path/to/slam_toolbox_params.yaml'],
        ),
        Node(
            package='<your_package_name>',
            executable='sparkmax_control_node',
            output='screen',
        ),
    ])
