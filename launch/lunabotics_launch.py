from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='control_pkg', executable='testcontrol.py', output='screen'),
        Node(package='control_pkg', executable='drivetrain', output='screen'),
        Node(package='control_pkg', executable='scoop', output='screen')
        # etc.
    ])
