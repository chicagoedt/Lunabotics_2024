from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='control_pkg', executable='testcontrol.py', output='screen'),
        Node(package='motor_control_pkg', executable='drivetrain', output='screen'),
        Node(package='motor_control_pkg', executable='scoop', output='screen'),
        Node(package='brain_pkg', executable='brain.py', output='screen'),
        Node(package='control_pkg', executable='canable_start.sh', output='screen')
    ])
