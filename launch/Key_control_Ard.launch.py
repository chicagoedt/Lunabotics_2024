import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

#we want to launch 3 nodes: the Arduino Node, the Motor Controller and the Joystick

def generate_launch_description():

    # spawn Arduino node from the py_pkg pkg

    Arduino = Node(package='py_pkg',
                 executable= 'Arduino',
                 output = 'screen'
                 )

    MotorController = Node(package='py_pkg',
                 executable= 'Motor_Controller',
                 output = 'screen'
                 )
        
    #launch all 3 
    return LaunchDescription([
            MotorController,
            Arduino
    ])