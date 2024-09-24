#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ActCommand, Commands
from robot_interfaces.msg import MotorsVel, LinActPos, GuiGraphInput
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
import numpy as np
import time

class NavigationNode(Node):

    def __init__(self):
        super().__init__("Navigation_node")

        # Establish publishers and listeners
        self.publisher_command = self.create_publisher(Bool,'digging_command',10) 
        self.command_pub_ = self.create_publisher(MotorsVel,'motor_vel',self.motor_callback,10) 
        self.navigation_cmd_sub = self.create_subscription(UInt8,'nav_command',self.navigation,10)        
        self.get_logger().info("Navigation algorithms node activated")
        self.timer_ = self.create_timer(0.1,self.main_algorithm)
        self.command = ''

    def navigation(self,msg):
        cmd = msg.data
        if cmd == 0:
            self.command = 'stop'
        if cmd == 1:
            self.command = 'turn_right'
        if cmd == 2:
            self.command = 'turn_left'
        if cmd == 3:
            self.command = 'forward'

    def main_algorithm(self):

        done = Bool()
        inprogress = Bool()
        command = String()


        ## Select command to act
        #   self.command == "restpos" ?
        #       true ==> 

        if self.command == "stop":
            
            motors_vel = [0.0,0.0]
            self.motor_vel_callback(motors_vel)
        
        elif self.command == "turn_right":
            
            motors_vel = []
            self.motor_vel_callback(motors_vel)

        elif self.command == "turn_left":
            motors_vel = []
            self.motor_vel_callback(motors_vel)
        
        elif self.command == "forward":
            motors_vel = []
            self.motor_vel_callback(motors_vel)
    
    
    def motor_vel_callback(self,motors_vel):

        request_motor = MotorsVel()
        request_motor.rl_motor = motors_vel
        self.command_pub_.publish(request_motor)

def main(args=None):
    rclpy.init(args=args)
    node = AutCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()

