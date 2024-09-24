#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool 
from robot_interfaces.srv import Commands
from sensor_msgs.msg import Joy
import numpy as np

class CommandNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.subscriber_ = self.create_subscription(Bool,'digging_commands_success',self.success,10) 
        self.publisher = self.create_publisher(Joy,"joy",10)
        self.timer_cmd = self.create_timer(0.1,self.get_command)
        self.get_logger().info("Command node activated, press 'r' for resting position, 'd' for discharge, 's' for scooping ")
        self.in_progress = False
        self.buttons = [0,0,0,0]
        self.axes = [0.0,0.0,0.0,0.0]
    
    def get_command(self):
        
        if self.in_progress == False:
            
            self.input_value = input()

            if self.input_value == 'r':
                self.buttons = [0,0,0,0]
                self.buttons[0] = 1
                self.send_command()
                self.in_progress = True
            if self.input_value == 'd':
                self.buttons = [0,0,0,0]
                self.buttons[1] = 1
                self.send_command()
                self.in_progress = True
            if self.input_value == 's':
                self.buttons = [0,0,0,0]
                self.buttons[2] = 1
                self.send_command()
                self.in_progress = True
        # else:
        #     self.send_command(self.command)
            
        
    def send_command(self):

        msg = Joy()
        msg.buttons = self.buttons
        msg.axes = self.axes
        self.publisher.publish(msg)
    
    def success(self,msg):
        succ = msg.data
        if succ == True:
            self.get_logger().info("Command executed")
        else:
            self.get_logger().info("Command not performed")
        self.in_progress = False

        
def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
