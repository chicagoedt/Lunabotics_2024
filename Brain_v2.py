#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool 
from robot_interfaces.srv import Commands
import numpy as np

class CommandNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.subscriber_ = self.create_subscription(Bool,'digging_commands_success',self.success,10) 
        #self.timer_key = self.create_timer(0.1,self.get_key)
        self.timer_cmd = self.create_timer(0.1,self.get_command)
        self.get_logger().info("Command node activated, press 'r' for resting position, 'd' for discharge, 's' for scooping ")
        self.in_progress = False
        self.command = ''

    # def get_key(self):
    #     self.input_value = input()
    
    def get_command(self):
        
        if self.in_progress == False:
            
            self.input_value = input()

            if self.input_value == 'r':
                self.command = 'restpos'
                self.send_command(self.command)
                self.in_progress = True
            if self.input_value == 'd':
                self.command = 'discharge'
                self.send_command(self.command)
                self.in_progress = True
            if self.input_value == 's':
                self.command = 'scoop'
                self.send_command(self.command)
                self.in_progress = True
        # else:
        #     self.send_command(self.command)
            
        
    def send_command(self,command_str):

        commander = self.create_client(Commands,'digging_commands')

        while not commander.wait_for_service(1.0):
            self.get_logger().warn("Waiting for automatic controller node...")

        request_ = Commands.Request()
        request_.command = command_str
        future = commander.call_async(request_)
        done = future.add_done_callback(self.response_callback)


    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.operation_success}')
            if response.operation_success == True:
                self.get_logger().info("Command obtained")
            if response.operation_success == False:
                self.get_logger().info("Command failed")
                self.in_progress = False
            return response.operation_success
        except Exception as e:
            self.get_logger().error(f'Exception in service call: {e}')
    
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
