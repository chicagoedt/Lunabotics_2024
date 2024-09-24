#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import ActCommand, Commands
from std_msgs.msg import Bool
import numpy as np
import time

class AutCommandNode(Node):
    def __init__(self):
        super().__init__("Automatic_Controller")
        self.command_server_ = self.create_service(Commands,'digging_commands',self.control_algorithm)
        self.publisher_vibr = self.create_publisher(Bool,'vibr',10) 
        self.publisher_success = self.create_publisher(Bool,'digging_commands_success',10) 
        self.subscriber_success_ = self.create_subscription(Bool,'PID_success',self.PID_success, 10)
        self.get_logger().info("Digging automatic algorithms node activated")
        self.command = ''
        self.timer_ = self.create_timer(0.1,self.main_algorithm)
        self.partial_done = False
        self.in_progress = False
        self.i = 0
        self.counter = 0
        self.command_received = 0
        

    #target has teh shape [14_inch,4_inch,motors,vibrator,time=None]
    def control_algorithm(self,request,response):
        self.command = request.command
        response.operation_success = True
        self.get_logger().info(str(response.operation_success))
        return response
    
    def main_algorithm(self):

        done = Bool()
        if self.command == "restpos":
            if not(self.in_progress):
                self.counter = 0
                linact = self.angle2inch([0.550,0.898])
                target = [linact[0],linact[1],0.0,False]
                self.get_logger().info(str(target))
                self.service_callback(target)
                self.in_progress = True
            if self.partial_done:
                self.get_logger().info("Rest position achieved")
                done.data = True
                self.publisher_success.publish(done)
                self.partial_done = False
                self.in_progress = False
                self.command = ''

        if self.command == "discharge":
            if not(self.in_progress):
                self.counter = 0
                linact = self.angle2inch([1.325,0.898])
                target = [linact[0],linact[1],0.0,False]
                self.service_callback(target)
                self.in_progress = True
            if self.partial_done:
                vibr = Bool()
                vibr.data = True
                self.publisher_vibr.publish(vibr)
                time.sleep(5)
                vibr.data = False
                self.publisher_vibr.publish(vibr)
                self.get_logger().info("Digging performed")
                done.data = True
                self.publisher_success.publish(done)
                self.in_progress = False
                self.partial_done = False
                self.command = ''
        
        if self.command == "scoop":
            
            target_angles = [[0.629,-0.507,0.0,False,0.0],[0.0,-0.138,0.0,True,0.0],[-0.138,-0.138,0.2,True,0.0],[0.550,0.898,0.2,True,0.0],[0.550,0.898,0.0,False,0.0]]
            if self.i == 0 or self.partial_done == True:
                self.counter = 0
                if self.i>= np.shape(target_angles)[0]:
                    self.i = 0
                    self.get_logger().info("Scooping performed")
                    done.data = True
                    self.publisher_success.publish(done)
                    self.in_progress = False
                    self.partial_done = False
                    self.command = ''
                else:
                    linact = self.angle2inch(target_angles[self.i][:2])
                    target = [linact[0],linact[1],target_angles[self.i][2],target_angles[self.i][3],target_angles[self.i][4]]
                    self.service_callback(target)
                    self.partial_done = False
                    self.i += 1
                    self.in_progress = True
                

    def angle2inch(self,angles):
        act_14 = np.sqrt(686.6443-453.2148*np.cos(angles[0]+1.3415))-19.5098
        act_4 = np.sqrt((3.1811*np.cos(angles[1])-1.9201)**2+(11.0201+3.1811*np.sin(angles[1]))**2)-9.5102
        return [act_14,act_4]

        
    def service_callback(self,target):
        
        liftact_client_ = self.create_client(ActCommand,'des_lift_pos')
        tiltact_client_ = self.create_client(ActCommand,'des_tilt_pos')
        motor_client_ = self.create_client(ActCommand,'des_motor_vel')
        vibr = Bool()

        while not liftact_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for lift act PID node...")
        
        while not tiltact_client_.wait_for_service(1.0):
           self.get_logger().warn("Waiting for tilt act PID node...")
        
        while not motor_client_.wait_for_service(1.0):
           self.get_logger().warn("Waiting for motor PID node...")

        if len(target) == 5:
            time = target[4]
        else:
            time = 0.0

        request_lift = ActCommand.Request()
        request_tilt = ActCommand.Request()
        request_motor = ActCommand.Request()
        request_lift.rl_motor = [target[0],time]
        request_tilt.rl_motor = [target[1],time]
        request_motor.rl_motor = [target[2],target[2]]
        vibr.data = target[3]

        future_lift = liftact_client_.call_async(request_lift)
        future_tilt = tiltact_client_.call_async(request_tilt)
        future_motor = motor_client_.call_async(request_motor) 
        self.publisher_vibr.publish(vibr)

        future_lift.add_done_callback(self.response_callback)
        future_tilt.add_done_callback(self.response_callback)
        future_motor.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.done}')
            if response.done:
                self.command_received +=1
            if self.command_received == 3:
                self.command_received = 0
                self.get_logger().info("Commands received")
            return response.done
        except Exception as e:
            self.get_logger().error(f'Exception in service call: {e}')

    def PID_success(self,msg):

        if msg.data:
            self.counter +=1

        if self.counter == 3:
            self.counter = 0
            self.partial_done = True


def main(args=None):
    rclpy.init(args=args)
    node = AutCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
