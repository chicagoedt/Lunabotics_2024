#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MotorsVel
from robot_interfaces.srv import ActCommand
from std_msgs.msg import Bool
import time

class PIDMotorNode(Node):

    def __init__(self):
        
        super().__init__("PID_motor")

        self.declare_param()
        
        self.get_param()

        self.subscriber_feedback = self.create_subscription(MotorsVel,self.fb_topic,self.get_feedback,10) 
        self.publisher_command = self.create_publisher(MotorsVel,self.cmd_topic,10)
        self.server_command = self.create_service(ActCommand,self.cmd_server,self.callback_des_pos)
        self.publisher_success_ = self.create_publisher(Bool,'PID_success',10)
        self.STOP_sub = self.create_subscription(Bool,'STOP',self.reset,10)
        #self.timer_ = self.create_timer(0.1,self.main_algorithm)

        self.feedback = [0.0,0.0]
        self.command = [0.0,0.0]
        self.des_vel = [0.0,0.0] 

        self.get_logger().info("Motor controller node activated")

    def declare_param(self):

        self.declare_parameter('feedback_topic','motor_feedback')
        self.declare_parameter('command_topic','motor_vel')
        self.declare_parameter('command_server','des_motor_vel')
        self.declare_parameter('range',14)
        self.declare_parameter('feedback_off_range',[255,543,224,579])
        self.declare_parameter('PID',[1.0,0.0,0.0])

    def get_param(self):

        self.fb_topic = self.get_parameter('feedback_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.cmd_server = self.get_parameter('command_server').get_parameter_value().string_value
        self.range = self.get_parameter('range').get_parameter_value().integer_value
        self.fb_off_range = self.get_parameter('feedback_off_range').get_parameter_value().integer_array_value
        self.PID = self.get_parameter('PID').get_parameter_value().double_array_value
        self.time = 0.0
        self.timer_ = self.create_timer(0.1,self.main)
    
    def get_feedback(self,msg):
        self.feedback = msg.rl_motor

    def callback_des_pos(self,request,response):
        
        self.time = time.time()
        self.des_vel = request.rl_motor
        command = MotorsVel()
        done = Bool()
        feedback = self.feedback
        
        #while self.des_vel != feedback:
            
            #self.command = [self.PID[0]*(self.des_vel[i]-feedback[i]) for i in range(2)]
        command.rl_motor = self.des_vel
        self.publisher_command.publish(command) 
        done.data = True
        self.publisher_success_.publish(done)
            #time.sleep(1)
            #feedback = self.feedback
            
        response.done = True
        return response

    def main(self):
        if self.time != 0.0:
            timer = time.time()
            if timer-self.time > 10.0:
                command = MotorsVel()
                command.rl_motor = [0.0,0.0]
                self.publisher_command.publish(command)
                self.time = 0.0

    def reset(self,msg):
        stop = msg.data
        if stop:
            command = MotorsVel()
            request = [0.0,0.0]
            command.rl_motor = request
            self.publisher_command.publish(command) 



def main(args=None):
    rclpy.init(args=args)
    node = PIDMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
