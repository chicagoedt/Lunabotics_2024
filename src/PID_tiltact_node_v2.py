#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MotorsVel,LinActPos
from robot_interfaces.srv import ActCommand
from std_msgs.msg import Bool
import time

class PIDLinActNode(Node):

    def __init__(self):
        
        super().__init__("PID_tilt")

        self.declare_param()

        self.get_param()

        self.subscriber_feedback = self.create_subscription(LinActPos,self.fb_topic,self.get_feedback,10) 
        self.publisher_command = self.create_publisher(MotorsVel,self.cmd_topic,10)
        self.server_command = self.create_service(ActCommand,self.cmd_server,self.callback_des_pos)
        self.publisher_success_ = self.create_publisher(Bool,'PID_success',10)
        self.timer_ = self.create_timer(0.1,self.main_algorithm)

        self.feedback = [0.0,0.0]
        self.command = [0.0,0.0]
        self.des_pos = 0.0
        self.toll = 0.1
        self.adjust = 0.0
        self.diff = 0.0
        self.time = 0.0
        self.in_progress = False
        self.reached = [False,False]
        self.timeout = 0.0
        self.enter_stop = 0
        self.internal_check = 0
        self.fb_memory = 0.0

        self.get_logger().info("PID tilt actuator node activated")
        self.get_logger().info(str(self.PID))

    def declare_param(self):

        self.declare_parameter('feedback_topic','tilt_feedback_filt')
        self.declare_parameter('command_topic','tilt_vel')
        self.declare_parameter('command_server','des_tilt_pos')
        self.declare_parameter('range',4)
        self.declare_parameter('feedback_off_range',[64,868,64,868])
        self.declare_parameter('PID',[1.0,0.0,0.0])
        self.declare_parameter('v_max',2.0)

    def get_param(self):

        self.fb_topic = self.get_parameter('feedback_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.cmd_server = self.get_parameter('command_server').get_parameter_value().string_value
        self.range = self.get_parameter('range').get_parameter_value().integer_value
        self.fb_off_range = self.get_parameter('feedback_off_range').get_parameter_value().integer_array_value
        self.PID = self.get_parameter('PID').get_parameter_value().double_array_value
        self.vmax = self.get_parameter('v_max').get_parameter_value().double_value

    def get_feedback(self,msg):

        fb = msg.rl_lin_act[1]
        feedback = (fb-self.fb_off_range[2])/(self.fb_off_range[3])*self.range 
        self.feedback = [feedback, feedback] #in inches
        if self.internal_check == 0:
            self.fb_memory = feedback
        self.internal_check += 1
        if self.internal_check == 10:
            self.internal_check = 0
        #self.get_logger().info("feedback:" + str(self.feedback))

    def callback_des_pos(self,request,response):
        
        if len(request.rl_motor) !=2:
            self.get_logger().error("Wrong data length, command and time needed")
            response.done = False
            return response
        
        self.des_pos = request.rl_motor[0]
        self.time = request.rl_motor[1]
        self.reached = [False,False]
        #self.get_logger().info("Command received:" + str(self.des_pos))
        self.in_progress = True
        if self.time != 0.0:
            self.maxv = abs(self.des_pos-self.feedback[0])/self.time
        else:
            self.maxv = self.vmax

        response.done = True
        return response
    
    def main_algorithm(self):

        command = MotorsVel()
        success = Bool()

        if self.in_progress:
            
            c = [abs(elem-self.des_pos) < self.toll for elem in self.feedback]
            #self.get_logger().info(self.fb_topic + str(c))

            if abs(self.fb_memory - self.feedback[0])<3:
                self.reached = [True,True]
            
            if all(self.reached) or self.des_pos == -1.0 :
                command.rl_motor = [0.0,0.0]
                success.data = True
                self.publisher_command.publish(command) 
                self.publisher_success_.publish(success)
                self.in_progress = False

            else:
                time = -1
                if c[0]:
                    self.command[0] = 0.0
                    self.reached[0] = True
                    if self.enter_stop == 0:
                        self.timeout = time.time()
                        self.enter_stop = 1
                    time = time.time()
                else:
                    self.command[0] = self.PID[0]*(self.des_pos-self.feedback[0]) - self.adjust*self.diff
                if c[1]:
                    self.command[1] = 0.0
                    self.reached[1] = True
                    if self.enter_stop == 0:
                        self.timeout = time.time()
                        self.enter_stop = 1
                    time = time.time()
                else:
                    self.command[1] = self.PID[0]*(self.des_pos-self.feedback[1]) + self.adjust*self.diff

                if time-self.timeout > 1.5:
                    c = [True, True]
                    self.command = [0.0,0.0]
                    self.timeout = 0
                    success.data = True
                    self.in_progress = False

                if abs(self.command[0])>self.maxv:
                    self.command[0] = self.command[0]/abs(self.command[0])*self.maxv
                if abs(self.command[1])>self.maxv:
                    self.command[1] = self.command[1]/abs(self.command[1])*self.maxv
                    
                command.rl_motor = self.command
                self.publisher_command.publish(command) 
                #self.get_logger().info("feedback_while:" + str(self.feedback))


def main(args=None):
    rclpy.init(args=args)
    node = PIDLinActNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()