#!/usr/bin/env python3

import rclpy
import serial
import struct
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from robot_interfaces.msg import SensorsFeedback
import threading
import time

class ArduinoNode(Node):

    def __init__(self):
        super().__init__("motor_test")
        Arduino_port = '/dev/ttyUSB0'
        rate = 9600
        self.ser = serial.Serial(Arduino_port,rate)
        self.motor = [int(0),int(0),int(0),int(0),int(0),int(0)] #list of 6 components: right wheel, left wheel, lift1, lift2, tilt, vibr
        self.command = [int(0),int(0),int(0),int(0),int(0),int(0)] #list of 6 components: right wheel, left wheel, lift1, lift2, tilt, vibr
        self.feedback = [0.0,0.0,0.0]
        self.offset = np.array([188,93])  #490 Hz and 245 Hz
        forward_max = np.array([250,149])
        backward_max = np.array([130,37])
        self.forw_range = forward_max-self.offset
        self.back_range = self.offset-backward_max
        self.subscriber_motor1 = self.create_subscription(Float32,"motor_1",lambda msg: self.cmdvelcallback(msg,motor=0, max_vel=1,min_vel=1,freq=0),10)
        self.subscriber_motor2 = self.create_subscription(Float32,"motor_2",lambda msg: self.cmdvelcallback(msg,motor=1,max_vel=1,min_vel=1,freq=0),10)
        self.subscriber_lift1 = self.create_subscription(Float32,"lift_1",lambda msg: self.cmdvelcallback(msg,motor=2, max_vel=1,min_vel=1,freq=0),10)
        self.subscriber_lift2 = self.create_subscription(Float32,"lift_2",lambda msg: self.cmdvelcallback(msg,motor=3, max_vel=1,min_vel=1,freq=0),10)
        self.subscriber_tilt = self.create_subscription(Float32,"tilt",lambda msg: self.cmdvelcallback(msg,motor=4,max_vel=1,min_vel=1,freq=1),10)
        self.subscriber_vibr = self.create_subscription(Float32,"vibr",lambda msg: self.cmdvelcallback(msg,motor=5,max_vel=1,min_vel=1,freq=1),10)
        self.publisher_fb = self.create_publisher(SensorsFeedback,'sensor_feedback',10)
        self.timer_command = self.create_timer(0.1,self.Arduino_command)
        #self.thread = threading.Thread(target=self.Arduino_feedback)
        #self.thread.daemon = True
        self.get_logger().info("Arduino node activated")
        self.file = 'feedback.txt'

    def cmdvelcallback(self,msg,motor,max_vel,min_vel, freq):
        #limit the values between -1 and 1
        desw = msg.data
        desw = desw*((desw>0)/max_vel+(desw<0)/min_vel) #take the percentage of the maximum velocity
        desw = int(desw*(self.forw_range[freq]*(desw>0)+self.back_range[freq]*(desw<0)))+self.offset[freq] #compute the PWM value
        self.motor[motor] = desw
        self.command[motor] = desw
        
    def Arduino_command(self):
        
        self.command[2] = self.motor[2]-int((self.feedback[1]-self.feedback[0])/2.5)
        self.command[3] = self.motor[3]-int((self.feedback[0]-self.feedback[1])/2.5)
        if abs(self.command[2]-188)>62:
            self.command[2] = 188 + np.sign(self.motor[2]-188)*62
        if abs(self.command[3]-188)>62:
            self.command[3] = 188 + np.sign(self.motor[3]-188)*62
        packed_data = struct.pack('6B',*self.command)
        self.ser.write(packed_data)
        self.get_logger().info('Command:'+str(self.command) + str(self.ser.in_waiting))  
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()
            values = line.split(',')
            command_values = [value for value in values[:6]]
            actuators_feedback = [int(value) for value in values[6:9]]
            motors_feedback = [value for value in values[9:]]
            self.get_logger().info('act_feedback' + str(actuators_feedback))  
            msg = SensorsFeedback()
            msg.right_lin_act = int(actuators_feedback[0])
            msg.left_lin_act = int(actuators_feedback[1])
            msg.tilt_act = int(actuators_feedback[2]) 
            self.publisher_fb.publish(msg)
            self.feedback[0]=actuators_feedback[0]
            self.feedback[1]=actuators_feedback[1]
            self.feedback[2]=actuators_feedback[2]
            with open(self.file,'w') as file:
                text = str(actuators_feedback) + '\n'
                file.write(text)
                
    def start_threading(self):
        self.thread.start()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    #node.start_threading()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
    
