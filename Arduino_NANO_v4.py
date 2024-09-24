#!/usr/bin/env python3

import rclpy
import serial
import struct
import numpy as np
from rclpy.node import Node
from robot_interfaces.msg import LinActPos, ArduinoCommandsFeedback, MotorsVel, ArduinoCommands
from std_msgs.msg import Bool,String
import time

#the Arduino node takes the commands from the arduino_commands topic, evaluate the PWM signal and gives the given command and the feedback back
class ArduinoNode(Node):

    def __init__(self):
        super().__init__("Arduino_Nano_Node")
        Arduino_port = '/dev/ttyUSB0'
        rate = 9600
        self.ser = serial.Serial(Arduino_port,rate)
        self.motor = [int(0),int(0),int(0),int(0),int(0), int(0)] #list of 6 components:start, right wheel, left wheel, lift r, lift l, tilt r, tilt l, vibr
        self.test = 0
        #self.ser.close()
        #time.sleep(3)
        #self.ser.open()
        self.ON_scoop = 240  #490 Hz and 245 Hz
        self.ON_dump = 210
        self.vibr = 250
        #self.forw_range = forward_max-self.offset
        #self.back_range = self.offset-backward_max
        self.subscriber_command = self.create_subscription(Bool,"vibr",self.cmdvelcallback,10)
        self.publisher_lift_fb = self.create_publisher(LinActPos,"lift_feedback",10)
        self.publisher_tilt_fb = self.create_publisher(LinActPos,"tilt_feedback",10)
        self.publisher_motor_fb = self.create_publisher(MotorsVel,"motor_feedback",10)
        self.publisher_command_fb = self.create_publisher(ArduinoCommandsFeedback,'given_commands_2',10)
        #self.pub_mode = self.create_subscription(String,"aut_command",self.phase_now,10)
        self.timer_command = self.create_timer(0.1,self.Arduino_command)
        self.get_logger().info("Arduino node activated")
        self.initialize()

    def cmdvelcallback(self,msg):
        #the values are between -1 and 1 create the PWM signals
        commands = msg.data
        motor = commands*self.vibr
        
        self.get_logger().info(str(motor))
        r_motor = motor           #compute the PWM value
        l_motor = motor           #compute the PWM value
        r_lin_act = motor         #compute the PWM value
        l_lin_act = motor         #compute the PWM value
        #l_tilt_act = int(commands[4]*(self.forw_range[1]*(commands[4]>0)+self.back_range[1]*(commands[4]<0)))+self.offset[1]        #compute the PWM value
        r_tilt_act = motor        #compute the PWM value
        l_tilt_act = motor        #compute the PWM value
        vibr = motor              #compute the PWM value
        self.motor = [r_motor, l_motor,r_lin_act,l_lin_act, r_tilt_act, l_tilt_act]

    def initialize(self):
        packed_data = struct.pack('6B',*self.motor)
        self.ser.write(packed_data)

    def Arduino_command(self):
        
        #self.get_logger().info('Command:'+str(self.motor) + str(self.ser.in_waiting)) 
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()
            values = line.split(',')
            command_values = [int(value) for value in values[:6]]
            lift_feedback = [int(value) for value in values[6:8]]
            # tilt_feedback = int(values[9])
            # motors_feedback = [value for value in values[10:]]  
            ##in case of second tilt untoggle the following two lines and delete the two lines above
            tilt_feedback = [int(value) for value in values[8:10]]
            motors_feedback = [value for value in values[10:]]
            command = ArduinoCommandsFeedback()
            command.commands = command_values
            lift = LinActPos()
            lift.rl_lin_act = lift_feedback
            tilt = LinActPos()
            #tilt.rl_lin_act = [tilt_feedback,tilt_feedback]
            tilt.rl_lin_act = tilt_feedback
            #motor = MotorsVel()
            #motor.rl_motor = motors_feedback
            self.publisher_command_fb.publish(command)
            self.publisher_lift_fb.publish(lift)
            self.publisher_tilt_fb.publish(tilt)
            #self.publisher_motor_fb.publish(motor)
        
        packed_data = struct.pack('6B',*self.motor)
        self.ser.write(packed_data)
         
    def phase_now(self,msg):
        phase = msg.data
        if phase=='scoop':
            self.vibr = self.ON_scoop
        if phase == 'discharge':
            self.vibr = self.ON_dump

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    rclpy.spin(node)
    rclpy.shutdown()
    node.ser.close()


if __name__== "__main__":
    main()

