#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import numpy as np

class CommandNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.publisher_ = self.create_publisher(Joy,'joy',10) 
        self.timer_ = self.create_timer(0.1,self.get_command)
        self.get_logger().info("Command node activated, press u-i-o-j-k-l-m-,-. for moving the robot,"+
                               "w-s-x-d-a for the linear actuators, g for the vibrator, "+
                               "8-2-6-4 for changing lin act velocity, t-b for vibrator, y-h for linear. r-f for angular")
        self.analog = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.vel=1.0
        self.ang=1.0
        self.linvel = 0.9 
        self.tiltvel= 0.9
        self.digital=[int(0),int(0)]
        self.vibr = 0.50

    def get_command(self):
        msg = Joy()
        input_value = input()
        if input_value == 'o':
            self.analog[2]=self.ang*0.2
            self.analog[3]=-self.vel * 0.7
        if input_value == 'i':
            self.analog[2]=0.0
            self.analog[3]=-self.vel
        if input_value == 'u':
            self.analog[2]=-self.ang*0.2
            self.analog[3]=-self.vel * 0.7
        if input_value == 'j':
            self.analog[2]=self.ang*0.7
            self.analog[3]=0.0
        if input_value == 'k':
            self.analog[2]=0.0
            self.analog[3]=0.0
        if input_value == 'l':
            self.analog[2]=-self.ang*0.7
            self.analog[3]=0.0
        if input_value == 'm':
            self.analog[2]=self.ang*0.2
            self.analog[3]=self.vel * 0.7
        if input_value == ',':
            self.analog[2]=0.0
            self.analog[3]=self.vel
        if input_value == '.':
            self.analog[2]=-self.ang*0.2
            self.analog[3]=self.vel * 0.7

        if input_value == 'w':
            self.analog[1]=self.linvel
        if input_value == 's':
            self.analog[1]=0.0
            self.analog[0]=0.0
        if input_value == 'x':
            self.analog[1]=-self.linvel
        if input_value == 'a':
            self.analog[0]=-self.tiltvel
        if input_value == 'd':
            self.analog[0]=self.tiltvel

        if input_value == 'g':
            self.analog[4]=self.vibr*(self.analog[4]==0)

        if input_value == '5':
            self.analog[1] = 1.0
            self.analog[0] = 0.5
            self.analog[3] = 0.5*self.vel
            self.analog[2] = 0.0

        if input_value == '7':
            self.analog[1] = 1.0
            self.analog[0] = -0.5
            self.analog[3] = 0.5*self.vel
            self.analog[2] = 0.0

        if input_value == 'y':
            self.vel = self.vel*(1+5/58)
            if self.vel > 1.0:
                self.vel = 1.0
                self.get_logger().info("Max speed")
            self.get_logger().info("Linear velocity incremented")
        
        if input_value == 'h':
            self.vel = self.vel*(1-5/58)
            self.get_logger().info("Linear velocity decreased")

        if input_value == 'r':
            self.ang = self.ang*1.1
            self.ang = np.tanh(self.ang)
            self.get_logger().info("Angular velocity incremented")
        
        if input_value == 'f':
            self.ang = self.ang*0.9
            self.get_logger().info("Angular velocity decreased")
        
        if input_value == '8':
            self.linvel = self.linvel*(1+5/58)
            if self.linvel > 1.0:
                self.linvel = 1.0
                self.get_logger().info("Max speed")
            self.get_logger().info("Linear actuator velocity incremented")
        
        if input_value == '2':
            self.linvel = self.linvel*(1-5/58)
            self.get_logger().info("Linear actuator velocity decreased")
        
        if input_value == '6':
            self.tiltvel = self.tiltvel*(1+5/58)
            if self.tiltvel > 1.0:
                self.tiltvel = 1.0
                self.get_logger().info("Max speed")
            self.get_logger().info("Linear tilt velocity incremented")
        
        if input_value == '4':
            self.tiltvel = self.tiltvel*(1-5/58)
            self.get_logger().info("Linear tilt velocity decreased")
        
        if input_value == 't':
            self.vibr = self.vibr*(1+5/58)
            if self.vibr > 1.0:
                self.vibr = 1.0
                self.get_logger().info("Max speed")
            self.get_logger().info("Vibrator velocity incremented")
        
        if input_value == 'b':
            self.vibr = self.vibr*(1-5/58)
            self.vibr = np.tanh(self.vibr)
            self.get_logger().info("Vibrator velocity decreased")

        msg.axes = self.analog
        msg.buttons = self.digital

        self.publisher_.publish(msg)
        



def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
