#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robot_interfaces.msg import ArduinoCommands, MotorsVel

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("Motor_Controller")
        self.motor_vel = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] #right motor,left motor, right linear actuator, left linear actuator, tilt actuator, vibrator
        self.max_forward_vel = [1.0,1.0,0.4,0.4,2.0,2.0,0.5]
        self.max_backward_vel = [1.0,1.0,0.4,0.4,2.0,2.0,0.0]
        self.subscriber_motors = self.create_subscription(MotorsVel,"motor_vel",lambda msg: self.cmdvelcallback(msg,motor=0),10)
        self.subscriber_lift = self.create_subscription(MotorsVel,"lift_vel",lambda msg: self.cmdvelcallback(msg,motor=2),10)
        self.subscriber_tilt = self.create_subscription(MotorsVel,"tilt_vel",lambda msg: self.cmdvelcallback(msg,motor=4),10)
        self.subscriber_vibr = self.create_subscription(Bool,"vibr",lambda msg: self.cmdvelcallback(msg,motor=6),10)
        self.publisher_command = self.create_publisher(ArduinoCommands,"arduino_commands",10)
        self.timer_ = self.create_timer(0.1,self.Arduino_callback)
        self.get_logger().info("Motor controller node activated")

    def cmdvelcallback(self,msg,motor):
        if motor != 6:
            desw = msg.rl_motor
            desw = [val*((val>0)/self.max_forward_vel[motor]+(val<0)/self.max_backward_vel[motor]) for val in desw] #take the percentage of the maximum velocity
            self.motor_vel[motor:motor+2] = desw
        else:
            desw = msg.data
            desw = desw*self.max_forward_vel[motor] 
            self.motor_vel[motor] = desw

    def Arduino_callback(self):
        msg = ArduinoCommands()
        commands = self.motor_vel#[:4]
        # commands.append(self.motor_vel[5])
        # commands.append(self.motor_vel[6])
        msg.motor_command = commands
        self.publisher_command.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
