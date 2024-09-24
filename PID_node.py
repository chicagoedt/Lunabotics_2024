#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from robot_interfaces.msg import SensorsFeedback

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.subscriber_ = self.create_subscription(Joy,'joy',self.motor_controller,10) 
        self.publisher_motor1 = self.create_publisher(Float32,"motor_1",10)
        self.publisher_motor2 = self.create_publisher(Float32,"motor_2",10)
        self.publisher_lift1 = self.create_publisher(Float32,"lift_1",10)
        self.publisher_lift2 = self.create_publisher(Float32,"lift_2",10)
        self.publisher_tilt = self.create_publisher(Float32,"tilt",10)
        self.publisher_vibr = self.create_publisher(Float32,"vibr",10)
        self.get_logger().info("Motor controller node activated")

    def motor_controller(self,msg):
        analog = msg.axes
        digital = msg.buttons
        motor_1 = Float32()
        motor_2 = Float32()
        lift_1 = Float32()
        lift_2 = Float32()
        tilt = Float32()
        vibr = Float32()
        motor_1.data = (-analog[3]-analog[2])/1.2
        motor_2.data = (analog[3]-analog[2])/1.2
        lift_1.data = analog[1]
        lift_2.data = analog[1]
        tilt.data = analog[0]
        vibr.data = analog[4]
        self.publisher_motor1.publish(motor_1)
        self.publisher_motor2.publish(motor_2)
        self.publisher_lift1.publish(lift_1)
        self.publisher_lift2.publish(lift_2)
        self.publisher_tilt.publish(tilt)
        self.publisher_vibr.publish(vibr)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()