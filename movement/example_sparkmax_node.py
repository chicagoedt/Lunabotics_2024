#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# this is where we load cod's libarry

class SparkMaxController(Node):
    def __init__(self):
        super().__init__('sparkmax_controller')

        # self.left_motor = SparkMax(device_id=1, bus="can0")
        # self.right_motor = SparkMax(device_id=2, bus="can0")

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Robot-specific parameters
        self.wheel_radius = 0.05    # 5 cm radius
        self.wheel_separation = 0.30  # 30 cm separation

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z


        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)

        # convert to m/s moter speed
        # assume direct pass:
        # self.left_motor.set_speed(v_left)
        # self.right_motor.set_speed(v_right)

        self.get_logger().info(f"Setting motor speeds -> Left: {v_left:.2f}, Right: {v_right:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SparkMaxController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
