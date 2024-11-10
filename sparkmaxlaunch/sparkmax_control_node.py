import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sparkmax  # i need to import cod's library here, i havent got around to it yet

class SparkMaxControlNode(Node):
    def __init__(self):
        super().__init__('sparkmax_control_node')
        
        
        self.left_motor = sparkmax.MotorController(id=1)
        self.right_motor = sparkmax.MotorController(id=2)
        
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info("SparkMAX Control Node started, waiting for /cmd_vel commands")

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        left_speed = linear_speed - angular_speed
        right_speed = linear_speed + angular_speed

        self.left_motor.set_speed(left_speed) # to be changed 
        self.right_motor.set_speed(right_speed)

        self.get_logger().info(f"Set left motor to {left_speed} and right motor to {right_speed}")

def main(args=None):
    rclpy.init(args=args)
    node = SparkMaxControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
