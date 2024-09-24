#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import Jetson.GPIO as GPIO

class JetsonPWMNode(Node):
    def __init__(self):
        super().__init__("jetson")
        self.subscriber_ = self.create_subscription(Bool,'vibr',self.command,10) 
        self.command_subscriber_ = self.create_subscription(String,'aut_command',self.phase_now,10)
        self.get_logger().info("Jetson node activated")
        self.min_max = [35.0,50.0]
        # Set the GPIO pin number you want to use
        self.PWM_PIN = 33
        # Set the PWM frequency (in Hz) and duty cycle (0.0 to 100.0)
        self.PWM_FREQUENCY = 245 
        self.DUTY_CYCLE = 42.0  
        # Initialize the GPIO settings
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PWM_PIN, GPIO.OUT)
        # Create PWM object
        self.pwm = GPIO.PWM(self.PWM_PIN, self.PWM_FREQUENCY)
        self.prev_cmd = False
    
    def command(self,msg):
        
        command = msg.data
        if command:
            
            if not self.prev_cmd:
                self.pwm.start(self.DUTY_CYCLE)
                self.get_logger().info('given command: %.f' %self.DUTY_CYCLE)
                #give the command to the Jetson
            else:
                return
        
        else:
            self.pwm.stop()

    def phase_now(self,msg):
        if msg.data == 'scoop':
            self.DUTY_CYCLE = 26.0
        if msg.data == 'discharge':
            self.DUTY_CYCLE = 42.0
        


def main(args=None):
    rclpy.init(args=args)
    node = JetsonPWMNode()
    rclpy.spin(node)
    rclpy.shutdown()
    # Cleanup GPIO
    node.pwm.stop()
    GPIO.cleanup()


if __name__== "__main__":
    main()











