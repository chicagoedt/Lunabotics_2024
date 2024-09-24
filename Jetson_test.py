#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import Jetson.GPIO as GPIO

class JetsonPWMNode(Node):
    def __init__(self):
        super().__init__("jetson_test")
        self.get_logger().info("Jetson test node activated")
        # Set the GPIO pin number you want to use
        self.PWM_PIN = 33
        # Set the PWM frequency (in Hz) and duty cycle (0.0 to 100.0)
        self.declare_parameter('frequency','250')
        self.PWM_FREQUENCY = int(self.get_parameter('frequency').get_parameter_value().string_value)
        self.DUTY_CYCLE = 20.0  
        # Initialize the GPIO settings
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PWM_PIN, GPIO.OUT)
        # Create PWM object
        self.pwm = GPIO.PWM(self.PWM_PIN, self.PWM_FREQUENCY)
        #start PWM
        self.timer_ = self.create_timer(2,self.vibrator_on)
        
    
    def vibrator_on(self):
       
        self.pwm.stop()
        self.pwm.start(self.DUTY_CYCLE)
        self.get_logger().info('given command: %.f' %self.DUTY_CYCLE)
        self.DUTY_CYCLE += 2.0
        if self.DUTY_CYCLE > 50.0:
            self.DUTY_CYCLE = 20.0
        time.sleep(1)
        self.pwm.stop()
        


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











