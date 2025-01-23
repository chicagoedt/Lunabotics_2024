#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import DutyCycleOutput

class TestControlNode(Node):
	def __init__(self):
        	super().__init__('test_control')
        
        	self.duty_publisher = self.create_publisher(DutyCycleOutput, 'control/duty_cycle', 10)
        	self.timer = self.create_timer(0.5, self.send_command_manual)
        	
        	self.left_drive_output, self.right_drive_output, self.left_lift_output, self.right_lift_output, self.tilt_output, self.vibrator_output = (0.0,0.0,0.0,0.0,0.0,0.0)
        
	def send_command_manual(self):
		outputPercentage = 0.2
		
		userInput = input("Press 1 for left motor, 2 for right motor, 3 for left lift, 4 for right lift, 5 for tilt, 6 for jiggle")
		
		if userInput == '1':
			self.left_drive_output = outputPercentage
			print("1 recieved")
		if userInput == '2':
			self.right_drive_output = outputPercentage
			print("2 recieved")
		if userInput == '3':
			self.left_lift_output = outputPercentage
			print("3 recieved")
		if userInput == '4':
			self.right_lift_output = outputPercentage
			print("4 recieved")
		if userInput == '5':
			self.tilt_output = outputPercentage
			print("5 recieved")
		if userInput == '6':
			self.vibrator_output = outputPercentage
			print("6 recieved")
		#Takes in user input to find out which motors recieve power
		
		msg = DutyCycleOutput()
		
		msg.left_drive_output = self.left_drive_output
		msg.right_drive_output = self.right_drive_output
		msg.left_lift_output = self.left_lift_output
		msg.right_lift_output = self.right_lift_output
		msg.tilt_output = self.tilt_output
		msg.vibrator_output = self.vibrator_output
		
		self.duty_publisher.publish(msg)
		
def main(args=None):
    rclpy.init(args=args)
    node = TestControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()

