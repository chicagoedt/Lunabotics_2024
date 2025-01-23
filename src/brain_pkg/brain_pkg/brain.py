#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces_pkg.msg import DutyCycleOutput
import curses

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain')

        self.duty_publisher = self.create_publisher(DutyCycleOutput, 'control/duty_cycle', 10)
        self.timer = self.create_timer(0.5, self.send_command_manual)

        self.left_drive_output, self.right_drive_output, self.left_lift_output, self.right_lift_output, self.tilt_output, self.vibrator_output = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def send_command_manual(self, stdscr):
        outputPercentage = 0.2

        # Clear the screen
        stdscr.clear()
        stdscr.refresh()

        # Print instructions
        stdscr.addstr(0, 0, "Press 1 for left motor, 2 for right motor, 3 for left lift, 4 for right lift, 5 for tilt, 6 for jiggle.")
        stdscr.addstr(1, 0, "Press 'q' to quit.")
        stdscr.refresh()

        while True:
            userInput = stdscr.getch()  # Get the key press

            if userInput == ord('1'):
                self.left_drive_output = outputPercentage
                stdscr.addstr(2, 0, "1 received: Left motor activated.")
            elif userInput == ord('2'):
                self.right_drive_output = outputPercentage
                stdscr.addstr(2, 0, "2 received: Right motor activated.")
            elif userInput == ord('3'):
                self.left_lift_output = outputPercentage
                stdscr.addstr(2, 0, "3 received: Left lift activated.")
            elif userInput == ord('4'):
                self.right_lift_output = outputPercentage
                stdscr.addstr(2, 0, "4 received: Right lift activated.")
            elif userInput == ord('5'):
                self.tilt_output = outputPercentage
                stdscr.addstr(2, 0, "5 received: Tilt activated.")
            elif userInput == ord('6'):
                self.vibrator_output = outputPercentage
                stdscr.addstr(2, 0, "6 received: Vibrator activated.")
            elif userInput == ord('q'):  # Exit on 'q'
                break

            # Prepare the message to publish
            msg = DutyCycleOutput()
            msg.left_drive_output = self.left_drive_output
            msg.right_drive_output = self.right_drive_output
            msg.left_lift_output = self.left_lift_output
            msg.right_lift_output = self.right_lift_output
            msg.tilt_output = self.tilt_output
            msg.vibrator_output = self.vibrator_output

            self.duty_publisher.publish(msg)

            stdscr.refresh()

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()

    # Use curses.wrapper to handle initialization and cleanup of curses
    curses.wrapper(node.send_command_manual)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

