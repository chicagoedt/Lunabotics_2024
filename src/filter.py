#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import LinActPos

class FilterNode(Node):

    def __init__(self):
        
        super().__init__("filter_lift")

        self.declare_param()

        self.get_param()

        self.subscriber_feedback = self.create_subscription(LinActPos,self.sub_topic,self.get_feedback,10) 
        self.publisher_feedback = self.create_publisher(LinActPos,self.pub_topic,10)

        self.feedback = [0.0,0.0]
        self.prev = [0.0, 0.0]
        self.count = 0

        self.get_logger().info("filter node activated")

    def declare_param(self):

        self.declare_parameter('subscriber_topic','lift_feedback')
        self.declare_parameter('publisher_topic','lift_feedback_filt')
        self.declare_parameter('percent_taken',0.9)

    def get_param(self):

        self.sub_topic = self.get_parameter('subscriber_topic').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('publisher_topic').get_parameter_value().string_value
        self.perc = self.get_parameter('percent_taken').get_parameter_value().double_value

    def get_feedback(self,msg):

        self.feedback = msg.rl_lin_act
        if self.count == 0:
            self.prev = self.feedback
            self.count = 1
        else:
            for i in range(2):
                self.feedback[i] = int(self.perc*self.prev[i]+(1-self.perc)*self.feedback[i])
                if self.feedback[i]>1024:
                    self.feedback[i] = 1024
                    
            if self.sub_topic == 'tilt_feedback':
                #self.feedback[0] = self.feedback[1]
                #self.feedback[0] = int(-self.feedback[0]+866)
                self.feedback[1] = self.feedback[0]
                
            feedback = LinActPos()
            feedback.rl_lin_act = self.feedback
            self.publisher_feedback.publish(feedback)
            self.prev = self.feedback


def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__== "__main__":
    main()
