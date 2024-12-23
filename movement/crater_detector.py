#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CraterDetector(Node):
    def __init__(self):
        super().__init__('crater_detector')
        self.bridge = CvBridge()

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # tbd publish a new topic indicating crater locations
        # or an occupancy grid update, or a marker array for visualization, etc.

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # convert depth to meters (depends on your camera’s scaling)
        #raw depth is often in millimeters if type=16UC1.
        # adjust as needed
        depth_in_meters = depth_image.astype(np.float32) * 0.001

        # "very close" surfaces vs. "far" holes
        _, close_mask = cv2.threshold(depth_in_meters, 0.1, 255, cv2.THRESH_BINARY_INV)
        _, far_mask = cv2.threshold(depth_in_meters, 2.0, 255, cv2.THRESH_BINARY)

        # "unusual" areas - naive approach
        combined_mask = cv2.bitwise_and(close_mask, far_mask)

        combined_mask = combined_mask.astype(np.uint8)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            self.get_logger().info(f"Detected {len(contours)} potential crater/hill regions")

        # change detections into obstacles
        # in a costmap, or do further analysis.

def main(args=None):
    rclpy.init(args=args)
    node = CraterDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 run lunabot crater_detector

