#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Header

class CameraPublisher(Node):
    
        def __init__(self):
            super().__init__('camera_publisher')
            self.publisher_ = self.create_publisher(Image, 'webcam', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.bridge = CvBridge()
    
        def timer_callback(self):
            cap = cv2.VideoCapture(0)
            self.get_logger().info('camera opened: %s' % (cap.isOpened()))
            if not cap.isOpened():
                self.get_logger().warn('Error opening camera')
                exit()
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn("Can't receive frame (stream end?). Exiting ...")
                exit()
            self.get_logger().info('Publishing: %s' % (frame))
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()