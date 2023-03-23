#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

class KittiPublisher(Node):

    def __init__(self):
        super().__init__('kitti_publisher')
        self.publisher_ = self.create_publisher(Image, 'kitti_image', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        kitti_dir = os.path.join(os.getcwd(),'src', 'dee_image_publisher', 'dataset', 'kitti', '2011_09_26', '2011_09_26_drive_0001_sync')
        image_files = sorted(os.listdir(os.path.join(kitti_dir, 'image_02', 'data')))
        for image_file in image_files:
            image_path = os.path.join(kitti_dir, 'image_02', 'data', image_file)
            img = cv2.imread(image_path)
            self.get_logger().info('Publishing: %s, path %s' % (image_file, image_path))
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    kitti_publisher = KittiPublisher()

    rclpy.spin(kitti_publisher)

    kitti_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()