#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from std_msgs.msg import Header
from rosbags.image import message_to_cvimage

class RosbagPublisher(Node):
    def __init__(self):
        super().__init__('rosbag_publisher')
        self.publisher_ = self.create_publisher(Image, 'rosbag_image', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

        # open rosbag file
        bag_dir = os.path.join(os.getcwd(), 'src', 'dee_image_publisher', 'dataset', 'bag', 'dee_webcam_bag1')
        self.reader = SequentialReader()
        self.storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
        self.converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.reader.open(self.storage_options, self.converter_options)

        # get all topics and types
        self.topic_metadata = self.reader.get_all_topics_and_types()
        self.topic_names = []
        for topic in self.topic_metadata:
            self.topic_names.append(topic.name)
        self.topic_names.sort()

    def timer_callback(self):
        for topic in self.topic_names:
           (msg_topic, msg, t) = self.reader.read_next()
           if topic == msg_topic:
                # img_array = np.frombuffer(msg, dtype=np.uint8)
                img_ros = message_to_cvimage(msg)
                msg = self.bridge.cv2_to_imgmsg(img_ros, encoding='passthrough')
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                self.get_logger().info('Publishing image from bag file at time: {}'.format(t))
                self.publisher_.publish(msg)
        self.reader.close()

def main(args=None):
    rclpy.init(args=args)

    rosbag_publisher = RosbagPublisher()

    rclpy.spin(rosbag_publisher)

    rosbag_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()