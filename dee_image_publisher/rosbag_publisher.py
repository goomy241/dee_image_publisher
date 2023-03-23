#!/usr/bin/env python3
# ref: https://github.com/ros2/rosbag2/blob/c7c7954d4d9944c160d7b3d716d1cb95d34e37e4/rosbag2_py/test/test_sequential_reader.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from std_msgs.msg import Header
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

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
        topic_metadata = self.reader.get_all_topics_and_types()
        self.type_map = {topic_metadata[i].name: topic_metadata[i].type for i in range(len(topic_metadata))}

    def timer_callback(self):
        while self.reader.has_next():
           (msg_topic, msg, t) = self.reader.read_next()
           print(msg_topic + "\t")
           if msg_topic == '/webcam':
                msg_type = get_message(self.type_map[msg_topic])
                msg = deserialize_message(msg, msg_type)
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                ros_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
                ros_msg.header = Header()
                ros_msg.header.stamp = self.get_clock().now().to_msg()
                self.get_logger().info('Publishing image from bag file at time: {}'.format(t))
                self.publisher_.publish(ros_msg)
        self.get_logger().info('End of bag file')

def main(args=None):
    rclpy.init(args=args)

    rosbag_publisher = RosbagPublisher()

    rclpy.spin(rosbag_publisher)

    rosbag_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()