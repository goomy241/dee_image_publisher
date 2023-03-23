import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from std_msgs.msg import Header

class RosbagPlayTest(Node):
    def __init__(self):
        super().__init__('rosbag_play_test')
        self.publisher_ = self.create_publisher(String, 'chatter_test', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # open rosbag file
        bag_dir = os.path.join(os.getcwd(), 'src', 'dee_image_publisher', 'dataset', 'bag', 'chatter')
        self.reader = SequentialReader()
        self.storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
        self.converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        self.reader.open(self.storage_options, self.converter_options)

    def timer_callback(self):
        while self.reader.has_next():
           (msg_topic, msg, t) = self.reader.read_next()
        #    msg = msg.decode('utf-8')
           print(type(msg))
        #    print("-------"+msg_topic)
           if '/chatter' == msg_topic:
                msg_ros = String()
                msg_ros.data = msg.decode('utf-8')
                self.publisher_.publish(msg_ros)
                self.get_logger().info('Publishing: %s, topic %s' % (msg_ros, msg_topic))
        # self.reader.reset()
def main(args=None):
    rclpy.init(args=args)
    rosbag_play_test = RosbagPlayTest()
    rclpy.spin(rosbag_play_test)
    # rosbag_play_test.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()