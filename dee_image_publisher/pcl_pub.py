import pcl
import os
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

class PclNode(Node):
    def __init__(self):
        super().__init__('pcl_node')
        self.publisher_ = self.create_publisher(PointCloud2, 'pcl_pointcloud', 10)

        self.kitti_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'dataset', 'kitti', '2011_09_26', '2011_09_26_drive_0001_sync')
        self.image_files = os.path.join(self.kitti_dir, 'image_02', 'data')
        self.velodyne_files = os.path.join(self.kitti_dir, 'velodyne_points', 'data')

        self.seq = 0
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        cloud = pcl.PointCloud()
        file_name = os.path.join(self.velodyne_files, '{:010d}.bin'.format(self.seq))
        points = np.fromfile(file_name, dtype=np.float32).reshape(-1, 4)
        cloud.from_array(points[:, :3])

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField( name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    ]
        
        pcl_msg = PointCloud2()
        pcl_msg.header = header
        pcl_msg.fields = fields
        pcl_msg.width = cloud.size
        pcl_msg.height = 1
        pcl_msg.point_step = 12
        pcl_msg.is_dense = False
        pcl_msg.data = np.asarray(cloud, np.float32).tostring()

        self.publisher_.publish(pcl_msg)
        self.seq += 1

def main(args=None):
    rclpy.init(args=args)
    node = PclNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    