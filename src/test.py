import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from builtin_interfaces.msg import Time

class PointCloudFixer(Node):
    def __init__(self):
        super().__init__('pointcloud_timestamp_fixer')
        self.sub = self.create_subscription(PointCloud2, '/scan_3d/points', self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/scan_3d/points_fixed', 10)

    def callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id= "velodyne"
        self.pub.publish(msg)

def main():
    rclpy.init()
    while (True):
        rclpy.spin(PointCloudFixer())
        rclpy.spin()
