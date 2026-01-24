# cloud_publisher.py
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class CloudPublisher:
    def __init__(self, node):
        self.pub = node.create_publisher(
            PointCloud2, "/lidar_3d/points", 10
        )
        self.node = node

    def publish(self, points):
        if not points:
            return

        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = "lidar"

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pub.publish(cloud)
