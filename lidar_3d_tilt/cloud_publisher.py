# cloud_publisher.py (Modified)
from sensor_msgs.msg import PointCloud2, PointField
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

        # Define fields for XYZ + Intensity (using Z as intensity)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = point_cloud2.create_cloud(header, fields, points)
        self.pub.publish(cloud)