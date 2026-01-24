# main.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from config import *
from servo_reader import ServoReader
from lidar_driver import LidarDriver
from point_builder import PointBuilder
from cloud_publisher import CloudPublisher

class Lidar3DTiltNode(Node):
    def __init__(self):
        super().__init__("lidar_3d_tilt")

        self.lidar = LidarDriver(LIDAR_LAUNCH)
        self.servo = ServoReader(ARDUINO_PORT, ARDUINO_BAUD)
        self.builder = PointBuilder()
        self.publisher = CloudPublisher(self)

        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.publish_cloud
        )

    def scan_callback(self, msg):
        self.builder.process_scan(
            msg,
            self.servo.get_tilt_rad()
        )

    def publish_cloud(self):
        pts = self.builder.get_cloud()
        self.publisher.publish(pts)

    def shutdown(self):
        self.servo.close()
        self.lidar.stop()
        self.destroy_node()

def main():
    rclpy.init()
    node = Lidar3DTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
