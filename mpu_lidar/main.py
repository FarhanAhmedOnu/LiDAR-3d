import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

from config import *
from imu_reader import IMUReader
from servo_controller import FastServoController
from lidar_driver import LidarDriver
from point_builder import PointBuilder
from cloud_publisher import CloudPublisher

class Lidar3DNode(Node):
    def __init__(self):
        super().__init__("lidar_3d_node")
        
        self.lidar = LidarDriver(LIDAR_LAUNCH)
        self.imu = IMUReader(IMU_PORT, BAUD)
        self.servo = FastServoController(SERVO_PORT, BAUD)
        self.builder = PointBuilder()
        self.publisher = CloudPublisher(self)

        # 1. Calibration Sequence
        print("Moving to calibration position (90 deg)...")
        self.servo.move_to(90.0) 
        
        # IMPORTANT: Give the servo physical time to reach 90 before sampling IMU
        time.sleep(3.0) 
        
        print("Sampling IMU for offset...")
        self.imu.calibrate() 
        
        print("Calibration complete. Starting sweep.")
        self.servo.start_sweep()
        
        # 2. Start Operations
        self.servo.start_sweep()
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_timer(1.0/PUBLISH_RATE_HZ, self.pub_cb)

    def scan_cb(self, msg):
        self.builder.process_scan(msg, self.imu.get_tilt_rad())

    def pub_cb(self):
        self.publisher.publish(self.builder.get_cloud())

def main():
    rclpy.init()
    node = Lidar3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.servo.move_to(TILT_CENTER)
        node.lidar.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()