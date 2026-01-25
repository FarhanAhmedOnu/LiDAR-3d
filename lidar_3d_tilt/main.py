# main.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import traceback
import sys
import time

from config import *
from servo_controller import ServoController
from lidar_driver import LidarDriver
from point_builder import PointBuilder
from cloud_publisher import CloudPublisher

class Lidar3DTiltNode(Node):
    def __init__(self):
        super().__init__("lidar_3d_tilt")
        
        print("Initializing 3D LiDAR Tilt Node...")
        
        try:
            # Initialize LiDAR first
            print("Starting LiDAR...")
            self.lidar = LidarDriver(LIDAR_LAUNCH)
            time.sleep(2)
            
            # Initialize servo
            print("Starting servo controller...")
            self.servo = ServoController(ARDUINO_PORT, ARDUINO_BAUD)
            time.sleep(1)  # Let servo start sweeping
            
            # Initialize other components
            self.builder = PointBuilder()
            self.publisher = CloudPublisher(self)
            
            # Setup laser scan subscription
            self.create_subscription(
                LaserScan, "/scan", self.scan_callback, 10
            )
            
            # Setup cloud publishing timer
            self.create_timer(
                1.0 / PUBLISH_RATE_HZ,
                self.publish_cloud
            )
            
            print("3D LiDAR Tilt Node initialized successfully")
            
        except Exception as e:
            print(f"ERROR during initialization: {e}")
            traceback.print_exc()
            self.shutdown()
            raise
    
    def scan_callback(self, msg):
        try:
            tilt_rad = self.servo.get_tilt_rad()
            self.builder.process_scan(msg, tilt_rad)
        except Exception as e:
            self.get_logger().error(f"Scan callback error: {e}")
    
    def publish_cloud(self):
        try:
            pts = self.builder.get_cloud()
            self.publisher.publish(pts)
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")
    
    def shutdown(self):
        print("Shutting down node...")
        if hasattr(self, 'servo'):
            self.servo.close()
        if hasattr(self, 'lidar'):
            self.lidar.stop()
        
        try:
            self.destroy_node()
        except:
            pass

def main():
    rclpy.init()
    node = None
    
    try:
        node = Lidar3DTiltNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCtrl+C pressed - shutting down")
    except Exception as e:
        print(f"Fatal error: {e}")
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()
        rclpy.shutdown()
        print("Program terminated")

if __name__ == "__main__":
    main()