import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data # <--- CRITICAL IMPORT
from sensor_msgs.msg import LaserScan
import time
import sys

from config import *
from imu_reader import IMUReader
from servo_controller import FastServoController
from lidar_driver import LidarDriver
from point_builder import PointBuilder
from cloud_publisher import CloudPublisher
from data_logger import DataLogger

class Lidar3DNode(Node):
    def __init__(self):
        super().__init__("lidar_3d_node")
        
        print("Initializing Hardware...")
        self.lidar = LidarDriver(LIDAR_LAUNCH)
        self.imu = IMUReader(IMU_PORT, BAUD)
        self.servo = FastServoController(SERVO_PORT, BAUD)
        self.builder = PointBuilder()
        self.publisher = CloudPublisher(self)
        self.logger = DataLogger(OUTPUT_FILENAME)

        # State flags
        self.sweep_started = False
        self.first_scan_received = False

        # 1. Calibration Sequence
        print("Moving to calibration position (90 deg)...")
        self.servo.move_to(90.0) 
        time.sleep(3.0) 
        
        print("Sampling IMU for offset...")
        self.imu.calibrate() 
        print("Calibration complete.")
        
        # 2. Subscribe using SENSOR DATA QoS
        # This fixes the issue if the LiDAR is publishing "Best Effort"
        self.create_subscription(
            LaserScan, 
            "/scan", 
            self.scan_cb, 
            qos_profile_sensor_data
        )
        
        self.create_timer(1.0/PUBLISH_RATE_HZ, self.pub_cb)
        self.create_timer(1.0, self.check_status)

        print(">>> WAITING FOR LIDAR DATA... (The sweep will start automatically)")

    def scan_cb(self, msg):
        # 1. Auto-Start Logic: Only start sweeping once we confirm we have data
        if not self.first_scan_received:
            self.first_scan_received = True
            print("!!! LIDAR DATA RECEIVED !!! Starting Sweep...")
            self.servo.start_sweep()
            self.sweep_started = True

        # 2. Get Tilt
        tilt = self.imu.get_tilt_rad()
        
        # 3. Save Data (Only if we are actually sweeping)
        if self.sweep_started:
            self.logger.record(msg, tilt)
            
        # 4. Live Process
        self.builder.process_scan(msg, tilt)

    def pub_cb(self):
        self.publisher.publish(self.builder.get_cloud())

    def check_status(self):
        # Only check for completion if we have actually started
        if self.sweep_started and self.servo.sweep_count >= SWEEPS_TO_RECORD:
            print(f"\n--- REACHED {SWEEPS_TO_RECORD} SWEEPS ---")
            print(f"Captured {len(self.logger.data)} frames.")
            print("Stopping and saving data...")
            self.logger.save()
            raise SystemExit 

def main():
    rclpy.init()
    node = Lidar3DNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        print("Shutting down...")
        node.servo.move_to(TILT_CENTER)
        node.lidar.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()