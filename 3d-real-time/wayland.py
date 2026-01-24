#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import serial
import threading
import subprocess
import signal
import os
import time
import math
import datetime

# =========================
# CONFIGURATION
# =========================
ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

LIDAR_LAUNCH = ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"]

MIN_RANGE = 0.10
MAX_RANGE = 12.0

ANGLE_RESOLUTION_DEG = 0.5
PUBLISH_RATE_HZ = 5
MAX_BUFFER_POINTS = 120_000

# 🔧 MEASURE THESE (meters)
LIDAR_OFFSET_X = 0.01   # forward from tilt axis
LIDAR_OFFSET_Z = 0.045   # above tilt axis

# =========================
class Lidar3DTilt(Node):
    def __init__(self):
        super().__init__("lidar_3d_tilt")

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"scan_{ts}.xyz"
        with open(self.filename, "w") as f:
            f.write("# X Y Z\n")

        self.get_logger().info(f"Saving to {self.filename}")

        self.lidar_process = subprocess.Popen(
            LIDAR_LAUNCH,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        time.sleep(3)

        self.ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.01)
        self.current_tilt_rad = 0.0
        self.running = True

        self.sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.pc_pub = self.create_publisher(
            PointCloud2, "/lidar_3d/points", 10
        )

        self.angle_bins = {}
        self.updated_bins = set()

        threading.Thread(target=self.read_servo_angle, daemon=True).start()

        # Publish timer
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_cloud)

    # =========================
    def read_servo_angle(self):
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    self.current_tilt_rad = math.radians(float(line))
            except:
                pass

    # =========================
    def scan_callback(self, msg):
        tilt_deg = math.degrees(self.current_tilt_rad)
        tilt_bin = round(tilt_deg / ANGLE_RESOLUTION_DEG) * ANGLE_RESOLUTION_DEG
        tilt = math.radians(tilt_bin)

        angle = msg.angle_min
        pts = []

        cos_t = math.cos(tilt)
        sin_t = math.sin(tilt)

        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE:
                # LiDAR frame
                xl = r * math.cos(angle)
                yl = r * math.sin(angle)
                zl = 0.0

                # Tilt rotation + lever-arm correction
                x = cos_t * xl + sin_t * zl + LIDAR_OFFSET_X * cos_t
                y = yl
                z = -sin_t * xl + cos_t * zl + LIDAR_OFFSET_Z

                pts.append((x, y, z))
            angle += msg.angle_increment

        if pts:
            self.angle_bins[tilt_bin] = pts
            self.updated_bins.add(tilt_bin)

    # =========================
    def publish_cloud(self):
        if not self.angle_bins:
            return

        all_pts = []
        for pts in self.angle_bins.values():
            all_pts.extend(pts)

        all_pts = all_pts[-MAX_BUFFER_POINTS:]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar"

        cloud = point_cloud2.create_cloud_xyz32(header, all_pts)
        self.pc_pub.publish(cloud)

        with open(self.filename, "a") as f:
            for p in all_pts:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    # =========================
    def shutdown(self):
        self.running = False
        try:
            self.ser.close()
        except:
            pass
        try:
            os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGINT)
        except:
            pass
        self.destroy_node()

# =========================
def main():
    rclpy.init()
    node = Lidar3DTilt()
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
