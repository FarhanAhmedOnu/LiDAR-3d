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

import numpy as np
import open3d as o3d

# =========================
# CONFIGURATION
# =========================
ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

LIDAR_LAUNCH = ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"]

MIN_RANGE = 0.05
MAX_RANGE = 12.0

ANGLE_RESOLUTION_DEG = 0.5     # tilt bin size
MAX_BUFFER_POINTS = 150_000

ENABLE_OPEN3D = True

# =========================
# NODE
# =========================
class Lidar3DTilt(Node):
    def __init__(self):
        super().__init__("lidar_3d_tilt")

        # ---------- File ----------
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"scan_{ts}.xyz"
        with open(self.filename, "w") as f:
            f.write("# X Y Z\n")

        self.get_logger().info(f"Saving to {self.filename}")

        # ---------- Launch LiDAR ----------
        self.lidar_process = subprocess.Popen(
            LIDAR_LAUNCH,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        time.sleep(3)

        # ---------- Serial ----------
        self.ser = serial.Serial(
            ARDUINO_PORT,
            ARDUINO_BAUD,
            timeout=0.01
        )

        self.current_tilt_rad = 0.0
        self.running = True

        # ---------- ROS ----------
        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.pc_pub = self.create_publisher(
            PointCloud2,
            "/lidar_3d/points",
            10
        )

        # ---------- Data ----------
        self.angle_bins = {}      # tilt_bin -> [(x,y,z)]
        self.points = []
        self.point_count = 0

        # ---------- Threads ----------
        threading.Thread(target=self.read_servo_angle, daemon=True).start()

        # ---------- Open3D ----------
        if ENABLE_OPEN3D:
            self.init_open3d()

    # =========================
    # SERVO ANGLE READER
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
    # LIDAR CALLBACK
    # =========================
    def scan_callback(self, msg):
        tilt_deg = math.degrees(self.current_tilt_rad)
        tilt_bin = round(tilt_deg / ANGLE_RESOLUTION_DEG) * ANGLE_RESOLUTION_DEG
        tilt_rad = math.radians(tilt_bin)

        angle = msg.angle_min
        new_slice = []

        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE:
                x = r * math.cos(tilt_rad) * math.cos(angle)
                y = r * math.cos(tilt_rad) * math.sin(angle)
                z = r * math.sin(tilt_rad)
                new_slice.append((x, y, z))
            angle += msg.angle_increment

        if not new_slice:
            return

        # 🔁 Replace slice for this tilt angle
        self.angle_bins[tilt_bin] = new_slice

        # Rebuild cloud
        all_pts = []
        for pts in self.angle_bins.values():
            all_pts.extend(pts)

        self.points = all_pts[-MAX_BUFFER_POINTS:]
        self.point_count = len(self.points)

        # ---------- Save ----------
        with open(self.filename, "a") as f:
            for p in new_slice:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

        # ---------- Publish ----------
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar"

        cloud_msg = point_cloud2.create_cloud_xyz32(header, self.points)
        self.pc_pub.publish(cloud_msg)

        # ---------- Visualize ----------
        if ENABLE_OPEN3D:
            self.update_open3d()

        if self.point_count % 10000 == 0:
            self.get_logger().info(f"Points in cloud: {self.point_count}")

    # =========================
    # OPEN3D
    # =========================
    def init_open3d(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("3D LiDAR", width=900, height=700)
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

    def update_open3d(self):
        if not self.points:
            return
        self.pcd.points = o3d.utility.Vector3dVector(
            np.asarray(self.points)
        )
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    # =========================
    # CLEAN SHUTDOWN (NO ROS SHUTDOWN HERE)
    # =========================
    def shutdown(self):
        self.running = False

        try:
            if ENABLE_OPEN3D:
                self.vis.destroy_window()
        except:
            pass

        try:
            if self.ser:
                self.ser.close()
        except:
            pass

        try:
            if self.lidar_process:
                os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGINT)
        except:
            pass

        self.destroy_node()

# =========================
# MAIN
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
        print(f"\nSaved {node.point_count} points")

if __name__ == "__main__":
    main()
