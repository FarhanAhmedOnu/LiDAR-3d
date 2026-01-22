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
# CONFIG
# =========================
ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 115200

LIDAR_LAUNCH = ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"]

MIN_RANGE = 0.05
MAX_RANGE = 12.0

ENABLE_OPEN3D = True
MAX_BUFFER_POINTS = 150_000

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
        self.points = []
        self.point_count = 0

        # ---------- Threads ----------
        threading.Thread(target=self.read_servo_angle, daemon=True).start()

        # ---------- Open3D ----------
        if ENABLE_OPEN3D:
            self.init_open3d()

    # =========================
    # SERVO READER
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
        tilt = self.current_tilt_rad
        angle = msg.angle_min

        new_pts = []

        for r in msg.ranges:
            if MIN_RANGE < r < MAX_RANGE:
                x = r * math.cos(tilt) * math.cos(angle)
                y = r * math.cos(tilt) * math.sin(angle)
                z = r * math.sin(tilt)

                new_pts.append((x, y, z))

            angle += msg.angle_increment

        if not new_pts:
            return

        # ---------- Store ----------
        self.points.extend(new_pts)
        self.points = self.points[-MAX_BUFFER_POINTS:]
        self.point_count += len(new_pts)

        # ---------- Save ----------
        with open(self.filename, "a") as f:
            for p in new_pts:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

        # ---------- Publish ----------
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar"

        cloud_msg = point_cloud2.create_cloud_xyz32(
            header, self.points
        )
        self.pc_pub.publish(cloud_msg)

        # ---------- Visualize ----------
        if ENABLE_OPEN3D:
            self.update_open3d()

        if self.point_count % 10000 == 0:
            self.get_logger().info(f"Points: {self.point_count}")

    # =========================
    # OPEN3D
    # =========================
    def init_open3d(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("3D LiDAR", width=900, height=700)
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

    def update_open3d(self):
        self.pcd.points = o3d.utility.Vector3dVector(
            np.asarray(self.points)
        )
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    # =========================
    # SHUTDOWN
    # =========================
    def shutdown(self):
        self.running = False

        if ENABLE_OPEN3D:
            self.vis.destroy_window()

        if self.ser:
            self.ser.close()

        if self.lidar_process:
            os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGINT)

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
        rclpy.shutdown()
        print(f"\nSaved {node.point_count} points")

if __name__ == "__main__":
    main()
