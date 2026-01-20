#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import subprocess
import signal
import sys
import os

class Lidar3DScanner(Node):
    def __init__(self):
        super().__init__('lidar_3d_scanner')
        
        # --- 1. Auto-Start LIDAR Driver ---
        self.lidar_process = None
        self.start_lidar_driver()

        # --- 2. Setup ROS 2 Subscription ---
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.scan_received_event = threading.Event()
        
        self.is_running = True
        self.servo_ready = False

        # --- 3. Setup Serial (Arduino) ---
        self.arduino_port = '/dev/ttyACM0' 
        self.baud_rate = 9600
        self.ser = None
        
        if not self.connect_and_test_servo():
            self.stop_lidar_driver()
            sys.exit(1)

        # --- 4. Setup Matplotlib Visualization ---
        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("3D Lidar Scan (Close window to quit)")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.set_zlabel("Z (Height)")
        
        # Set fixed limits so the plot doesn't jump around
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-1, 1)

        # Storage for all points
        self.all_x = []
        self.all_y = []
        self.all_z = []

        # Handle window close event
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def on_close(self, event):
        print("\n[User] Window closed.")
        self.is_running = False

    def start_lidar_driver(self):
        print("[System] Launching RPLIDAR C1 Driver...")
        try:
            self.lidar_process = subprocess.Popen(
                ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            time.sleep(3)
        except FileNotFoundError:
            print("[Error] 'ros2' not found.")
            sys.exit(1)

    def stop_lidar_driver(self):
        if self.lidar_process:
            os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGINT)
            try:
                self.lidar_process.wait(timeout=5)
            except:
                self.lidar_process.kill()

    def connect_and_test_servo(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=2)
            time.sleep(2)
            self.ser.write(b"90\n")
            start = time.time()
            while time.time() - start < 3.0:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    if "Moved to" in line:
                        self.servo_ready = True
                        return True
                time.sleep(0.1)
            return False
        except serial.SerialException:
            return False

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received_event.set()

    def move_servo(self, angle):
        if not self.servo_ready: return False
        self.ser.write(f"{angle}\n".encode())
        while True:
            try:
                if "Moved to" in self.ser.readline().decode(): return True
            except: return False

    def polar_to_cartesian_3d(self, ranges, angle_min, angle_inc, servo_angle_deg):
        tilt_rad = math.radians(servo_angle_deg - 90) 
        angle = angle_min
        new_x, new_y, new_z = [], [], []
        
        for r in ranges:
            if 0.05 < r < 4.0: # Limit range to 4 meters for cleaner plot
                x_l = r * math.cos(angle)
                y_l = r * math.sin(angle)
                
                # Rotation logic
                x_3d = x_l * math.cos(tilt_rad)
                y_3d = y_l 
                z_3d = x_l * math.sin(tilt_rad) 
                
                new_x.append(x_3d)
                new_y.append(y_3d)
                new_z.append(z_3d)
            angle += angle_inc
        return new_x, new_y, new_z

    def run_scanner(self):
        # Enable interactive mode for Matplotlib
        plt.ion()
        plt.show()
        
        sweep_range = list(range(70, 110, 2)) # Smaller range for testing
        
        try:
            while self.is_running:
                for angle in sweep_range:
                    if not self.is_running: break
                    
                    self.move_servo(angle)
                    
                    self.scan_received_event.clear()
                    start_wait = time.time()
                    while not self.scan_received_event.is_set():
                        rclpy.spin_once(self, timeout_sec=0.05)
                        if time.time() - start_wait > 1.0: break
                    
                    if self.latest_scan:
                        xs, ys, zs = self.polar_to_cartesian_3d(
                            self.latest_scan.ranges, 
                            self.latest_scan.angle_min, 
                            self.latest_scan.angle_increment, 
                            angle
                        )
                        
                        # Add new points to storage
                        self.all_x.extend(xs)
                        self.all_y.extend(ys)
                        self.all_z.extend(zs)
                        
                        # Update Plot
                        # We clear and redraw (slower but simpler)
                        self.ax.clear()
                        self.ax.set_xlim(-2, 2)
                        self.ax.set_ylim(-2, 2)
                        self.ax.set_zlim(-1, 1)
                        self.ax.set_title(f"Scanning Angle: {angle}")
                        
                        # Plot points
                        self.ax.scatter(self.all_x, self.all_y, self.all_z, s=2, c=self.all_z, cmap='viridis')
                        
                        self.fig.canvas.draw()
                        self.fig.canvas.flush_events()
                        
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        print("Cleaning up...")
        self.is_running = False
        if self.ser:
            self.ser.write(b"90\n")
            self.ser.close()
        self.stop_lidar_driver()
        self.destroy_node()
        plt.close()

def main():
    rclpy.init()
    scanner = Lidar3DScanner()
    scanner.run_scanner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()