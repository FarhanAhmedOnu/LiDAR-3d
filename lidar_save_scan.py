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
import datetime

class Lidar3DScanner(Node):
    def __init__(self):
        super().__init__('lidar_3d_scanner')
        
        # --- Config ---
        self.min_angle = 60
        self.max_angle = 120
        self.angle_step = 1    # Set to 2 or 3 to go even faster (lower resolution)
        self.render_every_n_steps = 3  # Only update plot every 3 moves to reduce stutter
        
        # --- File Saving ---
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"scan_{timestamp}.xyz"
        print(f"[System] Saving data to: {self.filename}")
        with open(self.filename, "w") as f:
            f.write("X Y Z\n")

        # --- Driver & ROS ---
        self.lidar_process = None
        self.start_lidar_driver()
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.scan_received_event = threading.Event()
        self.is_running = True

        # --- Serial / Servo ---
        self.arduino_port = '/dev/ttyACM0' 
        self.baud_rate = 115200
        self.ser = None
        if not self.connect_and_test_servo():
            self.stop_lidar_driver()
            sys.exit(1)

        # --- Visualization Data Structure ---
        # Dictionary stores points by angle: { 90: ([x...], [y...], [z...]) }
        # This allows us to overwrite data when we loop back.
        self.scan_data = {} 

        # --- Matplotlib Setup ---
        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        self.fig.canvas.mpl_connect('close_event', self.on_close)

    def setup_plot(self):
        self.ax.set_title(f"Realtime Scan (Looping {self.min_angle}-{self.max_angle})")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-1, 1)

    def on_close(self, event):
        self.is_running = False

    def start_lidar_driver(self):
        try:
            self.lidar_process = subprocess.Popen(
                ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, preexec_fn=os.setsid
            )
            time.sleep(3)
        except Exception as e:
            print(f"[Error] Lidar launch failed: {e}")
            sys.exit(1)

    def stop_lidar_driver(self):
        if self.lidar_process:
            os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGINT)

    def connect_and_test_servo(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            time.sleep(2)
            # Clear garbage
            self.ser.reset_input_buffer()
            return True
        except serial.SerialException:
            print("[Error] Could not connect to Arduino.")
            return False

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received_event.set()

    def move_servo(self, angle):
        # 1. Clear input buffer so we don't read old "Moved to" messages
        self.ser.reset_input_buffer()
        
        # 2. Send command
        self.ser.write(f"{angle}\n".encode())
        
        # 3. Fast Wait loop
        while True:
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode().strip()
                    if "Moved to" in line:
                        return True
                except:
                    pass
            # Very short sleep to prevent CPU hogging, but fast enough for response
            time.sleep(0.005) 

    def polar_to_cartesian_3d(self, ranges, angle_min, angle_inc, servo_angle_deg):
        tilt_rad = math.radians(servo_angle_deg - 90) 
        angle = angle_min
        xs, ys, zs = [], [], []
        
        # Downsample factor (take every 2nd point) for performance
        step = 2 
        
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if 0.05 < r < 4.0: 
                # Current lidar angle
                theta = angle_min + (i * angle_inc)
                
                x_l = r * math.cos(theta)
                y_l = r * math.sin(theta)
                
                # Rotate by servo tilt
                x_3d = x_l * math.cos(tilt_rad)
                y_3d = y_l 
                z_3d = x_l * math.sin(tilt_rad) 
                
                xs.append(x_3d)
                ys.append(y_3d)
                zs.append(z_3d)
        return xs, ys, zs

    def update_plot(self):
        # Flatten the dictionary to get all points
        all_x, all_y, all_z = [], [], []
        
        # We only grab points from the dictionary (which auto-refreshes angles)
        for ang in self.scan_data:
            dx, dy, dz = self.scan_data[ang]
            all_x.extend(dx)
            all_y.extend(dy)
            all_z.extend(dz)
            
        self.ax.clear()
        self.setup_plot()
        # s=1 makes points smaller and faster to render
        self.ax.scatter(all_x, all_y, all_z, s=1, c=all_z, cmap='viridis')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run_scanner(self):
        plt.ion()
        plt.show()
        
        current_angle = self.min_angle
        direction = 1 # 1 for up, -1 for down
        loop_counter = 0

        try:
            while self.is_running:
                # 1. Move Servo
                self.move_servo(current_angle)
                
                # 2. Get Lidar Data
                self.scan_received_event.clear()
                # Fast timeout loop
                start_wait = time.time()
                while not self.scan_received_event.is_set():
                    rclpy.spin_once(self, timeout_sec=0.01)
                    if time.time() - start_wait > 0.5: # Don't wait too long
                        break
                
                if self.latest_scan:
                    xs, ys, zs = self.polar_to_cartesian_3d(
                        self.latest_scan.ranges, 
                        self.latest_scan.angle_min, 
                        self.latest_scan.angle_increment, 
                        current_angle
                    )
                    
                    # 3. Store Data (Overwrites old data for this angle!)
                    self.scan_data[current_angle] = (xs, ys, zs)
                    
                    # 4. Save to file (Append mode)
                    with open(self.filename, "a") as f:
                        for x, y, z in zip(xs, ys, zs):
                            f.write(f"{x:.3f} {y:.3f} {z:.3f}\n")

                    # 5. Visualizer Update (Only every N steps)
                    if loop_counter % self.render_every_n_steps == 0:
                        self.update_plot()
                
                # 6. Calculate Next Angle (Ping-Pong Logic)
                current_angle += (self.angle_step * direction)
                if current_angle >= self.max_angle:
                    direction = -1
                    current_angle = self.max_angle
                elif current_angle <= self.min_angle:
                    direction = 1
                    current_angle = self.min_angle
                
                loop_counter += 1
                        
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        print("\n[System] Stopping...")
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