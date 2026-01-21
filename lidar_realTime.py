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
        
        # --- 1. File Saving Setup ---
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"scan_{timestamp}.xyz"
        print(f"[System] Data will be saved to: {self.filename}")
        
        with open(self.filename, "w") as f:
            f.write("# 3D Lidar Scan Data\n")
            f.write("# Format: X Y Z\n")

        # --- 2. Auto-Start LIDAR Driver ---
        self.lidar_process = None
        self.start_lidar_driver()

        # --- 3. Setup ROS 2 Subscription ---
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.scan_received_event = threading.Event()
        
        self.is_running = True
        self.servo_ready = False
        
        # Store points in dictionary for fast refresh by angle
        self.points_dict = {}  # angle -> (xs, ys, zs)
        
        # Scan direction control
        self.scan_forward = True
        self.min_angle = 60
        self.max_angle = 120

        # --- 4. Setup Serial (Arduino) ---
        self.arduino_port = '/dev/ttyACM0' 
        self.baud_rate = 115200  # Increased baud rate for faster communication
        self.ser = None
        
        if not self.connect_and_test_servo():
            self.stop_lidar_driver()
            sys.exit(1)

        # --- 5. Setup Matplotlib Visualization ---
        plt.ion()  # Interactive mode ON
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title(f"Recording to {self.filename} (Close to Stop)")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-1, 1)
        
        # Optimize visualization
        self.scatter_plot = None
        self.last_update_time = time.time()
        self.update_interval = 0.033  # ~30 FPS
        
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
            print(f"[System] Connecting to Arduino at {self.arduino_port}...")
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=2)
            time.sleep(2)  # Wait for Arduino reset
            
            # Test communication
            self.ser.write(b"90\n")
            self.ser.flush()
            
            start = time.time()
            while time.time() - start < 3.0:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"[Arduino] {line}")
                    if "Moved to" in line:
                        self.servo_ready = True
                        print("[System] Servo ready!")
                        return True
                time.sleep(0.1)
            print("[Error] Servo not responding properly")
            return False
        except serial.SerialException as e:
            print(f"[Error] Serial connection failed: {e}")
            return False

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received_event.set()

    def move_servo_fast(self, angle):
        """Move servo as fast as possible"""
        if not self.servo_ready: 
            return False
        
        try:
            # Send angle command
            cmd = f"{angle}\n".encode()
            self.ser.write(cmd)
            self.ser.flush()
            
            # Wait for acknowledgment but with timeout
            start_time = time.time()
            while time.time() - start_time < 0.2:  # Reduced timeout for faster movement
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "Moved to" in line:
                        return True
                time.sleep(0.01)
            return True  # Assume move was successful even if we don't get confirmation
        except Exception as e:
            print(f"[Error] Servo communication error: {e}")
            return False

    def polar_to_cartesian_3d(self, ranges, angle_min, angle_inc, servo_angle_deg):
        tilt_rad = math.radians(servo_angle_deg - 90) 
        angle = angle_min
        new_x, new_y, new_z = [], [], []
        
        # Pre-calculate cos/sin for performance
        cos_tilt = math.cos(tilt_rad)
        sin_tilt = math.sin(tilt_rad)
        
        for r in ranges:
            if 0.05 < r < 4.0: 
                x_l = r * math.cos(angle)
                y_l = r * math.sin(angle)
                
                x_3d = x_l * cos_tilt
                y_3d = y_l 
                z_3d = x_l * sin_tilt 
                
                new_x.append(x_3d)
                new_y.append(y_3d)
                new_z.append(z_3d)
            angle += angle_inc
        return new_x, new_y, new_z

    def save_points(self, xs, ys, zs):
        """Appends new points to the XYZ file."""
        if not xs: return
        try:
            with open(self.filename, "a") as f:
                for x, y, z in zip(xs, ys, zs):
                    f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
        except Exception as e:
            print(f"[Error] Failed to write to file: {e}")

    def update_visualization(self):
        """Optimized visualization update"""
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return False
        
        # Collect all points from dictionary
        all_x, all_y, all_z = [], [], []
        for angle in self.points_dict:
            xs, ys, zs = self.points_dict[angle]
            all_x.extend(xs)
            all_y.extend(ys)
            all_z.extend(zs)
        
        if not all_x:
            return False
        
        # Update scatter plot
        if self.scatter_plot is None:
            self.scatter_plot = self.ax.scatter(all_x, all_y, all_z, s=2, c=all_z, cmap='viridis')
        else:
            # Update existing scatter plot data
            self.scatter_plot._offsets3d = (all_x, all_y, all_z)
        
        self.ax.set_title(f"Points: {len(all_x)} | Angles: {len(self.points_dict)}")
        
        # Update display
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        
        self.last_update_time = current_time
        return True

    def run_scanner(self):
        print("[System] Starting 3D scanner...")
        
        # Initialize first angle
        current_angle = self.min_angle
        
        try:
            while self.is_running:
                # Move servo to current angle
                if not self.move_servo_fast(current_angle):
                    print(f"[Warning] Failed to move to angle {current_angle}")
                
                # Wait for scan data
                self.scan_received_event.clear()
                start_wait = time.time()
                while not self.scan_received_event.is_set() and self.is_running:
                    rclpy.spin_once(self, timeout_sec=0.01)  # Reduced timeout
                    if time.time() - start_wait > 0.5:  # Reduced timeout
                        print(f"[Warning] Timeout waiting for scan at angle {current_angle}")
                        break
                
                if self.latest_scan:
                    # Convert scan to 3D points
                    xs, ys, zs = self.polar_to_cartesian_3d(
                        self.latest_scan.ranges, 
                        self.latest_scan.angle_min, 
                        self.latest_scan.angle_increment, 
                        current_angle
                    )
                    
                    # 1. Save to file
                    self.save_points(xs, ys, zs)
                    
                    # 2. Refresh points for this angle (replace old data)
                    self.points_dict[current_angle] = (xs, ys, zs)
                    
                    # 3. Update visualization (optimized)
                    self.update_visualization()
                
                # Determine next angle (bidirectional scanning)
                if self.scan_forward:
                    current_angle += 1
                    if current_angle >= self.max_angle:
                        self.scan_forward = False
                        # Jump back to 60 as requested
                        current_angle = self.min_angle
                else:
                    current_angle -= 1
                    if current_angle <= self.min_angle:
                        self.scan_forward = True
                        # Jump to 120 as requested
                        current_angle = self.max_angle
                
                # Small delay to prevent overwhelming the system
                time.sleep(0.02)  # Reduced delay for faster scanning
                
        except KeyboardInterrupt:
            print("\n[User] Scan interrupted by user")
        except Exception as e:
            print(f"[Error] Scanner error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        print(f"\n[System] Scan complete. Saved data to '{self.filename}'")
        print(f"[System] Total angles scanned: {len(self.points_dict)}")
        
        self.is_running = False
        
        # Return servo to center position
        if self.ser:
            try:
                self.ser.write(b"90\n")
                self.ser.flush()
                time.sleep(0.5)
                self.ser.close()
            except:
                pass
        
        # Stop lidar driver
        self.stop_lidar_driver()
        
        # Close node
        self.destroy_node()
        
        # Keep plot open for final viewing
        if plt.get_fignums():
            print("[System] Close the plot window to exit...")
            plt.ioff()
            plt.show()

def main():
    rclpy.init()
    scanner = Lidar3DScanner()
    
    try:
        scanner.run_scanner()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()