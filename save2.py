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
        
        # --- File Saving with Intensity ---
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"Rscan_{timestamp}.Rxyz"
        print(f"[System] Data will be saved to: {self.filename}")
        
        # Write header (now includes Intensity)
        with open(self.filename, "w") as f:
            f.write("# 3D Lidar Scan Data with Intensity\n")
            f.write("# Format: X Y Z Intensity\n")

        # --- Auto-Start LIDAR Driver ---
        self.lidar_process = None
        self.start_lidar_driver()

        # --- ROS 2 Subscription ---
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.scan_received_event = threading.Event()
        
        self.is_running = True
        self.servo_ready = False

        # --- Serial (Arduino) ---
        self.arduino_port = '/dev/ttyUSB1' 
        self.baud_rate = 115200
        self.ser = None
        
        if not self.connect_and_test_servo():
            print("[Error] Servo connection failed.")
            self.stop_lidar_driver()
            sys.exit(1)

        # --- Matplotlib Visualization (optional) ---
        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title(f"Recording to {self.filename} (Close to Stop)")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-1, 1)

        self.all_x, self.all_y, self.all_z, self.all_intensity = [], [], [], []
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
            time.sleep(2) # Wait for Arduino reset
            
            # Flush any startup messages (like "READY" from servo.ino)
            self.ser.reset_input_buffer()

            # Send Test Command
            print("[System] Testing servo connection...")
            self.ser.write(b"ANGLE:90\n") 
            
            start = time.time()
            while time.time() - start < 3.0:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    if "ACK_ANGLE:" in line:
                        self.servo_ready = True
                        print("[System] Servo handshake successful.")
                        return True
                time.sleep(0.1)
            return False
        except serial.SerialException as e:
            print(f"[Error] Serial exception: {e}")
            return False

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received_event.set()

    def move_servo(self, angle):
        if not self.servo_ready: return False
        
        command = f"ANGLE:{angle}\n"
        self.ser.write(command.encode())
        
        # Wait for acknowledgment
        start_wait = time.time()
        while time.time() - start_wait < 1.0: # 1 second timeout
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    if "ACK_ANGLE:" in line: 
                        return True
            except: 
                return False
            time.sleep(0.01)
        return False

    def polar_to_cartesian_3d(self, ranges, intensities, angle_min, angle_inc, servo_angle_deg):
        """Convert polar (r, angle) to 3D Cartesian, filtering valid points and returning intensities."""
        # Convert to numpy arrays for vectorised operations
        ranges = np.array(ranges)
        intensities = np.array(intensities) if intensities is not None else None
        angles = angle_min + np.arange(len(ranges)) * angle_inc

        # Valid range: between 0.05 m and 12 m (C1 max range) and finite
        valid = (ranges > 0.05) & (ranges < 12.0) & np.isfinite(ranges)
        if not np.any(valid):
            return [], [], [], []

        r_valid = ranges[valid]
        a_valid = angles[valid]
        i_valid = intensities[valid] if intensities is not None else None

        # Convert servo angle to radians (0° = vertical, so subtract 90)
        tilt_rad = math.radians(servo_angle_deg - 90)

        # 2D coordinates in LiDAR plane
        x_l = r_valid * np.cos(a_valid)
        y_l = r_valid * np.sin(a_valid)

        # Rotate around Y axis
        x_3d = x_l * np.cos(tilt_rad)
        z_3d = x_l * np.sin(tilt_rad)
        y_3d = y_l

        return x_3d, y_3d, z_3d, i_valid

    def save_points(self, xs, ys, zs, intensities):
        """Append points with intensity to the XYZ file."""
        if len(xs) == 0:
            return
        try:
            with open(self.filename, "a") as f:
                for x, y, z, i in zip(xs, ys, zs, intensities):
                    f.write(f"{x:.4f} {y:.4f} {z:.4f} {i:.2f}\n")
        except Exception as e:
            print(f"[Error] Failed to write to file: {e}")

    def run_scanner(self):
        plt.ion()
        plt.show()
        
        # Full sweep range (e.g., 60 to 120 degrees)
        sweep_range = list(range(60, 120, 1)) 
        
        try:
            # --- Perform exactly one sweep ---
            for angle in sweep_range:
                if not self.is_running:  # allow early exit if user closes window
                    break
                
                if self.move_servo(angle):
                    self.scan_received_event.clear()
                    start_wait = time.time()
                    while not self.scan_received_event.is_set():
                        rclpy.spin_once(self, timeout_sec=0.05)
                        if time.time() - start_wait > 1.0:
                            break
                    
                    if self.latest_scan:
                        # Extract intensities (if present)
                        intensities = self.latest_scan.intensities if hasattr(self.latest_scan, 'intensities') else None
                        
                        xs, ys, zs, ints = self.polar_to_cartesian_3d(
                            self.latest_scan.ranges,
                            intensities,
                            self.latest_scan.angle_min,
                            self.latest_scan.angle_increment,
                            angle
                        )
                        
                        # Save to file (with intensity)
                        self.save_points(xs, ys, zs, ints)

                        # Update visualization (color by intensity)
                        self.all_x.extend(xs)
                        self.all_y.extend(ys)
                        self.all_z.extend(zs)
                        self.all_intensity.extend(ints)
                        
                        self.ax.clear()
                        self.ax.set_xlim(-2, 2)
                        self.ax.set_ylim(-2, 2)
                        self.ax.set_zlim(-1, 1)
                        self.ax.set_title(f"Scanning Angle: {angle} | Points: {len(self.all_x)}")
                        
                        # Scatter with color mapped to intensity
                        sc = self.ax.scatter(self.all_x, self.all_y, self.all_z,
                                             s=2, c=self.all_intensity, cmap='viridis')
                        self.fig.colorbar(sc, ax=self.ax, label='Intensity')
                        
                        self.fig.canvas.draw()
                        self.fig.canvas.flush_events()
                else:
                    print(f"[Warning] Servo failed to move to {angle}")
            
            # Sweep complete – allow a moment to view the final plot (optional)
            # time.sleep(2)   # uncomment if you want to see the result before closing
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        print(f"\n[System] Saved {len(self.all_x)} points to '{self.filename}'")
        self.is_running = False
        if self.ser:
            self.ser.write(b"ANGLE:90\n")
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