#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time
import math
import numpy as np
import open3d as o3d
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
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.latest_scan = None
        self.scan_received_event = threading.Event()
        
        # Flags for control
        self.is_running = True
        self.servo_ready = False

        # --- 3. Setup Serial (Arduino) ---
        self.arduino_port = '/dev/ttyACM0' 
        self.baud_rate = 9600
        self.ser = None
        
        if not self.connect_and_test_servo():
            self.stop_lidar_driver()
            sys.exit(1)

        # --- 4. Setup 3D Visualization ---
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name="3D Lidar Reconstruction (Press Q to Exit)")
        
        # Register 'Q' key to quit safely
        self.vis.register_key_callback(ord('Q'), self.quit_callback)
        
        # Create PointCloud
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        
        # Add coordinate frame
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        self.vis.add_geometry(origin)

        self.all_points = []

    def start_lidar_driver(self):
        """Starts the ROS 2 driver for C1 in a subprocess."""
        print("[System] Launching RPLIDAR C1 Driver...")
        try:
            # We use 'exec' style to make sure we can kill it later
            # Note: This assumes 'ros2' is in your path (you sourced setup.bash)
            self.lidar_process = subprocess.Popen(
                ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"],
                stdout=subprocess.DEVNULL,  # Hide generic logs to keep terminal clean
                stderr=subprocess.PIPE,     # Capture errors if needed
                preexec_fn=os.setsid        # Create a new process group so we can kill the whole tree
            )
            # Give it a moment to spin up
            time.sleep(3)
            if self.lidar_process.poll() is not None:
                print("[Error] LIDAR driver failed to start immediately.")
                print(self.lidar_process.stderr.read().decode())
                sys.exit(1)
            print("[System] LIDAR Driver Running.")
        except FileNotFoundError:
            print("[Error] 'ros2' command not found. Did you source ROS?")
            sys.exit(1)

    def stop_lidar_driver(self):
        """Safely stops the LIDAR driver to protect the motor."""
        if self.lidar_process:
            print("[System] Stopping LIDAR Driver...")
            # Send SIGINT (CTRL+C) to the process group
            os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGINT)
            try:
                self.lidar_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.lidar_process.kill()
            print("[System] LIDAR Stopped.")

    def connect_and_test_servo(self):
        """Connects to serial and performs a handshake check."""
        try:
            print(f"[Servo] Connecting to {self.arduino_port}...")
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=2)
            time.sleep(2) # Wait for Arduino reset

            # Handshake Test: Move to 90 degrees
            print("[Servo] Testing connection (Moving to 90)...")
            self.ser.flushInput()
            self.ser.write(b"90\n")
            
            # Read response
            start_time = time.time()
            while time.time() - start_time < 3.0: # 3 second timeout
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "Moved to" in line:
                        print(f"[Servo] Connection Confirmed: {line}")
                        self.servo_ready = True
                        return True
                time.sleep(0.1)
            
            print("[Error] Servo did not respond! Check power and cables.")
            return False

        except serial.SerialException as e:
            print(f"[Error] Could not open Serial port: {e}")
            return False

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received_event.set()

    def move_servo(self, angle):
        if not self.servo_ready: return False
        
        command = f"{angle}\n"
        self.ser.write(command.encode('utf-8'))
        
        # Block until confirmed
        while True:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if "Moved to" in line:
                    return True
            except serial.SerialException:
                return False

    def quit_callback(self, vis):
        """Called when 'Q' is pressed in Open3D window."""
        print("\n[User] Quit requested via GUI.")
        self.is_running = False
        return False # This doesn't close window immediately, we handle it in main loop

    def polar_to_cartesian_3d(self, ranges, angle_min, angle_inc, servo_angle_deg):
        tilt_rad = math.radians(servo_angle_deg - 90) 
        points = []
        angle = angle_min
        
        for r in ranges:
            if r > 0.05 and r < 12.0:
                # 2D Lidar coords
                x_l = r * math.cos(angle)
                y_l = r * math.sin(angle)
                
                # 3D Rotation (Pitch around Y axis)
                x_3d = x_l * math.cos(tilt_rad)
                y_3d = y_l 
                z_3d = x_l * math.sin(tilt_rad) 
                
                points.append([x_3d, y_3d, z_3d])
            angle += angle_inc
        return points

    def run_scanner(self):
        try:
            # Scanning Loop
            # We sweep up (60->120) then down (120->60)
            sweep_range = list(range(60, 120, 1)) + list(range(120, 60, -1))
            
            while self.is_running:
                for angle in sweep_range:
                    if not self.is_running: break
                    
                    # 1. Update Open3D Window
                    self.vis.poll_events()
                    self.vis.update_renderer()
                    
                    # 2. Move Servo
                    self.move_servo(angle)
                    
                    # 3. Wait for Scan
                    self.scan_received_event.clear()
                    # Wait up to 1 second for ROS data
                    start_wait = time.time()
                    got_data = False
                    while time.time() - start_wait < 1.0:
                        rclpy.spin_once(self, timeout_sec=0.05)
                        if self.scan_received_event.is_set():
                            got_data = True
                            break
                    
                    if not got_data:
                        print("[Warning] No laser data received. Is LIDAR spinning?")
                        continue

                    # 4. Process
                    scan = self.latest_scan
                    new_points = self.polar_to_cartesian_3d(
                        scan.ranges, scan.angle_min, scan.angle_increment, angle
                    )
                    
                    if new_points:
                        self.all_points.extend(new_points)
                        np_points = np.asarray(self.all_points)
                        self.pcd.points = o3d.utility.Vector3dVector(np_points)
                        
                        # Color by Z height
                        z_vals = np_points[:, 2]
                        if len(z_vals) > 0:
                            min_z, max_z = np.min(z_vals), np.max(z_vals)
                            rng = max_z - min_z if max_z != min_z else 1
                            colors = np.zeros_like(np_points)
                            colors[:, 1] = (z_vals - min_z) / rng
                            colors[:, 0] = 1 - colors[:, 1]
                            self.pcd.colors = o3d.utility.Vector3dVector(colors)
                        
                        self.vis.update_geometry(self.pcd)

        except KeyboardInterrupt:
            print("\n[User] Ctrl+C detected.")
        finally:
            self.cleanup()

    def cleanup(self):
        print("[System] Shutting down...")
        self.is_running = False
        
        # 1. Move Servo to Rest
        if self.ser and self.ser.is_open:
            try:
                print("[Servo] Parking at 90 degrees...")
                self.ser.write(b"90\n")
                time.sleep(0.5)
                self.ser.close()
            except:
                pass

        # 2. Stop ROS Node
        self.destroy_node()
        
        # 3. Stop LIDAR Driver
        self.stop_lidar_driver()
        
        # 4. Close GUI
        self.vis.destroy_window()
        print("[System] Cleanup Complete. Safe to unplug.")

def main(args=None):
    rclpy.init(args=args)
    scanner = Lidar3DScanner()
    scanner.run_scanner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()