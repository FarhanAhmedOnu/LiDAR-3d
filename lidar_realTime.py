#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
import threading
import subprocess
import signal
import sys
import os
import datetime
from collections import defaultdict
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

class Lidar3DScanner(Node):
    def __init__(self):
        super().__init__('lidar_3d_scanner')
        
        # --- File Saving Setup ---
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"scan_{timestamp}.xyz"
        print(f"[System] Data will be saved to: {self.filename}")
        
        with open(self.filename, "w") as f:
            f.write("# 3D Lidar Scan Data\n")
            f.write("# Format: X Y Z\n")

        # --- Auto-Start LIDAR Driver ---
        self.lidar_process = None
        self.start_lidar_driver()

        # --- Setup ROS 2 Subscription ---
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.scan_received_event = threading.Event()
        
        self.is_running = True
        self.servo_ready = False
        
        # --- Scanning Parameters ---
        self.min_angle = 60
        self.max_angle = 120
        self.current_angle = self.min_angle
        self.angle_step = 1
        self.forward_direction = True  # True: min->max, False: max->min
        self.scan_delay = 0.05  # Default delay between scans
        
        # --- Data Storage with angle-based organization ---
        self.points_by_angle = defaultdict(lambda: [[], [], []])  # Store points by angle
        self.all_x, self.all_y, self.all_z = [], [], []
        
        # --- Setup Serial (Arduino) ---
        self.arduino_port = '/dev/ttyACM0' 
        self.baud_rate = 9600
        self.ser = None
        
        if not self.connect_and_test_servo():
            self.stop_lidar_driver()
            sys.exit(1)

        # --- Setup Visualization Windows ---
        self.setup_visualization()
        
    def setup_visualization(self):
        # Create main figure for 3D visualization
        self.fig = plt.figure(figsize=(14, 8))
        
        # Main 3D plot
        self.ax3d = self.fig.add_subplot(121, projection='3d')
        self.ax3d.set_title(f"3D Lidar Scan - Recording to: {self.filename}")
        self.ax3d.set_xlabel("X (m)")
        self.ax3d.set_ylabel("Y (m)")
        self.ax3d.set_zlabel("Z (m)")
        self.ax3d.set_xlim(-2, 2)
        self.ax3d.set_ylim(-2, 2)
        self.ax3d.set_zlim(-1, 1)
        
        # Control window area (matplotlib subplot)
        self.ax_control = self.fig.add_subplot(122)
        self.ax_control.axis('off')
        
        # Create control elements
        self.create_control_panel()
        
        # Initialize scatter plot
        self.scatter = self.ax3d.scatter([], [], [], s=2, c=[], cmap='viridis', alpha=0.8)
        
        # Add angle indicator text
        self.angle_text = self.ax3d.text2D(0.05, 0.95, f"Servo Angle: {self.current_angle}°", 
                                         transform=self.ax3d.transAxes, fontsize=12,
                                         bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        self.points_text = self.ax3d.text2D(0.05, 0.90, f"Points: 0", 
                                          transform=self.ax3d.transAxes, fontsize=12,
                                          bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        self.direction_text = self.ax3d.text2D(0.05, 0.85, f"Direction: {'Forward' if self.forward_direction else 'Reverse'}", 
                                             transform=self.ax3d.transAxes, fontsize=12,
                                             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        # Set tight layout
        plt.tight_layout()
        
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        
    def create_control_panel(self):
        # Create a rectangle for the control panel
        control_rect = [0.65, 0.3, 0.3, 0.6]  # [left, bottom, width, height]
        
        # Clear the axis for control panel
        self.ax_control.clear()
        self.ax_control.axis('off')
        
        # Add title for control panel
        self.ax_control.text(0.5, 0.95, "Control Panel", ha='center', va='top', 
                           fontsize=14, fontweight='bold', transform=self.ax_control.transAxes)
        
        # Create axes for slider
        slider_ax = self.fig.add_axes([0.7, 0.6, 0.2, 0.03])
        self.speed_slider = Slider(slider_ax, 'Speed', 0.01, 0.5, valinit=self.scan_delay, 
                                  valfmt='%0.3f s', color='#0066cc')
        
        # Create axes for angle display
        angle_ax = self.fig.add_axes([0.7, 0.5, 0.2, 0.05])
        angle_ax.axis('off')
        self.angle_display = angle_ax.text(0.5, 0.5, f"Current Angle: {self.current_angle}°", 
                                          ha='center', va='center', fontsize=12)
        
        # Create axes for buttons
        button_ax = self.fig.add_axes([0.7, 0.3, 0.2, 0.1])
        button_ax.axis('off')
        
        # Add direction toggle button
        self.direction_button = Button(button_ax, 'Switch Direction', color='lightgray', hovercolor='0.9')
        self.direction_button.on_clicked(self.toggle_direction)
        
        # Update function for slider
        def update_speed(val):
            self.scan_delay = val
        
        self.speed_slider.on_changed(update_speed)
        
    def toggle_direction(self, event):
        self.forward_direction = not self.forward_direction
        print(f"[System] Direction changed to: {'Forward' if self.forward_direction else 'Reverse'}")
        self.update_visualization()
        
    def on_close(self, event):
        print("\n[User] Window closed.")
        self.is_running = False

    def start_lidar_driver(self):
        print("[System] Launching RPLIDAR C1 Driver...")
        try:
            self.lidar_process = subprocess.Popen(
                ["ros2", "launch", "rplidar_ros", "rplidar_c1_launch.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            time.sleep(5)  # Increased wait time for driver initialization
            print("[System] Lidar driver started successfully")
        except FileNotFoundError:
            print("[Error] 'ros2' not found.")
            sys.exit(1)

    def stop_lidar_driver(self):
        if self.lidar_process:
            print("[System] Stopping Lidar driver...")
            os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGINT)
            try:
                self.lidar_process.wait(timeout=5)
            except:
                self.lidar_process.kill()
                self.lidar_process.wait()

    def connect_and_test_servo(self):
        max_attempts = 5
        for attempt in range(max_attempts):
            try:
                print(f"[System] Connecting to Arduino on {self.arduino_port} (attempt {attempt + 1}/{max_attempts})...")
                self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=2)
                time.sleep(2)
                
                # Test servo communication
                self.ser.write(b"90\n")
                self.ser.flush()
                
                start = time.time()
                while time.time() - start < 3.0:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if "Moved to" in line:
                            self.servo_ready = True
                            print("[System] Servo communication established")
                            return True
                self.ser.close()
                time.sleep(1)
            except serial.SerialException as e:
                print(f"[Error] Serial connection failed: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(2)
        print("[Error] Failed to establish servo communication")
        return False

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.scan_received_event.set()

    def move_servo(self, angle):
        if not self.servo_ready:
            return False
        
        # Clamp angle to valid range
        angle = max(self.min_angle, min(self.max_angle, angle))
        
        try:
            command = f"{angle}\n".encode()
            self.ser.write(command)
            self.ser.flush()
            
            # Wait for servo response
            start_time = time.time()
            while time.time() - start_time < 1.0:  # 1 second timeout
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "Moved to" in line:
                        self.current_angle = angle
                        return True
                time.sleep(0.01)
            return False
        except Exception as e:
            print(f"[Error] Servo movement failed: {e}")
            return False

    def polar_to_cartesian_3d(self, ranges, angle_min, angle_inc, servo_angle_deg):
        tilt_rad = math.radians(servo_angle_deg - 90)
        angle = angle_min
        new_x, new_y, new_z = [], [], []
        
        for r in ranges:
            if 0.05 < r < 4.0:  # Valid range checks
                x_l = r * math.cos(angle)
                y_l = r * math.sin(angle)
                
                x_3d = x_l * math.cos(tilt_rad)
                y_3d = y_l 
                z_3d = x_l * math.sin(tilt_rad)
                
                new_x.append(x_3d)
                new_y.append(y_3d)
                new_z.append(z_3d)
            angle += angle_inc
        return new_x, new_y, new_z

    def save_points(self, xs, ys, zs):
        """Appends new points to the XYZ file."""
        if not xs:
            return
        
        try:
            with open(self.filename, "a") as f:
                for x, y, z in zip(xs, ys, zs):
                    f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
        except Exception as e:
            print(f"[Error] Failed to write to file: {e}")

    def update_visualization(self):
        """Update the visualization with current data."""
        if not self.is_running:
            return
        
        # Collect all points
        all_x, all_y, all_z = [], [], []
        for angle_data in self.points_by_angle.values():
            xs, ys, zs = angle_data
            all_x.extend(xs)
            all_y.extend(ys)
            all_z.extend(zs)
        
        # Update scatter plot data
        if all_x:
            self.scatter._offsets3d = (all_x, all_y, all_z)
            
            # Color by height (z-value)
            colors = np.array(all_z)
            norm_colors = (colors - min(colors)) / (max(colors) - min(colors) + 1e-10)
            self.scatter.set_array(norm_colors)
        
        # Update text displays
        self.angle_text.set_text(f"Servo Angle: {self.current_angle}°")
        self.points_text.set_text(f"Points: {len(all_x)}")
        self.direction_text.set_text(f"Direction: {'Forward' if self.forward_direction else 'Reverse'}")
        
        # Update control panel display
        self.angle_display.set_text(f"Current Angle: {self.current_angle}°")
        
        # Draw the plot
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run_scanner(self):
        print("[System] Starting 3D scanner...")
        plt.ion()
        plt.show(block=False)
        
        try:
            while self.is_running and plt.fignum_exists(self.fig.number):
                # Determine next angle based on direction
                if self.forward_direction:
                    next_angle = self.current_angle + self.angle_step
                    if next_angle > self.max_angle:
                        next_angle = self.max_angle
                        self.forward_direction = False  # Reverse direction
                else:
                    next_angle = self.current_angle - self.angle_step
                    if next_angle < self.min_angle:
                        next_angle = self.min_angle
                        self.forward_direction = True  # Forward direction
                
                # Move servo
                if not self.move_servo(next_angle):
                    print(f"[Warning] Failed to move servo to {next_angle}°")
                    time.sleep(self.scan_delay)
                    continue
                
                # Wait for scan data
                self.scan_received_event.clear()
                scan_timeout = 2.0  # Increased timeout
                start_wait = time.time()
                
                while not self.scan_received_event.is_set() and self.is_running:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    if time.time() - start_wait > scan_timeout:
                        print(f"[Warning] Timeout waiting for scan data at angle {next_angle}°")
                        break
                
                if self.latest_scan:
                    # Clear previous points for this angle (refresh functionality)
                    if self.current_angle in self.points_by_angle:
                        self.points_by_angle[self.current_angle] = [[], [], []]
                    
                    # Convert scan data
                    xs, ys, zs = self.polar_to_cartesian_3d(
                        self.latest_scan.ranges, 
                        self.latest_scan.angle_min, 
                        self.latest_scan.angle_increment, 
                        self.current_angle
                    )
                    
                    # Store points for this angle
                    self.points_by_angle[self.current_angle] = [xs, ys, zs]
                    
                    # Save to file
                    self.save_points(xs, ys, zs)
                    
                    # Update visualization
                    self.update_visualization()
                
                # Apply delay from speed control
                time.sleep(self.scan_delay)
                
                # Process GUI events
                plt.pause(0.001)
                
        except KeyboardInterrupt:
            print("\n[System] Keyboard interrupt received")
        except Exception as e:
            print(f"[Error] Unexpected error: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        print(f"\n[System] Cleaning up...")
        print(f"[System] Saved scan data to '{self.filename}'")
        
        self.is_running = False
        
        # Return servo to center position
        if self.ser and self.servo_ready:
            try:
                self.ser.write(b"90\n")
                time.sleep(0.5)
                self.ser.close()
            except:
                pass
        
        # Stop lidar driver
        self.stop_lidar_driver()
        
        # Close node
        self.destroy_node()
        
        # Close plot
        if plt.fignum_exists(self.fig.number):
            plt.close(self.fig)
        
        print("[System] Cleanup complete")

def main():
    rclpy.init()
    scanner = Lidar3DScanner()
    
    try:
        scanner.run_scanner()
    except Exception as e:
        print(f"[Error] Main loop failed: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()