import serial
import time
import threading
from config import *

class FastServoController:
    def __init__(self, port, baud):
        print(f"Connecting to Servo on {port}...")
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2) # Vital: Wait for Arduino to reboot
            self.ser.reset_input_buffer()
        except Exception as e:
            print(f"SERVO ERROR: {e}")
            
        self.running = True

    def move_to(self, angle):
        """Sends absolute angle to Arduino"""
        try:
            msg = f"ANGLE:{angle:.2f}\n"
            self.ser.write(msg.encode())
        except Exception as e:
            print(f"Servo Write Error: {e}")

    def start_sweep(self):
        self.sweep_thread = threading.Thread(target=self._sweep_loop, daemon=True)
        self.sweep_thread.start()

    def _sweep_loop(self):
        print("Starting Servo Sweep...")
        while self.running:
            # Move Max
            self.move_to(TILT_MAX)
            time.sleep(1.2) # Give it time to physically move
            # Move Min
            self.move_to(TILT_MIN)
            time.sleep(1.2)