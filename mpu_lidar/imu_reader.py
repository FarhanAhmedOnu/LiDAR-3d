import serial
import math
import threading
import time

class IMUReader:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.current_pitch = 0.0
        self.offset = 0.0
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if "PITCH:" in line:
                    raw_val = float(line.split(":")[1])
                    self.current_pitch = raw_val - self.offset
            except:
                pass

    def calibrate(self):
        """Zeroes the IMU based on current position (assumed 90 deg)"""
        print("Calibrating IMU...")
        samples = []
        for _ in range(20):
            samples.append(self.current_pitch + self.offset)
            time.sleep(0.05)
        self.offset = sum(samples) / len(samples)
        print(f"Calibration done. Offset: {self.offset:.2f}")

    def get_tilt_rad(self):
        return math.radians(self.current_pitch)