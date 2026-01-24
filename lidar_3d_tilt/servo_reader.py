# servo_reader.py
import serial
import math
import threading

class ServoReader:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.current_tilt_rad = 0.0
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    self.current_tilt_rad = math.radians(float(line))
            except:
                pass

    def get_tilt_rad(self):
        return self.current_tilt_rad

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except:
            pass
