import pickle
import time
import os

class DataLogger:
    def __init__(self, filename="lidar_data.pkl"):
        self.filename = filename
        self.data = []
        self.start_time = time.time()
        print(f"Data Logger initialized. Saving to: {self.filename}")

    def record(self, scan_msg, tilt_rad):
        # We save a dictionary with the raw scan and the specific tilt at that moment
        entry = {
            'timestamp': time.time() - self.start_time,
            'tilt': tilt_rad,
            'scan': scan_msg  # Saving the full ROS object
        }
        self.data.append(entry)

    def save(self):
        if not self.data:
            print("No data to save.")
            return
            
        print(f"Saving {len(self.data)} frames to disk...")
        try:
            with open(self.filename, 'wb') as f:
                pickle.dump(self.data, f)
            print(f"Successfully saved {os.path.abspath(self.filename)}")
        except Exception as e:
            print(f"Failed to save data: {e}")