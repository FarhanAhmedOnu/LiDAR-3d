# point_builder.py
import math
import time
from config import *

class PointBuilder:
    def __init__(self):
        self.angle_bins = {}
        self.last_reset_time = 0.0

    def process_scan(self, scan, tilt_rad):
        tilt_deg = math.degrees(tilt_rad)

        # Sweep reset (Fix #2B)
        if tilt_deg < SWEEP_RESET_THRESHOLD:
            if time.time() - self.last_reset_time > 1.0:
                self.angle_bins.clear()
                self.last_reset_time = time.time()

        tilt_bin = round(tilt_deg / ANGLE_RESOLUTION_DEG) * ANGLE_RESOLUTION_DEG
        tilt = math.radians(tilt_bin)

        cos_t = math.cos(tilt)
        sin_t = math.sin(tilt)

        angle = scan.angle_min
        pts = []

        for r in scan.ranges:
            if MIN_RANGE < r < MAX_RANGE:
                xl = r * math.cos(angle)
                yl = r * math.sin(angle)

                # Lever-arm compensated transform
                x = cos_t * xl + LIDAR_OFFSET_X * cos_t
                y = yl
                z = -sin_t * xl + LIDAR_OFFSET_Z

                pts.append((x, y, z))
            angle += scan.angle_increment

        # Fix #1: ALWAYS overwrite bin (even if empty)
        self.angle_bins[tilt_bin] = pts

    def get_cloud(self):
        all_pts = []
        for pts in self.angle_bins.values():
            all_pts.extend(pts)
        return all_pts[-MAX_BUFFER_POINTS:]
