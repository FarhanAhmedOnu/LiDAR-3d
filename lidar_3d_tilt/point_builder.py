# point_builder.py
import math
import time
from config import *

class PointBuilder:
    def __init__(self):
        self.angle_bins = {}
        # self.last_reset_time = 0.0 # No longer needed for clearing

    def process_scan(self, scan, tilt_rad):
        tilt_deg = math.degrees(tilt_rad)

        # Removed the self.angle_bins.clear() block to prevent points from vanishing.
        
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

                x = cos_t * xl + LIDAR_OFFSET_X * cos_t
                y = yl
                z = -sin_t * xl + LIDAR_OFFSET_Z

                # We include Z again at the end for the "Intensity" field to drive color
                pts.append((x, y, z, z)) 
            angle += scan.angle_increment

        self.angle_bins[tilt_bin] = pts

    def get_cloud(self):
        all_pts = []
        for pts in self.angle_bins.values():
            all_pts.extend(pts)
        return all_pts[-MAX_BUFFER_POINTS:]
