import math
from config import *

class PointBuilder:
    def __init__(self):
        self.angle_bins = {} # Key: tilt_bin, Value: List of (x,y,z,intensity)

    def process_scan(self, scan, tilt_rad):
        tilt_deg = math.degrees(tilt_rad)
        # Binning ensures old data is overwritten by new data at the same angle
        tilt_bin = round(tilt_deg / ANGLE_RESOLUTION_DEG) * ANGLE_RESOLUTION_DEG
        
        cos_t = math.cos(tilt_rad)
        sin_t = math.sin(tilt_rad)
        
        pts = []
        angle = scan.angle_min
        for r in scan.ranges:
            if 0.15 < r < 12.0:
                xl = r * math.cos(angle)
                yl = r * math.sin(angle)
                
                # Lever-arm compensation
                x = cos_t * xl + LIDAR_OFFSET_X * cos_t
                y = yl
                z = -sin_t * xl + LIDAR_OFFSET_Z
                
                # Intensity field = Z (for Rviz color mapping)
                pts.append((x, y, z, z))
            angle += scan.angle_increment
            
        self.angle_bins[tilt_bin] = pts

    def get_cloud(self):
        all_pts = []
        for bin_pts in self.angle_bins.values():
            all_pts.extend(bin_pts)
        return all_pts[-MAX_BUFFER_POINTS:]