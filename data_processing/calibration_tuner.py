import pickle
import numpy as np
import open3d as o3d
import math

# INITIAL GUESSES (From your config)
params = {
    "offset_x": 0.01,
    "offset_z": 0.05,
    "tilt_adjust": 0.0
}

FILENAME = "lidar_sweep_data.pkl"

def load_data(filename):
    with open(filename, "rb") as f:
        return pickle.load(f)

def regenerate_cloud(data):
    """Re-calculates XYZ based on current params"""
    points = []
    
    ox = params["offset_x"]
    oz = params["offset_z"]
    t_adj = params["tilt_adjust"]

    for entry in data:
        # Apply manual tilt correction if IMU was slightly off
        tilt = entry['tilt'] + t_adj 
        scan = entry['scan']
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        mask = (ranges > 0.15) & (ranges < 12.0)
        r = ranges[mask]
        a = angles[mask]
        
        if len(r) == 0: continue

        # 1. Polar to Local Cartesian
        xl = r * np.cos(a)
        yl = r * np.sin(a)
        
        # 2. Rotate by Tilt & Apply Offsets (THE CRITICAL MATH)
        cos_t = math.cos(tilt)
        sin_t = math.sin(tilt)
        
        # If walls are spherical, these variables (ox, oz) are the culprits
        x = cos_t * xl + ox * cos_t
        y = yl
        z = -sin_t * xl + oz
        
        points.append(np.column_stack((x, y, z)))

    if not points: return np.zeros((1,3))
    return np.vstack(points)

def update_vis(vis, pcd, data):
    # Recalculate points
    xyz = regenerate_cloud(data)
    pcd.points = o3d.utility.Vector3dVector(xyz)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    
    print(f"Offsets -> X: {params['offset_x']:.3f} | Z: {params['offset_z']:.3f} | Tilt: {params['tilt_adjust']:.3f}")

def main():
    data = load_data(FILENAME)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(regenerate_cloud(data))
    
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=1000, height=800)
    vis.add_geometry(pcd)

    # --- CONTROLS ---
    # Q/A changes Offset X (Forward/Back)
    # W/S changes Offset Z (Up/Down)
    # E/D changes Tilt Zero (Pitch correction)
    
    def adjust(key, param, val):
        params[param] += val
        update_vis(vis, pcd, data)
        return False

    vis.register_key_callback(ord("Q"), lambda v: adjust("Q", "offset_x", 0.005))
    vis.register_key_callback(ord("A"), lambda v: adjust("A", "offset_x", -0.005))
    vis.register_key_callback(ord("W"), lambda v: adjust("W", "offset_z", 0.005))
    vis.register_key_callback(ord("S"), lambda v: adjust("S", "offset_z", -0.005))
    vis.register_key_callback(ord("E"), lambda v: adjust("E", "tilt_adjust", 0.01))
    vis.register_key_callback(ord("D"), lambda v: adjust("D", "tilt_adjust", -0.01))

    print("\n--- TUNING CONTROLS ---")
    print("  Q / A : Increase/Decrease X Offset (Depth distortion)")
    print("  W / S : Increase/Decrease Z Offset (Height distortion)")
    print("  E / D : Adjust Tilt Horizon")
    print("-----------------------")
    
    vis.run()
    vis.destroy_window()
    
    print("\nFINAL CALIBRATION VALUES:")
    print(params)
    print("Update your config.py with these values!")

if __name__ == "__main__":
    main()