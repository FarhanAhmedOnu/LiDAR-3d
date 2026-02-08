import pickle
import numpy as np
import open3d as o3d
import math
import multiprocessing
import time

# --- CONFIGURATION ---
LIDAR_OFFSET_X = 0.01 
LIDAR_OFFSET_Z = 0.03
FILENAME = "lidar_sweep_data.pkl"

def load_data(filename):
    print(f"Loading {filename}...")
    with open(filename, "rb") as f:
        data = pickle.load(f)
    return data

def raw_to_xyz(data):
    """Converts raw polar coordinates to Cartesian (x,y,z)."""
    all_points = []
    for entry in data:
        tilt = entry['tilt']
        scan = entry['scan']
        
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        
        # Valid range mask (0.15m to 12.0m)
        mask = (ranges > 0.15) & (ranges < 12.0)
        r = ranges[mask]
        a = angles[mask]
        
        if len(r) == 0: continue

        # 1. Polar to Local Cartesian
        xl = r * np.cos(a)
        yl = r * np.sin(a)
        
        # 2. Rotate by Tilt & Apply Offsets
        cos_t = math.cos(tilt)
        sin_t = math.sin(tilt)
        
        x = cos_t * xl + LIDAR_OFFSET_X * cos_t
        y = yl
        z = -sin_t * xl + LIDAR_OFFSET_Z
        
        all_points.append(np.column_stack((x, y, z)))

    return np.vstack(all_points)

def apply_filters(pcd):
    """Applies Voxel, SOR, and Radius filters sequentially."""
    print("-----------------------------------------")
    print(f"Original Points: {len(pcd.points)}")

    # 1. Voxel Grid (Downsampling)
    # Merges points within 2cm cubes. Good for uniform density.
    pcd = pcd.voxel_down_sample(voxel_size=0.02)
    print(f"After Voxel Grid (0.02m): {len(pcd.points)}")

    # 2. Statistical Outlier Removal (SOR)
    # Removes points that are further away from their 20 nearest neighbors than 2.0x standard deviation.
    # Good for removing "ghost" points floating in the air.
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)
    print(f"After SOR Filter: {len(pcd.points)}")

    # 3. Radius Outlier Removal
    # Removes points that don't have at least 5 neighbors within 10cm.
    # Good for removing isolated dust specks.
    cl, ind = pcd.remove_radius_outlier(nb_points=5, radius=0.10)
    pcd = pcd.select_by_index(ind)
    print(f"After Radius Filter: {len(pcd.points)}")
    print("-----------------------------------------")
    
    return pcd

def view_worker(xyz_data, window_name, color):
    """Worker process to open a standalone window"""
    # Re-create cloud inside the process (safer than pickling Open3D objects)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz_data)
    pcd.paint_uniform_color(color)
    
    # Add coordinate frame for reference
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    
    print(f"Opening Window: {window_name}")
    o3d.visualization.draw_geometries([pcd, axes], window_name=window_name, width=800, height=600)

def main():
    # 1. Load & Convert
    try:
        raw_data = load_data(FILENAME)
    except FileNotFoundError:
        print(f"Could not find {FILENAME}. Run the recorder first!")
        return

    raw_xyz = raw_to_xyz(raw_data)
    
    # 2. Create Raw Cloud Object (for processing)
    pcd_raw = o3d.geometry.PointCloud()
    pcd_raw.points = o3d.utility.Vector3dVector(raw_xyz)
    
    # 3. Process
    pcd_processed = apply_filters(pcd_raw)
    
    # Extract processed XYZ for sending to worker
    processed_xyz = np.asarray(pcd_processed.points)

    print("\nLaunching separate windows...")
    print("Window 1: RED   (Raw Data)")
    print("Window 2: GREEN (Processed Data)")

    # 4. Multiprocessing Visualization
    # We pass numpy arrays (raw_xyz, processed_xyz) instead of Open3D objects to avoid errors
    p1 = multiprocessing.Process(target=view_worker, args=(raw_xyz, "RAW DATA (Red)", [1, 0, 0]))
    p2 = multiprocessing.Process(target=view_worker, args=(processed_xyz, "PROCESSED DATA (Green)", [0, 1, 0]))

    p1.start()
    p2.start()
    
    p1.join()
    p2.join()

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn') # Safer for GUI apps
    main()