import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.pyplot as plt

def load_xyz(filename):
    """ Loads X, Y, Z from file, ignoring intensity for the calculation. """
    pts = []
    print(f"Loading {filename}...")
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith("#"): continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                except ValueError:
                    continue
    return np.array(pts)

def main():
    # --- 1. Load File ---
    if len(sys.argv) < 2:
        files = sorted([f for f in os.listdir('.') if f.endswith('.Rxyz') or f.endswith('.xyz')], key=os.path.getctime)
        if not files:
            print("Error: No .xyz or .Rxyz files found.")
            sys.exit(1)
        filename = files[-1]
        print(f"Auto-loading latest file: {filename}")
    else:
        filename = sys.argv[1]

    points = load_xyz(filename)
    if points.size == 0:
        print("Error: No points loaded.")
        sys.exit(1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    print(f"Total points: {len(points)}")
    print("-" * 40)

    # --- 2. RANSAC Plane Fitting ---
    # distance_threshold: Points within this distance (e.g., 2cm) are considered part of the wall initially
    threshold = 0.02 
    print(f"Finding best fit plane (Threshold={threshold}m)...")
    
    # plane_model = [a, b, c, d] for equation ax + by + cz + d = 0
    # inliers = indices of points that fit the plane
    plane_model, inliers = pcd.segment_plane(distance_threshold=threshold,
                                             ransac_n=3,
                                             num_iterations=2000)

    [a, b, c, d] = plane_model
    print(f"Plane Equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # --- 3. Extract Wall Points ---
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    
    # Get the XYZ data of the wall points only
    wall_points = np.asarray(inlier_cloud.points)
    print(f"Points assigned to wall: {len(wall_points)}")

    # --- 4. Calculate RMSE ---
    # Distance from point (x,y,z) to plane ax+by+cz+d=0 is |ax+by+cz+d| / sqrt(a^2+b^2+c^2)
    # Open3D returns normalized coefficients (a^2+b^2+c^2 = 1), so we just need |ax+by+cz+d|
    
    distances = np.abs(np.dot(wall_points, np.array([a, b, c])) + d)
    
    rmse = np.sqrt(np.mean(distances ** 2))
    mean_error = np.mean(distances)
    std_dev = np.std(distances)

    print("-" * 40)
    print(f"RESULTS FOR {filename}")
    print(f"RMSE (Root Mean Square Error): {rmse*1000:.4f} mm")
    print(f"Mean Absolute Error:           {mean_error*1000:.4f} mm")
    print(f"Standard Deviation:            {std_dev*1000:.4f} mm")
    print("-" * 40)

    # --- 5. Histogram of Errors (Optional but helpful) ---
    plt.figure(figsize=(10, 5))
    plt.hist(distances * 1000, bins=50, color='blue', alpha=0.7)
    plt.title("Wall Flatness Error Distribution")
    plt.xlabel("Distance from Ideal Plane (mm)")
    plt.ylabel("Count")
    plt.grid(True)
    print("Close the histogram window to see the 3D view...")
    plt.show()

    # --- 6. Visualization ---
    # Wall = Red, Everything else = Grey
    inlier_cloud.paint_uniform_color([1, 0, 0])      # Red Wall
    outlier_cloud.paint_uniform_color([0.8, 0.8, 0.8]) # Grey Noise/Background
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="RMSE Visualizer (Red=Wall)", width=1280, height=720)
    vis.add_geometry(inlier_cloud)
    vis.add_geometry(outlier_cloud)
    
    # Add coordinate frame for context
    vis.get_render_option().point_size = 2.0
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()