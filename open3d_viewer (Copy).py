import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.cm as cm

def load_xyz(filename):
    """
    Load points and optionally intensity from .xyz or .Rxyz file.
    Returns (points, intensities).
    Intensities is None if the file doesn't have a 4th column.
    """
    pts = []
    intensities = []
    
    print(f"Loading {filename}...")
    
    with open(filename, 'r') as f:
        for line in f:
            # Skip comments
            if line.startswith("#"):
                continue
            
            parts = line.split()
            
            # Ensure line has at least X, Y, Z
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    pts.append([x, y, z])
                    
                    # If there's a 4th column, treat it as intensity
                    if len(parts) >= 4:
                        intensities.append(float(parts[3]))
                except ValueError:
                    # Skip lines that can't be parsed as floats
                    continue

    points = np.array(pts)
    if intensities:
        return points, np.array(intensities)
    else:
        return points, None

def main():
    # --- 1. File Selection Logic ---
    if len(sys.argv) < 2:
        # Search for .Rxyz or .xyz files in current directory
        files = sorted(
            [f for f in os.listdir('.') if f.endswith('.Rxyz') or f.endswith('.xyz')], 
            key=os.path.getctime
        )
        if not files:
            print("Error: No .xyz or .Rxyz files found in current directory.")
            sys.exit(1)
        filename = files[-1]
        print(f"No file specified. Auto-loading latest file: {filename}")
    else:
        filename = sys.argv[1]

    # --- 2. Load Data ---
    points, intensities = load_xyz(filename)

    if points.size == 0:
        print(f"Error: No points found in '{filename}'.")
        sys.exit(1)

    print(f"Loaded {len(points)} points.")

    # --- 3. Create Open3D Point Cloud ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # --- 4. Coloring Logic (Industry Standard) ---
    if intensities is not None:
        print(f"Coloring by Intensity.")
        print(f"Raw Intensity Range: {intensities.min():.2f} - {intensities.max():.2f}")
        
        # Robust Normalization: Clip top/bottom 1% outliers.
        # This prevents highly reflective objects (license plates) from washing out the rest of the scene.
        p_min, p_max = np.percentile(intensities, [1, 99])
        i_clipped = np.clip(intensities, p_min, p_max)
        
        # Normalize to 0-1
        i_norm = (i_clipped - p_min) / (p_max - p_min + 1e-8)
        
        # Apply 'Jet' colormap (Standard Rainbow: Blue=Low, Red=High)
        colors = cm.jet(i_norm)[:, :3]
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
    else:
        print("No intensity data found. Coloring by Height (Z).")
        
        # Fallback: Color by Height (Z)
        z = points[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
        colors = cm.jet(z_norm)[:, :3]
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # --- 5. Visualization Setup ---
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Lidar Viewer - {filename}", width=1280, height=720)
    
    # Add the point cloud
    vis.add_geometry(pcd)
    
    # Configure Render Options for "Lidar Look"
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])  # Black background (Standard)
    opt.point_size = 2.0                          # Make points slightly thicker
    opt.show_coordinate_frame = True              # Show X,Y,Z axis helper
    
    # Initialize View Control (optional, sets a nice zoom if needed)
    view_ctl = vis.get_view_control()
    view_ctl.set_front([0, 0, 1])  # Top-down view approx
    view_ctl.set_lookat([0, 0, 0])
    view_ctl.set_up([0, 1, 0])
    view_ctl.set_zoom(0.8)

    print("Opening visualizer... (Close window to exit)")
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()