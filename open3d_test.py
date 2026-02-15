import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.cm as cm

def load_xyz(filename):
    """
    Load points from .xyz or .Rxyz file.
    Returns points (N, 3).
    """
    pts = []
    print(f"Loading {filename}...")
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith("#"): continue
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except ValueError:
                        continue
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)

    return np.array(pts)

def main():
    # --- 1. File Selection Logic ---
    if len(sys.argv) < 2:
        files = sorted(
            [f for f in os.listdir('.') if f.endswith('.Rxyz') or f.endswith('.xyz')], 
            key=os.path.getctime
        )
        if not files:
            print("Error: No .xyz or .Rxyz files found in current directory.")
            sys.exit(1)
        filename = files[-1]
        print(f"Auto-loading latest file: {filename}")
    else:
        filename = sys.argv[1]

    # --- 2. Load Data ---
    points = load_xyz(filename)
    if points.size == 0:
        print(f"Error: No points found in '{filename}'.")
        sys.exit(1)

    print(f"Loaded {len(points)} points.")

    # --- 3. Create Open3D Point Cloud ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # --- 4. Depth Enhancement (Normals) ---
    print("Estimating normals for better 3D shading...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # --- 5. Coloring for Depth (The "Magma" Look) ---
    # We ignore intensity and force coloring by Z (Height) for depth perception
    print("Applying Depth-based coloring (Magma colormap)...")
    
    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    z_range = z_max - z_min
    
    if z_range == 0: z_range = 1e-8
    
    # Normalize Z to 0-1 range
    z_norm = (z - z_min) / z_range
    
    # Use 'magma' or 'inferno' - these are perceptually uniform and great for depth
    # Dark colors = Low, Bright colors = High
    colormap = cm.magma 
    colors = colormap(z_norm)[:, :3]

    pcd.colors = o3d.utility.Vector3dVector(colors)

    # --- 6. Visualization with Save Callback ---
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"Depth Viewer: {filename}", width=1280, height=720)

    def save_for_freecad(vis):
        output_name = os.path.splitext(filename)[0] + "_export.ply"
        print(f"\n[ACTION] 'S' pressed.")
        print(f"[SAVING] Exporting to '{output_name}'...")
        o3d.io.write_point_cloud(output_name, pcd, write_ascii=False)
        print(f"[SUCCESS] Saved. Closing viewer.")
        vis.destroy_window()
        return False

    vis.register_key_callback(ord('S'), save_for_freecad)
    vis.register_key_callback(ord('s'), save_for_freecad)

    vis.add_geometry(pcd)

    # --- 7. Aesthetic Render Options ---
    opt = vis.get_render_option()
    
    # Pure Black background makes the colored depth pop out
    opt.background_color = np.asarray([0.0, 0.0, 0.0]) 
    
    # Slightly larger points fill the gaps better
    opt.point_size = 3.5 
    
    # Turn on lighting (requires the normals we calculated in step 4)
    # This adds subtle shadows to the points based on their angle
    opt.light_on = True 

    # View Control
    view_ctl = vis.get_view_control()
    view_ctl.set_front([0.5, -0.5, 0.5]) # Angled view usually shows depth better than top-down
    view_ctl.set_lookat([0, 0, 0])
    view_ctl.set_up([0, 0, 1])
    view_ctl.set_zoom(0.7)

    print("---------------------------------------")
    print("Viewer controls:")
    print("  [Mouse] Rotate/Pan/Zoom")
    print("  [S]     Save as .ply and Close")
    print("---------------------------------------")
    
    vis.run()

if __name__ == "__main__":
    main()