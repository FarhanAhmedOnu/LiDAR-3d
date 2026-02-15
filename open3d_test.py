import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.cm as cm

class CameraMover:
    """Helper class to handle camera movement logic via Arrow Keys/WASD"""
    def __init__(self, vis, step=2.0):
        self.vis = vis
        self.step = step
    
    def pan(self, dx, dy):
        ctr = self.vis.get_view_control()
        ctr.camera_local_translate(dx, dy, 0)
        return False
        
    def zoom(self, dz):
        ctr = self.vis.get_view_control()
        ctr.camera_local_translate(0, 0, dz)
        return False

    def rotate(self, deg):
        ctr = self.vis.get_view_control()
        ctr.rotate(deg, 0)
        return False

def load_xyz(filename):
    """
    Load points and intensity from .xyz or .Rxyz file.
    Returns (points, intensities).
    """
    pts = []
    intensities = []
    
    print(f"Loading {filename}...")
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith("#"): continue
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                        # Capture intensity if available (Column 4)
                        if len(parts) >= 4:
                            intensities.append(float(parts[3]))
                    except ValueError:
                        continue
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)

    return np.array(pts), (np.array(intensities) if intensities else None)

def main():
    # --- 1. File Selection Logic ---
    if len(sys.argv) < 2:
        files = sorted(
            [f for f in os.listdir('.') if f.endswith('.Rxyz') or f.endswith('.xyz')], 
            key=os.path.getctime
        )
        if not files:
            print("Error: No .xyz or .Rxyz files found.")
            sys.exit(1)
        filename = files[-1]
        print(f"Auto-loading latest file: {filename}")
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

    # --- 4. Normals for 3D Shading ---
    # Even when coloring by intensity, normals allow the light to create shadows
    # which helps you see the "shape" of the object.
    print("Estimating normals (this helps depth perception)...")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # --- 5. Coloring by Intensity ---
    if intensities is not None:
        print(f"Coloring by Intensity (Turbo colormap).")
        
        # Robust Normalization: Clip top/bottom 2% outliers
        # This prevents one super-bright pixel from making the rest look dark
        i_min, i_max = np.percentile(intensities, [2, 98])
        
        # Avoid divide by zero
        if i_max - i_min == 0: i_max += 1e-8
            
        i_clipped = np.clip(intensities, i_min, i_max)
        i_norm = (i_clipped - i_min) / (i_max - i_min)
        
        # Using 'turbo' because low values are Blue (visible against black background)
        # 'magma' low values are Black, which would disappear.
        colors = cm.turbo(i_norm)[:, :3]
    else:
        print("Warning: No intensity found. Falling back to Height coloring.")
        z = points[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min())
        colors = cm.turbo(z_norm)[:, :3]

    pcd.colors = o3d.utility.Vector3dVector(colors)

    # --- 6. Visualization & Controls ---
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"Intensity Viewer: {filename}", width=1280, height=720)

    # Initialize Mover (Adjust 'step' to change speed)
    mover = CameraMover(vis, step=2.0)

    # --- Key Mappings ---
    # Arrows (Pan)
    vis.register_key_callback(265, lambda v: mover.pan(0, mover.step))    # Up
    vis.register_key_callback(264, lambda v: mover.pan(0, -mover.step))   # Down
    vis.register_key_callback(263, lambda v: mover.pan(-mover.step, 0))   # Left
    vis.register_key_callback(262, lambda v: mover.pan(mover.step, 0))    # Right

    # W/S (Zoom/Dolly)
    vis.register_key_callback(ord('W'), lambda v: mover.zoom(mover.step * 2))
    vis.register_key_callback(ord('S'), lambda v: mover.zoom(-mover.step * 2))
    
    # A/D (Rotate)
    vis.register_key_callback(ord('A'), lambda v: mover.rotate(-10.0))
    vis.register_key_callback(ord('D'), lambda v: mover.rotate(10.0))

    # X (Save)
    def save_ply(vis):
        output = os.path.splitext(filename)[0] + "_intensity_export.ply"
        print(f"[SAVING] to {output}...")
        o3d.io.write_point_cloud(output, pcd, write_ascii=False)
        return False
    vis.register_key_callback(ord('X'), save_ply)

    vis.add_geometry(pcd)

    # --- 7. Aesthetics ---
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.0, 0.0, 0.0]) # Pure Black
    opt.point_size = 3.5
    
    # Lighting ON makes 3D shapes visible even when colored by intensity
    opt.light_on = True 

    # View Control
    view_ctl = vis.get_view_control()
    view_ctl.set_front([0.5, -0.5, 0.5])
    view_ctl.set_lookat([0, 0, 0])
    view_ctl.set_up([0, 0, 1])
    view_ctl.set_zoom(0.7)

    print("---------------------------------------")
    print("CONTROLS:")
    print("  [Arrows] Pan")
    print("  [W / S]  Zoom In / Out")
    print("  [A / D]  Rotate")
    print("  [Mouse]  Orbit")
    print("  [X]      Save .ply")
    print("---------------------------------------")
    
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    main()