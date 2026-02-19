import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.cm as cm


def load_xyz(filename):
    """
    Load points and optionally intensity from .xyz or .Rxyz file.
    Returns (points, intensities).
    """
    pts = []
    intensities = []

    print(f"Loading {filename}...")

    with open(filename, 'r') as f:
        for line in f:
            if line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    pts.append([x, y, z])

                    if len(parts) >= 4:
                        intensities.append(float(parts[3]))
                except ValueError:
                    continue

    points = np.array(pts)
    if intensities:
        return points, np.array(intensities)
    else:
        return points, None


def main():
    # --- 1. File Selection ---
    if len(sys.argv) < 2:
        files = sorted(
            [f for f in os.listdir('.') if f.endswith('.xyz') or f.endswith('.Rxyz')],
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
        print("Error: No points loaded.")
        sys.exit(1)

    print(f"Loaded {len(points)} points.")

    # --- 3. Slight Z exaggeration (servo-based 3D scan aid) ---
    # points[:, 2] *= 1.4
    points[:, 2] *= 1.0
    
    xy_scale = 1.6   # try 1.2–2.0

    points[:, 0] *= 1   # X
    points[:, 1] *= 1   # Y

    # --- 4. Create Point Cloud ---
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # --- 5. Depth-based coloring (PRIMARY depth cue) ---
    dist = np.linalg.norm(points, axis=1)

    d_min, d_max = np.percentile(dist, [2, 98])
    dist = np.clip(dist, d_min, d_max)
    d_norm = (dist - d_min) / (d_max - d_min + 1e-8)

    # Perceptually uniform colormap
    colors = cm.viridis(1.0 - d_norm)[:, :3]

    # --- 6. Depth fog (strong human depth perception cue) ---
    fog_strength = 0.65
    fog = (d_norm ** 1.5)[:, None]
    colors = colors * (1.0 - fog_strength * fog)

    pcd.colors = o3d.utility.Vector3dVector(colors)

    # --- 7. Normals for lighting ---
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.15,
            max_nn=30
        )
    )

    # --- 8. Visualization ---
    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=f"Lidar Depth Viewer - {filename}",
        width=1280,
        height=720
    )

    vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 2.5
    opt.light_on = True
    opt.show_coordinate_frame = True

    # --- 9. Perspective camera (CRITICAL for depth) ---
    view = vis.get_view_control()
    view.set_lookat(pcd.get_center())
    view.set_front([0.7, -0.3, -0.6])   # angled view
    view.set_up([0, 0, 1])
    view.set_zoom(0.45)

    print("Opening viewer (close window to exit)")
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
