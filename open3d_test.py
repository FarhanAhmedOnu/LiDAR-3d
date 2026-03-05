import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.cm as cm


# -------------------------------------------------
# Load xyz / Rxyz
# -------------------------------------------------
def load_xyz(filename):
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


# -------------------------------------------------
# Convert points → colored Open3D cloud
# -------------------------------------------------
def make_cloud(points):
    points = points.copy()

    # scaling
    points[:, 2] *= 1.0
    points[:, 0] *= 1
    points[:, 1] *= 1

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # depth coloring
    dist = np.linalg.norm(points, axis=1)
    d_min, d_max = np.percentile(dist, [2, 98])
    dist = np.clip(dist, d_min, d_max)
    d_norm = (dist - d_min) / (d_max - d_min + 1e-8)

    colors = cm.viridis(1.0 - d_norm)[:, :3]

    fog_strength = 0.65
    fog = (d_norm ** 1.5)[:, None]
    colors = colors * (1.0 - fog_strength * fog)

    pcd.colors = o3d.utility.Vector3dVector(colors)

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.15,
            max_nn=30
        )
    )

    return pcd


# -------------------------------------------------
# Main
# -------------------------------------------------
def main():

    # ---- collect all scans ----
    files = sorted(
        [f for f in os.listdir('.') if f.endswith('.xyz') or f.endswith('.Rxyz')],
        key=os.path.getctime
    )

    if not files:
        print("No scan files found")
        return

    # start at latest
    current_idx = len(files) - 1

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("LiDAR Scan Browser", 1280, 720)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 2.5
    opt.light_on = True
    opt.show_coordinate_frame = True

    geometry = None

    # -------------------------------------------
    # load + display function
    # -------------------------------------------
    def load_index(idx):
        nonlocal geometry, current_idx

        current_idx = idx
        filename = files[current_idx]

        pts, _ = load_xyz(filename)
        geometry = make_cloud(pts)

        vis.clear_geometries()
        vis.add_geometry(geometry)

        view = vis.get_view_control()
        view.set_lookat(geometry.get_center())
        view.set_front([0.7, -0.3, -0.6])
        view.set_up([0, 0, 1])
        view.set_zoom(0.45)

        vis.update_renderer()

        print(f"[{current_idx+1}/{len(files)}] Showing: {filename}")


    # -------------------------------------------
    # Key callbacks
    # -------------------------------------------
    def previous_scan(vis):
        if current_idx > 0:
            load_index(current_idx - 1)
        return False

    def next_scan(vis):
        if current_idx < len(files) - 1:
            load_index(current_idx + 1)
        return False


    # P = previous, N = next
    vis.register_key_callback(ord("P"), previous_scan)
    vis.register_key_callback(ord("N"), next_scan)

    # initial load
    load_index(current_idx)

    print("Controls:")
    print("   P → previous scan")
    print("   N → next scan")
    print("   Mouse → rotate/zoom")

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
