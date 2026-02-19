import open3d as o3d
import numpy as np
import os
import matplotlib.cm as cm


# =========================================================
# Load xyz / Rxyz
# =========================================================
def load_xyz(filename):
    pts = []

    with open(filename, 'r') as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                except:
                    pass

    return np.asarray(pts)


# =========================================================
# Build point cloud with filters
# =========================================================
def make_cloud(points, state):

    pts = points.copy()

    # ---------- scaling (optional) ----------
    pts[:, 2] *= 1.0

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    # =====================================================
    # 1) VOXEL DOWNSAMPLE
    # =====================================================
    if state["voxel_enabled"]:
        pcd = pcd.voxel_down_sample(state["voxel_size"])

    # =====================================================
    # 2) STATISTICAL OUTLIER REMOVAL (SOR)
    # =====================================================
    if state["sor_enabled"]:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=state["sor_neighbors"],
            std_ratio=state["sor_std"]
        )

    pts = np.asarray(pcd.points)

    # =====================================================
    # Depth coloring
    # =====================================================
    dist = np.linalg.norm(pts, axis=1)

    d_min, d_max = np.percentile(dist, [2, 98])
    dist = np.clip(dist, d_min, d_max)
    d_norm = (dist - d_min) / (d_max - d_min + 1e-8)

    colors = cm.viridis(1.0 - d_norm)[:, :3]

    fog_strength = 0.65
    fog = (d_norm ** 1.5)[:, None]
    colors *= (1.0 - fog_strength * fog)

    pcd.colors = o3d.utility.Vector3dVector(colors)

    # normals AFTER filtering
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=max(state["voxel_size"], 0.03) * 3,
            max_nn=30
        )
    )

    return pcd


# =========================================================
# Main
# =========================================================
def main():

    # runtime state (NO GLOBALS)
    state = {
        "voxel_enabled": True,
        "voxel_size": 0.03,

        "sor_enabled": True,
        "sor_neighbors": 20,
        "sor_std": 1.5
    }

    # gather scans
    files = sorted(
        [f for f in os.listdir('.') if f.endswith('.xyz') or f.endswith('.Rxyz')],
        key=os.path.getctime
    )

    if not files:
        print("No scans found")
        return

    current_idx = len(files) - 1

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window("LiDAR Scan Browser + Filters", 1280, 720)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0, 0, 0])
    opt.point_size = 2.5
    opt.light_on = True
    opt.show_coordinate_frame = True

    geometry = None


    # -----------------------------------------------------
    # reload cloud
    # -----------------------------------------------------
    def load_index(idx):
        nonlocal geometry, current_idx

        current_idx = idx
        filename = files[current_idx]

        pts = load_xyz(filename)
        geometry = make_cloud(pts, state)

        vis.clear_geometries()
        vis.add_geometry(geometry)

        view = vis.get_view_control()
        view.set_lookat(geometry.get_center())
        view.set_front([0.7, -0.3, -0.6])
        view.set_up([0, 0, 1])
        view.set_zoom(0.45)

        vis.update_renderer()

        print(
            f"[{current_idx+1}/{len(files)}] {filename} | "
            f"voxel={state['voxel_enabled']} {state['voxel_size']:.3f} | "
            f"SOR={state['sor_enabled']} k={state['sor_neighbors']} std={state['sor_std']}"
        )


    # =====================================================
    # KEY CALLBACKS
    # =====================================================
    def prev_scan(vis):
        if current_idx > 0:
            load_index(current_idx - 1)
        return False

    def next_scan(vis):
        if current_idx < len(files) - 1:
            load_index(current_idx + 1)
        return False


    # ---------- voxel ----------
    def toggle_voxel(vis):
        state["voxel_enabled"] = not state["voxel_enabled"]
        load_index(current_idx)
        return False

    def voxel_smaller(vis):
        state["voxel_size"] *= 0.3
        load_index(current_idx)
        return False

    def voxel_larger(vis):
        state["voxel_size"] *= 1.25
        load_index(current_idx)
        return False


    # ---------- SOR ----------
    def toggle_sor(vis):
        state["sor_enabled"] = not state["sor_enabled"]
        load_index(current_idx)
        return False

    def sor_weaker(vis):
        state["sor_neighbors"] = max(5, state["sor_neighbors"] - 5)
        load_index(current_idx)
        return False

    def sor_stronger(vis):
        state["sor_neighbors"] += 5
        load_index(current_idx)
        return False


    # =====================================================
    # register keys
    # =====================================================
    vis.register_key_callback(ord("P"), prev_scan)
    vis.register_key_callback(ord("N"), next_scan)

    vis.register_key_callback(ord("V"), toggle_voxel)
    vis.register_key_callback(ord("["), voxel_smaller)
    vis.register_key_callback(ord("]"), voxel_larger)

    vis.register_key_callback(ord("O"), toggle_sor)
    vis.register_key_callback(ord(","), sor_weaker)
    vis.register_key_callback(ord("."), sor_stronger)


    load_index(current_idx)

    print("""
Controls
--------
P/N : prev/next
V   : voxel toggle
[ ] : voxel size
O   : outlier toggle
, . : outlier strength
""")

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
