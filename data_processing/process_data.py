import pickle
import numpy as np
import open3d as o3d
import math
import time
import matplotlib.pyplot as plt

# ---------------- CONFIGURATION ----------------
LIDAR_OFFSET_X = 0.00
LIDAR_OFFSET_Z = 0.03
FILENAME = "lidar_sweep_data.pkl"


# ---------------- DATA LOADING ----------------
def load_data(filename):
    print(f"Loading {filename}...")
    with open(filename, "rb") as f:
        data = pickle.load(f)
    return data


# ---------------- POLAR → CARTESIAN ----------------
def raw_to_xyz(data):
    """Converts raw polar coordinates to Cartesian (x,y,z)."""
    all_points = []

    for entry in data:
        tilt = entry["tilt"]
        scan = entry["scan"]

        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        # Valid range mask
        mask = (ranges > 0.15) & (ranges < 12.0)
        r = ranges[mask]
        a = angles[mask]

        if len(r) == 0:
            continue

        # Polar to local Cartesian
        xl = r * np.cos(a)
        yl = r * np.sin(a)

        # Rotate by tilt & apply offsets
        cos_t = math.cos(tilt)
        sin_t = math.sin(tilt)

        x = cos_t * xl + LIDAR_OFFSET_X * cos_t
        y = yl
        z = -sin_t * xl + LIDAR_OFFSET_Z

        all_points.append(np.column_stack((x, y, z)))

    return np.vstack(all_points)


# ---------------- FILTERING ----------------
def apply_filters(pcd):
    print("-----------------------------------------")
    print(f"Original Points: {len(pcd.points)}")

    # 1. Voxel Downsample
    pcd = pcd.voxel_down_sample(voxel_size=0.02)
    print(f"After Voxel Grid (0.02m): {len(pcd.points)}")

    # 2. Statistical Outlier Removal
    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=20, std_ratio=2.0
    )
    pcd = pcd.select_by_index(ind)
    print(f"After SOR Filter: {len(pcd.points)}")

    # 3. Radius Outlier Removal
    cl, ind = pcd.remove_radius_outlier(
        nb_points=5, radius=0.10
    )
    pcd = pcd.select_by_index(ind)
    print(f"After Radius Filter: {len(pcd.points)}")
    print("-----------------------------------------")

    return pcd


# ---------------- COLORING ----------------
def colorize_by_height(pcd):
    """Apply smooth colormap based on Z height."""
    points = np.asarray(pcd.points)
    z = points[:, 2]

    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)

    cmap = plt.get_cmap("viridis")
    colors = cmap(z_norm)[:, :3]

    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


# ---------------- CAMERA SYNC ----------------
def synchronize_cameras(source_vis, target_vis):
    """Copy camera from source to target."""
    ctr_src = source_vis.get_view_control()
    ctr_tgt = target_vis.get_view_control()

    param = ctr_src.convert_to_pinhole_camera_parameters()
    ctr_tgt.convert_from_pinhole_camera_parameters(param)


# ---------------- MAIN ----------------
def main():
    # 1. Load Data
    try:
        raw_data = load_data(FILENAME)
    except FileNotFoundError:
        print(f"Could not find {FILENAME}.")
        return

    raw_xyz = raw_to_xyz(raw_data)

    # 2. Create Raw Cloud
    pcd_raw = o3d.geometry.PointCloud()
    pcd_raw.points = o3d.utility.Vector3dVector(raw_xyz)

    # 3. Processed Cloud
    pcd_processed = apply_filters(pcd_raw)

    # 4. Colorize both
    pcd_raw = colorize_by_height(pcd_raw)
    pcd_processed = colorize_by_height(pcd_processed)

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    # 5. Create Two Windows
    vis_raw = o3d.visualization.Visualizer()
    vis_processed = o3d.visualization.Visualizer()

    vis_raw.create_window("RAW DATA", width=800, height=600, left=50, top=50)
    vis_processed.create_window("PROCESSED DATA", width=800, height=600, left=900, top=50)

    vis_raw.add_geometry(pcd_raw)
    vis_raw.add_geometry(axes)

    vis_processed.add_geometry(pcd_processed)
    vis_processed.add_geometry(axes)

    print("\nMove the RAW window — PROCESSED will follow.\n")

    # 6. Event Loop with Camera Sync
    while True:
        vis_raw.poll_events()
        vis_raw.update_renderer()

        synchronize_cameras(vis_raw, vis_processed)

        vis_processed.poll_events()
        vis_processed.update_renderer()

        time.sleep(0.01)


if __name__ == "__main__":
    main()
