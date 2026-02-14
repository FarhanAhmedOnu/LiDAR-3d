import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.cm as cm   # for colormap

def load_xyz(filename):
    """Load points and optionally intensity from .xyz file.
       Returns (points, intensities) where intensities is None if not present."""
    pts = []
    intensities = []
    with open(filename) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 3:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                pts.append([x, y, z])
                if len(parts) >= 4:
                    intensities.append(float(parts[3]))
    points = np.array(pts)
    if intensities:
        return points, np.array(intensities)
    else:
        return points, None

if len(sys.argv) < 2:
    files = sorted([f for f in os.listdir('.') if f.endswith('.Rxyz')], key=os.path.getctime)
    if not files:
        print("No .xyz files found.")
        sys.exit(1)
    filename = files[-1]
else:
    filename = sys.argv[1]

points, intensities = load_xyz(filename)

if points.size == 0:
    print(f"Error: No points in '{filename}'.")
    sys.exit(1)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Color the point cloud
if intensities is not None:
    # Normalise intensity to [0,1]
    i_norm = (intensities - intensities.min()) / (intensities.max() - intensities.min() + 1e-8)
    # Use matplotlib colormap (viridis) to get RGB colours
    colours = cm.viridis(i_norm)[:, :3]   # viridis returns RGBA, take first 3
    pcd.colors = o3d.utility.Vector3dVector(colours)
    print("Coloured by intensity")
else:
    # Fallback: colour by height (Z)
    z = points[:, 2]
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
    colours = np.c_[z_norm, 1 - z_norm, np.zeros_like(z_norm)]
    pcd.colors = o3d.utility.Vector3dVector(colours)
    print("No intensity data – coloured by height")

o3d.visualization.draw_geometries(
    [pcd],
    window_name=filename,
    width=1280,
    height=720
)