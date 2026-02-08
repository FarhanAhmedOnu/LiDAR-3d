import open3d as o3d
import numpy as np
import sys
import os

def load_xyz(filename):
    pts = []
    with open(filename) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 3:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(pts)

if len(sys.argv) < 2:
    files = sorted([f for f in os.listdir('.') if f.endswith('.xyz')], key=os.path.getctime)
    # filename = "scan_20260120_150754.xyz"
    filename = files[-1]
else:
    filename = sys.argv[1]

points = load_xyz(filename)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Color by height (Z)
z = points[:, 2]
z_norm = (z - z.min()) / (z.max() - z.min())
colors = np.c_[z_norm, 1 - z_norm, np.zeros_like(z_norm)]
pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries(
    [pcd],
    window_name=filename,
    width=1280,
    height=720
)
