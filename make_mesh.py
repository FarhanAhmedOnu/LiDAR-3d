import open3d as o3d
import numpy as np
import sys
import os
import matplotlib.cm as cm

class LidarToMesh:
    def __init__(self):
        self.pcd = None
        self.mesh = None
        self.vis = None
        self.filename = "None"

    def load_xyz(self, filename):
        """Loads .xyz/.Rxyz files and handles parsing errors."""
        self.filename = filename
        pts = []
        intensities = []
        
        print(f"[1/5] Loading {filename}...")
        try:
            with open(filename, 'r') as f:
                for line in f:
                    if line.startswith("#"): continue
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                            if len(parts) >= 4:
                                intensities.append(float(parts[3]))
                        except ValueError:
                            continue
        except FileNotFoundError:
            print(f"Error: File '{filename}' not found.")
            sys.exit(1)

        # Create Open3D PointCloud
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector(np.array(pts))

        # Colorize based on intensity (or height if missing)
        if intensities:
            # Normalize intensities for coloring
            i_arr = np.array(intensities)
            p_min, p_max = np.percentile(i_arr, [1, 99])
            i_norm = (np.clip(i_arr, p_min, p_max) - p_min) / (p_max - p_min + 1e-8)
            colors = cm.viridis(i_norm)[:, :3]
        else:
            # Color by Z height
            z = np.array(pts)[:, 2]
            z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
            colors = cm.jet(z_norm)[:, :3]
            
        self.pcd.colors = o3d.utility.Vector3dVector(colors)
        print(f"      Loaded {len(pts)} points.")

    def preprocess(self):
        """Cleans noise and computes normals (crucial for meshing)."""
        print("[2/5] Pre-processing...")
        
        # 1. Statistical Outlier Removal (Cleans 'ghost' points)
        cl, ind = self.pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        self.pcd = self.pcd.select_by_index(ind)
        print(f"      Removed {len(ind)} outlier points.")

        # 2. Estimate Normals
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        # 3. Orient Normals (CRITICAL for LiDAR)
        # Since LiDAR scans from 0,0,0, all normals should face the origin.
        self.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
        print("      Normals computed and oriented.")

    def generate_mesh(self, depth=9):
        """Runs Poisson reconstruction and cleans low-density artifacts."""
        print(f"[3/5] Generating Surface (Poisson Depth={depth})...")
        
        # 1. Poisson Surface Reconstruction
        # 'depth' controls resolution. Higher (9-12) = more detail but slower.
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            self.pcd, depth=depth, width=0, scale=1.1, linear_fit=False
        )
        
        # 2. Crop low density vertices
        # Poisson creates a closed "bubble". We remove vertices that aren't supported by actual points.
        print("      Trimming low-density artifacts...")
        vertices_to_remove = densities < np.quantile(densities, 0.05) # Tune 0.05 if holes appear
        mesh.remove_vertices_by_mask(vertices_to_remove)
        
        # 3. Post-processing
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.7, 0.7, 0.7]) # Grey clay look
        
        self.mesh = mesh
        print(f"      Mesh generated with {len(np.asarray(mesh.vertices))} vertices.")

    def save_mesh(self, vis):
        """Callback to save the mesh."""
        out_name = os.path.splitext(self.filename)[0] + "_mesh.ply"
        print(f"\n[SAVE] Exporting smooth mesh to: {out_name}")
        o3d.io.write_triangle_mesh(out_name, self.mesh)
        print("[DONE] Saved.")
        return False

    def run_visualization(self):
        print("[4/5] Opening Viewer...")
        print("---------------------------------------")
        print("CONTROLS:")
        print("  [S] Save Mesh (.ply)")
        print("  [W] Toggle Wireframe")
        print("  [B] Toggle Back-face Culling")
        print("---------------------------------------")

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="Smooth Mesh Viewer", width=1280, height=720)

        # Add geometry
        vis.add_geometry(self.mesh)
        
        # Set nice viewing angle
        ctr = vis.get_view_control()
        ctr.set_front([-0.5, -0.5, 0.5])
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, 0, 1])
        
        # Render Options for "Smooth" look
        opt = vis.get_render_option()
        opt.mesh_show_back_face = True
        opt.background_color = np.asarray([0.1, 0.1, 0.1])
        
        # Register Save Key
        vis.register_key_callback(ord('S'), self.save_mesh)
        
        vis.run()
        vis.destroy_window()

def main():
    # File selection
    target_file = ""
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
    else:
        # Auto-find latest Rxyz or xyz
        files = sorted(
            [f for f in os.listdir('.') if f.endswith('.Rxyz') or f.endswith('.xyz')],
            key=os.path.getctime
        )
        if not files:
            print("No .xyz or .Rxyz files found.")
            return
        target_file = files[-1]

    # Pipeline
    app = LidarToMesh()
    app.load_xyz(target_file)
    app.preprocess()
    app.generate_mesh(depth=9) # Try depth=8 for speed, depth=10 for quality
    app.run_visualization()

if __name__ == "__main__":
    main()