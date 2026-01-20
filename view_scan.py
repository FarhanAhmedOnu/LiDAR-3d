#!/usr/bin/env python3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def load_xyz(filename):
    xs, ys, zs = [], [], []
    print(f"Loading {filename}...")
    try:
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith("#"): continue # Skip comments
                parts = line.strip().split()
                if len(parts) >= 3:
                    xs.append(float(parts[0]))
                    ys.append(float(parts[1]))
                    zs.append(float(parts[2]))
        return xs, ys, zs
    except Exception as e:
        print(f"Error reading file: {e}")
        return [], [], []

def main():
    # Check if user provided a filename argument
    if len(sys.argv) < 2:
        # If not, find the most recent .xyz file in the current folder
        files = [f for f in os.listdir('.') if f.endswith('.xyz')]
        if not files:
            print("No .xyz files found in this folder!")
            print("Usage: python3 view_scan.py <filename.xyz>")
            return
        # Sort by creation time to get the newest one
        files.sort(key=os.path.getctime)
        filename = files[-1]
        print(f"No file specified. Opening newest scan: {filename}")
    else:
        filename = sys.argv[1]

    # Load Data
    x, y, z = load_xyz(filename)
    
    if not x:
        print("File is empty or invalid.")
        return

    print(f"Displaying {len(x)} points.")

    # Visualize
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(f"Viewing: {filename}")
    
    # Plot using scatter
    # s=1 makes points small, c=z colors them by height
    scatter = ax.scatter(x, y, z, s=1, c=z, cmap='viridis')
    
    # Add a color bar to indicate height
    fig.colorbar(scatter, label='Height (Z)')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Initial view angle
    ax.view_init(elev=30, azim=45)
    
    print("Close the window to exit.")
    plt.show()

if __name__ == "__main__":
    main()