import open3d as o3d
import numpy as np

def main():
    # Load the point cloud
    pcd = o3d.io.read_point_cloud("/home/heven/output.pcd")

    # Create a visualizer with editing capabilities
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # User picks points
    vis.destroy_window()

    # Get picked points indices
    picked_points_indices = vis.get_picked_points()
    picked_points = np.asarray(pcd.points)[picked_points_indices]

    # Print picked points
    for point in picked_points:
        print("Point coordinates:", point)

if __name__ == "__main__":
    main()