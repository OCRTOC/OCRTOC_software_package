import open3d as o3d
import rospkg
import os
import yaml
import numpy as np
import sys
# Load the obj file
# mesh = o3d.io.read_triangle_mesh("path/to/your/obj/file.obj")

# # Compute the bounding box
# bbox = mesh.get_axis_aligned_bounding_box()

# # Create a visualizer object
# vis = o3d.visualization.Visualizer()

# # Add the mesh and bounding box to the visualizer
# vis.create_window()
# vis.add_geometry(mesh)
# vis.add_geometry(bbox)

# # Set the camera position
# vis.get_view_control().set_front([0, 0, -1])
# vis.get_view_control().set_up([0, 1, 0])
# vis.get_view_control().set_zoom(0.5)

# # Run the visualizer
# vis.run()

ros_model_package_training = "ocrtoc_materials"

if __name__ == '__main__':
    if len(sys.argv) !=2:
        print("Please specify object name! ")
    object_name = sys.argv[1]
    object_model_dir  = os.path.join(rospkg.RosPack().get_path(ros_model_package_training), "models")
    object_model_path = os.path.join(rospkg.RosPack().get_path(ros_model_package_training), "models", object_name, "visual.ply")
    mesh = o3d.io.read_point_cloud(object_model_path)
    bbox_path   = os.path.join(rospkg.RosPack().get_path(ros_model_package_training), "models", object_name, "bbox.npy")
    bbox_points = np.load(bbox_path)
    obb = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(bbox_points))
    # create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.add_geometry(obb)
    vis.get_view_control().set_front([0, 0, -1])
    vis.get_view_control().set_up([0, 1, 0])
    vis.get_view_control().set_zoom(0.5)

    # # Run the visualizer
    vis.run()