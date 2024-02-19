import rospkg
import os
import numpy as np
import yaml
import open3d as o3d
from pose_6d import load_model_pcd
# for training
ros_model_package_training  = "ocrtoc_materials"
# for evaluation
ros_model_package_eval      = "ocrtoc_materials_test"


def calculate_objects_boundingbox(object_names: list) :
    """
        Calculate bounding box of all objects in the ocrtoc material in object coordinate
        object_names
        :param
        :return a dictionary contains object with oriented bounding box
    """
    bbox_list = {}
    for object_name in object_names :
        model_path = os.path.join(rospkg.RosPack().get_path(ros_model_package_training), "models")
        try:
            point_cloud = load_model_pcd(object_name, model_path, type = 'collision.obj')
            bbox = point_cloud.get_axis_aligned_bounding_box()
        except:
            print(f"Can not load model from {model_path}, object: {object_name}/collision.obj ")
            continue
        bbox_list[object_name] = bbox
    return bbox_list

if __name__ == '__main__':
    # save boundingbox to ocrtoc_materials/models/bbox.yaml
    object_model_path = os.path.join(rospkg.RosPack().get_path(ros_model_package_training), "models")
    model_names = os.listdir(object_model_path)
    print (model_names)
    bbox_list = calculate_objects_boundingbox(model_names)

    # IO operation dump the bbox into
    bbox_points = {}
    for object_name, bbox in bbox_list.items():
        bbox_points[object_name] =  np.asarray(bbox.get_box_points())
        bbox_file = os.path.join(rospkg.RosPack().get_path(ros_model_package_training), "models", object_name, "bbox.npy")
        np.save(bbox_file,  bbox_points[object_name])
