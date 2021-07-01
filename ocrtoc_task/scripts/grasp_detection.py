#! /usr/bin/env python
# Author: Minghao Gou

import copy
import os
import sys
import yaml
import time
from collections import Counter
import open3d as o3d
import numpy as np
from transforms3d.quaternions import quat2mat

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import PoseStamped
import rospkg
import rospy

from ocrtoc_common.transform_interface import TransformInterface

NUM_POINTS = 100
FLYING_Z_THRESH = 0.02
GRASPING_DISTANCE_THRESH = 0.05

class Grasp_Detector():
    def __init__(self, task_index, log_file, update_frequency = 0.2, min_count = 5):
        self.task_index = task_index
        self.update_frequency = update_frequency
        self.log_file = log_file
        self.min_count = min_count
        
        rospy.wait_for_service('/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/get_model_state', GetModelState)
        self.grasped_alias_set = set()
        self.object_list = self.load_object_list()
        self.alias_list = self.load_alias_list()
        self.mesh_name_dict = self.load_mesh_name_dict()
        self.object_points_dict = self.load_object_points_dict()
        self.transform_manager = TransformInterface()
        self.running_flag = True
        rospy.loginfo('alias list:{}'.format(self.alias_list))

    def get_eelink_pose(self):
        numpy_trans = self.transform_manager.lookup_numpy_transform('world', 'panda_ee_link')
        return numpy_trans

    def load_object_list(self):
        rospack = rospkg.RosPack()
        file_name = self.task_index + '.yaml'
        yaml_path = os.path.join(rospack.get_path('ocrtoc_materials'),
                                                'targets', file_name)
        rospy.loginfo("Load yaml file: {}".format(yaml_path))
        with open(yaml_path, "r") as f:
            object_list = yaml.load(f)
        return object_list

    def load_alias_list(self):
        alias_list = []
        for key in sorted(self.object_list):
            index = 1
            for model in self.object_list[key]:
                alias = key + '_v' + str(index)
                alias_list.append(alias)
                index += 1
        return alias_list

    def get_ros_object_pose(self):
        object_pose_dict = dict()
        query_request = GetModelStateRequest()
        for alias in self.alias_list:
            query_request.model_name = alias
            pose_6d = self.get_model_state(query_request).pose
            object_pose_dict[alias] = pose_6d
        return object_pose_dict

    def get_4x4_object_poses(self):
        object_pose_dict = dict()
        ros_object_pose_dict = self.get_ros_object_pose()
        for alias in self.alias_list:
            ros_pose = ros_object_pose_dict[alias]
            t = np.array([ros_pose.position.x, ros_pose.position.y, ros_pose.position.z])
            quat = np.array([ros_pose.orientation.w, ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z])
            mat = quat2mat(quat)
            T = np.vstack((
                np.hstack((
                    mat,
                    t.reshape((3,1))
                )),
                np.array((0, 0, 0, 1))
            ))
            object_pose_dict[alias] = T
        return object_pose_dict
    
    def get_current_points_dict(self):
        current_points_dict = dict()
        pose_dict = self.get_4x4_object_poses()
        for alias in self.alias_list:
            T = pose_dict[alias] # (4, 4)
            points = self.object_points_dict[alias] # (NUM_POINTS, 3)
            new_points = np.matmul(T[:3, :3], points.T).T + T[:3, 3]
            current_points_dict[alias] = new_points
        return current_points_dict

    def get_mesh_name_from_alias(self, alias):
        return alias.split('-'[-1])

    def load_mesh_name_dict(self):
        mesh_name_dict = dict()
        for alias in self.alias_list:
            postfix = alias.split('_')[-1]
            mesh_name_dict[alias] = alias[:-(len(postfix)+1)]
        return mesh_name_dict

    def load_object_points(self, mesh_name):
        rospack = rospkg.RosPack()
        mesh_path = os.path.join(rospack.get_path('ocrtoc_materials'),'models', mesh_name, 'visual.ply')
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        pcd = mesh.sample_points_poisson_disk(NUM_POINTS)
        # o3d.visualization.draw_geometries([pcd])
        points = np.asarray(pcd.points)
        return points

    def load_object_points_dict(self):
        points_dict = dict()
        for alias in self.alias_list:
            points_dict[alias] = self.load_object_points(self.mesh_name_dict[alias])
        return points_dict
    
    def vis_current_points(self):
        current_points_dict = self.get_current_points_dict()
        pcds = []
        for alias in self.alias_list:
            current_points = current_points_dict[alias]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(current_points)
            pcds.append(pcd)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
        # o3d.visualization.draw_geometries([*pcds, frame])
    
    def get_flying_alias_list(self):
        flying_alias_list = []
        current_points_dict = self.get_current_points_dict()
        for alias in self.alias_list:
            current_points = current_points_dict[alias]
            min_z = np.min(current_points[:, 2])
            if min_z > FLYING_Z_THRESH:
                flying_alias_list.append(alias)
        return flying_alias_list

    def get_grasping_alias(self):
        distance_list = []
        eelink_pose = self.get_eelink_pose()
        object_poses = self.get_4x4_object_poses()
        for alias in self.alias_list:
            dt = object_poses[alias][:3, 3] - eelink_pose[:3, 3]
            distance_list.append(np.linalg.norm(dt))
        min_index = np.argmin(np.array(distance_list))
        min_distance = distance_list[min_index]
        if min_distance < GRASPING_DISTANCE_THRESH:
            return self.alias_list[min_index]
        else:
            return None

    def get_current_grasped_alias(self):
        grasping_alias = self.get_grasping_alias()
        flying_alias_list = self.get_flying_alias_list()
        if grasping_alias is not None and grasping_alias in flying_alias_list:
            return grasping_alias
        else:
            return None

    def monitor_main_loop(self):
        counter = Counter()
        self.running_flag = True
        while self.running_flag:
            current_grasped_alias = self.get_current_grasped_alias()
            if current_grasped_alias:
                counter.update([current_grasped_alias])
                if counter[current_grasped_alias] >= self.min_count:
                    self.grasped_alias_set.add(current_grasped_alias)
                    rospy.loginfo('Detected Grasped Alias:{}'.format(current_grasped_alias))
                    counter.clear()
                    continue
            else:
                counter.clear()
            
            time.sleep(self.update_frequency)
        rospy.logwarn('Received stop signal and grasp detector stops')

    def stop(self):
        self.running_flag = False
        rospy.loginfo('Sending stop signal to grasp detector')

    def write_result(self):
        log_dir = os.path.dirname(self.log_file)
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        with open(self.log_file, 'w') as log:
            for alias in self.grasped_alias_set:
                log.write('{}\n'.format(alias))
        rospy.loginfo('Write grasp detector log file')

if __name__ == '__main__':
    try:
        rospy.init_node('grasp_detection')
        task_index = rospy.get_param('~task_index')
        log_file = rospy.get_param('~log_file')
        rospy.loginfo("Task Index:{}".format(task_index))
    except:
        print("Usage:")
        print("roslaunch ocrtoc_task grasp_detection_simulator.launch task_index:=0-0")
        sys.exit()
    g = Grasp_Detector(task_index = task_index, log_file = log_file)
    g.monitor_main_loop()
