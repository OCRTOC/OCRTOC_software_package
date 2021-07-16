#! /usr/bin/env python3
# Author: Minghao Gou

import copy
import os
import sys
import yaml
import time
from collections import Counter
import open3d as o3d
import numpy as np
from transforms3d.euler import euler2mat
from itertools import permutations

from hash_rotation_matrix import HashableRotationMatrix

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import PoseStamped
import rospkg
import rospy

NUM_POINTS = 100
TABLE_WIDTH = 0.6
TABLE_LENGTH = 1.2
FIX_MAX_ERR = np.linalg.norm(np.array([TABLE_LENGTH, TABLE_WIDTH])) / 2.0
OBJECT_MAX_ERR_COEFFICIENT = 5.0
INF_DISTANCE = 9999999.99

class ScoreCalculator():
    def __init__(self, task_index, symmetric_metric = 'adds', debug = False):
        self.task_index = task_index
        self.log_file_path = os.path.join(
            rospkg.RosPack().get_path('ocrtoc_task'),
            'evaluation',
            '{}_score.txt'.format(self.task_index)
        )
        self.symmetric_metric = symmetric_metric
        if not self.symmetric_metric == 'adds':
            raise NotImplementedError('Currently, only ADDS metric is implemented')
        self.debug = debug
        self.object_list = self.load_object_list()
        self.alias_list = self.load_alias_list()
        self.mesh_name_dict = self.load_mesh_name_dict()
        self.object_points_dict = self.load_object_points_dict()
        self.mesh_property = self.load_mesh_property()
        rospy.loginfo('alias list:{}'.format(self.alias_list))
    
    def load_mesh_property(self):
        mesh_property_config = os.path.join(
            rospkg.RosPack().get_path('ocrtoc_materials'),
            'models',
            'models.yaml'
        )
        with open(mesh_property_config, "r") as f:
            mesh_property = yaml.load(f)
        rospy.logwarn('object_points_dict:{}'.format(self.object_points_dict.keys()))
        for mesh_name in mesh_property.keys():
            # mesh_property[mesh_name]['rotation_matrix_set'] = {HashableRotationMatrix.UnitRotationMatrix()}
            if mesh_name in self.object_points_dict.keys():
                max_dist, _ = self.get_farest_point(
                    self.object_points_dict[mesh_name],
                    self.object_points_dict[mesh_name]
                )
                mesh_property[mesh_name]['diameter'] = np.max(max_dist)
            if mesh_property[mesh_name]['x'] == 1 and mesh_property[mesh_name]['y'] == 1 and mesh_property[mesh_name]['z'] == 1:
                mesh_property[mesh_name]['symmetric'] = False
            else:
                mesh_property[mesh_name]['symmetric'] = True
        rospy.logwarn(mesh_property)
        return mesh_property

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
        for idx, key in enumerate(sorted(self.object_list)):
            alias = 'object-' + str(idx + 1) + '-' + key
            alias_list.append(alias)
        return alias_list

    def get_object_poses(self, type):
        """Get object poses

        Args:
            type(string): target or dump

        Returns:
            dict: key-mesh_name, content-[{'t':t, 'mat':mat}].
        """
        rospack = rospkg.RosPack()
        file_name = self.task_index + '.yaml'
        if type == 'target':
            yaml_path = os.path.join(rospack.get_path('ocrtoc_materials'),
                                                'targets', file_name)
        elif type == 'dump':
            yaml_path = os.path.join(rospack.get_path('ocrtoc_task'),
                                                'evaluation', file_name)
        else:
            raise TypeError('Unknown type:{}'.format(type))
        if not os.path.exists(yaml_path):
            raise ValueError('Yaml file {} doesn\'t exists'.format(yaml_path))
        rospy.loginfo("Load yaml file:{}".format(yaml_path))
        f = open(yaml_path, "r")
        pose_yaml_dict = yaml.load(f)
        f.close()
        object_pose = dict()
        for mesh_name in pose_yaml_dict.keys():
            mesh_pose = []
            for pose_list in pose_yaml_dict[mesh_name]:
                x, y, z, rx, ry, rz = pose_list
                t = np.array([x,y,z])
                mat = euler2mat(rx, ry, rz)
                mesh_pose.append({'t': t, 'mat': mat})
            object_pose[mesh_name] = mesh_pose
        return object_pose

    def get_points_dict(self, type):
        """Get posed points

        Args:
            type(string): target or dump

        Returns:
            dict: key-mesh_name, content-np.array
        """
        points_dict = dict()
        pose_dict = self.get_object_poses(type = type)
        for mesh_name in pose_dict.keys():
            points_list = []
            for instance_pose in pose_dict[mesh_name]:
                t = instance_pose['t']
                mat = instance_pose['mat']
                points = self.object_points_dict[mesh_name] # (NUM_POINTS, 3)
                new_points = np.matmul(mat, points.T).T + t
                points_list.append(new_points)
            points_dict[mesh_name] = points_list
        return points_dict

    def get_mesh_name_from_alias(self, alias):
        return alias.split('-'[-1])

    def load_mesh_name_dict(self):
        mesh_name_dict = dict()
        for alias in self.alias_list:
            mesh_name_dict[alias] = alias.split('-')[-1]
        return mesh_name_dict

    def load_object_points(self, mesh_name):
        rospack = rospkg.RosPack()
        mesh_path = os.path.join(rospack.get_path('ocrtoc_materials'),'models', mesh_name, 'visual.ply')
        mesh = o3d.io.read_triangle_mesh(mesh_path)
        pcd = mesh.sample_points_poisson_disk(NUM_POINTS)
        points = np.asarray(pcd.points)
        return points

    def load_object_points_dict(self):
        points_dict = dict()
        for idx, key in enumerate(sorted(self.object_list)):
            points_dict[key] = self.load_object_points(key)
        return points_dict
    
    def compute_add(self, points_1, points_2):
        dump_pcd = o3d.geometry.PointCloud()
        dump_pcd.points = o3d.utility.Vector3dVector(points_1)
        dump_pcd.paint_uniform_color([0.0, 0.0, 1.0])
        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(points_2)
        target_pcd.paint_uniform_color([1.0, 0.0, 0.0])
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
        lines = []
        for i in range(0, NUM_POINTS, 10):
            box, v1, v2 = self.draw_line(points_1[i], points_2[i])
            lines.append(box)
        if self.debug:
            o3d.visualization.draw_geometries([frame, target_pcd, dump_pcd] + lines)
        if not points_1.shape == points_2.shape:
            raise ValueError('The shape of points must be the same')
        add = np.mean(np.linalg.norm((points_1 - points_2), axis = 1))
        return add
        
    def get_closest_point(self, points_1, points_2):
        def norm(t):
            return np.sqrt(np.sum(t * t, axis=-1))
        points_1 = np.array(points_1)
        points_2 = np.array(points_2)
        points_1 = points_1[:, np.newaxis]
        points_2 = points_2[np.newaxis, :]
        dist = norm(points_1 - points_2)
        indices = np.argmin(dist, axis = -1)
        min_dist = dist[np.array(list(range(points_1.shape[0]))), indices]
        return min_dist, indices

    def get_farest_point(self, points_1, points_2):
        def norm(t):
            return np.sqrt(np.sum(t * t, axis=-1))
        points_1 = np.array(points_1)
        points_2 = np.array(points_2)
        points_1 = points_1[:, np.newaxis]
        points_2 = points_2[np.newaxis, :]
        dist = norm(points_1 - points_2)
        indices = np.argmax(dist, axis = -1)
        max_dist = dist[np.array(list(range(points_1.shape[0]))), indices]
        return max_dist, indices

    def compute_adds(self, points_1, points_2):
        # each of point in points_1 to best point in points_2
        min_dist, indices = self.get_closest_point(points_1, points_2)
        dump_pcd = o3d.geometry.PointCloud()
        dump_pcd.points = o3d.utility.Vector3dVector(points_1)
        dump_pcd.paint_uniform_color([0.0, 0.0, 1.0])
        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(points_2)
        target_pcd.paint_uniform_color([1.0, 0.0, 0.0])
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
        lines = []
        for i in range(0, NUM_POINTS, 10):
            box, v1, v2 = self.draw_line(points_1[i], points_2[indices[i]])
            lines.append(box)
        if self.debug:
            o3d.visualization.draw_geometries([frame, target_pcd, dump_pcd] + lines)
        return np.mean(min_dist)

    def calculate_score(self):
        score_list = []
        mesh_name_list = []
        target_points_dict = self.get_points_dict(type = 'target')
        dump_points_dict = self.get_points_dict(type = 'dump')
        log_file = open(self.log_file_path, 'w')
        log_file_dir = os.path.join(rospkg.RosPack().get_path('ocrtoc_task'), 'evaluation')
        if not os.path.exists(log_file_dir):
            os.makedirs(log_file_dir)
        for mesh_name in target_points_dict.keys():
            if len(target_points_dict[mesh_name]) != len(dump_points_dict[mesh_name]):
                rospy.logerr('Number of instances of an object in target and dump poses should be the same')
                raise ValueError('Number of instances of an object in target and dump poses don\'t match')
            mesh_min_err = INF_DISTANCE
            min_permu_err_list = [INF_DISTANCE for i in range(len(target_points_dict[mesh_name]))]
            for index_list in permutations(range(len(target_points_dict[mesh_name]))):
                permu_err_list = []
                rospy.loginfo('index_list for mesh {}:{}\n{}'.format(mesh_name, index_list, '=' * 40))
                log_file.write('index_list for mesh {}:{}\n{}\n'.format(mesh_name, index_list, '=' * 40))
                for target_idx, dump_idx in zip(range(len(target_points_dict[mesh_name])), index_list):
                    dump_points = dump_points_dict[mesh_name][dump_idx]
                    target_points = target_points_dict[mesh_name][target_idx]
                    max_bound = FIX_MAX_ERR
                    if self.symmetric_metric == 'adds':
                        if (not mesh_name in self.mesh_property.keys()) or (self.mesh_property[mesh_name]['symmetric'] == False):
                            score = self.compute_add(dump_points, target_points)
                            actual_score = min(score, max_bound)
                            log_file.write('mesh_name:{}, method: ADD, score={}, max_bound = {}, actual score = {}\n'.format(
                                mesh_name, score, max_bound, actual_score))
                            rospy.loginfo('mesh_name:{}, method: ADD, score={}, max_bound = {}, actual score = {}'.format(
                                mesh_name, score, max_bound, actual_score))
                        else:
                            score = self.compute_adds(dump_points, target_points)
                            actual_score = min(score, max_bound)
                            log_file.write('mesh_name:{}, method: ADDS, score={}, max_bound = {}, actual score = {}\n'.format(
                                mesh_name, score, max_bound, actual_score))
                            rospy.loginfo('mesh_name:{}, method: ADDS, score={}, max_bound = {}, actual score = {}'.format(
                                mesh_name, score, max_bound, actual_score))
                    else:
                        raise ValueError('Unknown metric')
                    permu_err_list.append(actual_score)
                mean_permu_err = np.mean(np.array(permu_err_list, dtype = np.float32))
                if mean_permu_err <= mesh_min_err:
                    mesh_min_err = mean_permu_err
                    min_permu_err_list = permu_err_list
            score_list += min_permu_err_list
            mesh_name_list += len(target_points_dict[mesh_name]) * [mesh_name]
        scene_score = np.mean(np.array(score_list))
        log_file.write('{}\n'.format('-' * 60 + '\n' + '-' * 60))
        for mesh_name, score in zip(mesh_name_list, score_list):
            log_file.write('{}: {}\n'.format(mesh_name, score))
        log_file.write('{}\nScene Index:{}, Symmetric object metric:{}\nScene mean score:{}'.format(
            '-' * 60 + '\n' + '-' * 60, self.task_index, self.symmetric_metric,  scene_score))
        rospy.loginfo('\033[032m\nScene mean score:{}\033[0m'.format(scene_score))
        return scene_score

    def draw_line(self, p1, p2, a = 1e-3, color = np.array((0.0,1.0,0.0))):
        '''Get a open3d.geometry.TriangleMesh of a line

        Args:
            p1(np.array): the first point.
            p2(np.array): the second point.
            a(float): the length of the square of the bottom face.
        Returns:
            open3d.geometry.TriangleMesh: the line.
        '''
        vertex_1 = o3d.geometry.TriangleMesh.create_box(1.5 * a,1.5 * a,1.5 * a)
        vertex_1.translate(p1 - np.array((0.75 * a, 0.75 * a, 0.75 * a)))
        vertex_1.vertex_colors = o3d.utility.Vector3dVector(np.tile(np.array((1.0,0,0)), (8, 1))) 
        vertex_2 = o3d.geometry.TriangleMesh.create_box(1.5 * a,1.5 * a,1.5 * a)
        vertex_2.translate(p2 - np.array((0.75 * a, 0.75 * a, 0.75 * a)))
        vertex_2.vertex_colors = o3d.utility.Vector3dVector(np.tile(np.array((1.0,0,0)), (8, 1))) 
        d = np.linalg.norm(p1 - p2)
        v1 = (p2 - p1) / d
        v2 = np.cross(np.array((0,0,1.0)), v1)
        v3 = np.cross(v1, v2)
        R = np.stack((v3, v2, v1)).astype(np.float64).T
        box = o3d.geometry.TriangleMesh.create_box(width = a, height = a, depth = d)
        box = box.translate(np.array((-a / 2, -a / 2, 0)))
        trans_matrix = np.vstack((np.hstack((R, np.zeros((3,1)))), np.array((0,0,0,1))))
        # print('trans_matrix:{}'.format(trans_matrix))
        box = box.transform(trans_matrix)
        box = box.translate(p1)
        box.vertex_colors = o3d.utility.Vector3dVector(np.tile(color, (8, 1))) 
        return box, vertex_1, vertex_2

    def vis_current_points(self):
        current_points_dict = self.get_current_points_dict()
        pcds = []
        for alias in self.alias_list:
            current_points = current_points_dict[alias]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(current_points)
            pcds.append(pcd)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)

if __name__ == '__main__':
    rospy.init_node('calculate score')
    try:
        task_index = rospy.get_param('~task_index')
        rospy.loginfo("Task index: " + task_index)
    except:
        print("Usage:")
        print("roslaunch ocrtoc_task calculate_score_from_dump_pose.launch task_index:=0-0")
        sys.exit()
    
    score_calculator = ScoreCalculator(task_index = task_index)
    score_calculator.calculate_score()

