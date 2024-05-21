import os
import yaml
import numpy as np
from transforms3d.euler import euler2mat
from itertools import permutations
import ocrtoc_env.src.iou as iou
import ocrtoc_env.src.box as box
import matplotlib.pyplot as plt
import math
import mujoco
T_MAX = 120 #MAX time allowed for a task

class ScoreCalculator():
    def __init__(self, env, task_index, IoU, time_cost = 120, debug = False):
        self.env = env
        self.task_index = task_index.replace("_","-")
        self.IoU_threshold = IoU
        self.time_cost = time_cost
        self.log_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'evaluation',
            '{}_score.txt'.format(self.task_index))
       
        self.debug = debug
        self.object_list = self.load_object_list()

    def load_object_list(self):
        file_name = self.task_index + '.yaml'
        yaml_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),'ocrtoc_materials_mujoco',
                                                'targets', file_name)
        print("Load yaml file: {}".format(yaml_path))
        with open(yaml_path, "r") as f:
            object_list = yaml.safe_load(f)
        return object_list

    def get_6dpose_dump_simulator(self,env,task_index):
        output_yaml = {}
        for key in sorted(self.object_list):
            output_key = key
            output_pose = []
            index = 1
            for model in self.object_list[key]:
                alias = key + '_v' + str(index)
                body_id = mujoco.mj_name2id(env.unwrapped.model, mujoco.mjtObj.mjOBJ_BODY, alias)
                pos = env.unwrapped.data.xpos[body_id]
                quat = env.unwrapped.data.xquat[body_id]
                roll, pitch, yaw = self.euler_from_quaternion(quat[1],quat[2],quat[3],quat[0])
                xyzrpy = [pos[0], pos[1],
                pos[2], roll, pitch, yaw]
                for i in range(len(xyzrpy)):
                    xyzrpy[i] = round(xyzrpy[i], 4)
                output_pose.append(xyzrpy)
                index += 1
            output_yaml[output_key] = output_pose
        return output_yaml
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



    def get_target_object_poses(self):
        """Get object poses

        Args:
            type(string): target or dump

        Returns:
            dict: key-mesh_name, content-[{'t':t, 'mat':mat}].
        """
        object_pose = dict()
        for mesh_name in self.object_list.keys():
            mesh_pose = []
            for pose_list in self.object_list[mesh_name]:
                x, y, z, rx, ry, rz = pose_list
                t = np.array([x,y,z])
                mat = euler2mat(rx, ry, rz)
                mesh_pose.append({'t': t, 'mat': mat})
            object_pose[mesh_name] = mesh_pose
        return object_pose
    
    def get_dump_object_poses(self,env,task_index):
        """Get object poses

        Args:
            type(string): target or dump

        Returns:
            dict: key-mesh_name, content-[{'t':t, 'mat':mat}].
        """
        pose_yaml_dict = self.get_6dpose_dump_simulator(env,task_index)
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

    def draw_boxes(self,boxes = [], clips = [], colors = ['r', 'b', 'g' , 'k']):
        """Draw a list of boxes.

            The boxes are defined as a list of vertices
        """
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        for i, b in enumerate(boxes):
            x, y, z = b[:, 0], b[:, 1], b[:, 2]
            ax.scatter(x, y, z, c = 'r')
            for e in box.EDGES:
                ax.plot(x[e], y[e], z[e], linewidth=2, c=colors[i % len(colors)])

        if (len(clips)):
            points = np.array(clips)
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=100, c='k')

        plt.gca().patch.set_facecolor('white')
        ax.set_aspect('equal')
        # rotate the axes and update
        ax.view_init(30, 12)
        plt.draw()
        plt.show()

    def box_convert(self,bbox):
            """
            Convert boxes to proper vertices order

            """
            new_order = [0, 3, 2, 5, 1, 6, 7, 4]
            new_bbox = [bbox[i] for i in new_order]
            return new_bbox

    def compute_IoU(self,dump_pose, target_pose, mesh_name, debug):
        """
            Compute IoU(Intersection over Union) of two bounding boxes given names of objects

        """
        bbox_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),'ocrtoc_materials_mujoco',
                                                'models', mesh_name)
        object_bbox_gen = np.load(bbox_path+'/bbox.npy')
        object_bbox = self.box_convert(object_bbox_gen)
        center_point = sum(object_bbox)/8
        dump_bbox = np.zeros(shape=(9, 3))
        target_bbox = np.zeros(shape=(9, 3))
        dump_bbox[0] = np.dot(np.array(dump_pose['mat']),np.array(center_point) )+ np.array(dump_pose['t'])
        target_bbox[0] = np.dot(np.array(target_pose['mat']),np.array(center_point) )+ np.array(target_pose['t'])
        for vertex_index in range(len(object_bbox)):
            dump_bbox[vertex_index+1] = np.dot(np.array(dump_pose['mat']),np.array(object_bbox[vertex_index]) )+ np.array(dump_pose['t'])
            target_bbox[vertex_index+1] = np.dot(np.array(target_pose['mat']),np.array(object_bbox[vertex_index]) )+ np.array(target_pose['t'])

        box_dump_bbox = box.Box(vertices=dump_bbox)
        box_target_bbox = box.Box(vertices=target_bbox)
        loss = iou.IoU(box_dump_bbox, box_target_bbox)
        # bboxes visualization
        if debug == True:
            self.draw_boxes([box_dump_bbox.vertices, box_target_bbox.vertices],  clips=loss.intersection_points)
        iou_score = loss.iou()
        print('iou = ', iou_score)
        return iou_score

    def calculate_score(self, debug):
        score_list = []
        mesh_name_list = []
        target_pose_dict = self.get_target_object_poses()
        dump_pose_dict = self.get_dump_object_poses(self.env, self.task_index)
        # print("target",target_pose_dict)
        # print("dump",dump_pose_dict)
        log_file = open(self.log_file_path, 'a')
        log_file_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'evaluation')
        if not os.path.exists(log_file_dir):
            os.makedirs(log_file_dir)
        for mesh_name in target_pose_dict.keys():
            if len(target_pose_dict[mesh_name]) != len(dump_pose_dict[mesh_name]):
                raise ValueError('Number of instances of an object in target and dump poses don\'t match')
            mesh_max_IoU = 0.0
            max_permu_IoU_list = [0.0 for i in range(len(target_pose_dict[mesh_name]))]
            for index_list in permutations(range(len(target_pose_dict[mesh_name]))):
                permu_IoU_list = []
                print('index_list for mesh {}:{}\n{}'.format(mesh_name, index_list, '=' * 40))
                log_file.write('index_list for mesh {}:{}\n{}\n'.format(mesh_name, index_list, '=' * 40))
                for target_idx, dump_idx in zip(range(len(target_pose_dict[mesh_name])), index_list):
                    dump_pose = dump_pose_dict[mesh_name][dump_idx]
                    target_pose = target_pose_dict[mesh_name][target_idx]
                    actual_score = self.compute_IoU(dump_pose, target_pose, mesh_name ,debug)
                    log_file.write('mesh_name:{}, IoU score = {}\n'.format(
                        mesh_name,  actual_score))
                    print('mesh_name:{}, IoU score = {}'.format(
                        mesh_name, actual_score))
                    permu_IoU_list.append(actual_score)
                mean_permu_IoU = np.mean(np.array(permu_IoU_list, dtype = np.float32))
                if mean_permu_IoU >= mesh_max_IoU:
                    mesh_max_IoU = mean_permu_IoU
                    max_permu_IoU_list = permu_IoU_list
            score_list += max_permu_IoU_list
            mesh_name_list += len(target_pose_dict[mesh_name]) * [mesh_name]
        scene_score = len([1 for i in score_list if i > self.IoU_threshold]) / len(score_list)
        scene_score_time = scene_score * 3600 * len(score_list) / min(self.time_cost,T_MAX)
        print("time cost:",self.time_cost)
        log_file.write('{}\n'.format('-' * 60 + '\n' + '-' * 60))
        for mesh_name, score in zip(mesh_name_list, score_list):
            log_file.write('{}: {}\n'.format(mesh_name, score))
        log_file.write('{}\nScene Index:{}, \nScene mean score:{}, \nMean Rearrangement per hour:{}'.format(
            '-' * 60 + '\n' + '-' * 60, self.task_index, scene_score, scene_score_time))
        log_file.write('\n')
        log_file.write('\n')
        print('\033[032m\nScene mean score:{}\033[0m'.format(scene_score))
        return scene_score, scene_score_time

