import os
import yaml
import numpy as np
import mujoco
T_MAX = 120 #MAX time allowed for a task

class ScoreCalculator():
    def __init__(self, env, time_cost = 120):
        self.env = env
        self.time_cost = time_cost
        self.log_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'evaluation',
            '{}_score.txt'.format("language_conditioned_tasks"))
        self.target_description = self.load_target_description()

    # load task target description 
    def load_target_description(self):
        file_name = 'language_conditioned_target.yaml'
        yaml_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),'ocrtoc_materials_mujoco',
                                                'targets', file_name)
        print("Load yaml file: {}".format(yaml_path))
        with open(yaml_path, "r") as f:
            target_description = yaml.safe_load(f)
        return target_description['Prompts']

    # get objects pose from simulation
    def get_object_pose(self,env,object_list):
        object_id_dict = dict()
        object_pos_dict = dict()
        box_pos_dict = dict()
        for item in object_list:
            object_id_dict[item] = []
            object_pos_dict[item] = []
            box_pos_dict[item] = []
        nbody = env.unwrapped.model.nbody
        for name_id in range(nbody):
            name = mujoco.mj_id2name(env.unwrapped.model, mujoco.mjtObj.mjOBJ_BODY, name_id)
            for object in object_list:
                if name.startswith(object):
                    object_id_dict[object].append(name_id)
                    object_pos_dict[object].append(np.array(env.unwrapped.data.body(name_id).xpos[0:2]))
                if name.startswith('box_'+object):
                    box_pos_dict[object].append(np.array(env.unwrapped.data.body(name_id).xpos[0:2]))
        return object_pos_dict, box_pos_dict
    
    # compute score and write into log file
    def calculate_score(self):
        object_list = []
        succeed_object_rate_list = []
        total_object_number = 0
        succeed_object_number = 0
        for item in self.target_description:
            object_list.append(item.split(" ")[1])
        object_pos_dict, box_pos_dict = self.get_object_pose(self.env,object_list)
        for object_name in object_pos_dict:
            succeed_object_number_each = 0
            total_object_number_each = 0
            for object_pos in object_pos_dict[object_name]:
                total_object_number += 1
                total_object_number_each += 1
                if (box_pos_dict[object_name][0][0] - 0.15 <= object_pos[0] and object_pos[0] <= box_pos_dict[object_name][0][0]  + 0.15):
                    if (box_pos_dict[object_name][0][1] - 0.18<= object_pos[1] and object_pos[1] <= box_pos_dict[object_name][0][1]+0.18):
                        succeed_object_number += 1
                        succeed_object_number_each += 1
            succeed_object_rate_list.append(succeed_object_number_each/total_object_number_each)
        log_file = open(self.log_file_path, 'a')
        log_file_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'evaluation')
        if not os.path.exists(log_file_dir):
            os.makedirs(log_file_dir)
        scene_score = succeed_object_number / total_object_number
        print("time cost:",self.time_cost)
        log_file.write('{}\n'.format('-' * 60 + '\n' + '-' * 60))

        log_file.write('{}\nScene:Language conditioned task with objects {},\nSucceed rate of each catergory is:{}, \nScene mean score:{}, \nTime Cost:{}'.format(
            '-' * 60 + '\n' + '-' * 60,object_list,succeed_object_rate_list, scene_score, self.time_cost))
        log_file.write('\n')
        log_file.write('\n')
        print('\033[032m\nTotal numbers of object:{}\033[0m'.format(total_object_number))
        print('\033[032m\nNumbers of succeed grasped object:{}\033[0m'.format(succeed_object_number))
        print('\033[032m\nScene mean score:{}\033[0m'.format(scene_score))
        return scene_score

