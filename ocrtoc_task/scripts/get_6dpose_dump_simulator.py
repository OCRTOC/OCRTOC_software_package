#! /usr/bin/env python3
# Author: Minghao Gou

import copy
import os
import sys
import yaml

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import PoseStamped
import rospkg
import rospy
import tf

class PoseDumper():
    def __init__(self, task_index):
        self.task_index = task_index

    def dump(self):
        rospy.wait_for_service('/get_model_state')
        get_model_state = rospy.ServiceProxy('/get_model_state', GetModelState)
        query_request = GetModelStateRequest()

        rospack = rospkg.RosPack()
        file_name = self.task_index + '.yaml'
        yaml_path = os.path.join(rospack.get_path('ocrtoc_materials'),
                                                'targets', file_name)
        print("Load yaml file: ", yaml_path)
        with open(yaml_path, "r") as f:
            object_list = yaml.load(f)
            output_yaml = {}
            for key in sorted(object_list):
                output_key = key
                output_pose = []
                index = 1
                for model in object_list[key]:
                    alias = key + '_v' + str(index)
                    query_request.model_name = alias
                    pose_6d = get_model_state(query_request).pose
                    q = [pose_6d.orientation.x, pose_6d.orientation.y,
                        pose_6d.orientation.z, pose_6d.orientation.w]
                    euler = tf.transformations.euler_from_quaternion(q)
                    xyzrpy = [pose_6d.position.x, pose_6d.position.y,
                            pose_6d.position.z, euler[0], euler[1], euler[2]]
                    for i in range(len(xyzrpy)):
                        xyzrpy[i] = round(xyzrpy[i], 4)
                    output_pose.append(xyzrpy)
                    index += 1
                output_yaml[output_key] = output_pose
            print(output_yaml)
        output_yaml_dir = os.path.join(rospack.get_path('ocrtoc_task'), 'evaluation')
        if not os.path.exists(output_yaml_dir):
            os.makedirs(output_yaml_dir)
        output_yaml_path = os.path.join(output_yaml_dir, file_name)
        write_f = open(output_yaml_path,'w')
        yaml.dump(output_yaml, write_f)

if __name__ == '__main__':
    rospy.init_node('get_6d_pose')
    try:
        task_index = rospy.get_param('~task_index')
        rospy.loginfo("Task index: " + task_index)
    except:
        print("Usage:")
        print("roslaunch ocrtoc_task dump_6dpose_simulator.launch task_index:=0-0")
        sys.exit()
    
    pd = PoseDumper(task_index=task_index)
    pd.dump()

