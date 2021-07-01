#! /usr/bin/env python3
import random
import numpy as np
import open3d as o3d
from transforms3d.quaternions import mat2quat
import os
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from ocrtoc_msg.msg import PerceptionResult
from ocrtoc_msg.srv import PerceptionTarget, PerceptionTargetResponse
from ocrtoc_perception import Perceptor

current_dir = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(current_dir, '..', 'config')

def format_dict(d, depth = 0, space = 2):
    f_dict = ''
    for key in d.keys():
        if type(d[key]) == dict:
            f_dict += ' ' * depth * space + '{}:\n'.format(key)
            f_dict += format_dict(d[key], depth + 1, space)
        else:
            f_dict += ' ' * depth * space + '{}: {}\n'.format(key,d[key])
    return f_dict

class PerceptionServer(object):
    def __init__(self, service_name, config_name):
        config_file = open(config_name, 'r')
        self.config = yaml.load(config_file.read(), Loader = yaml.FullLoader)
        config_file.close()
        rospy.loginfo('\033[034mPerception server config:\n{}\033[0m'.format(format_dict(self.config)))
        self.use_camera = self.config['use_camera']
        self.perception_target_server_name = service_name
        self.perceptor = Perceptor(config = self.config)
        rospy.Service(self.perception_target_server_name,
                      PerceptionTarget,
                      self.perception_target_handler)
        rospy.loginfo(self.perception_target_server_name + " service is ready.")

    def perception_target_handler(self, request):
        response_poses = self.perceptor.get_response(
            object_list = request.target_object_list,
        )

        rospy.loginfo('*' * 50)
        rospy.loginfo("ROS service " + self.perception_target_server_name + " request:")
        rospy.loginfo(request.target_object_list)

        response_result = PerceptionTargetResponse()

        ### Response example
        for each_object in request.target_object_list:

            perception_result = PerceptionResult()
            perception_result.object_name = each_object
            if not each_object in response_poses.keys():
                perception_result.be_recognized = False
            else:
                response_pose = response_poses[each_object]
                perception_result.be_recognized = True
                perception_result.object_pose = PoseStamped()
                perception_result.object_pose.header.frame_id = "world"
                perception_result.object_pose.pose.position.x = response_pose['object_pose']['x']
                perception_result.object_pose.pose.position.y = response_pose['object_pose']['y']
                perception_result.object_pose.pose.position.z = response_pose['object_pose']['z']
                perception_result.object_pose.pose.orientation.w = response_pose['object_pose']['qw']
                perception_result.object_pose.pose.orientation.x = response_pose['object_pose']['qx']
                perception_result.object_pose.pose.orientation.y = response_pose['object_pose']['qy']
                perception_result.object_pose.pose.orientation.z = response_pose['object_pose']['qz']
                perception_result.is_graspable = response_pose['graspable']
                perception_result.grasp_pose = PoseStamped()
                if perception_result.is_graspable:
                    perception_result.grasp_pose.header.frame_id = "world"
                    perception_result.grasp_pose.pose.position.x = response_pose['grasp_pose']['x']
                    perception_result.grasp_pose.pose.position.y = response_pose['grasp_pose']['y']
                    perception_result.grasp_pose.pose.position.z = response_pose['grasp_pose']['z']
                    perception_result.grasp_pose.pose.orientation.w = response_pose['grasp_pose']['qw']
                    perception_result.grasp_pose.pose.orientation.x = response_pose['grasp_pose']['qx']
                    perception_result.grasp_pose.pose.orientation.y = response_pose['grasp_pose']['qy']
                    perception_result.grasp_pose.pose.orientation.z = response_pose['grasp_pose']['qz']
            response_result.perception_result_list.append(perception_result)
        ### Response example end

        rospy.loginfo("ROS service " + self.perception_target_server_name + " response:")
        rospy.loginfo(response_result)
        rospy.loginfo('-' * 50)

        return response_result