#! /usr/bin/env python

import copy

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import PoseStamped

from ocrtoc_msg.msg import PerceptionResult
from ocrtoc_msg.srv import PerceptionTarget, PerceptionTargetResponse

def perception_target_handler(request):
    rospy.loginfo('=' * 50)
    rospy.loginfo(request.target_object_list)
    response_result = PerceptionTargetResponse()

    rospy.wait_for_service('/get_model_state')
    get_model_state = rospy.ServiceProxy('/get_model_state',
                                         GetModelState)
    sapien_request = GetModelStateRequest()

    for each_object in request.target_object_list:
        perception_result = PerceptionResult()
        perception_result.object_name = each_object

        sapien_request.model_name = each_object
        sapien_response = get_model_state(sapien_request)

        perception_result.be_recognized = True
        perception_result.object_pose = PoseStamped()
        perception_result.object_pose.header.frame_id = "world"
        perception_result.object_pose.pose = sapien_response.pose
        perception_result.is_graspable = True
        perception_result.grasp_pose = copy.deepcopy(perception_result.object_pose)
        perception_result.grasp_pose.pose.position.z += 0.1 # meter
        response_result.perception_result_list.append(perception_result)

    rospy.loginfo('-' * 50)
    rospy.loginfo(response_result)
    rospy.loginfo('=' * 50)

    return response_result

if __name__ == '__main__':
    rospy.init_node('fake_perception')
    rospy.Service("fake_perception_action_target",
                  PerceptionTarget,
                  perception_target_handler)
    rospy.loginfo("fake_perception_action_target service is ready.")
    rospy.spin()
