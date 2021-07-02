#! /usr/bin/env python3

import rospy
from ocrtoc_msg.srv import PerceptionTarget, PerceptionTargetRequest

if __name__ == "__main__":
    rospy.wait_for_service('/perception_action_target')
    try:
        service_call = rospy.ServiceProxy('/perception_action_target',
                                          PerceptionTarget)
        request = PerceptionTargetRequest()
        request.target_object_list = ['lipton_tea'] # 0-0
        response = service_call(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
