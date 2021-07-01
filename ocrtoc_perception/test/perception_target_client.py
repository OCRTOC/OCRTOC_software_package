#! /usr/bin/env python3

import rospy
from ocrtoc_msg.srv import PerceptionTarget, PerceptionTargetRequest

if __name__ == "__main__":
    rospy.wait_for_service('/perception_action_target')
    try:
        service_call = rospy.ServiceProxy('/perception_action_target',
                                          PerceptionTarget)
        request = PerceptionTargetRequest()
        request.target_object_list = ['lipton_tea',
                                      'orion_pie',
                                      'plastic_banana',
                                      'suger_1',
                                      'potato_chip_1'] # 1-1
        response = service_call(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
