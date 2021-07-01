#! /usr/bin/env python

import rospy
from ocrtoc_common.srv import *

if __name__ == "__main__":
    print('='*80)
    rospy.wait_for_service('/get_manipulator_info')
    try:
        service_call = rospy.ServiceProxy('/get_manipulator_info', ManipulatorInfo)
        request = ManipulatorInfoRequest()
        response = service_call(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print()
    print()

    print('='*80)
    rospy.wait_for_service('/get_manipulator_state')
    try:
        service_call = rospy.ServiceProxy('/get_manipulator_state', ManipulatorState)
        request = ManipulatorStateRequest()
        response = service_call(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print()
    print()

    rospy.sleep(2.0)
    request = JointSpaceGoalRequest()
    request.joint_goal = [1.48, -0.22 , -0.19, -2.41, -0.645, 1.51, 0.08]
    print('='*80)
    print(response.joint_position_list)
    print(request.joint_goal)
    rospy.wait_for_service('/send_joint_space_goal')
    try:
        service_call = rospy.ServiceProxy('/send_joint_space_goal', JointSpaceGoal)
        response = service_call(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print()
    print()

    rospy.sleep(5.0)
    request = PoseGoalRequest()
    # Goal in 'world' frame
    request.goal.position.x = -0.343232254619
    request.goal.position.y = -0.525294298568
    request.goal.position.z = 0.488844847508
    request.goal.orientation.x = -0.332872961262
    request.goal.orientation.y = 0.623855146947
    request.goal.orientation.z = 0.334828632737
    request.goal.orientation.w = 0.622808264226
    print('='*80)
    print(request.goal)
    rospy.wait_for_service('/send_pose_goal')
    try:
        service_call = rospy.ServiceProxy('/send_pose_goal', PoseGoal)
        response = service_call(request)
        print(response)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print()
    print()
