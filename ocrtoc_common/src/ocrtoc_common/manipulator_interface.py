#!/usr/bin/env python

import sys

import rospy
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg

from ocrtoc_common.srv import *

class ManipulatorInterface(object):
    def __init__(self, group_name):
        rospy.loginfo(sys.argv)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        rospy.Service("get_manipulator_info", ManipulatorInfo,
                      self.get_manipulator_info_handler)
        rospy.loginfo("get_manipulator_info service is ready.")
        rospy.Service("get_manipulator_state", ManipulatorState,
                      self.get_manipulator_state_handler)
        rospy.loginfo("get_manipulator_state service is ready.")
        rospy.Service("send_joint_space_goal", JointSpaceGoal,
                      self.send_joint_space_goal_handler)
        rospy.loginfo("send_joint_space_goal service is ready.")
        rospy.Service("send_pose_goal", PoseGoal,
                      self.send_pose_goal_handler)
        rospy.loginfo("send_pose_goal service is ready.")

        # Get basic information.
        self.group_names = self.robot.get_group_names()
        self.end_effector_link = self.move_group.get_end_effector_link()
        self.planning_frame = self.move_group.get_planning_frame()

    def print_basic_info(self):
        rospy.loginfo("======= Manipulator information =======")
        rospy.loginfo("group_names: %s", self.group_names)
        rospy.loginfo("end_effector_link: %s", self.end_effector_link)
        rospy.loginfo("planning_frame: %s", self.planning_frame)
        rospy.loginfo("=======================================")

    def get_manipulator_info_handler(self, request):
        self.print_basic_info()
        response_result = ManipulatorInfoResponse()
        response_result.group_names = self.group_names
        response_result.end_effector_link = self.end_effector_link
        response_result.planning_frame = self.planning_frame
        return response_result

    def get_manipulator_state_handler(self, request):
        response_result = ManipulatorStateResponse()
        response_result.current_pose = self.move_group.get_current_pose()
        response_result.joint_position_list = \
            self.move_group.get_current_joint_values()
        return response_result

    def send_joint_space_goal_handler(self, request):
        rospy.loginfo("Get a joint space goal:")
        rospy.loginfo(request)

        response_result = JointSpaceGoalResponse()
        response_result.successed = \
            self.move_group.go(request.joint_goal, wait=True)
        #self.move_group.set_joint_value_target(request.joint_goal)
        #traj = self.move_group.plan()
        #self.move_group.execute(traj)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        rospy.loginfo("Done.")
        return response_result

    def send_pose_goal_handler(self, request):
        rospy.loginfo("Get a position goal:")
        rospy.loginfo(request)
        self.move_group.clear_pose_targets()

        response_result = PoseGoalResponse()
        #response_result.successed = \
        #    self.move_group.go(request.goal, wait=True)

        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(request.goal)
        traj = self.move_group.plan()
        traj_len = len(traj.joint_trajectory.points)
        if traj_len < 1:
            rospy.logerr("No valid trajectory.")
            response_result.successed = False
            response_result.text = "No valid trajectory."
        else:
            rospy.loginfo("Find a trajectory with %d points.", traj_len)
            execute_result = self.move_group.execute(traj)

            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            self.move_group.clear_pose_targets()

            if execute_result is True:
                rospy.loginfo("Done.")
                response_result.successed = True
                response_result.text = "Done."
            else:
                rospy.logerr("Execute failed.")
                response_result.successed = False
                response_result.text = "Execute failed."
        return response_result
