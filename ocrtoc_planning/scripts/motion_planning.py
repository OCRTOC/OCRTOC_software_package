# license
import copy
from math import pi
import numpy as np
import os
import sys
import yaml

import actionlib
import control_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Transform
import moveit_commander
import moveit_msgs.msg
import rospkg
import rospy

from ocrtoc_common.transform_interface import TransformInterface

class MotionPlanner(object):
    def __init__(self):
        # load parameters
        rospack = rospkg.RosPack()
        motion_config_path = os.path.join(rospack.get_path('ocrtoc_planning'), 'config/motion_planner_parameter.yaml')
        with open(motion_config_path, "r") as f:
            config_parameters = yaml.load(f)
            self._max_try = config_parameters["max_try"]
            self._up_distance = config_parameters["up_distance"]
            self._exit_distance = config_parameters["exit_distance"]
            self._entrance_distance = config_parameters["entrance_distance"]
            self._up1 = config_parameters["up1"]
            self._up2 = config_parameters["up2"]
            self._max_attempts = config_parameters["max_attempts"]
            self._plan_step_length = config_parameters["plan_step_length"]

            self._via_pose = Pose()
            self._via_pose.position.x = config_parameters["via_pose1_px"]
            self._via_pose.position.y = config_parameters["via_pose1_py"]
            self._via_pose.position.z = config_parameters["via_pose1_pz"]
            self._via_pose.orientation.x = config_parameters["via_pose_ox"]
            self._via_pose.orientation.y = config_parameters["via_pose_oy"]
            self._via_pose.orientation.z = config_parameters["via_pose_oz"]
            self._via_pose.orientation.w = config_parameters["via_pose_ow"]

            self._via_pose2 = Pose()
            self._via_pose2.position.x = config_parameters["via_pose2_px"]
            self._via_pose2.position.y = config_parameters["via_pose2_py"]
            self._via_pose2.position.z = config_parameters["via_pose2_pz"]
            self._via_pose2.orientation.x = config_parameters["via_pose_ox"]
            self._via_pose2.orientation.y = config_parameters["via_pose_oy"]
            self._via_pose2.orientation.z = config_parameters["via_pose_oz"]
            self._via_pose2.orientation.w = config_parameters["via_pose_ow"]

            self._via_pose3 = Pose()
            self._via_pose3.position.x = config_parameters["via_pose3_px"]
            self._via_pose3.position.y = config_parameters["via_pose3_pz"]
            self._via_pose3.position.z = config_parameters["via_pose3_pz"]
            self._via_pose3.orientation.x = config_parameters["via_pose_ox"]
            self._via_pose3.orientation.y = config_parameters["via_pose_oy"]
            self._via_pose3.orientation.z = config_parameters["via_pose_oz"]
            self._via_pose3.orientation.w = config_parameters["via_pose_ow"]

            self._home_joints = [config_parameters["home_joint1"] * pi / 180,
                                 config_parameters["home_joint2"] * pi / 180,
                                 config_parameters["home_joint3"] * pi / 180,
                                 config_parameters["home_joint4"] * pi / 180,
                                 config_parameters["home_joint5"] * pi / 180,
                                 config_parameters["home_joint6"] * pi / 180,
                                 config_parameters["home_joint7"] * pi / 180]

            self._group_name = config_parameters["group_name"]

        moveit_commander.roscpp_initialize(sys.argv)
        self._move_group = moveit_commander.MoveGroupCommander(self._group_name, wait_for_servers = 30.0)
        self._move_group.allow_replanning(True)
        self._move_group.set_start_state_to_current_state()
        self._end_effector = self._move_group.get_end_effector_link()
        self._at_home_pose = False
        self._gripper_client = actionlib.SimpleActionClient('/franka_gripper/gripper_action', control_msgs.msg.GripperCommandAction)
        self._transformer = TransformInterface()
        entrance_transformation = Transform()
        entrance_transformation.translation.x = 0
        entrance_transformation.translation.y = 0
        entrance_transformation.translation.z = self._entrance_distance
        entrance_transformation.rotation.x = 0
        entrance_transformation.rotation.y = 0
        entrance_transformation.rotation.z = 0
        entrance_transformation.rotation.w = 1
        self._entrance_transformation_matrix = self._transformer.ros_transform_to_matrix4x4(entrance_transformation)

        ee_transform = self._transformer.lookup_ros_transform("panda_ee_link", "panda_link8")
        self._ee_transform_matrix = self._transformer.ros_transform_to_matrix4x4(ee_transform.transform)

        self.to_home_pose()
        self.place()
        rospy.sleep(1.0)

    # move to specified home pose
    def to_home_pose(self):
        self._move_group.set_joint_value_target(self._home_joints)
        to_home_result = self._move_group.go()
        rospy.sleep(1.0)
        print('to home pose result:{}'.format(to_home_result))
        return to_home_result

    # generate path in joint space
    def move_joint_space(self, pose_goal):
        self._move_group.set_pose_target(pose_goal)
        plan = self._move_group.go(wait=True)
        self._move_group.stop()
        self._move_group.clear_pose_targets()
        rospy.sleep(1.0)

    # move robot to home pose, then move from home pose to target pose
    def move_from_home_pose(self, pose_goal):
        from_home_result = True
        to_home_result = self.to_home_pose()

        if to_home_result:
            points_to_target = self.get_points_to_target(pose_goal)
            fraction = 0
            attempts = 0
            while fraction < 1.0 and attempts < self._max_attempts:
                (plan, fraction) = self._move_group.compute_cartesian_path(
                    points_to_target,                # way points
                    self._plan_step_length,          # step length
                    0.0,                             # disable jump
                    True                             # enable avoid_collision
                    )
                attempts += 1

            if fraction == 1.0:
                self._move_group.execute(plan)
                from_home_result = True
            else:
                from_home_result = False
        else:
            from_home_result = False

        rospy.loginfo('move from home pose result' + str(from_home_result))
        return from_home_result

    # generate cartesian straight path
    def move_cartesian_space_upright(self, pose_goal, via_up = False):
        # get a list of way points to target pose, including entrance pose, transformation needed

        # transform panda_ee_link goal to panda_link8 goal.
        group_goal = self.ee_goal_to_link8_goal(pose_goal)

        points_to_target = self.get_points_to_target_upright(group_goal)
        for point in points_to_target:
            fraction = 0
            attempts = 0
            waypoints = []
            waypoints.append(copy.deepcopy(point))
            while fraction < 1.0 and attempts < self._max_attempts:
                (plan, fraction) = self._move_group.compute_cartesian_path(
                    waypoints,                # way points
                    self._plan_step_length,          # step length
                    0.0,                             # disable jump
                    True                             # enable avoid_collision
                    )
                attempts += 1

            if fraction == 1.0:
                rospy.loginfo('Path computed successfully, moving robot')
                self._move_group.execute(plan)
                self._move_group.stop()
                self._move_group.clear_pose_targets()
                rospy.loginfo('Path execution completed')
                move_result = True
            else:
                rospy.loginfo('Action failed')
                move_result = False
                break
        else:
            move_result = True
            rospy.loginfo('Action success')

        rospy.loginfo('(upright path) Action finished, action result' + str(move_result))
        return move_result

    # generate cartesian straight path
    def move_cartesian_space(self, pose_goal, via_up=False):
        points_to_target = self.get_points_to_target(pose_goal, via_up)  # get a list of way points to target pose, including entrance pose, transformation needed
        fraction = 0
        attempts = 0
        while fraction < 1.0 and attempts < self._max_attempts:
            (plan, fraction) = self._move_group.compute_cartesian_path(
                points_to_target,                # way points
                self._plan_step_length,          # step length
                0.0,                             # disable jump
                True                             # enable avoid_collision
                )
            attempts += 1

        if fraction == 1.0:
            rospy.loginfo('Path computed successfully, moving robot')
            self._move_group.execute(plan)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            rospy.loginfo('Path execution completed')
            move_result = True
        else:
            rospy.loginfo('Fisrt path planning failed, try to planing 2nd path')
            move_result = self.move_cartesian_space2(pose_goal, via_up)
            if move_result:
                pass
            else:
                move_result = self.move_cartesian_space3(pose_goal, via_up)

        rospy.loginfo('move finished, move result' + str(move_result))
        return move_result

    # generate cartesian straight path2
    def move_cartesian_space2(self, pose_goal, via_up=False):
        points_to_target = self.get_points_to_target2(pose_goal, via_up)  # get a list of way points to target pose, including entrance pose, transformation needed
        fraction = 0
        attempts = 0
        while fraction < 1.0 and attempts < self._max_attempts:
            (plan, fraction) = self._move_group.compute_cartesian_path(
                points_to_target,                # way points
                self._plan_step_length,          # step length
                0.0,                             # disable jump
                True                             # enable avoid_collision
                )
            attempts += 1

        if fraction == 1.0:
            rospy.loginfo('Path2 computed successfully, moving robot')
            self._move_group.execute(plan)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            rospy.loginfo('Path2 execution completed')
            move_result = True
        else:
            rospy.loginfo('2nd path planning failed, try to planing the 3rd path')
            move_result = False
        return move_result

    # generate cartesian straight path3
    def move_cartesian_space3(self, pose_goal, via_up=False):
        points_to_target = self.get_points_to_target3(pose_goal, via_up)  # get a list of way points to target pose, including entrance pose, transformation needed
        fraction = 0
        attempts = 0
        while fraction < 1.0 and attempts < self._max_attempts:
            (plan, fraction) = self._move_group.compute_cartesian_path(
                points_to_target,                # way points
                self._plan_step_length,          # step length
                0.0,                             # disable jump
                True                             # enable avoid_collision
                )
            attempts += 1

        if fraction == 1.0:
            rospy.loginfo('Path3 computed successfully, moving robot')
            self._move_group.execute(plan)
            self._move_group.stop()
            self._move_group.clear_pose_targets()
            rospy.loginfo('Path3 execution completed')
            move_result = True
        else:
            rospy.loginfo('All path tried, but fail to find an available path!')
            rospy.loginfo('All path tried, but fail to find an available path!')
            rospy.loginfo('All path tried, but fail to find an available path!')
            move_result = False
        return move_result

    # generate cartesian straight path
    def move_cartesian_space_discrete(self, pose_goal, via_up=False):
        points_to_target = self.get_points_to_target(pose_goal, via_up)  # get a list of way points to target pose, including entrance pose, transformation needed
        for point in points_to_target:
            fraction = 0
            attempts = 0
            waypoints = []
            waypoints.append(copy.deepcopy(point))
            while fraction < 1.0 and attempts < self._max_attempts:
                (plan, fraction) = self._move_group.compute_cartesian_path(
                    waypoints,                # way points
                    self._plan_step_length,          # step length
                    0.0,                             # disable jump
                    True                             # enable avoid_collision
                    )
                attempts += 1

            if fraction == 1.0:
                rospy.loginfo('Path computed successfully, moving robot')
                self._move_group.execute(plan)
                self._move_group.stop()
                self._move_group.clear_pose_targets()
                rospy.loginfo('Path execution completed')
                move_result = True
            else:
                rospy.loginfo('Action failed')
                move_result = False
                break
        else:
            move_result = True
            rospy.loginfo('Action success')

        rospy.loginfo('(discrete path) Action finished, action result' + str(move_result))
        return move_result

    # generate cartesian straight path2
    def move_cartesian_space2_discrete(self, pose_goal, via_up=False):
        points_to_target = self.get_points_to_target2(pose_goal, via_up)  # get a list of way points to target pose, including entrance pose, transformation needed
        for point in points_to_target:
            fraction = 0
            attempts = 0
            waypoints = []
            waypoints.append(copy.deepcopy(point))
            while fraction < 1.0 and attempts < self._max_attempts:
                (plan, fraction) = self._move_group.compute_cartesian_path(
                    waypoints,                # way points
                    self._plan_step_length,          # step length
                    0.0,                             # disable jump
                    True                             # enable avoid_collision
                    )
                attempts += 1

            if fraction == 1.0:
                rospy.loginfo('Path computed successfully, moving robot')
                self._move_group.execute(plan)
                self._move_group.stop()
                self._move_group.clear_pose_targets()
                rospy.loginfo('Path execution completed')
                move_result = True
            else:
                rospy.loginfo('Action failed')
                move_result = False
                break
        else:
            move_result = True
            rospy.loginfo('Action success')

        rospy.loginfo('Action finished, action result' + str(move_result))
        return move_result

    # generate cartesian straight path3
    def move_cartesian_space3_discrete(self, pose_goal, via_up=False):
        points_to_target = self.get_points_to_target3(pose_goal, via_up)  # get a list of way points to target pose, including entrance pose, transformation needed
        for point in points_to_target:
            fraction = 0
            attempts = 0
            waypoints = []
            waypoints.append(copy.deepcopy(point))
            while fraction < 1.0 and attempts < self._max_attempts:
                (plan, fraction) = self._move_group.compute_cartesian_path(
                    waypoints,                # way points
                    self._plan_step_length,          # step length
                    0.0,                             # disable jump
                    True                             # enable avoid_collision
                    )
                attempts += 1

            if fraction == 1.0:
                rospy.loginfo('Path computed successfully, moving robot')
                self._move_group.execute(plan)
                self._move_group.stop()
                self._move_group.clear_pose_targets()
                rospy.loginfo('Path execution completed')
                move_result = True
            else:
                rospy.loginfo('Action failed')
                move_result = False
                break
        else:
            move_result = True
            rospy.loginfo('Action success')

        rospy.loginfo('Action finished, action result' + str(move_result))
        return move_result

    # pick action
    def pick(self):
        self._gripper_client.wait_for_server()
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = 0.0
        goal.command.max_effort = 30

        self._gripper_client.send_goal(goal)
        rospy.sleep(2.0)

    # place action
    def place(self):
        self._gripper_client.wait_for_server()
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = 0.039
        goal.command.max_effort = 30

        self._gripper_client.send_goal(goal)
        rospy.sleep(2.0)

    # get a list of via points from current position to target position (add exit and entrance point to pick and place position)
    def get_points_to_target_upright(self, target_pose):
        points_to_target = []
        current_pose = self._move_group.get_current_pose(self._end_effector).pose
        exit_pose = copy.deepcopy(current_pose)
        exit_pose.position.z += self._up1
        points_to_target.append(copy.deepcopy(exit_pose))

        enter_pose = copy.deepcopy(target_pose)
        enter_pose.position.z += self._up2

        points_to_target.append(copy.deepcopy(enter_pose))
        points_to_target.append(copy.deepcopy(target_pose))
        return points_to_target

    # get a list of via points from current position to target position (add exit and entrance point to pick and place position)
    def get_points_to_target(self, target_pose, via_up = False):
        points_to_target = []
        current_pose = self._move_group.get_current_pose(self._end_effector).pose
        current_pose_matrix = self._transformer.ros_pose_to_matrix4x4(current_pose)
        exit_pose_matrix = current_pose_matrix.dot(self._entrance_transformation_matrix)
        exit_pose = self._transformer.matrix4x4_to_ros_pose(exit_pose_matrix)
        points_to_target.append(copy.deepcopy(exit_pose))

        points_to_target.append(copy.deepcopy(self._via_pose))

        target_pose_matrix = self._transformer.ros_pose_to_matrix4x4(target_pose)
        enter_pose_matrix = target_pose_matrix.dot(self._entrance_transformation_matrix)
        enter_pose = self._transformer.matrix4x4_to_ros_pose(enter_pose_matrix)

        if via_up:
            up_pose = Pose()
            up_pose.position.x = enter_pose.position.x
            up_pose.position.y = enter_pose.position.y
            up_pose.position.z = enter_pose.position.z + self._up_distance
            up_pose.orientation.x = enter_pose.orientation.x
            up_pose.orientation.y = enter_pose.orientation.y
            up_pose.orientation.z = enter_pose.orientation.z
            up_pose.orientation.w = enter_pose.orientation.w
            points_to_target.append(copy.deepcopy(up_pose))
        else:
            pass

        points_to_target.append(copy.deepcopy(enter_pose))
        points_to_target.append(copy.deepcopy(target_pose))
        return points_to_target

    # get a list of via points from current position to target position (via_pose2)
    def get_points_to_target2(self, target_pose, via_up=False):
        points_to_target = []
        exit_pose = self._move_group.get_current_pose(self._end_effector).pose
        exit_pose.position.z = exit_pose.position.z + self._exit_distance
        points_to_target.append(copy.deepcopy(exit_pose))

        points_to_target.append(copy.deepcopy(self._via_pose2))

        target_pose_matrix = self._transformer.ros_pose_to_matrix4x4(target_pose)
        enter_pose_matrix = target_pose_matrix.dot(self._entrance_transformation_matrix)
        enter_pose = self._transformer.matrix4x4_to_ros_pose(enter_pose_matrix)

        if via_up:
            up_pose = Pose()
            up_pose.position.x = enter_pose.position.x
            up_pose.position.y = enter_pose.position.y
            up_pose.position.z = enter_pose.position.z + self._up_distance
            up_pose.orientation.x = enter_pose.orientation.x
            up_pose.orientation.y = enter_pose.orientation.y
            up_pose.orientation.z = enter_pose.orientation.z
            up_pose.orientation.w = enter_pose.orientation.w
            points_to_target.append(copy.deepcopy(up_pose))
        else:
            pass

        points_to_target.append(copy.deepcopy(enter_pose))
        points_to_target.append(copy.deepcopy(target_pose))
        return points_to_target

    # get a list of via points from current position to target position (via_pose3)
    def get_points_to_target3(self, target_pose, via_up=False):
        points_to_target = []
        exit_pose = self._move_group.get_current_pose(self._end_effector).pose
        exit_pose.position.z = exit_pose.position.z + self._exit_distance
        points_to_target.append(copy.deepcopy(exit_pose))

        points_to_target.append(copy.deepcopy(self._via_pose3))

        target_pose_matrix = self._transformer.ros_pose_to_matrix4x4(target_pose)
        enter_pose_matrix = target_pose_matrix.dot(self._entrance_transformation_matrix)
        enter_pose = self._transformer.matrix4x4_to_ros_pose(enter_pose_matrix)

        if via_up:
            up_pose = Pose()
            up_pose.position.x = enter_pose.position.x
            up_pose.position.y = enter_pose.position.y
            up_pose.position.z = enter_pose.position.z + self._up_distance
            up_pose.orientation.x = enter_pose.orientation.x
            up_pose.orientation.y = enter_pose.orientation.y
            up_pose.orientation.z = enter_pose.orientation.z
            up_pose.orientation.w = enter_pose.orientation.w
            points_to_target.append(copy.deepcopy(up_pose))
        else:
            pass

        points_to_target.append(copy.deepcopy(enter_pose))
        points_to_target.append(copy.deepcopy(target_pose))
        return points_to_target

    def ee_goal_to_link8_goal(self, ee_goal):
        ee_goal_matrix = self._transformer.ros_pose_to_matrix4x4(ee_goal)
        link8_goal_matrix = ee_goal_matrix.dot(self._ee_transform_matrix)
        link8_goal = self._transformer.matrix4x4_to_ros_pose(link8_goal_matrix)
        return link8_goal

# motion planning test function
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sim_moveit_execution', anonymous=True)

    cartesian = rospy.get_param('~cartesian', True)
    print('move strait in cartesian space: ', cartesian)

    robot = moveit_commander.RobotCommander()

    group_names = robot.get_group_names()
    print('Available planning groups: ', group_names)  # 'manipulator' / 'endeffector'

    group_name = 'manipulator'
    move_group = moveit_commander.MoveGroupCommander(group_name)
    end_effector_link = move_group.get_end_effector_link()
    print('end-effector link: ', end_effector_link)

    move_group.allow_replanning(True)

    move_group.set_goal_position_tolerance(0.001)  # unit: meter
    move_group.set_goal_orientation_tolerance(0.001)  # unit: rad

    move_group.set_max_acceleration_scaling_factor(0.5)
    move_group.set_max_velocity_scaling_factor(0.5)

    # move_group.set_start_state_to_current_state()

    waypoints = []
    current_pose = move_group.get_current_pose(end_effector_link).pose
    print('current pose: ', current_pose)


    wpose = copy.deepcopy(current_pose)
    wpose.position.z -= 0.15
    if cartesian:
        waypoints.append(copy.deepcopy(wpose))
    else:
        move_group.set_pose_target(wpose)
        move_group.go()
        rospy.sleep(1.0)

    wpose.position.y += 0.15
    if cartesian:
        waypoints.append(copy.deepcopy(wpose))
    else:
        move_group.set_pose_target(wpose)
        move_group.go()
        rospy.sleep(1.0)

    wpose.position.x += 0.1
    if cartesian:
        waypoints.append(copy.deepcopy(wpose))
    else:
        move_group.set_pose_target(wpose)
        move_group.go()
        rospy.sleep(1.0)

    # planning
    if cartesian:
        fraction = 0.0  # path planning cover rate
        max_attemps = 100   # maximum try times
        attempts = 0     # already try times


    # plan a cartesian path that pass all waypoints
    while attempts < max_attemps and fraction < 1:
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints list
            0.01,       # end-effector step, computed inverse kinematics every 0.01m
            0.0,        # jump_threshold, 0 means jump not allowed
            True        # avoid_collision
            )
        attempts += 1
        # print progress
        if attempts % 20 == 0:
            rospy.loginfo("Still trying after" + str(attempts) + "attempts......")

    # if planning succeed, fraction == 1, move robot
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully, moving arm")
        move_group.execute(plan)
        rospy.loginfo("Path execution complete")
    else:
        rospy.loginfo("Path planning failed with only" + str(fraction) + "success after" + str(attempts) + "attempts")

    rospy.sleep(1.0)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    rospy.init_node('test')
    # first test
    # main()

    # second test
    planner = MotionPlanner()
    end_effector_link = planner._move_group.get_end_effector_link()
    print('manipulator end-effector link name: {}'.format(end_effector_link))
    # pose_goal = []
    # pose_target = Pose()
    # pose_target.position.x = 0.1
    # pose_target.position.y = 0.5
    # pose_target.position.z = 0.3
    # pose_target.orientation.x = 1
    # pose_target.orientation.y = 0
    # pose_target.orientation.z = 0
    # pose_target.orientation.w = 0
    # pose_goal.append(copy.deepcopy(pose_target))

    # # pose_target.position.y += 0.15
    # # pose_goal.append(copy.deepcopy(pose_target))

    # # pose_target.position.x += 0.15
    # # pose_goal.append(copy.deepcopy(pose_target))

    # planner.move_cartesian_space(pose_goal)
