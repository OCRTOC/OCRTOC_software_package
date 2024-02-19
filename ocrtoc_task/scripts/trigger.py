#! /usr/bin/env python
import os
import numpy as np
import sys
import yaml
from threading import Thread

import actionlib
import rospy
import rospkg
import tf

from grasp_detection import Grasp_Detector
from get_6dpose_dump_simulator import PoseDumper
from calculate_score_from_dump_pose_new import ScoreCalculator

from geometry_msgs.msg import Pose, PoseArray
import ocrtoc_msg.msg



if __name__ == '__main__':
    rospy.init_node('task_manager')

    task_manager = actionlib.SimpleActionClient(
        '/solution_server', ocrtoc_msg.msg.OrganizationAction)

    # get scenes directory
    try:
        task_file = rospy.get_param('~task_file',os.path.join(
            rospkg.RosPack().get_path('ocrtoc_materials'),
            'targets/1-1.yaml'
        ))
        task_index = rospy.get_param('~task_index')
        log_file = rospy.get_param('~log_file')
        task_name = os.path.basename(task_file).split('.')[0]
        evaluation = rospy.get_param('~evaluation')
        if not type(evaluation) == bool:
            rospy.logerr("evalution must be True of False")
        rospy.loginfo("Task file: " + task_file)
    except:
        print("Usage:")
        print("rosrun ocrtoc_task trigger.py task_file:=/root/ocrtoc_ws/src/ocrtoc_materials/targets/1-1.yaml")
        sys.exit()

    task_manager.wait_for_server()
    if evaluation:
        gd = Grasp_Detector(task_index = task_index, log_file = log_file)
        gd_thread = Thread(target = gd.monitor_main_loop)
        gd_thread.setDaemon(True)
        gd_thread.start()
    # 1.Creates a actionlib goal from scene task configuration.
    goal = ocrtoc_msg.msg.OrganizationGoal()
    goal.scene_id = task_name
    goal.frame_id = 'world'
    target_dict = yaml.load(open(task_file, 'r').read())
    for each_object in target_dict:
        goal.object_list.append(each_object)
        object_pose = PoseArray()

        for pose in target_dict[each_object]:
            one_pose = Pose()
            print('='*80)
            print(pose)
            one_pose.position.x = pose[0]
            one_pose.position.y = pose[1]
            one_pose.position.z = pose[2]
            quaternion = tf.transformations.quaternion_from_euler(
                pose[3], pose[4], pose[5])
            one_pose.orientation.x = quaternion[0]
            one_pose.orientation.y = quaternion[1]
            one_pose.orientation.z = quaternion[2]
            one_pose.orientation.w = quaternion[3]
            object_pose.poses.append(one_pose)
            print(object_pose)

        goal.pose_list.append(object_pose)

    print(goal)

    # 2.Sends the goal to the user solution.
    start_time = rospy.get_time()
    while start_time < 0.001:
        start_time = rospy.get_time()
    rospy.loginfo("Start: " + str(start_time))
    task_manager.send_goal(goal)

    # 3.Waits for the user solution to finish performing the action.
    finished_before_timeout = False
    threshold = 600.0  # seconds.
    finished_before_timeout = \
        task_manager.wait_for_result(rospy.Duration(threshold))
    if finished_before_timeout:
        rospy.loginfo("Action done.")
    else:
        rospy.loginfo("Action did not finish before the time out. Canceling")
        task_manager.cancel_all_goals()
        rospy.sleep(2.0)
    if evaluation:
        gd.stop()
        gd.write_result()
    rospy.loginfo(task_manager.get_result())
    end_time = rospy.get_time()
    while end_time < 0.001:
        end_time = rospy.get_time()
    rospy.loginfo("End: " + str(end_time))

    # 4.Print.
    rospy.loginfo("Get result.")
    time_cost = end_time - start_time
    rospy.loginfo("Task name: " + task_name)
    rospy.loginfo("Time cost: " + str(time_cost) + ' seconds')
    rospy.loginfo("Done.")

    if evaluation:
        # 5. Dump 6dpose
        pd = PoseDumper(task_index = task_index)
        pd.dump()

        # 6. Calculate Score
        IoU_threshold = rospy.get_param('~IoU_threshold')

        score_calculator = ScoreCalculator(task_index = task_index, IoU = IoU_threshold, time_cost = time_cost)
        score_calculator.calculate_score()
