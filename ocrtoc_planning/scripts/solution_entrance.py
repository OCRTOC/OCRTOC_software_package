#! /usr/bin/env python

import actionlib
import rospy
import ocrtoc_msg.msg
from task_planning import TaskPlanner

class SolutionServer(object):
    def __init__(self, name):
        # Init action.
        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(
            self.action_name, ocrtoc_msg.msg.OrganizationAction,
            execute_cb=self.execute_callback, auto_start=False)
        self.action_server.start()
        rospy.loginfo(self.action_name + " is running.")

        # create messages that are used to publish feedback/result.
        self.feedback = ocrtoc_msg.msg.OrganizationFeedback()
        self.result = ocrtoc_msg.msg.OrganizationResult()

    def execute_callback(self, goal):
        rospy.loginfo("Got task!")
        print(goal)

        # extract pose information
        blocks = goal.object_list
        goal_cartesian_poses = goal.pose_list

        # start task planning
        self.planner = TaskPlanner(blocks, goal_cartesian_poses)
        self.planner.cycle_plan_all()
        rospy.loginfo("planning finished")

        # Example: set status "Aborted" and quit.
        if self.action_server.is_preempt_requested():
            self.result.status = "Aborted"
            self.action_server.set_aborted(self.result)
            return

        # Example: send feedback
        self.feedback.text = "write_feedback_text_here"
        self.action_server.publish_feedback(self.feedback)
        rospy.loginfo("Pub feedback")
        rospy.sleep(1.0)

        # Example: set status "Finished" and quit.
        self.result.status = "Finished"
        rospy.loginfo("Done.")
        self.action_server.set_succeeded(self.result)
        ##### User code example ends #####


if __name__ == '__main__':
    rospy.init_node('solution_server')
    solution_server = SolutionServer('solution_server')
    rospy.spin()
