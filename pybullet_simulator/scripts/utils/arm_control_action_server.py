#! /usr/bin/env python

import rospy

import actionlib

import control_msgs.msg
import trajectory_msgs.msg

class ArmControlAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.JointTrajectoryFeedback()
    _result = control_msgs.msg.JointTrajectoryResult()
    _goal = control_msgs.msg.JointTrajectoryGoal()


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.JointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # append current arm state as feedback
        # self._feedback.position = 0

        # publish info to the console for the user
        rospy.loginfo('%s: Executing' % (self._action_name))

        # # Test trajectory
        # point1 = trajectory_msgs.msg.JointTrajectoryPoint()
        # point2 = trajectory_msgs.msg.JointTrajectoryPoint()
        # point1.positions = [0]
        # point1.time_from_start.secs = 0
        # point1.time_from_start.nsecs = 0
        # point2.positions = [1]
        # point2.time_from_start.secs = 1
        # point2.time_from_start.nsecs = 0
        # temp_goal = control_msgs.msg.JointTrajectoryGoal()
        # temp_goal.trajectory.joint_names = ['shoulder_pan_joint']
        # temp_goal.trajectory.points = [point1, point2]
        # print('temp_goal:', temp_goal)

        robot.executeArmJointTrajectory(goal.trajectory)

        # # start executing the action
        # for i in range(10):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     # real work here
        #     self._feedback.position = i*0.1
        #
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()

        if success:
            # self._result.position = self._feedback.position
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    import pybullet
    from env.pybullet_env import SimRobot, Manipulator

    # connect to simulation
    pybullet.connect(pybullet.SHARED_MEMORY)

    robot = Manipulator.fromID(id=1)

    rospy.init_node('arm_control_action')
    print(rospy.get_name())

    server = ArmControlAction('/arm_control_action')
    rospy.spin()