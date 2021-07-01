#! /usr/bin/env python

import rospy

import actionlib

import control_msgs.msg

class GripperControlAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()
    _goal = control_msgs.msg.GripperCommandGoal()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, goal %f' % (self._action_name, goal.command.position))

        robot.gripperControl(gripper_opening_length=goal.command.position, T=1.0)

        # # append current gripper position as feedback
        # self._feedback.position = 0

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
            self._result.position = self._feedback.position
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    import pybullet
    from env.pybullet_env import SimRobot, Manipulator

    # connect to simulation
    pybullet.connect(pybullet.SHARED_MEMORY)

    robot = Manipulator.fromID(id=1)

    rospy.init_node('gripper_control_action')
    server = GripperControlAction('/gripper_control_action')
    rospy.spin()