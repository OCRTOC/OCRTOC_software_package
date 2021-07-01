import rospy
import copy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ocrtoc_common.srv import JointSpaceGoalRequest, JointSpaceGoal

class ArmController(object):
    def __init__(self, topic = 'arm_controller/command'):
        # Init action.
        self.arm_cmd_pub = rospy.Publisher(
            rospy.resolve_name(topic),
            JointTrajectory, queue_size=10)
        rospy.loginfo("Arm controller initialized with topic:{}".format(topic))
        self.secs = 100

    def exec_joint_goal(self, goal):
        request = JointSpaceGoalRequest()
        request.joint_goal = goal
        # print('='*80)
        rospy.loginfo('Request joint space goal:{}'.format(request.joint_goal))
        rospy.wait_for_service('/send_joint_space_goal')
        try:
            service_call = rospy.ServiceProxy('/send_joint_space_goal', JointSpaceGoal)
            response = service_call(request)
            rospy.loginfo('Finish joint space goal:{}'.format(request.joint_goal))
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)
