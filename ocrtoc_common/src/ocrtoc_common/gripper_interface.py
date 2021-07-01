import actionlib
import control_msgs.msg
import rospy

class GripperInterface(object):
    """Python gripper interface

    Args:
        topic_name(str): the ros topic name for the gripper action
    """
    def __init__(self, topic_name = '/franka_gripper/gripper_action'):
        self.topic_name = topic_name
        self._gripper_client = actionlib.SimpleActionClient(self.topic_name, control_msgs.msg.GripperCommandAction)

    def go_to_position(self, position, max_effort = 30, wait_time = 2.0):
        """Move the gripper to position

        Args:
            position(float): the target distance between the two fingers
            max_effort(float): max effort
            wait_time(float): time to wait after sending the command.
        """
        self._gripper_client.wait_for_server()
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = position / 2.0 # The topic target is the distance from one finger to the center.
        goal.command.max_effort = max_effort

        self._gripper_client.send_goal(goal)
        rospy.sleep(wait_time)

    def close(self):
        self.go_to_position(position = 0.0, max_effort = 30, wait_time = 2.0)

    def open(self):
        self.go_to_position(position = 0.078, max_effort = 30, wait_time = 2.0)