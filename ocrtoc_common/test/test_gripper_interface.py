from ocrtoc_common.gripper_interface import GripperInterface
import rospy
import time

if __name__ == '__main__':
    rospy.init_node('test_transform_interface')
    g = GripperInterface(topic_name = '/franka_gripper/gripper_action')


    # go to specific position(distance between fingers)
    distance = 0.04
    print('-' * 80)
    print('Move the gripper to position {}'.format(distance))
    g.go_to_position(position = distance)
    time.sleep(1.0)

    # open the gripper
    print('-' * 80)
    print('Open the gripper')
    g.open()
    time.sleep(1.0)

    # close the gripper
    print('-' * 80)
    print('Close the gripper')
    g.close()
    time.sleep(1.0)

    # open the gripper
    print('-' * 80)
    print('Open the gripper')
    g.open()