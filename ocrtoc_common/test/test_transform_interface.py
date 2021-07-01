#!/usr/bin/env python

import rospy
from ocrtoc_common.transform_interface import TransformInterface
from geometry_msgs.msg import PoseStamped, Quaternion

if __name__ == '__main__':
    rospy.init_node('test_transform_interface')
    transform_manager = TransformInterface()
    ros_trans = transform_manager.lookup_ros_transform('world', 'panda_link8')
    print('-'*80)
    print(ros_trans)
    print(type(ros_trans))

    numpy_trans = transform_manager.lookup_numpy_transform('world', 'panda_link8')
    print('-'*80)
    print(numpy_trans)
    print(type(numpy_trans))

    panda_link8_pose = PoseStamped()
    panda_link8_pose.header.frame_id = "panda_link8"
    panda_link8_pose.pose.position.x = 1.0
    panda_link8_pose.pose.orientation.w = 1.0
    print('-'*80)
    print(panda_link8_pose)

    world_pose = transform_manager.do_transform_ros_posestamped(
        panda_link8_pose, ros_trans)
    print('-'*80)
    print(world_pose)
