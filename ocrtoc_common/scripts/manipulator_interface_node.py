#!/usr/bin/env python

import rospy
from ocrtoc_common.manipulator_interface import ManipulatorInterface

if __name__ == '__main__':
    rospy.init_node('manipulator_interface_node')
    m_i = ManipulatorInterface('panda_arm')
    m_i.print_basic_info()
    rospy.spin()
