#! /usr/bin/env python3

import rospy
from perception_server import PerceptionServer

if __name__ == '__main__':
    rospy.init_node('perception_action')
    config_name = rospy.get_param('~config')
    service_name = rospy.get_param('~service_name')
    perception_action = PerceptionServer(
        service_name = service_name,
        config_name = config_name
    )
    rospy.spin()
