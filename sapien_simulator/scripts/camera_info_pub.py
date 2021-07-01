#! /usr/bin/python3
import rospy
from sensor_msgs.msg import CameraInfo

if __name__ == '__main__':
    rospy.init_node("camera_info_pub")
    # TODO: change the entities below to match TOC challenge
    pub_names = ['kinect_color_pub', 'kinect_depth_pub',
                 'realsense_color_pub', 'realsense_depth_pub']
    topic_names = ['/kinect/color/camera_info',
                   '/realsense/depth/camera_info',
                   '/realsense/color/camera_info',
                   '/realsense/depth/camera_info']
    frame_ids = ['rgb_camera_link', 'depth_camera_link',
                 'realsense_color_optical_frame',
                 'realsense_depth_optical_frame']
    data_length = len(pub_names)
    hw = [(1920, 1080)] * data_length
    K = [
            [917.9434734500945, 0.0, 639.5, 0.0, 917.9434734500945, 359.5, 0.0, 0.0, 1.0],
        ] * data_length

    pubs = []
    messages = []

    for i in range(data_length):
        publisher = rospy.Publisher(topic_names[i], CameraInfo, queue_size=1)
        pubs.append(publisher)
        msg = CameraInfo()
        msg.height = hw[i][0]
        msg.width = hw[i][1]
        msg.header.frame_id = frame_ids[i]
        msg.K = K[i]
        P = K[i].copy()
        P.insert(3, 0.0)
        P.insert(7, 0.0)
        P.insert(11, 0.0)
        msg.P = P
        messages.append(msg)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break
        for i in range(data_length):
            pubs[i].publish(messages[i])
