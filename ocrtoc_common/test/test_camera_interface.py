#!/usr/bin/env python
from matplotlib import pyplot as plt
import open3d as o3d

import rospy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from ocrtoc_common.camera_interface import CameraInterface

if __name__ == '__main__':
    rospy.init_node('test_camera_interface')
    camera_manager = CameraInterface()
    color_topic_name = "/realsense/color/image_raw"
    depth_topic_name = "/realsense/aligned_depth_to_color/image_raw"
    points_topic_name = "/realsense/depth/points"
    color_camera_info_topic = "/realsense/color/camera_info"
    camera_manager.subscribe_topic(color_topic_name, Image)
    camera_manager.subscribe_topic(depth_topic_name, Image)
    camera_manager.subscribe_topic(points_topic_name, PointCloud2)
    camera_manager.subscribe_topic(color_camera_info_topic, CameraInfo)
    rospy.sleep(2.0)
    print('-'*80)
    print('ros image topic')
    print(camera_manager.get_ros_image(color_topic_name).encoding)
    print('-'*80)
    print('numpy image array')
    print(camera_manager.get_numpy_image_with_encoding(color_topic_name)[0])
    print('-'*80)
    print('numpy image encoding')
    print(camera_manager.get_numpy_image_with_encoding(color_topic_name)[1])

    plt.imshow(camera_manager.get_numpy_image_with_encoding(color_topic_name)[0]), plt.show()
    print('-'*80)
    print('numpy depth image array')
    print(camera_manager.get_numpy_image_with_encoding(depth_topic_name)[0])
    print('-'*80)
    print('numpy depth image topic')
    print(camera_manager.get_numpy_image_with_encoding(depth_topic_name)[1])
    print('-'*80)
    print('numpy depth image shape')
    print(camera_manager.get_numpy_image_with_encoding(depth_topic_name)[0].shape)
    print('-'*80)
    depth = camera_manager.get_numpy_image_with_encoding(depth_topic_name)[0]
    plt.imshow(depth)
    plt.show()
    print(len(camera_manager.get_ros_points(points_topic_name).data))
    print('-'*80)
    print(camera_manager.get_numpy_points(points_topic_name))
    pcd = camera_manager.get_o3d_pcd(points_topic_name)
    o3d.visualization.draw_geometries([pcd])
    
    print('numpy points.shape:{}'.format(camera_manager.get_numpy_points(points_topic_name).shape))

    print('-'*80)
    print(camera_manager.get_ros_camera_info(color_camera_info_topic))
    print('-'*80)
    print(camera_manager.get_dict_camera_info(color_camera_info_topic))
    # d = camera_manager.get_dict_camera_info(color_camera_info_topic)
    # import numpy as np
    # print(np.array(d['K']).reshape((3,3)))

