#!/usr/bin/env python

import ctypes
import numpy as np
import struct
import threading
import open3d as o3d

import ros_numpy
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

class Callback(object):
    def __init__(self):
        self._event = threading.Event()
        self._msg = None

    def __call__(self, msg):
        self._msg = msg
        self._event.set()

    def get_msg(self, timeout=0.5):
        self._event.wait(timeout)
        return self._msg

class CameraInterface(object):
    def __init__(self):
        self.subscribe_topic_dict = {}
        self.topic_type_list = [CameraInfo, Image, PointCloud2]

    def topic_name_to_callback_name(self, topic_name):
        return topic_name.replace('/', '_') + '_callback'

    def check_topic_subscribed(self, topic_name):
        if topic_name in self.subscribe_topic_dict:
            return True
        else:
            rospy.logerr(topic_name + " was not subscribed!")
            return False

    def subscribe_topic(self, topic_name, topic_type):
        if topic_name in self.subscribe_topic_dict:
            rospy.logerr(topic_name + " has been subscribed!")
            return False
        if topic_type not in self.topic_type_list:
            rospy.logerr("Wrong topic type!")
            rospy.logerr("Options: Sensor_msgs/CameraInfo Sensor_msgs/Image Sensor_msgs/PointCloud2")
            return False

        # Subscribe topic
        callback_name = self.topic_name_to_callback_name(topic_name)
        setattr(self, callback_name, Callback())
        rospy.loginfo("Subscribe topic: " + topic_name)
        rospy.loginfo("Topic type(" + str(topic_type))
        rospy.loginfo("Callback function: " + callback_name)
        rospy.Subscriber(topic_name, topic_type, self.__dict__[callback_name])
        # Print topic list
        self.subscribe_topic_dict.update({topic_name: topic_type})
        rospy.loginfo("Subscribe list:")
        rospy.loginfo(self.subscribe_topic_dict)
        return True

    def get_ros_camera_info(self, topic_name):
        if self.check_topic_subscribed(topic_name) is False:
            return None
        if self.subscribe_topic_dict[topic_name] is not CameraInfo:
            rospy.logerr("Wrong topic type! Expectant Type is Sensor_msgs/CameraInfo")
            return None

        callback_name = self.topic_name_to_callback_name(topic_name)
        return self.__dict__[callback_name].get_msg()

    def get_dict_camera_info(self, topic_name):
        if self.check_topic_subscribed(topic_name) is False:
            return None
        if self.subscribe_topic_dict[topic_name] is not CameraInfo:
            rospy.logerr("Wrong topic type! Expectant Type is Sensor_msgs/CameraInfo")
            return None

        callback_name = self.topic_name_to_callback_name(topic_name)
        ros_msg = self.__dict__[callback_name].get_msg()
        msg_dict = {'frame_id': ros_msg.header.frame_id,
                    'image_height': ros_msg.height,
                    'image_width': ros_msg.width,
                    'D': list(ros_msg.D),
                    'K': list(ros_msg.K)}
        return msg_dict

    def get_ros_image(self, topic_name):
        if self.check_topic_subscribed(topic_name) is False:
            return None
        if self.subscribe_topic_dict[topic_name] is not Image:
            rospy.logerr("Wrong topic type! Expectant Type is Sensor_msgs/Image")
            return None

        callback_name = self.topic_name_to_callback_name(topic_name)
        return self.__dict__[callback_name].get_msg()

    def get_numpy_image_with_encoding(self, topic_name):
        """
        Get image in numpy format.

        Args:
            topic_name(str): topic name.

        Returns:
            numpy.ndarray, str: numpy array of the image and its encoding
        """
        ros_image = self.get_ros_image(topic_name)
        encoding = ros_image.encoding
        return ros_numpy.numpify(ros_image), encoding

    def get_ros_points(self, topic_name):
        if self.check_topic_subscribed(topic_name) is False:
            return None
        if self.subscribe_topic_dict[topic_name] is not PointCloud2:
            rospy.logerr("Wrong topic type! Expectant Type is Sensor_msgs/PointCloud2")
            return None

        callback_name = self.topic_name_to_callback_name(topic_name)
        return self.__dict__[callback_name].get_msg()

    def get_numpy_points(self, topic_name):
        if self.check_topic_subscribed(topic_name) is False:
            return None
        if self.subscribe_topic_dict[topic_name] is not PointCloud2:
            rospy.logerr("Wrong topic type! Expectant Type is Sensor_msgs/PointCloud2")
            return None

        callback_name = self.topic_name_to_callback_name(topic_name)
        points_msg = self.__dict__[callback_name].get_msg()

        # ros_point_list is a list of points with type sensor_msgs.point_cloud2.Point
        ros_points_list = point_cloud2.read_points_list(points_msg, skip_nans=True)
        xyzrgb_points_list = []
        for each_point in ros_points_list:
            x = each_point.x
            y = each_point.y
            z = each_point.z
            rgb = each_point.rgb
            # cast float32 to int so that bitwise operations are possible
            str_var = struct.pack('>f', rgb) # '>f' means "big-endian float"
            int_var = struct.unpack('>l', str_var)[0] # '>l' means "big-endian long"
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(int_var).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            xyzrgb_points_list.append([x, y, z, r, g, b])

        return np.array(xyzrgb_points_list)

    def get_o3d_pcd(self, topic_name, use_graspnet_camera_frame = False):
        ## pay attention that the camera coordinate systems are different in here and in graspnet ##
        if self.check_topic_subscribed(topic_name) is False:
            return None
        if self.subscribe_topic_dict[topic_name] is not PointCloud2:
            rospy.logerr("Wrong topic type! Expectant Type is Sensor_msgs/PointCloud2")
            return None
        np_points = self.get_numpy_points(topic_name = topic_name)
        pcd = o3d.geometry.PointCloud()
        if use_graspnet_camera_frame:
            points = np.zeros(shape = (len(np_points), 3), dtype = np_points.dtype)
            points[:,0] = -np_points[:,1]
            points[:,1] = -np_points[:,2]
            points[:,2] = np_points[:,0]
        else:
            points = np_points[:,:3]
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(np_points[:,3:] / 255.0)
        return pcd