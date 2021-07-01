import sys
if sys.version_info.major == 3:
    sys.path.insert(1, '/usr/local/lib/python3.6/dist-packages')

import numpy as np
import transforms3d

from geometry_msgs.msg import Pose, PoseStamped
import geometry_msgs.msg
import rospy
import tf
import tf2_ros

class TransformInterface(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def lookup_ros_transform(self, parent_frame, child_frame, timeout=1.0):
        try:
            transform = self.tfBuffer.lookup_transform(parent_frame,
                                                       child_frame,
                                                       rospy.Time(),
                                                       rospy.Duration(timeout))
            return transform
        except tf2_ros.TransformException as e:
            rospy.logerr(e)
            return None

    def lookup_numpy_transform(self, parent_frame, child_frame, timeout=1.0):
        transform = self.lookup_ros_transform(parent_frame,
                                              child_frame,
                                              timeout)
        if transform is None:
            return None

        numpy_transform = self.ros_transform_to_matrix4x4(transform.transform)
        return numpy_transform

    def quaternion_to_euler(self, quaternion):
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    def ros_quaternion_to_euler(self, ros_quaternion):
        quaternion = (ros_quaternion.x, ros_quaternion.y,
                      ros_quaternion.z, ros_quaternion.w)
        euler = self.quaternion_to_euler(quaternion)
        return euler

    def ros_transform_to_matrix4x4(self, transform):
        matrix = np.eye(4, dtype=np.float)
        matrix[0:3, 0:3] = transforms3d.quaternions.quat2mat(
            [transform.rotation.w, transform.rotation.x,
             transform.rotation.y, transform.rotation.z])
        matrix[0:3, 3] = np.array(
            [transform.translation.x, transform.translation.y,
             transform.translation.z]).reshape([3, ])
        return matrix

    def ros_pose_to_matrix4x4(self, pose):
        matrix = np.eye(4, dtype=np.float)
        matrix[0:3, 0:3] = transforms3d.quaternions.quat2mat(
            [pose.orientation.w, pose.orientation.x,
             pose.orientation.y, pose.orientation.z])
        matrix[0:3, 3] = np.array(
            [pose.position.x, pose.position.y,
             pose.position.z]).reshape([3, ])
        return matrix

    def matrix4x4_to_ros_pose(self, matrix):
        pose = Pose()
        quaternion = transforms3d.quaternions.mat2quat(matrix[0:3, 0:3])
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        pose.orientation.w = quaternion[0]
        pose.orientation.x = quaternion[1]
        pose.orientation.y = quaternion[2]
        pose.orientation.z = quaternion[3]
        return pose

    def do_transform_ros_posestamped(
        self, source_pose_stamped, transform_stamped):
        # ROS API tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        child_frame_id = transform_stamped.child_frame_id
        if child_frame_id != source_pose_stamped.header.frame_id:
            rospy.logerr("Invalid frame_id!")
            return None

        target_pose_stamped = PoseStamped()
        transform_matrix = self.ros_transform_to_matrix4x4(
            transform_stamped.transform)
        source_pose_matrix = self.ros_pose_to_matrix4x4(
            source_pose_stamped.pose)
        target_pose_matrix = transform_matrix.dot(source_pose_matrix)

        target_pose_stamped.header = source_pose_stamped.header
        target_pose_stamped.header.frame_id = transform_stamped.header.frame_id
        target_pose_stamped.pose = \
            self.matrix4x4_to_ros_pose(target_pose_matrix)
        return target_pose_stamped
