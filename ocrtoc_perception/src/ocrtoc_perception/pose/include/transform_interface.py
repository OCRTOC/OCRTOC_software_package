import math
import numpy as np
from pyquaternion import Quaternion
import sys
import transforms3d

if sys.version_info.major == 2:
    import geometry_msgs.msg
    import rospy
    import tf2_ros


def lookup_transform(parent_link, child_link):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tf_parent_child = tfBuffer.lookup_transform(parent_link,
                                                child_link,
                                                rospy.Time(0),
                                                rospy.Duration(10))
    pose_parent_child = np.eye(4, dtype=np.float)
    pose_parent_child[0:3, 0:3] = transforms3d.quaternions.quat2mat(
        [tf_parent_child.transform.rotation.w,
         tf_parent_child.transform.rotation.x,
         tf_parent_child.transform.rotation.y,
         tf_parent_child.transform.rotation.z])
    pose_parent_child[0:3, 3] = np.array(
        [tf_parent_child.transform.translation.x,
         tf_parent_child.transform.translation.y,
         tf_parent_child.transform.translation.z]).reshape([3, ])
    return pose_parent_child


def position_quaternion_to_matrix(position_quaternion):
    pose_matrix = np.eye(4, dtype=np.float)
    pose_matrix[0:3, 3] = np.array(position_quaternion[0:3]).reshape([3, ])
    pose_matrix[0:3, 0:3] = transforms3d.quaternions.quat2mat(
        position_quaternion[3:7])
    return pose_matrix


def position_rpy_to_matrix(position_rpy):
    pose_matrix = np.eye(4, dtype=np.float)
    pose_matrix[0:3, 3] = np.array(position_rpy[0:3]).reshape([3, ])
    pose_matrix[0:3, 0:3] = transforms3d.euler.euler2mat(position_rpy[3],
                                                         position_rpy[4],
                                                         position_rpy[5])
    return pose_matrix


def matrix_to_position_rpy(matrix):
    rpy = transforms3d.euler.mat2euler(matrix)
    position_rpy = [matrix[0, 3], matrix[1, 3], matrix[2, 3],
                    rpy[0], rpy[1], rpy[2]]
    return position_rpy


def position_rpy_dot(position_rpy1, position_rpy2):
    matrix1 = position_rpy_to_matrix(position_rpy1)
    matrix2 = position_rpy_to_matrix(position_rpy2)
    matrix = matrix1.dot(matrix2)
    return matrix_to_position_rpy(matrix)


def matrix_to_position_quaternion(matrix):
    quaternion = transforms3d.quaternions.mat2quat(matrix[0:3, 0:3])
    position_quaternion = [matrix[0, 3], matrix[1, 3], matrix[2, 3],
                           quaternion[0], quaternion[1], quaternion[2],
                           quaternion[3]]

    return position_quaternion


def rt_dict_to_matrix(rt_dict):
    pose_matrix = np.eye(4, dtype=np.float32)
    pose_matrix[0:3, 0:3] = np.array(rt_dict['R'])
    pose_matrix[0:3, 3] = np.array(rt_dict['t']).reshape([3])
    return pose_matrix


def position_axis_angle_to_matrix(position, axis, angle):
    matrix = np.identity(4)
    matrix[0:3, 0:3] = transforms3d.axangles.axangle2mat(
        axis, angle)
    matrix[0:3, 3] = np.array(position).reshape(([3, ]))
    return matrix


def position_quaternion_to_msg_pose(position_quaternion):
    msg_pose = geometry_msgs.msg.Pose()
    msg_pose.position.x = position_quaternion[0]
    msg_pose.position.y = position_quaternion[1]
    msg_pose.position.z = position_quaternion[2]

    msg_pose.orientation.w = position_quaternion[3]
    msg_pose.orientation.x = position_quaternion[4]
    msg_pose.orientation.y = position_quaternion[5]
    msg_pose.orientation.z = position_quaternion[6]
    return msg_pose


def interpolate_pose(start_pose, end_pose, num_poses):
    assert num_poses > 0
    # pose: x, y, z, qw, qx, qy, qz
    start_pose = np.array(start_pose)
    end_pose = np.array(end_pose)
    if num_poses > 1:
        num_sample = num_poses - 1
    else:
        num_sample = 1
    position_sample = (end_pose[0:3] - start_pose[0:3]) / num_sample

    q_start = Quaternion(start_pose[3],
                         start_pose[4], start_pose[5], start_pose[6])
    q_end = Quaternion(end_pose[3], end_pose[4], end_pose[5], end_pose[6])
    poses = []
    for i in range(num_poses):
        position = start_pose[0:3] + position_sample * i
        orientation = Quaternion.slerp(q_start, q_end,
                                       float(i) / num_sample)
        poses.append([position[0], position[1], position[2], orientation.w,
                      orientation.x, orientation.y, orientation.z])
    return poses


def position_degree_rpy_to_radian(postion_degree_rpy):
    return [postion_degree_rpy[0],
            postion_degree_rpy[1],
            postion_degree_rpy[2],
            math.radians(postion_degree_rpy[3]),
            math.radians(postion_degree_rpy[4]),
            math.radians(postion_degree_rpy[5])]


def position_radian_rpy_to_degree(postion_radian_rpy):
    return [postion_radian_rpy[0],
            postion_radian_rpy[1],
            postion_radian_rpy[2],
            math.degrees(postion_radian_rpy[3]),
            math.degrees(postion_radian_rpy[4]),
            math.degrees(postion_radian_rpy[5])]


def list_degrees_to_radians(list_degrees):
    for i in range(len(list_degrees)):
        list_degrees[i] = math.radians(list_degrees[i])
    return list_degrees


def normalize_matrix_rpy(matrix):
    rpy = transforms3d.euler.mat2euler(matrix[0:3, 0:3])
    matrix[0:3, 0:3] = transforms3d.euler.euler2mat(rpy[0], rpy[1], rpy[2])
    return matrix


class PositionRpyPoseGenerator:
    def __init__(self):
        self.poses = []

    def generate_interpolated_poses(self,
                                    position_rpy_start,
                                    position_rpy_end,
                                    num_poses):
        position_quaternion_start = matrix_to_position_quaternion(
            position_rpy_to_matrix(position_rpy_start))
        position_quaternion_end = matrix_to_position_quaternion(
            position_rpy_to_matrix(position_rpy_end))
        for pose in interpolate_pose(position_quaternion_start,
                                     position_quaternion_end,
                                     num_poses):
            self.poses.append(pose)

    def generate_rotated_poses(self, position_rpy_origin, radius, num_poses):
        # rotate around an object
        angle_sample = 2 * math.pi / num_poses

        q_origin = transforms3d.quaternions.mat2quat(
            transforms3d.euler.euler2mat(position_rpy_origin[3],
                                         position_rpy_origin[4],
                                         position_rpy_origin[5]))

        self.poses.append([position_rpy_origin[0],
                           position_rpy_origin[1],
                           position_rpy_origin[2],
                           q_origin[0], q_origin[1], q_origin[2],
                           q_origin[3]])

        for i in range(num_poses):
            angle = angle_sample * i
            rotation_z = transforms3d.euler.axangle2mat([0, 0, 1], angle)
            rotation_45 = transforms3d.euler.axangle2mat(
                rotation_z.dot(np.array([0, 1, 0])), math.radians(45))

            q_orientation = transforms3d.quaternions.mat2quat(
                rotation_45.dot(rotation_z.dot(
                    transforms3d.euler.euler2mat(
                        position_rpy_origin[3],
                        position_rpy_origin[4],
                        position_rpy_origin[5]))))

            self.poses.append(
                [position_rpy_origin[0] + radius * math.cos(angle),
                 position_rpy_origin[1] + radius * math.sin(angle),
                 position_rpy_origin[2],
                 q_orientation[0],
                 q_orientation[1],
                 q_orientation[2],
                 q_orientation[3]])

    def generate_x_circle_poses(self, position_rpy_origin, radius, num_poses):
        # rotate around an object
        if num_poses != 0:
            angle_sample = 2 * math.pi / num_poses

        rotation_matrix_origin = transforms3d.euler.euler2mat(
            position_rpy_origin[3],
            position_rpy_origin[4],
            position_rpy_origin[5])
        q_origin = transforms3d.quaternions.mat2quat(rotation_matrix_origin)

        self.poses.append([position_rpy_origin[0],
                           position_rpy_origin[1],
                           position_rpy_origin[2],
                           q_origin[0], q_origin[1], q_origin[2],
                           q_origin[3]])

        for i in range(num_poses):
            angle = angle_sample * i
            radius_position = rotation_matrix_origin.dot(
                np.array([0,
                          radius * math.cos(angle),
                          radius * math.sin(angle)]).reshape([3, ]))
            self.poses.append(
                [position_rpy_origin[0] + radius_position[0],
                 position_rpy_origin[1] + radius_position[1],
                 position_rpy_origin[2] + radius_position[2],
                 q_origin[0], q_origin[1], q_origin[2], q_origin[3]])

    def generate_x_circle_rotated_poses(self,
                                      position_rpy_origin,
                                      radius,
                                      num_poses,
                                      oblique_angle):
        # rotate around an object
        if num_poses != 0:
            angle_sample = 2 * math.pi / num_poses

        rotation_matrix_origin = transforms3d.euler.euler2mat(
            position_rpy_origin[3],
            position_rpy_origin[4],
            position_rpy_origin[5])
        q_origin = transforms3d.quaternions.mat2quat(rotation_matrix_origin)

        for i in range(num_poses):
            angle = angle_sample * i

            axis_z = transforms3d.euler.axangle2mat([1, 0, 0], angle).dot(
                np.array([0, 0, 1]).reshape([3, ]))
            rotation_oblique = transforms3d.euler.euler2mat(
                position_rpy_origin[3],
                position_rpy_origin[4],
                position_rpy_origin[5]).dot(
                transforms3d.euler.axangle2mat(axis_z,
                                               -oblique_angle))
            q_orientation = transforms3d.quaternions.mat2quat(rotation_oblique)

            radius_position = rotation_oblique.dot(
                np.array([0,
                          radius * math.cos(angle),
                          radius * math.sin(angle)]).reshape([3, ]))
            self.poses.append(
                [position_rpy_origin[0] + radius_position[0],
                 position_rpy_origin[1] + radius_position[1],
                 position_rpy_origin[2] + radius_position[2],
                 q_orientation[0], q_orientation[1], q_orientation[2],
                 q_orientation[3]])

    def generate_y_circle_poses(self, position_rpy_origin, radius, num_poses):
        # rotate around an object
        if num_poses != 0:
            angle_sample = 2 * math.pi / num_poses

        rotation_matrix_origin = transforms3d.euler.euler2mat(
            position_rpy_origin[3],
            position_rpy_origin[4],
            position_rpy_origin[5])
        q_origin = transforms3d.quaternions.mat2quat(rotation_matrix_origin)

        self.poses.append([position_rpy_origin[0],
                           position_rpy_origin[1],
                           position_rpy_origin[2],
                           q_origin[0], q_origin[1], q_origin[2], q_origin[3]])

        for i in range(num_poses):
            angle = angle_sample * i
            radius_position = rotation_matrix_origin.dot(
                np.array([radius * math.sin(angle),
                          0,
                          radius * math.cos(angle)]).reshape([3, ]))
            self.poses.append(
                [position_rpy_origin[0] + radius_position[0],
                 position_rpy_origin[1] + radius_position[1],
                 position_rpy_origin[2] + radius_position[2],
                 q_origin[0], q_origin[1], q_origin[2], q_origin[3]])

    def generate_y_circle_rotated_poses(self,
                                      position_rpy_origin,
                                      radius,
                                      num_poses,
                                      oblique_angle):
        # rotate around an object
        if num_poses != 0:
            angle_sample = 2 * math.pi / num_poses

        rotation_matrix_origin = transforms3d.euler.euler2mat(
            position_rpy_origin[3],
            position_rpy_origin[4],
            position_rpy_origin[5])
        q_origin = transforms3d.quaternions.mat2quat(rotation_matrix_origin)

        for i in range(num_poses):
            angle = angle_sample * i

            axis_z = transforms3d.euler.axangle2mat([0, 1, 0], angle).dot(
                np.array([1, 0, 0]).reshape([3, ]))
            rotation_oblique = transforms3d.euler.euler2mat(
                position_rpy_origin[3],
                position_rpy_origin[4],
                position_rpy_origin[5]).dot(
                transforms3d.euler.axangle2mat(axis_z,
                                               -oblique_angle))
            q_orientation = transforms3d.quaternions.mat2quat(rotation_oblique)

            radius_position = rotation_oblique.dot(
                np.array([radius * math.sin(angle),
                          0,
                          radius * math.cos(angle)]).reshape([3, ]))
            self.poses.append(
                [position_rpy_origin[0] + radius_position[0],
                 position_rpy_origin[1] + radius_position[1],
                 position_rpy_origin[2] + radius_position[2],
                 q_orientation[0], q_orientation[1], q_orientation[2],
                 q_orientation[3]])


if __name__ == '__main__':
    rpy = transforms3d.euler.mat2euler(transforms3d.axangles.axangle2mat(
        [0, 1, 0], math.radians(90)))
    print(rpy)
    print(math.degrees(rpy[0]))
