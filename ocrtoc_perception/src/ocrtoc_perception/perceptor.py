# Author: Minghao Gou

import numpy as np
import cv2
from numpy.core.numeric import full
import rospy
import rospkg
import open3d as o3d
import open3d_plus as o3dp
from transforms3d.quaternions import mat2quat
import time
import copy
import os
from copy import deepcopy

from .arm_controller import ArmController
from .graspnet import GraspNetBaseLine
from .pose.pose_6d import get_6d_pose_by_geometry, load_model_pcd
from .pose.pose_correspondence import get_pose_superglue

from ocrtoc_common.camera_interface import CameraInterface
from ocrtoc_common.transform_interface import TransformInterface
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

def crop_pcd(pcds, i, reconstruction_config):
    points, colors = o3dp.pcd2array(pcds[i])
    mask = points[:, 2] > reconstruction_config['z_min']
    mask = mask & (points[:, 2] < reconstruction_config['z_max'])
    mask = mask & (points[:, 0] > reconstruction_config['x_min'])
    mask = mask & (points[:, 0] < reconstruction_config['x_max'])
    mask = mask & (points[:, 1] < reconstruction_config['y_max'])
    mask = mask & (points[:, 1] > reconstruction_config['y_min'])
    pcd = o3dp.array2pcd(points[mask], colors[mask])
    return pcd

def kinect_process_pcd(pcd, reconstruction_config):
    points, colors = o3dp.pcd2array(pcd)
    mask = points[:, 2] > reconstruction_config['z_min']
    mask = mask & (points[:, 2] < reconstruction_config['z_max'])
    mask = mask & (points[:, 0] > reconstruction_config['x_min'])
    mask = mask & (points[:, 0] < reconstruction_config['x_max'])
    mask = mask & (points[:, 1] < reconstruction_config['y_max'])
    mask = mask & (points[:, 1] > reconstruction_config['y_min'])
    pcd = o3dp.array2pcd(points[mask], colors[mask])
    return pcd.voxel_down_sample(reconstruction_config['voxel_size'])

def process_pcds(pcds, use_camera, reconstruction_config):
    trans = dict()
    start_id = reconstruction_config['{}_camera_order'.format(use_camera)][0]
    pcd = copy.deepcopy(crop_pcd(pcds, start_id, reconstruction_config))
    pcd.estimate_normals()
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors = reconstruction_config['nb_neighbors'],
        std_ratio = reconstruction_config['std_ratio']
    )
    idx_list = reconstruction_config['{}_camera_order'.format(use_camera)]
    for i in idx_list[1:]: # the order are decided by the camera pose
        voxel_size = reconstruction_config['voxel_size']
        income_pcd = copy.deepcopy(crop_pcd(pcds, i, reconstruction_config))
        income_pcd, _ = income_pcd.remove_statistical_outlier(
            nb_neighbors = reconstruction_config['nb_neighbors'],
            std_ratio = reconstruction_config['std_ratio']
        )
        income_pcd.estimate_normals()
        income_pcd = income_pcd.voxel_down_sample(voxel_size)
        transok_flag = False
        for _ in range(reconstruction_config['icp_max_try']): # try 5 times max
            reg_p2p = o3d.pipelines.registration.registration_icp(
                income_pcd,
                pcd,
                reconstruction_config['max_correspondence_distance'],
                np.eye(4, dtype = np.float),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(reconstruction_config['icp_max_iter'])
            )
            if (np.trace(reg_p2p.transformation) > reconstruction_config['translation_thresh']) \
                and (np.linalg.norm(reg_p2p.transformation[:3, 3]) < reconstruction_config['rotation_thresh']):
                # trace for transformation matrix should be larger than 3.5
                # translation should less than 0.05
                transok_flag = True
                break
        if not transok_flag:
            reg_p2p.transformation = np.eye(4, dtype = np.float32)
        income_pcd = income_pcd.transform(reg_p2p.transformation)
        trans[i] = reg_p2p.transformation
        pcd = o3dp.merge_pcds([pcd, income_pcd])
        cd = pcd.voxel_down_sample(voxel_size)
        pcd.estimate_normals()
    return trans, pcd

class Perceptor():
    def __init__(
        self,
        config
    ):
        self.config = config
        self.debug = self.config['debug']
        self.graspnet_baseline = GraspNetBaseLine(
            checkpoint_path = os.path.join(
                rospkg.RosPack().get_path('ocrtoc_perception'),
                self.config['graspnet_checkpoint_path']
            )
        )
        self.color_info_topic_name = self.config['color_info_topic_name']
        self.color_topic_name = self.config['color_topic_name']
        self.depth_topic_name = self.config['depth_topic_name']
        self.points_topic_name = self.config['points_topic_name']
        self.kinect_color_topic_name = self.config['kinect_color_topic_name']
        self.kinect_depth_topic_name = self.config['kinect_depth_topic_name']
        self.kinect_points_topic_name = self.config['kinect_points_topic_name']
        self.use_camera = self.config['use_camera']
        self.arm_controller = ArmController(topic = self.config['arm_topic'])
        self.camera_interface = CameraInterface()
        rospy.sleep(2)
        self.transform_interface = TransformInterface()
        self.transform_from_frame = self.config['transform_from_frame']
        if self.use_camera in ['realsense', 'both']:
            self.camera_interface.subscribe_topic(self.color_info_topic_name, CameraInfo)
            self.camera_interface.subscribe_topic(self.color_topic_name, Image)
            self.camera_interface.subscribe_topic(self.points_topic_name, PointCloud2)
            time.sleep(2)
            self.color_transform_to_frame = self.get_color_image_frame_id()
            self.points_transform_to_frame = self.get_points_frame_id()
        if self.use_camera in ['kinect', 'both']:
            self.camera_interface.subscribe_topic(self.kinect_color_topic_name, Image)
            self.camera_interface.subscribe_topic(self.kinect_points_topic_name, PointCloud2)
            time.sleep(2)
            self.kinect_color_transform_to_frame = self.get_kinect_color_image_frame_id()
            self.kinect_points_transform_to_frame = self.get_kinect_points_frame_id()
        self.fixed_arm_poses = np.loadtxt(
            os.path.join(
                rospkg.RosPack().get_path('ocrtoc_perception'),
                self.config['realsense_camera_fix_pose_file'],
            ),
            delimiter = ','
        )
        self.fixed_arm_poses_both = np.loadtxt(
            os.path.join(
                rospkg.RosPack().get_path('ocrtoc_perception'),
                self.config['both_camera_fix_pose_file'],
            ),
            delimiter = ','
        )

    def get_color_image(self):
        return self.camera_interface.get_numpy_image_with_encoding(self.color_topic_name)[0]

    def get_color_camK(self):
        d = self.camera_interface.get_dict_camera_info(self.color_info_topic_name)
        return (np.array(d['K']).reshape((3,3)))

    def get_depth_image(self):
        return self.camera_interface.get_numpy_image_with_encoding(self.depth_topic_name)[0]

    def get_color_image_frame_id(self):
        return self.camera_interface.get_ros_image(self.color_topic_name).header.frame_id

    def get_depth_image_frame_id(self):
        return self.camera_interface.get_ros_image(self.depth_topic_name).header.frame_id

    def get_points_frame_id(self):
        return self.camera_interface.get_ros_points(self.points_topic_name).header.frame_id

    def get_kinect_color_image_frame_id(self):
        return self.camera_interface.get_ros_image(self.kinect_color_topic_name).header.frame_id

    def get_kinect_depth_image_frame_id(self):
        return self.camera_interface.get_ros_image(self.kinect_depth_topic_name).header.frame_id

    def get_kinect_points_frame_id(self):
        return self.camera_interface.get_ros_points(self.kinect_points_topic_name).header.frame_id

    def get_pcd(self, use_graspnet_camera_frame = False):
        pcd = self.camera_interface.get_o3d_pcd(self.points_topic_name)
        return pcd

    def kinect_get_pcd(self, use_graspnet_camera_frame = False):
        return self.camera_interface.get_o3d_pcd(self.kinect_points_topic_name)

    def get_color_transform_matrix(self):
        return self.transform_interface.lookup_numpy_transform(self.transform_from_frame, self.color_transform_to_frame)

    def get_points_transform_matrix(self):
        return self.transform_interface.lookup_numpy_transform(self.transform_from_frame, self.points_transform_to_frame)

    def get_kinect_color_transform_matrix(self):
        return self.transform_interface.lookup_numpy_transform(self.transform_from_frame, self.kinect_color_transform_to_frame)

    def get_kinect_points_transform_matrix(self):
        return self.transform_interface.lookup_numpy_transform(self.transform_from_frame, self.kinect_points_transform_to_frame)

    def capture_data(self):
        t1 = time.time()
        pcds = []
        color_images = []
        camera_poses = []
        # capture images by realsense. The camera will be moved to different locations.
        if self.use_camera in ['realsense', 'both']:
            if self.use_camera == 'realsense':
                arm_poses = self.fixed_arm_poses
            else:
                arm_poses = np.array(self.fixed_arm_poses_both).tolist()
            for arm_pose in arm_poses:
                self.arm_controller.exec_joint_goal(arm_pose)
                rospy.sleep(2.0)
                time.sleep(1.0)
                color_image = self.get_color_image()
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGBA2RGB)
                if self.debug:
                    cv2.imshow('color', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                color_images.append(color_image)

                points_trans_matrix = self.get_points_transform_matrix()
                if self.debug:
                    print('points_trans_matrix:', points_trans_matrix)
                camera_poses.append(self.get_color_transform_matrix())
                pcd = self.get_pcd(use_graspnet_camera_frame = False)
                pcd.transform(points_trans_matrix)
                pcd = kinect_process_pcd(pcd, self.config['reconstruction'])
                if self.debug:
                    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
                    o3d.visualization.draw_geometries([pcd, frame])
                pcds.append(pcd)

        # capture image by kinect
        if self.use_camera in ['kinect', 'both']:
            points_trans_matrix = self.kinect_get_points_transform_matrix()
            full_pcd_kinect = self.kinect_get_pcd(use_graspnet_camera_frame = False) # in sapien frame.
            full_pcd_kinect.transform(points_trans_matrix)
            full_pcd_kinect = kinect_process_pcd(full_pcd_kinect, self.config['reconstruction'])
            if self.use_camera == 'both':
                pcds.append(full_pcd_kinect)
        # if more than one images are used, the scene will be reconstructed by regitration.
        if self.use_camera in ['realsense', 'both']:
            trans, full_pcd_realsense = process_pcds(pcds, use_camera = self.use_camera, reconstruction_config = self.config['reconstruction'])

        if self.use_camera == 'realsense':
            full_pcd = full_pcd_realsense
        elif self.use_camera == 'kinect':
            full_pcd = full_pcd_kinect
        elif self.use_camera == 'both':
            full_pcd = full_pcd_realsense
        else:
            raise ValueError('"use_camrera" should be "kinect", "realsense" or "both"')
        if self.debug:
            t2 = time.time()
            rospy.loginfo('Capturing data time:{}'.format(t2 - t1))
        return full_pcd, color_images, camera_poses

    def compute_6d_pose(self, full_pcd, color_images, camera_poses, pose_method, object_list):
        camK = self.get_color_camK()
        if pose_method == 'icp':
            if self.debug:
                print('Using ICP to obtain 6d poses')
            object_poses = get_6d_pose_by_geometry(
                pcd = full_pcd,
                model_names = object_list,
                geometry_6d_config = self.config['6d_pose_by_geometry'],
                debug = self.debug
            )
        elif pose_method == 'superglue':
            if self.debug:
                rospy.logdebug('Using SuperGlue and PnP to obatin 6d poses')
            object_poses = get_pose_superglue(
                obj_list = object_list,
                images = color_images,
                camera_poses = camera_poses,
                camera_matrix = camK,
                superglue_config = self.config['superglue']
            )
        else:
            rospy.roserr('Unknown pose method:{}'.format(pose_method))
            raise ValueError('Unknown pose method:{}'.format(pose_method))
        if self.debug:
            # Visualize 6dpose estimation result.
            geolist = []
            geolist.append(full_pcd)
            for object_name in object_list:
                if object_name in object_poses.keys():
                    this_model_pcd = load_model_pcd(object_name, os.path.join(
                            rospkg.RosPack().get_path('ocrtoc_materials'),
                            'models'
                        )
                    )
                    geolist.append(this_model_pcd.transform(object_poses[object_name]['pose']))
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
            total_pcd = o3dp.merge_pcds([*geolist])
            o3d.visualization.draw_geometries([*geolist, frame])
            rospy.loginfo(object_poses)
        return object_poses

    def compute_grasp_pose(self, full_pcd):
        points, _ = o3dp.pcd2array(full_pcd)
        grasp_pcd = copy.deepcopy(full_pcd)
        grasp_pcd.points = o3d.utility.Vector3dVector(-points)

        # generating grasp poses.
        gg = self.graspnet_baseline.inference(grasp_pcd)
        gg.translations = -gg.translations
        gg.rotation_matrices = -gg.rotation_matrices
        gg.translations = gg.translations + gg.rotation_matrices[:, :, 0] * self.config['graspnet']['refine_approach_dist']
        gg = self.graspnet_baseline.collision_detection(gg, points)

        # all the returned result in 'world' frame. 'gg' using 'graspnet' gripper frame.
        return gg

    def assign_grasp_pose(self, gg, object_poses):
        grasp_poses = dict()
        dist_thresh = self.config['response']['dist_thresh']
        # - dist_thresh: float of the minimum distance from the grasp pose center to the object center. The unit is millimeter.
        angle_thresh = self.config['response']['angle_thresh']
        # - angle_thresh:
        #             /|
        #            / |
        #           /--|
        #          /   |
        #         /    |
        # Angle should be smaller than this angle

        object_names = []
        # gg: GraspGroup in 'world' frame of 'graspnet' gripper frame.
        # x is the approaching direction.
        ts = gg.translations
        rs = gg.rotation_matrices
        depths = gg.depths
        scores = gg.scores

        # move the center to the eelink frame
        ts = ts + rs[:,:,0] * (np.vstack((depths, depths, depths)).T)
        eelink_rs = np.zeros(shape = (len(rs), 3, 3), dtype = np.float32)

        # the coordinate systems are different in graspnet and ocrtoc
        eelink_rs[:,:,0] = rs[:,:,2]
        eelink_rs[:,:,1] = -rs[:,:,1]
        eelink_rs[:,:,2] = rs[:,:,0]

        # min_dist: np.array of the minimum distance to any object(must > dist_thresh)
        min_dists = np.inf * np.ones((len(rs)))

        # min_object_ids: np.array of the id of the nearest object.
        min_object_ids = -1 * np.ones(shape = (len(rs)), dtype = np.int32)


        # first round to find the object that each grasp belongs to.

        # Pay attention that even the grasp pose may be accurate,
        # poor 6dpose estimation result may lead to bad grasping result
        # as this step depends on the 6d pose estimation result.
        angle_mask = (rs[:, 2, 0] < -np.cos(angle_thresh / 180.0 * np.pi))
        for i, object_name in enumerate(object_poses.keys()):
            object_names.append(object_name)
            object_pose = object_poses[object_name]

            dists = np.linalg.norm(ts - object_pose['pose'][:3,3], axis=1)
            object_mask = np.logical_and(dists < min_dists, dists < dist_thresh)

            min_object_ids[object_mask] = i
            min_dists[object_mask] = dists[object_mask]
        remain_gg = []
        # second round to calculate the parameters
        for i, object_name in enumerate(object_poses.keys()):
            object_pose = object_poses[object_name]

            obj_id_mask = (min_object_ids == i)
            add_angle_mask = (obj_id_mask & angle_mask)
            # For safety and planning difficulty reason, grasp pose with small angle with gravity direction will be accept.
            # if no grasp pose is available within the safe cone. grasp pose with the smallest angle will be used without
            # considering the angle.
            if np.sum(add_angle_mask) < self.config['response']['mask_thresh']: # actually this should be mask == 0, for safety reason, < 0.5 is used.
                mask = obj_id_mask
                sorting_method = 'angle'
            else:
                mask = add_angle_mask
                sorting_method = 'score'
            if self.debug:
                print(f'{object_name} using sorting method{sorting_method}, mask num:{np.sum(mask)}')
            i_scores = scores[mask]
            i_ts = ts[mask]
            i_eelink_rs = eelink_rs[mask]
            i_rs = rs[mask]
            i_gg = gg[mask]
            if np.sum(mask) < self.config['response']['mask_thresh']: # actually this should be mask == 0, for safety reason, < 0.5 is used.
                # ungraspable
                grasp_poses[object_name] = None
            else:
                if sorting_method == 'score':
                    best_grasp_id = np.argmax(i_scores)
                elif sorting_method == 'angle':
                    best_grasp_id = np.argmin(i_rs[:, 2, 0])
                else:
                    raise ValueError('Unknown sorting method')
                best_g = i_gg[int(best_grasp_id)]
                remain_gg.append(best_g.to_open3d_geometry())
                grasp_rotation_matrix = i_eelink_rs[best_grasp_id]
                if np.linalg.norm(np.cross(grasp_rotation_matrix[:,0], grasp_rotation_matrix[:,1]) - grasp_rotation_matrix[:,2]) > 0.1:
                    if self.debug:
                        print('\033[031mLeft Hand Coordinate System Grasp!\033[0m')
                    grasp_rotation_matrix[:,0] = - grasp_rotation_matrix[:, 0]
                else:
                    if self.debug:
                        print('\033[032mRight Hand Coordinate System Grasp!\033[0m')
                gqw, gqx, gqy, gqz = mat2quat(grasp_rotation_matrix)
                grasp_poses[object_name] = {
                    'x': i_ts[best_grasp_id][0],
                    'y': i_ts[best_grasp_id][1],
                    'z': i_ts[best_grasp_id][2],
                    'qw': gqw,
                    'qx': gqx,
                    'qy': gqy,
                    'qz': gqz
                }
        return grasp_poses, remain_gg

    def percept(
            self,
            object_list,
            pose_method,
        ):
        '''
        Generate object 6d poses and grasping poses.
        Only geometry infomation is used in this implementation.

        There are mainly three steps.
        - Moving the camera to different predefined locations and capture RGBD images. Reconstruct the 3D scene.
        - Generating objects 6d poses by mainly icp matching.
        - Generating grasping poses by graspnet-baseline.

        Args:
            object_list(list): strings of object names.
            pose_method: string of the 6d pose estimation method, "icp" or "superglue".
        Returns:
            dict, dict: object 6d poses and grasp poses.
        '''
        # Capture Data
        full_pcd, color_images, camera_poses = self.capture_data()
        # Compute Grasping Poses (Many Poses in a Scene)
        gg = self.compute_grasp_pose(full_pcd)
        if self.debug:
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
            o3d.visualization.draw_geometries([frame, full_pcd, *gg.to_open3d_geometry_list()])

        # Computer Object 6d Poses
        object_poses = self.compute_6d_pose(
            full_pcd = full_pcd,
            color_images = color_images,
            camera_poses = camera_poses,
            pose_method = pose_method,
            object_list = object_list
        )

        # Assign the Best Grasp Pose on Each Object
        grasp_poses, remain_gg = self.assign_grasp_pose(gg, object_poses)
        if self.debug:
            o3d.visualization.draw_geometries([full_pcd, *remain_gg.to_open3d_geometry_list()])
        return object_poses, grasp_poses

    def get_response(self, object_list):
        '''
        Generating the defined ros perception message given the targe object list.

        Args:
            object_list(list): strings of object names.

        Returns:
            dict: both object and grasp poses which is close to the ros msg format.
        '''
        object_poses, grasp_poses = self.percept(
            object_list = object_list,
            pose_method = self.config['pose_method']
        )

        #####################################################
        # format of response_poses:
        # -------graspable
        #     |
        #     ---object_pose
        #     |  |
        #     |  |--x
        #     |  |
        #     |  |--y
        #     |  |
        #     |  ...
        #     |  |
        #     |  ---qw
        #     |
        #     ---grasp_pose (exists if graspable == True)
        #        |
        #        |--x
        #        |
        #        |--y
        #        |
        #        ...
        #        |
        #        ---qw
        #####################################################
        ### the keys of response_poses are object names.
        response_poses = dict()
        for object_name in object_poses.keys():
            response_poses[object_name] = dict()
            qw, qx, qy, qz = mat2quat(object_poses[object_name]['pose'][:3,:3])
            response_poses[object_name]['object_pose'] = {
                'x': object_poses[object_name]['pose'][0, 3],
                'y': object_poses[object_name]['pose'][1, 3],
                'z': object_poses[object_name]['pose'][2, 3],
                'qw': qw,
                'qx': qx,
                'qy': qy,
                'qz': qz
            }
            if grasp_poses[object_name] is None:
                response_poses[object_name]['graspable'] = False
            else:
                response_poses[object_name]['graspable'] = True
                response_poses[object_name]['grasp_pose'] = grasp_poses[object_name]
        return response_poses
