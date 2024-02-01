# Author: Minghao Gou

import numpy as np
import open3d as o3d
import open3d_plus as o3dp
import numpy as np
import os
import torch
import rospkg
from tqdm import tqdm
from itertools import product
from copy import deepcopy
import pickle
import time

def generate_views(N, phi=(np.sqrt(5)-1)/2, center=np.zeros(3, dtype=np.float32), R=1):
    idxs = np.arange(N, dtype=np.float32)
    Z = (2 * idxs + 1) / N - 1
    X = np.sqrt(1 - Z**2) * np.cos(2 * idxs * np.pi * phi)
    Y = np.sqrt(1 - Z**2) * np.sin(2 * idxs * np.pi * phi)
    views = np.stack([X,Y,Z], axis=1)
    views = R * np.array(views) + center
    return views

def generate_angles(N):
    return np.arange(N) / N * 2.0 * np.pi

def viewpoint_params_to_matrix(towards, angle):
    axis_x = towards
    axis_y = np.array([-axis_x[1], axis_x[0], 0])
    if np.linalg.norm(axis_y) == 0:
        axis_y = np.array([0, 1, 0])
    axis_x = axis_x / np.linalg.norm(axis_x)
    axis_y = axis_y / np.linalg.norm(axis_y)
    axis_z = np.cross(axis_x, axis_y)
    R1 = np.array([[1, 0, 0],
                   [0, np.cos(angle), -np.sin(angle)],
                   [0, np.sin(angle), np.cos(angle)]])
    R2 = np.c_[axis_x, np.c_[axis_y, axis_z]]
    matrix = R2.dot(R1)
    return matrix

def batch_viewpoint_params_to_matrix(batch_towards, batch_angle):
    axis_x = batch_towards
    ones = np.ones(axis_x.shape[0], dtype=axis_x.dtype)
    zeros = np.zeros(axis_x.shape[0], dtype=axis_x.dtype)
    axis_y = np.stack([-axis_x[:,1], axis_x[:,0], zeros], axis=-1)
    mask_y = (np.linalg.norm(axis_y, axis=-1) == 0)
    axis_y[mask_y] = np.array([0, 1, 0])
    axis_x = axis_x / np.linalg.norm(axis_x, axis=-1, keepdims=True)
    axis_y = axis_y / np.linalg.norm(axis_y, axis=-1, keepdims=True)
    axis_z = np.cross(axis_x, axis_y)
    sin = np.sin(batch_angle)
    cos = np.cos(batch_angle)
    R1 = np.stack([ones, zeros, zeros, zeros, cos, -sin, zeros, sin, cos], axis=-1)
    R1 = R1.reshape([-1,3,3])
    R2 = np.stack([axis_x, axis_y, axis_z], axis=-1)
    matrix = np.matmul(R2, R1)
    return matrix.astype(np.float32)

def cross_viewpoint_params_to_matrix(batch_towards, batch_angles):
    '''
    **Input:**
    - batch_towards: numpy array of shape (num_views, 3) of the toward vectors.
    - batch_angles: numpy array of shape (num_angles,) of the inplane rotation angles.
    **Output:**
    - numpy array of shape (num_views, num_angles, 3, 3) of the output rotation matrix
    '''
    num_views = batch_towards.shape[0]
    num_angles = len(batch_angles)
    batch_towards = np.repeat(batch_towards,num_angles,axis=0)
    batch_angles = np.tile(batch_angles,num_views)
    axis_x = batch_towards
    ones = np.ones(axis_x.shape[0], dtype=axis_x.dtype)
    zeros = np.zeros(axis_x.shape[0], dtype=axis_x.dtype)
    axis_y = np.stack([-axis_x[:,1], axis_x[:,0], zeros], axis=-1)
    axis_x = axis_x / np.linalg.norm(axis_x, axis=-1, keepdims=True)
    axis_y = axis_y / np.linalg.norm(axis_y, axis=-1, keepdims=True)
    axis_z = np.cross(axis_x, axis_y)
    sin = np.sin(batch_angles)
    cos = np.cos(batch_angles)
    R1 = np.stack([ones, zeros, zeros, zeros, cos, -sin, zeros, sin, cos], axis=-1)
    R1 = R1.reshape([-1,3,3])
    R2 = np.stack([axis_x, axis_y, axis_z], axis=-1)
    matrix = np.matmul(R2, R1)
    return matrix.astype(np.float32).reshape(num_views,num_angles,3,3)

def r_t_to_4x4mat(r, t):
    '''
    r: 3x3
    t: 3
    return: 4x4
    '''
    return np.vstack((np.hstack((r, t.reshape((3,1)))), np.array([0,0,0,1.0])))

def load_model_pcd(model_name, models_dir, type = "visual.ply"):
    model_path = os.path.join(models_dir, model_name, type)
    mesh = o3d.io.read_triangle_mesh(model_path)
    pcd = mesh.sample_points_uniformly(2000)
    return pcd

def norm(t):
    return torch.sqrt(torch.sum(t * t, dim=-1))

def torch_get_closest_point(A, B):
    A = A.unsqueeze(1)
    B = B.unsqueeze(0)
    dist = norm(A - B)
    print(f'A.shape:{A.shape}, B.shape:{B.shape}, dist.shape:{dist.shape}')
    indices = torch.argmin(dist, axis = -1)
    return dist, indices

def torch_get_smallest_dist(A, B):
    A = A.unsqueeze(1).cuda()
    B = B.unsqueeze(0).cuda()
    dist = norm(A - B)
    min_dist_0 = torch.min(dist, axis = 0)
    min_dist_1 = torch.min(dist, axis = 1)
    return min_dist_1.values.cpu(), min_dist_0.values.cpu()

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal,
            max_nn=30
        )
    )
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_feature,
            max_nn=100
        )
    )
    return pcd_down, pcd_fpfh

def get_6d_pose_by_geometry(
        pcd,
        model_names,
        geometry_6d_config,
        debug = False
    ):
    '''
    Generate object 6d poses given the reconstructed point cloud
    and object models. Several clusters of point cloud are segmented
    from the reconstructed point cloud. Different object models
    with different ratational template are tried to register with
    these point cloud clusters. Best registration results are considered
    the 6dpose of each model.

    Args:
        pcd(o3d.geometry.PointCloud): full view point cloud.
        model_names(list): object model names of the scene.
        geometry_6d_config(dict): configurations.
        debug(bool): whether in debug mode.

    Returns:
        dict: object 6d poses.
    '''
    points, colors = o3dp.pcd2array(pcd)
    n_models = len(model_names)
    # remove point cloud outliers
    statistical_outlier_config = geometry_6d_config['statistical_outlier']
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors = statistical_outlier_config['nb_neighbors'],
        std_ratio = statistical_outlier_config['std_ratio']
    )
    if debug:
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(0.1)
        o3d.visualization.draw_geometries([pcd, frame])
    num_points = len(points)

    # The desktop points are found and removed
    segment_plane_config = geometry_6d_config['segment_plane']
    arg, in_plane_list = pcd.segment_plane(
        distance_threshold=segment_plane_config['distance_threshold'],
        ransac_n = segment_plane_config['ransac_n'],
        num_iterations = segment_plane_config['num_iterations']
    )
    mask = np.ones(num_points, dtype = bool)
    mask[in_plane_list] = False
    mask = mask & (points[:,2] > geometry_6d_config['z_min_bound'])

    colors = colors[mask]
    points = points[mask]
    pcd = o3dp.array2pcd(points, colors)

    # remove outlier again after segment plane
    radius_outlier_config = geometry_6d_config['radius_outlier']
    pcd,_= pcd.remove_radius_outlier(
        nb_points = radius_outlier_config['nb_points'],
        radius = radius_outlier_config['radius']
    )

    # loading object models, point cloud are used.
    model_pcds = []
    for model_id in tqdm(range(len(model_names)), 'loading models'):
        model_pcds.append(
            load_model_pcd(
                model_name = model_names[model_id],
                models_dir = os.path.join(
                    rospkg.RosPack().get_path(geometry_6d_config['models_package']),
                    geometry_6d_config['models_dir']
                )
            )
        )

    # generating template rotations
    num_views = geometry_6d_config['template']['num_views']
    num_angles = geometry_6d_config['template']['num_angles']
    views = generate_views(num_views)
    angles = generate_angles(num_angles)

    full_view_pcd = o3d.geometry.PointCloud()
    full_view_pcd.points = o3d.utility.Vector3dVector(points)
    full_view_pcd.colors = o3d.utility.Vector3dVector(colors)
    if debug:
        o3d.visualization.draw_geometries([full_view_pcd, frame])

    # clustering point cloud clusters
    cluster_dbscan_config = geometry_6d_config['cluster_dbscan']
    intvec = pcd.cluster_dbscan(
        eps = cluster_dbscan_config['eps'],
        min_points = cluster_dbscan_config['min_points'],
        print_progress = cluster_dbscan_config['print_progress']
    )
    cluster_id = np.array(intvec)
    n_clusters = cluster_id.max() + 1
    if n_clusters >= n_models:
        # at most one object can be assigned to each point cloud cluster.
        one_model_per_cluster = True
    else:
        one_model_per_cluster = False
    if debug:
        print('number of cluster:{}'.format(n_clusters))
    points, colors = o3dp.pcd2array(pcd)

    best_dict_list = []
    final_dict_dict = {}
    for view, angle in tqdm(product(views, angles), 'template matrix'):
        init_rotation_matrix = viewpoint_params_to_matrix(view, angle)

        for i in range(n_clusters):
            mask_i = (cluster_id == i)
            cluster_pcd = o3dp.array2pcd(points[mask_i], colors[mask_i])
            # preprocess point cloud to get better registration result.
            # refer to http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
            # for more details on registration.
            cluster_pcd_down, cluster_pcd_fpfh = preprocess_point_cloud(
                cluster_pcd,
                voxel_size=geometry_6d_config['voxel_size']
            )
            center = np.mean(np.asarray(cluster_pcd_down.points), axis = 0)
            init_matrix = np.vstack((np.hstack((init_rotation_matrix, center.reshape((3,1)))), np.array((0,0,0,1.0))))
            best_dict = {
                'max_ratio':0,
                'model_name':'',
                'model_id':-1,
                'pose':np.eye(4, dtype = float),
                'cluster_id':-1
            }
            # each object model with each rotation will be tried to register with each cluster.
            for model_id in range(len(model_names)):
                model_name = model_names[model_id]
                model_pcd = model_pcds[model_id]
                this_model_pcd = deepcopy(model_pcd)
                this_model_pcd_down, this_model_pcd_fpfh = preprocess_point_cloud(
                    this_model_pcd,
                    voxel_size = geometry_6d_config['voxel_size']
                )
                reg_result = o3d.pipelines.registration.registration_icp(
                    this_model_pcd_down,
                    cluster_pcd_down,
                    max_correspondence_distance = geometry_6d_config['icp']['max_correspondence_distance'],
                    init = init_matrix,
                    estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
                )
                reg_trans = reg_result.transformation

                ratio = reg_result.fitness
                if best_dict['max_ratio'] < ratio:
                    best_dict = {
                        'max_ratio': ratio,
                        'model_name': model_name,
                        'model_id': model_id,
                        'pose': reg_trans,
                        'cluster_id': i
                    }
            selected_model_pcd = deepcopy(model_pcds[best_dict['model_id']])
            pose = best_dict['pose']
            selected_model_pcd.transform(pose)
            best_dict_list.append(best_dict)

        # Find the best pose for each object model
        for model_id, model_name in enumerate(model_names):
            if model_name in final_dict_dict.keys():
                best_ratio = final_dict_dict[model_name]['ratio']
                best_pose = final_dict_dict[model_name]['pose']
                best_cluster_id = final_dict_dict[model_name]['cluster_id']
                model_found = True
            else:
                best_ratio = 0
                best_pose = np.eye(4)
                best_cluster_id = -1
                model_found = False
            for best_dict in best_dict_list:
                if (best_dict['model_name'] == model_name) and (best_dict['model_id'] == model_id):
                    if best_ratio < best_dict['max_ratio']:
                        best_ratio = best_dict['max_ratio']
                        best_pose = best_dict['pose']
                        best_cluster_id = best_dict['cluster_id']
                        model_found = True
            if model_found:
                final_dict_dict[model_name] = {
                    'ratio':best_ratio,
                    'pose': best_pose,
                    'cluster_id': best_cluster_id
                }
    if debug:
        print('final dict dict:',final_dict_dict)
    if one_model_per_cluster:
        # Sometimes, the best pose of objects are found on the same cluster.
        # Only the best one will be used.
        cluster_id_dict = dict()
        for model_name_in_dict in final_dict_dict.keys():
            if final_dict_dict[model_name_in_dict]['cluster_id'] in cluster_id_dict.keys():
                if final_dict_dict[model_name_in_dict]['ratio'] > cluster_id_dict[final_dict_dict[model_name_in_dict]['cluster_id']]['ratio']:
                    cluster_id_dict[final_dict_dict[model_name_in_dict]['cluster_id']] = {'model_name': model_name_in_dict, 'ratio': final_dict_dict[model_name_in_dict]['ratio']}
            else:
                if debug:
                    print({'model_name': model_name_in_dict, 'ratio': final_dict_dict[model_name_in_dict]['ratio']})
                cluster_id_dict[final_dict_dict[model_name_in_dict]['cluster_id']] = {'model_name': model_name_in_dict, 'ratio': final_dict_dict[model_name_in_dict]['ratio']}
        final_model_names = []
        if debug:
            print(f'cluster-id dict:\n{cluster_id_dict}')
        for cluster_id in cluster_id_dict.keys():
            final_model_names.append(cluster_id_dict[cluster_id]['model_name'])
        new_final_dict_dict = dict()
        for model_name_in_dict in final_dict_dict.keys():
            if model_name_in_dict in final_model_names:
                new_final_dict_dict[model_name_in_dict] = final_dict_dict[model_name_in_dict]
        print(f'final_dict after selection:{final_dict_dict}')
        final_dict_dict = new_final_dict_dict
    return final_dict_dict
