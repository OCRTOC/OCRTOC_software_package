import sys
import os
import trimesh
import cv2
import json
import numpy as np
import PIL
import io
import base64
import pickle as pkl
import pycolmap
import time
import torch
import matplotlib
from tqdm import tqdm
from scipy.spatial.transform import Rotation
import argparse
import rospkg

from .include import io_interface
from .utils import get_3d_points, get_projection_image, flann_matching, quaternion_matrix, load_object_list, render_image
from .SuperGluePretrainedNetwork.models.matching import Matching
from .SuperGluePretrainedNetwork.models.utils import read_image, convert_image

def get_match_kps(kps1, kps2, matches):
    match_idx_kps1 = np.nonzero(matches+1)
    kps1 = kps1[match_idx_kps1]
    match_idx_kps1 = matches[match_idx_kps1]
    kps2 = kps2[match_idx_kps1]
    return kps1, kps2

def check_match_quality(raw_mesh, view, kp1, kp2, matches, camera_matrix, min_correspondence_thresh):
    template_points_2d, image_points_2d = get_match_kps(kp2, kp1, matches[0])
    obj_points, img_points, idxs = get_3d_points(
        raw_mesh, view, image_points_2d, template_points_2d, camera_matrix)

    if len(obj_points) < min_correspondence_thresh:
        return False, False, False

    obj_points = np.stack(obj_points)
    img_points = np.stack(img_points)

    cfg = {
        'model': 'PINHOLE',
        'width': 1280,
        'height': 720,
        'params': [float(camera_matrix[0, 0]), float(camera_matrix[1, 1]), float(camera_matrix[0, 2]), float(camera_matrix[1, 2])]
    }
    ret = pycolmap.absolute_pose_estimation(
        img_points.reshape(-1, 2), obj_points, cfg, 24)

    if ret['success'] and sum(ret['inliers']) > min_correspondence_thresh:
        qvec = ret['qvec']
        tvec = ret['tvec']
        Tcw = quaternion_matrix(qvec)
        Tcw[:3, 3] = tvec
        inliers_kps1 = []
        inliers_kps2 = []

        for i in range(len(ret['inliers'])):
            if ret['inliers'][i]:
                inliers_kps1.append(image_points_2d[idxs[i]])
                inliers_kps2.append(template_points_2d[idxs[i]])

        return inliers_kps1, inliers_kps2, Tcw

    return False, False, False

def compute_pose(matching, renderer, obj_list, image, camera_Twc, model_suffix, camera_matrix, rendered_object_dir, models_dir, config, rot=0):
    resize = config['resize']
    image, inp, scale = convert_image(image, config['device'], resize, rot, False)

    kps1_dict = matching.extract_feat(inp)
    kp1 = kps1_dict['keypoints'][0].cpu().numpy()
    if rot == 2:
        kp1[:, 0] = image.shape[1]-1-kp1[:, 0]
        kp1[:, 1] = image.shape[0]-1-kp1[:, 1]
    kp1 *= scale

    ret = {}

    for obj in tqdm(obj_list, 'Calculating object poses for one image'):
        max_matches = 0
        max_pose_matrix = None
        raw_mesh_path = os.path.join(
            models_dir, obj, 'visual.ply')

        raw_mesh = trimesh.load(raw_mesh_path)

        for template_id in range(0, len(renderer.views), 2):
            template_name = io_interface.get_view_name(template_id)

            base_dir = os.path.join(rendered_object_dir, obj+model_suffix)
            t_image, t_inp, t_scale = read_image(os.path.join(
                base_dir, "{}.png".format(template_name)), 'cuda', resize, 0, False)

            kps2_dict = matching.extract_feat(t_inp)

            kp2 = kps2_dict['keypoints'][0].cpu().numpy()
            kp2 *= t_scale
            pred = {k+'0': v for k, v in kps1_dict.items()}
            pred['image0'] = inp
            pred = {**pred, **{k+'1': v for k, v in kps2_dict.items()}}
            pred['image1'] = t_inp
            pred = matching(pred)
            matches = pred['matches1'].cpu().numpy()

            if (matches > 0).sum() > config['correspondence_thresh']:
                inliers_kps1, inliers_kps2, pose_matrix = check_match_quality(
                    raw_mesh, renderer.views[template_id], kp1, kp2, matches, camera_matrix, min_correspondence_thresh = config['min_correspondence_thresh'])

                if inliers_kps1 is not False and len(inliers_kps1) > max_matches:
                    max_matches = len(inliers_kps1)
                    pose_matrix = camera_Twc@pose_matrix

                    max_pose_matrix = pose_matrix

        ret[obj] = (max_matches, max_pose_matrix)

    return ret


def get_pose_superglue(obj_list, images, camera_poses, camera_matrix, superglue_config):
    models_dir = os.path.join(
        rospkg.RosPack().get_path(superglue_config['models_package']),
        superglue_config['models_dir']
    )
    rendered_object_dir = os.path.join(
        rospkg.RosPack().get_path('ocrtoc_perception'),
        superglue_config['rendered_object_dir']
    )
    realsense_radius = superglue_config['realsense_radius']
    img_h = superglue_config['img_h']
    img_w = superglue_config['img_w']
    camera_info = {
        'D': [0.0, 0.0, 0.0, 0.0, 0.0],
        'K': list(camera_matrix.reshape(-1)),
        'image_height': img_h,
        'image_width': img_w
    }

    rendered_image_suffix = '{}_{}'.format(
        camera_info['image_height'], camera_info['image_width'])

    renderer = render_image(camera_info, models_dir, obj_list,
                            rendered_object_dir, rendered_image_suffix, realsense_radius)

    config = {
        'superpoint': superglue_config['superpoint'],
        'superglue': superglue_config['superglue']
    }
    matching = Matching(config).eval().cuda()

    pose_ret = {}
    for obj in obj_list:
        pose_ret[obj] = (0, None)

    for image, camera_pose in zip(images, camera_poses):
        ret = compute_pose(matching, renderer, obj_list, image, camera_pose,
                           rendered_image_suffix, camera_matrix, rendered_object_dir, models_dir, rot=0, config = superglue_config['compute_pose'])
        for model, (num_matches, pose) in ret.items():
            if num_matches > pose_ret[model][0]:
                pose_ret[model] = (num_matches, pose)

        ret = compute_pose(matching, renderer, obj_list, image, camera_pose,
                           rendered_image_suffix, camera_matrix, rendered_object_dir, models_dir, rot=2,
                           config = superglue_config['compute_pose'])
        for model, (num_matches, pose) in ret.items():
            if num_matches > pose_ret[model][0]:
                pose_ret[model] = (num_matches, pose)
    object_poses = dict()
    for obj_name in pose_ret.keys():
        if pose_ret[obj_name][1] is not None:
            object_poses[obj_name] = dict()
            object_poses[obj_name]['pose'] = pose_ret[obj_name][1]
    return object_poses
