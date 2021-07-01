#! /usr/bin/env python3
import os
os.environ['PYOPENGL_PLATFORM']='egl'
import sys
# sys.path.insert(1, '/usr/local/lib/python3.5/dist-packages')
import cv2
import math
import numpy as np
import pyrender
import trimesh

sys.path.append('../bop_toolkit')
from ..bop_toolkit_lib import view_sampler
sys.path.append('../include')
from . import io_interface

def kp_to_dict(kp):
    kp_dict = []
    for kpi in kp:
        kp_dict.append({'angle': kpi.angle,
                        'class_id': kpi.class_id,
                        'octave': kpi.octave,
                        'pt': [kpi.pt[0], kpi.pt[1]],
                        'response': kpi.response,
                        'size': kpi.size})
    return kp_dict


class ModelRenderer:
    def __init__(self, camera_info, view_radius):
        self.flip_yz_rotation = np.array([[1., 0., 0.],
                                          [0., -1., 0.],
                                          [0., 0., -1.]])

        # Set up the camera, z-axis away from the scene, x-axis right, y-axis up
        self.camera = pyrender.IntrinsicsCamera(fx=camera_info['K'][0],
                                                fy=camera_info['K'][4],
                                                cx=camera_info['K'][2],
                                                cy=camera_info['K'][5])
        self.renderer = pyrender.OffscreenRenderer(camera_info['image_width'],
                                                   camera_info['image_height'])
        # Set up the light, a single spot light in the same spot as the camera
        self.light = pyrender.SpotLight(
            color=np.ones(3),
            intensity=view_radius * view_radius * 20,
            innerConeAngle=np.pi / 16.0)

        self.views, views_level = view_sampler.sample_views(
            min_n_views=15,
            radius=view_radius,
            azimuth_range=(0, 2 * math.pi),
            elev_range=(-0.5 * math.pi, 0.5 * math.pi),
            mode='hinterstoisser')

        self.num_views = len(self.views)
        for view_id in range(self.num_views):
            view = self.views[view_id]
            view['id'] = '{:0>6d}'.format(view_id)
            view['R'] = view['R'].tolist()
            view['t'] = view['t'].tolist()

        # sift = cv2.xfeatures2d.SURF_create()
        self.sift = cv2.SIFT_create()
        self.view_radius = view_radius

    def reset_light(self, intensity):
        self.light = pyrender.SpotLight(
            color=np.ones(3),
            intensity=intensity,
            innerConeAngle=np.pi / 16.0)

    def render_model_images(self, model_path, images_path, cal_features):
        # Load the FUZE bottle trimesh and put it in a scene
        fuze_trimesh = trimesh.load(model_path)
        mesh = pyrender.Mesh.from_trimesh(fuze_trimesh)

        for view_id in range(self.num_views):
            camera_pose = np.eye(4, dtype=np.float32)
            # cv to gl
            camera_pose[0:3, 0:3] = np.dot(self.flip_yz_rotation,
                                           np.array(self.views[view_id]['R']))
            camera_pose[0:3, 3] = \
                np.dot(self.flip_yz_rotation,
                       np.array(self.views[view_id]['t']).reshape([3]))

            # gl in object
            camera_pose[0:3, 0:3] = np.linalg.inv(camera_pose[0:3, 0:3])
            camera_pose[0:3, 3] = np.dot(camera_pose[0:3, 0:3],
                                         -camera_pose[0:3, 3])

            scene = pyrender.Scene()
            scene.add(mesh)
            scene.add(self.camera, pose=camera_pose)
            scene.add(self.light, pose=camera_pose)
            # Render the scene
            color, depth = self.renderer.render(scene)

            if not os.path.exists(images_path):
                os.makedirs(images_path)
            if cal_features:
                # Find keypoints and descriptors directly
                kp, des = self.sift.detectAndCompute(
                    cv2.cvtColor(color, cv2.COLOR_RGB2GRAY), None)

                io_interface.dump_pickle(
                    [kp_to_dict(kp), des],
                    images_path + '/{:0>6d}'.format(view_id) + '.pickle')

            cv2.imwrite(images_path + '/{:0>6d}'.format(view_id) + '.png',
                        cv2.cvtColor(color, cv2.COLOR_RGB2BGR))

    def render_snapshot(self, input_camera_pose, model_path, save_path):
        # Load the FUZE bottle trimesh and put it in a scene
        fuze_trimesh = trimesh.load(model_path)
        mesh = pyrender.Mesh.from_trimesh(fuze_trimesh)

        # gl in object
        camera_pose = np.eye(4, dtype=np.float32)
        camera_pose[0:3, 0:3] = np.dot(self.flip_yz_rotation, input_camera_pose)
        camera_pose[0:3, 3] = \
            np.dot(self.flip_yz_rotation,
                    np.array([[0], [0], [self.view_radius]]).reshape([3]))
        camera_pose[0:3, 0:3] = np.linalg.inv(camera_pose[0:3, 0:3])
        camera_pose[0:3, 3] = np.dot(camera_pose[0:3, 0:3],
                                     -camera_pose[0:3, 3])

        scene = pyrender.Scene()
        scene.add(mesh)
        scene.add(self.camera, pose=camera_pose)
        scene.add(self.light, pose=camera_pose)
        # Render the scene
        color, depth = self.renderer.render(scene)
        # Save png
        cv2.imwrite(save_path, cv2.cvtColor(color, cv2.COLOR_RGB2BGR))

    def render_models_projection(self,
                                 meshes, transformation, color_image):
        scene = pyrender.Scene()
        for mesh in meshes:
            scene.add(pyrender.Mesh.from_trimesh(mesh))

        camera_pose = np.eye(4, dtype=np.float32)
        # cv to gl
        camera_pose[0:3, 0:3] = np.dot(self.flip_yz_rotation,
                                       transformation[0:3, 0:3])
        camera_pose[0:3, 3] = np.dot(self.flip_yz_rotation,
                                     transformation[0:3, 3].reshape([3]))

        # gl in object
        camera_pose[0:3, 0:3] = np.linalg.inv(camera_pose[0:3, 0:3])
        camera_pose[0:3, 3] = np.dot(camera_pose[0:3, 0:3],
                                     -camera_pose[0:3, 3])

        scene.add(self.camera, pose=camera_pose)
        scene.add(self.light, pose=camera_pose)

        # Render the scene
        color, depth = self.renderer.render(scene)

        result_img = cv2.addWeighted(color, 0.7, color_image, 0.3, 0)
        return result_img


def render_models(model_renderer, images_path, models_path, models, over_write, suffix=''):
    io_interface.dump_yaml(model_renderer.views,
                           os.path.join(images_path, 'views.yaml'))

    for model_id, model in enumerate(models):
        print('rendering ' + model)

        model_path = os.path.join(models_path, model, 'visual.ply')
        model_image_path = os.path.join(images_path, model+suffix)
        if over_write or not os.path.exists(model_image_path):
            model_renderer.render_model_images(
                model_path, os.path.join(images_path, model+suffix), True)


def show_intersection(mesh, ray_origins, ray_directions, locations):
    # stack rays into line segments for visualization as Path3D
    ray_visualize = trimesh.load_path(np.hstack((
        ray_origins,
        ray_origins + ray_directions)).reshape(-1, 2, 3))

    # make mesh transparent- ish
    mesh.visual.face_colors = [100, 100, 100, 100]

    # create a visualization scene with rays, hits, and mesh
    scene = trimesh.Scene([
        mesh,
        ray_visualize,
        trimesh.points.PointCloud(locations)])

    # display the scene
    scene.show()


def kp_mesh_intersection(kpoints, mesh, camera_matrix_inv):
    ray_origins = []
    ray_directions = []
    for p in kpoints:
        uv = np.array([p['pt'][0], p['pt'][1], 1])
        direction = camera_matrix_inv.dot(uv)

        # create some rays
        ray_origins.append([0, 0, 0])
        ray_directions.append(direction)

    ray_origins = np.array(ray_origins)
    ray_directions = np.array(ray_directions)
    # run the mesh-ray test
    locations, index_ray, index_tri = mesh.ray.intersects_location(
        ray_origins=ray_origins,
        ray_directions=ray_directions,
        multiple_hits=False)

    if locations.shape[0] < 4:
        return False, locations, index_ray

    show_intersection = False
    if show_intersection:
        show_intersection(mesh, ray_origins, ray_directions, locations)

    return True, locations, index_ray

