import csv
import os
import pickle
import xml.etree.ElementTree as ET
from collections import defaultdict
from copy import deepcopy as copy
from typing import Dict, Iterable

import numpy as np
import sapien.core as sapien
from transforms3d.euler import euler2quat


def dict_process_numerics(d: dict, num_type=float):
    ret = {}
    for key in d:
        try:
            num = num_type(d[key])
        except ValueError:
            ret[key] = d[key]
        else:
            ret[key] = num
    return ret


OBJECT_INFOS = None


def load_object_infos():
    """
    load and parse object info from objects.csv
    """
    global OBJECT_INFOS

    if OBJECT_INFOS is None:
        materials_dir = '/root/ocrtoc_materials'
        OBJECT_INFOS = {}
        objects_info_csv = csv.DictReader(open(os.path.join(materials_dir, 'objects.csv')))
        for idx, row in enumerate(objects_info_csv):
            info = dict(row)
            object_name = info['object']
            del info['object']
            OBJECT_INFOS[object_name] = dict_process_numerics(info, num_type=float)
            OBJECT_INFOS[object_name]['id'] = idx

    return copy(OBJECT_INFOS)


def load_goal_sdf(sdf_file, scene, table_height):
    model_path = os.getenv('SAPIEN_MODEL_PATH')
    assert model_path, 'SAPIEN_MODEL_PATH environment variable is required'
    if model_path[-1] != '/':
        model_path += '/'
    sdf = ET.parse(sdf_file).getroot()
    world = sdf.find('world')
    actors = []
    for sdf_model in world.findall('model'):
        builder = scene.create_actor_builder()
        sdf_link = sdf_model.find('link')
        sdf_inertial = sdf_link.find('inertial')
        assert sdf_inertial is not None
        vs = sdf_link.findall('visual')
        for v in vs:
            sdf_geom = v.find('geometry')
            sdf_mesh = sdf_geom.find('mesh')
            sdf_uri = sdf_mesh.find('uri')
            sdf_scale = sdf_mesh.find('scale')
            assert sdf_uri is not None and sdf_scale is not None
            filename = sdf_uri.text.replace('package://models/', model_path)
            scale = [float(x) for x in sdf_scale.text.strip().split()]
            assert len(scale) == 3
            assert os.path.isfile(filename), filename
            builder.add_visual_from_file(filename, scale=scale)
        model_pose = sdf_model.find('pose')
        model = builder.build_kinematic(name=f"{sdf_model.attrib['name']}_goal")
        for body in model.get_visual_bodies():
            body.set_visibility(0.3)
        xyzrpy = np.array([float(x) for x in model_pose.text.strip().split()])
        xyzrpy[2] += table_height
        model.set_pose(sapien.Pose(xyzrpy[:3], euler2quat(*xyzrpy[3:])))
        model.set_velocity([0, 0, 0])
        actors.append(model)
    for sdf in actors:
        current_pose = sdf.get_pose()
        position = current_pose.p
        position[2] += 100
        sdf.set_pose(sapien.Pose(position, current_pose.q))
    return actors


def load_sapien_sdf(sdf_file, scene, table_height):
    model_path = os.getenv('SAPIEN_MODEL_PATH')
    assert model_path, 'SAPIEN_MODEL_PATH environment variable is required'
    if model_path[-1] != '/':
        model_path += '/'
    sdf = ET.parse(sdf_file).getroot()
    world = sdf.find('world')
    actors = []
    id2name = {}
    id2scale = {}
    for l in world.findall('light'):
        assert l.attrib['type'] == 'point'
        color = [float(x) / 3.14 for x in l.find('diffuse').text.split()]
        position = np.array([float(x) for x in l.find('pose').text.split()][:3])
        position[2] += table_height
        scene.add_point_light(position, color)
    for sdf_model in world.findall('model'):
        builder = scene.create_actor_builder()
        sdf_link = sdf_model.find('link')
        sdf_pose = sdf_model.find('pose')
        sdf_inertial = sdf_link.find('inertial')
        assert sdf_inertial is not None
        cs = sdf_link.findall('collision')
        vs = sdf_link.findall('visual')

        uri_node = next(sdf_model.iterfind(".//uri"))
        model_name = (uri_node.text.split("://")[-1]).split("/")[0]
        scale_node = next(sdf_model.iterfind(".//scale"))
        model_scale = [float(x) for x in scale_node.text.strip().split()]

        for col in cs:
            sdf_geom = col.find('geometry')
            sdf_mesh = sdf_geom.find('mesh')
            sdf_uri = sdf_mesh.find('uri')
            sdf_scale = sdf_mesh.find('scale')
            assert sdf_uri is not None and sdf_scale is not None
            filename = sdf_uri.text.replace('package://models/', model_path)
            scale = [float(x) for x in sdf_scale.text.strip().split()]
            assert len(scale) == 3
            assert os.path.isfile(filename), filename
            friction = float(col.find('surface').find('friction').find('ode').find('mu').text)
            assert friction == 0.5  # will all be 0.5
            builder.add_multiple_collisions_from_file(filename, scale=scale)

        for v in vs:
            sdf_geom = v.find('geometry')
            sdf_mesh = sdf_geom.find('mesh')
            sdf_uri = sdf_mesh.find('uri')
            sdf_scale = sdf_mesh.find('scale')
            assert sdf_uri is not None and sdf_scale is not None
            filename = sdf_uri.text.replace('package://models/', model_path)
            scale = [float(x) for x in sdf_scale.text.strip().split()]
            assert len(scale) == 3
            assert os.path.isfile(filename), filename
            builder.add_visual_from_file(filename, scale=scale)

        sdf_mass = sdf_inertial.find('mass')
        sdf_pose = sdf_inertial.find('pose')
        sdf_inertia = sdf_inertial.find('inertia')
        assert sdf_mass is not None and sdf_pose is not None and sdf_inertia is not None
        mass = float(sdf_mass.text)
        xyzrpy = [float(x) for x in sdf_pose.text.strip().split()]
        assert len(xyzrpy) == 6
        ixx = float(sdf_inertia.find('ixx').text)
        iyy = float(sdf_inertia.find('iyy').text)
        izz = float(sdf_inertia.find('izz').text)
        ixy = float(sdf_inertia.find('ixy').text)
        ixz = float(sdf_inertia.find('ixz').text)
        iyz = float(sdf_inertia.find('iyz').text)
        assert ixy == ixz == iyz == 0
        builder.set_mass_and_inertia(mass, sapien.Pose(xyzrpy[:3], euler2quat(*xyzrpy[3:])), [ixx, iyy, izz])
        model_pose = sdf_model.find('pose')
        model = builder.build(name=sdf_model.attrib['name'])
        xyzrpy = np.array([float(x) for x in model_pose.text.strip().split()])
        xyzrpy[2] += table_height
        model.set_pose(sapien.Pose(xyzrpy[:3], euler2quat(*xyzrpy[3:])))
        model.set_velocity([0, 0, 0])
        model.set_damping(1, 1)
        actors.append(model)

        id2name[model.get_id()] = model_name
        id2scale[model.get_id()] = model_scale

    return actors, id2name, id2scale


def load_assets(assets, scales, scene, table_height):
    model_path = os.getenv("SAPIEN_MODEL_PATH")
    actors = []
    for asset, scale in zip(assets, scales):
        builder = scene.create_actor_builder()
        vf = os.path.join(model_path, asset, "visual_mesh.obj")
        cf = os.path.join(model_path, asset, "collision_mesh.obj")
        print(vf, cf)
        builder.add_visual_from_file(vf, scale=[scale] * 3)
        builder.add_multiple_convex_shapes_from_file(cf, scale=[scale] * 3)
        actor = builder.build(name=asset)
        actors.append(actor)
    return actors


def find_scales():
    pickle_file = "obj_scales.pkl"
    if os.path.isfile(pickle_file):
        with open(pickle_file, "rb") as f:
            return pickle.load(f)

    scene_path = "/root/ocrtoc_materials/scenes"
    worlds = os.listdir(scene_path)
    obj2scales = defaultdict(set)

    for w in worlds:
        # sdf = ET.parse(os.path.join(scene_path, w, "target.world"))
        sdf = ET.parse(os.path.join(scene_path, w, "input.world"))
        root = sdf.getroot()
        cols = root.findall(".//collision")
        for col in cols:
            mesh = col.find(".//mesh")

            uri = mesh.find("uri").text
            scale = mesh.find("scale").text

            model_name = uri.split("://")[1].split("/")[0]
            scale = float(scale.split(" ")[0])

            obj2scales[model_name].add(scale)

    with open(pickle_file, "wb") as f:
        pickle.dump(obj2scales, f)
    return obj2scales


def find_scenes():
    import csv

    csv_file = "scenes.csv"
    if os.path.isfile(csv_file):
        with open(csv_file, "r") as f:
            csv_reader = csv.DictReader(f)
            scenes_info = []
            for row in csv_reader:
                row['object_names'] = eval(row['object_names'])
                row['scales'] = eval(row['scales'])
                scenes_info.append(row)
            # print(scenes_info[0])
            return scenes_info

    scene_path = "/root/ocrtoc_materials/scenes"
    worlds = os.listdir(scene_path)

    def natsort(scene_id: str):
        level_id, variant_id = scene_id.split('-')
        return int(level_id), int(variant_id)

    worlds = sorted(worlds, key=natsort)

    scenes_info = []

    for w in worlds:
        scene_info = {'scene_id': w, 'object_names': [], 'scales': []}

        # sdf = ET.parse(os.path.join(scene_path, w, "target.world"))
        sdf = ET.parse(os.path.join(scene_path, w, "input.world"))
        root = sdf.getroot()
        cols = root.findall(".//collision")
        for col in cols:
            mesh = col.find(".//mesh")

            uri = mesh.find("uri").text
            scale = mesh.find("scale").text

            model_name = uri.split("://")[1].split("/")[0]
            scale = float(scale.split(" ")[0])

            scene_info['object_names'].append(model_name)
            scene_info['scales'].append(scale)
        scenes_info.append(scene_info)

    with open(csv_file, "w") as f:
        csv_writer = csv.DictWriter(f, fieldnames=['scene_id', 'object_names', 'scales'])
        csv_writer.writeheader()
        csv_writer.writerows(scenes_info)
    return scenes_info


def list2str(l: Iterable):
    return ''.join([str(i) + ' ' for i in l])[:-1]


def dump_scene_json(objects: Dict, save_name: str):
    js = {}
    for obj_name in objects:
        js[obj_name] = objects[obj_name]['pose_mat'].tolist()

    import json
    json.dump(js, open(save_name, 'w'))


def dump_sapien_sdf(
        objects: Dict, save_name: str,
        sdf_template: str = '/root/ocrtoc_materials/world.template'):
    """
    :param objects: Dict[Dict] {obj_name: {...}}
    :param save_name:
    :param sdf_template:
    :return:
    """
    obj_infos = load_object_infos()

    tree = ET.parse(sdf_template)
    sdf = tree.getroot()

    world = sdf.find('world')
    model_template = world.find('model')
    world.remove(model_template)

    for obj_name in objects:
        obj_info = obj_infos[obj_name]
        obj_specs = objects[obj_name]
        model = copy(model_template)

        model.attrib['name'] = obj_name
        model.find('pose').text = list2str(obj_specs['pose'])

        link = model.find('link')
        link.attrib['name'] = f'link_{obj_name}'

        cpath = link.find('collision').find('.//uri')
        cpath.text = f'model://{obj_name}/collision_mesh.obj'

        vpath = link.find('visual').find('.//uri')
        vpath.text = f'model://{obj_name}/visual_mesh.obj'

        inertial = link.find('inertial')

        inertial.find('.//mass').text = str(obj_info['mass'])
        inertial.find('.//pose').text = list2str([
            obj_info['cm_x'], obj_info['cm_y'], obj_info['cm_z'],
            obj_info['q_yaw'], obj_info['q_pitch'], obj_info['q_roll']
        ])
        inertial.find('.//ixx').text = str(obj_info['Ixx'])
        inertial.find('.//iyy').text = str(obj_info['Iyy'])
        inertial.find('.//izz').text = str(obj_info['Izz'])

        world.append(model)

    tree.write(save_name)
