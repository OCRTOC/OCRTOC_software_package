import numpy as np
import pickle
from plyfile import PlyData, PlyElement
import time
import sys
import yaml


def dump_yaml(data, path):
    f = open(path, 'w')
    yaml.dump(data, f)
    f.close()

def load_yaml(yaml_path):
    f = open(yaml_path)
    if sys.version_info.major == 2:
        yaml_file = yaml.load(f, Loader=yaml.Loader)
    else:
        yaml_file = yaml.load(f, Loader=yaml.FullLoader)
    f.close()
    return yaml_file

def dump_pickle(data, path):
    f = open(path, 'wb')
    pickle.dump(data, f)
    f.close()


def load_pickle(pickle_path):
    f = open(pickle_path, 'rb')
    pickle_file = pickle.load(f)
    f.close()
    return pickle_file


def get_view_name(view_id):
    return '{:0>6d}'.format(view_id)


def read_ply(filename):
    """ read XYZ point cloud from filename PLY file """
    plydata = PlyData.read(filename)
    pc = plydata['vertex'].data
    pc_array = np.array([[x, y, z] for x, y, z in pc])
    return pc_array


def write_ply(save_path, points, text=True):
    """
    save_path : path to save: '/yy/XX.ply'
    pt: point_cloud: size (N,3)
    """
    points = [(points[i, 0], points[i, 1], points[i, 2], points[i, 3],
               points[i, 4], points[i, 5]) for i in
              range(points.shape[0])]
    # colors = [(colors[i, 0], colors[i, 1], colors[i, 2]) for i in
    #           range(colors.shape[0])]

    ply_points = PlyElement.describe(
        np.array(points, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                ('red', 'u1'),
                                ('green', 'u1'),
                                ('blue', 'u1')]),
        'vertex', comments=['vertices'])
    PlyData([ply_points], text=text).write(save_path)

    # face = PlyElement.describe(np.array(face_data, dtype=[......]), 'face')
    # color = PlyElement.describe(np.array(color_data, dtype=[......]), 'color')
    # normals = PlyElement.describe(np.array(normals_data, dtype=[......]),
    #                               'normals')
    # PlyData([point, face, color, normals]).write(save_path)


def get_time():
    return time.strftime("%Y%m%d%H%M%S", time.localtime())