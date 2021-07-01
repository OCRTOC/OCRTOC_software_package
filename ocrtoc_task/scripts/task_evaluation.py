# license
import csv
from enum import Enum, unique
import numpy as np


@unique
class Ptstype(Enum):
    ADC = 0
    NADC = 1
    ADV = 2
    ADA = 3


@unique
class Metrictype(Enum):
    asymmetric = 0
    symmetric = 1
    symmetric_x = 2
    symmetric_x90 = 3
    symmetric_x180 = 4
    symmetric_y = 5
    symmetric_y90 = 6
    symmetric_y180 = 7
    symmetric_z = 8
    symmetric_z90 = 9
    symmetric_z180 = 10


def transform_pts_Rt(pts, R, t):
    """Applies a rigid transformation to 3D points.

    :param pts: nx3 ndarray with 3D points.
    :param R: 3x3 ndarray with a rotation matrix.
    :param t: 3x1 ndarray with a translation vector.
    :return: nx3 ndarray with transformed 3D points.
    """
    assert (pts.shape[1] == 3)
    pts_t = R.dot(pts.T) + t.reshape((3, 1))
    return pts_t.T


def get_ada_pts(min_x, max_x, min_y, max_y, min_z, max_z):
    return np.array([[min_x, 0., 0.],
                     [max_x, 0., 0.],
                     [0., min_y, 0.],
                     [0., max_y, 0.],
                     [0., 0., min_z],
                     [0., 0., max_z]])


def get_adc_pts():
    return get_adv_pts(-1., 1., -1., 1., -1., 1.)


def get_adv_pts(min_x, max_x, min_y, max_y, min_z, max_z):
    return np.array([[min_x, min_y, min_z],
                     [min_x, min_y, max_z],
                     [min_x, max_y, min_z],
                     [min_x, max_y, max_z],
                     [max_x, min_y, min_z],
                     [max_x, min_y, max_z],
                     [max_x, max_y, min_z],
                     [max_x, max_y, max_z]])


def pts_pose_error(R_est, t_est, R_gt, t_gt, pts):
    """
    :param R_est: 3x3 ndarray with the estimated rotation matrix.
    :param t_est: 3x1 ndarray with the estimated translation vector.
    :param R_gt: 3x3 ndarray with the ground-truth rotation matrix.
    :param t_gt: 3x1 ndarray with the ground-truth translation vector.
    :param pts: nx3 ndarray with 3D model points.
    :return: The calculated error.
    """
    pts_est = transform_pts_Rt(pts, R_est, t_est)
    pts_gt = transform_pts_Rt(pts, R_gt, t_gt)
    e = np.linalg.norm(pts_est - pts_gt, axis=1).mean()
    return e


def total_pose_error(level_pose_errors):
    """Total pose error of all levels' tasks.

    :param level_pose_errors: 5x1 ndarray with 5 levels' pose errors.
    :return: The total pose error.
    """
    return np.sum(level_pose_errors)


def level_pose_error(scene_pose_errors):
    """Pose error of a level's tasks.

    :param scene_pose_errors: nx1 ndarray with all scenes' pose errors.
        n is depending on how many scenes can be finished.
    :return: The level pose error.
    """
    return np.mean(scene_pose_errors)


def scene_pose_error(object_pose_errors):
    """Pose error of a scene.

    :param object_pose_errors: nx1 ndarray with all objects' pose errors.
        n is depending on how many objects are counted.
    :return: The scene pose error.
    """
    return np.mean(object_pose_errors)


def object_pose_error(R_est, t_est, R_gt, t_gt,
                      model_info, max_error_threshold):
    """Pose error of an object.

    :param R_est: 3x3 ndarray with the estimated rotation matrix.
    :param t_est: 3x1 ndarray with the estimated translation vector.
    :param R_gt: 3x3 ndarray with the ground-truth rotation matrix.
    :param t_gt: 3x1 ndarray with the ground-truth translation vector.
    :param model_info: information about an object model, e.g. min_x, max_x,
        min_y, max_y, min_z, max_z, width, length, height.
    :return: The object pose error.
    """
    # precomputed rotation along [1, 0, 0]
    R_x90 = np.array([[1., 0., 0.],
                      [0., 0., -1.],
                      [0., 1., 0.]])
    R_x180 = np.array([[1., 0., 0.],
                       [0., -1., 0.],
                       [0., 0., -1.]])
    R_x270 = np.array([[1., 0., 0.],
                       [0., 0., 1.],
                       [0., -1., 0.]])

    # precomputed rotation along [0, 1, 0]
    R_y90 = np.array([[0., 0., 1.],
                      [0., 1., 0.],
                      [-1., 0., 0.]])
    R_y180 = np.array([[-1., 0., 0.],
                       [0., 1., 0.],
                       [0., 0., -1.]])
    R_y270 = np.array([[0., 0., -1.],
                       [0., 1., 0.],
                       [1., 0., 0.]])

    # precomputed rotation along [0, 0, 1]
    R_z90 = np.array([[0., -1., 0.],
                      [1., 0., 0.],
                      [0., 0., 1.]])
    R_z180 = np.array([[-1., 0., 0.],
                       [0., -1., 0.],
                       [0., 0., 1.]])
    R_z270 = np.array([[0., 1., 0.],
                       [-1., 0., 0.],
                       [0., 0., 1.]])

    pts_type = Ptstype.ADC

    cube_length = (model_info['width'] + model_info['length']
                   + model_info['height']) / 3.
    pose_error = max_error_threshold * cube_length

    pts = get_adc_pts() * cube_length
    if model_info['metric'] == Metrictype.asymmetric.name:
        if pts_type == Ptstype.ADC:
            pose_error = min(pose_error,
                             pts_pose_error(R_est, t_est, R_gt, t_gt, pts))
        elif pts_type == Ptstype.NADC:
            pose_error = min(pose_error,
                             pts_pose_error(R_est, t_est, R_gt, t_gt, pts)
                             / cube_length)
        elif pts_type == Ptstype.ADV:
            pts = get_adv_pts(model_info['min_x'], model_info['max_x'],
                              model_info['min_y'], model_info['max_y'],
                              model_info['min_z'], model_info['max_z'])
            pose_error = min(pose_error,
                             pts_pose_error(R_est, t_est, R_gt, t_gt, pts))
        elif pts_type == Ptstype.ADA:
            pts = get_ada_pts(model_info['min_x'], model_info['max_x'],
                              model_info['min_y'], model_info['max_y'],
                              model_info['min_z'], model_info['max_z'])
            pose_error = min(pose_error,
                             pts_pose_error(R_est, t_est, R_gt, t_gt, pts))
        else:
            print('Unknown pts type! ' + pts_type.name)
            assert False
    elif model_info['metric'] == Metrictype.symmetric_x.name:
        pts = np.array([[model_info['min_x'], 0., 0.],
                        [model_info['max_x'], 0., 0.]])
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_x90.name:
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_x90), t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_x180), t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_x270), t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_x180.name:
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_x180), t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_y.name:
        pts = np.array([[0., model_info['min_y'], 0.],
                        [0., model_info['max_y'], 0.]])
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_y90.name:
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_y90), t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_y180), t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_y270), t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_y180.name:
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_y180), t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_z.name:
        pts = np.array([[0., 0., model_info['min_z']],
                        [0., 0., model_info['max_z']]])
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_z90.name:
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_z90), t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_z180), t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_z270), t_gt, pts))
    elif model_info['metric'] == Metrictype.symmetric_z180.name:
        pose_error = min(pose_error,
                         pts_pose_error(R_est, t_est, R_gt, t_gt, pts),
                         pts_pose_error(R_est, t_est,
                                        np.dot(R_gt, R_z180), t_gt, pts))
    else:
        print('Unknown metric! ' + model_info['metric'])
        assert False

    return pose_error


if __name__ == '__main__':
    csv_file = open('/home/rar-s15/catkin_github/src/'
                    'simulator_for_manipulation_and_grasping/'
                    'ocrtoc_materials/objects.csv')
    objects_db = csv.DictReader(csv_file)
    objects_dict = {}
    for item in objects_db:
        for key in item.keys():
            try:
                item[key] = float(item[key])
            except:
                pass
        objects_dict[item['object']] = item

    print('metric: ' + objects_dict['a_cups']['metric'])

    R_est = np.array([[1., 0., 0.],
                      [0., 1., 0.],
                      [0., 0., 1.]])
    t_est = np.array([0.01, 0.01, 0.01])
    R_gt = np.array([[0., -1., 0.],
                     [1., 0., 0.],
                     [0., 0., 1.]])
    t_gt = np.array([0., 0., 0.])

    max_error_threshold = 5.
    object_error = object_pose_error(R_est, t_est, R_gt, t_gt,
                                     objects_dict['adjustable_wrench'],
                                     max_error_threshold)
    print('asymmetric object_error: ' + str(object_error))

    object_error = object_pose_error(R_est, t_est, R_gt, t_gt,
                                     objects_dict['a_cups'],
                                     max_error_threshold)
    print('symmetric object_error: ' + str(object_error))
