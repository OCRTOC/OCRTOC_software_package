from collections import namedtuple

import numpy as np

from viewer import MAX_COLS, MAX_ROWS
from pddlstream.language.generator import outputs_from_boolean

GRASP = np.array([0, 0])


def is_valid(p):
    return np.greater_equal(p, [0, 0]) and np.greater([MAX_COLS, MAX_ROWS], p)


def get_length(vec, ord=1):
    return np.linalg.norm(vec, ord=ord)


def get_difference(p1, p2):
    return np.array(p2) - p1


def collision_test2(p1, p2):
    return get_length(get_difference(p1, p2)) < 1e-3

def collision_test(p1, p2):
    r = np.zeros(5)
    r[0] = collision_test2(p1, p2)
    if np.array(p2)[1] == 2:
        p2_1 = p2 + np.array([0, -1])
        p2_2 = p2 + np.array([0, -2])
        r[3] = collision_test2(p1, p2_1)
        r[4] = collision_test2(p1, p2_2)
    elif np.array(p2)[1] == 1:
        p2_1 = p2 + np.array([0, -1])
        r[3] = collision_test2(p1, p2_1)
    else:
        pass
    return np.linalg.norm(r) > 0.5


def noisy_collision_gen_fn(*args):
    while True:
        if np.random.random() < 0.75:
            yield outputs_from_boolean(False)
        else:
            yield outputs_from_boolean(not collision_test(*args))


def distance_fn(q1, q2):
    return get_length(get_difference(q1, q2))

##################################################

DiscreteTAMPState = namedtuple('DiscreteTAMPState', ['conf', 'holding', 'block_poses'])
DiscreteTAMPProblem = namedtuple('DiscreteTAMPProblem', ['initial', 'poses', 'goal_poses'])
BLOCK_TEMPLATE = 'b{}'
INITIAL_CONF = np.array([0, -1])


def get_shift_one_problem(n_blocks=2, n_poses=9):
    assert(1 <= n_blocks <= n_poses)
    blocks = [BLOCK_TEMPLATE.format(i) for i in range(n_blocks)]
    poses = [np.array([x, 0]) for x in range(n_poses)]

    block_poses = dict(zip(blocks, poses))
    initial = DiscreteTAMPState(INITIAL_CONF, None, block_poses)
    goal_poses = {blocks[0]: poses[1]}

    return DiscreteTAMPProblem(initial, poses[n_blocks:], goal_poses)


def get_shift_all_problem(initial_block_poses, goal_block_poses, temp_poses=[]):
    print("enter my shift all problem")
    initial = DiscreteTAMPState(INITIAL_CONF, None, initial_block_poses)
    available_poses = goal_block_poses.values() + temp_poses

    return DiscreteTAMPProblem(initial, available_poses, goal_block_poses)
